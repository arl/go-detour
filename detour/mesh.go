package detour

import (
	"encoding/binary"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"unsafe"

	"github.com/arl/gogeo/f32"
	"github.com/arl/gogeo/f32/d3"
	"github.com/arl/math32"
)

// A NavMesh is a navigation mesh based on tiles of convex polygons.
//
// The navigation mesh consists of one or more tiles defining three primary
// types of structural data:
//   - A polygon mesh which defines most of the navigation graph. (See recast.PolyMesh
//     for its structure.)
//   - A detail mesh used for determining surface height on the polygon mesh. (See
//     recast.PolyMeshDetail for its structure.)
//   - Off-mesh connections, which define custom point-to-point edges within the
//     navigation graph.
//
// The general build process is as follows:
//   - Create recast.PolyMesh and recast.PolyMeshDetail data using the recast
//     build pipeline.
//   - Optionally, create off-mesh connection data.
//   - Combine the source data into a NavMeshCreateParams structure.
//   - Create a tile data array using CreateNavMeshData().
//   - Allocate at detour.NavMesh object and initialize it. (For single tile
//     navigation meshes, the tile data is loaded during this step.)
//   - For multi-tile navigation meshes, load the tile data using
//     TileNavMesh.addTile().
//
// Notes:
//
//   - This class is usually used in conjunction with the NavMeshQuery class for
//     pathfinding.
//   - Technically, all navigation meshes are tiled. A 'solo' mesh is simply a
//     navigation mesh initialized to have only a single tile.
//   - This class does not implement any asynchronous methods. So the
//     detour.Status result of all methods will always contain either a success or
//     failure flag.
//
// see NavMeshQuery, CreateNavMeshData, NavMeshCreateParams
type NavMesh struct {
	Params                NavMeshParams // Current initialization params. TODO: do not store this info twice.
	Orig                  d3.Vec3       // Origin of the tile (0,0)
	TileWidth, TileHeight float32       // Dimensions of each tile.
	MaxTiles              int32         // Max number of tiles.
	TileLUTSize           int32         // Tile hash lookup size (must be pot).
	TileLUTMask           int32         // Tile hash lookup mask.
	posLookup             []*MeshTile   // Tile hash lookup.
	nextFree              *MeshTile     // Freelist of tiles.
	Tiles                 []MeshTile    // List of tiles.
	saltBits              uint32        // Number of salt bits in the tile ID.
	tileBits              uint32        // Number of tile bits in the tile ID.
	polyBits              uint32        // Number of poly bits in the tile ID.
}

// Decode reads a tiled navigation mesh from r and returns it.
//
// returned error will be different from nil in case of failure.
func Decode(r io.Reader) (*NavMesh, error) {
	// Read header.
	var (
		hdr navMeshSetHeader
		err error
	)

	err = binary.Read(r, binary.LittleEndian, &hdr)
	if err != nil {
		return nil, err
	}

	if hdr.Magic != navMeshSetMagic {
		return nil, fmt.Errorf("wrong magic number: %x", hdr.Magic)
	}

	if hdr.Version != navMeshSetVersion {
		return nil, fmt.Errorf("wrong version: %d", hdr.Version)
	}

	var mesh NavMesh
	status := mesh.Init(&hdr.Params)
	if StatusFailed(status) {
		return nil, fmt.Errorf("status failed 0x%x", status)
	}

	// Read tiles.
	for i := uint32(0); i < hdr.NumTiles; i++ {

		var (
			tileHdr navMeshTileHeader
			err     error
		)
		err = binary.Read(r, binary.LittleEndian, &tileHdr)
		if err != nil {
			return nil, err
		}

		if tileHdr.TileRef == 0 || tileHdr.DataSize == 0 {
			break
		}

		data := make([]byte, tileHdr.DataSize)
		if data == nil {
			break
		}
		_, err = r.Read(data)
		if err != nil {
			return nil, err
		}
		status, _ := mesh.AddTile(data, tileHdr.TileRef)
		if status&Failure != 0 {
			return nil, fmt.Errorf("couldn't add tile %d(), status: 0x%x", i, status)
		}
	}
	return &mesh, nil
}

// SaveToFile saves the navigation mesh as a binary file.
func (m *NavMesh) SaveToFile(fn string) error {
	f, err := os.Create(fn)
	if err != nil {
		return err
	}

	return m.ToWriter(f)
}

// ToWriter writes binary navigation mesh to io.Writer.
func (m *NavMesh) ToWriter(w io.Writer) error {
	// Store header.
	var header navMeshSetHeader
	header.Magic = navMeshSetMagic
	header.Version = navMeshSetVersion
	header.NumTiles = 0
	for i := int32(0); i < m.MaxTiles; i++ {
		if m.Tiles[i].DataSize == 0 {
			continue
		}
		header.NumTiles++
	}
	header.Params = m.Params

	if _, err := header.WriteTo(w); err != nil {
		return fmt.Errorf("Error writing header: %v", err)
	}

	// Store tiles.
	for i := int32(0); i < m.MaxTiles; i++ {
		tile := &m.Tiles[i]
		if tile.DataSize == 0 {
			continue
		}

		var tileHeader navMeshTileHeader
		tileHeader.TileRef = m.TileRef(tile)
		tileHeader.DataSize = tile.DataSize
		if _, err := tileHeader.WriteTo(w); err != nil {
			return err
		}
		var data []byte = make([]byte, tile.DataSize)
		// first Serialize the tile header
		tile.Header.serialize(data)
		// then the tile itself
		tile.serialize(data[tile.Header.size():])
		if _, err := w.Write(data); err != nil {
			return err
		}
	}
	return nil
}

// Returns binary navigation mesh file size.
func (m *NavMesh) Size() int32 {
	var header navMeshSetHeader
	header.Params = m.Params
	hSize := header.Size()

	var tilesSize int32 = 0
	var tileHeader navMeshTileHeader
	for i := int32(0); i < m.MaxTiles; i++ {
		tile := &m.Tiles[i]
		if tile.DataSize == 0 {
			continue
		}

		tilesSize += int32(tileHeader.Size()) + tile.DataSize
	}

	return int32(hSize) + tilesSize
}

// InitForSingleTile set up the navigation mesh for single tile use.
//
//	Arguments:
//	 data     Data of the new tile. (See: CreateNavMeshData)
//	 dataSize The data size of the new tile.
//	 flags    The tile flags. (See: TileFlags)
//
// Return The status flags for the operation.
//
//	see CreateNavMeshData
func (m *NavMesh) InitForSingleTile(data []uint8, flags int) Status {
	var header MeshHeader
	header.unserialize(data)

	// Make sure the data is in right format.
	if header.Magic != navMeshMagic {
		return Failure | WrongMagic
	}
	if header.Version != navMeshVersion {
		return Failure | WrongVersion
	}

	var params NavMeshParams
	copy(params.Orig[:], header.BMin[:])
	params.TileWidth = header.BMax[0] - header.BMin[0]
	params.TileHeight = header.BMax[2] - header.BMin[2]
	params.MaxTiles = 1
	params.MaxPolys = uint32(header.PolyCount)

	status := m.Init(&params)
	if StatusFailed(status) {
		return status
	}

	status, _ = m.AddTile(data, TileRef(flags))
	return status
}

// Init initializes the navigation mesh for tiled use.
//
//	Arguments:
//	 params  Initialization parameters.
//
// Return the status flags for the operation.
func (m *NavMesh) Init(params *NavMeshParams) Status {
	m.Params = *params
	m.Orig = d3.NewVec3From(params.Orig[0:3])
	m.TileWidth = params.TileWidth
	m.TileHeight = params.TileHeight

	// Init tiles
	m.MaxTiles = int32(params.MaxTiles)
	m.TileLUTSize = int32(math32.NextPow2(uint32(params.MaxTiles / 4)))
	if m.TileLUTSize == 0 {
		m.TileLUTSize = 1
	}
	m.TileLUTMask = m.TileLUTSize - 1

	m.Tiles = make([]MeshTile, m.MaxTiles)
	m.posLookup = make([]*MeshTile, m.TileLUTSize)
	m.nextFree = nil
	for i := m.MaxTiles - 1; i >= 0; i-- {
		m.Tiles[i].Salt = 1
		m.Tiles[i].Next = m.nextFree
		m.nextFree = &m.Tiles[i]
	}

	// Init ID generator values.
	m.tileBits = math32.Ilog2(math32.NextPow2(uint32(params.MaxTiles)))
	m.polyBits = math32.Ilog2(math32.NextPow2(uint32(params.MaxPolys)))
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	if 31 < 32-m.tileBits-m.polyBits {
		m.saltBits = 31
	} else {
		m.saltBits = 32 - m.tileBits - m.polyBits
	}

	// if m.saltBits < 10 {
	if m.saltBits < 8 {
		return Status(Failure | InvalidParam)
	}

	return Success
}

// AddTile adds a tile to the navigation mesh.
//
//	Arguments:
//	 data      Data for the new tile mesh. (See: CreateNavMeshData)
//	 flags     Tile flags. (See: tileFlags)
//	 lastRef   The desired reference for the tile. (When reloading a tile.)
//	           optional, defaults to 0
//
// Return The status flags for the operation and the tile reference. (If the
// tile was successfully added.)
//
// The add operation will fail if the data is in the wrong format, the allocated tile
// space is full, or there is a tile already at the specified reference.
//
// The lastRef parameter is used to restore a tile with the same tile reference
// it had previously used. In this case the PolyRef's for the tile will be
// restored to the same values they were before the tile was
// removed.
//
// The nav mesh assumes exclusive access to the data passed and will make
// changes to the dynamic portion of the data. For that reason the data should
// not be reused in other nav meshes until the tile has been successfully
// removed from this nav mesh.
//
// see CreateNavMeshData, removeTileBvTree
func (m *NavMesh) AddTile(data []byte, lastRef TileRef) (Status, TileRef) {
	var hdr MeshHeader
	hdr.unserialize(data)

	// Make sure the data is in right format.
	if hdr.Magic != navMeshMagic {
		return Failure | WrongMagic, 0
	}
	if hdr.Version != navMeshVersion {
		return Failure | WrongVersion, 0
	}

	// Make sure the location is free.
	if m.TileAt(hdr.X, hdr.Y, hdr.Layer) != nil {
		return Failure, 0
	}

	// Allocate a tile.
	var tile *MeshTile
	if lastRef == 0 {
		if m.nextFree != nil {
			tile = m.nextFree
			m.nextFree = tile.Next
			tile.Next = nil
		}
	} else {
		// Try to relocate the tile to specific index with same salt.
		tileIndex := int32(m.decodePolyIDTile(PolyRef(lastRef)))
		if tileIndex >= m.MaxTiles {
			log.Fatalln("tileIndex >= m.m_maxTiles", tileIndex, m.MaxTiles)
			return Failure | OutOfMemory, 0
		}
		// Try to find the specific tile id from the free list.
		target := &m.Tiles[tileIndex]
		var prev *MeshTile
		tile = m.nextFree
		for tile != nil && tile != target {
			prev = tile
			tile = tile.Next
		}
		// Could not find the correct location.
		if tile != target {
			log.Fatalln("couldn't find the correct tile location")
			return Failure | OutOfMemory, 0
		}
		// Remove from freelist
		if prev == nil {
			m.nextFree = tile.Next
		} else {
			prev.Next = tile.Next
		}

		// Restore salt.
		tile.Salt = m.decodePolyIDSalt(PolyRef(lastRef))
	}

	// Make sure we could allocate a tile.
	if tile == nil {
		log.Fatalln("couldn't allocate tile")
		return Failure | OutOfMemory, 0
	}

	// Insert tile into the position lut.
	h := computeTileHash(hdr.X, hdr.Y, m.TileLUTMask)
	tile.Next = m.posLookup[h]
	m.posLookup[h] = tile

	tile.unserialize(&hdr, data[hdr.size():])

	// If there are no items in the bvtree, reset the tree pointer.
	if len(tile.BvTree) == 0 {
		tile.BvTree = nil
	}

	// Build links freelist
	tile.LinksFreeList = 0
	tile.Links[hdr.MaxLinkCount-1].Next = nullLink

	var i int32
	for ; i < hdr.MaxLinkCount-1; i++ {
		tile.Links[i].Next = uint32(i + 1)
	}

	// Init tile.
	tile.Header = &hdr
	tile.Data = make([]byte, len(data))
	copy(tile.Data, data)
	tile.DataSize = int32(len(data))
	tile.Flags = 0

	m.connectIntLinks(tile)

	// Base off-mesh connections to their starting polygons and connect
	// connections inside the tile.
	m.baseOffMeshLinks(tile)
	m.connectExtOffMeshLinks(tile, tile, -1)

	// Create connections with neighbour tiles.
	maxNeis := int32(32)
	neis := make([]*MeshTile, maxNeis)
	var nneis int32

	// Connect with layers in current tile.
	nneis = m.TilesAt(hdr.X, hdr.Y, neis, maxNeis)
	var j int32
	for j = 0; j < nneis; j++ {
		if neis[j] == tile {
			continue
		}

		m.connectExtLinks(tile, neis[j], -1)
		m.connectExtLinks(neis[j], tile, -1)
		m.connectExtOffMeshLinks(tile, neis[j], -1)
		m.connectExtOffMeshLinks(neis[j], tile, -1)
	}

	// Connect with neighbour tiles.
	for i = 0; i < 8; i++ {
		nneis = m.neighbourTilesAt(hdr.X, hdr.Y, i, neis, maxNeis)
		for j = 0; j < nneis; j++ {
			m.connectExtLinks(tile, neis[j], i)
			m.connectExtLinks(neis[j], tile, oppositeTile(i))
			m.connectExtOffMeshLinks(tile, neis[j], i)
			m.connectExtOffMeshLinks(neis[j], tile, oppositeTile(i))
		}
	}

	return Success, m.TileRef(tile)
}

// Removes the specified tile from the navigation mesh.
//
//	Arguments:
//	 ref      The reference of the tile to remove.
//
//	Return values:
//	 data     Data associated with deleted tile.
//	 st       The status flags for the operation.
//
// This function returns the data for the tile so that, if desired,
// it can be added back to the navigation mesh at a later point.
//
// see AddTile
func (m *NavMesh) RemoveTile(ref TileRef) (data []uint8, st Status) {
	if ref == 0 {
		return data, Failure | InvalidParam
	}
	tileIndex := m.decodePolyIDTile(PolyRef(ref))
	tileSalt := m.decodePolyIDSalt(PolyRef(ref))
	if tileIndex >= uint32(m.MaxTiles) {
		return data, Failure | InvalidParam
	}
	tile := &m.Tiles[tileIndex]
	if tile.Salt != tileSalt {
		return data, Failure | InvalidParam
	}
	if tile.Data != nil {
		data = tile.Data
	}

	// Remove tile from hash lookup.
	h := computeTileHash(tile.Header.X, tile.Header.Y, m.TileLUTMask)
	var (
		prev *MeshTile
		cur  *MeshTile = m.posLookup[h]
	)
	for cur != nil {
		if cur == tile {
			if prev != nil {
				prev.Next = cur.Next
			} else {
				m.posLookup[h] = cur.Next
			}
			break
		}
		prev = cur
		cur = cur.Next
	}

	// Remove connections to neighbour tiles.
	const MAX_NEIS = 32
	var (
		neis  [MAX_NEIS]*MeshTile
		nneis int
	)

	// Disconnect from other layers in current tile.
	nneis = int(m.TilesAt(tile.Header.X, tile.Header.Y, neis[:], MAX_NEIS))
	for j := 0; j < nneis; j++ {
		if neis[j] == tile {
			continue
		}
		m.unconnectLinks(neis[j], tile)
	}

	// Disconnect from neighbour tiles.
	for i := 0; i < 8; i++ {
		nneis = int(m.neighbourTilesAt(tile.Header.X, tile.Header.Y, int32(i), neis[:], MAX_NEIS))
		for j := 0; j < nneis; j++ {
			m.unconnectLinks(neis[j], tile)
		}
	}

	tile.Header = nil
	tile.Flags = 0
	tile.LinksFreeList = 0
	tile.Polys = nil
	tile.Verts = nil
	tile.Links = nil
	tile.DetailMeshes = nil
	tile.DetailVerts = nil
	tile.DetailTris = nil
	tile.BvTree = nil
	tile.OffMeshCons = nil
	tile.Data = nil
	tile.DataSize = 0

	// Update salt, salt should never be zero.
	tile.Salt = (tile.Salt + 1) & ((1 << m.saltBits) - 1)
	if tile.Salt == 0 {
		tile.Salt++
	}

	// Add to free list.
	tile.Next = m.nextFree
	m.nextFree = tile

	return data, Success
}

// TileAt returns the tile at the specified grid location.
//
//	Arguments:
//	 x        The tile's x-location. (x, y, layer)
//	 y        The tile's y-location. (x, y, layer)
//	 layer    The tile's layer. (x, y, layer)
//
// Return the tile, or null if it does not exist.
func (m *NavMesh) TileAt(x, y, layer int32) *MeshTile {
	var (
		h    int32
		tile *MeshTile
	)

	// Find tile based on hash.
	h = computeTileHash(x, y, m.TileLUTMask)
	tile = m.posLookup[h]
	for tile != nil {
		if tile.Header != nil &&
			tile.Header.X == x &&
			tile.Header.Y == y &&
			tile.Header.Layer == layer {
			return tile
		}
		tile = tile.Next
	}
	return nil
}

func (m *NavMesh) connectIntLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	var (
		i    int32
		base PolyRef
	)
	base = m.polyRefBase(tile)

	for i = 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		poly.FirstLink = nullLink

		if poly.Type() == polyTypeOffMeshConnection {
			continue
		}

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for j := int32(poly.VertCount - 1); j >= 0; j-- {
			// Skip hard and non-internal edges.
			if poly.Neis[j] == 0 || ((poly.Neis[j] & extLink) != 0) {
				continue
			}

			idx := allocLink(tile)
			if idx != nullLink {
				link := &tile.Links[idx]
				link.Ref = base | PolyRef(poly.Neis[j]-1)
				link.Edge = uint8(j)
				link.Side = 0xff
				link.BMin = 0
				link.BMax = 0
				// Add to linked list.
				link.Next = poly.FirstLink
				poly.FirstLink = idx
			}
		}
	}
}

// polyRefBase returns the polygon reference for the base polygon in the
// specified tile.
//
// Example use case:
//
//	base := navmesh.polyRefBase(tile);
//	for i = 0; i < tile.Header.PolyCount; i++ {
//	    poly = &tile.polys[i]
//	    ref := base | PolyRef(i)
//
//	    // Use the reference to access the polygon data.
//	}
func (m *NavMesh) polyRefBase(tile *MeshTile) PolyRef {
	if tile == nil {
		return 0
	}

	it := (uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.Tiles[0]))) / unsafe.Sizeof(*tile)
	return m.encodePolyID(tile.Salt, uint32(it), 0)
}

func computeTileHash(x, y, mask int32) int32 {
	const (
		h1 int64 = 0x8da6b343 // Large multiplicative constants
		h2 int64 = 0xd8163841 // here arbitrarily chosen primes
	)
	n := h1*int64(x) + h2*int64(y)
	return int32(n) & mask
}

func allocLink(tile *MeshTile) uint32 {
	if tile.LinksFreeList == nullLink {
		return nullLink
	}
	link := tile.LinksFreeList
	tile.LinksFreeList = tile.Links[link].Next
	return link
}

func freeLink(tile *MeshTile, link uint32) {
	tile.Links[link].Next = tile.LinksFreeList
	tile.LinksFreeList = link
}

// encodePolyID derives a standard polygon reference.
//
//	Arguments:
//	 salt     The tile's salt value.
//	 it       The index of the tile.
//	 ip       The index of the polygon within the tile.
func (m *NavMesh) encodePolyID(salt, it, ip uint32) PolyRef {
	return (PolyRef(salt) << (m.polyBits + m.tileBits)) |
		(PolyRef(it) << m.polyBits) | PolyRef(ip)
}

// PolyRef is a polygon reference.
type PolyRef uint32

// Link defines a Link between polygons.
//
// Note: This structure is rarely if ever used by the end user.
// see MeshTile
type Link struct {
	Ref  PolyRef // Neighbour reference. (The neighbor that is linked to.)
	Next uint32  // Index of the next link.
	Edge uint8   // Index of the polygon edge that owns this link.
	Side uint8   // If a boundary link, defines on which side the link is.
	BMin uint8   // If a boundary link, defines the minimum sub-edge area.
	BMax uint8   // If a boundary link, defines the maximum sub-edge area.
}

// A PolyDetail defines the location of detail sub-mesh data within a MeshTile.
type PolyDetail struct {
	VertBase  uint32 // The offset of the vertices in the MeshTile.DetailVerts slice.
	TriBase   uint32 // The offset of the triangles in the MeshTile.DetailTris slice.
	VertCount uint8  // The number of vertices in the sub-mesh.
	TriCount  uint8  // The number of triangles in the sub-mesh.
}

// A BvNode is a bounding volume node.
//
// Note: This structure is rarely if ever used by the end user.
// see MeshTile
type BvNode struct {
	BMin [3]uint16 // Minimum bounds of the node's AABB. [(x, y, z)]
	BMax [3]uint16 // Maximum bounds of the node's AABB. [(x, y, z)]
	I    int32     // The node's index. (Negative for escape sequence.)
}

// OffMeshConnection defines an navigation mesh off-mesh connection within a
// MeshTile object.
//
// An off-mesh connection is a user defined traversable connection made up to
// two vertices.
type OffMeshConnection struct {
	// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	Pos [6]float32

	// The radius of the endpoints. [Limit: >= 0]
	Rad float32

	// The polygon reference of the connection within the tile.
	Poly uint16

	// Link flags.
	// Note: These are not the connection's user defined flags.
	// Those are assigned via the connection's Poly definition.
	// These are link flags used for internal purposes.
	Flags uint8

	// End point side.
	Side uint8

	// The id of the offmesh connection. (User assigned
	// when the navigation mesh is built)
	UserID uint32
}

const (
	// A magic number used to detect compatibility of navigation tile data.
	navMeshMagic int32 = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V'

	// A version number used to detect compatibility of navigation tile data.
	navMeshVersion = 7

	// A magic number used to detect the compatibility of navigation tile states.
	navMeshStateMagic = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S'

	// A version number used to detect compatibility of navigation tile states.
	navMeshStateVersion = 1
)

// Flags representing the type of a navigation mesh polygon.
type polyTypes uint32

const (
	// The polygon is a standard convex polygon that is part of the surface of the mesh.
	polyTypeGround polyTypes = 0
	// The polygon is an off-mesh connection consisting of two vertices.
	polyTypeOffMeshConnection = 1
)

const (
	// A flag that indicates that an entity links to an external entity.
	// (E.g. A polygon edge is a portal that links to another polygon.)
	extLink uint16 = 0x8000

	// A value that indicates the entity does not link to anything.
	nullLink uint32 = 0xffffffff
)

// decodePolyIdTile extracts the tile's index from the specified polygon
// reference.
//
//	see encodePolyID
func (m *NavMesh) decodePolyIDTile(ref PolyRef) uint32 {
	tileMask := PolyRef((PolyRef(1) << m.tileBits) - 1)
	return uint32((ref >> m.polyBits) & tileMask)
}

// Extracts a tile's salt value from the specified polygon reference.
//
//	see encodePolyID
func (m *NavMesh) decodePolyIDSalt(ref PolyRef) uint32 {
	saltMask := (PolyRef(1) << m.saltBits) - 1
	return uint32((ref >> (m.polyBits + m.tileBits)) & saltMask)
}

// Builds internal polygons links for a tile.
func (m *NavMesh) baseOffMeshLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	var (
		i    int32
		con  *OffMeshConnection
		poly *Poly
		base PolyRef
	)

	base = m.polyRefBase(tile)

	// Base off-mesh connection start points.
	for i = 0; i < tile.Header.OffMeshConCount; i++ {
		con = &tile.OffMeshCons[i]
		poly = &tile.Polys[con.Poly]

		var (
			ext []float32
			p   []float32
		)
		ext = []float32{con.Rad, tile.Header.WalkableClimb, con.Rad}

		// Find polygon to connect to.
		p = con.Pos[0:3] // First vertex
		nearestPt := make([]float32, 3)
		ref := m.FindNearestPolyInTile(tile, p, ext, nearestPt)
		if ref == 0 {
			continue
		}
		// findNearestPoly may return too optimistic results, further check to make sure.
		if math32.Sqr(nearestPt[0]-p[0])+math32.Sqr(nearestPt[2]-p[2]) > math32.Sqr(con.Rad) {
			continue
		}
		// Make sure the location is on current mesh.
		var v d3.Vec3 = tile.Verts[poly.Verts[0]*3 : poly.Verts[0]*3+3]
		v.Assign(nearestPt)

		// Link off-mesh connection to target poly.
		idx := allocLink(tile)
		if idx != nullLink {
			link := &tile.Links[idx]
			link.Ref = ref
			link.Edge = uint8(0)
			link.Side = 0xff
			link.BMin = 0
			link.BMax = 0
			// Add to linked list.
			link.Next = poly.FirstLink
			poly.FirstLink = idx
		}

		// Start end-point is always connect back to off-mesh connection.
		tidx := allocLink(tile)
		if tidx != nullLink {
			landPolyIdx := uint16(m.decodePolyIDPoly(ref))
			landPoly := &tile.Polys[landPolyIdx]
			link := &tile.Links[tidx]
			link.Ref = base | PolyRef(con.Poly)
			link.Edge = 0xff
			link.Side = 0xff
			link.BMin = 0
			link.BMax = 0
			// Add to linked list.
			link.Next = landPoly.FirstLink
			landPoly.FirstLink = tidx
		}
	}
}

// FindNearestPolyInTile finds the nearest polygon within a tile.
func (m *NavMesh) FindNearestPolyInTile(tile *MeshTile, center, extents, nearestPt d3.Vec3) PolyRef {
	bmin := center.Sub(extents)
	bmax := center.Add(extents)

	// Get nearby polygons from proximity grid.
	var polys [128]PolyRef
	polyCount := m.queryPolygonsInTile(tile, bmin, bmax, polys[:], 128)

	// Find nearest polygon amongst the nearby polygons.
	var (
		nearest            PolyRef
		nearestDistanceSqr float32 = math.MaxFloat32
	)
	for i := int32(0); i < polyCount; i++ {
		ref := polys[i]
		var (
			posOverPoly bool
			d           float32
		)
		closestPtPoly := d3.NewVec3()
		m.closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly)

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		diff := center.Sub(closestPtPoly)
		if posOverPoly {
			d = math32.Abs(diff[1]) - tile.Header.WalkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = diff.LenSqr()
		}

		if d <= nearestDistanceSqr {
			nearestPt.Assign(closestPtPoly)
			nearestDistanceSqr = d
			nearest = ref
		}
	}

	return nearest
}

// QueryPolygonsInTile queries polygons within a tile.
func (m *NavMesh) queryPolygonsInTile(
	tile *MeshTile,
	qmin, qmax d3.Vec3,
	polys []PolyRef,
	maxPolys int32) int32 {

	if tile.BvTree != nil {
		var (
			node            *BvNode
			nodeIdx, endIdx int32
			tbmin, tbmax    d3.Vec3
			qfac            float32
			bmin, bmax      [3]uint16
		)
		nodeIdx = 0
		endIdx = tile.Header.BvNodeCount

		tbmin = d3.NewVec3From(tile.Header.BMin[:])
		tbmax = d3.NewVec3From(tile.Header.BMax[:])
		qfac = tile.Header.BvQuantFactor

		// Calculate quantized box
		// clamp query box to world box.
		minx := f32.Clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := f32.Clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := f32.Clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := f32.Clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := f32.Clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := f32.Clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
		// Quantize
		bmin[0] = uint16(uint32(qfac*minx) & 0xfffe)
		bmin[1] = uint16(uint32(qfac*miny) & 0xfffe)
		bmin[2] = uint16(uint32(qfac*minz) & 0xfffe)
		bmax[0] = uint16(uint32(qfac*maxx+1) | 1)
		bmax[1] = uint16(uint32(qfac*maxy+1) | 1)
		bmax[2] = uint16(uint32(qfac*maxz+1) | 1)

		// Traverse tree
		base := m.polyRefBase(tile)
		var n int32
		for nodeIdx < endIdx {
			node = &tile.BvTree[nodeIdx]
			overlap := OverlapQuantBounds(bmin[:], bmax[:], node.BMin[:], node.BMax[:])
			isLeafNode := node.I >= 0

			if isLeafNode && overlap {
				if n < maxPolys {
					n++
					polys[n] = base | PolyRef(node.I)
				}
			}

			if overlap || isLeafNode {
				nodeIdx++
			} else {
				escapeIndex := -node.I
				nodeIdx += escapeIndex
			}
		}

		return n

	}

	var (
		bmin, bmax [3]float32
		n, i       int32
	)
	base := m.polyRefBase(tile)
	for i = 0; i < tile.Header.PolyCount; i++ {
		p := &tile.Polys[i]
		// Do not return off-mesh connection polygons.
		if p.Type() == polyTypeOffMeshConnection {
			continue
		}
		// Calc polygon bounds.
		v := tile.Verts[p.Verts[0]*3 : 3]
		d3.Vec3(bmin[:]).Assign(v)
		d3.Vec3(bmax[:]).Assign(v)
		var j uint8
		for j = 1; j < p.VertCount; j++ {
			v = tile.Verts[p.Verts[j]*3 : 3]
			d3.Vec3Min(bmin[:], v)
			d3.Vec3Max(bmax[:], v)
		}
		if OverlapBounds(qmin, qmax, bmin[:], bmax[:]) {
			if n < maxPolys {
				n++
				polys[n] = base | PolyRef(i)
			}
		}
	}
	return n
}

// decodePolyIdPoly extracts the polygon's index (within its tile) from the
// specified polygon reference.
//
//	See encodePolyID
func (m *NavMesh) decodePolyIDPoly(ref PolyRef) uint32 {
	polyMask := PolyRef((1 << m.polyBits) - 1)
	return uint32(ref & polyMask)
}

// closestPointOnPoly finds the closest point on the specified polygon.
//
//	Arguments:
//	 [in] ref          The reference id of the polygon.
//	 [in] pos          The position to check. [(x, y, z)]
//	 [out]closest      The closest point on the polygon. [(x, y, z)]
//	 [out]posOverPoly  True of the position is over the polygon.
func (m *NavMesh) closestPointOnPoly(ref PolyRef, pos, closest d3.Vec3, posOverPoly *bool) {
	var (
		tile *MeshTile
		poly *Poly
	)

	m.TileAndPolyByRefUnsafe(ref, &tile, &poly)

	// Off-mesh connections don't have detail polygons.
	if poly.Type() == polyTypeOffMeshConnection {
		var (
			v0, v1    d3.Vec3
			d0, d1, u float32
		)
		v0 = tile.Verts[poly.Verts[0]*3 : 3]
		v1 = tile.Verts[poly.Verts[1]*3 : 3]
		d0 = pos.Dist(v0)
		d1 = pos.Dist(v1)
		u = d0 / (d0 + d1)
		closest = v0.Lerp(v1, u)
		if posOverPoly != nil {
			*posOverPoly = false
		}
		return
	}

	ip := (uintptr(unsafe.Pointer(poly)) - uintptr(unsafe.Pointer(&tile.Polys[0]))) / unsafe.Sizeof(*poly)
	pd := &tile.DetailMeshes[uint32(ip)]

	// Clamp point to be inside the polygon.
	verts := make([]float32, VertsPerPolygon*3)
	edged := make([]float32, VertsPerPolygon)
	edget := make([]float32, VertsPerPolygon)
	nv := poly.VertCount
	var i uint8
	for i = 0; i < nv; i++ {
		idx := i * 3
		jdx := poly.Verts[i] * 3
		copy(verts[idx:], tile.Verts[jdx:jdx+3])
	}

	closest.Assign(pos)
	if !distancePtPolyEdgesSqr(pos, verts, int32(nv), edged, edget) {
		// Point is outside the polygon, clamp to nearest edge.
		dmin := edged[0]
		var imin uint8
		for i = 1; i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = i
			}
		}
		va := d3.NewVec3From(verts[imin*3 : imin*3+3])
		vidx := ((imin + 1) % nv) * 3
		vb := d3.NewVec3From(verts[vidx : vidx+3])
		closest = va.Lerp(vb, edget[imin])

		if posOverPoly != nil {
			*posOverPoly = false
		}
	} else {
		if posOverPoly != nil {
			*posOverPoly = true
		}
	}

	// Find height at the location.
	var j uint8
	for j = 0; j < pd.TriCount; j++ {
		vidx := (pd.TriBase + uint32(j)) * 4
		t := tile.DetailTris[vidx : vidx+3]
		var (
			v [3]d3.Vec3
			k int
		)
		for k = 0; k < 3; k++ {
			if t[k] < poly.VertCount {
				vidx := poly.Verts[t[k]] * 3
				v[k] = tile.Verts[vidx : vidx+3]
			} else {
				vidx := (pd.VertBase + uint32(t[k]-poly.VertCount)) * 3
				v[k] = tile.DetailVerts[vidx : vidx+3]
			}
		}
		var h float32
		if closestHeightPointTriangle(closest, v[0], v[1], v[2], &h) {
			closest[1] = h
			break
		}
	}
}

// TileAndPolyByRefUnsafe returns the tile and polygon for the specified polygon
// reference.
//
// Warning: only use this function if it is known that the provided polygon
// reference is valid. This function is faster than TileAndPolyByRef, but it
// does not validate the reference.
// TODO: Use Go-idioms: change signature and returns tile and poly
func (m *NavMesh) TileAndPolyByRefUnsafe(ref PolyRef, tile **MeshTile, poly **Poly) {
	var salt, it, ip uint32
	m.DecodePolyID(ref, &salt, &it, &ip)
	*tile = &m.Tiles[it]
	*poly = &m.Tiles[it].Polys[ip]
}

// DecodePolyID decodes a standard polygon reference.
//
//	Arguments:
//	 [in]ref    The polygon reference to decode.
//	 [out]salt  The tile's salt value.
//	 [out]it    The index of the tile.
//	 [out]ip    The index of the polygon within the tile.
//
// see encodePolyID
// TODO: Use Go-idioms: change signature and returns salt, it and ip
func (m *NavMesh) DecodePolyID(ref PolyRef, salt, it, ip *uint32) {
	saltMask := (PolyRef(1) << m.saltBits) - 1
	tileMask := (PolyRef(1) << m.tileBits) - 1
	polyMask := (PolyRef(1) << m.polyBits) - 1

	*salt = uint32((ref >> (m.polyBits + m.tileBits)) & saltMask)
	*it = uint32((ref >> m.polyBits) & tileMask)
	*ip = uint32(ref & polyMask)
}

// Builds external polygon links for a tile.
func (m *NavMesh) connectExtOffMeshLinks(tile, target *MeshTile, side int32) {
	if tile == nil {
		return
	}

	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	var oppositeSide uint8
	if side == -1 {
		oppositeSide = 0xff
	} else {
		oppositeSide = uint8(oppositeTile(side))
	}

	var i int32
	for i = 0; i < target.Header.OffMeshConCount; i++ {
		targetCon := &target.OffMeshCons[i]
		if targetCon.Side != oppositeSide {
			continue
		}

		targetPoly := &target.Polys[targetCon.Poly]
		// Skip off-mesh connections which start location could not be connected at all.
		if targetPoly.FirstLink == nullLink {
			continue
		}

		ext := []float32{targetCon.Rad, target.Header.WalkableClimb, targetCon.Rad}

		// Find polygon to connect to.
		p := targetCon.Pos[3:6]
		nearestPt := d3.NewVec3()
		ref := m.FindNearestPolyInTile(tile, p, ext, nearestPt)
		if ref == 0 {
			continue
		}

		// findNearestPoly may return too optimistic results, further check to make sure.
		if math32.Sqr(nearestPt[0]-p[0])+math32.Sqr(nearestPt[2]-p[2]) > math32.Sqr(targetCon.Rad) {
			continue
		}

		// Make sure the location is on current mesh.
		var v d3.Vec3 = target.Verts[targetPoly.Verts[1]*3:]
		v.Assign(nearestPt)

		// Link off-mesh connection to target poly.
		idx := allocLink(target)
		if idx != nullLink {
			link := &target.Links[idx]
			link.Ref = ref
			link.Edge = uint8(1)
			link.Side = oppositeSide
			link.BMin = 0
			link.BMax = 0
			// Add to linked list.
			link.Next = targetPoly.FirstLink
			targetPoly.FirstLink = idx
		}

		// Link target poly to off-mesh connection.
		if (uint32(targetCon.Flags) & offMeshConBidir) != 0 {
			tidx := allocLink(tile)
			if tidx != nullLink {
				landPolyIdx := uint16(m.decodePolyIDPoly(ref))
				landPoly := &tile.Polys[landPolyIdx]
				link := &tile.Links[tidx]
				link.Ref = m.polyRefBase(target) | PolyRef(targetCon.Poly)
				link.Edge = 0xff
				if side == -1 {
					link.Side = 0xff
				} else {
					link.Side = uint8(side)
				}
				link.BMin = 0
				link.BMax = 0
				// Add to linked list.
				link.Next = landPoly.FirstLink
				landPoly.FirstLink = tidx
			}
		}
	}
}

// TilesAt returns all tiles at the specified grid location. (All layers.)
//
//	Arguments:
//	 [in] x          The tile's x-location. (x, y)
//	 [in] y          The tile's y-location. (x, y)
//	 [out]tiles      A pointer to an array of tiles that will hold the result.
//	 [in] maxTiles   The maximum tiles the tiles parameter can hold.
//
// Return The number of tiles returned in the tiles array.
//
// Note: This function will not fail if the tiles array is too small to hold the
// entire result set. It will simply fill the array to capacity.
func (m *NavMesh) TilesAt(x, y int32, tiles []*MeshTile, maxTiles int32) int32 {
	var n int32

	// Find tile based on hash.
	h := computeTileHash(x, y, m.TileLUTMask)
	tile := m.posLookup[h]
	for tile != nil {
		if tile.Header != nil && tile.Header.X == x && tile.Header.Y == y {
			if n < maxTiles {
				tiles[n] = tile
				n++
			}
		}
		tile = tile.Next
	}
	return n
}

// Builds external polygon links for a tile.
func (m *NavMesh) connectExtLinks(tile, target *MeshTile, side int32) {
	if tile == nil {
		return
	}

	// Connect border links.
	var i int32
	for i = 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]

		// Create new links.
		nv := poly.VertCount
		var j int32
		for j = 0; j < int32(nv); j++ {
			// Skip non-portal edges.
			if (poly.Neis[j] & extLink) == 0 {
				continue
			}

			dir := (int32)(poly.Neis[j] & 0xff)
			if side != -1 && dir != side {
				continue
			}

			// Create new links
			va := tile.Verts[poly.Verts[j]*3:]
			vb := tile.Verts[poly.Verts[(j+1)%int32(nv)]*3:]
			nei := make([]PolyRef, 4)
			neia := make([]float32, 4*2)
			nnei := m.findConnectingPolys(va, vb, target, oppositeTile(dir), nei, neia, 4)

			var k int32
			for k = 0; k < nnei; k++ {
				idx := allocLink(tile)
				if idx != nullLink {
					link := &tile.Links[idx]
					link.Ref = nei[k]
					link.Edge = uint8(j)
					link.Side = uint8(dir)

					link.Next = poly.FirstLink
					poly.FirstLink = idx

					// Compress portal limits to a byte value.
					if dir == 0 || dir == 4 {
						tmin := (neia[k*2+0] - va[2]) / (vb[2] - va[2])
						tmax := (neia[k*2+1] - va[2]) / (vb[2] - va[2])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.BMin = uint8(f32.Clamp(tmin, 0.0, 1.0) * 255.0)
						link.BMax = uint8(f32.Clamp(tmax, 0.0, 1.0) * 255.0)
					} else if dir == 2 || dir == 6 {
						tmin := (neia[k*2+0] - va[0]) / (vb[0] - va[0])
						tmax := (neia[k*2+1] - va[0]) / (vb[0] - va[0])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.BMin = uint8(f32.Clamp(tmin, 0.0, 1.0) * 255.0)
						link.BMax = uint8(f32.Clamp(tmax, 0.0, 1.0) * 255.0)
					}
				}
			}
		}
	}
}

// Returns all polygons in neighbour tile based
// on portal defined by the segment.
func (m *NavMesh) findConnectingPolys(
	va, vb []float32,
	tile *MeshTile,
	side int32,
	con []PolyRef,
	conarea []float32,
	maxcon int32) int32 {

	if tile == nil {
		return 0
	}

	amin := make([]float32, 2)
	amax := make([]float32, 2)
	calcSlabEndPoints(va, vb, amin, amax, side)
	apos := slabCoord(va, side)

	// Remove links pointing to 'side' and compact the links array.
	bmin := make([]float32, 2)
	bmax := make([]float32, 2)
	l := extLink | uint16(side)
	var n int32

	base := m.polyRefBase(tile)

	var i int32
	for i = 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		nv := poly.VertCount
		var j uint8
		for j = 0; j < nv; j++ {
			// Skip edges which do not point to the right side.
			if poly.Neis[j] != l {
				continue
			}

			idx := poly.Verts[j] * 3
			vc := tile.Verts[idx : idx+3]
			idx = poly.Verts[(j+1)%nv] * 3
			vd := tile.Verts[idx : idx+3]
			bpos := slabCoord(vc, side)

			// Segments are not close enough.
			if math32.Abs(apos-bpos) > 0.01 {
				continue
			}

			// Check if the segments touch.
			calcSlabEndPoints(vc, vd, bmin, bmax, side)

			if !overlapSlabs(amin, amax, bmin, bmax, 0.01, tile.Header.WalkableClimb) {
				continue
			}

			// Add return value.
			if n < maxcon {
				conarea[n*2+0] = math32.Max(amin[0], bmin[0])
				conarea[n*2+1] = math32.Min(amax[0], bmax[0])
				con[n] = base | PolyRef(i)
				n++
			}
			break
		}
	}
	return n
}

func (m *NavMesh) unconnectLinks(tile *MeshTile, target *MeshTile) {
	if tile == nil || target == nil {
		return
	}

	targetNum := m.decodePolyIDTile(PolyRef(m.TileRef(target)))

	for i := int32(0); i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		j := poly.FirstLink
		pj := nullLink
		for j != nullLink {
			if m.decodePolyIDTile(tile.Links[j].Ref) == targetNum {
				// Remove link.
				nj := tile.Links[j].Next
				if pj == nullLink {
					poly.FirstLink = nj
				} else {
					tile.Links[pj].Next = nj
				}
				freeLink(tile, j)
				j = nj
			} else {
				// Advance
				pj = j
				j = tile.Links[j].Next
			}
		}
	}
}

func calcSlabEndPoints(va, vb d3.Vec3, bmin, bmax []float32, side int32) {
	if side == 0 || side == 4 {
		if va[2] < vb[2] {
			bmin[0] = va[2]
			bmin[1] = va[1]
			bmax[0] = vb[2]
			bmax[1] = vb[1]
		} else {
			bmin[0] = vb[2]
			bmin[1] = vb[1]
			bmax[0] = va[2]
			bmax[1] = va[1]
		}
	} else if side == 2 || side == 6 {
		if va[0] < vb[0] {
			bmin[0] = va[0]
			bmin[1] = va[1]
			bmax[0] = vb[0]
			bmax[1] = vb[1]
		} else {
			bmin[0] = vb[0]
			bmin[1] = vb[1]
			bmax[0] = va[0]
			bmax[1] = va[1]
		}
	}
}

func slabCoord(va d3.Vec3, side int32) float32 {
	if side == 0 || side == 4 {
		return va[0]
	} else if side == 2 || side == 6 {
		return va[2]
	}
	return 0
}

func overlapSlabs(amin, amax, bmin, bmax d3.Vec3, px, py float32) bool {
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	minx := math32.Max(amin[0]+px, bmin[0]+px)
	maxx := math32.Min(amax[0]-px, bmax[0]-px)
	if minx > maxx {
		return false
	}

	// Check vertical overlap.
	ad := (amax[1] - amin[1]) / (amax[0] - amin[0])
	ak := amin[1] - ad*amin[0]
	bd := (bmax[1] - bmin[1]) / (bmax[0] - bmin[0])
	bk := bmin[1] - bd*bmin[0]
	aminy := ad*minx + ak
	amaxy := ad*maxx + ak
	bminy := bd*minx + bk
	bmaxy := bd*maxx + bk
	dmin := bminy - aminy
	dmax := bmaxy - amaxy

	// Crossing segments always overlap.
	if dmin*dmax < 0 {
		return true
	}

	// Check for overlap at endpoints.
	thr := math32.Sqr(py * 2)
	if dmin*dmin <= thr || dmax*dmax <= thr {
		return true
	}

	return false
}

// Returns neighbour tile based on side.
func (m *NavMesh) neighbourTilesAt(x, y, side int32, tiles []*MeshTile, maxTiles int32) int32 {
	nx := x
	ny := y
	switch side {
	case 0:
		nx++
	case 1:
		nx++
		ny++
	case 2:
		ny++
	case 3:
		nx--
		ny++
	case 4:
		nx--
	case 5:
		nx--
		ny--
	case 6:
		ny--
	case 7:
		nx++
		ny--
	}

	return m.TilesAt(nx, ny, tiles, maxTiles)
}

// TileRefAt returns the tile reference for the tile at specified grid location.
//
//	Arguments:
//	 x       The tile's x-location. (x, y, layer)
//	 y       The tile's y-location. (x, y, layer)
//	 layer   The tile's layer. (x, y, layer)
//
// Return The tile reference of the tile, or 0 if there is none.
// TODO: verify that this function is equivalent with this other version
// (commented out below)
func (m *NavMesh) TileRefAt(x, y, layer int32) TileRef {
	// Find tile based on hash.
	h := computeTileHash(x, y, m.TileLUTMask)
	tile := m.posLookup[h]
	for tile != nil {
		if tile.Header != nil &&
			tile.Header.X == x &&
			tile.Header.Y == y &&
			tile.Header.Layer == layer {
			return m.TileRef(tile)
		}
		tile = tile.Next
	}
	return 0
}

func (m *NavMesh) TileByRef(ref TileRef) *MeshTile {
	if ref == 0 {
		return nil
	}
	tileIndex := m.decodePolyIDTile(PolyRef(ref))
	tileSalt := m.decodePolyIDSalt(PolyRef(ref))
	if int32(tileIndex) >= m.MaxTiles {
		return nil
	}
	tile := &m.Tiles[tileIndex]
	if tile.Salt != tileSalt {
		return nil
	}
	return tile
}

/*
func (m *NavMesh) TileRefAt(x, y, layer int32) TileRef {
	// Find tile based on hash.
	h := computeTileHash(x, y, m.TileLUTMask)
	tile := &m.posLookup[h]
	for *tile != nil {
		if (*tile).Header != nil &&
			(*tile).Header.X == x &&
			(*tile).Header.Y == y &&
			(*tile).Header.Layer == layer {
			return m.TileRef(*tile)
		}
		tile = &(*tile).Next
	}
	return 0
}
*/

// TileRef returns the tile reference for the specified tile.
func (m *NavMesh) TileRef(tile *MeshTile) TileRef {
	if tile == nil {
		return 0
	}

	it := (uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.Tiles[0]))) / unsafe.Sizeof(*tile)
	return TileRef(m.encodePolyID(tile.Salt, uint32(it), 0))
}

// IsValidPolyRef checks the validity of a polygon reference.
func (m *NavMesh) IsValidPolyRef(ref PolyRef) bool {
	if ref == 0 {
		return false
	}
	var salt, it, ip uint32
	m.DecodePolyID(ref, &salt, &it, &ip)
	if it >= uint32(m.MaxTiles) {
		return false
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return false
	}
	if ip >= uint32(m.Tiles[it].Header.PolyCount) {
		return false
	}
	return true
}

// TileAndPolyByRef returns the tile and polygon for the specified polygon
// reference.
//
//	Arguments:
//	 [in] ref     A known valid reference for a polygon.
//	 [out]tile    The tile containing the polygon.
//	 [out]poly    The polygon.
//
// TODO: Use Go-idioms: change signature and returns tile and poly
func (m *NavMesh) TileAndPolyByRef(ref PolyRef, tile **MeshTile, poly **Poly) Status {
	if ref == 0 {
		return Failure
	}
	var salt, it, ip uint32
	m.DecodePolyID(ref, &salt, &it, &ip)
	if it >= uint32(m.MaxTiles) {
		return Failure | InvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return Failure | InvalidParam
	}
	if ip >= uint32(m.Tiles[it].Header.PolyCount) {
		return Failure | InvalidParam
	}
	*tile = &m.Tiles[it]
	*poly = &m.Tiles[it].Polys[ip]
	return Success
}

// CalcTileLoc calculates the tile grid location for the specified world
// position.
//
//	Arguments:
//	 [in] pos  The world position for the query. [(x, y, z)]
//	 [out]tx   The tile's x-location. (x, y)
//	 [out]ty   The tile's y-location. (x, y)
func (m *NavMesh) CalcTileLoc(pos d3.Vec3) (tx, ty int32) {
	tx = int32(math32.Floor((pos[0] - m.Orig[0]) / m.TileWidth))
	ty = int32(math32.Floor((pos[2] - m.Orig[2]) / m.TileHeight))
	return tx, ty
}
