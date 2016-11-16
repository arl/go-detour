package detour

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

/// A navigation mesh based on tiles of convex polygons.
type DtNavMesh struct {
	Params                DtNavMeshParams // Current initialization params. TODO: do not store this info twice.
	Orig                  d3.Vec3         // Origin of the tile (0,0)
	TileWidth, TileHeight float32         // Dimensions of each tile.
	MaxTiles              int32           // Max number of tiles.
	TileLUTSize           int32           // Tile hash lookup size (must be pot).
	TileLUTMask           int32           // Tile hash lookup mask.
	posLookup             []*DtMeshTile   // Tile hash lookup.
	nextFree              *DtMeshTile     // Freelist of tiles.
	Tiles                 []DtMeshTile    // List of tiles.
	saltBits              uint32          // Number of salt bits in the tile ID.
	tileBits              uint32          // Number of tile bits in the tile ID.
	polyBits              uint32          // Number of poly bits in the tile ID.
}

func (m *DtNavMesh) init(params *DtNavMeshParams) DtStatus {
	m.Params = *params
	m.Orig = d3.NewVec3From(params.Orig[0:3])
	m.TileWidth = params.TileWidth
	m.TileHeight = params.TileHeight

	// Init tiles
	m.MaxTiles = int32(params.MaxTiles)
	m.TileLUTSize = int32(dtNextPow2(uint32(params.MaxTiles / 4)))
	if !(m.TileLUTSize == 0) {
		m.TileLUTSize = 1
	}
	m.TileLUTMask = m.TileLUTSize - 1

	m.Tiles = make([]DtMeshTile, m.MaxTiles, m.MaxTiles)
	m.posLookup = make([]*DtMeshTile, m.TileLUTSize, m.TileLUTSize)
	m.nextFree = nil
	for i := m.MaxTiles - 1; i >= 0; i-- {
		m.Tiles[i].Salt = 1
		m.Tiles[i].Next = m.nextFree
		m.nextFree = &m.Tiles[i]
	}

	// Init ID generator values.
	m.tileBits = dtIlog2(dtNextPow2(uint32(params.MaxTiles)))
	m.polyBits = dtIlog2(dtNextPow2(uint32(params.MaxPolys)))
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	if 31 < 32-m.tileBits-m.polyBits {
		m.saltBits = 31
	} else {
		m.saltBits = 32 - m.tileBits - m.polyBits
	}

	if m.saltBits < 10 {
		return DtStatus(DT_FAILURE | DT_INVALID_PARAM)
	}

	return DT_SUCCESS
}

// addTile adds a tile to the navigation mesh.
//  @param[in]		data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
//  @param[in]		dataSize	Data size of the new tile mesh.
//  @param[in]		flags		Tile flags. (See: #dtTileFlags)
//  @param[in]		lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
//  @param[out]	result		The tile reference. (If the tile was succesfully added.) [opt]
// @return The status flags for the operation.
//
// The add operation will fail if the data is in the wrong format, the allocated tile
// space is full, or there is a tile already at the specified reference.
//
// The lastRef parameter is used to restore a tile with the same tile
// reference it had previously used.  In this case the #DtPolyRef's for the
// tile will be restored to the same values they were before the tile was
// removed.
//
// The nav mesh assumes exclusive access to the data passed and will make
// changes to the dynamic portion of the data. For that reason the data
// should not be reused in other nav meshes until the tile has been successfully
// removed from this nav mesh.
//
// @see dtCreateNavMeshData, #removeTileBvTree
func (m *DtNavMesh) addTile(data []byte, dataSize int32, lastRef dtTileRef, result *dtTileRef) DtStatus {
	var hdr DtMeshHeader

	// prepare a reader on the received data
	r := newAlignedReader(bytes.NewReader(data), 4)
	binary.Read(r, binary.LittleEndian, &hdr)

	// Make sure the data is in right format.
	if hdr.Magic != DT_NAVMESH_MAGIC {
		return DT_FAILURE | DT_WRONG_MAGIC
	}
	if hdr.Version != DT_NAVMESH_VERSION {
		return DT_FAILURE | DT_WRONG_VERSION
	}

	// Make sure the location is free.
	if m.TileAt(hdr.X, hdr.Y, hdr.Layer) != nil {
		fmt.Println("TileAt failed")
		return DT_FAILURE
	}

	// Allocate a tile.
	var tile *DtMeshTile
	if lastRef == 0 {
		if m.nextFree != nil {
			tile = m.nextFree
			m.nextFree = tile.Next
			tile.Next = nil
		}
	} else {
		// Try to relocate the tile to specific index with same salt.
		tileIndex := int32(m.decodePolyIdTile(DtPolyRef(lastRef)))
		if tileIndex >= m.MaxTiles {
			log.Fatalln("tileIndex >= m.m_maxTiles", tileIndex, m.MaxTiles)
			return DT_FAILURE | DT_OUT_OF_MEMORY
		}
		// Try to find the specific tile id from the free list.
		target := &m.Tiles[tileIndex]
		var prev *DtMeshTile
		tile = m.nextFree
		for tile != nil && tile != target {
			prev = tile
			tile = tile.Next
		}
		// Could not find the correct location.
		if tile != target {
			log.Fatalln("couldn't find the correct tile location")
			return DT_FAILURE | DT_OUT_OF_MEMORY
		}
		// Remove from freelist
		if prev == nil {
			m.nextFree = tile.Next
		} else {
			prev.Next = tile.Next
		}

		// Restore salt.
		tile.Salt = m.decodePolyIdSalt(DtPolyRef(lastRef))
	}

	// Make sure we could allocate a tile.
	if tile == nil {
		log.Fatalln("couldn't allocate tile")
		return DT_FAILURE | DT_OUT_OF_MEMORY
	}

	// Insert tile into the position lut.
	h := computeTileHash(hdr.X, hdr.Y, m.TileLUTMask)
	tile.Next = m.posLookup[h]
	m.posLookup[h] = tile

	// Read header from binary data
	tile.Verts = make([]float32, 3*hdr.VertCount)
	var err error
	if err = r.readSlice(&tile.Verts, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.Verts:", err)
	}

	tile.Polys = make([]DtPoly, hdr.PolyCount)
	if err = r.readSlice(&tile.Polys, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.Polys:", err)
	}

	tile.Links = make([]DtLink, hdr.MaxLinkCount)
	if err = r.readSlice(&tile.Links, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.Links:", err)
	}

	tile.DetailMeshes = make([]DtPolyDetail, hdr.DetailMeshCount)
	if err = r.readSlice(&tile.DetailMeshes, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.DetailMeshes:", err)
	}

	tile.DetailVerts = make([]float32, 3*hdr.DetailVertCount)
	if err = r.readSlice(&tile.DetailVerts, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.DetailVerts:", err)
	}

	// TODO: check the code that use detailTris, in the original c/c++ code, the
	// 4th, and unused byte if kept in memory for alignment reasons. We keep
	// here for reference both methods, the first just keeps the first 3 bytes,
	// nothing is wasted
	if false {
		tile.DetailTris = make([]uint8, 3*hdr.DetailTriCount)
		for i := int32(0); i < hdr.DetailTriCount; i++ {
			// one Detail Tri takes actually 4 bytes (for alignment) even if only
			// the first 3 are significant. So we read them 4 by 4 and just keep the
			// first 3.
			tmp := make([]uint8, 4)
			if err = binary.Read(r, binary.LittleEndian, &tmp); err != nil {
				log.Fatalln("couldn't read tile.DetailTris:", err)
			}
			copy(tile.DetailTris[i*3:i*3+3], tmp[0:3])
		}

	} else {
		// this second method keep all, we just have to adjust when indexing the
		// DetailTris slice for usage
		tile.DetailTris = make([]uint8, 4*hdr.DetailTriCount)
		if err = binary.Read(r, binary.LittleEndian, &tile.DetailTris); err != nil {
			log.Fatalln("couldn't read tile.DetailTris:", err)
		}
	}

	tile.BvTree = make([]dtBVNode, hdr.BvNodeCount)
	if err = r.readSlice(&tile.BvTree, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.BvTree:", err)
	}

	for _, node := range tile.BvTree {
		if node.I > hdr.BvNodeCount {
			assert.True(false, "node.I > hdr.BvNodeCount: 0x%x 0x%x\n", node.I, hdr.BvNodeCount)
		}
	}

	tile.OffMeshCons = make([]DtOffMeshConnection, hdr.OffMeshConCount)
	if err = r.readSlice(&tile.OffMeshCons, binary.LittleEndian); err != nil {
		log.Fatalln("couldn't read tile.OffMeshCons:", err)
	}

	// If there are no items in the bvtree, reset the tree pointer.
	if len(tile.BvTree) == 0 {
		tile.BvTree = nil
	}

	// Build links freelist
	tile.LinksFreeList = 0
	tile.Links[hdr.MaxLinkCount-1].Next = DT_NULL_LINK
	var i int32
	for ; i < hdr.MaxLinkCount-1; i++ {
		tile.Links[i].Next = uint32(i + 1)
	}

	// Init tile.
	tile.Header = &hdr
	tile.Data = data
	tile.DataSize = dataSize
	tile.Flags = 0

	m.connectIntLinks(tile)

	// Base off-mesh connections to their starting polygons and connect
	// connections inside the tile.
	m.baseOffMeshLinks(tile)
	m.connectExtOffMeshLinks(tile, tile, -1)

	// Create connections with neighbour tiles.
	MAX_NEIS := int32(32)
	neis := make([]*DtMeshTile, MAX_NEIS)
	var nneis int32

	// Connect with layers in current tile.
	nneis = m.TilesAt(hdr.X, hdr.Y, neis, MAX_NEIS)
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
		nneis = m.NeighbourTilesAt(hdr.X, hdr.Y, i, neis, MAX_NEIS)
		for j = 0; j < nneis; j++ {
			//log.Println("connecting with neighbour tiles", neis[j])
			m.connectExtLinks(tile, neis[j], i)
			m.connectExtLinks(neis[j], tile, dtOppositeTile(i))
			m.connectExtOffMeshLinks(tile, neis[j], i)
			m.connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i))
		}
	}

	if result != nil {
		*result = m.TileRef(tile)
	}

	return DT_SUCCESS
}

func (m *DtNavMesh) TileAt(x, y, layer int32) *DtMeshTile {
	var (
		h    int32
		tile *DtMeshTile
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

func (m *DtNavMesh) connectIntLinks(tile *DtMeshTile) {
	if tile == nil {
		return
	}

	var (
		i    int32
		base DtPolyRef
	)
	base = m.getPolyRefBase(tile)

	for i = 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		poly.FirstLink = DT_NULL_LINK

		if poly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
			continue
		}

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for j := int32(poly.VertCount - 1); j >= 0; j-- {
			// Skip hard and non-internal edges.
			if poly.Neis[j] == 0 || ((poly.Neis[j] & DT_EXT_LINK) != 0) {
				continue
			}

			idx := allocLink(tile)
			if idx != DT_NULL_LINK {
				link := &tile.Links[idx]
				link.Ref = base | DtPolyRef(poly.Neis[j]-1)
				link.Edge = uint8(j)
				link.Side = 0xff
				link.Bmin = 0
				link.Bmax = 0
				// Add to linked list.
				link.Next = poly.FirstLink
				poly.FirstLink = idx
			}
		}
	}
}

// getPolyRefBase returns the polygon reference for the base polygon in the
//specified tile.
//
// Example use case:
//  base := navmesh.GetPolyRefBase(tile);
//  for i = 0; i < tile.Header.PolyCount; i++ {
//      poly = &tile.polys[i]
//      ref := base | DtPolyRef(i)
//
//      // Use the reference to access the polygon data.
//  }
func (m *DtNavMesh) getPolyRefBase(tile *DtMeshTile) DtPolyRef {
	if tile == nil {
		return 0
	}

	e := uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.Tiles[0]))
	ip := uint32(e / unsafe.Sizeof(*tile))

	assert.True(ip < uint32(len(m.Tiles)),
		"we should have ip < len(m.Tiles), instead ip = %d and len(m.Tiles) = %d", ip, len(m.Tiles))

	return m.encodePolyId(tile.Salt, ip, 0)
}

func computeTileHash(x, y, mask int32) int32 {
	h1 := 0x8da6b343 // Large multiplicative constants;
	h2 := 0xd8163841 // here arbitrarily chosen primes
	n := h1*int(x) + h2*int(y)
	return int32(n) & mask
}

func allocLink(tile *DtMeshTile) uint32 {
	if tile.LinksFreeList == DT_NULL_LINK {
		return DT_NULL_LINK
	}
	link := tile.LinksFreeList
	tile.LinksFreeList = tile.Links[link].Next
	return link
}

func freeLink(tile *DtMeshTile, link uint32) {
	tile.Links[link].Next = tile.LinksFreeList
	tile.LinksFreeList = link
}

// @name Encoding and Decoding
// These functions are generally meant for internal use only.

// encodePolyId derives a standard polygon reference.
//  salt[in] The tile's salt value.
//  it  [in] The index of the tile.
//  ip  [in] The index of the polygon within the tile.
func (m *DtNavMesh) encodePolyId(salt, it, ip uint32) DtPolyRef {
	return (DtPolyRef(salt) << (m.polyBits + m.tileBits)) | (DtPolyRef(it) << m.polyBits) | DtPolyRef(ip)
}

type DtPolyRef uint32

// DtLink defines a link between polygons.
//
// Note: This structure is rarely if ever used by the end user.
// @see DtMeshTile
type DtLink struct {
	Ref  DtPolyRef // Neighbour reference. (The neighbor that is linked to.)
	Next uint32    // Index of the next link.
	Edge uint8     // Index of the polygon edge that owns this link.
	Side uint8     // If a boundary link, defines on which side the link is.
	Bmin uint8     // If a boundary link, defines the minimum sub-edge area.
	Bmax uint8     // If a boundary link, defines the maximum sub-edge area.
}

// Defines the location of detail sub-mesh data within a dtMeshTile.
type DtPolyDetail struct {
	VertBase  uint32 // The offset of the vertices in the DtMeshTile:DetailVerts slice.
	TriBase   uint32 // The offset of the triangles in the DtMeshTile:DetailTris slice.
	VertCount uint8  // The number of vertices in the sub-mesh.
	TriCount  uint8  // The number of triangles in the sub-mesh.
}

// Bounding volume node.
// Note: This structure is rarely if ever used by the end user.
// see DtMeshTile
type dtBVNode struct {
	Bmin [3]uint16 // Minimum bounds of the node's AABB. [(x, y, z)]
	Bmax [3]uint16 // Maximum bounds of the node's AABB. [(x, y, z)]
	I    int32     // The node's index. (Negative for escape sequence.)
}

// DtOffMeshConnection defines an navigation mesh off-mesh connection within a
// DtMeshTile object.
// An off-mesh connection is a user defined traversable connection made up to
// two vertices.
type DtOffMeshConnection struct {
	// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	//Pos [6]float32
	PosA, PosB d3.Vec3

	// The radius of the endpoints. [Limit: >= 0]
	Rad float32

	// The polygon reference of the connection within the tile.
	Poly uint16

	// Link flags.
	// Note: These are not the connection's user defined flags. Those are
	// assigned via the connection's DtPoly definition. These are link flags
	// used for internal purposes.
	Flags uint8

	// End point side.
	Side uint8

	// The id of the offmesh connection. (User assigned when the navigation mesh
	// is built)
	UserId uint
}

const (
	// A magic number used to detect compatibility of navigation tile data.
	DT_NAVMESH_MAGIC int32 = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V'

	// A version number used to detect compatibility of navigation tile data.
	DT_NAVMESH_VERSION = 7

	// A magic number used to detect the compatibility of navigation tile states.
	DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S'

	// A version number used to detect compatibility of navigation tile states.
	DT_NAVMESH_STATE_VERSION = 1
)

// Flags representing the type of a navigation mesh polygon.
type dtPolyTypes uint32

const (
	// The polygon is a standard convex polygon that is part of the surface of the mesh.
	DT_POLYTYPE_GROUND dtPolyTypes = 0
	// The polygon is an off-mesh connection consisting of two vertices.
	DT_POLYTYPE_OFFMESH_CONNECTION = 1
)

const (
	// A flag that indicates that an entity links to an external entity.
	// (E.g. A polygon edge is a portal that links to another polygon.)
	DT_EXT_LINK uint16 = 0x8000

	// A value that indicates the entity does not link to anything.
	DT_NULL_LINK uint32 = 0xffffffff
)

// decodePolyIdTile extracts the tile's index from the specified polygon
// reference.
//  ref[in] The polygon reference.
//  see encodePolyId
func (m *DtNavMesh) decodePolyIdTile(ref DtPolyRef) uint32 {
	tileMask := DtPolyRef((DtPolyRef(1) << m.tileBits) - 1)
	return uint32((ref >> m.polyBits) & tileMask)
}

// Extracts a tile's salt value from the specified polygon reference.
//  ref[in] The polygon reference.
//  see encodePolyId
func (m *DtNavMesh) decodePolyIdSalt(ref DtPolyRef) uint32 {
	saltMask := (DtPolyRef(1) << m.saltBits) - 1
	return uint32((ref >> (m.polyBits + m.tileBits)) & saltMask)
}

func (m *DtNavMesh) baseOffMeshLinks(tile *DtMeshTile) {
	if tile == nil {
		return
	}

	var (
		i    int32
		con  *DtOffMeshConnection
		poly *DtPoly
		base DtPolyRef
	)

	base = m.getPolyRefBase(tile)

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
		p = con.PosA // First vertex
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
		var v d3.Vec3
		v = tile.Verts[poly.Verts[0]*3 : poly.Verts[0]*3+3]
		v.Assign(nearestPt)

		// Link off-mesh connection to target poly.
		idx := allocLink(tile)
		if idx != DT_NULL_LINK {
			link := &tile.Links[idx]
			link.Ref = ref
			link.Edge = uint8(0)
			link.Side = 0xff
			link.Bmin = 0
			link.Bmax = 0
			// Add to linked list.
			link.Next = poly.FirstLink
			poly.FirstLink = idx
		}

		// Start end-point is always connect back to off-mesh connection.
		tidx := allocLink(tile)
		if tidx != DT_NULL_LINK {
			landPolyIdx := uint16(m.decodePolyIdPoly(ref))
			landPoly := &tile.Polys[landPolyIdx]
			link := &tile.Links[tidx]
			link.Ref = base | DtPolyRef(con.Poly)
			link.Edge = 0xff
			link.Side = 0xff
			link.Bmin = 0
			link.Bmax = 0
			// Add to linked list.
			link.Next = landPoly.FirstLink
			landPoly.FirstLink = tidx
		}
	}
}

// FindNearestPolyInTile finds the nearest polygon within a tile.
func (m *DtNavMesh) FindNearestPolyInTile(tile *DtMeshTile, center, extents, nearestPt d3.Vec3) DtPolyRef {
	bmin := center.Sub(extents)
	bmax := center.Add(extents)

	// Get nearby polygons from proximity grid.
	var polys [128]DtPolyRef
	polyCount := m.QueryPolygonsInTile(tile, bmin, bmax, polys[:], 128)

	// Find nearest polygon amongst the nearby polygons.
	var nearest DtPolyRef
	nearestDistanceSqr := math32.MaxFloat32
	var i int32
	for i = 0; i < polyCount; i++ {
		ref := polys[i]
		var (
			posOverPoly bool
			d           float32
		)
		closestPtPoly := d3.NewVec3()
		m.ClosestPointOnPoly(ref, center, closestPtPoly, &posOverPoly)

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
func (m *DtNavMesh) QueryPolygonsInTile(tile *DtMeshTile, qmin, qmax d3.Vec3, polys []DtPolyRef, maxPolys int32) int32 {
	if tile.BvTree != nil {
		var (
			node            *dtBVNode
			nodeIdx, endIdx int32
			tbmin, tbmax    d3.Vec3
			qfac            float32
			bmin, bmax      [3]uint16
		)
		nodeIdx = 0
		endIdx = tile.Header.BvNodeCount

		tbmin = d3.NewVec3From(tile.Header.Bmin[:])
		tbmax = d3.NewVec3From(tile.Header.Bmax[:])
		qfac = tile.Header.BvQuantFactor

		// Calculate quantized box
		// dtClamp query box to world box.
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
		base := m.getPolyRefBase(tile)
		var n int32
		for nodeIdx < endIdx {
			node = &tile.BvTree[nodeIdx]
			overlap := dtOverlapQuantBounds(bmin[:], bmax[:], node.Bmin[:], node.Bmax[:])
			isLeafNode := node.I >= 0

			if isLeafNode && overlap {
				if n < maxPolys {
					n++
					polys[n] = base | DtPolyRef(node.I)
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

	} else {

		bmin := d3.NewVec3()
		bmax := d3.NewVec3()
		var n, i int32
		base := m.getPolyRefBase(tile)
		for i = 0; i < tile.Header.PolyCount; i++ {
			p := &tile.Polys[i]
			// Do not return off-mesh connection polygons.
			if p.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
				continue
			}
			// Calc polygon bounds.
			v := tile.Verts[p.Verts[0]*3 : 3]
			bmin.Assign(v)
			bmax.Assign(v)
			var j uint8
			for j = 1; j < p.VertCount; j++ {
				v = tile.Verts[p.Verts[j]*3 : 3]
				d3.Vec3Min(bmin, v)
				d3.Vec3Max(bmax, v)
			}
			if dtOverlapBounds(qmin, qmax, bmin, bmax) {
				if n < maxPolys {
					n++
					polys[n] = base | DtPolyRef(i)
				}
			}
		}
		return n
	}
}

// DecodePolyIdPoly extracts the polygon's index (within its tile) from the specified polygon reference.
//  ref[in] The polygon reference.
//  See encodePolyId
func (m *DtNavMesh) decodePolyIdPoly(ref DtPolyRef) uint32 {
	polyMask := DtPolyRef((1 << m.polyBits) - 1)
	return uint32(ref & polyMask)
}

// ClosestPointOnPoly finds the closest point on the specified polygon.
// ref         [in]	The reference id of the polygon.
// pos         [in]	The position to check. [(x, y, z)]
// closest     [out]	The closest point on the polygon. [(x, y, z)]
// posOverPoly [out]	True of the position is over the polygon.
func (m *DtNavMesh) ClosestPointOnPoly(ref DtPolyRef, pos, closest d3.Vec3, posOverPoly *bool) {
	var (
		tile *DtMeshTile
		poly *DtPoly
	)

	m.TileAndPolyByRefUnsafe(ref, &tile, &poly)

	// Off-mesh connections don't have detail polygons.
	if poly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
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

	e := uintptr(unsafe.Pointer(poly)) - uintptr(unsafe.Pointer(&tile.Polys[0]))
	ip := uint32(e / unsafe.Sizeof(*poly))

	pd := &tile.DetailMeshes[ip]

	// Clamp point to be inside the polygon.
	verts := make([]float32, DT_VERTS_PER_POLYGON*3)
	edged := make([]float32, DT_VERTS_PER_POLYGON)
	edget := make([]float32, DT_VERTS_PER_POLYGON)
	nv := poly.VertCount
	var i uint8
	for i = 0; i < nv; i++ {
		// TODO: could probably use copy
		idx := i * 3
		jdx := poly.Verts[i] * 3
		copy(verts[idx:idx+3], tile.Verts[jdx:jdx+3])
	}

	closest.Assign(pos)
	if !dtDistancePtPolyEdgesSqr(pos, verts, int32(nv), edged, edget) {
		// Point is outside the polygon, dtClamp to nearest edge.
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
		if dtClosestHeightPointTriangle(closest, v[0], v[1], v[2], &h) {
			closest[1] = h
			break
		}
	}
}

/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
func (m *DtNavMesh) TileAndPolyByRefUnsafe(ref DtPolyRef, tile **DtMeshTile, poly **DtPoly) {
	var salt, it, ip uint32
	m.decodePolyId(ref, &salt, &it, &ip)
	*tile = &m.Tiles[it]
	*poly = &m.Tiles[it].Polys[ip]
}

// Decodes a standard polygon reference.
//  ref  [in]   The polygon reference to decode.
//  salt [out]  The tile's salt value.
//  it	  [out]  The index of the tile.
//  ip	  [out]  The index of the polygon within the tile.
//  see encodePolyId
func (m *DtNavMesh) decodePolyId(ref DtPolyRef, salt, it, ip *uint32) {
	saltMask := (DtPolyRef(1) << m.saltBits) - 1
	tileMask := (DtPolyRef(1) << m.tileBits) - 1
	polyMask := (DtPolyRef(1) << m.polyBits) - 1

	*salt = uint32((ref >> (m.polyBits + m.tileBits)) & saltMask)
	*it = uint32((ref >> m.polyBits) & tileMask)
	*ip = uint32(ref & polyMask)
}

func (m *DtNavMesh) connectExtOffMeshLinks(tile, target *DtMeshTile, side int32) {
	if tile == nil {
		return
	}

	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	var oppositeSide uint8
	if side == -1 {
		oppositeSide = 0xff
	} else {
		oppositeSide = uint8(dtOppositeTile(side))
	}

	var i int32
	for i = 0; i < target.Header.OffMeshConCount; i++ {
		targetCon := &target.OffMeshCons[i]
		if targetCon.Side != oppositeSide {
			continue
		}

		panic("here7")
		targetPoly := &target.Polys[targetCon.Poly]
		// Skip off-mesh connections which start location could not be connected at all.
		if targetPoly.FirstLink == DT_NULL_LINK {
			continue
		}

		panic("here8")
		ext := []float32{targetCon.Rad, target.Header.WalkableClimb, targetCon.Rad}

		// Find polygon to connect to.
		p := targetCon.PosB
		nearestPt := d3.NewVec3()
		ref := m.FindNearestPolyInTile(tile, p, ext, nearestPt)
		if ref == 0 {
			continue
		}

		panic("here9")
		// findNearestPoly may return too optimistic results, further check to make sure.
		if math32.Sqr(nearestPt[0]-p[0])+math32.Sqr(nearestPt[2]-p[2]) > math32.Sqr(targetCon.Rad) {
			continue
		}

		panic("here10")
		// Make sure the location is on current mesh.
		var v d3.Vec3
		vidx := targetPoly.Verts[1] * 3
		v = target.Verts[vidx : vidx+3]
		v.Assign(nearestPt)

		// Link off-mesh connection to target poly.
		idx := allocLink(target)
		if idx != DT_NULL_LINK {
			link := &target.Links[idx]
			link.Ref = ref
			link.Edge = uint8(1)
			link.Side = oppositeSide
			link.Bmin = 0
			link.Bmax = 0
			// Add to linked list.
			link.Next = targetPoly.FirstLink
			targetPoly.FirstLink = idx
		}

		// Link target poly to off-mesh connection.
		if (uint32(targetCon.Flags) & DT_OFFMESH_CON_BIDIR) != 0 {
			tidx := allocLink(tile)
			if tidx != DT_NULL_LINK {
				landPolyIdx := uint16(m.decodePolyIdPoly(ref))
				landPoly := &tile.Polys[landPolyIdx]
				link := &tile.Links[tidx]
				link.Ref = m.getPolyRefBase(target) | DtPolyRef(targetCon.Poly)
				link.Edge = 0xff
				if side == -1 {
					link.Side = 0xff
				} else {
					link.Side = uint8(side)
				}
				link.Bmin = 0
				link.Bmax = 0
				// Add to linked list.
				link.Next = landPoly.FirstLink
				landPoly.FirstLink = tidx
			}
		}
	}
}

// This function will not fail if the tiles array is too small to hold the
// entire result set. It will simply fill the array to capacity.
func (m *DtNavMesh) TilesAt(x, y int32, tiles []*DtMeshTile, maxTiles int32) int32 {
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

func (m *DtNavMesh) connectExtLinks(tile, target *DtMeshTile, side int32) {
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
			if (poly.Neis[j] & DT_EXT_LINK) == 0 {
				continue
			}

			dir := (int32)(poly.Neis[j] & 0xff)
			if side != -1 && dir != side {
				continue
			}

			// Create new links
			idx := poly.Verts[j] * 3
			va := tile.Verts[idx : idx+3]
			idx = poly.Verts[(j+1)%int32(nv)] * 3

			vb := tile.Verts[idx : idx+3]
			nei := make([]DtPolyRef, 4)
			neia := make([]float32, 4*2)
			nnei := m.FindConnectingPolys(va, vb, target, dtOppositeTile(dir), nei, neia, 4)
			var k int32
			for k = 0; k < nnei; k++ {
				idx := allocLink(tile)
				if idx != DT_NULL_LINK {
					link := tile.Links[idx]
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
						link.Bmin = uint8(f32.Clamp(tmin, 0.0, 1.0) * 255.0)
						link.Bmax = uint8(f32.Clamp(tmax, 0.0, 1.0) * 255.0)
					} else if dir == 2 || dir == 6 {
						tmin := (neia[k*2+0] - va[0]) / (vb[0] - va[0])
						tmax := (neia[k*2+1] - va[0]) / (vb[0] - va[0])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.Bmin = uint8(f32.Clamp(tmin, 0.0, 1.0) * 255.0)
						link.Bmax = uint8(f32.Clamp(tmax, 0.0, 1.0) * 255.0)
					}
				}
			}
		}
	}
}

func (m *DtNavMesh) FindConnectingPolys(va, vb []float32, tile *DtMeshTile, side int32, con []DtPolyRef, conarea []float32, maxcon int32) int32 {
	if tile == nil {
		return 0
	}

	amin := make([]float32, 2)
	amax := make([]float32, 2)
	calcSlabEndPoints(va, vb, amin, amax, side)
	apos := getSlabCoord(va, side)

	// Remove links pointing to 'side' and compact the links array.
	bmin := make([]float32, 2)
	bmax := make([]float32, 2)
	m_ := DT_EXT_LINK | uint16(side)
	var n int32

	base := m.getPolyRefBase(tile)

	var i int32
	for i = 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		nv := poly.VertCount
		var j uint8
		for j = 0; j < nv; j++ {
			// Skip edges which do not point to the right side.
			if poly.Neis[j] != m_ {
				continue
			}

			idx := poly.Verts[j] * 3
			vc := tile.Verts[idx : idx+3]
			idx = poly.Verts[(j+1)%nv] * 3
			vd := tile.Verts[idx : idx+3]
			bpos := getSlabCoord(vc, side)

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
				con[n] = base | DtPolyRef(i)
				n++
			}
			break
		}
	}
	return n
}

func calcSlabEndPoints(va, vb []float32, bmin, bmax []float32, side int32) {
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

func getSlabCoord(va []float32, side int32) float32 {
	if side == 0 || side == 4 {
		return va[0]
	} else if side == 2 || side == 6 {
		return va[2]
	}
	return 0
}

func overlapSlabs(amin, amax, bmin, bmax []float32, px, py float32) bool {
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

func (m *DtNavMesh) NeighbourTilesAt(x, y, side int32, tiles []*DtMeshTile, maxTiles int32) int32 {
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

func (m *DtNavMesh) TileRefAt(x, y, layer int32) dtTileRef {
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

func (m *DtNavMesh) TileRef(tile *DtMeshTile) dtTileRef {
	if tile == nil {
		return 0
	}

	//const unsigned int it = (unsigned int)(tile - m_tiles);
	log.Fatal("use of unsafe in GetTileRef")
	it := uint32(uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.Tiles)))
	return dtTileRef(m.encodePolyId(tile.Salt, it, 0))
}

func (m *DtNavMesh) IsValidPolyRef(ref DtPolyRef) bool {
	if ref == 0 {
		return false
	}
	var salt, it, ip uint32
	m.decodePolyId(ref, &salt, &it, &ip)
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

// Returns the tile and polygon for the specified polygon reference.
//  @param[in]         ref             A known valid reference for a polygon.
//  @param[out]        tile    The tile containing the polygon.
//  @param[out]        poly    The polygon.
func (m *DtNavMesh) TileAndPolyByRef(ref DtPolyRef, tile **DtMeshTile, poly **DtPoly) DtStatus {
	if ref == 0 {
		return DT_FAILURE
	}
	var salt, it, ip uint32
	m.decodePolyId(ref, &salt, &it, &ip)
	if it >= uint32(m.MaxTiles) {
		return DT_FAILURE | DT_INVALID_PARAM
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return DT_FAILURE | DT_INVALID_PARAM
	}
	if ip >= uint32(m.Tiles[it].Header.PolyCount) {
		return DT_FAILURE | DT_INVALID_PARAM
	}
	*tile = &m.Tiles[it]
	*poly = &m.Tiles[it].Polys[ip]
	return DT_SUCCESS
}

// Calculates the tile grid location for the specified world position.
//  @param[in]	pos  The world position for the query. [(x, y, z)]
//  @param[out]	tx		The tile's x-location. (x, y)
//  @param[out]	ty		The tile's y-location. (x, y)
func (m *DtNavMesh) CalcTileLoc(pos d3.Vec3) (tx, ty int32) {
	tx = int32(math32.Floor((pos[0] - m.Orig[0]) / m.TileWidth))
	ty = int32(math32.Floor((pos[2] - m.Orig[2]) / m.TileHeight))
	return tx, ty
}
