package detour

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"unsafe"
)

/// A navigation mesh based on tiles of convex polygons.
type DtNavMesh struct {
	Params                DtNavMeshParams ///< Current initialization params. TODO: do not store this info twice.
	Orig                  [3]float32      ///< Origin of the tile (0,0)
	TileWidth, TileHeight float32         ///< Dimensions of each tile.
	MaxTiles              int32           ///< Max number of tiles.
	TileLUTSize           int32           ///< Tile hash lookup size (must be pot).
	TileLUTMask           int32           ///< Tile hash lookup mask.

	//m_posLookup **dtMeshTile ///< Tile hash lookup.
	posLookup []*DtMeshTile ///< Tile hash lookup.
	nextFree  *DtMeshTile   ///< Freelist of tiles.
	Tiles     []DtMeshTile  ///< List of tiles.

	saltBits uint32 ///< Number of salt bits in the tile ID.
	tileBits uint32 ///< Number of tile bits in the tile ID.
	polyBits uint32 ///< Number of poly bits in the tile ID.
}

func (m *DtNavMesh) init(params *DtNavMeshParams) DtStatus {
	m.Params = *params
	m.Orig = params.Orig
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
	//if (!m_tiles)
	//return DT_FAILURE | DT_OUT_OF_MEMORY;
	m.posLookup = make([]*DtMeshTile, m.TileLUTSize, m.TileLUTSize)
	//if (!m_posLookup)
	//return DT_FAILURE | DT_OUT_OF_MEMORY;
	//memset(m_tiles, 0, sizeof(dtMeshTile)*m_maxTiles);
	//memset(m_posLookup, 0, sizeof(dtMeshTile*)*m_tileLutSize);
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

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #DtPolyRef's for the
/// tile will be restored to the same values they were before the tile was
/// removed.
///
/// The nav mesh assumes exclusive access to the data passed and will make
/// changes to the dynamic portion of the data. For that reason the data
/// should not be reused in other nav meshes until the tile has been successfully
/// removed from this nav mesh.
///
/// @see dtCreateNavMeshData, #removeTile
func (m *DtNavMesh) addTile(data []byte, dataSize int32, lastRef dtTileRef, result *dtTileRef) DtStatus {

	var hdr DtMeshHeader
	// prepare a reader on the received data
	r := bytes.NewReader(data)
	binary.Read(r, binary.LittleEndian, &hdr)

	// Make sure the data is in right format.
	if hdr.Magic != DT_NAVMESH_MAGIC {
		return DT_FAILURE | DT_WRONG_MAGIC
	}
	if hdr.Version != DT_NAVMESH_VERSION {
		return DT_FAILURE | DT_WRONG_VERSION
	}

	// Make sure the location is free.
	if m.GetTileAt(hdr.X, hdr.Y, hdr.Layer) != nil {
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

	// Patch header pointers.
	headerSize := dtAlign4(uint32(unsafe.Sizeof(DtMeshHeader{})))
	vertsSize := dtAlign4(uint32(unsafe.Sizeof(float32(0))) * 3 * uint32(hdr.VertCount))
	polysSize := dtAlign4(uint32(unsafe.Sizeof(DtPoly{})) * uint32(hdr.PolyCount))
	linksSize := dtAlign4(uint32(unsafe.Sizeof(dtLink{})) * uint32(hdr.MaxLinkCount))
	detailMeshesSize := dtAlign4(uint32(unsafe.Sizeof(dtPolyDetail{})) * uint32(hdr.DetailMeshCount))
	detailVertsSize := dtAlign4(uint32(unsafe.Sizeof(float32(0))) * 3 * uint32(hdr.DetailVertCount))
	detailTrisSize := dtAlign4(uint32(unsafe.Sizeof(uint8(0))) * 4 * uint32(hdr.DetailTriCount))
	bvtreeSize := dtAlign4(uint32(unsafe.Sizeof(dtBVNode{})) * uint32(hdr.BvNodeCount))
	offMeshLinksSize := dtAlign4(uint32(unsafe.Sizeof(dtOffMeshConnection{})) * uint32(hdr.OffMeshConCount))

	log.Println("headerSize", headerSize)
	log.Println("vertsSize", vertsSize)
	log.Println("polysSize", polysSize)
	log.Println("linksSize", linksSize)
	log.Println("detailMeshesSize", detailMeshesSize)
	log.Println("detailVertsSize", detailVertsSize)
	log.Println("detailTrisSize", detailTrisSize)
	log.Println("bvtreeSize", bvtreeSize)
	log.Println("offMeshLinksSize", offMeshLinksSize)

	//unsigned char* d = data + headerSize;
	//tile.verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile.Verts = make([]float32, 3*hdr.VertCount)
	binary.Read(r, binary.LittleEndian, &tile.Verts)
	fmt.Println("verts", tile.Verts)

	//tile.polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	tile.Polys = make([]DtPoly, hdr.PolyCount)
	binary.Read(r, binary.LittleEndian, &tile.Polys)
	fmt.Println("polys", tile.Polys)

	//tile.links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	tile.Links = make([]dtLink, hdr.MaxLinkCount)
	binary.Read(r, binary.LittleEndian, &tile.Links)
	fmt.Println("links", tile.Links)

	//tile.detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	tile.DetailMeshes = make([]dtPolyDetail, hdr.DetailMeshCount)
	binary.Read(r, binary.LittleEndian, &tile.DetailMeshes)
	fmt.Println("detailMeshes", tile.DetailMeshes)

	//tile.detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile.DetailVerts = make([]float32, 3*hdr.DetailVertCount)
	binary.Read(r, binary.LittleEndian, &tile.DetailVerts)
	fmt.Println("detailVerts", tile.DetailVerts)

	//tile.detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	tile.DetailTris = make([]uint8, 4*hdr.DetailTriCount)
	binary.Read(r, binary.LittleEndian, &tile.DetailTris)
	fmt.Println("detailTris", tile.DetailTris)

	//tile.bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
	tile.BvTree = make([]dtBVNode, hdr.BvNodeCount)
	binary.Read(r, binary.LittleEndian, &tile.BvTree)
	fmt.Println("bvTree", tile.BvTree)

	//tile.offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);
	tile.OffMeshCons = make([]dtOffMeshConnection, hdr.OffMeshConCount)
	binary.Read(r, binary.LittleEndian, &tile.OffMeshCons)
	fmt.Println("offMeshCons", tile.OffMeshCons)

	// If there are no items in the bvtree, reset the tree pointer.
	if bvtreeSize == 0 {
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
	nneis = m.GetTilesAt(hdr.X, hdr.Y, neis, MAX_NEIS)
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
		nneis = m.GetNeighbourTilesAt(hdr.X, hdr.Y, i, neis, MAX_NEIS)
		for j = 0; j < nneis; j++ {
			//log.Println("connecting with neighbour tiles", neis[j])
			m.connectExtLinks(tile, neis[j], i)
			m.connectExtLinks(neis[j], tile, dtOppositeTile(i))
			m.connectExtOffMeshLinks(tile, neis[j], i)
			m.connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i))
		}
	}

	if result != nil {
		*result = m.GetTileRef(tile)
	}

	return DT_SUCCESS
}

func (m *DtNavMesh) GetTileAt(x, y, layer int32) *DtMeshTile {
	// Find tile based on hash.
	h := computeTileHash(x, y, m.TileLUTMask)

	//dtMeshTile* tile = m.m_posLookup[h];
	tile := m.posLookup[h]
	for tile != nil {
		if tile.Header != nil &&
			tile.Header.X == x &&
			tile.Header.Y == y &&
			tile.Header.Layer == layer {
			fmt.Println("return", tile)
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

	base := m.getPolyRefBase(tile)

	var i int32
	for i = 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		poly.FirstLink = DT_NULL_LINK

		if poly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
			continue
		}

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		var j int32
		for j = int32(poly.VertCount - 1); j >= 0; j-- {
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

/// @par
///
/// Example use case:
/// @code
///
/// const DtPolyRef base = navmesh.getPolyRefBase(tile);
/// for (int i = 0; i < tile.header.polyCount; ++i)
/// {
///     const dtPoly* p = &tile.polys[i];
///     const DtPolyRef ref = base | (DtPolyRef)i;
///
///     // Use the reference to access the polygon data.
/// }
/// @endcode
func (m *DtNavMesh) getPolyRefBase(tile *DtMeshTile) DtPolyRef {
	if tile == nil {
		return 0
	}
	//it := uint32(tile - m.m_tiles)
	//it := uint32(uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.m_tiles[0])))

	e := uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.Tiles[0]))
	ip := uint32(e / unsafe.Sizeof(*tile))

	//if it > uint32(len(m.m_tiles)) {
	//fmt.Println("e", e, "ip", ip)
	//log.Fatalln("houston...", it, ">", len(m.m_tiles))
	//}

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

/// @{
/// @name Encoding and Decoding
/// These functions are generally meant for internal use only.

/// Derives a standard polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	salt	The tile's salt value.
///  @param[in]	it		The index of the tile.
///  @param[in]	ip		The index of the polygon within the tile.
func (m *DtNavMesh) encodePolyId(salt, it, ip uint32) DtPolyRef {
	//#ifdef DT_POLYREF64
	//return ((DtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((DtPolyRef)it << DT_POLY_BITS) | (DtPolyRef)ip;
	//#else
	return (DtPolyRef(salt) << (m.polyBits + m.tileBits)) | (DtPolyRef(it) << m.polyBits) | DtPolyRef(ip)
	//#endif
}

const (
	DT_SALT_BITS uint32 = 16
	DT_TILE_BITS uint32 = 28
	DT_POLY_BITS uint32 = 20
)

type DtPolyRef uint32

/// Defines a link between polygons.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
type dtLink struct {
	Ref  DtPolyRef ///< Neighbour reference. (The neighbor that is linked to.)
	Next uint32    ///< Index of the next link.
	Edge uint8     ///< Index of the polygon edge that owns this link.
	Side uint8     ///< If a boundary link, defines on which side the link is.
	Bmin uint8     ///< If a boundary link, defines the minimum sub-edge area.
	Bmax uint8     ///< If a boundary link, defines the maximum sub-edge area.
}

/// Defines the location of detail sub-mesh data within a dtMeshTile.
type dtPolyDetail struct {
	VertBase  uint32 ///< The offset of the vertices in the dtMeshTile::detailVerts array.
	TriBase   uint32 ///< The offset of the triangles in the dtMeshTile::detailTris array.
	VertCount uint8  ///< The number of vertices in the sub-mesh.
	TriCount  uint8  ///< The number of triangles in the sub-mesh.
}

/// Bounding volume node.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
type dtBVNode struct {
	Bmin [3]uint16 ///< Minimum bounds of the node's AABB. [(x, y, z)]
	Bmax [3]uint16 ///< Maximum bounds of the node's AABB. [(x, y, z)]
	I    int32     ///< The node's index. (Negative for escape sequence.)
}

/// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
/// An off-mesh connection is a user defined traversable connection made up to two vertices.
type dtOffMeshConnection struct {
	/// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	Pos [6]float32

	/// The radius of the endpoints. [Limit: >= 0]
	Rad float32

	/// The polygon reference of the connection within the tile.
	Poly uint16

	/// Link flags.
	/// @note These are not the connection's user defined flags. Those are assigned via the
	/// connection's dtPoly definition. These are link flags used for internal purposes.
	Flags uint8

	/// End point side.
	Side uint8

	/// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	UserId uint
}

/// A magic number used to detect compatibility of navigation tile data.
const (
	DT_NAVMESH_MAGIC int32 = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V'

	/// A version number used to detect compatibility of navigation tile data.
	DT_NAVMESH_VERSION = 7

	/// A magic number used to detect the compatibility of navigation tile states.
	DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S'

	/// A version number used to detect compatibility of navigation tile states.
	DT_NAVMESH_STATE_VERSION = 1
)

/// Flags representing the type of a navigation mesh polygon.
type dtPolyTypes uint32

const (
	/// The polygon is a standard convex polygon that is part of the surface of the mesh.
	DT_POLYTYPE_GROUND dtPolyTypes = 0
	/// The polygon is an off-mesh connection consisting of two vertices.
	DT_POLYTYPE_OFFMESH_CONNECTION = 1
)

const (
	/// A flag that indicates that an entity links to an external entity.
	/// (E.g. A polygon edge is a portal that links to another polygon.)
	DT_EXT_LINK uint16 = 0x8000

	/// A value that indicates the entity does not link to anything.
	DT_NULL_LINK uint32 = 0xffffffff
)

/// Extracts the tile's index from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	ref		The polygon reference.
///  @see #encodePolyId
func (m *DtNavMesh) decodePolyIdTile(ref DtPolyRef) uint32 {
	//#ifdef DT_POLYREF64
	//const DtPolyRef tileMask = ((DtPolyRef)1<<DT_TILE_BITS)-1;
	//return (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//#else
	tileMask := DtPolyRef((DtPolyRef(1) << m.tileBits) - 1)
	return uint32((ref >> m.polyBits) & tileMask)
	//#endif
}

/// Extracts a tile's salt value from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	ref		The polygon reference.
///  @see #encodePolyId
func (m *DtNavMesh) decodePolyIdSalt(ref DtPolyRef) uint32 {
	//#ifdef DT_POLYREF64
	//const DtPolyRef saltMask = ((DtPolyRef)1<<DT_SALT_BITS)-1;
	//return (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//#else
	saltMask := (DtPolyRef(1) << m.saltBits) - 1
	return uint32((ref >> (m.polyBits + m.tileBits)) & saltMask)
	//#endif
}

func (m *DtNavMesh) baseOffMeshLinks(tile *DtMeshTile) {
	if tile == nil {
		return
	}

	base := m.getPolyRefBase(tile)

	var (
		i    int32
		con  *dtOffMeshConnection
		poly *DtPoly
	)

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
		//p = &con.Pos[0] // First vertex
		p = con.Pos[0:3] // First vertex
		//p = dtVec{con.Pos[0], con.Pos[1], con.Pos[2]} // First vertex

		nearestPt := make([]float32, 3)
		ref := m.findNearestPolyInTile(tile, p, ext, nearestPt)
		if ref == 0 {
			continue
		}
		// findNearestPoly may return too optimistic results, further check to make sure.
		if dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(con.Rad) {
			continue
		}
		// Make sure the location is on current mesh.
		v := tile.Verts[poly.Verts[0]*3 : 3]
		dtVcopy(v, nearestPt)

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

func (m *DtNavMesh) findNearestPolyInTile(tile *DtMeshTile, center, extents, nearestPt []float32) DtPolyRef {
	bmin := make([]float32, 3)
	bmax := make([]float32, 3)
	dtVsub(bmin, center, extents)
	dtVadd(bmax, center, extents)

	// Get nearby polygons from proximity grid.
	var polys [128]DtPolyRef
	polyCount := m.queryPolygonsInTile(tile, bmin, bmax, polys[:], 128)

	// Find nearest polygon amongst the nearby polygons.
	var nearest DtPolyRef
	nearestDistanceSqr := float32(math.MaxFloat32)
	var i int32
	for i = 0; i < polyCount; i++ {
		ref := polys[i]
		var (
			posOverPoly bool
			d           float32
		)
		closestPtPoly := make([]float32, 3)
		diff := make([]float32, 3)
		m.ClosestPointOnPoly(ref, center, closestPtPoly, &posOverPoly)

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff, center, closestPtPoly)
		if posOverPoly {
			d = dtAbs(diff[1]) - tile.Header.WalkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = dtVlenSqr(diff)
		}

		if d < nearestDistanceSqr {
			dtVcopy(nearestPt, closestPtPoly)
			nearestDistanceSqr = d
			nearest = ref
		}
	}

	return nearest
}

// queries polygons within a tile
func (m *DtNavMesh) queryPolygonsInTile(tile *DtMeshTile, qmin, qmax []float32, polys []DtPolyRef, maxPolys int32) int32 {
	if tile.BvTree != nil {

		var (
			// AR: those won't be needed once nodeIdx and endIdx are correctcly
			// used
			node *dtBVNode
			//end  *dtBVNode

			nodeIdx int32
			endIdx  int32

			tbmin, tbmax [3]float32
			qfac         float32
		)
		//node = &tile.BvTree[0]
		//end = &tile.BvTree[tile.Header.BvNodeCount]
		nodeIdx = 0
		endIdx = tile.Header.BvNodeCount

		tbmin = tile.Header.Bmin
		tbmin = tile.Header.Bmax
		qfac = tile.Header.BvQuantFactor

		// Calculate quantized box
		var bmin, bmax [3]uint16
		// dtClamp query box to world box.
		minx := dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
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

		bmin := make([]float32, 3)
		bmax := make([]float32, 3)
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
			dtVcopy(bmin, v)
			dtVcopy(bmax, v)
			var j uint8
			for j = 1; j < p.VertCount; j++ {
				v = tile.Verts[p.Verts[j]*3 : 3]
				dtVmin(bmin, v)
				dtVmax(bmax, v)
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

/// Extracts the polygon's index (within its tile) from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	ref		The polygon reference.
///  @see #encodePolyId
func (m *DtNavMesh) decodePolyIdPoly(ref DtPolyRef) uint32 {
	//#ifdef DT_POLYREF64
	//const DtPolyRef polyMask = ((DtPolyRef)1<<DT_POLY_BITS)-1;
	//return (unsigned int)(ref & polyMask);
	//#else
	polyMask := DtPolyRef((1 << m.polyBits) - 1)
	return uint32(ref & polyMask)
	//#endif
}

/// Finds the closest point on the specified polygon.
///  @param[in]		ref			The reference id of the polygon.
///  @param[in]		pos			The position to check. [(x, y, z)]
///  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
///  @param[out]	posOverPoly	True of the position is over the polygon.
/// @returns The status flags for the query.
func (m *DtNavMesh) ClosestPointOnPoly(ref DtPolyRef, pos, closest []float32, posOverPoly *bool) {
	var (
		tile *DtMeshTile
		poly *DtPoly
	)

	m.GetTileAndPolyByRefUnsafe(ref, &tile, &poly)

	// Off-mesh connections don't have detail polygons.
	if poly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
		var (
			v0, v1    []float32
			d0, d1, u float32
		)
		v0 = tile.Verts[poly.Verts[0]*3 : 3]
		v1 = tile.Verts[poly.Verts[1]*3 : 3]
		d0 = dtVdist(pos, v0)
		d1 = dtVdist(pos, v1)
		u = d0 / (d0 + d1)
		dtVlerp(closest, v0, v1, u)
		if posOverPoly != nil {
			*posOverPoly = false
		}
		return
	}

	// CAREFUL with unsafe, double check the ip variable is bounded to
	// tile.Polys length
	e := uintptr(unsafe.Pointer(poly)) - uintptr(unsafe.Pointer(&tile.Polys[0]))
	ip := uint32(e / unsafe.Sizeof(*poly))

	if ip > uint32(len(tile.Polys)) {
		log.Fatalln("houston...", ip, ">", len(tile.Polys))
	} else {
		log.Fatalln("OK with", ip, "<", len(tile.Polys))
	}
	//unsafe.Pointer(poly)
	//hdr := (*reflect.SliceHeader)(unsafe.Pointer(&tile.Polys)) // case 1
	//hdr.Data = uintptr(unsafe.Pointer(p))              // case 6 (this case)
	//hdr.Len = uintptr(n)

	//ip := uint32(poly - tile.Polys)

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
		dtVcopy(verts[idx:idx+3], tile.Verts[jdx:jdx+3])
	}

	dtVcopy(closest, pos)
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
		va := verts[imin*3 : 3]
		vb := verts[((imin+1)%nv)*3 : 3]
		dtVlerp(closest, va, vb, edget[imin])

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
		t := tile.DetailTris[(pd.TriBase+uint32(j))*4 : 3]
		v := make([][]float32, 3)
		var k int
		for k = 0; k < 3; k++ {
			if t[k] < poly.VertCount {
				v[k] = tile.Verts[poly.Verts[t[k]]*3 : 3]
			} else {
				v[k] = tile.DetailVerts[(pd.VertBase+uint32(t[k]-poly.VertCount))*3 : 3]
			}
		}
		var h float32
		if dtClosestHeightPointTriangle(closest, v[0], v[1], v[2], &h) {
			closest[1] = h
			break
		}
	}
}

/// @par
///
/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
func (m *DtNavMesh) GetTileAndPolyByRefUnsafe(ref DtPolyRef, tile **DtMeshTile, poly **DtPoly) {
	var salt, it, ip uint32
	m.DecodePolyId(ref, &salt, &it, &ip)
	*tile = &m.Tiles[it]
	*poly = &m.Tiles[it].Polys[ip]
}

/// Decodes a standard polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	ref   The polygon reference to decode.
///  @param[out]	salt	The tile's salt value.
///  @param[out]	it		The index of the tile.
///  @param[out]	ip		The index of the polygon within the tile.
///  @see #encodePolyId
func (m *DtNavMesh) DecodePolyId(ref DtPolyRef, salt, it, ip *uint32) {
	//#ifdef DT_POLYREF64
	//const DtPolyRef saltMask = ((DtPolyRef)1<<DT_SALT_BITS)-1;
	//const DtPolyRef tileMask = ((DtPolyRef)1<<DT_TILE_BITS)-1;
	//const DtPolyRef polyMask = ((DtPolyRef)1<<DT_POLY_BITS)-1;
	//salt = (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//it = (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//ip = (unsigned int)(ref & polyMask);
	//#else
	saltMask := (DtPolyRef(1) << m.saltBits) - 1
	tileMask := (DtPolyRef(1) << m.tileBits) - 1
	polyMask := (DtPolyRef(1) << m.polyBits) - 1

	*salt = uint32((ref >> (m.polyBits + m.tileBits)) & saltMask)
	*it = uint32((ref >> m.polyBits) & tileMask)
	*ip = uint32(ref & polyMask)
	//#endif
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
		p := targetCon.Pos[3:6]
		nearestPt := make([]float32, 3)
		ref := m.findNearestPolyInTile(tile, p, ext, nearestPt)
		if ref == 0 {
			continue
		}

		panic("here9")
		// findNearestPoly may return too optimistic results, further check to make sure.
		if dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(targetCon.Rad) {
			continue
		}

		panic("here10")
		// Make sure the location is on current mesh.
		v := target.Verts[targetPoly.Verts[1]*3 : 3]
		dtVcopy(v, nearestPt)

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

/// @par
///
/// This function will not fail if the tiles array is too small to hold the
/// entire result set.  It will simply fill the array to capacity.
func (m *DtNavMesh) GetTilesAt(x, y int32, tiles []*DtMeshTile, maxTiles int32) int32 {
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
		log.Println("connecting border links", poly)

		// Create new links.
		//		unsigned short m = DT_EXT_LINK | (unsigned short)side;

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
			nnei := m.findConnectingPolys(va, vb, target, dtOppositeTile(dir), nei, neia, 4)
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
							dtSwap(&tmin, &tmax)
						}
						link.Bmin = uint8(dtClamp(tmin, 0.0, 1.0) * 255.0)
						link.Bmax = uint8(dtClamp(tmax, 0.0, 1.0) * 255.0)
					} else if dir == 2 || dir == 6 {
						tmin := (neia[k*2+0] - va[0]) / (vb[0] - va[0])
						tmax := (neia[k*2+1] - va[0]) / (vb[0] - va[0])
						if tmin > tmax {
							dtSwap(&tmin, &tmax)
						}
						link.Bmin = uint8(dtClamp(tmin, 0.0, 1.0) * 255.0)
						link.Bmax = uint8(dtClamp(tmax, 0.0, 1.0) * 255.0)
					}
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
func (m *DtNavMesh) findConnectingPolys(va, vb []float32, tile *DtMeshTile, side int32, con []DtPolyRef, conarea []float32, maxcon int32) int32 {
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
			if dtAbs(apos-bpos) > 0.01 {
				continue
			}

			// Check if the segments touch.
			calcSlabEndPoints(vc, vd, bmin, bmax, side)

			if !overlapSlabs(amin, amax, bmin, bmax, 0.01, tile.Header.WalkableClimb) {
				continue
			}

			// Add return value.
			if n < maxcon {
				conarea[n*2+0] = dtMax(amin[0], bmin[0])
				conarea[n*2+1] = dtMin(amax[0], bmax[0])
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
	minx := dtMax(amin[0]+px, bmin[0]+px)
	maxx := dtMin(amax[0]-px, bmax[0]-px)
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
	thr := dtSqr(py * 2)
	if dmin*dmin <= thr || dmax*dmax <= thr {
		return true
	}

	return false
}

func (m *DtNavMesh) GetNeighbourTilesAt(x, y, side int32, tiles []*DtMeshTile, maxTiles int32) int32 {
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

	return m.GetTilesAt(nx, ny, tiles, maxTiles)
}

func (m *DtNavMesh) GetTileRefAt(x, y, layer int32) dtTileRef {
	// Find tile based on hash.
	h := computeTileHash(x, y, m.TileLUTMask)
	tile := m.posLookup[h]
	for tile != nil {
		if tile.Header != nil &&
			tile.Header.X == x &&
			tile.Header.Y == y &&
			tile.Header.Layer == layer {
			return m.GetTileRef(tile)
		}
		tile = tile.Next
	}
	return 0
}

func (m *DtNavMesh) GetTileRef(tile *DtMeshTile) dtTileRef {
	if tile == nil {
		return 0
	}

	//const unsigned int it = (unsigned int)(tile - m_tiles);
	it := uint32(uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.Tiles)))
	return dtTileRef(m.encodePolyId(tile.Salt, it, 0))
}

func (m *DtNavMesh) IsValidPolyRef(ref DtPolyRef) bool {
	if ref == 0 {
		return false
	}
	var salt, it, ip uint32
	m.DecodePolyId(ref, &salt, &it, &ip)
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

func (m *DtNavMesh) getTileAndPolyByRef(ref DtPolyRef, tile **DtMeshTile, poly **DtPoly) DtStatus {
	if ref == 0 {
		return DT_FAILURE
	}
	var salt, it, ip uint32
	m.DecodePolyId(ref, &salt, &it, &ip)
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

func (m *DtNavMesh) calcTileLoc(pos [3]float32, tx, ty *int32) {
	*tx = int32(math.Floor(float64((pos[0] - m.Orig[0]) / m.TileWidth)))
	*ty = int32(math.Floor(float64((pos[2] - m.Orig[2]) / m.TileHeight)))
}
