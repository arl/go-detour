package detour

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"unsafe"
)

/// A navigation mesh based on tiles of convex polygons.
type DtNavMesh struct {
	m_params                  dtNavMeshParams ///< Current initialization params. TODO: do not store this info twice.
	m_orig                    [3]float32      ///< Origin of the tile (0,0)
	m_tileWidth, m_tileHeight float32         ///< Dimensions of each tile.
	m_maxTiles                int32           ///< Max number of tiles.
	m_tileLutSize             int32           ///< Tile hash lookup size (must be pot).
	m_tileLutMask             int32           ///< Tile hash lookup mask.

	//m_posLookup **dtMeshTile ///< Tile hash lookup.
	m_posLookup []*dtMeshTile ///< Tile hash lookup.
	m_nextFree  *dtMeshTile   ///< Freelist of tiles.
	m_tiles     []dtMeshTile  ///< List of tiles.

	//#ifndef DT_POLYREF64
	m_saltBits uint32 ///< Number of salt bits in the tile ID.
	m_tileBits uint32 ///< Number of tile bits in the tile ID.
	m_polyBits uint32 ///< Number of poly bits in the tile ID.
	//#endif
}

func (m *DtNavMesh) init(params *dtNavMeshParams) dtStatus {
	m.m_params = *params
	m.m_orig = params.Orig
	m.m_tileWidth = params.TileWidth
	m.m_tileHeight = params.TileHeight

	// Init tiles
	m.m_maxTiles = int32(params.MaxTiles)
	m.m_tileLutSize = int32(dtNextPow2(uint32(params.MaxTiles / 4)))
	if !(m.m_tileLutSize == 0) {
		m.m_tileLutSize = 1
	}
	m.m_tileLutMask = m.m_tileLutSize - 1

	//m.m_tiles = (dtMeshTile*)dtAlloc(sizeof(dtMeshTile)*m_maxTiles, DT_ALLOC_PERM);
	m.m_tiles = make([]dtMeshTile, m.m_maxTiles, m.m_maxTiles)
	//if (!m_tiles)
	//return DT_FAILURE | DT_OUT_OF_MEMORY;
	m.m_posLookup = make([]*dtMeshTile, m.m_tileLutSize, m.m_tileLutSize)
	//if (!m_posLookup)
	//return DT_FAILURE | DT_OUT_OF_MEMORY;
	//memset(m_tiles, 0, sizeof(dtMeshTile)*m_maxTiles);
	//memset(m_posLookup, 0, sizeof(dtMeshTile*)*m_tileLutSize);
	m.m_nextFree = nil
	for i := m.m_maxTiles - 1; i >= 0; i-- {
		m.m_tiles[i].Salt = 1
		m.m_tiles[i].Next = m.m_nextFree
		m.m_nextFree = &m.m_tiles[i]
	}

	// Init ID generator values.
	//#ifndef DT_POLYREF64
	m.m_tileBits = dtIlog2(dtNextPow2(uint32(params.MaxTiles)))
	m.m_polyBits = dtIlog2(dtNextPow2(uint32(params.MaxPolys)))
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	if 31 < 32-m.m_tileBits-m.m_polyBits {
		m.m_saltBits = 31
	} else {
		m.m_saltBits = 32 - m.m_tileBits - m.m_polyBits
	}

	if m.m_saltBits < 10 {
		return dtStatus(DT_FAILURE | DT_INVALID_PARAM)
	}
	//#endif

	return DT_SUCCESS
}

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #dtPolyRef's for the
/// tile will be restored to the same values they were before the tile was
/// removed.
///
/// The nav mesh assumes exclusive access to the data passed and will make
/// changes to the dynamic portion of the data. For that reason the data
/// should not be reused in other nav meshes until the tile has been successfully
/// removed from this nav mesh.
///
/// @see dtCreateNavMeshData, #removeTile
func (m *DtNavMesh) addTile(data []byte, dataSize int32, lastRef dtTileRef, result *dtTileRef) dtStatus {

	var hdr dtMeshHeader
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
	if m.getTileAt(hdr.X, hdr.Y, hdr.Layer) != nil {
		return DT_FAILURE
	}

	// Allocate a tile.
	var tile *dtMeshTile
	if lastRef == 0 {
		if m.m_nextFree != nil {
			tile = m.m_nextFree
			m.m_nextFree = tile.Next
			tile.Next = nil
		}
	} else {
		// Try to relocate the tile to specific index with same salt.
		tileIndex := int32(m.decodePolyIdTile(dtPolyRef(lastRef)))
		if tileIndex >= m.m_maxTiles {
			log.Fatalln("tileIndex >= m.m_maxTiles", tileIndex, m.m_maxTiles)
			return DT_FAILURE | DT_OUT_OF_MEMORY
		}
		// Try to find the specific tile id from the free list.
		target := &m.m_tiles[tileIndex]
		var prev *dtMeshTile
		tile = m.m_nextFree
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
			m.m_nextFree = tile.Next
		} else {
			prev.Next = tile.Next
		}

		// Restore salt.
		tile.Salt = m.decodePolyIdSalt(dtPolyRef(lastRef))
	}

	// Make sure we could allocate a tile.
	if tile == nil {
		log.Fatalln("couldn't allocate tile")
		return DT_FAILURE | DT_OUT_OF_MEMORY
	}

	// Insert tile into the position lut.
	h := computeTileHash(hdr.X, hdr.Y, m.m_tileLutMask)
	tile.Next = m.m_posLookup[h]
	m.m_posLookup[h] = tile

	// Patch header pointers.
	headerSize := dtAlign4(uint32(unsafe.Sizeof(dtMeshHeader{})))
	vertsSize := dtAlign4(uint32(unsafe.Sizeof(float32(0))) * 3 * uint32(hdr.VertCount))
	polysSize := dtAlign4(uint32(unsafe.Sizeof(dtPoly{})) * uint32(hdr.PolyCount))
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
	//tile->verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile.Verts = make([]float32, 3*hdr.VertCount)
	binary.Read(r, binary.LittleEndian, &tile.Verts)
	fmt.Println("verts", tile.Verts)

	//tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	tile.Polys = make([]dtPoly, hdr.PolyCount)
	binary.Read(r, binary.LittleEndian, &tile.Polys)
	fmt.Println("polys", tile.Polys)

	//tile->links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	tile.Links = make([]dtLink, hdr.MaxLinkCount)
	binary.Read(r, binary.LittleEndian, &tile.Links)
	fmt.Println("links", tile.Links)

	//tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	tile.DetailMeshes = make([]dtPolyDetail, hdr.DetailMeshCount)
	binary.Read(r, binary.LittleEndian, &tile.DetailMeshes)
	fmt.Println("detailMeshes", tile.DetailMeshes)

	//tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile.DetailVerts = make([]float32, 3*hdr.DetailVertCount)
	binary.Read(r, binary.LittleEndian, &tile.DetailVerts)
	fmt.Println("detailVerts", tile.DetailVerts)

	//tile->detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	tile.DetailTris = make([]uint8, 4*hdr.DetailTriCount)
	binary.Read(r, binary.LittleEndian, &tile.DetailTris)
	fmt.Println("detailTris", tile.DetailTris)

	//tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
	tile.BvTree = make([]dtBVNode, hdr.BvNodeCount)
	binary.Read(r, binary.LittleEndian, &tile.BvTree)
	fmt.Println("bvTree", tile.BvTree)

	//tile->offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);
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

	//connectIntLinks(tile);

	//// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	//baseOffMeshLinks(tile);
	//connectExtOffMeshLinks(tile, tile, -1);

	//// Create connections with neighbour tiles.
	//static const int MAX_NEIS = 32;
	//dtMeshTile* neis[MAX_NEIS];
	//int nneis;

	//// Connect with layers in current tile.
	//nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
	//for (int j = 0; j < nneis; ++j)
	//{
	//if (neis[j] == tile)
	//continue;

	//connectExtLinks(tile, neis[j], -1);
	//connectExtLinks(neis[j], tile, -1);
	//connectExtOffMeshLinks(tile, neis[j], -1);
	//connectExtOffMeshLinks(neis[j], tile, -1);
	//}

	//// Connect with neighbour tiles.
	//for (int i = 0; i < 8; ++i)
	//{
	//nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
	//for (int j = 0; j < nneis; ++j)
	//{
	//connectExtLinks(tile, neis[j], i);
	//connectExtLinks(neis[j], tile, dtOppositeTile(i));
	//connectExtOffMeshLinks(tile, neis[j], i);
	//connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
	//}
	//}

	//if (result)
	//*result = getTileRef(tile);

	return DT_SUCCESS
}

func (m *DtNavMesh) getTileAt(x, y, layer int32) *dtMeshTile {
	// Find tile based on hash.
	h := computeTileHash(x, y, m.m_tileLutMask)

	//dtMeshTile* tile = m.m_posLookup[h];
	tile := m.m_posLookup[h]
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

func (m *DtNavMesh) connectIntLinks(tile *dtMeshTile) {
	if tile == nil {
		return
	}

	base := m.getPolyRefBase(tile)

	//CONTINUER ICI
	//CONTINUER ICI
	//CONTINUER ICI
	//CONTINUER ICI

	//for (int i = 0; i < tile->header->polyCount; ++i) {
	//dtPoly* poly = &tile->polys[i];
	//poly->firstLink = DT_NULL_LINK;

	//if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	//continue;

	//// Build edge links backwards so that the links will be
	//// in the linked list from lowest index to highest.
	//for (int j = poly->vertCount-1; j >= 0; --j)
	//{
	//// Skip hard and non-internal edges.
	//if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK)) continue;

	//unsigned int idx = allocLink(tile);
	//if (idx != DT_NULL_LINK)
	//{
	//dtLink* link = &tile->links[idx];
	//link->ref = base | (dtPolyRef)(poly->neis[j]-1);
	//link->edge = (unsigned char)j;
	//link->side = 0xff;
	//link->bmin = link->bmax = 0;
	//// Add to linked list.
	//link->next = poly->firstLink;
	//poly->firstLink = idx;
	//}
	//}
	//}
}

/// @par
///
/// Example use case:
/// @code
///
/// const dtPolyRef base = navmesh->getPolyRefBase(tile);
/// for (int i = 0; i < tile->header->polyCount; ++i)
/// {
///     const dtPoly* p = &tile->polys[i];
///     const dtPolyRef ref = base | (dtPolyRef)i;
///
///     // Use the reference to access the polygon data.
/// }
/// @endcode
func (m DtNavMesh) getPolyRefBase(tile *dtMeshTile) dtPolyRef {
	if tile == nil {
		return 0
	}
	//it := uint32(tile - m.m_tiles)
	it := uint32(uintptr(unsafe.Pointer(tile)) - uintptr(unsafe.Pointer(&m.m_tiles)))
	return m.encodePolyId(tile.Salt, it, 0)
}

func computeTileHash(x, y, mask int32) int32 {
	h1 := 0x8da6b343 // Large multiplicative constants;
	h2 := 0xd8163841 // here arbitrarily chosen primes
	n := h1*int(x) + h2*int(y)
	return int32(n) & mask
}

/// @{
/// @name Encoding and Decoding
/// These functions are generally meant for internal use only.

/// Derives a standard polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	salt	The tile's salt value.
///  @param[in]	it		The index of the tile.
///  @param[in]	ip		The index of the polygon within the tile.
func (m DtNavMesh) encodePolyId(salt, it, ip uint32) dtPolyRef {
	//#ifdef DT_POLYREF64
	//return ((dtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
	//#else
	return (dtPolyRef(salt) << (m.m_polyBits + m.m_tileBits)) | (dtPolyRef(it) << m.m_polyBits) | dtPolyRef(ip)
	//#endif
}

const (
	DT_SALT_BITS uint32 = 16
	DT_TILE_BITS uint32 = 28
	DT_POLY_BITS uint32 = 20
)

type dtPolyRef uint32

/// Defines a link between polygons.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
type dtLink struct {
	Ref  dtPolyRef ///< Neighbour reference. (The neighbor that is linked to.)
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
	I    int       ///< The node's index. (Negative for escape sequence.)
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

/// Extracts the tile's index from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	ref		The polygon reference.
///  @see #encodePolyId
func (m DtNavMesh) decodePolyIdTile(ref dtPolyRef) uint32 {
	//#ifdef DT_POLYREF64
	//const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
	//return (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//#else
	tileMask := dtPolyRef((dtPolyRef(1) << m.m_tileBits) - 1)
	return uint32((ref >> m.m_polyBits) & tileMask)
	//#endif
}

/// Extracts a tile's salt value from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]	ref		The polygon reference.
///  @see #encodePolyId
func (m DtNavMesh) decodePolyIdSalt(ref dtPolyRef) uint32 {
	//#ifdef DT_POLYREF64
	//const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
	//return (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//#else
	saltMask := (dtPolyRef(1) << m.m_saltBits) - 1
	return uint32((ref >> (m.m_polyBits + m.m_tileBits)) & saltMask)
	//#endif
}
