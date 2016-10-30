package detour

type NavMeshSetHeader struct {
	Magic    int32
	Version  int32
	NumTiles int32
	Params   dtNavMeshParams
}

//func (hdr *NavMeshSetHeader) decode(r io.Reader) error {
//err := binary.Read(r, binary.LittleEndian, &hdr.magic)
//if err != nil {
//return err
//}
//if hdr.magic != NAVMESHSET_MAGIC {
//return fmt.Errorf("wrong magic number: %x", hdr.magic)
//}

//err = binary.Read(r, binary.LittleEndian, &hdr.version)
//if err != nil {
//return err
//}
//if hdr.version != NAVMESHSET_VERSION {
//return fmt.Errorf("wrong version")
//}

//err = binary.Read(r, binary.LittleEndian, &hdr.numTiles)
//if err != nil {
//return err
//}

//return hdr.params.decode(r)
//}

/// Configuration parameters used to define multi-tile navigation meshes.
/// The values are used to allocate space during the initialization of a navigation mesh.
/// @see dtNavMesh::init()
/// @ingroup detour
type dtNavMeshParams struct {
	Orig       [3]float32 ///< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	TileWidth  float32    ///< The width of each tile. (Along the x-axis.)
	TileHeight float32    ///< The height of each tile. (Along the z-axis.)
	MaxTiles   int32      ///< The maximum number of tiles the navigation mesh can contain.
	MaxPolys   int32      ///< The maximum number of polygons each tile can contain.
}

//func (prm *dtNavMeshParams) decode(r io.Reader) error {
//err := binary.Read(r, binary.LittleEndian, &hdr.magic)
//if err != nil {
//return err
//}
//if hdr.magic != NAVMESHSET_MAGIC {
//return fmt.Errorf("wrong magic number: %x", hdr.magic)
//}

//err = binary.Read(r, binary.LittleEndian, &hdr.version)
//if err != nil {
//return err
//}
//if hdr.version != NAVMESHSET_VERSION {
//return fmt.Errorf("wrong version")
//}

//err = binary.Read(r, binary.LittleEndian, &hdr.numTiles)
//if err != nil {
//return err
//}

//return nil
//}

// Provides high level information related to a dtMeshTile object.
type dtMeshHeader struct {
	magic           int        // Tile magic number. (Used to identify the data format.)
	version         int        // Tile data format version number.
	x               int        // The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	y               int        // The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	layer           int        // The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	userId          uint       // The user defined id of the tile.
	polyCount       int        // The number of polygons in the tile.
	vertCount       int        // The number of vertices in the tile.
	maxLinkCount    int        // The number of allocated links.
	detailMeshCount int        // The number of sub-meshes in the detail mesh.
	detailVertCount int        // The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	detailTriCount  int        // The number of triangles in the detail mesh.
	bvNodeCount     int        // The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	offMeshConCount int        // The number of off-mesh connections.
	offMeshBase     int        // The index of the first polygon which is an off-mesh connection.
	walkableHeight  float32    // The height of the agents using the tile.
	walkableRadius  float32    // The radius of the agents using the tile.
	walkableClimb   float32    // The maximum climb height of the agents using the tile.
	bmin            [3]float32 // The minimum bounds of the tile's AABB. [(x, y, z)]
	bmax            [3]float32 // The maximum bounds of the tile's AABB. [(x, y, z)]

	/// The bounding volume quantization factor.
	bvQuantFactor float32
}

/// Defines a navigation mesh tile.
type dtMeshTile struct {
	salt uint // Counter describing modifications to the tile.

	linksFreeList uint           // Index to the next free link.
	header        *dtMeshHeader  // The tile header.
	polys         []dtPoly       // The tile polygons. [Size: dtMeshHeader::polyCount]
	verts         []float32      // The tile vertices. [Size: dtMeshHeader::vertCount]
	links         []dtLink       // The tile links. [Size: dtMeshHeader::maxLinkCount]
	detailMeshes  []dtPolyDetail // The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	detailVerts   []float32      // The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	detailTris []uint8

	// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	// (Will be null if bounding volumes are disabled.)
	bvTree []dtBVNode
	// The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]
	offMeshCons []dtOffMeshConnection

	data     []uint8     // The tile data. (Not directly accessed under normal situations.)
	dataSize int         // Size of the tile data.
	flags    int         // Tile flags. (See: #dtTileFlags)
	next     *dtMeshTile // The next free tile, or the next tile in the spatial grid.
}
