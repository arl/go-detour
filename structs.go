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
	MaxTiles   uint32     ///< The maximum number of tiles the navigation mesh can contain.
	MaxPolys   uint32     ///< The maximum number of polygons each tile can contain.
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
	Magic           int32      // Tile magic number. (Used to identify the data format.)
	Version         int32      // Tile data format version number.
	X               int32      // The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	Y               int32      // The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	Layer           int32      // The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	UserId          uint32     // The user defined id of the tile.
	PolyCount       int32      // The number of polygons in the tile.
	VertCount       int32      // The number of vertices in the tile.
	MaxLinkCount    int32      // The number of allocated links.
	DetailMeshCount int32      // The number of sub-meshes in the detail mesh.
	DetailVertCount int32      // The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	DetailTriCount  int32      // The number of triangles in the detail mesh.
	BvNodeCount     int32      // The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	OffMeshConCount int32      // The number of off-mesh connections.
	OffMeshBase     int32      // The index of the first polygon which is an off-mesh connection.
	WalkableHeight  float32    // The height of the agents using the tile.
	WalkableRadius  float32    // The radius of the agents using the tile.
	WalkableClimb   float32    // The maximum climb height of the agents using the tile.
	Bmin            [3]float32 // The minimum bounds of the tile's AABB. [(x, y, z)]
	Bmax            [3]float32 // The maximum bounds of the tile's AABB. [(x, y, z)]

	/// The bounding volume quantization factor.
	BvQuantFactor float32
}

/// Defines a navigation mesh tile.
type dtMeshTile struct {
	Salt uint32 // Counter describing modifications to the tile.

	LinksFreeList uint32         // Index to the next free link.
	Header        *dtMeshHeader  // The tile header.
	Polys         []dtPoly       // The tile polygons. [Size: dtMeshHeader::polyCount]
	Verts         []float32      // The tile vertices. [Size: dtMeshHeader::vertCount]
	Links         []dtLink       // The tile links. [Size: dtMeshHeader::maxLinkCount]
	DetailMeshes  []dtPolyDetail // The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	DetailVerts   []float32      // The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	DetailTris []uint8

	// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	// (Will be null if bounding volumes are disabled.)
	BvTree []dtBVNode
	// The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]
	OffMeshCons []dtOffMeshConnection

	Data     []uint8     // The tile data. (Not directly accessed under normal situations.)
	DataSize int32       // Size of the tile data.
	Flags    int32       // Tile flags. (See: #dtTileFlags)
	Next     *dtMeshTile // The next free tile, or the next tile in the spatial grid.
}
