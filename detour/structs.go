package detour

import (
	"encoding/binary"
	"math"
)

type navMeshSetHeader struct {
	Magic    int32
	Version  int32
	NumTiles int32
	Params   NavMeshParams
}

// NavMeshParams contains the configuration parameters used to define
// multi-tile navigation meshes.
//
// The values are used to allocate space during the initialization of a navigation mesh.
// see NavMesh.init()
type NavMeshParams struct {
	Orig       [3]float32 // The world space origin of the navigation mesh's tile space. [(x, y, z)]
	TileWidth  float32    // The width of each tile. (Along the x-axis.)
	TileHeight float32    // The height of each tile. (Along the z-axis.)
	MaxTiles   uint32     // The maximum number of tiles the navigation mesh can contain.
	MaxPolys   uint32     // The maximum number of polygons each tile can contain.
}

// MeshHeader provides high level information related to a MeshTile object.
type MeshHeader struct {
	Magic           int32      // Tile magic number. (Used to identify the data format.)
	Version         int32      // Tile data format version number.
	X               int32      // The x-position of the tile within the NavMesh tile grid. (x, y, layer)
	Y               int32      // The y-position of the tile within the NavMesh tile grid. (x, y, layer)
	Layer           int32      // The layer of the tile within the NavMesh tile grid. (x, y, layer)
	UserID          uint32     // The user defined id of the tile.
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
	BvQuantFactor   float32    // The bounding volume quantization factor.
}

func (s *MeshHeader) Serialize(dst []byte) int {
	if len(dst) < 100 {
		panic("undersized buffer for MeshHeader")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	little.PutUint32(dst[off:], uint32(s.Magic))
	little.PutUint32(dst[off+4:], uint32(s.Version))
	little.PutUint32(dst[off+8:], uint32(s.X))
	little.PutUint32(dst[off+12:], uint32(s.Y))
	little.PutUint32(dst[off+16:], uint32(s.Layer))
	little.PutUint32(dst[off+20:], uint32(s.UserID))
	little.PutUint32(dst[off+24:], uint32(s.PolyCount))
	little.PutUint32(dst[off+28:], uint32(s.VertCount))
	little.PutUint32(dst[off+32:], uint32(s.MaxLinkCount))
	little.PutUint32(dst[off+36:], uint32(s.DetailMeshCount))
	little.PutUint32(dst[off+40:], uint32(s.DetailVertCount))
	little.PutUint32(dst[off+44:], uint32(s.DetailTriCount))
	little.PutUint32(dst[off+48:], uint32(s.BvNodeCount))
	little.PutUint32(dst[off+52:], uint32(s.OffMeshConCount))
	little.PutUint32(dst[off+56:], uint32(s.OffMeshBase))
	little.PutUint32(dst[off+60:], uint32(math.Float32bits(s.WalkableHeight)))
	little.PutUint32(dst[off+64:], uint32(math.Float32bits(s.WalkableRadius)))
	little.PutUint32(dst[off+68:], uint32(math.Float32bits(s.WalkableClimb)))
	little.PutUint32(dst[off+72:], uint32(math.Float32bits(s.Bmin[0])))
	little.PutUint32(dst[off+76:], uint32(math.Float32bits(s.Bmin[1])))
	little.PutUint32(dst[off+80:], uint32(math.Float32bits(s.Bmin[2])))
	little.PutUint32(dst[off+84:], uint32(math.Float32bits(s.Bmax[0])))
	little.PutUint32(dst[off+88:], uint32(math.Float32bits(s.Bmax[1])))
	little.PutUint32(dst[off+92:], uint32(math.Float32bits(s.Bmax[2])))
	little.PutUint32(dst[off+96:], uint32(math.Float32bits(s.BvQuantFactor)))
	return 100
}

// MeshTile defines a navigation mesh tile.
type MeshTile struct {
	Salt          uint32       // Counter describing modifications to the tile.
	LinksFreeList uint32       // Index to the next free link.
	Header        *MeshHeader  // The tile header.
	Polys         []Poly       // The tile polygons. [Size: MeshHeader.polyCount]
	Verts         []float32    // The tile vertices. [Size: MeshHeader.vertCount]
	Links         []Link       // The tile links. [Size: MeshHeader.maxLinkCount]
	DetailMeshes  []PolyDetail // The tile's detail sub-meshes. [Size: MeshHeader.detailMeshCount]
	DetailVerts   []float32    // The detail mesh's unique vertices. [(x, y, z) * MeshHeader.detailVertCount]
	// The detail mesh's triangles. [(vertA, vertB, vertC) * MeshHeader.detailTriCount]
	DetailTris []uint8

	// The tile bounding volume nodes. [Size: MeshHeader.BvNodeCount]
	// (Will be null if bounding volumes are disabled.)
	BvTree []BvNode
	// The tile off-mesh connections. [Size: MeshHeader.offMeshConCount]
	OffMeshCons []OffMeshConnection

	Data     []uint8   // The tile data. (Not directly accessed under normal situations.)
	DataSize int32     // Size of the tile data.
	Flags    int32     // Tile flags. (See: tileFlags)
	Next     *MeshTile // The next free tile, or the next tile in the spatial grid.
}
