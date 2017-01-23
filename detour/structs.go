package detour

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"unsafe"
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

// TODO: should probably remove this useless function
func (s *MeshHeader) WriteTo(w io.Writer) (n int64, err error) {
	// write each field as little endian
	binary.Write(w, binary.LittleEndian, s.Magic)
	binary.Write(w, binary.LittleEndian, s.Version)
	binary.Write(w, binary.LittleEndian, s.X)
	binary.Write(w, binary.LittleEndian, s.Y)
	binary.Write(w, binary.LittleEndian, s.Layer)
	binary.Write(w, binary.LittleEndian, s.UserID)
	binary.Write(w, binary.LittleEndian, s.PolyCount)
	binary.Write(w, binary.LittleEndian, s.VertCount)
	binary.Write(w, binary.LittleEndian, s.MaxLinkCount)
	binary.Write(w, binary.LittleEndian, s.DetailMeshCount)
	binary.Write(w, binary.LittleEndian, s.DetailVertCount)
	binary.Write(w, binary.LittleEndian, s.DetailTriCount)
	binary.Write(w, binary.LittleEndian, s.BvNodeCount)
	binary.Write(w, binary.LittleEndian, s.OffMeshConCount)
	binary.Write(w, binary.LittleEndian, s.OffMeshBase)
	binary.Write(w, binary.LittleEndian, s.WalkableHeight)
	binary.Write(w, binary.LittleEndian, s.WalkableRadius)
	binary.Write(w, binary.LittleEndian, s.WalkableClimb)
	binary.Write(w, binary.LittleEndian, s.Bmin)
	binary.Write(w, binary.LittleEndian, s.Bmax)
	binary.Write(w, binary.LittleEndian, s.BvQuantFactor)
	// TODO: do not hard-code this
	return 100, nil
}

// TODO: should probably remove this useless function
func (s *MeshHeader) ReadFrom(r io.Reader) (n int64, err error) {
	// read each field as little endian
	binary.Read(r, binary.LittleEndian, &s.Magic)
	binary.Read(r, binary.LittleEndian, &s.Version)
	binary.Read(r, binary.LittleEndian, &s.X)
	binary.Read(r, binary.LittleEndian, &s.Y)
	binary.Read(r, binary.LittleEndian, &s.Layer)
	binary.Read(r, binary.LittleEndian, &s.UserID)
	binary.Read(r, binary.LittleEndian, &s.PolyCount)
	binary.Read(r, binary.LittleEndian, &s.VertCount)
	binary.Read(r, binary.LittleEndian, &s.MaxLinkCount)
	binary.Read(r, binary.LittleEndian, &s.DetailMeshCount)
	binary.Read(r, binary.LittleEndian, &s.DetailVertCount)
	binary.Read(r, binary.LittleEndian, &s.DetailTriCount)
	binary.Read(r, binary.LittleEndian, &s.BvNodeCount)
	binary.Read(r, binary.LittleEndian, &s.OffMeshConCount)
	binary.Read(r, binary.LittleEndian, &s.OffMeshBase)
	binary.Read(r, binary.LittleEndian, &s.WalkableHeight)
	binary.Read(r, binary.LittleEndian, &s.WalkableRadius)
	binary.Read(r, binary.LittleEndian, &s.WalkableClimb)
	binary.Read(r, binary.LittleEndian, &s.Bmin)
	binary.Read(r, binary.LittleEndian, &s.Bmax)
	binary.Read(r, binary.LittleEndian, &s.BvQuantFactor)
	// TODO: do not hard-code this
	return 100, nil
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

// UpdateData updates the Data field with the actual content of the tile
// for later serialization.

//we are doing exactly the same thing in navmeshcreate ligne 783,
//factorize this
func (s *MeshTile) UpdateData() error {
	var buf bytes.Buffer
	expected := int(s.DataSize) - int(unsafe.Sizeof(*s.Header))
	buf.Grow(expected)

	// write each field as little endian
	binary.Write(&buf, binary.LittleEndian, s.Verts)
	binary.Write(&buf, binary.LittleEndian, s.Polys)
	binary.Write(&buf, binary.LittleEndian, s.Links)
	for i := range s.DetailMeshes {
		s.DetailMeshes[i].WriteTo(&buf)
		//binary.Write(&buf, binary.LittleEndian, s.DetailMeshes)
	}
	binary.Write(&buf, binary.LittleEndian, s.DetailVerts)
	binary.Write(&buf, binary.LittleEndian, s.DetailTris)
	binary.Write(&buf, binary.LittleEndian, s.BvTree)
	binary.Write(&buf, binary.LittleEndian, s.OffMeshCons)

	// buffer should not be larger than DataSize
	if buf.Len() != expected {
		return fmt.Errorf("couldn't update MeshTile.Data(buf:%v, DataSize:%v)\n", buf.Len(), expected)
	}

	s.Data = buf.Bytes()
	return nil
}
