package detour

import (
	"encoding/binary"
	"io"
	"math"
)

type navMeshSetHeader struct {
	Magic    uint32
	Version  uint32
	NumTiles uint32
	Params   NavMeshParams
}

func (s *navMeshSetHeader) Size() int {
	return 12 + s.Params.size()
}

func (s *navMeshSetHeader) serialize(dst []byte) {
	if len(dst) < s.Size() {
		panic("undersized buffer for navMeshSetHeader")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	little.PutUint32(dst[off:], s.Magic)
	little.PutUint32(dst[off+4:], s.Version)
	little.PutUint32(dst[off+8:], s.NumTiles)
	s.Params.serialize(dst[off+12:])
}

func (s *navMeshSetHeader) WriteTo(w io.Writer) (int64, error) {
	buf := make([]byte, s.Size())
	s.serialize(buf)

	n, err := w.Write(buf)
	return int64(n), err
}

// NavMeshParams contains the configuration parameters used to define
// multi-tile navigation meshes.
//
// The values are used to allocate space during the initialization of a
// navigation mesh.
// see NavMesh.Init()
type NavMeshParams struct {
	Orig       [3]float32 // The world space origin of the navigation mesh's tile space. [(x, y, z)]
	TileWidth  float32    // The width of each tile. (Along the x-axis.)
	TileHeight float32    // The height of each tile. (Along the z-axis.)
	MaxTiles   uint32     // The maximum number of tiles the navigation mesh can contain.
	MaxPolys   uint32     // The maximum number of polygons each tile can contain.
}

// size returns the size of the serialized structure.
func (s *NavMeshParams) size() int {
	return 28
}

// serialize encodes the structure content into dst.
//
// The function panics is the destination slice is too small.
func (s *NavMeshParams) serialize(dst []byte) {
	if len(dst) < s.size() {
		panic("destination slice is too small")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	little.PutUint32(dst[off:], uint32(math.Float32bits(s.Orig[0])))
	little.PutUint32(dst[off+4:], uint32(math.Float32bits(s.Orig[1])))
	little.PutUint32(dst[off+8:], uint32(math.Float32bits(s.Orig[2])))
	little.PutUint32(dst[off+12:], uint32(math.Float32bits(s.TileWidth)))
	little.PutUint32(dst[off+16:], uint32(math.Float32bits(s.TileHeight)))
	little.PutUint32(dst[off+20:], uint32(s.MaxTiles))
	little.PutUint32(dst[off+24:], uint32(s.MaxPolys))
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
	BMin            [3]float32 // The minimum bounds of the tile's AABB. [(x, y, z)]
	BMax            [3]float32 // The maximum bounds of the tile's AABB. [(x, y, z)]
	BvQuantFactor   float32    // The bounding volume quantization factor.
}

func (s *MeshHeader) size() int {
	return 100
}

func (s *MeshHeader) serialize(dst []byte) {
	if len(dst) < s.size() {
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
	little.PutUint32(dst[off+72:], uint32(math.Float32bits(s.BMin[0])))
	little.PutUint32(dst[off+76:], uint32(math.Float32bits(s.BMin[1])))
	little.PutUint32(dst[off+80:], uint32(math.Float32bits(s.BMin[2])))
	little.PutUint32(dst[off+84:], uint32(math.Float32bits(s.BMax[0])))
	little.PutUint32(dst[off+88:], uint32(math.Float32bits(s.BMax[1])))
	little.PutUint32(dst[off+92:], uint32(math.Float32bits(s.BMax[2])))
	little.PutUint32(dst[off+96:], uint32(math.Float32bits(s.BvQuantFactor)))
}

func (s *MeshHeader) unserialize(src []byte) {
	if len(src) < s.size() {
		panic("undersized buffer for MeshHeader")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	s.Magic = int32(little.Uint32(src[off:]))
	s.Version = int32(little.Uint32(src[off+4:]))
	s.X = int32(little.Uint32(src[off+8:]))
	s.Y = int32(little.Uint32(src[off+12:]))
	s.Layer = int32(little.Uint32(src[off+16:]))
	s.UserID = little.Uint32(src[off+20:])
	s.PolyCount = int32(little.Uint32(src[off+24:]))
	s.VertCount = int32(little.Uint32(src[off+28:]))
	s.MaxLinkCount = int32(little.Uint32(src[off+32:]))
	s.DetailMeshCount = int32(little.Uint32(src[off+36:]))
	s.DetailVertCount = int32(little.Uint32(src[off+40:]))
	s.DetailTriCount = int32(little.Uint32(src[off+44:]))
	s.BvNodeCount = int32(little.Uint32(src[off+48:]))
	s.OffMeshConCount = int32(little.Uint32(src[off+52:]))
	s.OffMeshBase = int32(little.Uint32(src[off+56:]))
	s.WalkableHeight = math.Float32frombits(little.Uint32(src[off+60:]))
	s.WalkableRadius = math.Float32frombits(little.Uint32(src[off+64:]))
	s.WalkableClimb = math.Float32frombits(little.Uint32(src[off+68:]))
	s.BMin[0] = math.Float32frombits(little.Uint32(src[off+72:]))
	s.BMin[1] = math.Float32frombits(little.Uint32(src[off+76:]))
	s.BMin[2] = math.Float32frombits(little.Uint32(src[off+80:]))
	s.BMax[0] = math.Float32frombits(little.Uint32(src[off+84:]))
	s.BMax[1] = math.Float32frombits(little.Uint32(src[off+88:]))
	s.BMax[2] = math.Float32frombits(little.Uint32(src[off+92:]))
	s.BvQuantFactor = math.Float32frombits(little.Uint32(src[off+96:]))
}
