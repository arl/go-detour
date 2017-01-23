package detour

import (
	"encoding/binary"
	"fmt"
	"io"
)

// TileRef is a reference to a tile of the navigation mesh.
type TileRef uint32

type navMeshTileHeader struct {
	TileRef  TileRef
	DataSize int32
}

// Decode reads a tiled navigation mesh from r and returns the loaded NavMesh
// object or nil and an error in case of failure.
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
	var i int32
	for i = 0; i < hdr.NumTiles; i++ {

		var tileHdr navMeshTileHeader
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
		err = binary.Read(r, binary.LittleEndian, &data)
		if err != nil {
			return nil, err
		}
		status, _ := mesh.addTile(data, tileHdr.DataSize, tileHdr.TileRef)
		if status&Failure != 0 {
			return nil, fmt.Errorf("couldn't add tile %d(), status: 0x%x\n", i, status)
		}
	}
	return &mesh, nil
}

func SerializeTile(dst []byte,
	verts []float32,
	polys []Poly,
	links []Link,
	dmeshes []PolyDetail,
	dverts []float32,
	dtris []uint8,
	bvtree []BvNode,
	offMeshCons []OffMeshConnection,
) error {
	bufw := NewBufWriter(dst)
	for i := range verts {
		binary.Write(bufw, binary.LittleEndian, verts[i])
	}
	for i := range polys {
		binary.Write(bufw, binary.LittleEndian, polys[i])
	}
	for i := range links {
		binary.Write(bufw, binary.LittleEndian, links[i])
	}
	for i := range dmeshes {
		binary.Write(bufw, binary.LittleEndian, dmeshes[i])
		bufw.GoForward(2)
	}
	for i := range dverts {
		binary.Write(bufw, binary.LittleEndian, dverts[i])
	}
	for i := range dtris {
		binary.Write(bufw, binary.LittleEndian, dtris[i])
	}
	for i := range bvtree {
		binary.Write(bufw, binary.LittleEndian, bvtree[i])
	}
	for i := range offMeshCons {
		binary.Write(bufw, binary.LittleEndian, offMeshCons[i])
	}
	return nil
}
