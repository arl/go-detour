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
		//fmt.Printf("tile %d: %v\n", i, mesh.Tiles[i])
		if len(mesh.Tiles[i].OffMeshCons) > 0 {
			fmt.Printf("tile %d has off-mesh connections,\n%v\n", i, mesh.Tiles[i].OffMeshCons)
			fmt.Println("links", mesh.Tiles[i].Links)
		}
	}
	return &mesh, nil
}
