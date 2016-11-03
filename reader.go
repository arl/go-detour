package detour

import (
	"encoding/binary"
	"fmt"
	"io"
	"log"
)

type Reader struct{}

//func (r Reader) Read(p []byte) (n int, err error)

// Decode reads a PNG image from r and returns it as an image.Image.
// The type of Image returned depends on the PNG contents.
func Decode(r io.Reader) (*DtNavMesh, error) {
	// Read header.
	var (
		hdr NavMeshSetHeader
		err error
	)

	err = binary.Read(r, binary.LittleEndian, &hdr)
	if err != nil {
		return nil, err
	}
	fmt.Println(hdr)

	if hdr.Magic != NAVMESHSET_MAGIC {
		return nil, fmt.Errorf("wrong magic number: %x", hdr.Magic)
	}

	if hdr.Version != NAVMESHSET_VERSION {
		return nil, fmt.Errorf("wrong version: %d", hdr.Version)
	}

	var mesh DtNavMesh
	status := mesh.init(&hdr.Params)
	if DtStatusFailed(status) {
		return nil, fmt.Errorf("status failed 0x%x", status)
	}

	fmt.Println("numTiles", hdr.NumTiles)

	// Read tiles.
	var i int32
	for i = 0; i < hdr.NumTiles; i++ {

		var tileHdr NavMeshTileHeader
		err = binary.Read(r, binary.LittleEndian, &tileHdr)
		if err != nil {
			return nil, err
		}

		if tileHdr.TileRef == 0 || tileHdr.DataSize == 0 {
			break
		}

		log.Println("reading tile header", i, tileHdr)

		data := make([]byte, tileHdr.DataSize)
		if data == nil {
			break
		}
		err = binary.Read(r, binary.LittleEndian, &data)
		if err != nil {
			return nil, err
		}
		status := mesh.addTile(data, tileHdr.DataSize, tileHdr.TileRef, nil)
		if status&DT_FAILURE != 0 {
			log.Fatal("mesh.addTile() returned 0x%x\n", status)
		}
	}
	log.Println(hdr.NumTiles, "tiles added successfully")
	return &mesh, nil
}
