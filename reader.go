package detour

import (
	"encoding/binary"
	"fmt"
	"io"
	"reflect"
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
	status := mesh.init(&hdr.Params)
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

// alignedReader performs aligned reading operations. It ensures that after a
// successfull Read operation, the file offset has been moved by a multiple of
// the specified alignment. It is useful to read binary packed arrays or
// structures.
type alignedReader struct {
	r     io.ReadSeeker // ReadSeeker to which calls are forwarded
	align uint          // byte alignment
}

func align(x, a uint) uint {
	r := x % a
	if r == 0 {
		return x
	}
	return x + (a - r)
}

// newAlignedReader returns an alignedReader, performing read operations aligned
// on align bytes.
func newAlignedReader(r io.ReadSeeker, align uint) *alignedReader {
	return &alignedReader{r: r, align: align}
}

func (ar *alignedReader) Read(b []byte) (n int, err error) {
	n, err = ar.r.Read(b)
	pad := align(uint(n), ar.align) - uint(n)
	if pad != 0 {
		_, err = ar.r.Seek(int64(pad), io.SeekCurrent)
		if err != nil {
			return n, fmt.Errorf("couldn't seek %d padding bytes", pad)
		}
	}
	return
}

func (ar *alignedReader) readSlice(data interface{}, order binary.ByteOrder) error {
	var err error
	rt := reflect.TypeOf(data)
	rv := reflect.ValueOf(data)

	if rt.Kind() != reflect.Ptr || rv.Elem().Kind() != reflect.Slice {
		return fmt.Errorf("data must be a pointer to slice")
	}
	for idx := 0; idx < rv.Elem().Len(); idx++ {
		err = binary.Read(ar, order, rv.Elem().Index(idx).Addr().Interface())
		if err != nil {
			return err
		}
	}
	return nil
}
