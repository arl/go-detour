package detour

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
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
	var (
		little littleEndian
		off    int
	)
	for _, f := range verts {
		little.PutUint32(dst[off:], uint32(math.Float32bits(f)))
		off += 4
	}
	for i := range polys {
		p := &polys[i]
		little.PutUint32(dst[off:], uint32(p.FirstLink))
		off += 4

		for j := uint32(0); j < VertsPerPolygon; j++ {
			little.PutUint16(dst[off:], p.Verts[j])
			off += 2
		}

		for j := uint32(0); j < VertsPerPolygon; j++ {
			little.PutUint16(dst[off:], p.Neis[j])
			off += 2
		}

		little.PutUint16(dst[off:], p.Flags)
		dst[off+2] = p.VertCount
		dst[off+3] = p.AreaAndType
		off += 4
	}
	for i := range links {
		l := &links[i]

		little.PutUint32(dst[off:], uint32(l.Ref))
		little.PutUint32(dst[off+4:], l.Next)

		dst[off+8] = l.Edge
		dst[off+9] = l.Side
		dst[off+10] = l.Bmin
		dst[off+11] = l.Bmax
		off += 12
	}

	for i := range dmeshes {
		m := &dmeshes[i]

		little.PutUint32(dst[off:], m.VertBase)
		little.PutUint32(dst[off+4:], m.TriBase)
		dst[off+8] = m.VertCount
		dst[off+9] = m.TriCount
		off += 12
	}
	for i := range dverts {
		little.PutUint32(dst[off:], uint32(math.Float32bits(dverts[i])))
		off += 4
	}
	copy(dst[off:], dtris)
	off += len(dtris)

	for i := range bvtree {
		t := &bvtree[i]
		little.PutUint16(dst[off:], t.Bmin[0])
		little.PutUint16(dst[off+2:], t.Bmin[1])
		little.PutUint16(dst[off+4:], t.Bmin[2])
		little.PutUint16(dst[off+6:], t.Bmax[0])
		little.PutUint16(dst[off+8:], t.Bmax[1])
		little.PutUint16(dst[off+10:], t.Bmax[2])
		little.PutUint32(dst[off+12:], uint32(t.I))
		off += 16
	}
	for i := range offMeshCons {
		o := &offMeshCons[i]
		little.PutUint32(dst[off:], uint32(math.Float32bits(o.Pos[0])))
		little.PutUint32(dst[off+4:], uint32(math.Float32bits(o.Pos[1])))
		little.PutUint32(dst[off+8:], uint32(math.Float32bits(o.Pos[2])))
		little.PutUint32(dst[off+12:], uint32(math.Float32bits(o.Pos[3])))
		little.PutUint32(dst[off+16:], uint32(math.Float32bits(o.Pos[4])))
		little.PutUint32(dst[off+20:], uint32(math.Float32bits(o.Pos[5])))
		little.PutUint32(dst[off+24:], uint32(math.Float32bits(o.Rad)))
		little.PutUint16(dst[off+28:], o.Poly)
		dst[30] = o.Flags
		dst[31] = o.Side
		little.PutUint32(dst[off+32:], o.UserID)
		off += 36
	}
	return nil
}
