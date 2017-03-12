package recast

import (
	"io"

	"github.com/aurelien-rainone/gobj"
	"github.com/aurelien-rainone/math32"
)

type MeshLoaderOBJ struct {
	scale   float32
	verts   []float32
	tris    []int32
	normals []float32
}

func NewMeshLoaderOBJ() *MeshLoaderOBJ {
	return &MeshLoaderOBJ{
		scale:   1.0,
		verts:   make([]float32, 0),
		tris:    make([]int32, 0),
		normals: make([]float32, 0),
	}
}

func (mlo *MeshLoaderOBJ) Load(r io.Reader) error {
	var (
		obj *gobj.OBJFile
		err error
	)
	obj, err = gobj.Decode(r)
	if err != nil {
		return err
	}

	// copy vertices indices from OBJ,
	// multiplying them by the scale factor
	verts := obj.Verts()
	count := int32(len(verts)) * 3
	mlo.verts = make([]float32, count)
	for i := int32(0); i < count; i += 3 {
		mlo.verts[i] = float32(verts[i/3][0]) * mlo.scale
		mlo.verts[i+1] = float32(verts[i/3][1]) * mlo.scale
		mlo.verts[i+2] = float32(verts[i/3][2]) * mlo.scale
	}
	vertcount := int32(len(verts))

	// add polygons
	for _, p := range obj.Polys() {
		for i := 2; i < len(p); i++ {
			a := p[0]
			b := p[i-1]
			c := p[i]
			if a < 0 || a >= vertcount || b < 0 || b >= vertcount || c < 0 || c >= vertcount {
				continue
			}
			mlo.tris = append(mlo.tris, a, b, c)
		}
	}

	// Calculate normals.
	// TODO: factor this with recast.calcTriNormal
	var e0, e1 [3]float32
	mlo.normals = make([]float32, len(mlo.tris))
	for i := 0; i < len(mlo.tris); i += 3 {
		v0 := mlo.verts[mlo.tris[i]*3 : 3+mlo.tris[i]*3]
		v1 := mlo.verts[mlo.tris[i+1]*3 : 3+mlo.tris[i+1]*3]
		v2 := mlo.verts[mlo.tris[i+2]*3 : 3+mlo.tris[i+2]*3]
		for j := 0; j < 3; j++ {
			e0[j] = v1[j] - v0[j]
			e1[j] = v2[j] - v0[j]
		}
		n := mlo.normals[i : 3+i]
		n[0] = e0[1]*e1[2] - e0[2]*e1[1]
		n[1] = e0[2]*e1[0] - e0[0]*e1[2]
		n[2] = e0[0]*e1[1] - e0[1]*e1[0]
		d := math32.Sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2])
		if d > 0 {
			d = 1.0 / d
			n[0] *= d
			n[1] *= d
			n[2] *= d
		}
	}

	return nil
}

func (mlo *MeshLoaderOBJ) Scale() float32 {
	return mlo.scale
}

func (mlo *MeshLoaderOBJ) Verts() []float32 {
	return mlo.verts
}

func (mlo *MeshLoaderOBJ) Tris() []int32 {
	return mlo.tris
}

func (mlo *MeshLoaderOBJ) Normals() []float32 {
	return mlo.normals
}

func (mlo *MeshLoaderOBJ) VertCount() int32 {
	return int32(len(mlo.verts) / 3)
}

func (mlo *MeshLoaderOBJ) TriCount() int32 {
	return int32(len(mlo.tris) / 3)
}
