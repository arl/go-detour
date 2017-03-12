package recast

import (
	"io"

	"github.com/aurelien-rainone/gobj"
	"github.com/aurelien-rainone/math32"
)

type MeshLoaderObj struct {
	scale     float32
	verts     []float32
	tris      []int32
	normals   []float32
	vertcount int32
	tricount  int32
}

func NewMeshLoaderObj() *MeshLoaderObj {
	return &MeshLoaderObj{
		scale:   1.0,
		verts:   make([]float32, 0),
		tris:    make([]int32, 0),
		normals: make([]float32, 0),
	}
}

func (mlo *MeshLoaderObj) Load(r io.Reader) error {
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
	mlo.vertcount = int32(len(verts))

	// add polygons
	for _, p := range obj.Polys() {
		for i := 2; i < len(p); i++ {
			a := p[0]
			b := p[i-1]
			c := p[i]
			if a < 0 || a >= mlo.vertcount || b < 0 || b >= mlo.vertcount || c < 0 || c >= mlo.vertcount {
				continue
			}
			mlo.tris = append(mlo.tris, a, b, c)
			mlo.tricount++
		}
	}

	// Calculate normals.
	// TODO: factor this with recast.calcTriNormal
	mlo.normals = make([]float32, mlo.tricount*3)
	for i := int32(0); i < mlo.tricount*3; i += 3 {
		v0 := mlo.verts[mlo.tris[i]*3 : 3+mlo.tris[i]*3]
		v1 := mlo.verts[mlo.tris[i+1]*3 : 3+mlo.tris[i+1]*3]
		v2 := mlo.verts[mlo.tris[i+2]*3 : 3+mlo.tris[i+2]*3]
		var e0, e1 [3]float32
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

func (mlo *MeshLoaderObj) Scale() float32 {
	return mlo.scale
}

func (mlo *MeshLoaderObj) Verts() []float32 {
	return mlo.verts
}

func (mlo *MeshLoaderObj) Tris() []int32 {
	return mlo.tris
}

func (mlo *MeshLoaderObj) Normals() []float32 {
	return mlo.normals
}

func (mlo *MeshLoaderObj) VertCount() int32 {
	return mlo.vertcount
}

func (mlo *MeshLoaderObj) TriCount() int32 {
	return mlo.tricount
}
