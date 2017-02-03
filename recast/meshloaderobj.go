package recast

import (
	"github.com/aurelien-rainone/gobj"
	"github.com/aurelien-rainone/math32"
)

type MeshLoaderObj struct {
	m_filename  string
	m_scale     float32
	m_verts     []float32
	m_tris      []int32
	m_normals   []float32
	m_vertCount int32
	m_triCount  int32
}

func NewMeshLoaderObj() *MeshLoaderObj {
	return &MeshLoaderObj{
		m_scale:   1.0,
		m_verts:   make([]float32, 0),
		m_tris:    make([]int32, 0),
		m_normals: make([]float32, 0),
	}
}

func (mlo *MeshLoaderObj) Load(filename string) error {
	var (
		obj *gobj.OBJFile
		err error
	)
	obj, err = gobj.Load(filename)
	if err != nil {
		return err
	}

	// copy vertices indices from OBJ,
	// multiplying them by the scale factor
	verts := obj.Verts()
	count := int32(len(verts)) * 3
	mlo.m_verts = make([]float32, count)
	for i := int32(0); i < count; i += 3 {
		mlo.m_verts[i] = float32(verts[i/3][0]) * mlo.m_scale
		mlo.m_verts[i+1] = float32(verts[i/3][1]) * mlo.m_scale
		mlo.m_verts[i+2] = float32(verts[i/3][2]) * mlo.m_scale
	}
	mlo.m_vertCount = int32(len(verts))

	// add polygons
	for _, p := range obj.Polys() {
		for i := 2; i < len(p); i++ {
			a := p[0]
			b := p[i-1]
			c := p[i]
			if a < 0 || a >= mlo.m_vertCount || b < 0 || b >= mlo.m_vertCount || c < 0 || c >= mlo.m_vertCount {
				continue
			}
			mlo.m_tris = append(mlo.m_tris, a, b, c)
			mlo.m_triCount++
		}
	}

	// Calculate normals.
	// TODO: factor this with recast.calcTriNormal
	mlo.m_normals = make([]float32, mlo.m_triCount*3)
	for i := int32(0); i < mlo.m_triCount*3; i += 3 {
		v0 := mlo.m_verts[mlo.m_tris[i]*3 : 3+mlo.m_tris[i]*3]
		v1 := mlo.m_verts[mlo.m_tris[i+1]*3 : 3+mlo.m_tris[i+1]*3]
		v2 := mlo.m_verts[mlo.m_tris[i+2]*3 : 3+mlo.m_tris[i+2]*3]
		var e0, e1 [3]float32
		for j := 0; j < 3; j++ {
			e0[j] = v1[j] - v0[j]
			e1[j] = v2[j] - v0[j]
		}
		n := mlo.m_normals[i : 3+i]
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

	mlo.m_filename = filename
	return nil
}

func (mlo *MeshLoaderObj) Filename() string {
	return mlo.m_filename
}

func (mlo *MeshLoaderObj) Scale() float32 {
	return mlo.m_scale
}

func (mlo *MeshLoaderObj) Verts() []float32 {
	return mlo.m_verts
}

func (mlo *MeshLoaderObj) Tris() []int32 {
	return mlo.m_tris
}

func (mlo *MeshLoaderObj) Normals() []float32 {
	return mlo.m_normals
}

func (mlo *MeshLoaderObj) VertCount() int32 {
	return mlo.m_vertCount
}

func (mlo *MeshLoaderObj) TriCount() int32 {
	return mlo.m_triCount
}
