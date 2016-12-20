package detour

import (
	"errors"
	"fmt"

	"github.com/aurelien-rainone/gobj"
	"github.com/aurelien-rainone/math32"
)

type rcMeshLoaderObj struct {
	m_filename  string
	m_scale     float32
	m_verts     []float32
	m_tris      []int32
	m_normals   []float32
	m_vertCount int32
	m_triCount  int32
}

func newMeshLoaderObj() *rcMeshLoaderObj {
	return &rcMeshLoaderObj{
		m_scale: 1.0,
	}
}

func (mlo *rcMeshLoaderObj) load(filename string) error {
	var (
		obj *gobj.OBJFile
		err error
	)
	obj, err = gobj.Load(filename)
	if err != nil {
		return err
	}

	// check all polys are triangles
	for _, p := range obj.Polys() {
		if len(p) > 3 {
			return errors.New("meshLoaderObj supports only triangle faces")
		}
	}

	// copy triangles vertices indices from OBJ
	polys := obj.Polys()
	mlo.m_triCount = int32(len(polys))
	mlo.m_tris = make([]int32, mlo.m_triCount*3)
	for i := int32(0); i < mlo.m_triCount*3; i += 3 {
		mlo.m_tris[i] = polys[i/3][0]
		mlo.m_tris[i+1] = polys[i/3][1]
		mlo.m_tris[i+2] = polys[i/3][2]
	}

	// copy vertices indices from OBJ,
	// multiplying them by the scale factor
	verts := obj.Verts()
	mlo.m_vertCount = int32(len(verts)) * 3
	fmt.Println("len(verts)", len(verts))
	mlo.m_verts = make([]float32, mlo.m_vertCount)
	for i := int32(0); i < mlo.m_vertCount; i += 3 {
		mlo.m_verts[i] = float32(verts[i/3][0]) * mlo.m_scale
		mlo.m_verts[i+1] = float32(verts[i/3][1]) * mlo.m_scale
		mlo.m_verts[i+2] = float32(verts[i/3][2]) * mlo.m_scale
	}

	// Calculate normals.
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

	fmt.Println("obj.AABB()", obj.AABB())
	mlo.m_filename = filename
	return nil
}
