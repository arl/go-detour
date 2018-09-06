package detour

import (
	"testing"

	"github.com/arl/gogeo/f32/d3"
)

func TestCalcPolyCenter(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	polyTests := []struct {
		ref  PolyRef
		want d3.Vec3
	}{
		{0x440000, d3.Vec3{3.6002522, 0.189468, 10.873747}},
		{0x460007, d3.Vec3{11.460253, 0.189468, 14.758746}},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for _, tt := range polyTests {

		var (
			tile *MeshTile
			poly *Poly
		)
		mesh.TileAndPolyByRef(tt.ref, &tile, &poly)
		got := CalcPolyCenter(poly.Verts[:], int32(poly.VertCount), tile.Verts)
		if !got.Approx(tt.want) {
			t.Errorf("want centroid of poly 0x%x = %v, got %v", tt.ref, tt.want, got)
		}
	}
}

func TestFindNearestPolySpecialCases(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		msg     string  // test description
		pt      d3.Vec3 // point
		ext     d3.Vec3 // search extents
		wantSt  Status  // expected status
		wantRef PolyRef // expected ref (if query succeeded)
	}{
		{
			"search box does not intersect any poly",
			d3.Vec3{-5, 0, 10}, d3.Vec3{1, 1, 1}, Success, 0,
		},
		{
			"unallocated center vector",
			d3.Vec3{}, d3.Vec3{1, 1, 1}, Failure | InvalidParam, 0,
		},
		{
			"unallocated extents vector",
			d3.Vec3{0, 0, 0}, d3.Vec3{}, Failure | InvalidParam, 0,
		},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			q   *NavMeshQuery
			st  Status
			ref PolyRef
			f   QueryFilter
		)

		st, q = NewNavMeshQuery(mesh, 100)
		f = NewStandardQueryFilter()

		st, ref, _ = q.FindNearestPoly(tt.pt, tt.ext, f)
		if st != tt.wantSt {
			t.Errorf("%s, want status 0x%x, got 0x%x", tt.msg, tt.wantSt, st)
		}

		if StatusSucceed(st) && ref != tt.wantRef {
			t.Errorf("%s, want ref 0x%x, got 0x%x", tt.msg, tt.wantRef, ref)
		}
	}
}
