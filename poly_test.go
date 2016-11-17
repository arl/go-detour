package detour

import (
	"testing"

	"github.com/aurelien-rainone/gogeo/f32/d3"
)

func TestCalcPolyCenter(t *testing.T) {
	var (
		mesh *DtNavMesh
		err  error
	)

	polyTests := []struct {
		ref  DtPolyRef
		want d3.Vec3
	}{
		{0x440000, d3.Vec3{3.6002522, 0.189468, 10.873747}},
		{0x460007, d3.Vec3{11.460253, 0.189468, 14.758746}},
	}

	mesh, err = loadTestNavMesh("navmesh.bin")
	checkt(t, err)

	for _, tt := range polyTests {

		var (
			tile *DtMeshTile
			poly *DtPoly
		)
		mesh.TileAndPolyByRef(tt.ref, &tile, &poly)
		got := DtCalcPolyCenter(poly.Verts[:], int32(poly.VertCount), tile.Verts)
		if !got.Approx(tt.want) {
			t.Errorf("want centroid of poly 0x%x = %v, got %v", tt.ref, tt.want, got)
		}
	}
}
