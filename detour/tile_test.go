package detour

import (
	"testing"

	"github.com/aurelien-rainone/gogeo/f32/d3"
)

func TestFindNearestPolyInTile(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		pt   d3.Vec3   // point
		ext  d3.Vec3   // search extents
		want PolyRef // wanted poly ref
	}{
		{
			d3.Vec3{5, 0, 10},
			d3.Vec3{0, 1, 0},
			0x440004,
		},
		{
			d3.Vec3{50, 0, 30},
			d3.Vec3{1, 0, 1},
			0x620000,
		},
	}

	mesh, err = loadTestNavMesh("navmesh.bin")
	checkt(t, err)

	for _, tt := range pathTests {

		// calc tile location
		tx, ty := mesh.CalcTileLoc(tt.pt)
		tile := mesh.TileAt(tx, ty, 0)
		if tile == nil {
			t.Errorf("couldn't retrieve tile at point %v", tt.pt)
		}

		nearestPt := d3.NewVec3()
		got := mesh.FindNearestPolyInTile(tile, tt.pt, tt.ext, nearestPt)
		if got != tt.want {
			t.Errorf("got polyref 0x%x for pt:%v ext:%v, want 0x%x", got, tt.pt, tt.ext, tt.want)
		}
	}
}
