package detour

import (
	"os"
	"path/filepath"
	"reflect"
	"testing"
)

func checkt(t *testing.T, err error) {
	if err != nil {
		t.Fatalf("fail with error: %v", err)
	}
}

func loadTestNavMesh(fname string) (*DtNavMesh, error) {
	var (
		f   *os.File
		err error
	)

	f, err = os.Open(filepath.Join("testdata", fname))
	if err != nil {
		return nil, err
	}
	defer f.Close()
	return Decode(f)
}

func TestFindPath(t *testing.T) {
	var (
		mesh *DtNavMesh
		err  error
	)

	pathTests := []struct {
		org, dst [3]float32
		want     []DtPolyRef
	}{
		{
			[3]float32{5, 0, 10},
			[3]float32{50, 0, 30},
			[]DtPolyRef{0x440000, 0x460007, 0x520007, 0x540003, 0x5c0001, 0x5e0000, 0x600000, 0x620000},
		},
	}

	mesh, err = loadTestNavMesh("navmesh.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			orgRef, dstRef DtPolyRef       // refs for org and dst polys
			query          *DtNavMeshQuery // the query instance
			filter         *DtQueryFilter  // filter to use for various queries
			extents        [3]float32      // poly search distance for poly (3 axis)
			nearestPt      [3]float32
			st             DtStatus
			path           []DtPolyRef
		)

		query, st = NewDtNavMeshQuery(mesh, 1000)
		if DtStatusFailed(st) {
			t.Errorf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents = [3]float32{0, 2, 0}

		// create a default query filter
		filter = NewDtQueryFilter()

		// get org polygon reference
		st = query.FindNearestPoly(tt.org[:], extents[:], filter, &orgRef, nearestPt[:])
		if DtStatusFailed(st) {
			t.Errorf("FindNearestPoly failed with 0x%x\n", st)
		} else if orgRef == 0 {
			t.Errorf("org doesn't intersect any polygons")
		}

		if !mesh.IsValidPolyRef(orgRef) {
			t.Errorf("%d is not a valid poly ref", orgRef)
		}
		copy(tt.org[:], nearestPt[:])

		// get dst polygon reference
		st = query.FindNearestPoly(tt.dst[:], extents[:], filter, &dstRef, nearestPt[:])
		if DtStatusFailed(st) {
			t.Errorf("FindNearestPoly failed with 0x%x\n", st)
		} else if dstRef == 0 {
			t.Errorf("dst doesn't intersect any polygons")
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Errorf("%d is not a valid poly ref", dstRef)
		}
		copy(tt.dst[:], nearestPt[:])

		// FindPath
		var (
			pathCount int32
		)
		path = make([]DtPolyRef, 100)
		st = query.FindPath(orgRef, dstRef, tt.org[:], tt.dst[:], filter, &path, &pathCount, 100)
		if DtStatusFailed(st) {
			t.Errorf("query.FindPath failed with 0x%x\n", st)
		}

		if !reflect.DeepEqual(tt.want, path[:pathCount]) {
			t.Errorf("found path is not correct, want %#v, got %#v", tt.want, path[:pathCount])
		}
	}
}
