package detour

import (
	"os"
	"path/filepath"
	"reflect"
	"testing"

	"github.com/aurelien-rainone/gogeo/f32/d3"
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
		org, dst         d3.Vec3
		wantPath         []DtPolyRef
		wantStraightPath []d3.Vec3
	}{
		{
			d3.Vec3{5, 0, 10},
			d3.Vec3{50, 0, 30},
			[]DtPolyRef{0x440000, 0x460007, 0x520007, 0x540003, 0x5c0001, 0x5e0000, 0x600000, 0x620000},
			[]d3.Vec3{
				d3.NewVec3XYZ(5, 0, 10), d3.NewVec3XYZ(3.900252, 0.189468, 11.998747), d3.NewVec3XYZ(14.700253, 0.189468, 19.198748), d3.NewVec3XYZ(15.900252, 0.189468, 19.198748), d3.NewVec3XYZ(24.3, 0.189468, 28.798748), d3.NewVec3XYZ(31.8, 0.189468, 32.098747), d3.NewVec3XYZ(39.9, 0.189468, 32.098747), d3.NewVec3XYZ(50, 0, 30),
			},
		},
	}

	mesh, err = loadTestNavMesh("navmesh.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			orgRef, dstRef DtPolyRef       // refs for org and dst polys
			query          *DtNavMeshQuery // the query instance
			filter         *DtQueryFilter  // filter to use for various queries
			extents        d3.Vec3         // poly search distance for poly (3 axis)
			nearestPt      d3.Vec3
			st             DtStatus
			path           []DtPolyRef
		)

		query, st = NewDtNavMeshQuery(mesh, 1000)
		if DtStatusFailed(st) {
			t.Errorf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents = d3.NewVec3XYZ(0, 2, 0)

		// create a default query filter
		filter = NewDtQueryFilter()

		// get org polygon reference
		st = query.FindNearestPoly(tt.org, extents, filter, &orgRef, nearestPt)
		if DtStatusFailed(st) {
			t.Errorf("FindNearestPoly failed with 0x%x\n", st)
		} else if orgRef == 0 {
			t.Errorf("org doesn't intersect any polygons")
		}

		if !mesh.IsValidPolyRef(orgRef) {
			t.Errorf("%d is not a valid poly ref", orgRef)
		}
		copy(tt.org, nearestPt)

		// get dst polygon reference
		st = query.FindNearestPoly(tt.dst, extents, filter, &dstRef, nearestPt)
		if DtStatusFailed(st) {
			t.Errorf("FindNearestPoly failed with 0x%x\n", st)
		} else if dstRef == 0 {
			t.Errorf("dst doesn't intersect any polygons")
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Errorf("%d is not a valid poly ref", dstRef)
		}
		copy(tt.dst, nearestPt)

		// FindPath
		var (
			pathCount int32
		)
		path = make([]DtPolyRef, 100)
		st = query.FindPath(orgRef, dstRef, tt.org, tt.dst, filter, &path, &pathCount, 100)
		if DtStatusFailed(st) {
			t.Errorf("query.FindPath failed with 0x%x\n", st)
		}

		if !reflect.DeepEqual(tt.wantPath, path[:pathCount]) {
			t.Errorf("found path is not correct, want %#v, got %#v", tt.wantPath, path[:pathCount])
		}

		// FindStraightPath
		var (
			straightPathCount int32
			straightPath      []d3.Vec3
			straightPathFlags []uint8
			straightPathRefs  []DtPolyRef
			maxStraightPath   int32
		)
		// slices that receive the straight path
		maxStraightPath = 100
		straightPath = make([]d3.Vec3, maxStraightPath)
		for i := range straightPath {
			straightPath[i] = d3.NewVec3()
		}
		straightPathFlags = make([]uint8, maxStraightPath)
		straightPathRefs = make([]DtPolyRef, maxStraightPath)

		st = query.FindStraightPath(tt.org, tt.dst, path, pathCount, straightPath, straightPathFlags, straightPathRefs, &straightPathCount, 100, 0)
		if DtStatusFailed(st) {
			t.Errorf("query.FindStraightPath failed with 0x%x\n", st)
		}

		if (straightPathFlags[0] & DT_STRAIGHTPATH_START) == 0 {
			t.Errorf("straightPath start is not flagged DT_STRAIGHTPATH_START")
		}

		for i := int32(0); i < pathCount; i++ {
			if !straightPath[i].Approx(tt.wantStraightPath[i]) {
				t.Errorf("straightPath[%d] = %v, want %v", i, straightPath[i], tt.wantStraightPath[i])
			}
		}

		if !reflect.DeepEqual(tt.wantStraightPath, straightPath[:pathCount]) {
		}
	}
}
