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
				d3.NewVec3XYZ(5, 0, 10),
				d3.NewVec3XYZ(3.900252, 0.189468, 11.998747),
				d3.NewVec3XYZ(14.700253, 0.189468, 19.198748),
				d3.NewVec3XYZ(15.900252, 0.189468, 19.198748),
				d3.NewVec3XYZ(24.3, 0.189468, 28.798748),
				d3.NewVec3XYZ(31.8, 0.189468, 32.098747),
				d3.NewVec3XYZ(39.9, 0.189468, 32.098747),
				d3.NewVec3XYZ(50, 0, 30),
			},
		},
	}

	mesh, err = loadTestNavMesh("navmesh.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *DtNavMeshQuery // the query instance
			orgRef, dstRef DtPolyRef       // find poly query results
			org, dst       d3.Vec3         // find poly query results
			st             DtStatus        // status flags
			path           []DtPolyRef     // returned path
		)

		st, query = NewDtNavMeshQuery(mesh, 1000)
		if DtStatusFailed(st) {
			t.Errorf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(0, 2, 0)

		// create a default query filter
		filter := NewDtQueryFilter()

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if DtStatusFailed(st) {
			t.Errorf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Errorf("orgRef %d is not a valid poly ref", orgRef)
		}

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if DtStatusFailed(st) {
			t.Errorf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Errorf("dstRef %d is not a valid poly ref", dstRef)
		}

		// FindPath
		var (
			pathCount int32
		)
		path = make([]DtPolyRef, 100)
		st = query.FindPath(orgRef, dstRef, org, dst, filter, &path, &pathCount, 100)
		if DtStatusFailed(st) {
			t.Errorf("query.FindPath failed with 0x%x\n", st)
		}

		if !reflect.DeepEqual(tt.wantPath, path[:pathCount]) {
			t.Errorf("found path is not correct, want %#v, got %#v", tt.wantPath, path[:pathCount])
		}

		// FindStraightPath
		var (
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

		st, _ = query.FindStraightPath(tt.org, tt.dst, path, pathCount, straightPath, straightPathFlags, straightPathRefs, 100, 0)
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
	}
}

func TestFindPathSpecialCases(t *testing.T) {
	var (
		mesh *DtNavMesh
		err  error
	)

	pathTests := []struct {
		msg           string   // test description
		org, dst      d3.Vec3  // path origin and destination points
		wantStatus    DtStatus // expected status
		wantPathCount int32    // expected path count
	}{
		{"org == dst", d3.Vec3{5, 0, 10}, d3.Vec3{5, 0, 10}, DT_SUCCESS, 1},
	}

	mesh, err = loadTestNavMesh("navmesh.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *DtNavMeshQuery // the query instance
			orgRef, dstRef DtPolyRef       // find poly query results
			org, dst       d3.Vec3         // find poly query results
			st             DtStatus        // status flags
			path           []DtPolyRef     // returned path
		)

		st, query = NewDtNavMeshQuery(mesh, 1000)
		if DtStatusFailed(st) {
			t.Errorf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(0, 2, 0)

		// create a default query filter
		filter := NewDtQueryFilter()

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if DtStatusFailed(st) {
			t.Errorf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Errorf("invalid ref (0x%x) for nearest poly of %v, status: 0x%x", orgRef, tt.org, st)
		}

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if DtStatusFailed(st) {
			t.Errorf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Errorf("dstRef %d is not a valid poly ref", dstRef)
		}

		// FindPath
		var pathCount int32

		path = make([]DtPolyRef, 100)
		st = query.FindPath(orgRef, dstRef, org, dst, filter, &path, &pathCount, 100)

		if st != tt.wantStatus {
			t.Errorf("%s, got status 0x%x, want 0x%x", tt.msg, st, tt.wantStatus)
		}
		if pathCount != tt.wantPathCount {
			t.Errorf("%s, got pathCount 0x%x, want 0x%x", tt.msg, pathCount, tt.wantPathCount)
		}
	}
}
