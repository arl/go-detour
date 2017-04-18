package detour

import (
	"os"
	"path/filepath"
	"reflect"
	"testing"

	"fmt"

	"github.com/aurelien-rainone/go-detour/sample"
	"github.com/aurelien-rainone/gogeo/f32/d3"
)

func checkt(t *testing.T, err error) {
	if err != nil {
		t.Fatalf("fail with error: %v", err)
	}
}

func loadTestNavMesh(fname string) (*NavMesh, error) {
	var (
		f   *os.File
		err error
	)

	f, err = os.Open(filepath.Join("..", "testdata", fname))
	if err != nil {
		return nil, err
	}
	defer f.Close()
	return Decode(f)
}

func TestFindPathFindStraightPath(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		org, dst         d3.Vec3
		wantPath         []PolyRef
		wantStraightPath []d3.Vec3
	}{
		{
			d3.Vec3{37.298489, -1.776901, 11.652311},
			d3.Vec3{42.457218, 7.797607, 17.778244},
			[]PolyRef{
				0x18c,
				0x18a,
				0x156,
				0x157,
				0x159,
				0x158,
				0x160,
				0x15f,
				0x174,
				0x175,
				0x176,
				0x19d,
				0x19f},
			[]d3.Vec3{
				d3.NewVec3XYZ(37.298489, -1.776901, 11.652311),
				d3.NewVec3XYZ(35.310688, -0.469517, 5.899849),
				d3.NewVec3XYZ(34.410686, -0.669517, -1.600151),
				d3.NewVec3XYZ(35.610683, -0.069517, -1.900150),
				d3.NewVec3XYZ(36.510685, 0.730483, -0.400150),
				d3.NewVec3XYZ(41.010685, 7.930483, 15.199852),
				d3.NewVec3XYZ(42.457218, 7.797607, 17.778244),
			},
		},
	}

	mesh, err = loadTestNavMesh("mesh1.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *NavMeshQuery // the query instance
			filter         QueryFilter   // the query filter
			orgRef, dstRef PolyRef       // find poly query results
			org, dst       d3.Vec3       // find poly query results
			st             Status        // status flags
			path           []PolyRef     // returned path
		)

		st, query = NewNavMeshQuery(mesh, 1000)
		if StatusFailed(st) {
			t.Fatalf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(2, 4, 2)

		// create a default query filter
		filter = NewStandardQueryFilter()

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if StatusFailed(st) {
			t.Fatalf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Fatalf("orgRef %d is not a valid poly ref", orgRef)
		}

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if StatusFailed(st) {
			t.Fatalf("couldn't find nearest poly of %v, status: 0x%x\n", tt.dst, st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Fatalf("dstRef %d is not a valid poly ref", dstRef)
		}

		// FindPath
		var (
			pathCount int
		)
		path = make([]PolyRef, 100)
		pathCount, st = query.FindPath(orgRef, dstRef, org, dst, filter, path)
		if StatusFailed(st) {
			t.Fatalf("query.FindPath failed with 0x%x\n", st)
		}

		if !reflect.DeepEqual(tt.wantPath, path[:pathCount]) {
			t.Fatalf("found path is not correct, want %#v, got %#v", tt.wantPath, path[:pathCount])
		}

		// FindStraightPath
		var (
			straightPath      []d3.Vec3
			straightPathFlags []uint8
			straightPathRefs  []PolyRef
			straightPathCount int
			maxStraightPath   int32
		)
		// slices that receive the straight path
		maxStraightPath = 100
		straightPath = make([]d3.Vec3, maxStraightPath)
		for i := range straightPath {
			straightPath[i] = d3.NewVec3()
		}
		straightPathFlags = make([]uint8, maxStraightPath)
		straightPathRefs = make([]PolyRef, maxStraightPath)

		straightPathCount, st = query.FindStraightPath(tt.org, tt.dst, path[:pathCount], straightPath, straightPathFlags, straightPathRefs, 0)
		if StatusFailed(st) {
			t.Errorf("query.FindStraightPath failed with 0x%x\n", st)
		}

		if (straightPathFlags[0] & StraightPathStart) == 0 {
			t.Errorf("straightPath start is not flagged StraightPathStart")
		}

		if (straightPathFlags[straightPathCount-1] & StraightPathEnd) == 0 {
			t.Errorf("straightPath end is not flagged StraightPathEnd")
		}

		for i := 0; i < straightPathCount; i++ {
			if !straightPath[i].Approx(tt.wantStraightPath[i]) {
				t.Errorf("straightPath[%d] = %v, want %v", i, straightPath[i], tt.wantStraightPath[i])
			}
		}
	}
}

func TestFindPathSpecialCases(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		msg           string  // test description
		org, dst      d3.Vec3 // path origin and destination points
		wantStatus    Status  // expected status
		wantPathCount int     // expected path count
	}{
		{"org == dst", d3.Vec3{5, 0, 10}, d3.Vec3{5, 0, 10}, Success, 1},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *NavMeshQuery // the query instance
			filter         QueryFilter   // the query filter
			orgRef, dstRef PolyRef       // find poly query results
			org, dst       d3.Vec3       // find poly query results
			st             Status        // status flags
			path           []PolyRef     // returned path
		)

		st, query = NewNavMeshQuery(mesh, 1000)
		if StatusFailed(st) {
			t.Errorf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(0, 2, 0)

		// create a default query filter
		filter = NewStandardQueryFilter()

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if StatusFailed(st) {
			t.Errorf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Errorf("invalid ref (0x%x) for nearest poly of %v, status: 0x%x", orgRef, tt.org, st)
		}

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if StatusFailed(st) {
			t.Errorf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Errorf("dstRef %d is not a valid poly ref", dstRef)
		}

		// FindPath
		var pathCount int

		path = make([]PolyRef, 100)
		pathCount, st = query.FindPath(orgRef, dstRef, org, dst, filter, path)

		if st != tt.wantStatus {
			t.Errorf("%s, got status 0x%x, want 0x%x", tt.msg, st, tt.wantStatus)
		}
		if pathCount != tt.wantPathCount {
			t.Errorf("%s, got pathCount 0x%x, want 0x%x", tt.msg, pathCount, tt.wantPathCount)
		}
	}
}

func TestFindPathFindSlicedPath(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		org, dst         d3.Vec3
		wantPath         []PolyRef
		wantStraightPath []d3.Vec3
	}{
		{
			d3.Vec3{37.298489, -1.776901, 11.652311},
			d3.Vec3{42.457218, 7.797607, 17.778244},
			[]PolyRef{
				0x18c,
				0x18a,
				0x156,
				0x157,
				0x159,
				0x158,
				0x160,
				0x15f,
				0x174,
				0x175,
				0x176,
				0x19d,
				0x19f},
			[]d3.Vec3{
				d3.NewVec3XYZ(37.298489, -1.776901, 11.652311),
				d3.NewVec3XYZ(35.310688, -0.469517, 5.899849),
				d3.NewVec3XYZ(34.410686, -0.669517, -1.600151),
				d3.NewVec3XYZ(35.610683, -0.069517, -1.900150),
				d3.NewVec3XYZ(36.510685, 0.730483, -0.400150),
				d3.NewVec3XYZ(41.010685, 7.930483, 15.199852),
				d3.NewVec3XYZ(42.457218, 7.797607, 17.778244),
			},
		},
	}

	mesh, err = loadTestNavMesh("mesh1.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *NavMeshQuery // the query instance
			filter         QueryFilter   // the query filter
			orgRef, dstRef PolyRef       // find poly query results
			org, dst       d3.Vec3       // find poly query results
			st             Status        // status flags
			path           []PolyRef     // returned path
		)

		st, query = NewNavMeshQuery(mesh, 1000)
		if StatusFailed(st) {
			t.Fatalf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(2, 4, 2)

		// create a default query filter
		filter = NewStandardQueryFilter()

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if StatusFailed(st) {
			t.Fatalf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Fatalf("orgRef %d is not a valid poly ref", orgRef)
		}

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if StatusFailed(st) {
			t.Fatalf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Fatalf("dstRef %d is not a valid poly ref", dstRef)
		}

		// InitSlicedFindPath
		st = query.InitSlicedFindPath(orgRef, dstRef, org, dst, filter, 0)
		if StatusFailed(st) {
			t.Fatalf("query.InitSlicedFindPath failed with 0x%x\n", st)
		}

		// UpdateSlicedFindPath
		var doneIters int
		st = query.UpdateSlicedFindPath(2, &doneIters)
		if StatusFailed(st) {
			t.Fatalf("query.UpdateSlicedFindPath failed with 0x%x\n", st)
		}

		// FinalizeSlicedFindPath
		var pathCount int
		path = make([]PolyRef, 100)
		pathCount, st = query.FinalizeSlicedFindPath(path, 100)
		if StatusFailed(st) {
			t.Fatalf("query.FinalizeSlicedFindPath failed with 0x%x\n", st)
		}

		fmt.Printf("sliced path, with pathCount %d: %v\n", pathCount, path[:pathCount])
	}
}

func TestFindPathDeveler(t *testing.T) {
	loadTestNavMesh := func(fname string) (*NavMesh, error) {
		var (
			f   *os.File
			err error
		)

		f, err = os.Open(filepath.Join("..", fname))
		if err != nil {
			return nil, err
		}
		defer f.Close()
		return Decode(f)
	}
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		org, dst         d3.Vec3
		wantPath         []PolyRef
		wantStraightPath []d3.Vec3
	}{
		{
			d3.Vec3{49.50, 0, 30.8},
			d3.Vec3{0.1, 0, 0.1},
			// d3.Vec3{2.84, 0, 1.93},
			[]PolyRef{
				0x18c,
				0x18a,
				0x156,
				0x157,
				0x159,
				0x158,
				0x160,
				0x15f,
				0x174,
				0x175,
				0x176,
				0x19d,
				0x19f},
			[]d3.Vec3{
				d3.NewVec3XYZ(37.298489, -1.776901, 11.652311),
				d3.NewVec3XYZ(35.310688, -0.469517, 5.899849),
				d3.NewVec3XYZ(34.410686, -0.669517, -1.600151),
				d3.NewVec3XYZ(35.610683, -0.069517, -1.900150),
				d3.NewVec3XYZ(36.510685, 0.730483, -0.400150),
				d3.NewVec3XYZ(41.010685, 7.930483, 15.199852),
				d3.NewVec3XYZ(42.457218, 7.797607, 17.778244),
			},
		},
	}

	mesh, err = loadTestNavMesh("develer.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *NavMeshQuery        // the query instance
			filter         *StandardQueryFilter // the query filter
			orgRef, dstRef PolyRef              // find poly query results
			org, dst       d3.Vec3              // find poly query results
			st             Status               // status flags
			path           []PolyRef            // returned path
		)

		st, query = NewNavMeshQuery(mesh, 1000)
		if StatusFailed(st) {
			t.Fatalf("query creation failed with status 0x%x\n", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(0, 1, 0)

		// create a default query filter
		filter = NewStandardQueryFilter()
		filter.SetIncludeFlags(sample.PolyFlagsAll ^ sample.PolyFlagsDisabled)
		filter.SetExcludeFlags(0)

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		fmt.Printf("FindNearestPoly(pt:%v, ext:%v) => %v\n", tt.org, extents, org)
		if StatusFailed(st) {
			t.Fatalf("couldn't find nearest poly of %v, status: 0x%x\n", tt.org, st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Fatalf("orgRef %d is not a valid poly ref", orgRef)
		}

		var closest = d3.NewVec3()
		var posOverPoly bool
		st = query.ClosestPointOnPoly(orgRef, tt.org, closest, &posOverPoly)
		if StatusFailed(st) {
			t.Fatalf("error calling closestPointOnPoly, status: 0x%x\n", st)
		}
		fmt.Printf("ClosestPointOnPoly result -> closest:%v posOverPoly:%v\n", closest, posOverPoly)

		st = query.ClosestPointOnPolyBoundary(orgRef, tt.org, closest)
		if StatusFailed(st) {
			t.Fatalf("error calling closestPointOnPolyBoundary, status: 0x%x\n", st)
		}
		fmt.Printf("ClosestPointOnPolyBoundary result -> closest:%v\n", closest)

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		fmt.Printf("FindNearestPoly(pt:%v, ext:%v) => %v\n", tt.dst, extents, dst)

		if StatusFailed(st) {
			t.Fatalf("couldn't find nearest poly of %v, status: 0x%x\n", tt.dst, st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Fatalf("dstRef %d is not a valid poly ref", dstRef)
		}
		// check that the desired target really lies on the returned poly
		st = query.ClosestPointOnPoly(dstRef, tt.dst, closest, &posOverPoly)
		if StatusFailed(st) {
			t.Fatalf("error calling closestPointOnPoly, status: 0x%x\n", st)
		}
		if !posOverPoly {
			t.Fatalf("can't find a path to %v because it's not in any polygon\n", tt.dst)
		}
		fmt.Printf("ClosestPointOnPoly result -> closest:%v posOverPoly:%v\n", closest, posOverPoly)

		st = query.ClosestPointOnPolyBoundary(dstRef, tt.dst, closest)
		if StatusFailed(st) {
			t.Fatalf("error calling closestPointOnPolyBoundary, status: 0x%x\n", st)
		}
		fmt.Printf("ClosestPointOnPolyBoundary result -> closest:%v\n", closest)

		// FindPath
		var (
			pathCount int
		)
		path = make([]PolyRef, 100)
		pathCount, st = query.FindPath(orgRef, dstRef, org, dst, filter, path)
		if StatusFailed(st) {
			t.Fatalf("query.FindPath failed with 0x%x\n", st)
		}

		// check that the dstRef

		// if !reflect.DeepEqual(tt.wantPath, path[:pathCount]) {
		// 	t.Fatalf("found path is not correct, want %#v, got %#v", tt.wantPath, path[:pathCount])
		// }

		// FindStraightPath
		var (
			straightPath      []d3.Vec3
			straightPathFlags []uint8
			straightPathRefs  []PolyRef
			straightPathCount int
			maxStraightPath   int32
		)
		// slices that receive the straight path
		maxStraightPath = 100
		straightPath = make([]d3.Vec3, maxStraightPath)
		for i := range straightPath {
			straightPath[i] = d3.NewVec3()
		}
		straightPathFlags = make([]uint8, maxStraightPath)
		straightPathRefs = make([]PolyRef, maxStraightPath)

		straightPathCount, st = query.FindStraightPath(tt.org, tt.dst, path[:pathCount], straightPath, straightPathFlags, straightPathRefs, 0)
		if StatusFailed(st) {
			t.Errorf("query.FindStraightPath failed with 0x%x\n", st)
		}
		fmt.Println(straightPath[:straightPathCount])

		if (straightPathFlags[0] & StraightPathStart) == 0 {
			t.Errorf("straightPath start is not flagged StraightPathStart")
		}

		if (straightPathFlags[straightPathCount-1] & StraightPathEnd) == 0 {
			t.Errorf("straightPath end is not flagged StraightPathEnd")
		}

		for i := 0; i < straightPathCount; i++ {
			if !straightPath[i].Approx(tt.wantStraightPath[i]) {
				t.Errorf("straightPath[%d] = %v, want %v", i, straightPath[i], tt.wantStraightPath[i])
			}
		}
	}
}
