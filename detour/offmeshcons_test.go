package detour

import (
	"fmt"
	"log"
	"reflect"
	"testing"

	"github.com/aurelien-rainone/gogeo/f32/d3"
)

// FIXME: this test is disabled for now
func _TestOffMeshConnections(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		org, dst           d3.Vec3
		incFlags, excFlags uint16 // query filter include/exclude flags
		wantPath           []PolyRef
		wantStraightPath   []d3.Vec3
	}{
		{
			// latest result:
			/*
			   ps  -19.460140 4.234787 -4.727699  -1.402759 -0.000092 -2.314920  0xffef 0x0
			           findPath polys[0] ref:0x600029
			           findPath polys[1] ref:0x60003d
			           findPath polys[2] ref:0x600034
			           ps[0] pt{-19.460140,4.234787,-4.727699} ref:0x600029 flags0x1
			           ps[1] pt{-17.767578,2.514686,-0.300116} ref:0x60003d flags0x4
			           ps[2] pt{-6.194458,0.197294,1.019781} ref:0x600034 flags0x0
			           ps[3] pt{-1.402759,-0.000092,-2.314920} ref:0x0 flags0x2
			*/

			// oldest result:
			//offmeshconnections (omc) are currently only allowed inside a same tile or its direct neighbours :-(

			// with the new and already commited navmesh 'offmeshcons.bin', we
			// got this result on RecastDemo gui:
			//ps  -17.323172 3.074387 -3.366402  -4.377790 -0.000053 -1.633406  0xffef 0x0
			//p[0] pt{-17.323172,3.074387,-3.366402} flags0x1
			//p[1] pt{-17.767578,2.514686,-0.300116} flags0x4
			//p[2] pt{-6.194458,0.197294,1.019781} flags0x0
			//p[3] pt{-4.377790,-0.000053,-1.633406} flags0x2

			//d3.Vec3{13.512327, 9.998181, -39.930054},
			//d3.Vec3{20.308548, 11.554298, -57.635326},
			//d3.Vec3{-17.323172, 3.074387, -3.366402},
			//d3.Vec3{-4.377790, -0.000053, -1.633406},
			d3.Vec3{-19.460140, 4.234787, -4.727699},
			d3.Vec3{-1.402759, -0.000092, -2.314920},

			0xffef, // all poly but the disabled ones
			0x0,
			//[]PolyRef{0x600029, 0x600023, 0x600025, 0x60001d, 0x60001c, 0x60001e, 0x600020, 0x60001f, 0x600016, 0x600012, 0x60000b, 0x600018, 0x600015, 0x600006, 0x600005, 0x600007, 0x600008, 0x600034},
			[]PolyRef{0x600029, 0x60003d, 0x600034},
			[]d3.Vec3{

				d3.NewVec3XYZ(-19.460140, 4.234787, -4.727699),
				d3.NewVec3XYZ(-17.767578, 2.514686, -0.300116),
				d3.NewVec3XYZ(-6.194458, 0.197294, 1.019781),
				d3.NewVec3XYZ(-1.402759, -0.000092, -2.314920),
			},

			/*         []d3.Vec3{*/
			//d3.NewVec3XYZ(5, 0, 10),
			//d3.NewVec3XYZ(3.900252, 0.189468, 11.998747),
			//d3.NewVec3XYZ(14.700253, 0.189468, 19.198748),
			//d3.NewVec3XYZ(15.900252, 0.189468, 19.198748),
			//d3.NewVec3XYZ(24.3, 0.189468, 28.798748),
			//d3.NewVec3XYZ(31.8, 0.189468, 32.098747),
			//d3.NewVec3XYZ(39.9, 0.189468, 32.098747),
			//d3.NewVec3XYZ(50, 0, 30),
			/*},*/
		},
	}

	mesh, err = loadTestNavMesh("offmeshcons.bin")
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
			t.Error("query creation failed:", st)
		}
		// define the extents vector for the nearest polygon query
		extents := d3.NewVec3XYZ(2, 2, 2)

		// create a default query filter
		filter = NewStandardQueryFilter()
		filter.SetIncludeFlags(tt.incFlags)
		filter.SetExcludeFlags(tt.excFlags)

		// These are just sample areas to use consistent values across the samples.
		// The use should specify these base on his needs.
		const (
			samplePolyAreaGround int32 = iota
			samplePolyAreaWater
			samplePolyAreaRoad
			samplePolyAreaDoor
			samplePolyAreaGrass
			samplePolyAreaJump
		)

		const (
			samplePolyFlagsWalk     int32 = 0x01   // Ability to walk (ground, grass, road)
			samplePolyFlagsSwim           = 0x02   // Ability to swim (water).
			samplePolyFlagsDoor           = 0x04   // Ability to move through doors.
			samplePolyFlagsJump           = 0x08   // Ability to jump.
			samplePolyFlagsDisabled       = 0x10   // Disabled polygon
			samplePolyFlagsAll            = 0xffff // All abilities.
		)

		// Change costs.
		filter.SetAreaCost(samplePolyAreaGround, 1.0)
		filter.SetAreaCost(samplePolyAreaWater, 10.0)
		filter.SetAreaCost(samplePolyAreaRoad, 1.0)
		filter.SetAreaCost(samplePolyAreaDoor, 1.0)
		filter.SetAreaCost(samplePolyAreaGrass, 2.0)
		filter.SetAreaCost(samplePolyAreaJump, 1.5)

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if StatusFailed(st) {
			t.Fatal("couldn't find nearest poly of", tt.org, ":", st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Fatal("orgRef", orgRef, "is not a valid poly ref")
		}

		t.Logf("org poly reference:0x%x\n", orgRef)

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if StatusFailed(st) {
			t.Fatal("couldn't find nearest poly of", tt.dst, ":", st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Fatal("dstRef", dstRef, "is not a valid poly ref")
		}

		t.Logf("dst poly reference:0x%x\n", dstRef)

		// FindPath
		var (
			pathCount int
		)
		path = make([]PolyRef, 100)
		pathCount, st = query.FindPath(orgRef, dstRef, org, dst, filter, path)
		if StatusFailed(st) {
			t.Fatal("query.FindPath failed:", st)
		}

		if !reflect.DeepEqual(tt.wantPath, path[:pathCount]) {
			t.Fatalf("found path is not correct, want %#v, got %#v", tt.wantPath, path[:pathCount])
		}

		// FindStraightPath
		var (
			straightPath      []d3.Vec3
			straightPathFlags []uint8
			straightPathRefs  []PolyRef
			straightPathCount int32
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

		st, straightPathCount = query.FindStraightPath(tt.org, tt.dst, path, int32(pathCount), straightPath, straightPathFlags, straightPathRefs, 100, 0)
		if StatusFailed(st) {
			t.Fatal("query.FindStraightPath failed:", st)
		}

		if (straightPathFlags[0] & StraightPathStart) == 0 {
			t.Fatal("straightPath start is not flagged StraightPathStart")
		}

		fmt.Println("StraightPathCount == ", straightPathCount)
		if (straightPathFlags[straightPathCount-1] & StraightPathEnd) == 0 {
			t.Fatal("straightPath end is not flagged StraightPathEnd")
		}

		if int(straightPathCount) != len(tt.wantStraightPath) {
			t.Fatalf("found path and wanted path do not have the same length (%d != %d)", straightPathCount, len(tt.wantStraightPath))
		}
		for i := int32(0); i < straightPathCount; i++ {
			log.Printf("straightPath[%d].Flags = 0x%x\n", i, straightPathFlags[i])
		}
		for i := 0; i < pathCount; i++ {
			if !straightPath[i].Approx(tt.wantStraightPath[i]) {
				t.Errorf("straightPath[%d] = %v, want %v", i, straightPath[i], tt.wantStraightPath[i])
			}
		}
	}
}
