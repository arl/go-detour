package solomesh

import (
	"bytes"
	"io/ioutil"
	"os"
	"testing"

	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

func check(t *testing.T, err error) {
	if err != nil {
		t.Fatal(err)
	}
}

func checkb(b *testing.B, err error) {
	if err != nil {
		b.Fatal(err)
	}
}

func compareFiles(fn1, fn2 string) (bool, error) {
	// per comment, better to not read an entire file into memory
	// this is simply a trivial example.
	var (
		f1, f2 []byte
		err    error
	)
	f1, err = ioutil.ReadFile(fn1)
	if err != nil {
		return false, err
	}

	f2, err = ioutil.ReadFile(fn2)
	if err != nil {
		return false, err
	}

	return bytes.Equal(f1, f2), nil
}

const testDataDir = "../../testdata/sample/solomesh/"
const OBJDir = "../../testdata/obj/"

func testCreateSoloMesh(t *testing.T, objName string) {
	var (
		path, meshBinPath string
		outBin            string
		ctx               *recast.BuildContext

		soloMesh *SoloMesh
		err      error
		ok       bool
	)

	path = OBJDir + objName + ".obj"
	meshBinPath = testDataDir + objName + ".bin"
	outBin = "out.bin"

	ctx = recast.NewBuildContext(true)
	soloMesh = New(ctx)

	r, err := os.Open(path)
	check(t, err)
	defer r.Close()
	if err = soloMesh.LoadGeometry(r); err != nil {
		ctx.DumpLog("")
		t.Fatalf("couldn't load mesh %v", path)
	}
	navMesh, ok := soloMesh.Build()
	if !ok {
		ctx.DumpLog("")
		t.Fatalf("couldn't build navmesh for %v", objName)
	}

	navMesh.SaveToFile(outBin)
	defer os.Remove(outBin)

	t.Logf("%v navmesh successfully built", objName)
	ok, err = compareFiles(outBin, meshBinPath)
	if err != nil {
		t.Fatalf("couldn't compare %v and %v, %v", outBin, meshBinPath, err)
	}
	if !ok {
		t.Fatalf("%v and %v are different", outBin, meshBinPath)
	}
}

func TestCreateDevelerNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "develer")
}

func TestCreateDungeonNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "dungeon")
}

func TestCreateCubeNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "cube")
}

func TestCreateCube5DegreesNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "cube5xdeg")
}

func TestCreateCube45DegreesNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "cube45xdeg")
}

func TestCreateStair2NavMesh(t *testing.T) {
	testCreateSoloMesh(t, "stair2")
}

func TestCreateStair3NavMesh(t *testing.T) {
	testCreateSoloMesh(t, "stair3")
}

func TestCreateHillNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "hill")
}

func TestCreateTestNavMesh(t *testing.T) {
	testCreateSoloMesh(t, "nav_test")
}

func benchmarkCreateNavMesh(b *testing.B, meshName string) {
	path := testDataDir + meshName + ".obj"

	soloMesh := New(recast.NewBuildContext(false))
	r, err := os.Open(path)
	checkb(b, err)
	defer r.Close()
	if err := soloMesh.LoadGeometry(r); err != nil {
		b.Fatalf("couldn't load mesh %v: %v", path, err)
	}

	b.ResetTimer()
	for n := 0; n < b.N; n++ {
		_, ok := soloMesh.Build()
		if !ok {
			b.Fatalf("couldn't build navmesh for %v", path)
		}
	}
}

func BenchmarkCreateDevelerNavMesh(b *testing.B) {
	benchmarkCreateNavMesh(b, "develer")
}

func BenchmarkCreateCubeNavMesh(b *testing.B) {
	benchmarkCreateNavMesh(b, "cube")
}

func BenchmarkPathFindSoloMesh(b *testing.B) {
	benchs := []struct {
		xstart, ystart, zstart float32
		xend, yend, zend       float32
		incFlags, excFlags     uint16
	}{
		{18.138550, -2.370003, -21.319118, -19.206181, -2.369133, 24.802742, 0x3, 0x0},
		{18.252758, -2.368240, -7.000238, -19.206181, -2.369133, 24.802742, 0x3, 0x0},
		{18.252758, -2.368240, -7.000238, -22.759071, -2.369453, 2.003946, 0x3, 0x0},
		{18.252758, -2.368240, -7.000238, -24.483898, -2.369728, -6.778278, 0x3, 0x0},
		{18.252758, -2.368240, -7.000238, -24.068850, -2.370285, -18.879251, 0x3, 0x0},
		{18.252758, -2.368240, -7.000238, 12.124170, -2.369637, -21.222471, 0x3, 0x0},
		{10.830146, -2.366791, 19.002508, 12.124170, -2.369637, -21.222471, 0x3, 0x0},
		{10.830146, -2.366791, 19.002508, -7.146484, -2.368736, -16.031403, 0x3, 0x0},
		{10.830146, -2.366791, 19.002508, -21.615391, -2.368706, -3.264029, 0x3, 0x0},
		{10.830146, -2.366791, 19.002508, -22.651268, -2.369354, 1.053217, 0x3, 0x0},
		{10.830146, -2.366791, 19.002508, 19.181122, -2.368134, 3.011776, 0x3, 0x0},
		{10.830146, -2.366791, 19.002508, 19.041592, -2.368713, -7.404587, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, 19.041592, -2.368713, -7.404587, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, 21.846087, -2.368568, 17.918859, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, 0.967449, -2.368439, 25.767756, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, -17.518076, -2.368477, 26.569633, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, -22.141787, -2.369209, 2.440046, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, -23.296972, -2.369797, -17.411043, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, -1.564062, -2.369926, -20.452827, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, 16.905643, -2.370193, -21.811655, 0x3, 0x0},
		{6.054083, -2.365402, 3.330421, 19.289761, -2.368813, -6.954918, 0x3, 0x0},
	}

	var (
		err error
	)

	objName := "nav_test"
	path := testDataDir + objName + ".obj"

	ctx := recast.NewBuildContext(false)
	soloMesh := New(ctx)

	r, err := os.Open(path)
	checkb(b, err)
	defer r.Close()
	if err = soloMesh.LoadGeometry(r); err != nil {
		b.Fatalf("couldn't load mesh '%v': %s", path, err)
	}
	navMesh, ok := soloMesh.Build()
	if !ok {
		b.Fatalf("couldn't build navmesh for %v", objName)
	}

	st, query := detour.NewNavMeshQuery(navMesh, 2048)
	if detour.StatusFailed(st) {
		b.Fatalf("creation of navmesh query failed: %s", st)
	}

	const maxPolys = 256
	var (
		polys       [maxPolys]detour.PolyRef
		straight    [maxPolys]d3.Vec3
		polyPickExt = d3.NewVec3XYZ(2, 4, 2)
		spos, epos  d3.Vec3
	)

	spos, epos = d3.NewVec3(), d3.NewVec3()

	b.ResetTimer()
	for n := 0; n < b.N; n++ {
		for _, bb := range benchs {
			spos[0], spos[1], spos[2] = bb.xstart, bb.ystart, bb.zstart
			epos[0], epos[1], epos[2] = bb.xend, bb.yend, bb.zend

			filter := detour.NewStandardQueryFilter()
			filter.SetIncludeFlags(bb.incFlags)
			filter.SetExcludeFlags(bb.excFlags)

			var startRef, endRef detour.PolyRef
			_, startRef, _ = query.FindNearestPoly(spos, polyPickExt, filter)
			_, endRef, _ = query.FindNearestPoly(epos, polyPickExt, filter)

			// find path
			npolys, _ := query.FindPath(startRef, endRef, spos, epos, filter, polys[:])

			// find straight path
			if npolys != 0 {
				query.FindStraightPath(spos, epos, polys[:], straight[:], nil, nil, 0)
			}
		}
	}
}

func TestRaycastSoloMesh(t *testing.T) {
	type want struct {
		t                float32
		hitx, hity, hitz float32
	}

	tests := []struct {
		xstart, ystart, zstart float32
		xend, yend, zend       float32
		incFlags, excFlags     uint16
		want                   want
	}{
		{40.389084, 7.797607, 17.144299, 43.953857, 6.223053, 10.389969, 0xffef, 0x0,
			want{0.287881, 41.415318, 7.344322, 15.199852}},
		{40.389084, 7.797607, 17.144299, 45.056454, 7.418980, 12.680744, 0xffef, 0x0,
			want{0.435627, 42.422318, 7.632667, 15.199852}},
		{40.389084, 7.797607, 17.144299, 45.965542, 7.797607, 14.355331, 0xffef, 0x0,
			want{math32.MaxFloat32, 45.965542, 7.797607, 14.355331}},
		{0.631622, 12.705303, 2.767708, 3.878273, 11.266037, -0.112907, 0xffef, 0x0,
			want{math32.MaxFloat32, 3.878273, 11.266037, -0.112907}},
	}

	var (
		err error
	)

	objName := "nav_test"
	path := OBJDir + objName + ".obj"

	ctx := recast.NewBuildContext(false)
	soloMesh := New(ctx)
	r, err := os.Open(path)
	check(t, err)
	defer r.Close()
	if err = soloMesh.LoadGeometry(r); err != nil {
		t.Fatalf("couldn't load mesh '%v': %s", path, err)
	}
	navMesh, ok := soloMesh.Build()
	if !ok {
		t.Fatalf("couldn't build navmesh for %v", objName)
	}

	st, query := detour.NewNavMeshQuery(navMesh, 2048)
	if detour.StatusFailed(st) {
		t.Fatalf("creation of navmesh query failed: %s", st)
	}

	const maxPolys = 256
	var (
		polyPickExt        = d3.NewVec3XYZ(2, 4, 2)
		spos, epos, hitPos d3.Vec3
	)

	spos, epos, hitPos = d3.NewVec3(), d3.NewVec3(), d3.NewVec3()

	for _, tt := range tests {
		spos[0], spos[1], spos[2] = tt.xstart, tt.ystart, tt.zstart
		epos[0], epos[1], epos[2] = tt.xend, tt.yend, tt.zend

		filter := detour.NewStandardQueryFilter()
		filter.SetIncludeFlags(tt.incFlags)
		filter.SetExcludeFlags(tt.excFlags)

		var startRef detour.PolyRef
		_, startRef, _ = query.FindNearestPoly(spos, polyPickExt, filter)

		hit, st := query.Raycast(startRef, spos, epos, filter, 0, 0)
		if detour.StatusFailed(st) {
			t.Fatalf("Raycast (s:%f %f %f|e:%f %f %f |flags:%x %x) failed with status %s",
				tt.xstart, tt.ystart, tt.zstart, tt.xend, tt.yend, tt.zend, tt.incFlags, tt.excFlags, st)
		}

		if !math32.Approx(hit.T, tt.want.t) {
			t.Fatalf("Raycast (s:%f %f %f|e:%f %f %f |flags:%x %x) got t = %f want %f",
				tt.xstart, tt.ystart, tt.zstart, tt.xend, tt.yend, tt.zend, tt.incFlags, tt.excFlags, hit.T, tt.want.t)
		}

		if hit.T > 1.0 {
			// No hit
			hitPos = d3.NewVec3From(epos)
		} else {
			// Hit
			d3.Vec3Lerp(hitPos, spos, epos, hit.T)
		}

		wantHitPos := d3.NewVec3XYZ(tt.want.hitx, tt.want.hity, tt.want.hitz)
		if !hitPos.Approx(wantHitPos) {
			t.Fatalf("Raycast (s:%f %f %f|e:%f %f %f |flags:%x %x) got hit = %v want %v",
				tt.xstart, tt.ystart, tt.zstart, tt.xend, tt.yend, tt.zend, tt.incFlags, tt.excFlags, hitPos, wantHitPos)
		}
	}
}
