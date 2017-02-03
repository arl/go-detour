package recast

import (
	"bytes"
	"io/ioutil"
	"testing"

	"github.com/aurelien-rainone/go-detour/recast"
)

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

func testCreateSoloMesh(t *testing.T, objName string) {
	var (
		path, meshBinPath string
		outBin            string
		ctx               *recast.BuildContext

		soloMesh *SoloMesh
		err      error
		ok       bool
	)

	path = "testdata/" + objName + ".obj"
	meshBinPath = "testdata/" + objName + ".org.bin"
	outBin = "out.bin"

	ctx = recast.NewBuildContext(true)
	soloMesh = NewSoloMesh(ctx)
	if err = soloMesh.LoadGeometry(path); err != nil {
		ctx.DumpLog("")
		t.Fatalf("couldn't load mesh %v", path)
	}
	navMesh, ok := soloMesh.Build()
	if !ok {
		ctx.DumpLog("")
		t.Fatalf("couldn't build navmesh for %v", objName)
	}

	navMesh.SaveToFile(outBin)
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
	path := "testdata/" + meshName + ".obj"

	soloMesh := NewSoloMesh(recast.NewBuildContext(false))
	if err := soloMesh.LoadGeometry(path); err != nil {
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
