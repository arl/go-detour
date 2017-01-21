package recast

import "testing"

func TestCreateNavMesh(t *testing.T) {
	var meshName string

	meshName = "testdata/cube.obj"
	//meshName = "testdata/wallfloors.obj"
	//meshName = "testdata/dungeon.obj"
	// meshName = "testdata/nav_test.obj"
	soloMesh := NewSoloMesh()
	if !soloMesh.Load(meshName) {
		t.Fatalf("couldn't load mesh %v", meshName)
	}
	navMesh, ok := soloMesh.Build()
	if !ok {
		t.Fatalf("solomesh.Build failed")
	}

	navMesh.SaveToFile("out.bin")
	t.Logf("solomesh.Build success")
}
