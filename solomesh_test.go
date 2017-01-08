package recast

import "testing"

func TestCreateNavMesh(t *testing.T) {
	var meshName string

	meshName = "../testdata/wallfloors.obj"
	// meshName = "../testdata/dungeon.obj"
	// meshName = "../testdata/nav_test.obj"
	soloMesh := NewSoloMesh()
	soloMesh.Load(meshName)
	_, ok := soloMesh.Build()
	if !ok {
		t.Fatalf("solomesh.Build failed")
	}
	t.Logf("solomesh.Build success")
}
