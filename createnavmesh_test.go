package detour

import "testing"

func TestCreateNavMesh(t *testing.T) {
	var meshName string

	meshName = "testdata/wallfloors.obj"
	soloMesh := NewSoloMesh()
	soloMesh.Load(meshName)
	soloMesh.Build()
}
