package detour

import "github.com/aurelien-rainone/go-detour/recast"

type SoloMesh struct {
	ctx      *recast.Context
	buildCtx recast.BuildContext
	geom     InputGeom
	meshName string
	//cfg      rcConfig
}

func NewSoloMesh() *SoloMesh {
	sm := &SoloMesh{}
	sm.ctx = recast.NewContext(true, &sm.buildCtx)
	return sm
}

func (sm *SoloMesh) Load(path string) bool {
	// load geometry
	if !sm.geom.load(sm.ctx, path) {
		return false
	}
	sm.buildCtx.DumpLog("Geom load log %s:", path)
	return true
}

func (sm *SoloMesh) Build() bool {
	//bmin := sm.geom.NavMeshBoundsMin()
	//bmax := sm.geom.NavMeshBoundsMax()
	//verts := sm.geom.Mesh().Verts()
	//nverts := sm.geom.Mesh().VertCount()
	//tris := sm.geom.Mesh().Tris()
	//ntris := sm.geom.Mesh().TriCount()
	return true
}
