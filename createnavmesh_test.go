package detour

import "testing"

func TestCreateNavMesh(t *testing.T) {

	var (
		ctx      *rcContext
		buildCtx BuildContext
		geom     InputGeom
	)

	ctx = newRcContext(true, &buildCtx)

	ctx.Progressf("about to load geometry")
	geom.load(ctx, "testdata/wallfloors.obj")
	// geom.load(ctx, "testdata/dungeon.obj")
	//fmt.Println(geom)
	buildCtx.dumpLog("build")
}
