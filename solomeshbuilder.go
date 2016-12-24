package detour

import (
	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/math32"
)

type SoloMesh struct {
	ctx      *recast.Context
	buildCtx recast.BuildContext
	geom     InputGeom
	meshName string
	cfg      recast.Config
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
	keepInterResults := false

	bmin := sm.geom.NavMeshBoundsMin()
	bmax := sm.geom.NavMeshBoundsMax()
	verts := sm.geom.Mesh().Verts()
	nverts := sm.geom.Mesh().VertCount()
	tris := sm.geom.Mesh().Tris()
	ntris := sm.geom.Mesh().TriCount()

	//
	// Step 1. Initialize build config.
	//

	// TODO: continuer ici in Sample_SoloMesh.cpp (handleBuild)

	// Init build configuration from GUI
	// TODO: original comment says gfrom GUI but it will be either from command
	// line, from inifile or struct
	// for now, settings are the ones that work for wallfloors.obj

	// Rasterization settigs
	m_cellSize := float32(0.3)
	m_cellHeight := float32(0.2)

	// Agent properties
	m_agentHeight := float32(2.0)
	m_agentMaxClimb := float32(0.9)
	m_agentRadius := float32(0.0) // TODO; but should be different

	// Region
	m_regionMinSize := int32(8)
	m_regionMergeSize := int32(20)

	// Polygonization
	m_edgeMaxLen := int32(12)
	m_edgeMaxError := float32(.3)
	m_vertsPerPoly := int32(6)

	// Detail Mesh
	m_detailSampleDist := float32(6)
	m_detailSampleMaxError := float32(1)

	sm.cfg.Cs = m_cellSize
	sm.cfg.Ch = m_cellHeight
	sm.cfg.WalkableSlopeAngle = float32(45)
	sm.cfg.WalkableHeight = int32(math32.Ceil(m_agentHeight / sm.cfg.Ch))
	sm.cfg.WalkableClimb = int32(math32.Floor(m_agentMaxClimb / sm.cfg.Ch))
	sm.cfg.WalkableRadius = int32(math32.Ceil(m_agentRadius / sm.cfg.Cs))
	sm.cfg.MaxEdgeLen = int32(float32(m_edgeMaxLen) / m_cellSize)
	sm.cfg.MaxSimplificationError = m_edgeMaxError
	sm.cfg.MinRegionArea = m_regionMinSize * m_regionMinSize       // Note: area = size*size
	sm.cfg.MergeRegionArea = m_regionMergeSize * m_regionMergeSize // Note: area = size*size
	sm.cfg.MaxVertsPerPoly = m_vertsPerPoly

	if m_detailSampleDist < 0.9 {
		sm.cfg.DetailSampleDist = 0
	} else {
		sm.cfg.DetailSampleDist = m_cellSize * m_detailSampleDist
	}
	sm.cfg.DetailSampleMaxError = m_cellHeight * m_detailSampleMaxError

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	sm.cfg.BMin = bmin
	sm.cfg.BMax = bmax
	sm.cfg.Width, sm.cfg.Height = recast.CalcGridSize(sm.cfg.BMin, sm.cfg.BMax, sm.cfg.Cs)

	// Reset build times gathering.
	sm.ctx.ResetTimers()

	// Start the build process.
	sm.ctx.StartTimer(recast.RC_TIMER_TOTAL)

	sm.ctx.Progressf("Building navigation:")
	sm.ctx.Progressf(" - %d x %d cells", sm.cfg.Width, sm.cfg.Height)
	sm.ctx.Progressf(" - %.1fK verts, %.1fK tris", float64(nverts)/1000.0, float64(ntris)/1000.0)

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	m_solid := recast.NewHeightfield()
	if m_solid == nil {
		sm.ctx.Errorf("buildNavigation: Out of memory 'solid'.")
		return false
	}
	if !m_solid.Create(sm.ctx, sm.cfg.Width, sm.cfg.Height, sm.cfg.BMin[:], sm.cfg.BMax[:], sm.cfg.Cs, sm.cfg.Ch) {
		sm.ctx.Errorf("buildNavigation: Could not create solid heightfield.")
		return false
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas := make([]uint8, ntris)
	if len(m_triareas) == 0 {
		sm.ctx.Errorf("buildNavigation: Out of memory 'm_triareas' (%d).", ntris)
		return false
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	recast.MarkWalkableTriangles(sm.ctx, sm.cfg.WalkableSlopeAngle, verts, nverts, tris, ntris, m_triareas)
	if !recast.RasterizeTriangles(sm.ctx, verts, nverts, tris, m_triareas, ntris, m_solid, sm.cfg.WalkableClimb) {
		sm.ctx.Errorf("buildNavigation: Could not rasterize triangles.")
		return false
	}

	if !keepInterResults {
		m_triareas = nil
	}

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	recast.FilterLowHangingWalkableObstacles(sm.ctx, sm.cfg.WalkableClimb, m_solid)
	recast.FilterLedgeSpans(sm.ctx, sm.cfg.WalkableHeight, sm.cfg.WalkableClimb, m_solid)
	recast.FilterWalkableLowHeightSpans(sm.ctx, sm.cfg.WalkableHeight, m_solid)

	// CONTINUER ICI

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	/*
		m_chf := &CompactHeightfield{}
		if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf)) {
			m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
			return 0;
		}

		if (!m_keepInterResults) {
			rcFreeHeightField(m_solid);
			m_solid = 0;
		}

		// Erode the walkable area by agent radius.
		if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf)) {
			m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
			return 0;
		}

		// (Optional) Mark areas.
		const ConvexVolume* vols = m_geom.getConvexVolumes();
		for (int i  = 0; i < m_geom.getConvexVolumeCount(); ++i) {
			rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);
		}
	*/

	sm.buildCtx.DumpLog("Navmesh Build log")
	return true
}
