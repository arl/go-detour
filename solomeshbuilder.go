package detour

import (
	"fmt"

	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/math32"
)

type SamplePartitionType int

const (
	SAMPLE_PARTITION_WATERSHED SamplePartitionType = iota
	SAMPLE_PARTITION_MONOTONE
	SAMPLE_PARTITION_LAYERS
)

type SoloMesh struct {
	ctx           *recast.Context
	buildCtx      recast.BuildContext
	geom          InputGeom
	meshName      string
	cfg           recast.Config
	partitionType SamplePartitionType
}

func NewSoloMesh() *SoloMesh {
	sm := &SoloMesh{}
	sm.ctx = recast.NewContext(true, &sm.buildCtx)
	sm.partitionType = SAMPLE_PARTITION_MONOTONE
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

func (sm *SoloMesh) Build() ([]uint8, bool) {
	var navData []uint8
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
		return navData, false
	}
	if !m_solid.Create(sm.ctx, sm.cfg.Width, sm.cfg.Height, sm.cfg.BMin[:], sm.cfg.BMax[:], sm.cfg.Cs, sm.cfg.Ch) {
		sm.ctx.Errorf("buildNavigation: Could not create solid heightfield.")
		return navData, false
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas := make([]uint8, ntris)
	if len(m_triareas) == 0 {
		sm.ctx.Errorf("buildNavigation: Out of memory 'm_triareas' (%d).", ntris)
		return navData, false
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	recast.MarkWalkableTriangles(sm.ctx, sm.cfg.WalkableSlopeAngle, verts, nverts, tris, ntris, m_triareas)
	if !recast.RasterizeTriangles(sm.ctx, verts, nverts, tris, m_triareas, ntris, m_solid, sm.cfg.WalkableClimb) {
		sm.ctx.Errorf("buildNavigation: Could not rasterize triangles.")
		return navData, false
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

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_chf := &recast.CompactHeightfield{}
	if !recast.BuildCompactHeightfield(sm.ctx, sm.cfg.WalkableHeight, sm.cfg.WalkableClimb, m_solid, m_chf) {
		sm.ctx.Errorf("buildNavigation: Could not build compact data.")
		return navData, false
	}

	fmt.Println(m_chf)

	if !keepInterResults {
		m_solid.Free()
		m_solid = nil
	}

	// Erode the walkable area by agent radius.
	if !recast.ErodeWalkableArea(sm.ctx, sm.cfg.WalkableRadius, m_chf) {
		sm.ctx.Errorf("buildNavigation: Could not erode.")
		return navData, false
	}

	//// (Optional) Mark areas.
	vols := sm.geom.ConvexVolumes()
	for i := int32(0); i < sm.geom.ConvexVolumesCount(); i++ {
		recast.MarkConvexPolyArea(sm.ctx, vols[i].verts[:], vols[i].nverts, vols[i].hmin, vols[i].hmax, uint8(vols[i].area), m_chf)
	}

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if sm.partitionType == SAMPLE_PARTITION_WATERSHED {
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		//if (!rcBuildDistanceField(m_ctx, *m_chf))
		//{
		//m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
		//return navData, false
		//}

		//// Partition the walkable surface into simple regions without holes.
		//if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		//{
		//m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
		//return navData, false
		//}
	} else if sm.partitionType == SAMPLE_PARTITION_MONOTONE {
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if !BuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea) {
			m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.")
			return navData, false
		}
	} else {
		// SAMPLE_PARTITION_LAYERS
		// Partition the walkable surface into simple regions without holes.
		//if !rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea) {
		//m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.")
		//return navData, false
		//}
	}

	// END

	sm.ctx.StopTimer(recast.RC_TIMER_TOTAL)
	// Show performance stats.
	recast.LogBuildTimes(sm.ctx, sm.ctx.AccumulatedTime(recast.RC_TIMER_TOTAL))
	//	sm.ctx.Progressf(">> Polymesh: %d vertices  %d polygons", m_pmesh.nverts, m_pmesh.npolys);

	//m_tileBuildTime := sm.ctx.AccumulatedTime(recast.RC_TIMER_TOTAL) / 1000.0
	//dataSize = navDataSize
	sm.buildCtx.DumpLog("Navmesh Build log")
	return navData, true
}
