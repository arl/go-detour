package solomesh

import (
	"io"

	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/go-detour/sample"
	"github.com/aurelien-rainone/math32"
)

// SoloMesh allows building of single tile navigation meshes.
//
// TODO: rename SoloMeshBuilder or something like that to show that this is
// not actually a navmesh, but more an api to build and manage one
type SoloMesh struct {
	ctx           *recast.BuildContext
	geom          recast.InputGeom
	meshName      string
	cfg           recast.Config
	partitionType sample.PartitionType
	settings      Settings
}

// New creates a new solo mesh with default build settings.
func New(ctx *recast.BuildContext) *SoloMesh {
	sm := &SoloMesh{settings: NewSettings()}
	sm.ctx = ctx
	sm.partitionType = sample.PartitionMonotone
	return sm
}

// SetSettings sets the build settings for this solo mesh.
func (sm *SoloMesh) SetSettings(s Settings) {
	sm.settings = s
}

// LoadGeometry loads geometry from r that reads from a geometry definition
// file.
func (sm *SoloMesh) LoadGeometry(r io.Reader) error {
	return sm.geom.LoadOBJMesh(r)
}

// InputGeom returns the nav mesh input geometry.
func (sm *SoloMesh) InputGeom() *recast.InputGeom {
	return &sm.geom
}

// Build builds the navigation mesh for the input geometry provided
func (sm *SoloMesh) Build() (*detour.NavMesh, bool) {
	bmin := sm.geom.NavMeshBoundsMin()
	bmax := sm.geom.NavMeshBoundsMax()
	verts := sm.geom.Mesh().Verts()
	nverts := sm.geom.Mesh().VertCount()
	tris := sm.geom.Mesh().Tris()
	ntris := sm.geom.Mesh().TriCount()

	//
	// Step 1. Initialize build config.
	//

	// Rasterization settings
	cellSize := sm.settings.CellSize
	cellHeight := sm.settings.CellHeight

	// Agent properties
	agentHeight := sm.settings.AgentHeight
	agentMaxClimb := sm.settings.AgentMaxClimb
	agentRadius := sm.settings.AgentRadius

	// Region
	regionMinSize := sm.settings.RegionMinSize
	regionMergeSize := sm.settings.RegionMergeSize

	// Polygonization
	edgeMaxLen := sm.settings.EdgeMaxLen
	edgeMaxError := sm.settings.EdgeMaxError
	vertsPerPoly := sm.settings.VertsPerPoly

	// Detail Mesh
	detailSampleDist := sm.settings.DetailSampleDist
	detailSampleMaxError := sm.settings.DetailSampleMaxError

	sm.cfg.Cs = cellSize
	sm.cfg.Ch = cellHeight
	sm.cfg.WalkableSlopeAngle = sm.settings.WalkableSlopeAngle
	sm.cfg.WalkableHeight = int32(math32.Ceil(agentHeight / sm.cfg.Ch))
	sm.cfg.WalkableClimb = int32(math32.Floor(agentMaxClimb / sm.cfg.Ch))
	sm.cfg.WalkableRadius = int32(math32.Ceil(agentRadius / sm.cfg.Cs))
	sm.cfg.MaxEdgeLen = int32(float32(edgeMaxLen) / cellSize)
	sm.cfg.MaxSimplificationError = edgeMaxError
	sm.cfg.MinRegionArea = regionMinSize * regionMinSize       // Note: area = size*size
	sm.cfg.MergeRegionArea = regionMergeSize * regionMergeSize // Note: area = size*size
	sm.cfg.MaxVertsPerPoly = vertsPerPoly

	if detailSampleDist < 0.9 {
		sm.cfg.DetailSampleDist = 0
	} else {
		sm.cfg.DetailSampleDist = cellSize * detailSampleDist
	}
	sm.cfg.DetailSampleMaxError = cellHeight * detailSampleMaxError

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	sm.cfg.BMin = bmin
	sm.cfg.BMax = bmax
	sm.cfg.Width, sm.cfg.Height = recast.CalcGridSize(sm.cfg.BMin, sm.cfg.BMax, sm.cfg.Cs)

	// Reset build times gathering.
	sm.ctx.ResetTimers()

	// Start the build process.
	sm.ctx.StartTimer(recast.TimerTotal)

	sm.ctx.Progressf("Building navigation:")
	sm.ctx.Progressf(" - %d x %d cells", sm.cfg.Width, sm.cfg.Height)
	sm.ctx.Progressf(" - %.1fK verts, %.1fK tris", float64(nverts)/1000.0, float64(ntris)/1000.0)

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	var solid *recast.Heightfield
	solid = recast.NewHeightfield(sm.cfg.Width, sm.cfg.Height, sm.cfg.BMin[:], sm.cfg.BMax[:], sm.cfg.Cs, sm.cfg.Ch)

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	triAreas := make([]uint8, ntris)

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	recast.MarkWalkableTriangles(sm.ctx, sm.cfg.WalkableSlopeAngle, verts, nverts, tris, ntris, triAreas)
	if !recast.RasterizeTriangles(sm.ctx, verts, nverts, tris, triAreas, ntris, solid, sm.cfg.WalkableClimb) {
		sm.ctx.Errorf("buildNavigation: Could not rasterize triangles.")
		return nil, false
	}

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	recast.FilterLowHangingWalkableObstacles(sm.ctx, sm.cfg.WalkableClimb, solid)
	recast.FilterLedgeSpans(sm.ctx, sm.cfg.WalkableHeight, sm.cfg.WalkableClimb, solid)
	recast.FilterWalkableLowHeightSpans(sm.ctx, sm.cfg.WalkableHeight, solid)

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	chf := &recast.CompactHeightfield{}
	if !recast.BuildCompactHeightfield(sm.ctx, sm.cfg.WalkableHeight, sm.cfg.WalkableClimb, solid, chf) {
		sm.ctx.Errorf("buildNavigation: Could not build compact data.")
		return nil, false
	}

	// Erode the walkable area by agent radius.
	if !recast.ErodeWalkableArea(sm.ctx, sm.cfg.WalkableRadius, chf) {
		sm.ctx.Errorf("buildNavigation: Could not erode.")
		return nil, false
	}

	// (Optional) Mark areas.
	vols := sm.geom.ConvexVolumes()

	// TODO: : control that ConvexVolumeCount() is also 0 on original library
	for i := int32(0); i < sm.geom.ConvexVolumesCount(); i++ {
		recast.MarkConvexPolyArea(sm.ctx, vols[i].Verts[:], vols[i].NVerts, vols[i].HMin, vols[i].HMax, uint8(vols[i].Area), chf)
	}

	// Partition the heightfield so that we can use simple algorithm later to
	// triangulate the walkable areas. There are 3 partitioning methods, each
	// with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or
	//     overlaps
	//   - the are some corner cases where this method creates produces holes
	//     and overlaps
	//      - holes may appear when a small obstacles is close to large open
	//        area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e
	//        stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if
	//     you have large open areas
	// 2) Monotone partitioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps
	//     (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitioning
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than
	//     monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than
	//     monotone) if you have large open areas with small obstacles (not a
	//     problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized
	//     tiles

	if sm.partitionType == sample.PartitionWatershed {
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
	} else if sm.partitionType == sample.PartitionMonotone {
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if !recast.BuildRegionsMonotone(sm.ctx, chf, 0, sm.cfg.MinRegionArea, sm.cfg.MergeRegionArea) {
			sm.ctx.Errorf("buildNavigation: Could not build monotone regions.")
			return nil, false
		}
	} else {
		// SAMPLE_PARTITION_LAYERS
		// Partition the walkable surface into simple regions without holes.
		//if !rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea) {
		//m_ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.")
		//return navData, false
		//}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	cset := &recast.ContourSet{}
	if !recast.BuildContours(sm.ctx, chf, sm.cfg.MaxSimplificationError, sm.cfg.MaxEdgeLen, cset, recast.ContourTessWallEdges) {
		sm.ctx.Errorf("buildNavigation: Could not create contours.")
		return nil, false
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	var (
		ret   bool
		pmesh *recast.PolyMesh
	)
	pmesh, ret = recast.BuildPolyMesh(sm.ctx, cset, sm.cfg.MaxVertsPerPoly)
	if !ret {
		sm.ctx.Errorf("buildNavigation: Could not triangulate contours.")
		return nil, false
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	var dmesh *recast.PolyMeshDetail
	dmesh, ret = recast.BuildPolyMeshDetail(sm.ctx, pmesh, chf, sm.cfg.DetailSampleDist, sm.cfg.DetailSampleMaxError)
	if !ret {
		sm.ctx.Errorf("buildNavigation: Could not build detail mesh.")
		return nil, false
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if sm.cfg.MaxVertsPerPoly > int32(detour.VertsPerPolygon) {
		sm.ctx.Errorf("detour doesn't handle so many vertices per polygon. should be <= %v", detour.VertsPerPolygon)
		return nil, false
	}
	var (
		navData []uint8
		err     error
	)

	// Update poly flags from areas.
	for i := int32(0); i < pmesh.NPolys; i++ {
		if pmesh.Areas[i] == recast.WalkableArea {
			pmesh.Areas[i] = sample.PolyAreaGround
		}

		if pmesh.Areas[i] == sample.PolyAreaGround ||
			pmesh.Areas[i] == sample.PolyAreaGrass ||
			pmesh.Areas[i] == sample.PolyAreaRoad {
			pmesh.Flags[i] = sample.PolyFlagsWalk
		} else if pmesh.Areas[i] == sample.PolyAreaWater {
			pmesh.Flags[i] = sample.PolyFlagsSwim
		} else if pmesh.Areas[i] == sample.PolyAreaDoor {
			pmesh.Flags[i] = sample.PolyFlagsWalk | sample.PolyFlagsDoor
		}
	}

	var params detour.NavMeshCreateParams
	params.Verts = pmesh.Verts
	params.VertCount = pmesh.NVerts
	params.Polys = pmesh.Polys
	params.PolyAreas = pmesh.Areas
	params.PolyFlags = pmesh.Flags
	params.PolyCount = pmesh.NPolys
	params.Nvp = pmesh.Nvp
	params.DetailMeshes = dmesh.Meshes
	params.DetailVerts = dmesh.Verts
	params.DetailVertsCount = dmesh.NVerts
	params.DetailTris = dmesh.Tris
	params.DetailTriCount = dmesh.NTris
	params.OffMeshConVerts = sm.geom.OffMeshConnectionVerts()
	params.OffMeshConRad = sm.geom.OffMeshConnectionRads()
	params.OffMeshConDir = sm.geom.OffMeshConnectionDirs()
	params.OffMeshConAreas = sm.geom.OffMeshConnectionAreas()
	params.OffMeshConFlags = sm.geom.OffMeshConnectionFlags()
	params.OffMeshConUserID = sm.geom.OffMeshConnectionId()
	params.OffMeshConCount = sm.geom.OffMeshConnectionCount()
	params.WalkableHeight = agentHeight
	params.WalkableRadius = agentRadius
	params.WalkableClimb = agentMaxClimb
	copy(params.BMin[:], pmesh.BMin[:])
	copy(params.BMax[:], pmesh.BMax[:])
	params.Cs = sm.cfg.Cs
	params.Ch = sm.cfg.Ch
	params.BuildBvTree = true

	if navData, err = detour.CreateNavMeshData(&params); err != nil {
		sm.ctx.Errorf("Could not build Detour navmesh: %v", err)
		return nil, false
	}

	var (
		navMesh detour.NavMesh
		// navQuery *detour.NavMeshQuery
		status detour.Status
	)
	status = navMesh.InitForSingleTile(navData, 0)
	if detour.StatusFailed(status) {
		sm.ctx.Errorf("Could not init Detour navmesh")
		return nil, false
	}

	status, _ = detour.NewNavMeshQuery(&navMesh, 2048)
	if detour.StatusFailed(status) {
		sm.ctx.Errorf("Could not init Detour navmesh query")
		return nil, false
	}

	sm.ctx.StopTimer(recast.TimerTotal)
	// Log performance stats.
	recast.LogBuildTimes(sm.ctx, sm.ctx.AccumulatedTime(recast.TimerTotal))
	sm.ctx.Progressf(">> Polymesh: %d vertices  %d polygons", pmesh.NVerts, pmesh.NPolys)

	return &navMesh, true
}
