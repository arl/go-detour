package tilemesh

import (
	"io"
	"time"

	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/go-detour/sample"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

// TileMesh allows building multi-tile navigation meshes.
//
// TODO: rename TileMeshBuilder or something like that to show that this is
// not actually a navmesh, but more an api to build and manage one
type TileMesh struct {
	ctx               *recast.BuildContext
	geom              recast.InputGeom
	navMesh           detour.NavMesh
	meshName          string
	cfg               recast.Config
	partitionType     sample.PartitionType
	settings          recast.BuildSettings
	lastBuiltTileBMin d3.Vec3
	lastBuiltTileBMax d3.Vec3
	totalBuildTime    time.Duration
	tileBuildTime     time.Duration
	tileMemUsage      float32

	maxTiles        uint32
	maxPolysPerTile uint32
	tileTriCount    int32

	triAreas []uint8
	solid    *recast.Heightfield
	chf      *recast.CompactHeightfield
	cset     *recast.ContourSet
	pmesh    *recast.PolyMesh
	dmesh    *recast.PolyMeshDetail
}

// New creates a new tile mesh with default build settings.
func New(ctx *recast.BuildContext) *TileMesh {
	sm := &TileMesh{
		settings:          DefaultSettings(),
		lastBuiltTileBMin: d3.NewVec3(),
		lastBuiltTileBMax: d3.NewVec3(),
	}
	sm.ctx = ctx
	sm.partitionType = sample.PartitionMonotone
	return sm
}

// SetSettings sets the build settings for this tile mesh.
func (tm *TileMesh) SetSettings(s recast.BuildSettings) {
	tm.settings = s
}

// LoadGeometry loads geometry from r that reads from a geometry definition
// file.
func (tm *TileMesh) LoadGeometry(r io.Reader) error {
	return tm.geom.LoadOBJMesh(r)
}

// InputGeom returns the nav mesh input geometry.
func (tm *TileMesh) InputGeom() *recast.InputGeom {
	return &tm.geom
}

// Build builds the navigation mesh for the input geometry provided
func (tm *TileMesh) Build() (*detour.NavMesh, bool) {
	if tm.geom.Mesh() == nil {
		// TODO: error "no vertices and triangles"
		return nil, false
	}

	bmin := tm.geom.NavMeshBoundsMin()
	bmax := tm.geom.NavMeshBoundsMax()
	gw, gh := recast.CalcGridSize(bmin[:], bmax[:], tm.settings.CellSize)
	ts := int32(tm.settings.TileSize)
	tw := (gw + ts - 1) / ts
	th := (gh + ts - 1) / ts

	// Max tiles and max polys affect how the tile IDs are caculated.
	// There are 22 bits available for identifying a tile and a polygon.
	tileBits := math32.MinInt32(int32(math32.Ilog2(math32.NextPow2(uint32(tw*th)))), 14)
	polyBits := 22 - tileBits
	tm.maxTiles = 1 << uint(tileBits)
	tm.maxPolysPerTile = 1 << uint(polyBits)

	var (
		params detour.NavMeshParams
		status detour.Status
	)
	copy(params.Orig[:], tm.geom.NavMeshBoundsMin()[:3])
	params.TileWidth = tm.settings.TileSize * tm.settings.CellSize
	params.TileHeight = tm.settings.TileSize * tm.settings.CellSize
	params.MaxTiles = tm.maxTiles
	params.MaxPolys = tm.maxPolysPerTile
	status = tm.navMesh.Init(&params)
	if detour.StatusFailed(status) {
		tm.ctx.Errorf("TileMesh.Build: Could not init navmesh")
		return nil, false
	}
	status, _ = detour.NewNavMeshQuery(&tm.navMesh, 2048)
	if detour.StatusFailed(status) {
		tm.ctx.Errorf("TileMesh.Build: Could not init detour navmesh query")
		return nil, false
	}

	tm.buildAllTiles()

	return &tm.navMesh, true
}

func (tm *TileMesh) buildAllTiles() (*detour.NavMesh, bool) {
	bmin := tm.geom.NavMeshBoundsMin()
	bmax := tm.geom.NavMeshBoundsMax()
	gw, gh := recast.CalcGridSize(bmin[:], bmax[:], tm.settings.CellSize)
	ts := int32(tm.settings.TileSize)
	tw := (gw + ts - 1) / ts
	th := (gh + ts - 1) / ts
	tcs := tm.settings.TileSize * tm.settings.CellSize

	// Start the build process.
	tm.ctx.StartTimer(recast.TimerTemp)
	for y := int32(0); y < th; y++ {
		for x := int32(0); x < tw; x++ {

			tm.lastBuiltTileBMin[0] = bmin[0] + float32(x)*tcs
			tm.lastBuiltTileBMin[1] = bmin[1]
			tm.lastBuiltTileBMin[2] = bmin[2] + float32(y)*tcs

			tm.lastBuiltTileBMax[0] = bmin[0] + float32(x+1)*tcs
			tm.lastBuiltTileBMax[1] = bmax[1]
			tm.lastBuiltTileBMax[2] = bmin[2] + float32(y+1)*tcs

			data := tm.buildTileMesh(x, y, tm.lastBuiltTileBMin[:], tm.lastBuiltTileBMax[:])
			if data != nil {
				// Remove any previous data (navmesh owns and deletes the data).
				tm.navMesh.RemoveTile(tm.navMesh.TileRefAt(x, y, 0))
				// Let the navmesh own the data.
				tm.navMesh.AddTile(data, detour.TileRef(0))
			}
		}
	}

	// Start the build process.
	tm.ctx.StopTimer(recast.TimerTemp)

	tm.totalBuildTime = tm.ctx.AccumulatedTime(recast.TimerTemp)

	// TODO: probably useless
	return &tm.navMesh, true
}

func (tm *TileMesh) buildTileMesh(tx, ty int32, bmin, bmax []float32) []byte {
	if tm.geom.Mesh() == nil || tm.geom.ChunkyMesh() == nil {
		tm.ctx.Errorf("buildNavigation: Input mesh is not specified.")
		return nil
	}

	tm.tileMemUsage = 0
	tm.tileBuildTime = 0

	verts := tm.geom.Mesh().Verts()
	nverts := tm.geom.Mesh().VertCount()
	//tris := sm.geom.Mesh().Tris()
	ntris := tm.geom.Mesh().TriCount()
	chunkyMesh := tm.geom.ChunkyMesh()

	//
	// Step 1. Initialize build config.
	//

	// Rasterization settings
	cellSize := tm.settings.CellSize
	cellHeight := tm.settings.CellHeight

	// Agent properties
	agentHeight := tm.settings.AgentHeight
	agentMaxClimb := tm.settings.AgentMaxClimb
	agentRadius := tm.settings.AgentRadius

	// Region
	regionMinSize := tm.settings.RegionMinSize
	regionMergeSize := tm.settings.RegionMergeSize

	// Polygonization
	edgeMaxLen := tm.settings.EdgeMaxLen
	edgeMaxError := tm.settings.EdgeMaxError
	vertsPerPoly := tm.settings.VertsPerPoly

	// Detail Mesh
	detailSampleDist := tm.settings.DetailSampleDist
	detailSampleMaxError := tm.settings.DetailSampleMaxError

	tm.cfg.Cs = cellSize
	tm.cfg.Ch = cellHeight
	tm.cfg.WalkableSlopeAngle = tm.settings.AgentMaxSlope
	tm.cfg.WalkableHeight = int32(math32.Ceil(agentHeight / tm.cfg.Ch))
	tm.cfg.WalkableClimb = int32(math32.Floor(agentMaxClimb / tm.cfg.Ch))
	tm.cfg.WalkableRadius = int32(math32.Ceil(agentRadius / tm.cfg.Cs))
	tm.cfg.MaxEdgeLen = int32(float32(edgeMaxLen) / cellSize)
	tm.cfg.MaxSimplificationError = edgeMaxError
	tm.cfg.MinRegionArea = int32(regionMinSize * regionMinSize)       // Note: area = size*size
	tm.cfg.MergeRegionArea = int32(regionMergeSize * regionMergeSize) // Note: area = size*size
	tm.cfg.MaxVertsPerPoly = int32(vertsPerPoly)
	tm.cfg.TileSize = int32(tm.settings.TileSize)
	tm.cfg.BorderSize = tm.cfg.WalkableRadius + 3 // Reserve enough padding
	tm.cfg.Width = tm.cfg.TileSize + tm.cfg.BorderSize*2
	tm.cfg.Height = tm.cfg.TileSize + tm.cfg.BorderSize*2

	if detailSampleDist < 0.9 {
		tm.cfg.DetailSampleDist = 0
	} else {
		tm.cfg.DetailSampleDist = cellSize * detailSampleDist
	}
	tm.cfg.DetailSampleMaxError = cellHeight * detailSampleMaxError

	// Expand the heighfield bounding box by border size to find the extents of
	// geometry we need to build this tile.
	//
	// This is done in order to make sure that the navmesh tiles connect
	// correctly at the borders, and the obstacles close to the border work
	// correctly with the dilation process. No polygons (or contours) will be
	// created on the border area.
	//
	// IMPORTANT!
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build
	//   : |     | :
	//   : +-----+ :<-- geometry needed
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	//
	// For example if you build a navmesh for terrain, and want the navmesh
	// tiles to match the terrain tile size you will need to pass in data from
	// neighbour terrain tiles too! In a simple case, just pass in all the 8
	// neighbours, or use the bounding box below to only pass in a sliver of
	// each of the 8 neighbours.

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the area could be
	// specified by an user defined box, etc.
	copy(tm.cfg.BMin[:], bmin[:3])
	copy(tm.cfg.BMax[:], bmax[:3])
	tm.cfg.BMin[0] -= float32(tm.cfg.BorderSize) * tm.cfg.Cs
	tm.cfg.BMin[2] -= float32(tm.cfg.BorderSize) * tm.cfg.Cs
	tm.cfg.BMax[0] += float32(tm.cfg.BorderSize) * tm.cfg.Cs
	tm.cfg.BMax[2] += float32(tm.cfg.BorderSize) * tm.cfg.Cs

	// Reset build times gathering.
	tm.ctx.ResetTimers()

	// Start the build process.
	tm.ctx.StartTimer(recast.TimerTotal)

	tm.ctx.Progressf("Building navigation:")
	tm.ctx.Progressf(" - %d x %d cells", tm.cfg.Width, tm.cfg.Height)
	tm.ctx.Progressf(" - %.1fK verts, %.1fK tris", float64(nverts)/1000.0, float64(ntris)/1000.0)

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	tm.solid = recast.NewHeightfield(tm.cfg.Width, tm.cfg.Height, tm.cfg.BMin[:], tm.cfg.BMax[:], tm.cfg.Cs, tm.cfg.Ch)

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	tm.triAreas = make([]uint8, chunkyMesh.MaxTrisPerChunk)

	var tbmin, tbmax [2]float32
	tbmin[0] = tm.cfg.BMin[0]
	tbmin[1] = tm.cfg.BMin[2]
	tbmax[0] = tm.cfg.BMax[0]
	tbmax[1] = tm.cfg.BMax[2]
	var cid [512]int32 // TODO: Make grow when returning too many items.
	ncid := chunkyMesh.ChunksOverlappingRect(tbmin, tbmax, cid[:])
	if ncid == 0 {
		return nil
	}

	tm.tileTriCount = 0

	for i := 0; i < ncid; i++ {
		node := chunkyMesh.Nodes[cid[i]]
		ctris := chunkyMesh.Tris[node.I*3:]
		nctris := node.N

		tm.tileTriCount += nctris

		for ai := 0; ai < len(tm.triAreas); ai++ {
			tm.triAreas[ai] = 0
		}
		recast.MarkWalkableTriangles(tm.ctx, tm.cfg.WalkableSlopeAngle,
			verts, nverts, ctris, nctris, tm.triAreas)

		if !recast.RasterizeTriangles(tm.ctx, verts, nverts, ctris, tm.triAreas, nctris, tm.solid, tm.cfg.WalkableClimb) {
			return nil
		}
	}

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	recast.FilterLowHangingWalkableObstacles(tm.ctx, tm.cfg.WalkableClimb, tm.solid)
	recast.FilterLedgeSpans(tm.ctx, tm.cfg.WalkableHeight, tm.cfg.WalkableClimb, tm.solid)
	recast.FilterWalkableLowHeightSpans(tm.ctx, tm.cfg.WalkableHeight, tm.solid)

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	tm.chf = &recast.CompactHeightfield{}
	if !recast.BuildCompactHeightfield(tm.ctx, tm.cfg.WalkableHeight, tm.cfg.WalkableClimb, tm.solid, tm.chf) {
		tm.ctx.Errorf("buildNavigation: Could not build compact data.")
		return nil
	}

	// Erode the walkable area by agent radius.
	if !recast.ErodeWalkableArea(tm.ctx, tm.cfg.WalkableRadius, tm.chf) {
		tm.ctx.Errorf("buildNavigation: Could not erode.")
		return nil
	}

	// (Optional) Mark areas.
	vols := tm.geom.ConvexVolumes()

	// TODO: : control that ConvexVolumeCount() is also 0 on original library
	for i := int32(0); i < tm.geom.ConvexVolumesCount(); i++ {
		recast.MarkConvexPolyArea(tm.ctx, vols[i].Verts[:], vols[i].NVerts, vols[i].HMin, vols[i].HMax, uint8(vols[i].Area), tm.chf)
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

	if tm.partitionType == sample.PartitionWatershed {
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
	} else if tm.partitionType == sample.PartitionMonotone {
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if !recast.BuildRegionsMonotone(tm.ctx, tm.chf, tm.cfg.BorderSize, tm.cfg.MinRegionArea, tm.cfg.MergeRegionArea) {
			tm.ctx.Errorf("buildNavigation: Could not build monotone regions.")
			return nil
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
	tm.cset = &recast.ContourSet{}
	if !recast.BuildContours(tm.ctx, tm.chf, tm.cfg.MaxSimplificationError, tm.cfg.MaxEdgeLen, tm.cset, recast.ContourTessWallEdges) {
		tm.ctx.Errorf("buildNavigation: Could not create contours.")
		return nil
	}

	if tm.cset.NConts == 0 {
		return nil
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	var ret bool
	tm.pmesh, ret = recast.BuildPolyMesh(tm.ctx, tm.cset, tm.cfg.MaxVertsPerPoly)
	if !ret {
		tm.ctx.Errorf("buildNavigation: Could not triangulate contours.")
		return nil
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	tm.dmesh, ret = recast.BuildPolyMeshDetail(tm.ctx, tm.pmesh, tm.chf, tm.cfg.DetailSampleDist, tm.cfg.DetailSampleMaxError)
	if !ret {
		tm.ctx.Errorf("buildNavigation: Could not build detail mesh.")
		return nil
	}

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	var (
		navData []uint8
		err     error
	)
	if tm.cfg.MaxVertsPerPoly <= int32(detour.VertsPerPolygon) {
		if tm.pmesh.NVerts >= 0xffff {
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			tm.ctx.Errorf("Too many vertices per tile %d (max: %d).", tm.pmesh.NVerts, 0xffff)
			return nil
		}

		// Update poly flags from areas.
		for i := int32(0); i < tm.pmesh.NPolys; i++ {
			if tm.pmesh.Areas[i] == recast.WalkableArea {
				tm.pmesh.Areas[i] = sample.PolyAreaGround
			}

			if tm.pmesh.Areas[i] == sample.PolyAreaGround ||
				tm.pmesh.Areas[i] == sample.PolyAreaGrass ||
				tm.pmesh.Areas[i] == sample.PolyAreaRoad {
				tm.pmesh.Flags[i] = sample.PolyFlagsWalk
			} else if tm.pmesh.Areas[i] == sample.PolyAreaWater {
				tm.pmesh.Flags[i] = sample.PolyFlagsSwim
			} else if tm.pmesh.Areas[i] == sample.PolyAreaDoor {
				tm.pmesh.Flags[i] = sample.PolyFlagsWalk | sample.PolyFlagsDoor
			}
		}

		var params detour.NavMeshCreateParams
		params.Verts = tm.pmesh.Verts
		params.VertCount = tm.pmesh.NVerts
		params.Polys = tm.pmesh.Polys
		params.PolyAreas = tm.pmesh.Areas
		params.PolyFlags = tm.pmesh.Flags
		params.PolyCount = tm.pmesh.NPolys
		params.Nvp = tm.pmesh.Nvp
		params.DetailMeshes = tm.dmesh.Meshes
		params.DetailVerts = tm.dmesh.Verts
		params.DetailVertsCount = tm.dmesh.NVerts
		params.DetailTris = tm.dmesh.Tris
		params.DetailTriCount = tm.dmesh.NTris
		params.OffMeshConVerts = tm.geom.OffMeshConnectionVerts()
		params.OffMeshConRad = tm.geom.OffMeshConnectionRads()
		params.OffMeshConDir = tm.geom.OffMeshConnectionDirs()
		params.OffMeshConAreas = tm.geom.OffMeshConnectionAreas()
		params.OffMeshConFlags = tm.geom.OffMeshConnectionFlags()
		params.OffMeshConUserID = tm.geom.OffMeshConnectionId()
		params.OffMeshConCount = tm.geom.OffMeshConnectionCount()
		params.WalkableHeight = agentHeight
		params.WalkableRadius = agentRadius
		params.WalkableClimb = agentMaxClimb
		params.TileX = tx
		params.TileY = ty
		params.TileLayer = 0
		copy(params.BMin[:], tm.pmesh.BMin[:])
		copy(params.BMax[:], tm.pmesh.BMax[:])
		params.Cs = tm.cfg.Cs
		params.Ch = tm.cfg.Ch
		params.BuildBvTree = true

		if navData, err = detour.CreateNavMeshData(&params); err != nil {
			tm.ctx.Errorf("Could not build Detour navmesh: %v", err)
			return nil
		}
	}

	tm.tileMemUsage = float32(len(navData)) / 1024.0

	tm.ctx.StopTimer(recast.TimerTotal)
	// Log performance stats.
	recast.LogBuildTimes(tm.ctx, tm.ctx.AccumulatedTime(recast.TimerTotal))
	tm.ctx.Progressf(">> Polymesh: %d vertices  %d polygons", tm.pmesh.NVerts, tm.pmesh.NPolys)
	tm.tileBuildTime = tm.ctx.AccumulatedTime(recast.TimerTotal)

	return navData
}

func (tm *TileMesh) BuildTile(pos d3.Vec3) {
	bmin := tm.geom.NavMeshBoundsMin()
	bmax := tm.geom.NavMeshBoundsMax()

	ts := tm.settings.TileSize * tm.settings.CellSize
	tx := int32((pos[0] - bmin[0]) / ts)
	ty := int32((pos[2] - bmin[2]) / ts)

	tm.lastBuiltTileBMin[0] = bmin[0] + float32(tx)*ts
	tm.lastBuiltTileBMin[1] = bmin[1]
	tm.lastBuiltTileBMin[2] = bmin[2] + float32(ty)*ts

	tm.lastBuiltTileBMax[0] = bmin[0] + float32(tx+1)*ts
	tm.lastBuiltTileBMax[1] = bmax[1]
	tm.lastBuiltTileBMax[2] = bmin[2] + float32(ty+1)*ts

	tm.ctx.ResetLog()

	data := tm.buildTileMesh(tx, ty, tm.lastBuiltTileBMin, tm.lastBuiltTileBMax)

	// Remove any previous data (navmesh owns and deletes the data).
	tm.navMesh.RemoveTile(tm.navMesh.TileRefAt(tx, ty, 0))

	// Add tile, or leave the location empty.
	if data != nil {
		// Let the navmesh own the data.
		status, _ := tm.navMesh.AddTile(data, detour.TileRef(0))
		if detour.StatusFailed(status) {
			data = nil
		}
	}

	tm.ctx.DumpLog("Build Tile (%d,%d):", tx, ty)
}

func (tm *TileMesh) TilePos(pos d3.Vec3) (x, y int32) {
	bmin := tm.geom.NavMeshBoundsMin()

	ts := tm.settings.TileSize * tm.settings.CellSize
	return int32((pos[0] - bmin[0]) / ts), int32((pos[2] - bmin[2]) / ts)
}

func (tm *TileMesh) RemoveTile(pos d3.Vec3) {
	bmin := tm.geom.NavMeshBoundsMin()
	bmax := tm.geom.NavMeshBoundsMax()

	ts := tm.settings.TileSize * tm.settings.CellSize
	tx := int32((pos[0] - bmin[0]) / ts)
	ty := int32((pos[2] - bmin[2]) / ts)

	tm.lastBuiltTileBMin[0] = bmin[0] + float32(tx)*ts
	tm.lastBuiltTileBMin[1] = bmin[1]
	tm.lastBuiltTileBMin[2] = bmin[2] + float32(ty)*ts

	tm.lastBuiltTileBMax[0] = bmin[0] + float32(tx+1)*ts
	tm.lastBuiltTileBMax[1] = bmax[1]
	tm.lastBuiltTileBMax[2] = bmin[2] + float32(ty+1)*ts

	tm.navMesh.RemoveTile(tm.navMesh.TileRefAt(tx, ty, 0))
}
