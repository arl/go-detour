package detour

import (
	"log"
	"path/filepath"
)

const (
	MAX_OFFMESH_CONNECTIONS = 256
	MAX_VOLUMES             = 256
	MAX_CONVEXVOL_PTS       = 12
)

type ConvexVolume struct {
	verts      [MAX_CONVEXVOL_PTS * 3]float32
	hmin, hmax float32
	nverts     int32
	area       int32
}

type BuildSettings struct {
	// Cell size in world units
	cellSize float32
	// Cell height in world units
	cellHeight float32
	// Agent height in world units
	agentHeight float32
	// Agent radius in world units
	agentRadius float32
	// Agent max climb in world units
	agentMaxClimb float32
	// Agent max slope in degrees
	agentMaxSlope float32
	// Region minimum size in voxels.
	// regionMinSize = sqrt(regionMinArea)
	regionMinSize float32
	// Region merge size in voxels.
	// regionMergeSize = sqrt(regionMergeArea)
	regionMergeSize float32
	// Edge max length in world units
	edgeMaxLen float32
	// Edge max error in voxels
	edgeMaxError float32
	vertsPerPoly float32
	// Detail sample distance in voxels
	detailSampleDist float32
	// Detail sample max error in voxel heights.
	detailSampleMaxError float32
	// Partition type, see SamplePartitionType
	partitionType int32
	// Bounds of the area to mesh
	navMeshBMin [3]float32
	navMeshBMax [3]float32
	// Size of the tiles in voxels
	tileSize float32
}

type InputGeom struct {
	m_chunkyMesh *rcChunkyTriMesh
	m_mesh       *rcMeshLoaderObj

	m_meshBMin, m_meshBMax [3]float32
	m_buildSettings        BuildSettings
	m_hasBuildSettings     bool

	// @name Off-Mesh connections.
	//@{
	m_offMeshConVerts [MAX_OFFMESH_CONNECTIONS * 3 * 2]float32
	m_offMeshConRads  [MAX_OFFMESH_CONNECTIONS]float32
	m_offMeshConDirs  [MAX_OFFMESH_CONNECTIONS]uint8
	m_offMeshConAreas [MAX_OFFMESH_CONNECTIONS]uint8
	m_offMeshConFlags [MAX_OFFMESH_CONNECTIONS]uint8
	m_offMeshConId    [MAX_OFFMESH_CONNECTIONS]uint32
	m_offMeshConCount int32
	//@}

	// @name Convex Volumes.
	//@{
	m_volumes     [MAX_VOLUMES]ConvexVolume
	m_volumeCount int32
	//@}
}

func (ig *InputGeom) load(ctx *rcContext, path string) bool {

	switch filepath.Ext(path) {
	case "obj":
		return ig.loadMesh(ctx, path)
	case "gset":
		//return loadGeomSet(ctx, filepath);
		log.Printf("gset input geometry not implemented")
	}
	return false
}

func (ig *InputGeom) loadMesh(ctx *rcContext, path string) bool {
	if ig.m_mesh != nil {
		ig.m_chunkyMesh = nil
		ig.m_mesh = nil
	}
	ig.m_offMeshConCount = 0
	ig.m_volumeCount = 0

	ig.m_mesh = new(rcMeshLoaderObj)
	if ig.m_mesh == nil {
		ctx.log(RC_LOG_ERROR, "loadMesh: Out of memory 'm_mesh'.")
		return false
	}
	if ig.m_mesh.load(path) != nil {
		ctx.log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", path)
		return false
	}

	rcCalcBounds(ig.m_mesh.m_verts, ig.m_mesh.m_vertCount, ig.m_meshBMin[:], ig.m_meshBMax[:])

	ig.m_chunkyMesh = new(rcChunkyTriMesh)
	if ig.m_chunkyMesh == nil {
		ctx.log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.")
		return false
	}
	if !rcCreateChunkyTriMesh(ig.m_mesh.m_verts, ig.m_mesh.m_tris, ig.m_mesh.m_triCount, 256, ig.m_chunkyMesh) {
		ctx.log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.")
		return false
	}

	return true
}
