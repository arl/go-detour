package recast

import (
	"fmt"
	"path/filepath"
)

const (
	MAX_OFFMESH_CONNECTIONS = 256
	MAX_VOLUMES             = 256
	MAX_CONVEXVOL_PTS       = 12
)

type ConvexVolume struct {
	Verts      [MAX_CONVEXVOL_PTS * 3]float32
	HMin, HMax float32
	NVerts     int32
	Area       int32
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
	m_chunkyMesh *ChunkyTriMesh
	m_mesh       *MeshLoaderObj

	m_meshBMin, m_meshBMax [3]float32
	m_buildSettings        BuildSettings
	m_hasBuildSettings     bool

	// Off-Mesh connections.
	m_offMeshConVerts [MAX_OFFMESH_CONNECTIONS * 3 * 2]float32
	m_offMeshConRads  [MAX_OFFMESH_CONNECTIONS]float32
	m_offMeshConDirs  [MAX_OFFMESH_CONNECTIONS]uint8
	m_offMeshConAreas [MAX_OFFMESH_CONNECTIONS]uint8
	m_offMeshConFlags [MAX_OFFMESH_CONNECTIONS]uint16
	m_offMeshConId    [MAX_OFFMESH_CONNECTIONS]uint32
	m_offMeshConCount int32

	// Convex Volumes.
	m_volumes     [MAX_VOLUMES]ConvexVolume
	m_volumeCount int32
}

func (ig *InputGeom) Load(ctx *BuildContext, path string) error {

	switch filepath.Ext(path) {
	case ".obj":
		return ig.loadMesh(ctx, path)
	case ".gset":
		return fmt.Errorf("gset input geometry not implemented")
	}
	return fmt.Errorf("couldn't recognize input geometry file extension: '%s'", path)
}

func (ig *InputGeom) loadMesh(ctx *BuildContext, path string) error {
	var err error
	if ig.m_mesh != nil {
		ig.m_chunkyMesh = nil
		ig.m_mesh = nil
	}
	ig.m_offMeshConCount = 0
	ig.m_volumeCount = 0

	ig.m_mesh = NewMeshLoaderObj()
	if err = ig.m_mesh.Load(path); err != nil {
		return fmt.Errorf("could not load '%s'", err)
	}

	CalcBounds(ig.m_mesh.Verts(), ig.m_mesh.VertCount(), ig.m_meshBMin[:], ig.m_meshBMax[:])

	ig.m_chunkyMesh = new(ChunkyTriMesh)
	if !CreateChunkyTriMesh(ig.m_mesh.Verts(), ig.m_mesh.Tris(), ig.m_mesh.TriCount(), 256, ig.ChunkyMesh()) {
		return fmt.Errorf("failed to build chunky mesh for '%s'", path)
	}

	return nil
}

// Method to return static mesh data.
func (ig *InputGeom) Mesh() *MeshLoaderObj {
	return ig.m_mesh
}

func (ig *InputGeom) MeshBoundsMin() [3]float32 {
	return ig.m_meshBMin
}

func (ig *InputGeom) MeshBoundsMax() [3]float32 {
	return ig.m_meshBMax
}

func (ig *InputGeom) NavMeshBoundsMin() [3]float32 {
	if ig.m_hasBuildSettings {
		return ig.m_buildSettings.navMeshBMin
	} else {
		return ig.m_meshBMin
	}
}

func (ig *InputGeom) NavMeshBoundsMax() [3]float32 {
	if ig.m_hasBuildSettings {
		return ig.m_buildSettings.navMeshBMax
	} else {
		return ig.m_meshBMax
	}
}

func (ig *InputGeom) ChunkyMesh() *ChunkyTriMesh {
	return ig.m_chunkyMesh
}

func (ig *InputGeom) BuildSettings() *BuildSettings {
	if ig.m_hasBuildSettings {
		return &ig.m_buildSettings
	}
	return nil
}

func (ig *InputGeom) ConvexVolumes() []ConvexVolume {
	return ig.m_volumes[:]
}

func (ig *InputGeom) ConvexVolumesCount() int32 {
	return ig.m_volumeCount
}

func (ig *InputGeom) OffMeshConnectionVerts() []float32 {
	return ig.m_offMeshConVerts[:]
}

func (ig *InputGeom) OffMeshConnectionRads() []float32 {
	return ig.m_offMeshConRads[:]
}

func (ig *InputGeom) OffMeshConnectionAreas() []uint8 {
	return ig.m_offMeshConAreas[:]
}

func (ig *InputGeom) OffMeshConnectionFlags() []uint16 {
	return ig.m_offMeshConFlags[:]
}

func (ig *InputGeom) OffMeshConnectionId() []uint32 {
	return ig.m_offMeshConId[:]
}

func (ig *InputGeom) OffMeshConnectionDirs() []uint8 {
	return ig.m_offMeshConDirs[:]
}

func (ig *InputGeom) OffMeshConnectionCount() int32 {
	return ig.m_offMeshConCount
}
