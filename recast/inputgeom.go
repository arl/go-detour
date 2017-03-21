package recast

import (
	"fmt"
	"io"
)

const (
	maxOffMeshConnections = 256
	maxVolumes            = 256
	maxConvexVolPts       = 12
)

type ConvexVolume struct {
	Verts      [maxConvexVolPts * 3]float32
	HMin, HMax float32
	NVerts     int32
	Area       int32
}

type BuildSettings struct {
	// Cell size in world units
	CellSize float32

	// Cell height in world units
	CellHeight float32

	// Agent height in world units
	AgentHeight float32

	// Agent radius in world units
	AgentRadius float32

	// Agent max climb in world units
	AgentMaxClimb float32

	// Agent max slope in degrees
	AgentMaxSlope float32

	// Region minimum size in voxels.
	// regionMinSize = sqrt(regionMinArea)
	RegionMinSize float32

	// Region merge size in voxels.
	// regionMergeSize = sqrt(regionMergeArea)
	RegionMergeSize float32

	// Edge max length in world units
	EdgeMaxLen float32

	// Edge max error in voxels
	EdgeMaxError float32

	// VertsPerPolys is the number of vertices to consider per polygons
	VertsPerPoly float32

	// Detail sample distance in voxels
	DetailSampleDist float32

	// Detail sample max error in voxel heights
	DetailSampleMaxError float32

	// Partition type, see SamplePartitionType
	PartitionType int32

	// Size of the tiles in voxels
	TileSize float32
}

type InputGeom struct {
	chunkyMesh *ChunkyTriMesh
	mesh       *MeshLoaderOBJ

	meshBMin, meshBMax [3]float32

	// Off-Mesh connections.
	offMeshConVerts [maxOffMeshConnections * 3 * 2]float32
	offMeshConRads  [maxOffMeshConnections]float32
	offMeshConDirs  [maxOffMeshConnections]uint8
	offMeshConAreas [maxOffMeshConnections]uint8
	offMeshConFlags [maxOffMeshConnections]uint16
	offMeshConID    [maxOffMeshConnections]uint32
	offMeshConCount int32

	// Convex Volumes.
	volumes     [maxVolumes]ConvexVolume
	volumeCount int32
}

func (ig *InputGeom) LoadOBJMesh(r io.Reader) error {
	var err error
	if ig.mesh != nil {
		ig.chunkyMesh = nil
		ig.mesh = nil
	}
	ig.offMeshConCount = 0
	ig.volumeCount = 0

	ig.mesh = NewMeshLoaderOBJ()
	if err = ig.mesh.Load(r); err != nil {
		return err
	}

	CalcBounds(ig.mesh.Verts(), ig.mesh.VertCount(), ig.meshBMin[:], ig.meshBMax[:])

	ig.chunkyMesh = new(ChunkyTriMesh)
	if !createChunkyTriMesh(ig.mesh.Verts(), ig.mesh.Tris(), ig.mesh.TriCount(), 256, ig.ChunkyMesh()) {
		return fmt.Errorf("failed to build chunky mesh")
	}

	return nil
}

// Method to return static mesh data.
func (ig *InputGeom) Mesh() *MeshLoaderOBJ {
	return ig.mesh
}

func (ig *InputGeom) MeshBoundsMin() []float32 {
	return ig.meshBMin[:3]
}

func (ig *InputGeom) MeshBoundsMax() []float32 {
	return ig.meshBMax[:3]
}

func (ig *InputGeom) NavMeshBoundsMin() []float32 {
	return ig.meshBMin[:3]
}

func (ig *InputGeom) NavMeshBoundsMax() []float32 {
	return ig.meshBMax[:3]
}

func (ig *InputGeom) ChunkyMesh() *ChunkyTriMesh {
	return ig.chunkyMesh
}

func (ig *InputGeom) ConvexVolumes() []ConvexVolume {
	return ig.volumes[:]
}

func (ig *InputGeom) ConvexVolumesCount() int32 {
	return ig.volumeCount
}

func (ig *InputGeom) OffMeshConnectionVerts() []float32 {
	return ig.offMeshConVerts[:]
}

func (ig *InputGeom) OffMeshConnectionRads() []float32 {
	return ig.offMeshConRads[:]
}

func (ig *InputGeom) OffMeshConnectionAreas() []uint8 {
	return ig.offMeshConAreas[:]
}

func (ig *InputGeom) OffMeshConnectionFlags() []uint16 {
	return ig.offMeshConFlags[:]
}

func (ig *InputGeom) OffMeshConnectionId() []uint32 {
	return ig.offMeshConID[:]
}

func (ig *InputGeom) OffMeshConnectionDirs() []uint8 {
	return ig.offMeshConDirs[:]
}

func (ig *InputGeom) OffMeshConnectionCount() int32 {
	return ig.offMeshConCount
}

func (ig *InputGeom) addConvexVolume(verts []float32, nverts int, minh, maxh float32, area uint8) {
	if ig.volumeCount >= maxVolumes {
		return
	}
	vol := &ig.volumes[ig.volumeCount]
	ig.volumeCount++
	copy(vol.Verts[:], verts[3*nverts:])
	vol.HMin = minh
	vol.HMax = maxh
	vol.NVerts = int32(nverts)
	vol.Area = int32(area)
}

func (ig *InputGeom) deleteConvexVolume(i int) {
	ig.volumeCount--
	// copy last volume over the deleted one
	ig.volumes[i] = ig.volumes[ig.volumeCount]
}
