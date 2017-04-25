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

// InputGeom gathers the geometry used as input for navigation mesh building.
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

// LoadOBJMesh loads the geometry from a reader on a OBJ file.
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

// Mesh returns static mesh data.
func (ig *InputGeom) Mesh() *MeshLoaderOBJ {
	return ig.mesh
}

// MeshBoundsMin return the min point of the mesh bounding box.
func (ig *InputGeom) MeshBoundsMin() []float32 {
	return ig.meshBMin[:3]
}

// MeshBoundsMin return the max point of the mesh bounding box.
func (ig *InputGeom) MeshBoundsMax() []float32 {
	return ig.meshBMax[:3]
}

// NavMeshBoundsMin return the min point of the navmesh bounding box.
//
// TODO: currently returns the same point as MeshBoundsMin but it should
// normally depend on the nav mesh build settings
func (ig *InputGeom) NavMeshBoundsMin() []float32 {
	return ig.meshBMin[:3]
}

// NavMeshBoundsMax return the max point of the navmesh bounding box.
//
// TODO: currently returns the same point as MeshBoundsMax but it should
// normally depend on the nav mesh build settings
func (ig *InputGeom) NavMeshBoundsMax() []float32 {
	return ig.meshBMax[:3]
}

// ChunkyMesh returns the underlying chunky triangle mesh.
func (ig *InputGeom) ChunkyMesh() *ChunkyTriMesh {
	return ig.chunkyMesh
}

// ConvexVolumes returns the whole slice of convex volumes added to the input
// geometry
//
// Note: all convex volumes may not be valid, use ConvexVolumesCount in order
// to only access the N first valid convex volumes.
func (ig *InputGeom) ConvexVolumes() []ConvexVolume {
	return ig.volumes[:]
}

// ConvexVolumesCount returns the length of the slice of convex volumes added to the input
// geometry.
func (ig *InputGeom) ConvexVolumesCount() int32 {
	return ig.volumeCount
}

// OffMeshConnectionVerts returns the slice of verts of the off-mesh
// connections.
func (ig *InputGeom) OffMeshConnectionVerts() []float32 {
	return ig.offMeshConVerts[:]
}

// OffMeshConnectionRads returns the slice of orientation, expressed in
// radians, of the off-mesh connections.
func (ig *InputGeom) OffMeshConnectionRads() []float32 {
	return ig.offMeshConRads[:]
}

// OffMeshConnectionRads returns the slice of areas of the off-mesh
// connections.
func (ig *InputGeom) OffMeshConnectionAreas() []uint8 {
	return ig.offMeshConAreas[:]
}

// OffMeshConnectionRads returns the slice of flags of the off-mesh
// connections.
func (ig *InputGeom) OffMeshConnectionFlags() []uint16 {
	return ig.offMeshConFlags[:]
}

// OffMeshConnectionRads returns the slice of identifiers of the off-mesh
// connections.
func (ig *InputGeom) OffMeshConnectionId() []uint32 {
	return ig.offMeshConID[:]
}

// OffMeshConnectionRads returns the slice of directions of the off-mesh
// connections.
func (ig *InputGeom) OffMeshConnectionDirs() []uint8 {
	return ig.offMeshConDirs[:]
}

// OffMeshConnectionRads returns the length of the slice of the off-mesh
// connections.
func (ig *InputGeom) OffMeshConnectionCount() int32 {
	return ig.offMeshConCount
}

// AddConvexVolume adds a new convex volume to the input geometry.
//
// The convex volume is defined by the verts slice [x, y, z] * Number of
// vertices.
func (ig *InputGeom) AddConvexVolume(verts []float32, minh, maxh float32, area uint8) {
	if ig.volumeCount >= maxVolumes {
		return
	}
	vol := &ig.volumes[ig.volumeCount]
	ig.volumeCount++
	copy(vol.Verts[:], verts)
	vol.HMin = minh
	vol.HMax = maxh
	vol.NVerts = int32(len(verts))
	vol.Area = int32(area)
}

// DeleteConvexVolume deletes the ith convex volume.
func (ig *InputGeom) DeleteConvexVolume(i int) {
	ig.volumeCount--
	// copy last volume over the deleted one
	ig.volumes[i] = ig.volumes[ig.volumeCount]
}
