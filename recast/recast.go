package recast

import (
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

// Heighfield functions

// CalcBounds calculates the bounding box of an array of vertices.
//
//  Arguments:
//  verts     An array of vertices. [(x, y, z) * nv]
//  nv        The number of vertices in the verts array.
//  bmin      The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  bmax      The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
func CalcBounds(verts []float32, nv int32, bmin, bmax []float32) {
	// Calculate bounding box.
	copy(bmin, verts[:3])
	copy(bmax, verts[:3])

	var v []float32
	for i := int32(1); i < nv; i++ {
		v = verts[i*3:]
		d3.Vec3Min(bmin, v)
		d3.Vec3Max(bmax, v)
	}
}

// CalcGridSize calculates the grid size based on the bounding box and grid cell
// size.
//
//  Arguments:
//   bmin      The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//   bmax      The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//   cs        The xz-plane cell size. [Limit: > 0] [Units: wu]
//
// 	Returns:
//   w         The width along the x-axis. [Limit: >= 0] [Units: vx]
//   h         The height along the z-axis. [Limit: >= 0] [Units: vx]
func CalcGridSize(bmin, bmax [3]float32, cs float32) (w, h int32) {
	w = int32((bmax[0]-bmin[0])/cs + 0.5)
	h = int32((bmax[2]-bmin[2])/cs + 0.5)
	return
}

func calcTriNormal(v0, v1, v2, norm d3.Vec3) {
	d3.Vec3Cross(norm, v1.Sub(v0), v2.Sub(v0))
	norm.Normalize()
}

// Sets the area id of all triangles with a slope below the specified value
// to #RC_WALKABLE_AREA.
//
//  Arguments:
//   ctx                The build context to use during the operation.
//   walkableSlopeAngle The maximum slope that is considered walkable.
//                      [Limits: 0 <= value < 90] [Units: Degrees]
//   verts              The vertices. [(x, y, z) * nv]
//   nv                 The number of vertices.
//   tris               The triangle vertex indices. [(vertA, vertB, vertC) * nt]
//   nt                 The number of triangles.
//   areas              The triangle area ids. [Length: >= nt]
//
// Only sets the area id's for the walkable triangles. Does not alter the area
// id's for unwalkable triangles.
//
// See the cConfig documentation for more information on the configuration
// parameters.
//
// see Heightfield, ClearUnwalkableTriangles, RasterizeTriangles
func MarkWalkableTriangles(ctx *BuildContext, walkableSlopeAngle float32,
	verts []float32, nv int32,
	tris []int32, nt int32,
	areas []uint8) {
	walkableThr := math32.Cos(walkableSlopeAngle / 180.0 * math32.Pi)

	var norm [3]float32
	for i := int32(0); i < nt; i++ {
		tri := tris[i*3:]
		calcTriNormal(verts[tri[0]*3:], verts[tri[1]*3:], verts[tri[2]*3:], norm[:])
		// Check if the face is walkable.
		if norm[1] > walkableThr {
			areas[i] = WalkableArea
		}
	}
}

// ClearUnwalkableTriangles sets the area id of all triangles with a slope
// greater than or equal to the specified value to RC_NULL_AREA.
//
//  Arguments:
//   ctx                The build context to use during the operation.
//   walkableSlopeAngle The maximum slope that is considered walkable.
//                      [Limits: 0 <= value < 90] [Units: Degrees]
//   verts              The vertices. [(x, y, z) * nv]
//   nv                 The number of vertices.
//   tris               The triangle vertex indices. [(vertA, vertB, vertC) * nt]
//   nt                 The number of triangles.
//   areas              The triangle area ids. [Length: >= nt]
//
// Only sets the area id's for the unwalkable triangles. Does not alter the area
// id's for walkable triangles.
//
// See the Config documentation for more information on the configuration
// parameters.
//
// see Heightfield, ClearUnwalkableTriangles, RasterizeTriangles
func ClearUnwalkableTriangles(ctx *BuildContext, walkableSlopeAngle float32,
	verts []float32, nv int32,
	tris []int32, nt int32,
	areas []uint8) {
	walkableThr := math32.Cos(walkableSlopeAngle / 180.0 * math32.Pi)

	var norm [3]float32

	for i := int32(0); i < nt; i++ {
		tri := tris[i*3:]
		calcTriNormal(verts[tri[0]*3:], verts[tri[1]*3:], verts[tri[2]*3:], norm[:])
		// Check if the face is walkable.
		if norm[1] <= walkableThr {
			areas[i] = nullArea
		}
	}
}

// Recast performance timer categories.
// see BuildContext
type TimerLabel int

const (
	// The user defined total time of the build.
	TimerTotal = iota
	// A user defined build time.
	TimerTemp
	// The time to rasterize the triangles. (See: RasterizeTriangle)
	TimerRasterizeTriangles
	// The time to build the compact heightfield. (See: BuildCompactHeightfield)
	TimerBuildCompactHeightfield
	// The total time to build the contours. (See: BuildContours)
	TimerBuildContours
	// The time to trace the boundaries of the contours. (See: BuildContours)
	TimerBuildContoursTrace
	// The time to simplify the contours. (See: BuildContours)
	TimerBuildContoursSimplify
	// The time to filter ledge spans. (See: FilterLedgeSpans)
	TimerFilterBorder
	// The time to filter low height spans. (See: FilterWalkableLowHeightSpans)
	TimerFilterWalkable
	// The time to apply the median filter. (See: MedianFilterWalkableArea)
	TimerMedianArea
	// The time to filter low obstacles. (See: FilterLowHangingWalkableObstacles)
	TimerFilterLowObstacles
	// The time to build the polygon mesh. (See: BuildPolyMesh)
	TimerBuildPolymesh
	// The time to merge polygon meshes. (See: MergePolyMeshes)
	TimerMergePolymesh
	// The time to erode the walkable area. (See: ErodeWalkableArea)
	TimerErodeArea
	// The time to mark a box area. (See: MarkBoxArea)
	TimerMarkBoxArea
	// The time to mark a cylinder area. (See: MarkCylinderArea)
	TimerMarkCylinderArea
	// The time to mark a convex polygon area. (See: MarkConvexPolyArea)
	TimerMarkConvexPolyArea
	// The total time to build the distance field. (See: BuildDistanceField)
	TimerBuildDistanceField
	// The time to build the distances of the distance field. (See: BuildDistanceField)
	TimerBuildDistanceFieldDist
	// The time to blur the distance field. (See: BuildDistanceField)
	TimerBuildDistanceFieldBlur
	// The total time to build the regions. (See: BuildRegions, BuildRegionsMonotone)
	TimerBuildRegions
	// The total time to apply the watershed algorithm. (See: BuildRegions)
	TimerBuildRegionsWatershed
	// The time to expand regions while applying the watershed algorithm. (See: BuildRegions)
	TimerBuildRegionsExpand
	// The time to flood regions while applying the watershed algorithm. (See: BuildRegions)
	TimerBuildRegionsFlood
	// The time to filter out small regions. (See: BuildRegions, BuildRegionsMonotone)
	TimerBuildRegionsFilter
	// The time to build heightfield layers. (See: BuildHeightfieldLayers)
	TimerBuildLayers
	// The time to build the polygon mesh detail. (See: BuildPolyMeshDetail)
	TimerBuildPolyMeshDetail
	// The time to merge polygon mesh details. (See: MergePolyMeshDetails)
	TimerMergePolyMeshDetail
	// The maximum number of timers. (Used for iterating timers.)
	maxTimers
)

var (
	xOffset, yOffset [4]int32
	dirOffset        [5]int32
)

func init() {
	xOffset = [4]int32{-1, 0, 1, 0}
	yOffset = [4]int32{0, 1, 0, -1}
	dirOffset = [5]int32{3, 0, -1, 2, 1}
}

// SetCon sets the neighbor connection data for the specified direction.
//
//  Arguments:
//   s        The span to update.
//   dir      The direction to set. [Limits: 0 <= value < 4]
//   i        The index of the neighbor span.
func SetCon(s *CompactSpan, dir, i int32) {
	shift := uint32(uint32(dir * 6))
	con := uint32(s.Con)
	s.Con = (con ^ (0x3f << shift)) | ((uint32(i & 0x3f)) << shift)
}

// GetCon gets neighbor connection data for the specified direction.
//
//  Arguments:
//   s       The span to check.
//   dir     The direction to check. [Limits: 0 <= value < 4]
//
// Return the neighbor connection data for the specified direction,
// RC_NOT_CONNECTED if there is no connection.
func GetCon(s *CompactSpan, dir int32) int32 {
	shift := uint32(dir * 6)
	return int32((s.Con >> shift) & 0x3f)
}

// GetDirOffsetX gets the standard width (x-axis) offset for the specified
// direction.
//
//  Arguments:
//   dir     The direction. [Limits: 0 <= value < 4]
//
// Return the width offset to apply to the current cell position to move in the
// direction.
func GetDirOffsetX(dir int32) int32 {
	return xOffset[dir&0x03]
}

// GetDirOffsetY gets the standard height (z-axis) offset for the specified
// direction.
//
//  Arguments:
//   dir     The direction. [Limits: 0 <= value < 4]
//
// Return the height offset to apply to the current cell position to move in the
// direction.
func GetDirOffsetY(dir int32) int32 {
	return yOffset[dir&0x03]
}

// GetDirForOffset gets the direction for the specified offset. One of x and y
// should be 0.
//
//  Arguments:
//   x       The x offset. [Limits: -1 <= value <= 1]
//   y       The y offset. [Limits: -1 <= value <= 1]
//
// Return the direction that represents the offset.
func GetDirForOffset(x, y int32) int32 {
	return dirOffset[((y+1)<<1)+x]
}

func iMin(a, b int32) int32 {
	if a < b {
		return a
	}
	return b
}

func iMax(a, b int32) int32 {
	if a > b {
		return a
	}
	return b
}

func iAbs(a int32) int32 {
	if a < 0 {
		return -a
	}
	return a
}
