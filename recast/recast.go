package recast

import (
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

// Heighfield functions

// CalcBounds calculates the bounding box of an array of vertices.
//  @param[in]		verts	An array of vertices. [(x, y, z) * @p nv]
//  @param[in]		nv		The number of vertices in the @p verts array.
//  @param[out]	bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  @param[out]	bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
// TODO: should return bmin, bmax
func CalcBounds(verts []float32, nv int32, bmin, bmax []float32) {
	// Calculate bounding box.
	copy(bmin, verts[:3])
	copy(bmax, verts[:3])
	for i := int32(0); i < nv; i += 3 {
		v := verts[i : i+3]
		d3.Vec3Min(bmin, v)
		d3.Vec3Max(bmax, v)
	}
}

// CalcGridSize calculates the grid size based on the bounding box and grid cell
// size.

//  @param[in]		bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  @param[in]		bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  @param[in]		cs		The xz-plane cell size. [Limit: > 0] [Units: wu]
//  @param[out]	w		The width along the x-axis. [Limit: >= 0] [Units: vx]
//  @param[out]	h		The height along the z-axis. [Limit: >= 0] [Units: vx]
func CalcGridSize(bmin, bmax [3]float32, cs float32) (w, h int32) {
	w = int32((bmax[0]-bmin[0])/cs + 0.5)
	h = int32((bmax[2]-bmin[2])/cs + 0.5)
	return
}

func calcTriNormal(v0, v1, v2, norm d3.Vec3) {
	d3.Vec3Cross(norm, v1.Sub(v0), v2.Sub(v0))
	norm.Normalize()
}

/// @par
///
/// Only sets the area id's for the walkable triangles.  Does not alter the
/// area id's for unwalkable triangles.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
func MarkWalkableTriangles(ctx *Context, walkableSlopeAngle float32,
	verts []float32, nv int32,
	tris []int32, nt int32,
	areas []uint8) {
	walkableThr := math32.Cosh(walkableSlopeAngle / 180.0 * math32.Pi)

	var norm [3]float32
	for i := int32(0); i < nt; i++ {
		//tri := tris[i*3 : i*3+3]
		tri0, tri1, tri2 := tris[i*3], tris[1+i*3], tris[2+i*3]
		//calcTriNormal(verts[tri[0]*3tri[0]*3], &verts[tri[1]*3], &verts[tri[2]*3], norm);
		calcTriNormal(verts[tri0:tri0+3], verts[tri1:tri1+3], verts[tri2:tri2+3], norm[:])
		// Check if the face is walkable.
		if norm[1] > walkableThr {
			areas[i] = RC_WALKABLE_AREA
		}
	}
}

/// Recast performance timer categories.
/// @see Context
type TimerLabel int

const (
	/// The user defined total time of the build.
	RC_TIMER_TOTAL = iota /// A user defined build time.
	/// A user defined build time.
	RC_TIMER_TEMP
	/// The time to rasterize the triangles. (See: #rcRasterizeTriangle)
	RC_TIMER_RASTERIZE_TRIANGLES
	/// The time to build the compact heightfield. (See: #rcBuildCompactHeightfield)
	RC_TIMER_BUILD_COMPACTHEIGHTFIELD
	/// The total time to build the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS
	/// The time to trace the boundaries of the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS_TRACE
	/// The time to simplify the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS_SIMPLIFY
	/// The time to filter ledge spans. (See: #rcFilterLedgeSpans)
	RC_TIMER_FILTER_BORDER
	/// The time to filter low height spans. (See: #rcFilterWalkableLowHeightSpans)
	RC_TIMER_FILTER_WALKABLE
	/// The time to apply the median filter. (See: #rcMedianFilterWalkableArea)
	RC_TIMER_MEDIAN_AREA
	/// The time to filter low obstacles. (See: #rcFilterLowHangingWalkableObstacles)
	RC_TIMER_FILTER_LOW_OBSTACLES
	/// The time to build the polygon mesh. (See: #rcBuildPolyMesh)
	RC_TIMER_BUILD_POLYMESH
	/// The time to merge polygon meshes. (See: #rcMergePolyMeshes)
	RC_TIMER_MERGE_POLYMESH
	/// The time to erode the walkable area. (See: #rcErodeWalkableArea)
	RC_TIMER_ERODE_AREA
	/// The time to mark a box area. (See: #rcMarkBoxArea)
	RC_TIMER_MARK_BOX_AREA
	/// The time to mark a cylinder area. (See: #rcMarkCylinderArea)
	RC_TIMER_MARK_CYLINDER_AREA
	/// The time to mark a convex polygon area. (See: #rcMarkConvexPolyArea)
	RC_TIMER_MARK_CONVEXPOLY_AREA
	/// The total time to build the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD
	/// The time to build the distances of the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD_DIST
	/// The time to blur the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD_BLUR
	/// The total time to build the regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	RC_TIMER_BUILD_REGIONS
	/// The total time to apply the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_WATERSHED
	/// The time to expand regions while applying the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_EXPAND
	/// The time to flood regions while applying the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_FLOOD
	/// The time to filter out small regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	RC_TIMER_BUILD_REGIONS_FILTER
	/// The time to build heightfield layers. (See: #rcBuildHeightfieldLayers)
	RC_TIMER_BUILD_LAYERS
	/// The time to build the polygon mesh detail. (See: #rcBuildPolyMeshDetail)
	RC_TIMER_BUILD_POLYMESHDETAIL
	/// The time to merge polygon mesh details. (See: #rcMergePolyMeshDetails)
	RC_TIMER_MERGE_POLYMESHDETAIL
	/// The maximum number of timers.  (Used for iterating timers.)
	RC_MAX_TIMERS
)

var (
	xOffset, yOffset [4]int32
)

func init() {
	xOffset = [4]int32{-1, 0, 1, 0}
	yOffset = [4]int32{0, 1, 0, -1}
}

// Gets the standard width (x-axis) offset for the specified direction.
//  @param[in]		dir		The direction. [Limits: 0 <= value < 4]
//  @return The width offset to apply to the current cell position to move
//  	in the direction.
func GetDirOffsetX(dir int) int32 {
	return xOffset[dir&0x03]
}

// Gets the standard height (z-axis) offset for the specified direction.
//  @param[in]		dir		The direction. [Limits: 0 <= value < 4]
//  @return The height offset to apply to the current cell position to move
//  	in the direction.
func GetDirOffsetY(dir int) int32 {
	return yOffset[dir&0x03]
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
