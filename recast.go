package detour

import "github.com/aurelien-rainone/gogeo/f32/d3"

func rcCalcBounds(verts []float32, nv int32, bmin, bmax []float32) {
	// Calculate bounding box.
	bmin = verts[:3]
	bmax = verts[:3]
	for i := int32(1); i < nv; i++ {
		v := verts[i*3 : 3+i*3]
		d3.Vec3Min(bmin, v)
		d3.Vec3Min(bmax, v)
	}
}

/// Recast performance timer categories.
/// @see rcContext
type rcTimerLabel int

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
