package detour

import "github.com/aurelien-rainone/gogeo/f32/d3"

/// Defines polygon filtering and traversal costs for navigation mesh query operations.
/// @ingroup detour
/// @class dtQueryFilter
///
/// <b>The Default Implementation</b>
///
/// At construction: All area costs default to 1.0.  All flags are included
/// and none are excluded.
///
/// If a polygon has both an include and an exclude flag, it will be excluded.
///
/// The way filtering works, a navigation mesh polygon must have at least one flag
/// set to ever be considered by a query. So a polygon with no flags will never
/// be considered.
///
/// Setting the include flags to 0 will result in all polygons being excluded.
///
/// <b>Custom Implementations</b>
///
/// DT_VIRTUAL_QUERYFILTER must be defined in order to extend this class.
///
/// Implement a custom query filter by overriding the virtual passFilter()
/// and getCost() functions. If this is done, both functions should be as
/// fast as possible. Use cached local copies of data rather than accessing
/// your own objects where possible.
///
/// Custom implementations do not need to adhere to the flags or cost logic
/// used by the default implementation.
///
/// In order for A* searches to work properly, the cost should be proportional to
/// the travel distance. Implementing a cost modifier less than 1.0 is likely
/// to lead to problems during pathfinding.
///
/// @see dtNavMeshQuery
type DtQueryFilter struct {
	///< Cost per area type. (Used by default implementation.)
	areaCost [DT_MAX_AREAS]float32
	///< Flags for polygons that can be visited. (Used by default implementation.)
	includeFlags uint16
	///< Flags for polygons that should not be visted. (Used by default implementation.)
	excludeFlags uint16
}

func NewDtQueryFilter() *DtQueryFilter {
	qf := DtQueryFilter{
		includeFlags: 0xffff,
		excludeFlags: 0,
	}
	for i := int32(0); i < DT_MAX_AREAS; i++ {
		qf.areaCost[i] = 1.0
	}
	return &qf
}

/// Returns the traversal cost of the area.
///  @param[in]		i		The id of the area.
/// @returns The traversal cost of the area.
func (qf *DtQueryFilter) AreaCost(i int32) float32 { return qf.areaCost[i] }

/// Sets the traversal cost of the area.
///  @param[in]		i		The id of the area.
///  @param[in]		cost	The new cost of traversing the area.
func (qf *DtQueryFilter) SetAreaCost(i int32, cost float32) { qf.areaCost[i] = cost }

/// Returns the include flags for the filter.
/// Any polygons that include one or more of these flags will be
/// included in the operation.
func (qf *DtQueryFilter) IncludeFlags() uint16 { return qf.includeFlags }

/// Sets the include flags for the filter.
/// @param[in]		flags	The new flags.
func (qf *DtQueryFilter) SetIncludeFlags(flags uint16) { qf.includeFlags = flags }

/// Returns the exclude flags for the filter.
/// Any polygons that include one ore more of these flags will be
/// excluded from the operation.
func (qf *DtQueryFilter) ExcludeFlags() uint16 { return qf.excludeFlags }

/// Sets the exclude flags for the filter.
/// @param[in]		flags		The new flags.
func (qf *DtQueryFilter) SetExcludeFlags(flags uint16) { qf.excludeFlags = flags }

/// Returns true if the polygon can be visited.  (I.e. Is traversable.)
///  @param[in]		ref		The reference id of the polygon test.
///  @param[in]		tile	The tile containing the polygon.
///  @param[in]		poly  The polygon to test.
func (qf *DtQueryFilter) passFilter(ref DtPolyRef, tile *DtMeshTile, poly *DtPoly) bool {
	return (poly.Flags&qf.includeFlags) != 0 && (poly.Flags&qf.excludeFlags) == 0
}

/// Returns cost to move from the beginning to the end of a line segment
/// that is fully contained within a polygon.
///  @param[in]		pa			The start position on the edge of the previous and current polygon. [(x, y, z)]
///  @param[in]		pb			The end position on the edge of the current and next polygon. [(x, y, z)]
///  @param[in]		prevRef		The reference id of the previous polygon. [opt]
///  @param[in]		prevTile	The tile containing the previous polygon. [opt]
///  @param[in]		prevPoly	The previous polygon. [opt]
///  @param[in]		curRef		The reference id of the current polygon.
///  @param[in]		curTile		The tile containing the current polygon.
///  @param[in]		curPoly		The current polygon.
///  @param[in]		nextRef		The refernece id of the next polygon. [opt]
///  @param[in]		nextTile	The tile containing the next polygon. [opt]
///  @param[in]		nextPoly	The next polygon. [opt]

func (qf *DtQueryFilter) Cost(pa, pb d3.Vec3,
	prevRef DtPolyRef, prevTile *DtMeshTile, prevPoly *DtPoly,
	curRef DtPolyRef, curTile *DtMeshTile, curPoly *DtPoly,
	nextRef DtPolyRef, nextTile *DtMeshTile, nextPoly *DtPoly) float32 {

	return pa.Dist(pb) * qf.areaCost[curPoly.Area()]
}
