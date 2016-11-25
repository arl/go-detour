package detour

import "github.com/aurelien-rainone/gogeo/f32/d3"

// QueryFilter defines polygon filtering and traversal
// costs for navigation mesh query operations.
//
// Implement a custom query filter by overriding the virtual PassFilter() and
// Cost() functions. If this is done, both functions should be as fast as
// possible. Use cached local copies of data rather than accessing your own
// objects where possible.
//
// Custom implementations do not need to adhere to the flags or cost logic used
// by the default implementation.
//
// In order for A* searches to work properly, the cost should be proportional to
// the travel distance. Implementing a cost modifier less than 1.0 is likely to
// lead to problems during pathfinding.
type QueryFilter interface {

	// PassFilter returns true if the polygon can be visited.  (I.e. Is traversable.)
	//
	//  Arguments:
	//   ref     The reference id of the polygon test.
	//   tile    The tile containing the polygon.
	//   poly    The polygon to test.
	PassFilter(ref PolyRef, tile *MeshTile, poly *Poly) bool

	// Cost returns cost to move from the beginning to the end of a line segment
	// that is fully contained within a polygon.
	//
	//  Arguments:
	//   pa       The start position on the edge of the previous and current polygon. [(x, y, z)]
	//   pb       The end position on the edge of the current and next polygon. [(x, y, z)]
	//   prevRef  The reference id of the previous polygon.
	//   prevTile The tile containing the previous polygon.
	//   prevPoly The previous polygon.
	//   curRef   The reference id of the current polygon.
	//   curTile  The tile containing the current polygon.
	//   curPoly  The current polygon.
	//   nextRef  The reference id of the next polygon.
	//   nextTile The tile containing the next polygon.
	//   nextPoly The next polygon.
	Cost(pa, pb d3.Vec3,
		prevRef PolyRef, prevTile *MeshTile, prevPoly *Poly,
		curRef PolyRef, curTile *MeshTile, curPoly *Poly,
		nextRef PolyRef, nextTile *MeshTile, nextPoly *Poly) float32
}

// StandardQueryFilter is a standard implementation of the QueryFilter
// interface.
//
// At construction all area costs default to 1.0. All flags are included and
// none are excluded.
// If a polygon has both an include and an exclude flag, it will be excluded.
//
// The way filtering works, a navigation mesh polygon must have at least one
// flag set to ever be considered by a query. So a polygon with no flags will
// never be considered. Setting the include flags to 0 will result in all
// polygons being excluded.
//
// see NavMeshQuery
type StandardQueryFilter struct {
	// Cost per area type.
	areaCost [maxAreas]float32

	// Flags for polygons that can be visited.
	includeFlags uint16

	// Flags for polygons that should not be visted
	excludeFlags uint16
}

// NewStandardQueryFilter initializes a new standard query filter.
func NewStandardQueryFilter() *StandardQueryFilter {
	qf := StandardQueryFilter{
		includeFlags: 0xffff,
		excludeFlags: 0,
	}
	for i := int32(0); i < maxAreas; i++ {
		qf.areaCost[i] = 1.0
	}
	return &qf
}

// AreaCost returns the traversal cost of the area which id is i.
func (qf *StandardQueryFilter) AreaCost(i int32) float32 { return qf.areaCost[i] }

// SetAreaCost sets the traversal cost of the area which id is i.
func (qf *StandardQueryFilter) SetAreaCost(i int32, cost float32) { qf.areaCost[i] = cost }

// IncludeFlags returns the include flags for the filter.
//
// Any polygons that include one or more of these flags will be
// included in the operation.
func (qf *StandardQueryFilter) IncludeFlags() uint16 { return qf.includeFlags }

// SetIncludeFlags sets the include flags for the filter.
func (qf *StandardQueryFilter) SetIncludeFlags(flags uint16) { qf.includeFlags = flags }

// ExcludeFlags returns the exclude flags for the filter.
//
// Any polygons that include one ore more of these flags will be
// excluded from the operation.
func (qf *StandardQueryFilter) ExcludeFlags() uint16 { return qf.excludeFlags }

// SetExcludeFlags sets the exclude flags for the filter.
func (qf *StandardQueryFilter) SetExcludeFlags(flags uint16) { qf.excludeFlags = flags }

// PassFilter returns true if the polygon can be visited.  (I.e. Is traversable.)
//
//  Arguments:
//   ref     The reference id of the polygon test.
//   tile    The tile containing the polygon.
//   poly    The polygon to test.
func (qf *StandardQueryFilter) PassFilter(ref PolyRef, tile *MeshTile, poly *Poly) bool {
	return (poly.Flags&qf.includeFlags) != 0 && (poly.Flags&qf.excludeFlags) == 0
}

// Cost returns cost to move from the beginning to the end of a line segment
// that is fully contained within a polygon.
//
//  Arguments:
//   pa       The start position on the edge of the previous and current polygon. [(x, y, z)]
//   pb       The end position on the edge of the current and next polygon. [(x, y, z)]
//   prevRef  The reference id of the previous polygon.
//   prevTile The tile containing the previous polygon.
//   prevPoly The previous polygon.
//   curRef   The reference id of the current polygon.
//   curTile  The tile containing the current polygon.
//   curPoly  The current polygon.
//   nextRef  The reference id of the next polygon.
//   nextTile The tile containing the next polygon.
//   nextPoly The next polygon.
func (qf *StandardQueryFilter) Cost(pa, pb d3.Vec3,
	prevRef PolyRef, prevTile *MeshTile, prevPoly *Poly,
	curRef PolyRef, curTile *MeshTile, curPoly *Poly,
	nextRef PolyRef, nextTile *MeshTile, nextPoly *Poly) float32 {

	return pa.Dist(pb) * qf.areaCost[curPoly.Area()]
}
