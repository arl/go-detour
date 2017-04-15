package detour

import (
	"fmt"
	"log"
	"math"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

const (
	// HScale is the search heuristic scale.
	HScale float32 = 0.999
)

// Raycast should calculate movement cost along the ray and fill RaycastHit.Cost
// RaycastOptions
const RaycastUseCosts int = 0x01

// RaycastHit provides information about a raycast hit
// filled by NavMeshQuery.Raycast
type RaycastHit struct {
	// The hit parameter. (math.MaxFloat32 if no wall hit.)
	T float32

	// The normal of the nearest wall hit. [(x, y, z)]
	HitNormal d3.Vec3

	// The index of the edge on the final polygon where the wall was hit.
	HitEdgeIndex int

	// Pointer to an array of reference ids of the visited polygons. [opt]
	Path []PolyRef

	// The number of visited polygons. [opt]
	PathCount int

	// The maximum number of polygons the @p path array can hold.
	MaxPath int

	//  The cost of the path until hit.
	PathCost float32
}

// NavMeshQuery provides the ability to perform pathfinding related queries
// against a navigation mesh.
//
// For methods that support undersized slices, if the slices is too small to
// hold the entire result set the return status of the method will include the
// BufferTooSmall flag.
//
// Some methods can be used by multiple clients without side effects. (E.g. No
// change to the closed list. No impact on an in-progress sliced path query.
// Etc.). When that is the case it will be clearly stated in the method comment.
//
// Walls and portals: A wall is a polygon segment that is considered impassable.
// A portal is a passable segment between polygons. A portal may be treated as a
// wall based on the QueryFilter used for a query.
//
// see NavMesh, QueryFilter, NewNavMeshQuery()
type NavMeshQuery struct {
	nav          *NavMesh   // Pointer to navmesh data.
	query        queryData  // Sliced query state.
	tinyNodePool *NodePool  // Pointer to small node pool.
	nodePool     *NodePool  // Pointer to node pool.
	openList     *nodeQueue // Pointer to open list queue.
}

type queryData struct {
	status           Status
	lastBestNode     *Node
	lastBestNodeCost float32
	startRef, endRef PolyRef
	startPos, endPos d3.Vec3
	filter           QueryFilter
	options          uint32
	raycastLimitSqr  float32
}

// NewNavMeshQuery initializes the query object.
//
//  Arguments:
//   nav       Pointer to the NavMesh object to use for all queries.
//   maxNodes  Maximum number of search nodes. [Limits: 0 < value <= 65535]
//
// Return the status flags for the initialization of the query object and the
// query object.
//
// Must be the first function called after construction, before other
// functions are used.
// This function can be used multiple times.
func NewNavMeshQuery(nav *NavMesh, maxNodes int32) (Status, *NavMeshQuery) {
	if maxNodes > int32(nullIdx) || maxNodes > int32(1<<nodeParentBits)-1 {
		return Failure | InvalidParam, nil
	}

	q := &NavMeshQuery{}
	q.nav = nav

	if q.nodePool == nil || q.nodePool.MaxNodes() < maxNodes {
		if q.nodePool != nil {
			q.nodePool = nil
		}
		q.nodePool = newNodePool(maxNodes, int32(math32.NextPow2(uint32(maxNodes/4))))
		if q.nodePool == nil {
			return Failure | OutOfMemory, nil
		}
	} else {
		q.nodePool.Clear()
	}

	if q.tinyNodePool == nil {
		q.tinyNodePool = newNodePool(64, 32)
		if q.tinyNodePool == nil {
			return Failure | OutOfMemory, nil
		}
	} else {
		q.tinyNodePool.Clear()
	}

	if q.openList == nil || q.openList.capacity < maxNodes {
		if q.openList != nil {
			q.openList = nil
		}
		q.openList = newnodeQueue(maxNodes)
		if q.openList == nil {
			return Failure | OutOfMemory, nil
		}
	} else {
		q.openList.clear()
	}

	return Success, q
}

// FindPath finds a path from the start polygon to the end polygon.
//
//  Arguments:
//   startRef  The reference id of the start polygon.
//   endRef    The reference id of the end polygon.
//   startPos  A position within the start polygon. [(x, y, z)]
//   endPos    A position within the end polygon. [(x, y, z)]
//   filter    The polygon filter to apply to the query.
//   path      This slice will be filled with an ordered list of polygon
//             references representing the path. (Start to end.)
//
//  Returns:
//   pathCount the number of polygons in the found path slice.
//   st        status code (may be a partial result)
//
// If the end polygon cannot be reached through the navigation graph, the last
// polygon in the path will be the nearest to the end polygon. If the path array
// is to small to hold the full result, it will be filled as far as possible
// from the start polygon toward the end polygon.
//
// The start and end positions are used to calculate traversal costs.
// (The y-values impact the result.)
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) FindPath(
	startRef, endRef PolyRef,
	startPos, endPos d3.Vec3,
	filter QueryFilter,
	path []PolyRef) (pathCount int, st Status) {
	// Validate input
	if !q.nav.IsValidPolyRef(startRef) || !q.nav.IsValidPolyRef(endRef) ||
		len(startPos) < 3 || len(endPos) < 3 || filter == nil || path == nil || len(path) == 0 {
		return pathCount, Failure | InvalidParam
	}

	if startRef == endRef {
		path[0] = startRef
		return 1, Success
	}

	q.nodePool.Clear()
	q.openList.clear()

	var (
		startNode, lastBestNode *Node
		lastBestNodeCost        float32
	)
	startNode = q.nodePool.Node(startRef, 0)
	startNode.Pos.Assign(startPos)
	startNode.PIdx = 0
	startNode.Cost = 0
	startNode.Total = startPos.Dist(endPos) * HScale
	startNode.ID = startRef
	startNode.Flags = nodeOpen
	q.openList.push(startNode)

	lastBestNode = startNode
	lastBestNodeCost = startNode.Total

	outOfNodes := false

	for !q.openList.empty() {

		var bestNode *Node

		// Remove node from open list and put it in closed list.
		bestNode = q.openList.pop()
		bestNode.Flags &= ^nodeOpen
		bestNode.Flags |= nodeClosed

		// Reached the goal, stop searching.
		if bestNode.ID == endRef {
			lastBestNode = bestNode
			break
		}

		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		var (
			bestRef  PolyRef
			bestTile *MeshTile
			bestPoly *Poly
		)

		bestRef = bestNode.ID
		bestTile = nil
		bestPoly = nil
		q.nav.TileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly)

		// Get parent poly and tile.
		var (
			parentRef  PolyRef
			parentTile *MeshTile
			parentPoly *Poly
		)
		if bestNode.PIdx != 0 {
			parentRef = q.nodePool.NodeAtIdx(int32(bestNode.PIdx)).ID
		}
		if parentRef != 0 {
			q.nav.TileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly)
		}

		var i uint32
		for i = bestPoly.FirstLink; i != nullLink; i = bestTile.Links[i].Next {
			neighbourRef := bestTile.Links[i].Ref

			// Skip invalid ids and do not expand back to where we came from.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			var (
				neighbourTile *MeshTile
				neighbourPoly *Poly
			)
			q.nav.TileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly)

			if !filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// deal explicitly with crossing tile boundaries
			var crossSide uint8
			if bestTile.Links[i].Side != 0xff {
				crossSide = bestTile.Links[i].Side >> 1
			}

			// get the node
			neighbourNode := q.nodePool.Node(neighbourRef, crossSide)
			if neighbourNode == nil {
				outOfNodes = true
				continue
			}

			// If the node is visited the first time, calculate node position.
			if neighbourNode.Flags == 0 {

				status := q.edgeMidPoint(bestRef, bestPoly, bestTile,
					neighbourRef, neighbourPoly, neighbourTile,
					neighbourNode.Pos[:])
				if StatusFailed(status) {
					log.Println("getEdgeMidPoint failed:", status)
				}
			}

			// Calculate cost and heuristic.
			var cost, heuristic float32

			// Special case for last node.
			if neighbourRef == endRef {
				// Cost
				curCost := filter.Cost(bestNode.Pos[:], neighbourNode.Pos[:],
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				endCost := filter.Cost(neighbourNode.Pos[:], endPos[:],
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly,
					0, nil, nil)

				cost = bestNode.Cost + curCost + endCost
				heuristic = 0
			} else {
				// Cost
				curCost := filter.Cost(bestNode.Pos[:], neighbourNode.Pos[:],
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				cost = bestNode.Cost + curCost
				heuristic = neighbourNode.Pos.Dist(endPos) * HScale
			}

			total := cost + heuristic

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&nodeOpen) != 0 && total >= neighbourNode.Total {
				continue
			}
			// The node is already visited and process, and the new result is worse, skip.
			if (neighbourNode.Flags&nodeClosed) != 0 && total >= neighbourNode.Total {
				continue
			}

			// Add or update the node.
			neighbourNode.PIdx = q.nodePool.NodeIdx(bestNode)
			neighbourNode.ID = neighbourRef
			neighbourNode.Flags = (neighbourNode.Flags & NodeFlags(^NodeFlags(nodeClosed)))
			neighbourNode.Cost = cost
			neighbourNode.Total = total

			if (neighbourNode.Flags & nodeOpen) != 0 {
				// Already in open, update node location.
				q.openList.modify(neighbourNode)
			} else {
				// Put the node in open list.
				neighbourNode.Flags |= nodeOpen
				q.openList.push(neighbourNode)
			}

			// Update nearest node to target so far.
			if heuristic < lastBestNodeCost {
				lastBestNodeCost = heuristic
				lastBestNode = neighbourNode
			}
		}
	}

	pathCount, status := q.pathToNode(lastBestNode, path)

	if lastBestNode.ID != endRef {
		status |= PartialResult
	}

	if outOfNodes {
		status |= OutOfNodes
	}

	return pathCount, status
}

// Vertex flags returned by NavMeshQuery.FindStraightPath.
const (
	// The vertex is the start position in the path.
	StraightPathStart uint8 = 0x01
	// The vertex is the end position in the path.
	StraightPathEnd uint8 = 0x02
	// The vertex is the start of an off-mesh connection.
	StraightPathOffMeshConnection uint8 = 0x04
)

// Options for NavMeshQuery.FindStraightPath.
const (
	// Add a vertex at every polygon edge crossing where area changes.
	StraightPathAreaCrossings uint8 = 0x01
	// Add a vertex at every polygon edge crossing.
	StraightPathAllCrossings uint8 = 0x02
)

// FindStraightPath finds the straight path from the start to the end position
// within the polygon corridor
//
//  Arguments:
//   startPos          Path start position. [(x, y, z)]
//   endPos            Path end position. [(x, y, z)]
//   path              An array of polygon references that represent the
//                     path corridor.
//   straightPath      Points describing the straight path
//                     [Length: == straightPathCount].
//   straightPathFlags Flags describing each point.
//                     (See: StraightPathFlags)
//   straightPathRefs  The reference id of the polygon that is being
//                     entered at each point.
//   options           Query options. (see: StraightPathOptions)
//
// Returns The status flags for the query and the number of point in the
// straight path.
//
// The straightPath, straightPathFlags and straightPathRefs slices must already
// be allocated and contain the same number of elements.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) FindStraightPath(
	startPos, endPos d3.Vec3,
	path []PolyRef,
	straightPath []d3.Vec3,
	straightPathFlags []uint8,
	straightPathRefs []PolyRef,
	options int32) (straightPathCount int, st Status) {

	// parameter check
	if len(straightPath) == 0 {
		fmt.Println("len(straightPath) == 0")
		return 0, Failure | InvalidParam
	}
	if len(path) == 0 {
		fmt.Println("len(path) == 0")
		return 0, Failure | InvalidParam
	}

	var (
		stat  Status
		count int
	)

	// TODO: Should this be callers responsibility?
	closestStartPos := d3.NewVec3()
	if StatusFailed(q.closestPointOnPolyBoundary(path[0], startPos, closestStartPos)) {
		return 0, Failure | InvalidParam
	}

	closestEndPos := d3.NewVec3()
	if StatusFailed(q.closestPointOnPolyBoundary(path[len(path)-1], endPos, closestEndPos)) {
		return 0, Failure | InvalidParam
	}

	// Add start point.
	stat = q.appendVertex(closestStartPos, StraightPathStart, path[0],
		straightPath, straightPathFlags, straightPathRefs,
		&count)
	if stat != InProgress {
		fmt.Println("FindStraightPath returns", stat, count)
		return count, stat
	}

	if len(path) > 1 {
		portalApex := d3.NewVec3From(closestStartPos)
		portalLeft := d3.NewVec3From(portalApex)
		portalRight := d3.NewVec3From(portalApex)
		var (
			apexIndex     int
			leftIndex     int
			rightIndex    int
			leftPolyType  uint8
			rightPolyType uint8
		)

		leftPolyRef := path[0]
		rightPolyRef := path[0]

		for i := 0; i < len(path); i++ {
			left := d3.NewVec3()
			right := d3.NewVec3()
			var toType uint8

			if i+1 < len(path) {
				var fromType uint8 // fromType is ignored.

				// Next portal.
				if StatusFailed(q.portalPoints6(path[i], path[i+1], left, right, &fromType, &toType)) {
					// Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
					// Clamp the end point to path[i], and return the path so far.
					if StatusFailed(q.closestPointOnPolyBoundary(path[i], endPos, closestEndPos)) {
						// This should only happen when the first polygon is invalid.
						return 0, Failure | InvalidParam
					}

					// Apeend portals along the current straight path segment.
					if (options & int32(StraightPathAreaCrossings|StraightPathAllCrossings)) != 0 {
						// Ignore status return value as we're just about to return anyway.
						q.appendPortals(apexIndex, i, closestEndPos, path,
							straightPath, straightPathFlags, straightPathRefs,
							&count, options)
					}

					// Ignore status return value as we're just about to return anyway.
					q.appendVertex(closestEndPos, 0, path[i],
						straightPath, straightPathFlags, straightPathRefs,
						&count)

					stat = Success | PartialResult
					if count >= len(straightPath) {
						stat |= BufferTooSmall
					}
					fmt.Println("FindStraightPath 2 returns", stat, count)
					return count, stat
				}

				// If starting really close the portal, advance.
				if i == 0 {
					var t float32
					if distancePtSegSqr2D(portalApex, left, right, &t) < math32.Sqr(0.001) {
						continue
					}
				}
			} else {
				// End of the path.
				left.Assign(closestEndPos)
				right.Assign(closestEndPos)
				toType = uint8(polyTypeGround)
			}

			// Right vertex.
			if TriArea2D(portalApex, portalRight, right) <= 0.0 {
				if portalApex.Approx(portalRight) || TriArea2D(portalApex, portalLeft, right) > 0.0 {
					portalRight.Assign(right)
					if i+1 < len(path) {
						rightPolyRef = path[i+1]
					} else {
						rightPolyRef = 0
					}
					rightPolyType = toType
					rightIndex = i
				} else {
					// Append portals along the current straight path segment.
					if (options & int32(StraightPathAreaCrossings|StraightPathAllCrossings)) != 0 {
						stat = q.appendPortals(apexIndex, leftIndex, portalLeft, path,
							straightPath, straightPathFlags, straightPathRefs,
							&count, options)
						if stat != InProgress {
							fmt.Println("FindStraightPath 3 returns", stat, count)
							return count, stat
						}
					}

					portalApex.Assign(portalLeft)
					apexIndex = leftIndex

					var flags uint8
					if leftPolyRef == 0 {
						flags = StraightPathEnd
					} else if leftPolyType == polyTypeOffMeshConnection {
						flags = StraightPathOffMeshConnection
					}
					ref := leftPolyRef

					// Append or update vertex
					stat = q.appendVertex(portalApex, flags, ref,
						straightPath, straightPathFlags, straightPathRefs,
						&count)
					if stat != InProgress {
						fmt.Println("FindStraightPath 4 returns", stat, count)
						return count, stat
					}

					portalLeft.Assign(portalApex)
					portalRight.Assign(portalApex)
					leftIndex = apexIndex
					rightIndex = apexIndex

					// Restart
					i = apexIndex

					continue
				}
			}

			// Left vertex.
			if TriArea2D(portalApex, portalLeft, left) >= 0.0 {
				if portalApex.Approx(portalLeft) || TriArea2D(portalApex, portalRight, left) < 0.0 {
					portalLeft.Assign(left)
					if i+1 < len(path) {
						leftPolyRef = path[i+1]
					} else {
						leftPolyRef = 0
					}
					leftPolyType = toType
					leftIndex = i
				} else {
					// Append portals along the current straight path segment.
					if (options & int32(StraightPathAreaCrossings|StraightPathAllCrossings)) != 0 {
						stat = q.appendPortals(apexIndex, rightIndex, portalRight, path,
							straightPath, straightPathFlags, straightPathRefs,
							&count, options)
						if stat != InProgress {
							fmt.Println("FindStraightPath 5 returns", stat, count)
							return count, stat
						}
					}

					portalApex.Assign(portalRight)
					apexIndex = rightIndex

					var flags uint8
					if rightPolyRef == 0 {
						flags = StraightPathEnd
					} else if rightPolyType == polyTypeOffMeshConnection {
						flags = StraightPathOffMeshConnection
					}
					ref := rightPolyRef

					// Append or update vertex
					stat = q.appendVertex(portalApex, flags, ref,
						straightPath, straightPathFlags, straightPathRefs,
						&count)
					if stat != InProgress {
						fmt.Println("FindStraightPath 6 returns", stat, count)
						return count, stat
					}

					portalLeft.Assign(portalApex)
					portalRight.Assign(portalApex)
					leftIndex = apexIndex
					rightIndex = apexIndex

					// Restart
					i = apexIndex

					continue
				}
			}
		}

		// Append portals along the current straight path segment.
		if (options & int32(StraightPathAreaCrossings|StraightPathAllCrossings)) != 0 {
			stat = q.appendPortals(apexIndex, len(path)-1, closestEndPos, path,
				straightPath, straightPathFlags, straightPathRefs,
				&count, options)
			if stat != InProgress {
				fmt.Println("FindStraightPath 7 returns", stat, count)
				return count, stat
			}
		}
	}

	// Ignore status return value as we're just about to return anyway.
	q.appendVertex(closestEndPos, StraightPathEnd, 0,
		straightPath, straightPathFlags, straightPathRefs,
		&count)

	stat = Success
	if count >= len(straightPath) {
		stat |= BufferTooSmall
	}
	fmt.Println("FindStraightPath 8 returns", stat, count)
	return count, stat
}

// appendPortals appends intermediate portal points to a straight path.
func (q *NavMeshQuery) appendPortals(
	startIdx, endIdx int,
	endPos d3.Vec3,
	path []PolyRef,
	straightPath []d3.Vec3,
	straightPathFlags []uint8,
	straightPathRefs []PolyRef,
	straightPathCount *int,
	options int32) Status {

	startPos := straightPath[*straightPathCount-1]
	// Append or update last vertex
	var stat Status
	for i := startIdx; i < endIdx; i++ {
		// Calculate portal
		from := path[i]
		var (
			fromTile *MeshTile
			fromPoly *Poly
		)
		if StatusFailed(q.nav.TileAndPolyByRef(from, &fromTile, &fromPoly)) {
			return Failure | InvalidParam
		}

		to := path[i+1]
		var (
			toTile *MeshTile
			toPoly *Poly
		)
		if StatusFailed(q.nav.TileAndPolyByRef(to, &toTile, &toPoly)) {

			return Failure | InvalidParam
		}

		left := d3.NewVec3()
		right := d3.NewVec3()
		if StatusFailed(q.portalPoints8(from, fromPoly, fromTile, to, toPoly, toTile, left, right)) {
			break
		}

		if (options & int32(StraightPathAreaCrossings)) != 0 {
			// Skip intersection if only area crossings are requested.
			if fromPoly.Area() == toPoly.Area() {
				continue
			}
		}

		// Append intersection
		if hit, _, t := IntersectSegSeg2D(startPos, endPos, left, right); hit {
			pt := d3.NewVec3()
			d3.Vec3Lerp(pt, left, right, t)

			stat = q.appendVertex(pt, 0, path[i+1],
				straightPath, straightPathFlags, straightPathRefs,
				straightPathCount)
			if stat != InProgress {
				return stat
			}
		}
	}
	return InProgress
}

// appendVertex appends a vertex to a straight path.
func (q *NavMeshQuery) appendVertex(
	pos d3.Vec3,
	flags uint8,
	ref PolyRef,
	straightPath []d3.Vec3,
	straightPathFlags []uint8,
	straightPathRefs []PolyRef,
	straightPathCount *int) Status {

	if (*straightPathCount) > 0 && pos.Approx(straightPath[*straightPathCount-1]) {
		// The vertices are equal, update flags and poly.
		if len(straightPathFlags) > 0 {
			straightPathFlags[*straightPathCount-1] = flags
		}
		if len(straightPathRefs) > 0 {
			straightPathRefs[*straightPathCount-1] = ref
		}
	} else {
		// Append new vertex.
		straightPath[*straightPathCount].Assign(pos)
		if len(straightPathFlags) > 0 {
			straightPathFlags[*straightPathCount] = flags
		}
		if len(straightPathRefs) > 0 {
			straightPathRefs[*straightPathCount] = ref
		}
		(*straightPathCount)++

		// If there is no space to append more vertices, return.
		if (*straightPathCount) >= len(straightPath) {
			return Success | BufferTooSmall
		}

		// If reached end of path, return.
		if flags == StraightPathEnd {
			return Success
		}
	}
	return InProgress
}

// edgeMidPoint returns the edge mid point between two polygons.
func (q *NavMeshQuery) edgeMidPoint(
	from PolyRef, fromPoly *Poly, fromTile *MeshTile,
	to PolyRef, toPoly *Poly, toTile *MeshTile,
	mid d3.Vec3) Status {

	left, right := d3.NewVec3(), d3.NewVec3()

	if StatusFailed(q.portalPoints8(from, fromPoly, fromTile, to, toPoly, toTile, left, right)) {
		return Failure | InvalidParam
	}
	mid[0] = (left[0] + right[0]) * 0.5
	mid[1] = (left[1] + right[1]) * 0.5
	mid[2] = (left[2] + right[2]) * 0.5
	return Success
}

// portalPoints6 returns portal points between two polygons.
func (q *NavMeshQuery) portalPoints6(
	from, to PolyRef,
	left, right d3.Vec3,
	fromType, toType *uint8) Status {
	var (
		fromTile *MeshTile
		fromPoly *Poly
	)
	if StatusFailed(q.nav.TileAndPolyByRef(from, &fromTile, &fromPoly)) {
		return Failure | InvalidParam
	}
	*fromType = fromPoly.Type()

	var (
		toTile *MeshTile
		toPoly *Poly
	)
	if StatusFailed(q.nav.TileAndPolyByRef(to, &toTile, &toPoly)) {
		return Failure | InvalidParam
	}
	*toType = toPoly.Type()

	return q.portalPoints8(from, fromPoly, fromTile, to, toPoly, toTile, left, right)
}

// portalPoints8 returns portal points between two polygons.
func (q *NavMeshQuery) portalPoints8(
	from PolyRef, fromPoly *Poly, fromTile *MeshTile,
	to PolyRef, toPoly *Poly, toTile *MeshTile,
	left, right d3.Vec3) Status {

	// Find the link that points to the 'to' polygon.
	var link *Link
	for i := fromPoly.FirstLink; i != nullLink; i = fromTile.Links[i].Next {
		if fromTile.Links[i].Ref == to {
			link = &fromTile.Links[i]
			break
		}
	}
	if link == nil {
		return Failure | InvalidParam
	}

	// Handle off-mesh connections.
	if fromPoly.Type() == polyTypeOffMeshConnection {
		// Find link that points to first vertex.
		for i := fromPoly.FirstLink; i != nullLink; i = fromTile.Links[i].Next {
			if fromTile.Links[i].Ref == to {
				// TODO: AR, repass here and test
				v := fromTile.Links[i].Edge
				vidx := fromPoly.Verts[v] * 3
				copy(left, fromTile.Verts[vidx:vidx+3])
				copy(right, fromTile.Verts[vidx:vidx+3])
				return Success
			}
		}
		return Failure | InvalidParam
	}

	if toPoly.Type() == polyTypeOffMeshConnection {
		for i := toPoly.FirstLink; i != nullLink; i = toTile.Links[i].Next {
			if toTile.Links[i].Ref == from {
				// TODO: AR, repass here and test
				v := toTile.Links[i].Edge
				vidx := fromPoly.Verts[v] * 3
				copy(left, toTile.Verts[vidx:vidx+3])
				copy(right, toTile.Verts[vidx:vidx+3])
				return Success
			}
		}
		return Failure | InvalidParam
	}

	// Find portal vertices.
	v0 := fromPoly.Verts[link.Edge]
	v1 := fromPoly.Verts[(link.Edge+1)%fromPoly.VertCount]

	// TODO: AR TO BE TESTED!
	v0idx := v0 * 3
	copy(left, fromTile.Verts[v0idx:v0idx+3])
	v1idx := v1 * 3
	copy(right, fromTile.Verts[v1idx:v1idx+3])

	// If the link is at tile boundary, clamp the vertices to
	// the link width.
	if link.Side != 0xff {
		// Unpack portal limits.
		if link.BMin != 0 || link.BMax != 255 {
			s := float32(1.0 / 255.0)
			tmin := float32(link.BMin) * s
			tmax := float32(link.BMax) * s
			d3.Vec3Lerp(left, fromTile.Verts[v0idx:v0idx+3], fromTile.Verts[v1idx:v1idx+3], tmin)
			d3.Vec3Lerp(right, fromTile.Verts[v0idx:v0idx+3], fromTile.Verts[v1idx:v1idx+3], tmax)
		}
	}

	return Success
}

// pathToNode gets the path leading to the specified end node.
func (q *NavMeshQuery) pathToNode(
	endNode *Node,
	path []PolyRef) (pathCount int, st Status) {

	var (
		curNode *Node
		length  int
	)
	// Find the length of the entire path.
	curNode = endNode

	for {
		length++
		curNode = q.nodePool.NodeAtIdx(int32(curNode.PIdx))
		if curNode == nil {
			break
		}
	}

	// If the path cannot be fully stored then advance to the last node we will be able to store.
	curNode = endNode
	var writeCount int
	for writeCount = length; writeCount > len(path); writeCount-- {
		assert.True(curNode != nil, "curNode should not be nil")
		curNode = q.nodePool.NodeAtIdx(int32(curNode.PIdx))
	}

	// Write path
	for i := writeCount - 1; i >= 0; i-- {
		assert.True(curNode != nil, "curNode should not be nil")
		assert.True(int(i) < len(path), "i:%d should be < len(path):%d", i, len(path))

		path[i] = curNode.ID
		curNode = q.nodePool.NodeAtIdx(int32(curNode.PIdx))
	}

	assert.True(curNode == nil, "curNode should be nil")

	if length <= len(path) {
		pathCount = length
	} else {
		pathCount = len(path)
	}

	if length > len(path) {
		return pathCount, Success | BufferTooSmall
	}

	return pathCount, Success
}

// closestPointOnPoly uses the detail polygons to find the surface height.
// (Most accurate.)
//
// pos does not have to be within the bounds of the polygon or navigation mesh.
// See closestPointOnPolyBoundary() for a limited but faster option.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) closestPointOnPoly(ref PolyRef, pos, closest d3.Vec3, posOverPoly *bool) Status {
	assert.True(q.nav != nil, "NavMesh should not be nil")
	var (
		tile *MeshTile
		poly *Poly
	)

	if StatusFailed(q.nav.TileAndPolyByRef(ref, &tile, &poly)) {
		return Failure | InvalidParam
	}
	if tile == nil {
		return Failure | InvalidParam
	}

	// Off-mesh connections don't have detail polygons.
	if poly.Type() == polyTypeOffMeshConnection {
		var (
			v0, v1    d3.Vec3
			d0, d1, u float32
		)
		vidx := poly.Verts[0] * 3
		v0 = tile.Verts[vidx : vidx+3]
		vidx = poly.Verts[1] * 3
		v1 = tile.Verts[vidx : vidx+3]
		d0 = pos.Dist(v0)
		d1 = pos.Dist(v1)
		u = d0 / (d0 + d1)
		d3.Vec3Lerp(closest, v0, v1, u)
		if posOverPoly != nil {
			*posOverPoly = false
		}
		return Success
	}

	ip := (uintptr(unsafe.Pointer(poly)) - uintptr(unsafe.Pointer(&tile.Polys[0]))) / unsafe.Sizeof(*poly)
	pd := &tile.DetailMeshes[uint32(ip)]

	// Clamp point to be inside the polygon.
	verts := make([]float32, VertsPerPolygon*3)
	edged := make([]float32, VertsPerPolygon)
	edget := make([]float32, VertsPerPolygon)
	nv := poly.VertCount
	var i uint8
	for i = 0; i < nv; i++ {
		idx := i * 3
		jdx := poly.Verts[i] * 3
		copy(verts[idx:idx+3], tile.Verts[jdx:jdx+3])
	}

	closest.Assign(pos)
	if !distancePtPolyEdgesSqr(pos, verts, int32(nv), edged, edget) {
		// Point is outside the polygon, clamp to nearest edge.
		dmin := edged[0]
		var imin uint8
		for i = 1; i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = i
			}
		}
		idx := imin * 3
		va := verts[idx : idx+3]
		idx = ((imin + 1) % nv) * 3
		vb := verts[idx : idx+3]
		d3.Vec3Lerp(closest, va, vb, edget[imin])

		if posOverPoly != nil {
			*posOverPoly = false
		}
	} else {
		if posOverPoly != nil {
			*posOverPoly = true
		}
	}

	// Find height at the location.
	var j uint8
	var idx int
	for j = 0; j < pd.TriCount; j++ {
		idx = int((pd.TriBase + uint32(j)) * 4)
		t := tile.DetailTris[idx : idx+3]
		v := make([][]float32, 3)
		var k int
		for k = 0; k < 3; k++ {
			if t[k] < poly.VertCount {
				idx = int(poly.Verts[t[k]] * 3)
				v[k] = tile.Verts[idx : idx+3]
			} else {
				idx = int((pd.VertBase + uint32(t[k]-poly.VertCount)) * 3)
				v[k] = tile.DetailVerts[idx : idx+3]
			}
		}
		var h float32
		if closestHeightPointTriangle(closest, v[0], v[1], v[2], &h) {
			closest[1] = h
			break
		}
	}
	return Success
}

// closestPointOnPolyBoundary uses the detail polygons to find the surface
// height. (Much faster than closestPointOnPoly())
//
// If the provided position lies within the polygon's xz-bounds (above or
// below), then pos and closest will be equal. The height of closest will be the
// polygon boundary. The height detail is not used. pos does not have to be
// within the bounds of the polybon or the navigation mesh.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) closestPointOnPolyBoundary(ref PolyRef, pos, closest d3.Vec3) Status {
	var (
		tile *MeshTile
		poly *Poly
	)
	if StatusFailed(q.nav.TileAndPolyByRef(ref, &tile, &poly)) {
		return Failure | InvalidParam
	}

	// Collect vertices.
	var (
		verts [VertsPerPolygon * 3]float32
		edged [VertsPerPolygon]float32
		edget [VertsPerPolygon]float32
		nv    int32
	)
	for i := uint8(0); i < poly.VertCount; i++ {
		copy(verts[nv*3:nv*3+3], tile.Verts[poly.Verts[i]*3:poly.Verts[i]*3+3])
		nv++
	}

	inside := distancePtPolyEdgesSqr(pos, verts[:], nv, edged[:], edget[:])
	if inside {
		// Point is inside the polygon, return the point.
		closest.Assign(pos)
	} else {
		// Point is outside the polygon, clamp to nearest edge.
		dmin := edged[0]
		imin := int32(0)
		for i := int32(1); i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = i
			}
		}
		va := verts[imin*3 : imin*3+3]
		vidx := ((imin + 1) % nv) * 3
		vb := verts[vidx : vidx+3]
		d3.Vec3Lerp(closest, va, vb, edget[imin])
	}

	return Success
}

// FindNearestPoly finds the polygon nearest to the specified center point.
//
//  Arguments:
//   center   The center of the search box.
//   extents  A vector which components represent the
//            search distance along each axis.
//   filter   The polygon filter to apply to the query.
//
//  Return values:
//   st       The status flags for the query.
//   ref      The reference id of the nearest polygon.
//   pt       The nearest point on the polygon. [(x, y, z)]
//
// Note: If the search box does not intersect any polygons the returned status
// will be 'Success', but ref will be zero. So if in doubt, check ref before
// using pt.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) FindNearestPoly(center, extents d3.Vec3,
	filter QueryFilter) (st Status, ref PolyRef, pt d3.Vec3) {

	assert.True(q.nav != nil, "Nav should not be nil")

	query := newFindNearestPolyQuery(q, center)
	st = q.queryPolygons4(center, extents, filter, query)
	if StatusFailed(st) {
		return
	}

	// Only allocate pt if we actually found
	// a poly so the nearest point pt is valid.
	if ref = query.nearestRef; ref != 0 {
		pt = d3.NewVec3From(query.nearestPoint)
	}
	st = Success
	return
}

// queryPolygons6 finds polygons that overlap the search box.
//
//  Arguments:
//   center     The center of the search box.
//   extents    The search distance along each axis.
//   filter     The polygon filter to apply to the query.
//   polys      The reference ids of the polygons that overlap the query box.
//   polyCount  The number of polygons in the search result.
//   maxPolys   The maximum number of polygons the search result can hold.
//
//  Return values:
//   The status flags for the query.
//
// If no polygons are found, the function will return Success with a polyCount
// of zero. If polys is too small to hold the entire result set, then the array
// will be filled to capacity. The method of choosing which polygons from the
// full set are included in the partial result set is undefined.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) queryPolygons6(
	center, extents []float32,
	filter QueryFilter,
	polys []PolyRef,
	maxPolys int32) (polyCount int32, st Status) {

	if polys == nil || maxPolys < 0 {
		st = Failure | InvalidParam
		return
	}

	collector := newCollectPolysQuery(polys, maxPolys)

	st = q.queryPolygons4(center, extents, filter, collector)
	if StatusFailed(st) {
		return
	}

	polyCount = collector.numCollected
	st = Success
	if collector.overflow {
		st |= BufferTooSmall
	}
	return
}

// queryPolygons4 finds polygons that overlap the search box.
//
//  Arguments:
//   center   The center of the search box. [(x, y, z)]
//   extents  The search distance along each axis. [(x, y, z)]
//   filter   The polygon filter to apply to the query.
//   query    The query. Polygons found will be batched together and passed to
//            this query.
//
// The query will be invoked with batches of polygons. Polygons passed to the
// query have bounding boxes that overlap with the center and extents passed to
// this function. The polyQuery.process function is invoked multiple times until
// all overlapping polygons have been processed.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) queryPolygons4(
	center, extents d3.Vec3,
	filter QueryFilter,
	query polyQuery) Status {
	// parameter check
	if len(center) != 3 || len(extents) != 3 || filter == nil || query == nil {
		return Failure | InvalidParam
	}

	bmin := center.Sub(extents)
	bmax := center.Add(extents)

	// Find tiles the query touches.
	minx, miny := q.nav.CalcTileLoc(bmin)
	maxx, maxy := q.nav.CalcTileLoc(bmax)

	const maxNeis int32 = 32
	neis := make([]*MeshTile, maxNeis)

	for y := miny; y <= maxy; y++ {
		for x := minx; x <= maxx; x++ {
			nneis := q.nav.TilesAt(x, y, neis, maxNeis)
			for j := int32(0); j < nneis; j++ {
				q.queryPolygonsInTile(neis[j], bmin[:], bmax[:], filter, query)
			}
		}
	}
	return Success
}

// queryPolygonsInTile queries polygons within a tile.
func (q *NavMeshQuery) queryPolygonsInTile(
	tile *MeshTile,
	qmin, qmax []float32,
	filter QueryFilter,
	query polyQuery) {

	assert.True(q.nav != nil, "navmesh should not be nill")
	batchSize := int32(32)

	polyRefs := make([]PolyRef, batchSize)
	polys := make([]*Poly, batchSize)
	var n int32

	if len(tile.BvTree) > 0 {

		var (
			node            *BvNode
			nodeIdx, endIdx int32
			tbmin, tbmax    d3.Vec3
			qfac            float32
		)

		nodeIdx = 0
		endIdx = tile.Header.BvNodeCount

		tbmin = d3.NewVec3From(tile.Header.BMin[:])
		tbmax = d3.NewVec3From(tile.Header.BMax[:])
		qfac = tile.Header.BvQuantFactor

		// Calculate quantized box
		var bmin, bmax [3]uint16

		// Clamp query box to world box.
		minx := f32.Clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := f32.Clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := f32.Clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := f32.Clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := f32.Clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := f32.Clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
		// Quantize
		bmin[0] = uint16(qfac*minx) & 0xfffe
		bmin[1] = uint16(qfac*miny) & 0xfffe
		bmin[2] = uint16(qfac*minz) & 0xfffe
		bmax[0] = uint16(qfac*maxx+1) | 1
		bmax[1] = uint16(qfac*maxy+1) | 1
		bmax[2] = uint16(qfac*maxz+1) | 1

		// Traverse tree
		base := q.nav.polyRefBase(tile)
		// TODO: probably need to use an index or unsafe.Pointer here
		for nodeIdx < endIdx {
			node = &tile.BvTree[nodeIdx]
			overlap := OverlapQuantBounds(bmin[:], bmax[:], node.BMin[:], node.BMax[:])
			isLeafNode := node.I >= 0

			if isLeafNode && overlap {
				ref := base | PolyRef(node.I)
				if filter.PassFilter(ref, tile, &tile.Polys[node.I]) {
					polyRefs[n] = ref
					polys[n] = &tile.Polys[node.I]

					if n == batchSize-1 {
						query.process(tile, polys, polyRefs, batchSize)
						n = 0
					} else {
						n++
					}
				}
			}

			if overlap || isLeafNode {
				nodeIdx++
			} else {
				escapeIndex := -node.I
				nodeIdx += escapeIndex
			}
		}
	} else {
		var bmin, bmax d3.Vec3
		bmin = d3.NewVec3()
		bmax = d3.NewVec3()
		base := q.nav.polyRefBase(tile)
		for i := int32(0); i < tile.Header.PolyCount; i++ {
			p := &tile.Polys[i]
			// Do not return off-mesh connection polygons.
			if p.Type() == polyTypeOffMeshConnection {

				log.Fatalf("do return off-mesh connection polygons")
				continue
			}
			// Must pass filter
			ref := base | PolyRef(i)
			if !filter.PassFilter(ref, tile, p) {
				continue
			}
			// Calc polygon bounds.
			vidx := p.Verts[0] * 3
			v := tile.Verts[vidx : vidx+3]
			bmin.Assign(v)
			bmax.Assign(v)
			for j := uint8(1); j < p.VertCount; j++ {
				vidx = p.Verts[j] * 3
				v = tile.Verts[vidx : vidx+3]
				d3.Vec3Min(bmin, v)
				d3.Vec3Max(bmax, v)
			}
			if OverlapBounds(qmin, qmax, bmin[:], bmax[:]) {
				polyRefs[n] = ref
				polys[n] = p

				if n == batchSize-1 {
					query.process(tile, polys, polyRefs, batchSize)
					n = 0
				} else {
					n++
				}
			}
		}
	}

	// Process the last polygons that didn't make a full batch.
	if n > 0 {
		query.process(tile, polys, polyRefs, n)
	}
}

// NodePool returns the node pool.
func (q *NavMeshQuery) NodePool() *NodePool {
	return q.nodePool
}

// AttachedNavMesh returns the navigation mesh the query object is using.
func (q *NavMeshQuery) AttachedNavMesh() *NavMesh {
	return q.nav
}

// IsValidPolyRef returns true if the polygon reference is valid and passes the
// filter restrictions.
//
//  Arguments:
//   ref      The polygon reference to check.
//   filter   The filter to apply.
//
// Note: this method may be used by multiple clients without side effects.
func (q *NavMeshQuery) IsValidPolyRef(ref PolyRef, filter QueryFilter) bool {
	var (
		tile *MeshTile
		poly *Poly
	)
	status := q.nav.TileAndPolyByRef(ref, &tile, &poly)
	// If cannot get polygon, assume it does not exists and boundary is invalid.
	if StatusFailed(status) {
		return false
	}
	// If cannot pass filter, assume flags has changed and boundary is invalid.
	if !filter.PassFilter(ref, tile, poly) {
		return false
	}
	return true
}

// Casts a 'walkability' ray along the surface of the navigation mesh from
// the start position toward the end position.
//
//  Arguments:
//   startRef  The reference id of the start polygon.
//   startPos  A position within the start polygon representing
//             the start of the ray. [(x, y, z)]
//   endPos    The position to cast the ray toward. [(x, y, z)]
//   filter    The polygon filter to apply to the query.
//   options   Govern how the raycast behaves. See RaycastOptions
//   prevRef   Parent of start ref. Used during for cost calculation [opt]
//
//  Returns:
//   hit       Raycast hit structure which will be filled with the results.
//   st        status flags for the query.
//
// This method is meant to be used for quick, short distance checks.
//
// If the path array is too small to hold the result, it will be filled as
// far as possible from the start postion toward the end position.
//
// # Using the Hit Parameter t of RaycastHit
//
// If the hit parameter is a very high value (math.MaxFloat32), then the ray has
// hit the end position. In this case the path represents a valid corridor to
// the end position and the value of hitNormal is undefined.
//
// If the hit parameter is zero, then the start position is on the wall that was
// hit and the value of hitNormal is undefined.
//
// If 0 < t < 1.0 then the following applies:
//
// Example:
//  distanceToHitBorder = distanceToEndPosition * t
//  hitPoint = startPos + (endPos - startPos) * t
//
// # Use Case Restriction
//
// The raycast ignores the y-value of the end position. (2D check.) This places
// significant limits on how it can be used. For example:
//
// Consider a scene where there is a main floor with a second floor balcony that
// hangs over the main floor. So the first floor mesh extends below the balcony
// mesh. The start position is somewhere on the first floor. The end position is
// on the balcony.
//
// The raycast will search toward the end position along the first floor mesh.
// If it reaches the end position's xz-coordinates it will indicate
// math.MaxFloat32 (no wall hit), meaning it reached the end position. This is
// one example of why this method is meant for short distance checks.
func (q *NavMeshQuery) Raycast(
	startRef PolyRef,
	startPos, endPos d3.Vec3,
	filter QueryFilter,
	options int,
	prevRef PolyRef) (hit RaycastHit, st Status) {

	// Validate input
	if startRef == 0 || !q.nav.IsValidPolyRef(startRef) {
		st = Failure | InvalidParam
		return
	}
	if prevRef != 0 && !q.nav.IsValidPolyRef(prevRef) {
		st = Failure | InvalidParam
		return
	}

	var (
		dir, curPos, lastPos d3.Vec3
		verts                [VertsPerPolygon*3 + 3]float32
		n                    int
	)

	curPos = d3.NewVec3From(startPos)
	dir = endPos.Sub(startPos)
	hit.HitNormal = d3.NewVec3()

	st = Success

	var (
		prevTile, tile, nextTile *MeshTile
		prevPoly, poly, nextPoly *Poly
		curRef                   PolyRef
	)

	// The API input has been checked already, skip checking internal data.
	curRef = startRef
	q.nav.TileAndPolyByRefUnsafe(curRef, &tile, &poly)
	prevTile = tile
	prevPoly = poly
	nextTile = prevTile
	nextPoly = prevPoly
	if prevRef != 0 {
		q.nav.TileAndPolyByRefUnsafe(prevRef, &prevTile, &prevPoly)
	}

	for curRef != 0 {
		// Cast ray against current polygon.

		// Collect vertices.
		var nv int
		for i := 0; i < int(poly.VertCount); i++ {
			copy(verts[nv*3:], tile.Verts[poly.Verts[i]*3:3+poly.Verts[i]*3])
			nv++
		}

		var (
			tmax   float32
			segMax int
			res    bool
		)
		if _, tmax, _, segMax, res = IntersectSegmentPoly2D(startPos, endPos, verts[:], nv); !res {
			// Could not hit the polygon, keep the old t and report hit.
			hit.PathCount = n
			return
		}

		hit.HitEdgeIndex = segMax

		// Keep track of furthest t so far.
		if tmax > hit.T {
			hit.T = tmax
		}

		// Store visited polygons.
		if n < hit.MaxPath {
			hit.Path[n] = curRef
			n++
		} else {
			st |= BufferTooSmall
		}

		// Ray end is completely inside the polygon.
		if segMax == -1 {
			hit.T = math.MaxFloat32
			hit.PathCount = n

			// add the cost
			if (options & RaycastUseCosts) != 0 {
				hit.PathCost += filter.Cost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly, curRef, tile, poly)
			}
			return
		}

		// Follow neighbours.
		var nextRef PolyRef

		for i := uint32(poly.FirstLink); i != nullLink; i = tile.Links[i].Next {
			link := &tile.Links[i]

			// Find link which contains this edge.
			if int(link.Edge) != segMax {
				continue
			}

			// Get pointer to the next polygon.
			nextTile = nil
			nextPoly = nil
			q.nav.TileAndPolyByRefUnsafe(link.Ref, &nextTile, &nextPoly)

			// Skip off-mesh connections.
			if nextPoly.Type() == polyTypeOffMeshConnection {
				continue
			}

			// Skip links based on filter.
			if !filter.PassFilter(link.Ref, nextTile, nextPoly) {
				continue
			}

			// If the link is internal, just return the ref.
			if link.Side == 0xff {
				nextRef = link.Ref
				break
			}

			// If the link is at tile boundary,

			// Check if the link spans the whole edge, and accept.
			if link.BMin == 0 && link.BMax == 255 {
				nextRef = link.Ref
				break
			}

			// Check for partial edge links.
			v0 := poly.Verts[link.Edge]
			v1 := poly.Verts[(link.Edge+1)%poly.VertCount]
			left := tile.Verts[v0*3 : 3+v0*3]
			right := tile.Verts[v1*3 : 3+v1*3]

			// Check that the intersection lies inside the link portal.
			if link.Side == 0 || link.Side == 4 {
				// Calculate link size.
				var s float32 = 1.0 / 255.0
				lmin := left[2] + (right[2]-left[2])*(float32(link.BMin)*s)
				lmax := left[2] + (right[2]-left[2])*(float32(link.BMax)*s)
				if lmin > lmax {
					lmin, lmax = lmax, lmin
				}

				// Find Z intersection.
				z := startPos[2] + (endPos[2]-startPos[2])*tmax
				if z >= lmin && z <= lmax {
					nextRef = link.Ref
					break
				}
			} else if link.Side == 2 || link.Side == 6 {
				// Calculate link size.
				var s float32 = 1.0 / 255.0
				lmin := left[0] + (right[0]-left[0])*(float32(link.BMin)*s)
				lmax := left[0] + (right[0]-left[0])*(float32(link.BMax)*s)
				if lmin > lmax {
					lmin, lmax = lmax, lmin
				}

				// Find X intersection.
				x := startPos[0] + (endPos[0]-startPos[0])*tmax
				if x >= lmin && x <= lmax {
					nextRef = link.Ref
					break
				}
			}
		}

		// add the cost
		if (options & RaycastUseCosts) != 0 {
			// compute the intersection point at the furthest end of the polygon
			// and correct the height (since the raycast moves in 2d)
			copy(lastPos, curPos[:3])
			d3.Vec3Mad(curPos, startPos, dir, hit.T)
			e1 := verts[segMax*3 : 3+segMax*3]
			var e2 d3.Vec3 = verts[((segMax+1)%nv)*3 : 3+((segMax+1)%nv)*3]
			var eDir, diff [3]float32
			d3.Vec3Sub(eDir[:], e2, e1)
			d3.Vec3Sub(diff[:], curPos, e1)
			var s float32
			if math32.Sqr(eDir[0]) > math32.Sqr(eDir[2]) {
				s = diff[0] / eDir[0]
			} else {
				s = diff[2] / eDir[2]
			}
			curPos[1] = e1[1] + eDir[1]*s

			hit.PathCost += filter.Cost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, tile, poly, nextRef, nextTile, nextPoly)
		}

		if nextRef == 0 {
			// No neighbour, we hit a wall.

			// Calculate hit normal.
			a := segMax
			var b int
			if segMax+1 < nv {
				b = segMax + 1
			}
			va := verts[a*3 : 3+a*3]
			vb := verts[b*3 : 3+b*3]
			dx := vb[0] - va[0]
			dz := vb[2] - va[2]
			hit.HitNormal[0] = dz
			hit.HitNormal[1] = 0
			hit.HitNormal[2] = -dx
			hit.HitNormal.Normalize()

			hit.PathCount = n
			return
		}

		// No hit, advance to neighbour polygon.
		prevRef = curRef
		curRef = nextRef
		prevTile = tile
		tile = nextTile
		prevPoly = poly
		poly = nextPoly
	}

	hit.PathCount = n
	return
}
