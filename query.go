package detour

import (
	"fmt"
	"log"
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

// NavMeshQuery provides the ability to perform pathfinding related queries
// against a navigation mesh.
type NavMeshQuery struct {

	/////@}
	///// @name Sliced Pathfinding Functions
	///// Common use case:
	/////	-# Call initSlicedFindPath() to initialize the sliced path query.
	/////	-# Call updateSlicedFindPath() until it returns complete.
	/////	-# Call finalizeSlicedFindPath() to get the path.
	/////@{

	///// Intializes a sliced path query.
	/////  @param[in]		startRef	The refrence id of the start polygon.
	/////  @param[in]		endRef		The reference id of the end polygon.
	/////  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	/////  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[in]		options		query options (see: #dtFindPathOptions)
	///// @returns The status flags for the query.
	//Status initSlicedFindPath(PolyRef startRef, PolyRef endRef,
	//const float* startPos, const float* endPos,
	//const QueryFilter* filter, const unsigned int options = 0);

	///// Updates an in-progress sliced path query.
	/////  @param[in]		maxIter		The maximum number of iterations to perform.
	/////  @param[out]	doneIters	The actual number of iterations completed. [opt]
	///// @returns The status flags for the query.
	//Status updateSlicedFindPath(const int maxIter, int* doneIters);

	///// Finalizes and returns the results of a sliced path query.
	/////  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	/////  							[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount	The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	///// @returns The status flags for the query.
	//Status finalizeSlicedFindPath(PolyRef* path, int* pathCount, const int maxPath);

	///// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	///// polygon on the existing path that was visited during the search.
	/////  @param[in]		existing		An array of polygon references for the existing path.
	/////  @param[in]		existingSize	The number of polygon in the @p existing array.
	/////  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.)
	/////  								[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount		The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	///// @returns The status flags for the query.
	//Status finalizeSlicedFindPathPartial(const PolyRef* existing, const int existingSize,
	//PolyRef* path, int* pathCount, const int maxPath);

	/////@}
	///// @name Dijkstra Search Functions
	///// @{

	///// Finds the polygons along the navigation graph that touch the specified circle.
	/////  @param[in]		startRef		The reference id of the polygon where the search starts.
	/////  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	/////  @param[in]		radius			The radius of the search circle.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[out]	resultRef		The reference ids of the polygons touched by the circle. [opt]
	/////  @param[out]	resultParent	The reference ids of the parent polygons for each result.
	/////  								Zero if a result polygon has no parent. [opt]
	/////  @param[out]	resultCost		The search cost from @p centerPos to the polygon. [opt]
	/////  @param[out]	resultCount		The number of polygons found. [opt]
	/////  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	///// @returns The status flags for the query.
	//Status findPolysAroundCircle(PolyRef startRef, const float* centerPos, const float radius,
	//const QueryFilter* filter,
	//PolyRef* resultRef, PolyRef* resultParent, float* resultCost,
	//int* resultCount, const int maxResult) const;

	///// Finds the polygons along the naviation graph that touch the specified convex polygon.
	/////  @param[in]		startRef		The reference id of the polygon where the search starts.
	/////  @param[in]		verts			The vertices describing the convex polygon. (CCW)
	/////  								[(x, y, z) * @p nverts]
	/////  @param[in]		nverts			The number of vertices in the polygon.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[out]	resultRef		The reference ids of the polygons touched by the search polygon. [opt]
	/////  @param[out]	resultParent	The reference ids of the parent polygons for each result. Zero if a
	/////  								result polygon has no parent. [opt]
	/////  @param[out]	resultCost		The search cost from the centroid point to the polygon. [opt]
	/////  @param[out]	resultCount		The number of polygons found.
	/////  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	///// @returns The status flags for the query.
	//Status findPolysAroundShape(PolyRef startRef, const float* verts, const int nverts,
	//const QueryFilter* filter,
	//PolyRef* resultRef, PolyRef* resultParent, float* resultCost,
	//int* resultCount, const int maxResult) const;

	///// Gets a path from the explored nodes in the previous search.
	/////  @param[in]		endRef		The reference id of the end polygon.
	/////  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	/////  							[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount	The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	/////  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	/////  				@p endRef was not explored in the previous search. Returns Success | DT_BUFFER_TOO_SMALL
	/////  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	/////  				Otherwise returns Success.
	/////  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	/////  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	//Status getPathFromDijkstraSearch(PolyRef endRef, PolyRef* path, int* pathCount, int maxPath) const;

	///// @}
	///// @name Local Query Functions
	/////@{

	///// Finds the polygon nearest to the specified center point.
	/////  @param[in]		center		The center of the search box. [(x, y, z)]
	/////  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[out]	nearestRef	The reference id of the nearest polygon.
	/////  @param[out]	nearestPt	The nearest point on the polygon. [opt] [(x, y, z)]
	///// @returns The status flags for the query.
	//Status findNearestPoly(const float* center, const float* extents,
	//const QueryFilter* filter,
	//PolyRef* nearestRef, float* nearestPt) const;

	///// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
	/////  @param[in]		startRef		The reference id of the polygon where the search starts.
	/////  @param[in]		centerPos		The center of the query circle. [(x, y, z)]
	/////  @param[in]		radius			The radius of the query circle.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[out]	resultRef		The reference ids of the polygons touched by the circle.
	/////  @param[out]	resultParent	The reference ids of the parent polygons for each result.
	/////  								Zero if a result polygon has no parent. [opt]
	/////  @param[out]	resultCount		The number of polygons found.
	/////  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	///// @returns The status flags for the query.
	//Status findLocalNeighbourhood(PolyRef startRef, const float* centerPos, const float radius,
	//const QueryFilter* filter,
	//PolyRef* resultRef, PolyRef* resultParent,
	//int* resultCount, const int maxResult) const;

	///// Moves from the start to the end position constrained to the navigation mesh.
	/////  @param[in]		startRef		The reference id of the start polygon.
	/////  @param[in]		startPos		A position of the mover within the start polygon. [(x, y, x)]
	/////  @param[in]		endPos			The desired end position of the mover. [(x, y, z)]
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[out]	resultPos		The result position of the mover. [(x, y, z)]
	/////  @param[out]	visited			The reference ids of the polygons visited during the move.
	/////  @param[out]	visitedCount	The number of polygons visited during the move.
	/////  @param[in]		maxVisitedSize	The maximum number of polygons the @p visited array can hold.
	///// @returns The status flags for the query.
	//Status moveAlongSurface(PolyRef startRef, const float* startPos, const float* endPos,
	//const QueryFilter* filter,
	//float* resultPos, PolyRef* visited, int* visitedCount, const int maxVisitedSize) const;

	///// Casts a 'walkability' ray along the surface of the navigation mesh from
	///// the start position toward the end position.
	///// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
	/////  @param[in]		startRef	The reference id of the start polygon.
	/////  @param[in]		startPos	A position within the start polygon representing
	/////  							the start of the ray. [(x, y, z)]
	/////  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	/////  @param[out]	t			The hit parameter. (FLT_MAX if no wall hit.)
	/////  @param[out]	hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[out]	path		The reference ids of the visited polygons. [opt]
	/////  @param[out]	pathCount	The number of visited polygons. [opt]
	/////  @param[in]		maxPath		The maximum number of polygons the @p path array can hold.
	///// @returns The status flags for the query.
	//Status raycast(PolyRef startRef, const float* startPos, const float* endPos,
	//const QueryFilter* filter,
	//float* t, float* hitNormal, PolyRef* path, int* pathCount, const int maxPath) const;

	///// Casts a 'walkability' ray along the surface of the navigation mesh from
	///// the start position toward the end position.
	/////  @param[in]		startRef	The reference id of the start polygon.
	/////  @param[in]		startPos	A position within the start polygon representing
	/////  							the start of the ray. [(x, y, z)]
	/////  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[in]		flags		govern how the raycast behaves. See dtRaycastOptions
	/////  @param[out]	hit			Pointer to a raycast hit structure which will be filled by the results.
	/////  @param[in]		prevRef		parent of start ref. Used during for cost calculation [opt]
	///// @returns The status flags for the query.
	//Status raycast(PolyRef startRef, const float* startPos, const float* endPos,
	//const QueryFilter* filter, const unsigned int options,
	//dtRaycastHit* hit, PolyRef prevRef = 0) const;

	///// Finds the distance from the specified position to the nearest polygon wall.
	/////  @param[in]		startRef		The reference id of the polygon containing @p centerPos.
	/////  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	/////  @param[in]		maxRadius		The radius of the search circle.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[out]	hitDist			The distance to the nearest wall from @p centerPos.
	/////  @param[out]	hitPos			The nearest position on the wall that was hit. [(x, y, z)]
	/////  @param[out]	hitNormal		The normalized ray formed from the wall point to the
	/////  								source point. [(x, y, z)]
	///// @returns The status flags for the query.
	//Status findDistanceToWall(PolyRef startRef, const float* centerPos, const float maxRadius,
	//const QueryFilter* filter,
	//float* hitDist, float* hitPos, float* hitNormal) const;

	///// Returns the segments for the specified polygon, optionally including portals.
	/////  @param[in]		ref				The reference id of the polygon.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
	/////  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon.
	/////  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount]
	/////  @param[out]	segmentCount	The number of segments returned.
	/////  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
	///// @returns The status flags for the query.
	//Status getPolyWallSegments(PolyRef ref, const QueryFilter* filter,
	//float* segmentVerts, PolyRef* segmentRefs, int* segmentCount,
	//const int maxSegments) const;

	///// Returns random location on navmesh.
	///// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[in]		frand			Function returning a random number [0..1).
	/////  @param[out]	randomRef		The reference id of the random location.
	/////  @param[out]	randomPt		The random location.
	///// @returns The status flags for the query.
	//Status findRandomPoint(const QueryFilter* filter, float (*frand)(),
	//PolyRef* randomRef, float* randomPt) const;

	///// Returns random location on navmesh within the reach of specified location.
	///// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	///// The location is not exactly constrained by the circle, but it limits the visited polygons.
	/////  @param[in]		startRef		The reference id of the polygon where the search starts.
	/////  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[in]		frand			Function returning a random number [0..1).
	/////  @param[out]	randomRef		The reference id of the random location.
	/////  @param[out]	randomPt		The random location. [(x, y, z)]
	///// @returns The status flags for the query.
	//Status findRandomPointAroundCircle(PolyRef startRef, const float* centerPos, const float maxRadius,
	//const QueryFilter* filter, float (*frand)(),
	//PolyRef* randomRef, float* randomPt) const;

	///// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	/////  @param[in]		ref			The reference id of the polygon.
	/////  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	/////  @param[out]	height		The height at the surface of the polygon.
	///// @returns The status flags for the query.
	//Status getPolyHeight(PolyRef ref, const float* pos, float* height) const;

	///// @}
	///// @name Miscellaneous Functions
	///// @{

	///// Returns true if the polygon reference is valid and passes the filter restrictions.
	/////  @param[in]		ref			The polygon reference to check.
	/////  @param[in]		filter		The filter to apply.
	//bool isValidPolyRef(PolyRef ref, const QueryFilter* filter) const;

	///// Returns true if the polygon reference is in the closed list.
	/////  @param[in]		ref		The reference id of the polygon to check.
	///// @returns True if the polygon is in closed list.
	//bool isInClosedList(PolyRef ref) const;
	///// @}

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
//   [in]startRef    The reference id of the start polygon.
//   [in]endRef      The reference id of the end polygon.
//   [in]startPos    A position within the start polygon. [(x, y, z)]
//   [in]endPos      A position within the end polygon. [(x, y, z)]
//   [in]filter      The polygon filter to apply to the query.
//   [out]path       An ordered list of polygon references representing the
//                   path. (Start to end.) [(polyRef) * pathCount]
//   [out]pathCount  The number of polygons returned in the path array.
//   [in]maxPath     The maximum number of polygons the path array can hold.
//                   [Limit: >= 1]
//
// If the end polygon cannot be reached through the navigation graph, the last
// polygon in the path will be the nearest the end polygon.
//
// If the path array is to small to hold the full result, it will be filled as
// far as possible from the start polygon toward the end polygon.
//
// The start and end positions are used to calculate traversal costs.
// (The y-values impact the result.)
func (q *NavMeshQuery) FindPath(
	startRef, endRef PolyRef,
	startPos, endPos d3.Vec3,
	filter QueryFilter,
	path *[]PolyRef,
	pathCount *int32,
	maxPath int32) Status {

	if len(*path) < int(maxPath) {
		// immediately check the provided slice
		// is big enough to store maxPath nodes
		return Failure | InvalidParam
	}

	if pathCount != nil {
		*pathCount = 0
	}

	// Validate input
	if !q.nav.IsValidPolyRef(startRef) || !q.nav.IsValidPolyRef(endRef) ||
		len(startPos) < 3 || len(endPos) < 3 || filter == nil || maxPath <= 0 || path == nil || pathCount == nil {
		return Failure | InvalidParam
	}

	if startRef == endRef {
		(*path)[0] = startRef
		*pathCount = 1
		return Success
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

				status := q.getEdgeMidPoint(bestRef, bestPoly, bestTile,
					neighbourRef, neighbourPoly, neighbourTile,
					neighbourNode.Pos[:])
				if StatusFailed(status) {
					log.Fatalf("getEdgeMidPoint failed")
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

	status := q.pathToNode(lastBestNode, path, pathCount, maxPath)

	if lastBestNode.ID != endRef {
		status |= PartialResult
	}

	if outOfNodes {
		status |= OutOfNodes
	}

	return status
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
//   [in]startPos            Path start position. [(x, y, z)]
//   [in]endPos              Path end position. [(x, y, z)]
//   [in]path                An array of polygon references that represent the
//                           path corridor.
//   [in]pathSize            The number of polygons in the path array.
//   [out] straightPath      Points describing the straight path
//                           [Length: == straightPathCount].
//   [out] straightPathFlags Flags describing each point.
//                           (See: StraightPathFlags)
//   [out] straightPathRefs  The reference id of the polygon that is being
//                           entered at each point.
//   [in]  maxStraightPath   The maximum number of points the straight path
//                           arrays can hold.  [Limit: > 0]
//   [in]  options           Query options. (see: StraightPathOptions)
//
// Returns The status flags for the query and the number of point in the
// straight path.
//
// The straightPath, straightPathFlags and straightPathRefs slices must already
// be allocated and contain at least maxStraightPath elements.
func (q *NavMeshQuery) FindStraightPath(
	startPos, endPos d3.Vec3,
	path []PolyRef, pathSize int32,
	straightPath []d3.Vec3,
	straightPathFlags []uint8,
	straightPathRefs []PolyRef,
	maxStraightPath int32,
	options int32) (Status, int32) {

	assert.True(q.nav != nil, "NavMesh should not be nil")

	if maxStraightPath == 0 {
		fmt.Println("maxStraightPath == 0")
		return Failure | InvalidParam, 0
	}

	if len(path) == 0 {
		fmt.Println("len(path) == 0")
		return Failure | InvalidParam, 0
	}

	var (
		stat  Status
		count int32
	)

	// TODO: Should this be callers responsibility?
	closestStartPos := d3.NewVec3()
	if StatusFailed(q.closestPointOnPolyBoundary(path[0], startPos, closestStartPos)) {
		return Failure | InvalidParam, 0
	}

	closestEndPos := d3.NewVec3()
	if StatusFailed(q.closestPointOnPolyBoundary(path[pathSize-1], endPos, closestEndPos)) {
		return Failure | InvalidParam, 0
	}

	// Add start point.
	stat = q.appendVertex(closestStartPos, StraightPathStart, path[0],
		straightPath, straightPathFlags, straightPathRefs,
		&count, maxStraightPath)
	if stat != InProgress {
		return stat, count
	}

	if pathSize > 1 {
		portalApex := d3.NewVec3From(closestStartPos)
		portalLeft := d3.NewVec3From(portalApex)
		portalRight := d3.NewVec3From(portalApex)
		var (
			apexIndex     int32
			leftIndex     int32
			rightIndex    int32
			leftPolyType  uint8
			rightPolyType uint8
		)

		leftPolyRef := path[0]
		rightPolyRef := path[0]

		for i := int32(0); i < pathSize; i++ {
			left := d3.NewVec3()
			right := d3.NewVec3()
			var toType uint8

			if i+1 < pathSize {
				var fromType uint8 // fromType is ignored.

				// Next portal.
				if StatusFailed(q.getPortalPoints6(path[i], path[i+1], left, right, &fromType, &toType)) {
					// Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
					// Clamp the end point to path[i], and return the path so far.
					if StatusFailed(q.closestPointOnPolyBoundary(path[i], endPos, closestEndPos)) {
						// This should only happen when the first polygon is invalid.
						return Failure | InvalidParam, 0
					}

					// Apeend portals along the current straight path segment.
					if (options & int32(StraightPathAreaCrossings|StraightPathAllCrossings)) != 0 {
						// Ignore status return value as we're just about to return anyway.
						q.appendPortals(apexIndex, i, closestEndPos, path,
							straightPath, straightPathFlags, straightPathRefs,
							&count, maxStraightPath, options)
					}

					// Ignore status return value as we're just about to return anyway.
					q.appendVertex(closestEndPos, 0, path[i],
						straightPath, straightPathFlags, straightPathRefs,
						&count, maxStraightPath)

					stat = Success | PartialResult
					if count >= maxStraightPath {
						stat |= BufferTooSmall
					}
					return stat, count
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
					if i+1 < pathSize {
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
							&count, maxStraightPath, options)
						if stat != InProgress {
							return stat, count
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
						&count, maxStraightPath)
					if stat != InProgress {
						return stat, count
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
					if i+1 < pathSize {
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
							&count, maxStraightPath, options)
						if stat != InProgress {
							return stat, count
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
						&count, maxStraightPath)
					if stat != InProgress {
						return stat, count
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
			stat = q.appendPortals(apexIndex, pathSize-1, closestEndPos, path,
				straightPath, straightPathFlags, straightPathRefs,
				&count, maxStraightPath, options)
			if stat != InProgress {
				return stat, count
			}
		}
	}

	// Ignore status return value as we're just about to return anyway.
	q.appendVertex(closestEndPos, StraightPathEnd, 0,
		straightPath, straightPathFlags, straightPathRefs,
		&count, maxStraightPath)

	stat = Success
	if count >= maxStraightPath {
		stat |= BufferTooSmall
	}
	return stat, count
}

// appendPortals appends intermediate portal points to a straight path.
func (q *NavMeshQuery) appendPortals(
	startIdx, endIdx int32,
	endPos d3.Vec3,
	path []PolyRef,
	straightPath []d3.Vec3,
	straightPathFlags []uint8,
	straightPathRefs []PolyRef,
	straightPathCount *int32,
	maxStraightPath,
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
		if StatusFailed(q.getPortalPoints8(from, fromPoly, fromTile, to, toPoly, toTile, left, right)) {
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
				straightPathCount, maxStraightPath)
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
	straightPathCount *int32,
	maxStraightPath int32) Status {

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
		if (*straightPathCount) >= maxStraightPath {
			return Success | BufferTooSmall
		}

		// If reached end of path, return.
		if flags == StraightPathEnd {
			return Success
		}
	}
	return InProgress
}

// getEdgeMidPoint returns the edge mid point between two polygons.
func (q *NavMeshQuery) getEdgeMidPoint(
	from PolyRef, fromPoly *Poly, fromTile *MeshTile,
	to PolyRef, toPoly *Poly, toTile *MeshTile,
	mid []float32) Status {

	left := make([]float32, 3)
	right := make([]float32, 3)

	if StatusFailed(q.getPortalPoints8(from, fromPoly, fromTile, to, toPoly, toTile, left, right)) {
		return Failure | InvalidParam
	}
	mid[0] = (left[0] + right[0]) * 0.5
	mid[1] = (left[1] + right[1]) * 0.5
	mid[2] = (left[2] + right[2]) * 0.5
	return Success
}

// getPortalPoints6 returns portal points between two polygons.
func (q *NavMeshQuery) getPortalPoints6(
	from, to PolyRef,
	left, right d3.Vec3,
	fromType, toType *uint8) Status {

	assert.True(q.nav != nil, "NavMesh should not be nil")

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

	return q.getPortalPoints8(from, fromPoly, fromTile, to, toPoly, toTile, left, right)
}

// getPortalPoints8 returns portal points between two polygons.
func (q *NavMeshQuery) getPortalPoints8(
	from PolyRef, fromPoly *Poly, fromTile *MeshTile,
	to PolyRef, toPoly *Poly, toTile *MeshTile,
	left, right d3.Vec3) Status {

	// Find the link that points to the 'to' polygon.
	var link *link
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
		if link.Bmin != 0 || link.Bmax != 255 {
			s := float32(1.0 / 255.0)
			tmin := float32(link.Bmin) * s
			tmax := float32(link.Bmax) * s
			d3.Vec3Lerp(left, fromTile.Verts[v0idx:v0idx+3], fromTile.Verts[v1idx:v1idx+3], tmin)
			d3.Vec3Lerp(right, fromTile.Verts[v0idx:v0idx+3], fromTile.Verts[v1idx:v1idx+3], tmax)
		}
	}

	return Success
}

// pathToNode gets the path leading to the specified end node.
func (q *NavMeshQuery) pathToNode(
	endNode *Node,
	path *[]PolyRef,
	pathCount *int32,
	maxPath int32) Status {

	var (
		curNode *Node
		length  int32
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
	var writeCount int32
	for writeCount = length; writeCount > maxPath; writeCount-- {
		assert.True(curNode != nil, "curNode should not be nil")
		curNode = q.nodePool.NodeAtIdx(int32(curNode.PIdx))
	}

	// Write path
	for i := writeCount - 1; i >= 0; i-- {
		assert.True(curNode != nil, "curNode should not be nil")
		assert.True(int(i) < len(*path), "i:%d should be < len(*path):%d", i, len(*path))

		(*path)[i] = curNode.ID
		curNode = q.nodePool.NodeAtIdx(int32(curNode.PIdx))
	}

	assert.True(curNode == nil, "curNode should be nil")

	*pathCount = math32.MinInt32(length, maxPath)

	if length > maxPath {
		return Success | BufferTooSmall
	}

	return Success
}

// closestPointOnPoly uses the detail polygons to find the surface height.
// (Most accurate.)
//
// pos does not have to be within the bounds of the polygon or navigation mesh.
// See closestPointOnPolyBoundary() for a limited but faster option.
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

	e := uintptr(unsafe.Pointer(poly)) - uintptr(unsafe.Pointer(&tile.Polys[0]))
	ip := uint32(e / unsafe.Sizeof(*poly))

	assert.True(ip < uint32(len(tile.Polys)), "ip should be < len(tile.Polys), ip=%d, len(tile.Polys)=%d", ip, len(tile.Polys))

	pd := &tile.DetailMeshes[ip]

	// Clamp point to be inside the polygon.
	verts := make([]float32, vertsPerPolygon*3)
	edged := make([]float32, vertsPerPolygon)
	edget := make([]float32, vertsPerPolygon)
	nv := poly.VertCount
	var i uint8
	for i = 0; i < nv; i++ {
		// TODO: could probably use copy
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
// within the bounds of the polybon or the navigation
// mesh.
func (q *NavMeshQuery) closestPointOnPolyBoundary(ref PolyRef, pos, closest d3.Vec3) Status {
	assert.True(q.nav != nil, "NavMesh should not be nil")

	var (
		tile *MeshTile
		poly *Poly
	)
	if StatusFailed(q.nav.TileAndPolyByRef(ref, &tile, &poly)) {
		return Failure | InvalidParam
	}

	// Collect vertices.
	var (
		verts [vertsPerPolygon * 3]float32
		edged [vertsPerPolygon]float32
		edget [vertsPerPolygon]float32
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
// Note: If the search box does not intersect any polygons st will be
// Success, but ref will be zero. So if in doubt, check ref before using pt.
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
//   [in] center     The center of the search box.
//   [in] extents    The search distance along each axis.
//   [in] filter     The polygon filter to apply to the query.
//   [out]polys      The reference ids of the polygons that overlap the query box.
//   [out]polyCount  The number of polygons in the search result.
//   [in] maxPolys   The maximum number of polygons the search result can hold.
//
//  Return values:
//   The status flags for the query.
//
// If no polygons are found, the function will return Success with a
// polyCount of zero.
// If polys is too small to hold the entire result set, then the array will be
// filled to capacity. The method of choosing which polygons from the full set
// are included in the partial result set is undefined.
func (q *NavMeshQuery) queryPolygons6(
	center, extents []float32,
	filter QueryFilter,
	polys []PolyRef,
	polyCount *int32,
	maxPolys int32) Status {

	if polys == nil || polyCount == nil || maxPolys < 0 {
		return Failure | InvalidParam
	}

	collector := newCollectPolysQuery(polys, maxPolys)

	status := q.queryPolygons4(center, extents, filter, collector)
	if StatusFailed(status) {
		return status
	}

	*polyCount = collector.numCollected
	if collector.overflow {
		return Success | BufferTooSmall
	}
	return Success
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
// this function. The polyQuery.process function is invoked multiple times
// until all overlapping polygons have been processed.
func (q *NavMeshQuery) queryPolygons4(
	center, extents d3.Vec3,
	filter QueryFilter,
	query polyQuery) Status {

	assert.True(q.nav != nil, "navmesh should not be nill")

	if len(center) != 3 || len(extents) != 3 || filter == nil || query == nil {
		return Failure | InvalidParam
	}

	bmin := center.Sub(extents)
	bmax := center.Sub(extents)

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
			node            *bvNode
			nodeIdx, endIdx int32
			tbmin, tbmax    d3.Vec3
			qfac            float32
		)

		nodeIdx = 0
		endIdx = tile.Header.BvNodeCount

		tbmin = d3.NewVec3From(tile.Header.Bmin[:])
		tbmax = d3.NewVec3From(tile.Header.Bmax[:])
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
		base := q.nav.getPolyRefBase(tile)
		// TODO: probably need to use an index or unsafe.Pointer here
		for nodeIdx < endIdx {
			node = &tile.BvTree[nodeIdx]
			overlap := OverlapQuantBounds(bmin[:], bmax[:], node.Bmin[:], node.Bmax[:])
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
		base := q.nav.getPolyRefBase(tile)
		for i := int32(0); i < tile.Header.PolyCount; i++ {
			p := &tile.Polys[i]
			// Do not return off-mesh connection polygons.
			if p.Type() == polyTypeOffMeshConnection {
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
