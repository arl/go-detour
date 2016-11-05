package detour

import (
	"fmt"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
)

const (
	H_SCALE float32 = 0.999 // Search heuristic scale.
)

/// Provides the ability to perform pathfinding related queries against
/// a navigation mesh.
/// @ingroup detour
type DtNavMeshQuery struct {

	/// @name Standard Pathfinding Functions
	// /@{

	///// Finds the straight path from the start to the end position within the polygon corridor.
	/////  @param[in]		startPos			Path start position. [(x, y, z)]
	/////  @param[in]		endPos				Path end position. [(x, y, z)]
	/////  @param[in]		path				An array of polygon references that represent the path corridor.
	/////  @param[in]		pathSize			The number of polygons in the @p path array.
	/////  @param[out]	straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
	/////  @param[out]	straightPathFlags	Flags describing each point. (See: #dtStraightPathFlags) [opt]
	/////  @param[out]	straightPathRefs	The reference id of the polygon that is being entered at each point. [opt]
	/////  @param[out]	straightPathCount	The number of points in the straight path.
	/////  @param[in]		maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
	/////  @param[in]		options				Query options. (see: #dtStraightPathOptions)
	///// @returns The status flags for the query.
	//dtStatus findStraightPath(const float* startPos, const float* endPos,
	//const DtPolyRef* path, const int pathSize,
	//float* straightPath, unsigned char* straightPathFlags, DtPolyRef* straightPathRefs,
	//int* straightPathCount, const int maxStraightPath, const int options = 0) const;

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
	//dtStatus initSlicedFindPath(DtPolyRef startRef, DtPolyRef endRef,
	//const float* startPos, const float* endPos,
	//const dtQueryFilter* filter, const unsigned int options = 0);

	///// Updates an in-progress sliced path query.
	/////  @param[in]		maxIter		The maximum number of iterations to perform.
	/////  @param[out]	doneIters	The actual number of iterations completed. [opt]
	///// @returns The status flags for the query.
	//dtStatus updateSlicedFindPath(const int maxIter, int* doneIters);

	///// Finalizes and returns the results of a sliced path query.
	/////  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	/////  							[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount	The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	///// @returns The status flags for the query.
	//dtStatus finalizeSlicedFindPath(DtPolyRef* path, int* pathCount, const int maxPath);

	///// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	///// polygon on the existing path that was visited during the search.
	/////  @param[in]		existing		An array of polygon references for the existing path.
	/////  @param[in]		existingSize	The number of polygon in the @p existing array.
	/////  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.)
	/////  								[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount		The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	///// @returns The status flags for the query.
	//dtStatus finalizeSlicedFindPathPartial(const DtPolyRef* existing, const int existingSize,
	//DtPolyRef* path, int* pathCount, const int maxPath);

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
	//dtStatus findPolysAroundCircle(DtPolyRef startRef, const float* centerPos, const float radius,
	//const dtQueryFilter* filter,
	//DtPolyRef* resultRef, DtPolyRef* resultParent, float* resultCost,
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
	//dtStatus findPolysAroundShape(DtPolyRef startRef, const float* verts, const int nverts,
	//const dtQueryFilter* filter,
	//DtPolyRef* resultRef, DtPolyRef* resultParent, float* resultCost,
	//int* resultCount, const int maxResult) const;

	///// Gets a path from the explored nodes in the previous search.
	/////  @param[in]		endRef		The reference id of the end polygon.
	/////  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	/////  							[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount	The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	/////  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	/////  				@p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
	/////  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	/////  				Otherwise returns DT_SUCCESS.
	/////  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	/////  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	//dtStatus getPathFromDijkstraSearch(DtPolyRef endRef, DtPolyRef* path, int* pathCount, int maxPath) const;

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
	//dtStatus findNearestPoly(const float* center, const float* extents,
	//const dtQueryFilter* filter,
	//DtPolyRef* nearestRef, float* nearestPt) const;

	///// Finds polygons that overlap the search box.
	/////  @param[in]		center		The center of the search box. [(x, y, z)]
	/////  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[out]	polys		The reference ids of the polygons that overlap the query box.
	/////  @param[out]	polyCount	The number of polygons in the search result.
	/////  @param[in]		maxPolys	The maximum number of polygons the search result can hold.
	///// @returns The status flags for the query.
	//dtStatus queryPolygons(const float* center, const float* extents,
	//const dtQueryFilter* filter,
	//DtPolyRef* polys, int* polyCount, const int maxPolys) const;

	///// Finds polygons that overlap the search box.
	/////  @param[in]		center		The center of the search box. [(x, y, z)]
	/////  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[in]		query		The query. Polygons found will be batched together and passed to this query.
	//dtStatus queryPolygons(const float* center, const float* extents,
	//const dtQueryFilter* filter, DtPolyQuery* query) const;

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
	//dtStatus findLocalNeighbourhood(DtPolyRef startRef, const float* centerPos, const float radius,
	//const dtQueryFilter* filter,
	//DtPolyRef* resultRef, DtPolyRef* resultParent,
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
	//dtStatus moveAlongSurface(DtPolyRef startRef, const float* startPos, const float* endPos,
	//const dtQueryFilter* filter,
	//float* resultPos, DtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;

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
	//dtStatus raycast(DtPolyRef startRef, const float* startPos, const float* endPos,
	//const dtQueryFilter* filter,
	//float* t, float* hitNormal, DtPolyRef* path, int* pathCount, const int maxPath) const;

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
	//dtStatus raycast(DtPolyRef startRef, const float* startPos, const float* endPos,
	//const dtQueryFilter* filter, const unsigned int options,
	//dtRaycastHit* hit, DtPolyRef prevRef = 0) const;

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
	//dtStatus findDistanceToWall(DtPolyRef startRef, const float* centerPos, const float maxRadius,
	//const dtQueryFilter* filter,
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
	//dtStatus getPolyWallSegments(DtPolyRef ref, const dtQueryFilter* filter,
	//float* segmentVerts, DtPolyRef* segmentRefs, int* segmentCount,
	//const int maxSegments) const;

	///// Returns random location on navmesh.
	///// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[in]		frand			Function returning a random number [0..1).
	/////  @param[out]	randomRef		The reference id of the random location.
	/////  @param[out]	randomPt		The random location.
	///// @returns The status flags for the query.
	//dtStatus findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
	//DtPolyRef* randomRef, float* randomPt) const;

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
	//dtStatus findRandomPointAroundCircle(DtPolyRef startRef, const float* centerPos, const float maxRadius,
	//const dtQueryFilter* filter, float (*frand)(),
	//DtPolyRef* randomRef, float* randomPt) const;

	///// Finds the closest point on the specified polygon.
	/////  @param[in]		ref			The reference id of the polygon.
	/////  @param[in]		pos			The position to check. [(x, y, z)]
	/////  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
	/////  @param[out]	posOverPoly	True of the position is over the polygon.
	///// @returns The status flags for the query.
	//dtStatus closestPointOnPoly(DtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;

	///// Returns a point on the boundary closest to the source point if the source point is outside the
	///// polygon's xz-bounds.
	/////  @param[in]		ref			The reference id to the polygon.
	/////  @param[in]		pos			The position to check. [(x, y, z)]
	/////  @param[out]	closest		The closest point. [(x, y, z)]
	///// @returns The status flags for the query.
	//dtStatus closestPointOnPolyBoundary(DtPolyRef ref, const float* pos, float* closest) const;

	///// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	/////  @param[in]		ref			The reference id of the polygon.
	/////  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	/////  @param[out]	height		The height at the surface of the polygon.
	///// @returns The status flags for the query.
	//dtStatus getPolyHeight(DtPolyRef ref, const float* pos, float* height) const;

	///// @}
	///// @name Miscellaneous Functions
	///// @{

	///// Returns true if the polygon reference is valid and passes the filter restrictions.
	/////  @param[in]		ref			The polygon reference to check.
	/////  @param[in]		filter		The filter to apply.
	//bool isValidPolyRef(DtPolyRef ref, const dtQueryFilter* filter) const;

	///// Returns true if the polygon reference is in the closed list.
	/////  @param[in]		ref		The reference id of the polygon to check.
	///// @returns True if the polygon is in closed list.
	//bool isInClosedList(DtPolyRef ref) const;
	///// @}

	//private:
	//// Explicitly disabled copy constructor and copy assignment operator
	//dtNavMeshQuery(const dtNavMeshQuery&);
	//dtNavMeshQuery& operator=(const dtNavMeshQuery&);

	///// Queries polygons within a tile.
	//void queryPolygonsInTile(const DtMeshTile* tile, const float* qmin, const float* qmax,
	//const dtQueryFilter* filter, DtPolyQuery* query) const;

	///// Returns portal points between two polygons.
	//dtStatus getPortalPoints(DtPolyRef from, DtPolyRef to, float* left, float* right,
	//unsigned char& fromType, unsigned char& toType) const;
	//dtStatus getPortalPoints(DtPolyRef from, const DtPoly* fromPoly, const DtMeshTile* fromTile,
	//DtPolyRef to, const DtPoly* toPoly, const DtMeshTile* toTile,
	//float* left, float* right) const;

	///// Returns edge mid point between two polygons.
	//dtStatus getEdgeMidPoint(DtPolyRef from, DtPolyRef to, float* mid) const;

	//// Appends vertex to a straight path
	//dtStatus appendVertex(const float* pos, const unsigned char flags, const DtPolyRef ref,
	//float* straightPath, unsigned char* straightPathFlags, DtPolyRef* straightPathRefs,
	//int* straightPathCount, const int maxStraightPath) const;

	//// Appends intermediate portal points to a straight path.
	//dtStatus appendPortals(const int startIdx, const int endIdx, const float* endPos, const DtPolyRef* path,
	//float* straightPath, unsigned char* straightPathFlags, DtPolyRef* straightPathRefs,
	//int* straightPathCount, const int maxStraightPath, const int options) const;

	//// Gets the path leading to the specified end node.
	//dtStatus getPathToNode(struct dtNode* endNode, DtPolyRef* path, int* pathCount, int maxPath) const;

	nav   *DtNavMesh  ///< Pointer to navmesh data.
	query dtQueryData ///< Sliced query state.

	tinyNodePool *DtNodePool  ///< Pointer to small node pool.
	nodePool     *DtNodePool  ///< Pointer to node pool.
	openList     *DtNodeQueue ///< Pointer to open list queue.
}

type dtQueryData struct {
	status           DtStatus
	lastBestNode     *DtNode
	lastBestNodeCost float32
	startRef, endRef DtPolyRef
	startPos, endPos [3]float32
	//filter           *dtQueryFilter   // TODO: AR not defined for now
	options         uint32
	raycastLimitSqr float32
}

/// Initializes the query object.
///  @param[in]		nav			Pointer to the dtNavMesh object to use for all queries.
///  @param[in]		maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
/// @returns The status flags for the query.
/// @par
///
/// Must be the first function called after construction, before other
/// functions are used.
///
/// This function can be used multiple times.
func NewDtNavMeshQuery(nav *DtNavMesh, maxNodes int32) (*DtNavMeshQuery, DtStatus) {
	if maxNodes > int32(DT_NULL_IDX) || maxNodes > int32(1<<DT_NODE_PARENT_BITS)-1 {
		return nil, DT_FAILURE | DT_INVALID_PARAM
	}

	q := &DtNavMeshQuery{}
	q.nav = nav

	if q.nodePool == nil || q.nodePool.getMaxNodes() < maxNodes {
		if q.nodePool != nil {
			//m_nodePool.~dtNodePool();
			//dtFree(m_nodePool);
			q.nodePool = nil
		}
		//m_nodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(maxNodes, dtNextPow2(maxNodes/4));
		q.nodePool = newDtNodePool(maxNodes, int32(dtNextPow2(uint32(maxNodes/4))))
		if q.nodePool == nil {
			return nil, DT_FAILURE | DT_OUT_OF_MEMORY
		}
	} else {
		q.nodePool.clear()
	}

	if q.tinyNodePool == nil {
		q.tinyNodePool = newDtNodePool(64, 32)
		if q.tinyNodePool == nil {
			return nil, DT_FAILURE | DT_OUT_OF_MEMORY
		}
	} else {
		q.tinyNodePool.clear()
	}

	if q.openList == nil || q.openList.getCapacity() < maxNodes {
		if q.openList != nil {
			//m_openList.~dtNodeQueue();
			//dtFree(m_openList);
			q.openList = nil
		}
		q.openList = newDtNodeQueue(maxNodes)
		if q.openList == nil {
			return nil, DT_FAILURE | DT_OUT_OF_MEMORY
		}
	} else {
		q.openList.clear()
	}

	return q, DT_SUCCESS
}

/// Finds a path from the start polygon to the end polygon.
///  @param[in]		startRef	The reference id of the start polygon.
///  @param[in]		endRef		The reference id of the end polygon.
///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
///  @param[in]		filter		The polygon filter to apply to the query.
///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
///  							[(polyRef) * @p pathCount]
///  @param[out]	pathCount	The number of polygons returned in the @p path array.
///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
///
/// If the end polygon cannot be reached through the navigation graph,
/// the last polygon in the path will be the nearest the end polygon.
///
/// If the path array is to small to hold the full result, it will be filled as
/// far as possible from the start polygon toward the end polygon.
///
/// The start and end positions are used to calculate traversal costs.
/// (The y-values impact the result.)
///
func (q *DtNavMeshQuery) FindPath(startRef, endRef DtPolyRef,
	startPos, endPos []float32,
	filter *DtQueryFilter,
	path *[]DtPolyRef, pathCount *int32, maxPath int32) DtStatus {

	//dtAssert(q.m_nav);
	//dtAssert(q.m_nodePool);
	//dtAssert(q.m_openList);

	if pathCount != nil {
		*pathCount = 0
	}

	// Validate input
	if !q.nav.IsValidPolyRef(startRef) || !q.nav.IsValidPolyRef(endRef) ||
		len(startPos) < 3 || len(endPos) < 3 || filter == nil || maxPath <= 0 || path == nil || pathCount == nil {

		return DT_FAILURE | DT_INVALID_PARAM
	}

	if startRef == endRef {
		(*path)[0] = startRef
		*pathCount = 1
		return DT_SUCCESS
	}

	q.nodePool.clear()
	q.openList.clear()

	var (
		startNode, lastBestNode *DtNode
		lastBestNodeCost        float32
	)
	startNode = q.nodePool.getNode2(startRef)
	dtVcopy(startNode.Pos[:], startPos)
	startNode.PIdx = 0
	startNode.Cost = 0
	startNode.Total = dtVdist(startPos, endPos) * H_SCALE
	startNode.ID = startRef
	startNode.Flags = uint8(DT_NODE_OPEN)
	q.openList.push(startNode)

	lastBestNode = startNode
	lastBestNodeCost = startNode.Total

	outOfNodes := false

	for !q.openList.empty() {

		var bestNode *DtNode

		// Remove node from open list and put it in closed list.
		bestNode = q.openList.pop()
		bestNode.Flags &= ^(uint8(DT_NODE_OPEN))
		bestNode.Flags |= DT_NODE_CLOSED

		// Reached the goal, stop searching.
		if bestNode.ID == endRef {
			lastBestNode = bestNode
			break
		}

		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		var (
			bestRef  DtPolyRef
			bestTile *DtMeshTile
			bestPoly *DtPoly
		)

		bestRef = bestNode.ID
		bestTile = nil
		bestPoly = nil
		q.nav.TileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly)

		// Get parent poly and tile.
		var (
			parentRef  DtPolyRef
			parentTile *DtMeshTile
			parentPoly *DtPoly
		)
		if bestNode.PIdx != 0 {
			parentRef = q.nodePool.getNodeAtIdx(int32(bestNode.PIdx)).ID
		}
		if parentRef != 0 {
			q.nav.TileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly)
		}

		var i uint32
		for i = bestPoly.FirstLink; i != DT_NULL_LINK; i = bestTile.Links[i].Next {
			neighbourRef := bestTile.Links[i].Ref

			// Skip invalid ids and do not expand back to where we came from.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			var (
				neighbourTile *DtMeshTile
				neighbourPoly *DtPoly
			)
			q.nav.TileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly)

			if !filter.passFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// deal explicitly with crossing tile boundaries
			var crossSide uint8
			if bestTile.Links[i].Side != 0xff {
				crossSide = bestTile.Links[i].Side >> 1
			}

			// get the node
			neighbourNode := q.nodePool.getNode(neighbourRef, crossSide)
			if neighbourNode == nil {
				outOfNodes = true
				continue
			}

			// If the node is visited the first time, calculate node position.
			if neighbourNode.Flags == 0 {
				q.getEdgeMidPoint(bestRef, bestPoly, bestTile,
					neighbourRef, neighbourPoly, neighbourTile,
					neighbourNode.Pos[:])
			}

			// Calculate cost and heuristic.
			var cost, heuristic float32

			// Special case for last node.
			if neighbourRef == endRef {
				// Cost
				curCost := filter.getCost(bestNode.Pos[:], neighbourNode.Pos[:],
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				endCost := filter.getCost(neighbourNode.Pos[:], endPos[:],
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly,
					0, nil, nil)

				cost = bestNode.Cost + curCost + endCost
				heuristic = 0
			} else {
				// Cost
				curCost := filter.getCost(bestNode.Pos[:], neighbourNode.Pos[:],
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				cost = bestNode.Cost + curCost
				heuristic = dtVdist(neighbourNode.Pos[:], endPos[:]) * H_SCALE
			}

			total := cost + heuristic

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&uint8(DT_NODE_OPEN)) != 0 && total >= neighbourNode.Total {
				continue
			}
			// The node is already visited and process, and the new result is worse, skip.
			if (neighbourNode.Flags&uint8(DT_NODE_CLOSED)) != 0 && total >= neighbourNode.Total {
				continue
			}

			// Add or update the node.
			neighbourNode.PIdx = q.nodePool.getNodeIdx(bestNode)
			neighbourNode.ID = neighbourRef
			neighbourNode.Flags = (neighbourNode.Flags & ^uint8(DT_NODE_CLOSED))
			neighbourNode.Cost = cost
			neighbourNode.Total = total

			if (neighbourNode.Flags & uint8(DT_NODE_OPEN)) != 0 {
				// Already in open, update node location.
				q.openList.modify(neighbourNode)
			} else {
				// Put the node in open list.
				neighbourNode.Flags |= uint8(DT_NODE_OPEN)
				q.openList.push(neighbourNode)
			}

			// Update nearest node to target so far.
			if heuristic < lastBestNodeCost {
				lastBestNodeCost = heuristic
				lastBestNode = neighbourNode
			}
		}
	}

	status := q.getPathToNode(lastBestNode, path, pathCount, maxPath)

	if lastBestNode.ID != endRef {
		status |= DT_PARTIAL_RESULT
	}

	if outOfNodes {
		status |= DT_OUT_OF_NODES
	}

	return status
}

// Returns edge mid point between two polygons.
func (q *DtNavMeshQuery) getEdgeMidPoint(from DtPolyRef,
	fromPoly *DtPoly, fromTile *DtMeshTile,
	to DtPolyRef, toPoly *DtPoly, toTile *DtMeshTile, mid []float32) DtStatus {

	left := make([]float32, 3)
	right := make([]float32, 3)

	if DtStatusFailed(q.getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)) {
		return DT_FAILURE | DT_INVALID_PARAM
	}
	mid[0] = (left[0] + right[0]) * 0.5
	mid[1] = (left[1] + right[1]) * 0.5
	mid[2] = (left[2] + right[2]) * 0.5
	return DT_SUCCESS
}

// Returns portal points between two polygons.
func (q *DtNavMeshQuery) getPortalPoints(from DtPolyRef, fromPoly *DtPoly, fromTile *DtMeshTile,
	to DtPolyRef, toPoly *DtPoly, toTile *DtMeshTile,
	left, right []float32) DtStatus {

	// Find the link that points to the 'to' polygon.
	var link *DtLink
	for i := fromPoly.FirstLink; i != DT_NULL_LINK; i = fromTile.Links[i].Next {
		if fromTile.Links[i].Ref == to {
			link = &fromTile.Links[i]
			break
		}
	}
	if link == nil {
		return DT_FAILURE | DT_INVALID_PARAM
	}

	// Handle off-mesh connections.
	if fromPoly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
		// Find link that points to first vertex.
		for i := fromPoly.FirstLink; i != DT_NULL_LINK; i = fromTile.Links[i].Next {
			if fromTile.Links[i].Ref == to {
				// TODO: AR, repass here and test
				v := fromTile.Links[i].Edge
				vidx := fromPoly.Verts[v] * 3
				dtVcopy(left, fromTile.Verts[vidx:vidx+3])
				dtVcopy(right, fromTile.Verts[vidx:vidx+3])
				return DT_SUCCESS
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM
	}

	if toPoly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
		for i := toPoly.FirstLink; i != DT_NULL_LINK; i = toTile.Links[i].Next {
			if toTile.Links[i].Ref == from {
				// TODO: AR, repass here and test
				v := toTile.Links[i].Edge
				vidx := fromPoly.Verts[v] * 3
				dtVcopy(left, toTile.Verts[vidx:vidx+3])
				dtVcopy(right, toTile.Verts[vidx:vidx+3])
				return DT_SUCCESS
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM
	}

	// Find portal vertices.
	v0 := fromPoly.Verts[link.Edge]
	v1 := fromPoly.Verts[(link.Edge+1)%fromPoly.VertCount]

	// TODO: AR TO BE TESTED!
	v0idx := v0 * 3
	dtVcopy(left, fromTile.Verts[v0idx:v0idx+3])
	v1idx := v1 * 3
	dtVcopy(right, fromTile.Verts[v1idx:v1idx+3])

	// If the link is at tile boundary, dtClamp the vertices to
	// the link width.
	if link.Side != 0xff {
		// Unpack portal limits.
		if link.Bmin != 0 || link.Bmax != 255 {
			s := float32(1.0 / 255.0)
			tmin := float32(link.Bmin) * s
			tmax := float32(link.Bmax) * s
			dtVlerp(left, fromTile.Verts[v0idx:v0idx+3], fromTile.Verts[v1idx:v1idx+3], tmin)
			dtVlerp(right, fromTile.Verts[v0idx:v0idx+3], fromTile.Verts[v1idx:v1idx+3], tmax)
		}
	}

	return DT_SUCCESS
}

// Gets the path leading to the specified end node.
func (q *DtNavMeshQuery) getPathToNode(endNode *DtNode, path *[]DtPolyRef, pathCount *int32, maxPath int32) DtStatus {

	var (
		curNode *DtNode
		length  int32
	)
	// Find the length of the entire path.
	curNode = endNode

	for {
		length++
		curNode = q.nodePool.getNodeAtIdx(int32(curNode.PIdx))
		if curNode == nil {
			break
		}
	}

	// If the path cannot be fully stored then advance to the last node we will be able to store.
	curNode = endNode
	var writeCount int32
	for writeCount = length; writeCount > maxPath; writeCount-- {
		assert.True(curNode != nil, "curNode should not be nil")
		curNode = q.nodePool.getNodeAtIdx(int32(curNode.PIdx))
	}

	// Write path
	for i := writeCount - 1; i >= 0; i-- {
		assert.True(curNode != nil, "curNode should not be nil")
		(*path)[i] = curNode.ID
		curNode = q.nodePool.getNodeAtIdx(int32(curNode.PIdx))
	}

	assert.True(curNode == nil, "curNode should be nil")

	*pathCount = dtiMin(length, maxPath)

	if length > maxPath {
		return DT_SUCCESS | DT_BUFFER_TOO_SMALL
	}

	return DT_SUCCESS
}

/// @par
///
/// Uses the detail polygons to find the surface height. (Most accurate.)
///
/// @p pos does not have to be within the bounds of the polygon or navigation mesh.
///
/// See closestPointOnPolyBoundary() for a limited but faster option.
///
func (q *DtNavMeshQuery) closestPointOnPoly(ref DtPolyRef, pos, closest []float32, posOverPoly *bool) DtStatus {
	assert.True(q.nav != nil, "NavMesh should not be nil")
	var (
		tile *DtMeshTile
		poly *DtPoly
	)

	if DtStatusFailed(q.nav.TileAndPolyByRef(ref, &tile, &poly)) {
		return DT_FAILURE | DT_INVALID_PARAM
	}
	if tile == nil {
		return DT_FAILURE | DT_INVALID_PARAM
	}

	// Off-mesh connections don't have detail polygons.
	if poly.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
		var (
			v0, v1    []float32
			d0, d1, u float32
		)
		v0 = tile.Verts[poly.Verts[0]*3 : 3]
		v1 = tile.Verts[poly.Verts[1]*3 : 3]
		d0 = dtVdist(pos, v0)
		d1 = dtVdist(pos, v1)
		u = d0 / (d0 + d1)
		dtVlerp(closest, v0, v1, u)
		if posOverPoly != nil {
			*posOverPoly = false
		}
		return DT_SUCCESS
	}

	e := uintptr(unsafe.Pointer(poly)) - uintptr(unsafe.Pointer(&tile.Polys[0]))
	ip := uint32(e / unsafe.Sizeof(*poly))

	assert.True(ip < uint32(len(tile.Polys)), "ip should be < len(tile.Polys), ip=%d, len(tile.Polys)=%d", ip, len(tile.Polys))

	pd := &tile.DetailMeshes[ip]

	fmt.Println("ip", ip)
	fmt.Println("pd", pd)

	// Clamp point to be inside the polygon.
	verts := make([]float32, DT_VERTS_PER_POLYGON*3)
	edged := make([]float32, DT_VERTS_PER_POLYGON)
	edget := make([]float32, DT_VERTS_PER_POLYGON)
	nv := poly.VertCount
	var i uint8
	for i = 0; i < nv; i++ {
		// TODO: could probably use copy
		idx := i * 3
		jdx := poly.Verts[i] * 3
		dtVcopy(verts[idx:idx+3], tile.Verts[jdx:jdx+3])
	}

	dtVcopy(closest, pos)
	if !dtDistancePtPolyEdgesSqr(pos, verts, int32(nv), edged, edget) {
		// Point is outside the polygon, dtClamp to nearest edge.
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
		dtVlerp(closest, va, vb, edget[imin])

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
		if dtClosestHeightPointTriangle(closest, v[0], v[1], v[2], &h) {
			closest[1] = h
			break
		}
	}
	return DT_SUCCESS
}

/// Finds the polygon nearest to the specified center point.
///  @param[in]		center		The center of the search box. [(x, y, z)]
///  @param[in]		extents		The search distance along each axis. [(x, y, z)]
///  @param[in]		filter		The polygon filter to apply to the query.
///  @param[out]	nearestRef	The reference id of the nearest polygon.
///  @param[out]	nearestPt	The nearest point on the polygon. [opt] [(x, y, z)]
/// @returns The status flags for the query.
/// @note If the search box does not intersect any polygons the search will
/// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check
/// @p nearestRef before using @p nearestPt.
///
func (q *DtNavMeshQuery) FindNearestPoly(center, extents []float32,
	filter *DtQueryFilter,
	nearestRef *DtPolyRef, nearestPt []float32) DtStatus {

	assert.True(q.nav != nil, "Nav should not be nil")

	if nearestRef == nil {
		return DT_FAILURE | DT_INVALID_PARAM
	}

	query := newDtFindNearestPolyQuery(q, center)

	status := q.queryPolygons4(center, extents, filter, query)
	if DtStatusFailed(status) {
		return status
	}

	*nearestRef = query.NearestRef()
	// Only override nearestPt if we actually found a poly so the nearest point
	// is valid.
	if len(nearestPt) == 3 && *nearestRef != 0 {
		pt := query.NearestPoint()
		dtVcopy(nearestPt[:], pt[:])
	}
	return DT_SUCCESS
}

/// @par
///
/// If no polygons are found, the function will return #DT_SUCCESS with a
/// @p polyCount of zero.
///
/// If @p polys is too small to hold the entire result set, then the array will
/// be filled to capacity. The method of choosing which polygons from the
/// full set are included in the partial result set is undefined.
///
func (q *DtNavMeshQuery) queryPolygons6(center, extents []float32,
	filter *DtQueryFilter,
	polys []DtPolyRef, polyCount *int32, maxPolys int32) DtStatus {
	if polys == nil || polyCount == nil || maxPolys < 0 {
		return DT_FAILURE | DT_INVALID_PARAM
	}

	collector := newDtCollectPolysQuery(polys, maxPolys)

	status := q.queryPolygons4(center, extents, filter, collector)
	if DtStatusFailed(status) {
		return status
	}

	*polyCount = collector.numCollected
	if collector.overflow {
		return DT_SUCCESS | DT_BUFFER_TOO_SMALL
	}
	return DT_SUCCESS
}

/// @par
///
/// The query will be invoked with batches of polygons. Polygons passed
/// to the query have bounding boxes that overlap with the center and extents
/// passed to this function. The dtPolyQuery::process function is invoked multiple
/// times until all overlapping polygons have been processed.
///
func (q *DtNavMeshQuery) queryPolygons4(center, extents []float32,
	filter *DtQueryFilter, query DtPolyQuery) DtStatus {
	assert.True(q.nav != nil, "navmesh should not be nill")

	if len(center) != 3 || len(extents) != 3 || filter == nil || query == nil {
		return DT_FAILURE | DT_INVALID_PARAM
	}

	var bmin, bmax [3]float32
	dtVsub(bmin[:], center, extents)
	dtVadd(bmax[:], center, extents)

	// Find tiles the query touches.
	var minx, miny, maxx, maxy int32
	q.nav.calcTileLoc(bmin, &minx, &miny)
	q.nav.calcTileLoc(bmax, &maxx, &maxy)

	MAX_NEIS := int32(32)
	neis := make([]*DtMeshTile, MAX_NEIS)

	for y := miny; y <= maxy; y++ {
		for x := minx; x <= maxx; x++ {
			nneis := q.nav.TilesAt(x, y, neis, MAX_NEIS)
			for j := int32(0); j < nneis; j++ {
				q.queryPolygonsInTile(neis[j], bmin[:], bmax[:], filter, query)
			}
		}
	}

	return DT_SUCCESS
}

func (q *DtNavMeshQuery) queryPolygonsInTile(tile *DtMeshTile, qmin, qmax []float32,
	filter *DtQueryFilter, query DtPolyQuery) {
	assert.True(q.nav != nil, "navmesh should not be nill")
	batchSize := int32(32)

	polyRefs := make([]DtPolyRef, batchSize)
	polys := make([]*DtPoly, batchSize)
	var n int32

	if len(tile.BvTree) > 0 {

		var (
			node            *dtBVNode
			nodeIdx, endIdx int32
			tbmin, tbmax    [3]float32
			qfac            float32
		)

		//node = &tile.BvTree[0]
		//end = &tile.BvTree[tile.Header.BvNodeCount]
		nodeIdx = 0
		endIdx = tile.Header.BvNodeCount

		tbmin = tile.Header.Bmin
		tbmax = tile.Header.Bmax
		qfac = tile.Header.BvQuantFactor

		// Calculate quantized box
		var bmin, bmax [3]uint16
		// dtClamp query box to world box.
		minx := dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
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
			fmt.Println("node", node)
			overlap := dtOverlapQuantBounds(bmin[:], bmax[:], node.Bmin[:], node.Bmax[:])
			fmt.Println("overlap", overlap)
			fmt.Println("node.I", node.I)
			isLeafNode := node.I >= 0

			if isLeafNode && overlap {
				fmt.Println("base | DtPolyRef(node.I)", base, "|", DtPolyRef(node.I))
				ref := base | DtPolyRef(node.I)
				fmt.Printf("0x%x | 0x%x = 0x%x\n", base, DtPolyRef(node.I), ref)
				fmt.Println("ref", ref)
				fmt.Println("node.I", node.I)
				fmt.Println("len(tile.Poly)", len(tile.Polys))
				if filter.passFilter(ref, tile, &tile.Polys[node.I]) {
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
		var bmin, bmax [3]float32
		base := q.nav.getPolyRefBase(tile)
		for i := int32(0); i < tile.Header.PolyCount; i++ {
			p := &tile.Polys[i]
			// Do not return off-mesh connection polygons.
			if p.Type() == DT_POLYTYPE_OFFMESH_CONNECTION {
				continue
			}
			// Must pass filter
			ref := base | DtPolyRef(i)
			if !filter.passFilter(ref, tile, p) {
				continue
			}
			// Calc polygon bounds.
			vidx := p.Verts[0] * 3
			v := tile.Verts[vidx : vidx+3]
			dtVcopy(bmin[:], v)
			dtVcopy(bmax[:], v)
			for j := uint8(1); j < p.VertCount; j++ {
				vidx = p.Verts[j] * 3
				v = tile.Verts[vidx : vidx+3]
				dtVmin(bmin[:], v)
				dtVmax(bmax[:], v)
			}
			if dtOverlapBounds(qmin, qmax, bmin[:], bmax[:]) {
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

// Gets the node pool.
// @returns The node pool.
func (q *DtNavMeshQuery) NodePool() *DtNodePool {
	return q.nodePool
}

// Gets the navigation mesh the query object is using.
// @return The navigation mesh the query object is using.
func (q *DtNavMeshQuery) AttachedNavMesh() *DtNavMesh {
	return q.nav
}
