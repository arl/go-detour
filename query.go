package detour

/// Provides the ability to perform pathfinding related queries against
/// a navigation mesh.
/// @ingroup detour
type dtNavMeshQuery struct {

	/// @name Standard Pathfinding Functions
	// /@{

	/// Finds a path from the start polygon to the end polygon.
	///  @param[in]		startRef	The refrence id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	//dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef,
	//const float* startPos, const float* endPos,
	//const dtQueryFilter* filter,
	//dtPolyRef* path, int* pathCount, const int maxPath) const;

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
	//const dtPolyRef* path, const int pathSize,
	//float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
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
	//dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
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
	//dtStatus finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath);

	///// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	///// polygon on the existing path that was visited during the search.
	/////  @param[in]		existing		An array of polygon references for the existing path.
	/////  @param[in]		existingSize	The number of polygon in the @p existing array.
	/////  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.)
	/////  								[(polyRef) * @p pathCount]
	/////  @param[out]	pathCount		The number of polygons returned in the @p path array.
	/////  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	///// @returns The status flags for the query.
	//dtStatus finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
	//dtPolyRef* path, int* pathCount, const int maxPath);

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
	//dtStatus findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
	//const dtQueryFilter* filter,
	//dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
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
	//dtStatus findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
	//const dtQueryFilter* filter,
	//dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
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
	//dtStatus getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) const;

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
	//dtPolyRef* nearestRef, float* nearestPt) const;

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
	//dtPolyRef* polys, int* polyCount, const int maxPolys) const;

	///// Finds polygons that overlap the search box.
	/////  @param[in]		center		The center of the search box. [(x, y, z)]
	/////  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	/////  @param[in]		filter		The polygon filter to apply to the query.
	/////  @param[in]		query		The query. Polygons found will be batched together and passed to this query.
	//dtStatus queryPolygons(const float* center, const float* extents,
	//const dtQueryFilter* filter, dtPolyQuery* query) const;

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
	//dtStatus findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
	//const dtQueryFilter* filter,
	//dtPolyRef* resultRef, dtPolyRef* resultParent,
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
	//dtStatus moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
	//const dtQueryFilter* filter,
	//float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;

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
	//dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
	//const dtQueryFilter* filter,
	//float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const;

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
	//dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
	//const dtQueryFilter* filter, const unsigned int options,
	//dtRaycastHit* hit, dtPolyRef prevRef = 0) const;

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
	//dtStatus findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
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
	//dtStatus getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
	//float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
	//const int maxSegments) const;

	///// Returns random location on navmesh.
	///// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	/////  @param[in]		filter			The polygon filter to apply to the query.
	/////  @param[in]		frand			Function returning a random number [0..1).
	/////  @param[out]	randomRef		The reference id of the random location.
	/////  @param[out]	randomPt		The random location.
	///// @returns The status flags for the query.
	//dtStatus findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
	//dtPolyRef* randomRef, float* randomPt) const;

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
	//dtStatus findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius,
	//const dtQueryFilter* filter, float (*frand)(),
	//dtPolyRef* randomRef, float* randomPt) const;

	///// Finds the closest point on the specified polygon.
	/////  @param[in]		ref			The reference id of the polygon.
	/////  @param[in]		pos			The position to check. [(x, y, z)]
	/////  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
	/////  @param[out]	posOverPoly	True of the position is over the polygon.
	///// @returns The status flags for the query.
	//dtStatus closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;

	///// Returns a point on the boundary closest to the source point if the source point is outside the
	///// polygon's xz-bounds.
	/////  @param[in]		ref			The reference id to the polygon.
	/////  @param[in]		pos			The position to check. [(x, y, z)]
	/////  @param[out]	closest		The closest point. [(x, y, z)]
	///// @returns The status flags for the query.
	//dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const;

	///// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	/////  @param[in]		ref			The reference id of the polygon.
	/////  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	/////  @param[out]	height		The height at the surface of the polygon.
	///// @returns The status flags for the query.
	//dtStatus getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;

	///// @}
	///// @name Miscellaneous Functions
	///// @{

	///// Returns true if the polygon reference is valid and passes the filter restrictions.
	/////  @param[in]		ref			The polygon reference to check.
	/////  @param[in]		filter		The filter to apply.
	//bool isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const;

	///// Returns true if the polygon reference is in the closed list.
	/////  @param[in]		ref		The reference id of the polygon to check.
	///// @returns True if the polygon is in closed list.
	//bool isInClosedList(dtPolyRef ref) const;

	///// Gets the node pool.
	///// @returns The node pool.
	//class dtNodePool* getNodePool() const { return m_nodePool; }

	///// Gets the navigation mesh the query object is using.
	///// @return The navigation mesh the query object is using.
	//const dtNavMesh* getAttachedNavMesh() const { return m_nav; }

	///// @}

	//private:
	//// Explicitly disabled copy constructor and copy assignment operator
	//dtNavMeshQuery(const dtNavMeshQuery&);
	//dtNavMeshQuery& operator=(const dtNavMeshQuery&);

	///// Queries polygons within a tile.
	//void queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
	//const dtQueryFilter* filter, dtPolyQuery* query) const;

	///// Returns portal points between two polygons.
	//dtStatus getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
	//unsigned char& fromType, unsigned char& toType) const;
	//dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
	//dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
	//float* left, float* right) const;

	///// Returns edge mid point between two polygons.
	//dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	//dtStatus getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
	//dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
	//float* mid) const;

	//// Appends vertex to a straight path
	//dtStatus appendVertex(const float* pos, const unsigned char flags, const dtPolyRef ref,
	//float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	//int* straightPathCount, const int maxStraightPath) const;

	//// Appends intermediate portal points to a straight path.
	//dtStatus appendPortals(const int startIdx, const int endIdx, const float* endPos, const dtPolyRef* path,
	//float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	//int* straightPathCount, const int maxStraightPath, const int options) const;

	//// Gets the path leading to the specified end node.
	//dtStatus getPathToNode(struct dtNode* endNode, dtPolyRef* path, int* pathCount, int maxPath) const;

	m_nav   *DtNavMesh  ///< Pointer to navmesh data.
	m_query dtQueryData ///< Sliced query state.

	m_tinyNodePool *dtNodePool  ///< Pointer to small node pool.
	m_nodePool     *dtNodePool  ///< Pointer to node pool.
	m_openList     *dtNodeQueue ///< Pointer to open list queue.
}

type dtQueryData struct {
	status           dtStatus
	lastBestNode     *dtNode
	lastBestNodeCost float32
	startRef, endRef dtPolyRef
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
func (q *dtNavMeshQuery) init(nav *DtNavMesh, maxNodes int32) dtStatus {
	if maxNodes > int32(DT_NULL_IDX) || maxNodes > int32(1<<DT_NODE_PARENT_BITS)-1 {
		return DT_FAILURE | DT_INVALID_PARAM
	}

	q.m_nav = nav

	if q.m_nodePool == nil || q.m_nodePool.getMaxNodes() < maxNodes {
		if q.m_nodePool != nil {
			//m_nodePool.~dtNodePool();
			//dtFree(m_nodePool);
			q.m_nodePool = nil
		}
		//m_nodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(maxNodes, dtNextPow2(maxNodes/4));
		q.m_nodePool = newDtNodePool(maxNodes, int32(dtNextPow2(uint32(maxNodes/4))))
		if q.m_nodePool == nil {
			return DT_FAILURE | DT_OUT_OF_MEMORY
		}
	} else {
		q.m_nodePool.clear()
	}

	if q.m_tinyNodePool == nil {
		q.m_tinyNodePool = newDtNodePool(64, 32)
		if q.m_tinyNodePool == nil {
			return DT_FAILURE | DT_OUT_OF_MEMORY
		}
	} else {
		q.m_tinyNodePool.clear()
	}

	if q.m_openList == nil || q.m_openList.getCapacity() < maxNodes {
		if q.m_openList != nil {
			//m_openList.~dtNodeQueue();
			//dtFree(m_openList);
			q.m_openList = nil
		}
		q.m_openList = newDtNodeQueue(maxNodes)
		if q.m_openList == nil {
			return DT_FAILURE | DT_OUT_OF_MEMORY
		}
	} else {
		q.m_openList.clear()
	}

	return DT_SUCCESS
}
