package crowd

import (
	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/gogeo/f32/d3"
)

// A PathCorridor represents a dynamic polygon corridor used to plan agent
// movement.

// The corridor is loaded with a path, usually obtained from a
// detour.NavMeshQuery.FindPath() query. The corridor is then used to plan local
// movement, with the corridor automatically updating as needed to deal with
// inaccurate agent locomotion.
//
// Example of a common use case:
//
// - Construct a new path corridor object.
// - Obtain a path from a detour.NavMeshQuery object.
// - Use Reset() to set the agent's current position. (At the beginning of the
//   path.)
// - Use SetCorridor() to load the path and target.
// - Use FindCorners() to plan movement. (This handles dynamic path
//   straightening.)
// - Use MovePosition() to feed agent movement back into the corridor. (The
//   corridor will automatically adjust as needed.)
// - If the target is moving, use MoveTargetPosition() to update the end of the
//   corridor. (The corridor will automatically adjust as needed.)
// - Repeat the previous 3 steps to continue to move the agent.
//
// The corridor position and target are always constrained to the navigation
// mesh.
//
// One of the difficulties in maintaining a path is that floating point errors,
// locomotion inaccuracies, and/or local steering can result in the agent
// crossing the boundary of the path corridor, temporarily invalidating the
// path. This class uses local mesh queries to detect and update the corridor
// as needed to handle these types of issues.
//
// The fact that local mesh queries are used to move the position and target
// locations results in two beahviors that need to be considered:
//
// Every time a move function is used there is a chance that the path will
// become non-optimal. Basically, the further the target is moved from its
// original location, and the further the position is moved outside the original
// corridor, the more likely the path will become non-optimal. This issue can be
// addressed by periodically running the OptimizePathTopology() and
// OptimizePathVisibility() methods.
//
// All local mesh queries have distance limitations. (Review the
// detour.NavMeshQuery methods for details.) So the most accurate use case is to
// move the position and target in small increments. If a large increment is
// used, then the corridor may not be able to accurately find the new location.
// Because of this limitation, if a position is moved in a large increment, then
// compare the desired and resulting polygon references. If the two do not
// match, then path replanning may be needed. E.g. If you move the target, check
// LastPoly() to see if it is the expected polygon.
type PathCorridor struct {
	pos    [3]float32
	target [3]float32

	path    []detour.PolyRef
	npath   int
	maxPath int
}

// Allocates the corridor's path buffer.
//
//  Arguments:
//    maxPath  The maximum path size the corridor can handle.
//
// Return True if the initialization succeeded.
func (pc *PathCorridor) init(maxPath int) bool {
	pc.path = make([]detour.PolyRef, maxPath)
	pc.maxPath = maxPath
	return true
}

// Reset resets the path corridor to the specified position.
//
//  Arguments:
//  ref       The polygon reference containing the position.
//  pos       The new position in the corridor. [(x, y, z)]
//
// Essentially, the corridor is set of one polygon in size with the target equal
// to the position.
func (pc *PathCorridor) Reset(ref detour.PolyRef, pos d3.Vec3) {
	copy(pc.pos[:], pos[:3])
	copy(pc.target[:], pos[:3])
	pc.path[0] = ref
	pc.npath = 1
}

// FindCorners finds the corners in the corridor from the position toward
// the target. (The straightened path.)
//
//  Arguments:
//   cornerVerts The corner vertices. [(x, y, z) * cornerCount]
//               [Size: <= maxCorners]
//   cornerFlags The flag for each corner. [(flag) * cornerCount]
//               [Size: <= maxCorners]
//   cornerPolys The polygon reference for each corner.
//               [(polyRef) * cornerCount] [Size: <= @p maxCorners]
//   maxCorners  The maximum number of corners the buffers can hold.
//   navquery    The query object used to build the corridor.
//   filter      The filter to apply to the operation.
//
// Return the number of corners returned in the corner buffers.
// [0 <= value <= maxCorners]
//
// This is the function used to plan local movement within the corridor. One or
// more corners can be detected in order to plan movement. It performs
// essentially the same function as detour.NavMeshQuery.FindStraightPath.
//
// Due to internal optimizations, the maximum number of corners returned will be
// (maxCorners - 1). For example: If the buffers are sized to hold 10 corners,
// the function will never return more than 9 corners. So if 10 corners are
// needed, the buffers should be sized for 11 corners.
//
// If the target is within range, it will be the last corner and have a polygon
// reference id of zero.
func (pc *PathCorridor) FindCorners(cornerVerts []d3.Vec3, cornerFlags []uint8,
	cornerPolys []detour.PolyRef, maxCorners int,
	navquery *detour.NavMeshQuery, filter detour.QueryFilter) int {

	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}
	if pc.npath == 0 {
		panic("pc.m_npath should be != 0")
	}

	const MIN_TARGET_DIST = 0.01

	ncorners, _ := navquery.FindStraightPath(pc.pos[:], pc.target[:], pc.path,
		cornerVerts, cornerFlags, cornerPolys, int32(maxCorners))

	// Prune points in the beginning of the path which are too close.
	for ncorners != 0 {
		if (cornerFlags[0]&detour.StraightPathOffMeshConnection) != 0 ||
			d3.Vec3Dist2DSqr(pc.pos[:], cornerVerts[0]) > MIN_TARGET_DIST*MIN_TARGET_DIST {
			break
		}
		ncorners--
		if ncorners != 0 {
			copy(cornerFlags, cornerFlags[1:1+ncorners])
			copy(cornerPolys, cornerPolys[1:1+ncorners])
			copy(cornerVerts, cornerVerts[3:3*(1+ncorners)])
		}
	}

	// Prune points after an off-mesh connection.
	for i := 0; i < ncorners; i++ {
		if (cornerFlags[i] & detour.StraightPathOffMeshConnection) != 0 {
			ncorners = i + 1
			break
		}
	}

	return ncorners
}

// OptimizePathVisibility attempts to optimize the path if the specified point
// is visible from the current position.
//
//  Arguments:
//   next                  The point to search toward. [(x, y, z])
//   pathOptimizationRange The maximum range to search. [Limit: > 0]
//   navquer               The query object used to build the corridor.
//   filter                The filter to apply to the operation.
//
// Inaccurate locomotion or dynamic obstacle avoidance can force the argent
// position significantly outside the original corridor. Over time this can
// result in the formation of a non-optimal corridor. Non-optimal paths can also
// form near the corners of tiles.
//
// This function uses an efficient local visibility search to try to optimize
// the corridor between the current position and @p next.
//
// The corridor will change only if @p next is visible from the current position
// and moving directly toward the point is better than following the existing
// path.
//
// The more inaccurate the agent movement, the more beneficial this function
// becomes. Simply adjust the frequency of the call to match the needs to the
// agent.
//
// This function is not suitable for long distance searches.
func (pc *PathCorridor) OptimizePathVisibility(next d3.Vec3, pathOptimizationRange float32, navquery *detour.NavMeshQuery, filter detour.QueryFilter) {
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}

	// Clamp the ray to max distance.
	goal := d3.NewVec3From(next)
	dist := goal.Dist2D(pc.pos[:])

	// If too close to the goal, do not try to optimize.
	if dist < 0.01 {
		return
	}

	// Overshoot a little. This helps to optimize open fields in tiled meshes.
	dist += 0.01
	if pathOptimizationRange < dist {
		dist = pathOptimizationRange
	}

	// Adjust ray length.
	delta := goal.Sub(pc.pos[:])
	d3.Vec3Mad(goal, pc.pos[:], delta, pathOptimizationRange/dist)

	const MAX_RES = 32
	var (
		res  [MAX_RES]detour.PolyRef
		t    float32
		norm [3]float32
		nres int = 0
	)

	nres, t, _ = navquery.Raycast2(pc.path[0], pc.pos[:], goal, filter, norm[:], res[:], MAX_RES)
	if nres > 1 && t > 0.99 {
		pc.npath = MergeCorridorStartShortcut(pc.path, pc.npath, pc.maxPath, res[:], nres)
	}
}

// OptimizePathTopology attempts to optimize the path using a local area search.
// (Partial replanning.)
//
//  Arguments:
//   navquery  The query object used to build the corridor.
//   filter    The filter to apply to the operation.
//
// Inaccurate locomotion or dynamic obstacle avoidance can force the agent
// position significantly outside the original corridor. Over time this can
// result in the formation of a non-optimal corridor. This function will use a
// local area path search to try to re-optimize the corridor.
//
// The more inaccurate the agent movement, the more beneficial this function
// becomes. Simply adjust the frequency of the call to match the needs to the
// agent.
func (pc *PathCorridor) OptimizePathTopology(navquery *detour.NavMeshQuery, filter detour.QueryFilter) bool {
	if navquery == nil {
		panic("navquery should not be nil")
	}
	if filter == nil {
		panic("filter should not be nil")
	}
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}

	if pc.npath < 3 {
		return false
	}

	const (
		MAX_ITER = 32
		MAX_RES  = 32
	)

	var res [MAX_RES]detour.PolyRef
	navquery.InitSlicedFindPath(pc.path[0], pc.path[pc.npath-1], pc.pos[:], pc.target[:], filter, 0)
	navquery.UpdateSlicedFindPath(MAX_ITER, nil)
	nres, status := navquery.FinalizeSlicedFindPathPartial(pc.path, pc.npath, res[:], MAX_RES)

	if detour.StatusSucceed(status) && nres > 0 {
		pc.npath = MergeCorridorStartShortcut(pc.path, pc.npath, pc.maxPath, res[:], nres)
		return true
	}

	return false
}

func (pc *PathCorridor) MoveOverOffmeshConnection(offMeshConRef detour.PolyRef, refs []detour.PolyRef, startPos, endPos d3.Vec3, navquery *detour.NavMeshQuery) bool {
	if navquery == nil {
		panic("navquery should not be nil")
	}
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}
	if pc.npath == 0 {
		panic("pc.m_npath should not be 0")
	}

	// Advance the path up to and over the off-mesh connection.
	var (
		prevRef, polyRef detour.PolyRef = 0, pc.path[0]
		npos             int            = 0
	)
	for npos < pc.npath && polyRef != offMeshConRef {
		prevRef = polyRef
		polyRef = pc.path[npos]
		npos++
	}
	if npos == pc.npath {
		// Could not find offMeshConRef
		return false
	}

	// Prune path
	for i := npos; i < pc.npath; i++ {
		pc.path[i-npos] = pc.path[i]
	}
	pc.npath -= npos

	refs[0] = prevRef
	refs[1] = polyRef

	nav := navquery.AttachedNavMesh()
	if nav == nil {
		panic("nav should not be nil")
	}

	status := nav.OffMeshConnectionPolyEndPoints(refs[0], refs[1], startPos, endPos)
	if detour.StatusSucceed(status) {
		copy(pc.pos[:], endPos)
		return true
	}

	return false
}

func (pc *PathCorridor) FixPathStart(safeRef detour.PolyRef, safePos d3.Vec3) bool {
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}

	copy(pc.pos[:], safePos[:3])
	if pc.npath < 3 && pc.npath > 0 {
		pc.path[2] = pc.path[pc.npath-1]
		pc.path[0] = safeRef
		pc.path[1] = 0
		pc.npath = 3
	} else {
		pc.path[0] = safeRef
		pc.path[1] = 0
	}

	return true
}

func (pc *PathCorridor) TrimInvalidPath(safeRef detour.PolyRef, safePos d3.Vec3,
	navquery *detour.NavMeshQuery, filter detour.QueryFilter) bool {
	if navquery == nil {
		panic("navquery should not be nil")
	}
	if filter == nil {
		panic("filter should not be nil")
	}
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}

	// Keep valid path as far as possible.
	var n int
	for n < pc.npath && navquery.IsValidPolyRef(pc.path[n], filter) {
		n++
	}

	if n == pc.npath {
		// All valid, no need to fix.
		return true
	} else if n == 0 {
		// The first polyref is bad, use current safe values.
		copy(pc.pos[:], safePos[:3])
		pc.path[0] = safeRef
		pc.npath = 1
	} else {
		// The path is partially usable.
		pc.npath = n
	}

	// Clamp target pos to last poly
	tgt := d3.NewVec3From(pc.target[:])
	navquery.ClosestPointOnPolyBoundary(pc.path[pc.npath-1], tgt, pc.target[:])

	return true
}

// IsValid checks the current corridor path to see if its polygon references
// remain valid.
//
//  Arguments:
//   maxLookAhead  The number of polygons from the beginning of the corridor to
//                 search.
//   navquery      The query object used to build the corridor.
//   filter        The filter to apply to the operation.
//
// The path can be invalidated if there are structural changes to the underlying
// navigation mesh, or the state of a polygon within the path changes resulting
// in it being filtered out. (E.g. An exclusion or inclusion flag changes.)
func (pc *PathCorridor) IsValid(maxLookAhead int, navquery *detour.NavMeshQuery, filter detour.QueryFilter) bool {
	// Check that all polygons still pass query filter.
	n := pc.npath
	if maxLookAhead < pc.npath {
		n = maxLookAhead
	}
	for i := 0; i < n; i++ {
		if !navquery.IsValidPolyRef(pc.path[i], filter) {
			return false
		}
	}

	return true
}

// MovePosition moves the position from the current location to the desired
// location, adjusting the corridor as needed to reflect the change.
//
//  Arguments:
//   npos     The desired new position. [(x, y, z)]
//   navquery The query object used to build the corridor.
//   filter   The filter to apply to the operation.
//
// Returns true if move succeeded.
//
// Behavior:
//
// - The movement is constrained to the surface of the navigation mesh.
// - The corridor is automatically adjusted (shorted or lengthened) in order to
//   remain valid.
// - The new position will be located in the adjusted corridor's first polygon.
//
// The expected use case is that the desired position will be 'near' the current
// corridor. What is considered 'near' depends on local polygon density, query
// search extents, etc.
//
// The resulting position will differ from the desired position if the desired
// position is not on the navigation mesh, or it can't be reached using a local
// search.
func (pc *PathCorridor) MovePosition(npos d3.Vec3, navquery *detour.NavMeshQuery, filter detour.QueryFilter) bool {
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}
	if pc.npath == 0 {
		panic("pc.m_npath should not be 0")
	}

	// Move along navmesh and update new position.
	result := d3.NewVec3()
	const MAX_VISITED = 16
	var (
		visited  [MAX_VISITED]detour.PolyRef
		nvisited int = 0
	)
	status := navquery.MoveAlongSurface(pc.path[0], pc.pos[:], npos, filter,
		result, visited[:], &nvisited, MAX_VISITED)
	if detour.StatusSucceed(status) {
		pc.npath = MergeCorridorStartMoved(pc.path, pc.npath, pc.maxPath, visited[:], nvisited)

		// Adjust the position to stay on top of the navmesh.
		h, _ := navquery.PolyHeight(pc.path[0], result)
		result[1] = h
		d3.Vec3Copy(pc.pos[:], result)
		return true
	}
	return false
}

// MoveTargetPosition moves the target from the curent location to the desired
// location, adjusting the corridor as needed to reflect the change.
//
//  Arguments:
//   npos     The desired new target position. [(x, y, z)]
//   navquery The query object used to build the corridor.
//   filter   The filter to apply to the operation.
//
// Returns true if move succeeded.
//
// Behavior:
//
// - The movement is constrained to the surface of the navigation mesh.
// - The corridor is automatically adjusted (shorted or lengthened) in order to
//   remain valid.
// - The new target will be located in the adjusted corridor's last polygon.
//
// The expected use case is that the desired target will be 'near' the current
// corridor. What is considered 'near' depends on local polygon density, query
// search extents, etc.
//
// The resulting target will differ from the desired target if the desired
// target is not on the navigation mesh, or it can't be reached using a local
// search.
func (pc *PathCorridor) MoveTargetPosition(npos d3.Vec3, navquery *detour.NavMeshQuery, filter detour.QueryFilter) bool {
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}
	if pc.npath == 0 {
		panic("pc.m_npath should not be 0")
	}

	// Move along navmesh and update new position.
	const MAX_VISITED = 16
	var (
		visited  [MAX_VISITED]detour.PolyRef
		result       = d3.NewVec3()
		nvisited int = 0
	)
	status := navquery.MoveAlongSurface(pc.path[pc.npath-1], pc.target[:], npos, filter,
		result, visited[:], &nvisited, MAX_VISITED)
	if detour.StatusSucceed(status) {
		pc.npath = MergeCorridorEndMoved(pc.path, pc.npath, pc.maxPath, visited[:], nvisited)
		// TODO: should we do that?
		// Adjust the position to stay on top of the navmesh.
		/*	float h = m_target[1];
			navquery->getPolyHeight(m_path[m_npath-1], result, &h);
			result[1] = h;*/

		d3.Vec3Copy(pc.target[:], result)
		return true
	}
	return false
}

// SetCorridor loads a new path and target into the corridor.
//
//  Arguments:
//   target   The target location within the last polygon of the path.
//            [(x, y, z)]
//   path     The path corridor. [(polyRef) * npolys]
//   npath    The number of polygons in the path.
func (pc *PathCorridor) SetCorridor(target d3.Vec3, path []detour.PolyRef, npath int) {
	if pc.path == nil {
		panic("pc.m_path should not be nil")
	}
	if npath <= 0 {
		panic("npath should be positive")
	}
	if npath >= pc.maxPath {
		panic("we should have npath < m_maxpath")
	}

	copy(pc.target[:], target[:])
	copy(pc.path, path[:npath])
	pc.npath = npath
}

// Pos gets the current position within the corridor. (In the first polygon.)
func (pc *PathCorridor) Pos() d3.Vec3 {
	return pc.pos[:]
}

// Target gets the current target within the corridor. (In the last polygon.)
func (pc *PathCorridor) Target() d3.Vec3 {
	return pc.target[:]
}

// FirstPoly is the polygon reference id of the first polygon in the corridor,
// the polygon containing the position. (Or zero if there is no path.)
func (pc *PathCorridor) FirstPoly() detour.PolyRef {
	if pc.npath != 0 {
		return pc.path[0]
	}
	return 0
}

// LastPoly is the polygon reference id of the last polygon in the corridor, the
// polygon containing the target. (Or zero if there is no path.)
func (pc *PathCorridor) LastPoly() detour.PolyRef {
	if pc.npath != 0 {
		return pc.path[pc.npath-1]
	}
	return 0
}

// Path returns the corridor's path. [(polyRef) * #PathCount()]
func (pc *PathCorridor) Path() []detour.PolyRef {
	return pc.path
}

// PathCount returns the number of polygons in the current corridor path.
func (pc *PathCorridor) PathCount() int {
	return pc.npath
}

func MergeCorridorStartMoved(path []detour.PolyRef, npath, maxPath int,
	visited []detour.PolyRef, nvisited int) int {
	var (
		furthestPath    int = -1
		furthestVisited int = -1
	)

	// Find furthest common polygon.
	for i := npath - 1; i >= 0; i-- {
		found := false
		for j := nvisited - 1; j >= 0; j-- {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.

	// Adjust beginning of the buffer to include the visited.
	req := nvisited - furthestVisited
	orig := furthestPath + 1
	if npath < orig {
		orig = npath
	}
	size := 0
	if npath-orig > 0 {
		size = npath - orig
	}
	if req+size > maxPath {
		size = maxPath - req
	}
	if size > 0 {
		copy(path[req:], path[orig:orig+size])
	}

	// Store visited
	for i := 0; i < req; i++ {
		path[i] = visited[(nvisited-1)-i]
	}

	return req + size
}

func MergeCorridorEndMoved(path []detour.PolyRef, npath, maxPath int,
	visited []detour.PolyRef, nvisited int) int {
	var (
		furthestPath    int = -1
		furthestVisited int = -1
	)

	// Find furthest common polygon.
	for i := 0; i < npath; i++ {
		found := false
		for j := nvisited - 1; j >= 0; j-- {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.
	ppos := furthestPath + 1
	vpos := furthestVisited + 1
	count := nvisited - vpos
	if maxPath-ppos < count {
		count = maxPath - ppos
	}
	if ppos+count > maxPath {
		panic("ppos+count should be <= maxPath")
	}
	if count != 0 {
		copy(path[ppos:], visited[vpos:vpos+count])
	}

	return ppos + count
}

func MergeCorridorStartShortcut(path []detour.PolyRef, npath, maxPath int,
	visited []detour.PolyRef, nvisited int) int {
	var (
		furthestPath    int = -1
		furthestVisited int = -1
	)

	// Find furthest common polygon.
	for i := npath - 1; i >= 0; i-- {
		found := false
		for j := nvisited - 1; j >= 0; j-- {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.

	// Adjust beginning of the buffer to include the visited.
	req := furthestVisited
	if req <= 0 {
		return npath
	}

	orig := furthestPath
	size := npath - orig
	if size < 0 {
		size = 0
	}
	if req+size > maxPath {
		size = maxPath - req
	}
	if size != 0 {
		copy(path[req:], path[orig:orig+size])
	}

	// Store visited
	for i := 0; i < req; i++ {
		path[i] = visited[i]
	}

	return req + size
}
