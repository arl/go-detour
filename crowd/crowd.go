// This section contains detailed documentation for members that don't have a
// source file. It reduces clutter in the main section of the header.
//
// Members in this module implement local steering and dynamic avoidance
// features.
//
// The crowd is the big beast of the navigation features. It not only handles a
// lot of the path management for you, but also local steering and dynamic
// avoidance between members of the crowd. I.e. It can keep your agents from
// running into each other.
//
// Main class: Crowd
//
// The NavMeshQuery and PathCorridor classes provide perfectly good, easy to use
// path planning features. But in the end they only give you points that your
// navigation client should be moving toward. When it comes to deciding things
// like agent velocity and steering to avoid other agents, that is up to you to
// implement. Unless, of course, you decide to use Crowd.
//
// Basically, you add an agent to the crowd, providing various configuration
// settings such as maximum speed and acceleration. You also provide a local
// target to more toward. The crowd manager then provides, with every update,
// the new agent position and velocity for the frame. The movement will be
// constrained to the navigation mesh, and steering will be applied to ensure
// agents managed by the crowd do not collide with each other.
//
// This is very powerful feature set. But it comes with limitations.
//
// The biggest limitation is that you must give control of the agent's position
// completely over to the crowd manager. You can update things like maximum
// speed and acceleration. But in order for the crowd manager to do its thing,
// it can't allow you to constantly be giving it overrides to position and
// velocity. So you give up direct control of the agent's movement. It belongs
// to the crowd.
//
// The second biggest limitation revolves around the fact that the crowd manager
// deals with local planning. So the agent's target should never be more than
// 256 polygons aways from its current position. If it is, you risk your agent
// failing to reach its target. So you may still need to do long distance
// planning and provide the crowd manager with intermediate targets.
//
// Other significant limitations:
//
// - All agents using the crowd manager will use the same QueryFilter.
// - Crowd management is relatively expensive. The maximum agents under crowd
//  management at any one time is between 20 and 30. A good place to start is a
//  maximum of 25 agents for 0.5ms per frame.
package crowd

import (
	"unsafe"

	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/gogeo/f32"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

const (
	// The maximum number of neighbors that a crowd agent can take into account for
	// steering decisions.
	CrowdAgentMaxNeighbours = 6

	// The maximum number of corners a crowd agent will look ahead in the path.
	// This value is used for sizing the crowd agent corner buffers.  Due to the
	// behavior of the crowd manager, the actual number of useful corners will be
	// one less than this number.
	CrowdAgentMaxCorners = 4

	// The maximum number of crowd avoidance configurations supported by the crowd
	// manager.
	// see ObstacleAvoidanceParams, Crowd.setObstacleAvoidanceParams(),
	// Crowd.ObstacleAvoidanceParams(), CrowdAgentParams.obstacleAvoidanceType
	CrowdAgentMaxObstavoidanceParams = 8

	// The maximum number of query filter types supported by the crowd manager.
	// see detour.QueryFilter, Crowd.Filter() Crowd.EditableFilter(),
	// CrowdAgentParams.queryFilterType
	CrowdAgentMaxQueryFilterType = 16
)

// Provides neighbor data for agents managed by the crowd.
// see CrowdAgent.neis, Crowd
type CrowdNeighbour struct {
	idx  int     // The index of the neighbor in the crowd.
	dist float32 // The distance between the current agent and the neighbor.
}

// The type of navigation mesh polygon the agent is currently traversing.
type CrowdAgentState int

// TODO: probably should be uint8
const (
	CrowdAgentStateInvalid CrowdAgentState = iota // The agent is not in a valid state.
	CrowdAgentStateWalking                        // The agent is traversing a normal navigation mesh polygon.
	CrowdAgentStatOffMesh                         // The agent is traversing an off-mesh connection.
)

// Configuration parameters for a crowd agent.
// see CrowdAgent, Crowd.addAgent(), Crowd.updateAgentParameters()
type CrowdAgentParams struct {
	radius          float32 // Agent radius. [Limit: >= 0]
	height          float32 // Agent height. [Limit: > 0]
	maxAcceleration float32 // Maximum allowed acceleration. [Limit: >= 0]
	maxSpeed        float32 // Maximum allowed speed. [Limit: >= 0]

	// Defines how close a collision element must be before it is considered for
	// steering behaviors. [Limits: > 0]
	// Collision elements include other agents and navigation mesh boundaries.
	// This value is often based on the agent radius and/or maximum speed.
	// E.g. radius * 8
	collisionQueryRange float32

	// The path visibility optimization range. [Limit: > 0]
	// Only applicable if updateFlags includes the crowdOptimizeVis flag.
	// This value is often based on the agent radius. E.g. radius * 30
	// see PathCorridor.optimizePathVisibility()
	pathOptimizationRange float32

	// How aggresive the agent manager should be at avoiding collisions with
	// this agent. [Limit: >= 0].
	// A higher value will result in agents trying to stay farther away from
	// each other at the cost of more difficult steering in tight spaces.
	separationWeight float32

	// Flags that impact steering behavior. (See: UpdateFlags)
	updateFlags uint8

	// The index of the avoidance configuration to use for the agent.
	//
	// [Limits: 0 <= value <= CrowdMaxObstavoidanceParams]
	// Crowd permits agents to use different avoidance configurations.  This
	// value is the index of the ObstacleAvoidanceParams within the crowd.
	//
	// see ObstacleAvoidanceParams, Crowd.setObstacleAvoidanceParams(),
	// Crowd.ObstacleAvoidanceParams()
	obstacleAvoidanceType uint8

	// The index of the query filter used by this agent.
	queryFilterType uint8

	// User defined data attached to the agent.
	userData []byte
}

const (
	crowdAgentTargetNone uint8 = iota
	crowdAgentTargetFailed
	crowdAgentTargetValid
	crowdAgentTargetRequesting
	crowdAgentTargetWaitingForQueue
	crowdAgentTargetWaitingForPath
	crowdAgentTargetVelocity
)

// Represents an agent managed by a Crowd object.
type CrowdAgent struct {
	// True if the agent is active, false if the agent is in an unused slot in
	// the agent pool.
	active bool

	// The type of mesh polygon the agent is traversing. (See: CrowdAgentState)
	state uint8

	// True if the agent has valid path
	// (targetState == crowdAgentTargetValid) and the path does not lead to
	// the requested position, else false.
	partial bool

	// The path corridor the agent is using.
	corridor PathCorridor

	// The local boundary data for the agent.
	boundary LocalBoundary

	// Time since the agent's path corridor was optimized.
	topologyOptTime float32

	// The known neighbors of the agent.
	neis [CrowdAgentMaxNeighbours]CrowdNeighbour

	// The number of neighbors.
	nneis int

	// The desired speed.
	desiredSpeed float32

	// The current agent position. [(x, y, z)]
	npos d3.Vec3
	// A temporary value used to accumulate agent displacement during iterative
	// collision resolution. [(x, y, z)]
	disp d3.Vec3
	// The desired velocity of the agent. Based on the current path, calculated
	// from scratch each frame. [(x, y, z)]
	dvel d3.Vec3
	// The desired velocity adjusted by obstacle avoidance, calculated from
	// scratch each frame. [(x, y, z)]
	nvel d3.Vec3
	// The actual velocity of the agent. The change from nvel -> vel is
	// constrained by max acceleration. [(x, y, z)]
	vel d3.Vec3

	// The agent's configuration parameters.
	params CrowdAgentParams

	// The local path corridor corners for the agent. (Staight path.)
	// [(x, y, z) * #ncorners]
	cornerVerts [CrowdAgentMaxCorners * 3]float32

	// The local path corridor corner flags. (See: detour.StraightPathFlags)
	// [(flags) * #ncorners]
	cornerFlags [CrowdAgentMaxCorners]uint8

	// The reference id of the polygon being entered at the corner.
	// [(polyRef) * ncorners]
	cornerPolys [CrowdAgentMaxCorners]detour.PolyRef

	// The number of corners.
	ncorners int

	targetState      uint8          // State of the movement request.
	targetRef        detour.PolyRef // Target polyref of the movement request.
	targetPos        d3.Vec3        // Target position of the movement request (or velocity in case of crowdAgentTargetVelocity).
	targetPathqRef   PathQueueRef   // Path finder ref.
	targetReplan     bool           // Flag indicating that the current path is being replanned.
	targetReplanTime float32        // Time since the agent's target was replanned.
}

func NewCrowdAgent() *CrowdAgent {
	return &CrowdAgent{
		npos:      d3.NewVec3(),
		disp:      d3.NewVec3(),
		dvel:      d3.NewVec3(),
		nvel:      d3.NewVec3(),
		vel:       d3.NewVec3(),
		targetPos: d3.NewVec3(),
	}
}

type CrowdAgentAnimation struct {
	active                    bool
	initPos, startPos, endPos d3.Vec3
	polyRef                   detour.PolyRef
	t, tmax                   float32
}

// Crowd agent update flags.
// see CrowdAgentParams.updateFlags
type UpdateFlags int

const (
	CrowdAnticipateTurns   UpdateFlags = 1
	CrowdObstacleAvoidance             = 2
	CrowdSeparation                    = 4
	CrowOptimizeVis                    = 8  // Use PathCorridor.optimizePathVisibility() to optimize the agent path.
	CrowOptimizeTopo                   = 16 // Use PathCorridor.optimizePathTopology() to optimize the agent path.
)

type CrowdAgentDebugInfo struct {
	idx      int
	optStart [3]float32
	optEnd   [3]float32
	vod      *ObstacleAvoidanceDebugData
}

//  Crowd provides local steering behaviors for a group of agents.
//
// This is the core class of the crowd module. See the crowd documentation for a
// summary of the crowd features.
//
// A common method for setting up the crowd is as follows:
//
// - Allocate the crowd
// - Initialize the crowd using init().
// - Set the avoidance configurations using setObstacleAvoidanceParams().
// - Add agents using addAgent() and make an initial movement request using
// requestMoveTarget().
//
// A common process for managing the crowd is as follows:
//
// - Call update() to allow the crowd to manage its agents.
// - Retrieve agent information using getActiveAgents().
// - Make movement requests using requestMoveTarget() when movement goal changes.
// - Repeat every frame.
//
// Some agent configuration settings can be updated using
// updateAgentParameters(). But the crowd owns the agent position. So it is not
// possible to update an active agent's position. If agent position must be fed
// back into the crowd, the agent must be removed and re-added.
//
// Notes:
//
// - Path related information is available for newly added agents only after an
// update() has been performed.
// - Agent objects are kept in a pool and re-used. So it is important when using
// agent objects to check the value of CrowdAgent.active to determine if the
// agent is actually in use or not.
// - This class is meant to provide 'local' movement. There is a limit of 256
// polygons in the path corridor. So it is not meant to provide automatic
// pathfinding services over long distances.
//
// see init(), CrowdAgent
type Crowd struct {
	maxAgents    int
	agents       []CrowdAgent
	activeAgents []*CrowdAgent
	agentAnims   []CrowdAgentAnimation

	pathQ PathQueue

	obstacleQueryParams [CrowdAgentMaxObstavoidanceParams]ObstacleAvoidanceParams
	obstacleQuery       *ObstacleAvoidanceQuery

	grid *ProximityGrid

	pathResult    []detour.PolyRef
	maxPathResult int

	ext d3.Vec3

	filters [CrowdAgentMaxQueryFilterType]detour.QueryFilter

	maxAgentRadius float32

	velocitySampleCount int

	navquery *detour.NavMeshQuery
}

func (c *Crowd) updateTopologyOptimization(agents []*CrowdAgent, nagents int, dt float32) {
	if nagents == 0 {
		return
	}

	const (
		optTimeThr   = 0.5 // seconds
		optMaxAgents = 1
	)

	var (
		queue  [optMaxAgents]*CrowdAgent
		nqueue int = 0
	)

	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			continue
		}
		if ag.params.updateFlags&uint8(CrowOptimizeTopo) == 0 {
			continue
		}
		ag.topologyOptTime += dt
		if ag.topologyOptTime >= optTimeThr {
			nqueue = addToOptQueue(ag, queue[:], nqueue, optMaxAgents)
		}
	}

	for i := 0; i < nqueue; i++ {
		ag := queue[i]
		ag.corridor.OptimizePathTopology(c.navquery, c.filters[ag.params.queryFilterType])
		ag.topologyOptTime = 0
	}
}

func (c *Crowd) updateMoveRequest(dt float32) {
	const pathMaxAgents = 8
	var (
		queue  [pathMaxAgents]*CrowdAgent
		nqueue int = 0
	)

	// Fire off new requests.
	for i := 0; i < c.maxAgents; i++ {
		ag := &c.agents[i]
		if !ag.active {
			continue
		}
		if ag.state == uint8(CrowdAgentStateInvalid) {
			continue
		}
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			continue
		}

		if ag.targetState == crowdAgentTargetRequesting {
			path := ag.corridor.Path()
			npath := ag.corridor.PathCount()
			if npath == 0 {
				panic("npath should be != 0")
			}

			const maxRes = 32
			reqPos := d3.NewVec3()
			var reqPath [maxRes]detour.PolyRef // The path to the request location
			var reqPathCount int = 0

			// Quick search towards the goal.
			const maxIter = 20
			c.navquery.InitSlicedFindPath(path[0], ag.targetRef, ag.npos, ag.targetPos, c.filters[ag.params.queryFilterType], 0)
			c.navquery.UpdateSlicedFindPath(maxIter, nil)
			var status detour.Status = 0
			if ag.targetReplan { // && npath > 10)
				// Try to use existing steady path during replan if possible.
				reqPathCount, status = c.navquery.FinalizeSlicedFindPathPartial(path, npath, reqPath[:], maxRes)
			} else {
				// Try to move towards target when goal changes.
				reqPathCount, status = c.navquery.FinalizeSlicedFindPath(reqPath[:], maxRes)
			}

			if !detour.StatusFailed(status) && reqPathCount > 0 {
				// In progress or succeed.
				if reqPath[reqPathCount-1] != ag.targetRef {
					// Partial path, constrain target position inside the last polygon.
					status = c.navquery.ClosestPointOnPoly(reqPath[reqPathCount-1], ag.targetPos, reqPos, nil)
					if detour.StatusFailed(status) {
						reqPathCount = 0
					}
				} else {
					d3.Vec3Copy(reqPos, ag.targetPos)
				}
			} else {
				reqPathCount = 0
			}

			if reqPathCount == 0 {
				// Could not find path, start the request from current location.
				d3.Vec3Copy(reqPos, ag.npos)
				reqPath[0] = path[0]
				reqPathCount = 1
			}

			ag.corridor.SetCorridor(reqPos, reqPath[:], reqPathCount)
			ag.boundary.reset()
			ag.partial = false

			if reqPath[reqPathCount-1] == ag.targetRef {
				ag.targetState = uint8(crowdAgentTargetValid)
				ag.targetReplanTime = 0.0
			} else {
				// The path is longer or potentially unreachable, full plan.
				ag.targetState = uint8(crowdAgentTargetWaitingForQueue)
			}
		}

		if ag.targetState == uint8(crowdAgentTargetWaitingForQueue) {
			nqueue = addToPathQueue(ag, queue[:], nqueue, pathMaxAgents)
		}
	}

	for i := 0; i < nqueue; i++ {
		ag := queue[i]
		ag.targetPathqRef = c.pathQ.Request(ag.corridor.LastPoly(), ag.targetRef,
			ag.corridor.Target(), ag.targetPos, c.filters[ag.params.queryFilterType])
		if ag.targetPathqRef != PathQInvalid {
			ag.targetState = uint8(crowdAgentTargetWaitingForPath)
		}
	}

	// Update requests.
	c.pathQ.Update(maxItersPerUpdate)

	var status detour.Status

	// Process path results.
	for i := 0; i < c.maxAgents; i++ {
		ag := &c.agents[i]
		if !ag.active {
			continue
		}
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			continue
		}

		if ag.targetState == uint8(crowdAgentTargetWaitingForPath) {
			// Poll path queue.
			status = c.pathQ.GetRequestStatus(ag.targetPathqRef)
			if detour.StatusFailed(status) {
				// Path find failed, retry if the target location is still valid.
				ag.targetPathqRef = PathQInvalid
				if ag.targetRef != 0 {
					ag.targetState = uint8(crowdAgentTargetRequesting)
				} else {
					ag.targetState = uint8(crowdAgentTargetFailed)
				}
				ag.targetReplanTime = 0.0
			} else if detour.StatusSucceed(status) {
				path := ag.corridor.Path()
				npath := ag.corridor.PathCount()
				if npath == 0 {
					panic("npath should be != 0")
				}

				// Apply results.
				targetPos := d3.NewVec3From(ag.targetPos)
				res := c.pathResult[:]
				valid := true
				var nres int = 0
				status = c.pathQ.GetPathResult(ag.targetPathqRef, res, &nres, c.maxPathResult)
				if detour.StatusFailed(status) || nres == 0 {
					valid = false
				}

				if detour.StatusDetail(status, detour.PartialResult) {
					ag.partial = true
				} else {
					ag.partial = false
				}

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.

				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if valid && path[npath-1] != res[0] {
					valid = false
				}

				if valid {
					// Put the old path infront of the old path.
					if npath > 1 {
						// Make space for the old path.
						if (npath-1)+nres > c.maxPathResult {
							nres = c.maxPathResult - (npath - 1)
						}

						copy(res[npath-1:], res)
						// Copy old path in the beginning.
						copy(res, path[npath-1:])
						nres += npath - 1

						// Remove trackbacks
						for j := 0; j < nres; j++ {
							if j-1 >= 0 && j+1 < nres {
								if res[j-1] == res[j+1] {
									copy(res[(j-1):], res[j+1:nres])
									nres -= 2
									j -= 2
								}
							}
						}
					}

					// Check for partial path.
					if res[nres-1] != ag.targetRef {
						// Partial path, constrain target position inside the last polygon.
						nearest := d3.NewVec3()
						status = c.navquery.ClosestPointOnPoly(res[nres-1], targetPos, nearest, nil)
						if detour.StatusSucceed(status) {
							d3.Vec3Copy(targetPos, nearest)
						} else {
							valid = false
						}
					}
				}

				if valid {
					// Set current corridor.
					ag.corridor.SetCorridor(targetPos, res, nres)
					// Force to update boundary.
					ag.boundary.reset()
					ag.targetState = uint8(crowdAgentTargetValid)
				} else {
					// Something went wrong.
					ag.targetState = uint8(crowdAgentTargetFailed)
				}

				ag.targetReplanTime = 0.0
			}
		}
	}
}

func (c *Crowd) checkPathValidity(agents []*CrowdAgent, nagents int, dt float32) {
	const (
		checkLookAhead    = 10
		targetReplanDelay = 1.0 // seconds
	)

	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}

		ag.targetReplanTime += dt

		replan := false

		// First check that the current location is valid.
		idx := c.AgentIndex(ag)
		agentPos := d3.NewVec3From(ag.npos)
		agentRef := ag.corridor.FirstPoly()
		if !c.navquery.IsValidPolyRef(agentRef, c.filters[ag.params.queryFilterType]) {
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			nearest := d3.NewVec3From(agentPos)
			agentRef = 0
			_, agentRef, nearest = c.navquery.FindNearestPoly(ag.npos, c.ext, c.filters[ag.params.queryFilterType])
			d3.Vec3Copy(agentPos, nearest)

			if agentRef != 0 {
				// Could not find location in navmesh, set state to invalid.
				ag.corridor.Reset(0, agentPos)
				ag.partial = false
				ag.boundary.reset()
				ag.state = uint8(CrowdAgentStateInvalid)
				continue
			}

			// Make sure the first polygon is valid, but leave other valid
			// polygons in the path so that replanner can adjust the path better.
			ag.corridor.FixPathStart(agentRef, agentPos)
			//			ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			ag.boundary.reset()
			d3.Vec3Copy(ag.npos, agentPos)

			replan = true
		}

		// If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			continue
		}

		// Try to recover move request position.
		if ag.targetState != crowdAgentTargetNone && ag.targetState != crowdAgentTargetFailed {
			if !c.navquery.IsValidPolyRef(ag.targetRef, c.filters[ag.params.queryFilterType]) {
				// Current target is not valid, try to reposition.
				nearest := d3.NewVec3From(ag.targetPos)
				ag.targetRef = 0
				_, ag.targetRef, nearest = c.navquery.FindNearestPoly(ag.targetPos, c.ext, c.filters[ag.params.queryFilterType])
				d3.Vec3Copy(ag.targetPos, nearest)
				replan = true
			}
			if ag.targetRef == 0 {
				// Failed to reposition target, fail moverequest.
				ag.corridor.Reset(agentRef, agentPos)
				ag.partial = false
				ag.targetState = crowdAgentTargetNone
			}
		}

		// If nearby corridor is not valid, replan.
		if !ag.corridor.IsValid(checkLookAhead, c.navquery, c.filters[ag.params.queryFilterType]) {
			// Fix current path.
			//			ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			//			ag.boundary.reset();
			replan = true
		}

		// If the end of the path is near and it is not the requested location, replan.
		if ag.targetState == uint8(crowdAgentTargetValid) {
			if ag.targetReplanTime > targetReplanDelay &&
				ag.corridor.PathCount() < checkLookAhead &&
				ag.corridor.LastPoly() != ag.targetRef {
				replan = true
			}
		}

		// Try to replan path to goal.
		if replan {
			if ag.targetState != crowdAgentTargetNone {
				c.requestMoveTargetReplan(idx, ag.targetRef, ag.targetPos)
			}
		}
	}
}

func (c *Crowd) AgentIndex(agent *CrowdAgent) int {
	// TODO: use unsafe here
	it := (uintptr(unsafe.Pointer(agent)) - uintptr(unsafe.Pointer(&c.agents[0]))) / unsafe.Sizeof(*agent)
	return int(it)
}

func (c *Crowd) requestMoveTargetReplan(idx int, ref detour.PolyRef, pos d3.Vec3) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}

	ag := &c.agents[idx]

	// Initialize request.
	ag.targetRef = ref
	d3.Vec3Copy(ag.targetPos, pos)
	ag.targetPathqRef = PathQInvalid
	ag.targetReplan = true
	if ag.targetRef != 0 {
		ag.targetState = uint8(crowdAgentTargetRequesting)
	} else {
		ag.targetState = uint8(crowdAgentTargetFailed)
	}

	return true
}

func (c *Crowd) purge() {
	c.agents = nil
	c.maxAgents = 0
	c.activeAgents = nil
	c.agentAnims = nil
	c.pathResult = nil
	c.grid = nil
	c.obstacleQuery = nil
	c.navquery = nil
}

// Init initializes the crowd.
//
//  Arguments:
//   maxAgents       The maximum number of agents the crowd can manage.
//                   [Limit: >= 1]
//   maxAgentRadius  The maximum radius of any agent that will be added to the
//                   crowd. [Limit: > 0]
//   nav             The navigation mesh to use for planning.
//
// Return true if the initialization succeeded.
// May be called more than once to purge and re-initialize the crowd.
func (c *Crowd) Init(maxAgents int, maxAgentRadius float32, nav *detour.NavMesh) bool {
	c.purge()
	c.ext = d3.NewVec3()

	c.maxAgents = maxAgents
	c.maxAgentRadius = maxAgentRadius

	c.ext.SetXYZ(c.maxAgentRadius*2.0, c.maxAgentRadius*1.5, c.maxAgentRadius*2.0)

	c.grid = NewProximityGrid(c.maxAgents*4, maxAgentRadius*3)
	c.obstacleQuery = NewObstacleAvoidanceQuery(6, 8)

	// Init obstacle query params.
	c.obstacleQueryParams = [CrowdAgentMaxObstavoidanceParams]ObstacleAvoidanceParams{}
	for i := 0; i < CrowdAgentMaxObstavoidanceParams; i++ {
		params := &c.obstacleQueryParams[i]
		params.velBias = 0.4
		params.weightDesVel = 2.0
		params.weightCurVel = 0.75
		params.weightSide = 0.75
		params.weightToi = 2.5
		params.horizTime = 2.5
		params.gridSize = 33
		params.adaptiveDivs = 7
		params.adaptiveRings = 2
		params.adaptiveDepth = 5
	}

	// Allocate temp buffer for merging paths.
	c.maxPathResult = 256
	c.pathResult = make([]detour.PolyRef, c.maxPathResult)

	if !c.pathQ.Init(c.maxPathResult, maxPathQueueNodes, nav) {
		return false
	}

	c.agents = make([]CrowdAgent, c.maxAgents)
	c.activeAgents = make([]*CrowdAgent, c.maxAgents)
	c.agentAnims = make([]CrowdAgentAnimation, c.maxAgents)

	for i := 0; i < c.maxAgents; i++ {
		// TODO: to implement (we new NewCrowdAgent because it contains some
		// 3D vectors that need to be allocated
		c.agents[i] = *NewCrowdAgent()
		c.agents[i].active = false
		if !c.agents[i].corridor.init(c.maxPathResult) {
			return false
		}
	}

	for i := 0; i < c.maxAgents; i++ {
		c.agentAnims[i].active = false
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	var st detour.Status
	st, c.navquery = detour.NewNavMeshQuery(nav, maxCommonNodes)
	if c.navquery == nil {
		return false
	}
	if detour.StatusFailed(st) {
		return false
	}

	return true
}

// Sets the shared avoidance configuration for the specified index.
//
//  Arguments:
//   idx      The index. [Limits: 0 <= value < CrowdMaxObstavoidanceParams]
//   params   The new configuration.
func (c *Crowd) SetObstacleAvoidanceParams(idx int, params *ObstacleAvoidanceParams) {
	if idx >= 0 && idx < CrowdAgentMaxObstavoidanceParams {
		c.obstacleQueryParams[idx] = *params
	}
}

// Gets the shared avoidance configuration for the specified index.
//
//  Arguments:
//   idx      The index of the configuration to retreive.
//            [Limits:  0 <= value < CrowdMaxObstavoidanceParams]
//
// Return The requested configuration.
func (c *Crowd) ObstacleAvoidanceParams(idx int) *ObstacleAvoidanceParams {
	if idx >= 0 && idx < CrowdAgentMaxObstavoidanceParams {
		return &c.obstacleQueryParams[idx]
	}
	return nil
}

// Gets the specified agent from the pool.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//
// Return The requested agent.
// Agents in the pool may not be in use. Check CrowdAgent.active before
// using the returned object.
func (c *Crowd) Agent(idx int) *CrowdAgent {
	if idx < 0 || idx >= c.maxAgents {
		return nil
	}
	return &c.agents[idx]
}

// Gets the specified agent from the pool.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//
// Return The requested agent.
// Agents in the pool may not be in use. Check CrowdAgent.active before using
// the returned object.
func (c *Crowd) EditableAgent(idx int) *CrowdAgent {
	if idx < 0 || idx >= c.maxAgents {
		return nil
	}
	return &c.agents[idx]
}

// The maximum number of agents that can be managed by the object.
//
// Return The maximum number of agents.
func (c *Crowd) AgentCount() int {
	return c.maxAgents
}

// Adds a new agent to the crowd.
//
//  Arguments:
//   pos      The requested position of the agent. [(x, y, z)]
//   params   The configutation of the agent.
//
// Return The index of the agent in the agent pool. Or -1 if the agent could not
// be added.
//
// The agent's position will be constrained to the surface of the navigation
// mesh.
func (c *Crowd) AddAgent(pos d3.Vec3, params *CrowdAgentParams) int {
	// Find empty slot.
	var idx int = -1
	for i := 0; i < c.maxAgents; i++ {
		if !c.agents[i].active {
			idx = i
			break
		}
	}
	if idx == -1 {
		return -1
	}

	ag := &c.agents[idx]

	c.UpdateAgentParameters(idx, params)

	// Find nearest position on navmesh and place the agent there.
	status, ref, nearest := c.navquery.FindNearestPoly(pos, c.ext, c.filters[ag.params.queryFilterType])
	if detour.StatusFailed(status) {
		d3.Vec3Copy(nearest, pos)
		ref = 0
	}

	ag.corridor.Reset(ref, nearest)
	ag.boundary.reset()
	ag.partial = false

	ag.topologyOptTime = 0
	ag.targetReplanTime = 0
	ag.nneis = 0

	ag.dvel.SetXYZ(0, 0, 0)
	ag.nvel.SetXYZ(0, 0, 0)
	ag.vel.SetXYZ(0, 0, 0)
	d3.Vec3Copy(ag.npos, nearest)

	ag.desiredSpeed = 0

	if ref != 0 {
		ag.state = uint8(CrowdAgentStateWalking)
	} else {
		ag.state = uint8(CrowdAgentStateInvalid)
	}

	ag.targetState = crowdAgentTargetNone

	ag.active = true

	return idx
}

// Updates the specified agent's configuration.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//   params   The new agent configuration.
func (c *Crowd) UpdateAgentParameters(idx int, params *CrowdAgentParams) {
	if idx < 0 || idx >= c.maxAgents {
		return
	}
	c.agents[idx].params = *params
}

// Removes the agent from the crowd.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//
// The agent is deactivated and will no longer be processed. Its CrowdAgent
// object is not removed from the pool. It is marked as inactive so that it is
// available for reuse.
func (c *Crowd) RemoveAgent(idx int) {
	if idx >= 0 && idx < c.maxAgents {
		c.agents[idx].active = false
	}
}

// Submits a new move request for the specified agent.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//   ref      The position's polygon reference.
//   pos      The position within the polygon. [(x, y, z)]
//
// Return true if the request was successfully submitted.
// This method is used when a new target is set.
//
// The position will be constrained to the surface of the navigation mesh.
//
// The request will be processed during the next update().
func (c *Crowd) RequestMoveTarget(idx int, ref detour.PolyRef, pos d3.Vec3) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}
	if ref == 0 {
		return false
	}

	ag := &c.agents[idx]

	// Initialize request.
	ag.targetRef = ref
	d3.Vec3Copy(ag.targetPos, pos)
	ag.targetPathqRef = PathQInvalid
	ag.targetReplan = false
	if ag.targetRef != 0 {
		ag.targetState = uint8(crowdAgentTargetRequesting)
	} else {
		ag.targetState = uint8(crowdAgentTargetFailed)
	}

	return true
}

// Submits a new move request for the specified agent.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//   vel      The movement velocity. [(x, y, z)]
//
// Return true if the request was successfully submitted.
func (c *Crowd) RequestMoveVelocity(idx int, vel d3.Vec3) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}

	ag := &c.agents[idx]

	// Initialize request.
	ag.targetRef = 0
	d3.Vec3Copy(ag.targetPos, vel)
	ag.targetPathqRef = PathQInvalid
	ag.targetReplan = false
	ag.targetState = uint8(crowdAgentTargetVelocity)

	return true
}

// Resets any request for the specified agent.
//
//  Arguments:
//   idx      The agent index. [Limits: 0 <= value < AgentCount()]
//
// Return true if the request was successfully reseted.
func (c *Crowd) ResetMoveTarget(idx int) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}

	ag := &c.agents[idx]

	// Initialize request.
	ag.targetRef = 0
	ag.targetPos.SetXYZ(0, 0, 0)
	ag.dvel.SetXYZ(0, 0, 0)
	ag.targetPathqRef = PathQInvalid
	ag.targetReplan = false
	ag.targetState = crowdAgentTargetNone

	return true
}

// Gets the active agents int the agent pool.
//
//  Arguments:
//   agents    An array of agent pointers. [(CrowdAgent *) * maxAgents]
//   maxAgents The size of the crowd agent array.
//
// Return the number of agents returned in agents.
func (c *Crowd) ActiveAgents(agents []*CrowdAgent, maxAgents int) int {
	var n int
	for i := 0; i < c.maxAgents; i++ {
		if !c.agents[i].active {
			continue
		}
		if n < maxAgents {
			agents[n] = &c.agents[i]
			n++
		}
	}
	return n
}

// Updates the steering and positions of all agents.
//
//  Arguments:
//   dt       The time, in seconds, to Update the simulation. [Limit: > 0]
//   debug    A debug object to load with debug information. [Opt]
func (c *Crowd) Update(dt float32, debug *CrowdAgentDebugInfo) {
	c.velocitySampleCount = 0

	var debugIdx int = -1
	if debug != nil {
		debugIdx = debug.idx
	}

	agents := c.activeAgents
	nagents := c.ActiveAgents(agents, c.maxAgents)

	// Check that all agents still have valid paths.
	c.checkPathValidity(agents, nagents, dt)

	// Update async move request and path finder.
	c.updateMoveRequest(dt)

	// Optimize path topology.
	c.updateTopologyOptimization(agents, nagents, dt)

	// Register agents to proximity grid.
	c.grid.Clear()
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		p := ag.npos
		r := ag.params.radius
		c.grid.AddItem(uint16(i), p[0]-r, p[2]-r, p[0]+r, p[2]+r)
	}

	// Get nearby navmesh segments and agents to collide with.
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		updateThr := ag.params.collisionQueryRange * 0.25
		if d3.Vec3Dist2DSqr(ag.npos, ag.boundary.Center()) > math32.Sqr(updateThr) ||
			!ag.boundary.isValid(c.navquery, c.filters[ag.params.queryFilterType]) {
			ag.boundary.update(ag.corridor.FirstPoly(), ag.npos, ag.params.collisionQueryRange,
				c.navquery, c.filters[ag.params.queryFilterType])
		}
		// Query neighbour agents
		ag.nneis = getNeighbours(ag.npos, ag.params.height, ag.params.collisionQueryRange,
			ag, ag.neis[:], CrowdAgentMaxNeighbours,
			agents, nagents, c.grid)
		for j := 0; j < ag.nneis; j++ {
			ag.neis[j].idx = c.AgentIndex(agents[ag.neis[j].idx])
		}
	}

	// Find next corner to steer to.
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			continue
		}

		// Find corners for steering
		ag.ncorners = ag.corridor.FindCorners(ag.cornerVerts[:], ag.cornerFlags[:], ag.cornerPolys[:],
			CrowdAgentMaxCorners, c.navquery, c.filters[ag.params.queryFilterType])

		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		if (ag.params.updateFlags&uint8(CrowOptimizeVis)) != 0 && ag.ncorners > 0 {
			target := ag.cornerVerts[intMin(1, ag.ncorners-1)*3:]
			ag.corridor.OptimizePathVisibility(target, ag.params.pathOptimizationRange, c.navquery, c.filters[ag.params.queryFilterType])

			// Copy data for debug purposes.
			if debugIdx == i {
				d3.Vec3Copy(debug.optStart[:], ag.corridor.Pos())
				d3.Vec3Copy(debug.optEnd[:], target)
			}
		} else {
			// Copy data for debug purposes.
			if debugIdx == i {
				d3.Vec3(debug.optStart[:]).SetXYZ(0, 0, 0)
				d3.Vec3(debug.optEnd[:]).SetXYZ(0, 0, 0)
			}
		}
	}

	// Trigger off-mesh connections (depends on corners).
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			continue
		}

		// Check
		triggerRadius := ag.params.radius * 2.25
		if overOffmeshConnection(ag, triggerRadius) {
			// Prepare to off-mesh connection.
			idx := (uintptr(unsafe.Pointer(ag)) - uintptr(unsafe.Pointer(&c.agents[0]))) / unsafe.Sizeof(*ag)

			anim := &c.agentAnims[idx]

			// Adjust the path over the off-mesh connection.
			var refs [2]detour.PolyRef
			if ag.corridor.MoveOverOffmeshConnection(ag.cornerPolys[ag.ncorners-1], refs[:],
				anim.startPos, anim.endPos, c.navquery) {
				d3.Vec3Copy(anim.initPos, ag.npos)
				anim.polyRef = refs[1]
				anim.active = true
				anim.t = 0.0
				anim.tmax = (anim.startPos.Dist2D(anim.endPos) / ag.params.maxSpeed) * 0.5

				ag.state = uint8(CrowdAgentStatOffMesh)
				ag.ncorners = 0
				ag.nneis = 0
				continue
			} else {
				// Path validity check will ensure that bad/blocked connections will be replanned.
			}
		}
	}

	// Calculate steering.
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}
		if ag.targetState == crowdAgentTargetNone {
			continue
		}

		dvel := d3.NewVec3()

		if ag.targetState == crowdAgentTargetVelocity {
			d3.Vec3Copy(dvel, ag.targetPos)
			ag.desiredSpeed = ag.targetPos.Len()
		} else {
			// Calculate steering direction.
			if (ag.params.updateFlags & uint8(CrowdAnticipateTurns)) != 0 {
				calcSmoothSteerDirection(ag, dvel)
			} else {
				calcStraightSteerDirection(ag, dvel)
			}

			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			slowDownRadius := ag.params.radius * 2 // TODO: make less hacky.
			speedScale := getDistanceToGoal(ag, slowDownRadius) / slowDownRadius

			ag.desiredSpeed = ag.params.maxSpeed
			dvel.Scale(ag.desiredSpeed * speedScale)
		}

		// Separation
		if (ag.params.updateFlags & uint8(CrowdSeparation)) != 0 {
			separationDist := ag.params.collisionQueryRange
			invSeparationDist := 1.0 / separationDist
			separationWeight := ag.params.separationWeight

			var w float32
			disp := d3.NewVec3()

			for j := 0; j < ag.nneis; j++ {
				nei := &c.agents[ag.neis[j].idx]

				diff := d3.NewVec3()
				d3.Vec3Sub(diff, ag.npos, nei.npos)
				diff[1] = 0

				distSqr := diff.LenSqr()
				if distSqr < 0.00001 {
					continue
				}
				if distSqr > math32.Sqr(separationDist) {
					continue
				}
				dist := math32.Sqrt(distSqr)
				weight := separationWeight * (1.0 - math32.Sqr(dist*invSeparationDist))

				d3.Vec3Mad(disp, disp, diff, weight/dist)
				w += 1.0
			}

			if w > 0.0001 {
				// Adjust desired velocity.
				d3.Vec3Mad(dvel, dvel, disp, 1.0/w)
				// Clamp desired velocity to desired speed.
				speedSqr := dvel.LenSqr()
				desiredSqr := math32.Sqr(ag.desiredSpeed)
				if speedSqr > desiredSqr {
					dvel.Scale(desiredSqr / speedSqr)
				}
			}
		}

		// Set the desired velocity.
		d3.Vec3Copy(ag.dvel, dvel)
	}

	// Velocity planning.
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}

		if (ag.params.updateFlags & uint8(CrowdObstacleAvoidance)) != 0 {
			c.obstacleQuery.reset()

			// Add neighbours as obstacles.
			for j := 0; j < ag.nneis; j++ {
				nei := &c.agents[ag.neis[j].idx]
				c.obstacleQuery.addCircle(nei.npos, nei.params.radius, nei.vel, nei.dvel)
			}

			// Append neighbour segments as obstacles.
			for j := 0; j < ag.boundary.SegmentCount(); j++ {
				s := ag.boundary.Segment(j)
				if detour.TriArea2D(ag.npos, s, s[3:]) < 0.0 {
					continue
				}
				c.obstacleQuery.addSegment(s, s[3:])
			}

			var vod *ObstacleAvoidanceDebugData
			if debugIdx == i {
				vod = debug.vod
			}

			// Sample new safe velocity.
			var (
				adaptive     = true
				ns       int = 0
			)

			params := &c.obstacleQueryParams[ag.params.obstacleAvoidanceType]

			if adaptive {
				ns = c.obstacleQuery.sampleVelocityAdaptive(ag.npos, ag.params.radius, ag.desiredSpeed,
					ag.vel, ag.dvel, ag.nvel, params, vod)
			} else {
				ns = c.obstacleQuery.sampleVelocityGrid(ag.npos, ag.params.radius, ag.desiredSpeed,
					ag.vel, ag.dvel, ag.nvel, params, vod)
			}
			c.velocitySampleCount += ns
		} else {
			// If not using velocity planning, new velocity is directly the desired velocity.
			d3.Vec3Copy(ag.nvel, ag.dvel)
		}
	}

	// Integrate.
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}
		integrate(ag, dt)
	}

	// Handle collisions.
	const collisionResolveFactor = 0.7

	for iter := 0; iter < 4; iter++ {
		for i := 0; i < nagents; i++ {
			ag := agents[i]
			idx0 := c.AgentIndex(ag)

			if ag.state != uint8(CrowdAgentStateWalking) {
				continue
			}

			ag.disp.SetXYZ(0, 0, 0)

			var w float32

			for j := 0; j < ag.nneis; j++ {
				nei := &c.agents[ag.neis[j].idx]
				idx1 := c.AgentIndex(nei)

				diff := ag.npos.Sub(nei.npos)
				diff[1] = 0

				dist := diff.LenSqr()
				if dist > math32.Sqr(ag.params.radius+nei.params.radius) {
					continue
				}
				dist = math32.Sqrt(dist)
				pen := (ag.params.radius + nei.params.radius) - dist
				if dist < 0.0001 {
					// Agents on top of each other, try to choose diverging separation directions.
					if idx0 > idx1 {
						diff.SetXYZ(-ag.dvel[2], 0, ag.dvel[0])
					} else {
						diff.SetXYZ(ag.dvel[2], 0, -ag.dvel[0])
					}
					pen = 0.01
				} else {
					pen = (1.0 / dist) * (pen * 0.5) * collisionResolveFactor
				}

				d3.Vec3Mad(ag.disp, ag.disp, diff, pen)

				w += 1.0
			}

			if w > 0.0001 {
				iw := 1.0 / w
				ag.disp.Scale(iw)
			}
		}

		for i := 0; i < nagents; i++ {
			ag := agents[i]
			if ag.state != uint8(CrowdAgentStateWalking) {
				continue
			}

			d3.Vec3Add(ag.npos, ag.npos, ag.disp)
		}
	}

	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != uint8(CrowdAgentStateWalking) {
			continue
		}

		// Move along navmesh.
		ag.corridor.MovePosition(ag.npos, c.navquery, c.filters[ag.params.queryFilterType])
		// Get valid constrained position back.
		d3.Vec3Copy(ag.npos, ag.corridor.Pos())

		// If not using path, truncate the corridor to just one poly.
		if ag.targetState == crowdAgentTargetNone || ag.targetState == crowdAgentTargetVelocity {
			ag.corridor.Reset(ag.corridor.FirstPoly(), ag.npos)
			ag.partial = false
		}
	}

	// Update agents using off-mesh connection.
	for i := 0; i < c.maxAgents; i++ {
		anim := &c.agentAnims[i]
		if !anim.active {
			continue
		}
		ag := agents[i]

		anim.t += dt
		if anim.t > anim.tmax {
			// Reset animation
			anim.active = false
			// Prepare agent for walking.
			ag.state = uint8(CrowdAgentStateWalking)
			continue
		}

		// Update position
		ta := anim.tmax * 0.15
		tb := anim.tmax
		if anim.t < ta {
			u := tween(anim.t, 0.0, ta)
			d3.Vec3Lerp(ag.npos, anim.initPos, anim.startPos, u)
		} else {
			u := tween(anim.t, ta, tb)
			d3.Vec3Lerp(ag.npos, anim.startPos, anim.endPos, u)
		}

		// Update velocity.
		ag.vel.SetXYZ(0, 0, 0)
		ag.dvel.SetXYZ(0, 0, 0)
	}
}

// Gets the filter used by the crowd.
// Return the filter used by the crowd.
func (c *Crowd) Filter(i int) detour.QueryFilter {
	if i >= 0 && i < CrowdAgentMaxQueryFilterType {
		return c.filters[i]
	}
	return nil
}

// Gets the filter used by the crowd.
// Return the filter used by the crowd.
func (c *Crowd) EditableFilter(i int) detour.QueryFilter {
	if i >= 0 && i < CrowdAgentMaxQueryFilterType {
		return c.filters[i]
	}
	return nil
}

// Gets the search extents [(x, y, z)] used by the crowd for query operations.
// Return the search extents used by the crowd. [(x, y, z)]
func (c *Crowd) QueryExtents() d3.Vec3 {
	return c.ext
}

// Gets the velocity sample count.
// Return the velocity sample count.
func (c *Crowd) VelocitySampleCount() int {
	return c.velocitySampleCount
}

// Gets the crowd's proximity grid.
// Return the crowd's proximity grid.
func (c *Crowd) Grid() *ProximityGrid {
	return c.grid
}

// Gets the crowd's path request queue.
// Return the crowd's path request queue.
func (c *Crowd) PathQueue() *PathQueue {
	return &c.pathQ
}

// Gets the query object used by the crowd.
func (c *Crowd) NavMeshQuery() *detour.NavMeshQuery {
	return c.navquery
}

const (
	maxItersPerUpdate = 100
	maxPathQueueNodes = 4096
	maxCommonNodes    = 512
)

func tween(t, t0, t1 float32) float32 {
	return f32.Clamp((t-t0)/(t1-t0), 0.0, 1.0)
}

func integrate(ag *CrowdAgent, dt float32) {
	// Fake dynamic constraint.
	maxDelta := ag.params.maxAcceleration * dt
	dv := ag.nvel.Sub(ag.vel)
	ds := dv.Len()
	if ds > maxDelta {
		dv = dv.Scale(maxDelta / ds)
	}
	d3.Vec3Add(ag.vel, ag.vel, dv)

	// Integrate
	if ag.vel.Len() > 0.0001 {
		d3.Vec3Mad(ag.npos, ag.npos, ag.vel, dt)
	} else {
		ag.vel.SetXYZ(0, 0, 0)
	}
}

func overOffmeshConnection(ag *CrowdAgent, radius float32) bool {
	if ag.ncorners == 0 {
		return false
	}

	var offMeshConnection bool
	if (ag.cornerFlags[ag.ncorners-1] & detour.StraightPathOffMeshConnection) != 0 {
		offMeshConnection = true
	}
	if offMeshConnection {
		distSq := d3.Vec3Dist2DSqr(ag.npos, ag.cornerVerts[(ag.ncorners-1)*3:])
		if distSq < radius*radius {
			return false
		}
	}

	return false
}

func getDistanceToGoal(ag *CrowdAgent, rang float32) float32 {
	if ag.ncorners == 0 {
		return rang
	}

	var endOfPath bool
	if (ag.cornerFlags[ag.ncorners-1] & detour.StraightPathEnd) != 0 {
		endOfPath = true
	}
	if endOfPath {
		return math32.Min(ag.npos.Dist2D(ag.cornerVerts[(ag.ncorners-1)*3:]), rang)
	}

	return rang
}

func calcSmoothSteerDirection(ag *CrowdAgent, dir d3.Vec3) {
	if ag.ncorners == 0 {
		dir.SetXYZ(0, 0, 0)
		return
	}

	ip0 := int32(0)
	ip1 := math32.MinInt32(1, int32(ag.ncorners-1))
	p0 := ag.cornerVerts[ip0*3:]
	p1 := ag.cornerVerts[ip1*3:]

	var dir0, dir1 = d3.NewVec3(), d3.NewVec3()
	d3.Vec3Sub(dir0, p0, ag.npos)
	d3.Vec3Sub(dir1, p1, ag.npos)
	dir0[1] = 0
	dir1[1] = 0

	len0 := dir0.Len()
	len1 := dir1.Len()
	if len1 > 0.001 {
		d3.Vec3Scale(dir1, dir1, 1.0/len1)
	}

	dir[0] = dir0[0] - dir1[0]*len0*0.5
	dir[1] = 0
	dir[2] = dir0[2] - dir1[2]*len0*0.5

	dir.Normalize()
}

func calcStraightSteerDirection(ag *CrowdAgent, dir d3.Vec3) {
	if ag.ncorners == 0 {
		dir.SetXYZ(0, 0, 0)
		return
	}
	d3.Vec3Sub(dir, ag.cornerVerts[:], ag.npos)
	dir[1] = 0
	dir.Normalize()
}

func addNeighbour(idx int, dist float32, neis []CrowdNeighbour, nneis, maxNeis int) int {
	// Insert neighbour based on the distance.
	var nei *CrowdNeighbour
	if nneis == 0 {
		nei = &neis[nneis]
	} else if dist >= neis[nneis-1].dist {
		if nneis >= maxNeis {
			return nneis
		}
		nei = &neis[nneis]
	} else {
		var i int
		for i = 0; i < nneis; i++ {
			if dist <= neis[i].dist {
				break
			}
		}

		tgt := i + 1
		n := intMin(nneis-i, maxNeis-tgt)

		if tgt+n > maxNeis {
			panic("tgt+n should be <= maxNeis")
		}

		if n > 0 {
			copy(neis[tgt:], neis[i:i+n])
		}
		nei = &neis[i]
	}

	nei = &CrowdNeighbour{}

	nei.idx = idx
	nei.dist = dist

	return intMin(nneis+1, maxNeis)
}

func getNeighbours(pos d3.Vec3, height, rang float32,
	skip *CrowdAgent, result []CrowdNeighbour, maxResult int,
	agents []*CrowdAgent, nagents int, grid *ProximityGrid) int {
	var n int

	const MAX_NEIS = 32
	var ids [MAX_NEIS]uint16
	nids := grid.QueryItems(pos[0]-rang, pos[2]-rang,
		pos[0]+rang, pos[2]+rang,
		ids[:], MAX_NEIS)

	for i := 0; i < nids; i++ {
		ag := agents[ids[i]]

		if ag == skip {
			continue
		}

		// Check for overlap.
		diff := pos.Sub(ag.npos)
		if math32.Abs(diff[1]) >= (height+ag.params.height)/2.0 {
			continue
		}
		diff[1] = 0
		distSqr := diff.LenSqr()
		if distSqr > math32.Sqr(rang) {
			continue
		}

		n = addNeighbour(int(ids[i]), distSqr, result, n, maxResult)
	}
	return n
}

func addToOptQueue(newag *CrowdAgent, agents []*CrowdAgent, nagents, maxAgents int) int {
	// Insert neighbour based on greatest time.
	var slot int = 0
	if nagents == 0 {
		slot = nagents
	} else if newag.topologyOptTime <= agents[nagents-1].topologyOptTime {
		if nagents >= maxAgents {
			return nagents
		}
		slot = nagents
	} else {
		var i int
		for i = 0; i < nagents; i++ {
			if newag.topologyOptTime >= agents[i].topologyOptTime {
				break
			}
		}

		tgt := i + 1
		n := intMin(nagents-i, maxAgents-tgt)

		if tgt+n > maxAgents {
			panic("tgt+n should be <= maxAgents")
		}

		if n > 0 {
			copy(agents[tgt:], agents[i:i+n])
		}
		slot = i
	}

	agents[slot] = newag

	return intMin(nagents+1, maxAgents)
}

func addToPathQueue(newag *CrowdAgent, agents []*CrowdAgent, nagents, maxAgents int) int {
	// Insert neighbour based on greatest time.
	var slot int = 0
	if nagents == 0 {
		slot = nagents
	} else if newag.targetReplanTime <= agents[nagents-1].targetReplanTime {
		if nagents >= maxAgents {
			return nagents
		}
		slot = nagents
	} else {
		var i int
		for i = 0; i < nagents; i++ {
			if newag.targetReplanTime >= agents[i].targetReplanTime {
				break
			}
		}

		tgt := i + 1
		n := intMin(nagents-i, maxAgents-tgt)

		if tgt+n > maxAgents {
			panic("tgt+n should be <= maxAgents")
		}

		if n > 0 {
			copy(agents[tgt:], agents[i:i+n])
		}
		slot = i
	}

	agents[slot] = newag

	return intMin(nagents+1, maxAgents)
}
