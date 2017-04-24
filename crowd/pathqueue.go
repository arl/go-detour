package crowd

import (
	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/gogeo/f32/d3"
)

const (
	PathQInvalid = 0
	MaxQueue     = 8
)

type PathQueueRef uint32

type pathQuery struct {
	ref PathQueueRef
	// Path find start and end location.
	startPos, endPos [3]float32
	startRef, endRef detour.PolyRef
	// Result.
	path  []detour.PolyRef
	npath int
	// State.
	status    detour.Status
	keepAlive int
	filter    detour.QueryFilter // TODO: This is potentially dangerous!
}

type PathQueue struct {
	m_queue       [MaxQueue]pathQuery
	m_nextHandle  PathQueueRef
	m_maxPathSize int
	m_queueHead   int
	m_navquery    *detour.NavMeshQuery
}

func NewPathQueue() *PathQueue {
	pq := &PathQueue{
		m_nextHandle:  1,
		m_maxPathSize: 0,
		m_queueHead:   0,
		m_navquery:    nil,
	}
	for i := 0; i < MaxQueue; i++ {
		pq.m_queue[i].path = nil
	}
	return pq
}

func (pq *PathQueue) purge() {
	pq.m_navquery = nil
	for i := 0; i < MaxQueue; i++ {
		pq.m_queue[i].path = nil
	}
}

func (pq *PathQueue) Init(maxPathSize, maxSearchNodeCount int, nav *detour.NavMesh) bool {
	pq.purge()

	var status detour.Status
	status, pq.m_navquery = detour.NewNavMeshQuery(nav, int32(maxSearchNodeCount))
	if detour.StatusFailed(status) {
		return false
	}

	pq.m_maxPathSize = maxPathSize
	for i := 0; i < MaxQueue; i++ {
		pq.m_queue[i].ref = PathQInvalid
		pq.m_queue[i].path = make([]detour.PolyRef, pq.m_maxPathSize)
		if pq.m_queue[i].path == nil {
			return false
		}
	}

	pq.m_queueHead = 0
	return true
}

func (pq *PathQueue) Update(maxIters int) {
	const MAX_KEEP_ALIVE = 2 // in update ticks.

	// Update path request until there is nothing to update
	// or upto maxIters pathfinder iterations has been consumed.
	iterCount := maxIters

	for i := 0; i < MaxQueue; i++ {
		q := &pq.m_queue[pq.m_queueHead%MaxQueue]

		// Skip inactive requests.
		if q.ref == PathQInvalid {
			pq.m_queueHead++
			continue
		}

		// Handle completed request.
		if detour.StatusSucceed(q.status) || detour.StatusFailed(q.status) {
			// If the path result has not been read in few frames, free the slot.
			q.keepAlive++
			if q.keepAlive > MAX_KEEP_ALIVE {
				q.ref = PathQInvalid
				q.status = 0
			}

			pq.m_queueHead++
			continue
		}

		// Handle query start.
		if q.status == 0 {
			q.status = pq.m_navquery.InitSlicedFindPath(q.startRef, q.endRef, q.startPos[:], q.endPos[:], q.filter, 0)
		}
		// Handle query in progress.
		if detour.StatusInProgress(q.status) {
			iters := 0
			q.status = pq.m_navquery.UpdateSlicedFindPath(iterCount, &iters)
			iterCount -= iters
		}
		if detour.StatusSucceed(q.status) {
			q.npath, q.status = pq.m_navquery.FinalizeSlicedFindPath(q.path, pq.m_maxPathSize)
		}

		if iterCount <= 0 {
			break
		}

		pq.m_queueHead++
	}
}

func (pq *PathQueue) Request(startRef, endRef detour.PolyRef,
	startPos, endPos d3.Vec3,
	filter detour.QueryFilter) PathQueueRef {

	// Find empty slot
	var slot int = -1
	for i := 0; i < MaxQueue; i++ {
		if pq.m_queue[i].ref == PathQInvalid {
			slot = i
			break
		}
	}
	// Could not find slot.
	if slot == -1 {
		return PathQInvalid
	}

	var ref PathQueueRef = pq.m_nextHandle
	pq.m_nextHandle++
	if pq.m_nextHandle == PathQInvalid {
		pq.m_nextHandle++
	}

	q := &pq.m_queue[slot]
	q.ref = ref
	copy(q.startPos[:], startPos[:3])
	q.startRef = startRef
	copy(q.endPos[:], endPos[:3])
	q.endRef = endRef

	q.status = 0
	q.npath = 0
	q.filter = filter
	q.keepAlive = 0

	return ref

}

func (pq *PathQueue) GetRequestStatus(ref PathQueueRef) detour.Status {
	for i := 0; i < MaxQueue; i++ {
		if pq.m_queue[i].ref == ref {
			return pq.m_queue[i].status
		}
	}
	return detour.Failure
}

func (pq *PathQueue) GetPathResult(ref PathQueueRef, path []detour.PolyRef, pathSize *int, maxPath int) detour.Status {
	for i := 0; i < MaxQueue; i++ {
		if pq.m_queue[i].ref == ref {
			q := &pq.m_queue[i]
			details := q.status & detour.StatusDetailMask
			// Free request for reuse.
			q.ref = PathQInvalid
			q.status = 0
			// Copy path
			var n int
			if q.npath < maxPath {
				n = q.npath
			} else {
				n = maxPath
			}
			copy(path, q.path[:n])
			*pathSize = n
			return details | detour.Success
		}
	}
	return detour.Failure
}

func (pq *PathQueue) GetNavQuery() *detour.NavMeshQuery {
	return pq.m_navquery
}
