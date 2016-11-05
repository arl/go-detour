package detour

import (
	"log"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
)

type DtNodeQueue struct {
	heap     []*DtNode
	capacity int32
	size     int32
}

func newDtNodeQueue(n int32) *DtNodeQueue {
	q := &DtNodeQueue{}

	q.capacity = n
	assert.True(q.capacity > 0, "dtNodeQueue capacity must be > 0")

	q.heap = make([]*DtNode, q.capacity+1)
	assert.True(len(q.heap) > 0, "allocation error")

	return q
}

func (q *DtNodeQueue) bubbleUp(i int32, node *DtNode) {
	parent := (i - 1) / 2
	// note: (index > 0) means there is a parent
	for (i > 0) && (q.heap[parent].Total > node.Total) {
		q.heap[i] = q.heap[parent]
		i = parent
		parent = (i - 1) / 2
	}
	q.heap[i] = node
}

func (q *DtNodeQueue) trickleDown(i int32, node *DtNode) {
	child := (i * 2) + 1
	for child < q.size {
		if ((child + 1) < q.size) &&
			(q.heap[child].Total > q.heap[child+1].Total) {
			child++
		}
		q.heap[i] = q.heap[child]
		i = child
		child = (i * 2) + 1
	}
	q.bubbleUp(i, node)
}

func (q *DtNodeQueue) clear() {
	q.size = 0
}

func (q *DtNodeQueue) top() *DtNode {
	return q.heap[0]
}

func (q *DtNodeQueue) pop() *DtNode {
	result := q.heap[0]
	q.size--
	q.trickleDown(0, q.heap[q.size])
	return result
}

func (q *DtNodeQueue) push(node *DtNode) {
	q.size++
	q.bubbleUp(q.size-1, node)
}

func (q *DtNodeQueue) modify(node *DtNode) {
	for i := int32(0); i < q.size; i++ {
		if q.heap[i] == node {
			q.bubbleUp(i, node)
			return
		}
	}
}

func (q *DtNodeQueue) empty() bool {
	return q.size == 0
}

func (q *DtNodeQueue) getMemUsed() int32 {
	log.Fatal("use of unsafe in getMemUsed")
	return int32(unsafe.Sizeof(*q)) +
		int32(unsafe.Sizeof(DtNode{}))*(q.capacity+1)
}

func (q *DtNodeQueue) getCapacity() int32 {
	return q.capacity
}
