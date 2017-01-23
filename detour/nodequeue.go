package detour

import (
	"log"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
)

type nodeQueue struct {
	heap     []*Node
	capacity int32
	size     int32
}

func newnodeQueue(n int32) *nodeQueue {
	q := &nodeQueue{}

	q.capacity = n
	assert.True(q.capacity > 0, "nodeQueue capacity must be > 0")

	q.heap = make([]*Node, q.capacity+1)
	assert.True(len(q.heap) > 0, "allocation error")

	return q
}

func (q *nodeQueue) bubbleUp(i int32, node *Node) {
	parent := (i - 1) / 2
	// note: (index > 0) means there is a parent
	for (i > 0) && (q.heap[parent].Total > node.Total) {
		q.heap[i] = q.heap[parent]
		i = parent
		parent = (i - 1) / 2
	}
	q.heap[i] = node
}

func (q *nodeQueue) trickleDown(i int32, node *Node) {
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

func (q *nodeQueue) clear() {
	q.size = 0
}

func (q *nodeQueue) top() *Node {
	return q.heap[0]
}

func (q *nodeQueue) pop() *Node {
	result := q.heap[0]
	q.size--
	q.trickleDown(0, q.heap[q.size])
	return result
}

func (q *nodeQueue) push(node *Node) {
	q.size++
	q.bubbleUp(q.size-1, node)
}

func (q *nodeQueue) modify(node *Node) {
	for i := int32(0); i < q.size; i++ {
		if q.heap[i] == node {
			q.bubbleUp(i, node)
			return
		}
	}
}

func (q *nodeQueue) empty() bool {
	return q.size == 0
}

func (q *nodeQueue) memUsed() int32 {
	log.Fatal("use of unsafe in memUsed")
	return int32(unsafe.Sizeof(*q)) +
		int32(unsafe.Sizeof(Node{}))*(q.capacity+1)
}
