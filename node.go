package detour

import (
	"log"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

func hashRef(a PolyRef) uint32 {
	a += ^(a << 15)
	a ^= (a >> 10)
	a += (a << 3)
	a ^= (a >> 6)
	a += ^(a << 11)
	a ^= (a >> 16)
	return uint32(a)
}

// NodeFlags represent flags associated to a node.
type NodeFlags uint8

const (
	nodeOpen NodeFlags = 1 << iota
	nodeClosed
	nodeParentDetached // parent of the node is not adjacent. Found using raycast.
)

// NodeIndex is the index of a node inside the pool.
type NodeIndex uint16

const (
	nullIdx NodeIndex = ^NodeIndex(0)
)

const (
	nodeParentBits uint32 = 24
	nodeStateBits  uint32 = 2
)

// Node represents a node in a weighted graph.
type Node struct {
	Pos   d3.Vec3 // Position of the node.
	Cost  float32 // Cost from previous node to current node.
	Total float32 // Cost up to the node.
	//unsigned int pidx : nodeParentBits;	// Index to parent node.
	//unsigned int state : DT_NODE_STATE_BITS;	// extra state information. A polyRef can have multiple nodes with different extra info. see maxStatesPerNode
	//unsigned int flags : 3;						// Node flags. A combination of NodeFlags.

	// fucking C bitfields!!! as we allocate the Node ourselves, we can always use:
	PIdx  uint32
	State uint8
	//Flags uint8
	Flags NodeFlags
	ID    PolyRef // Polygon ref the node corresponds to.
}

func newNode() Node {
	return Node{
		Pos: d3.NewVec3(),
	}
}

const (
	maxStatesPerNode int32 = 1 << nodeStateBits // number of extra states per node. See Node::state
)

// NodePool is a pool of nodes, it allocated them and allows
// them to be reused.
type NodePool struct {
	nodes       []Node
	first, next []NodeIndex
	maxNodes    int32
	hashSize    int32
	nodeCount   int32
}

func newNodePool(maxNodes, hashSize int32) *NodePool {
	np := &NodePool{
		maxNodes: maxNodes,
		hashSize: hashSize,
	}
	assert.True(math32.NextPow2(uint32(np.hashSize)) == uint32(np.hashSize),
		"m_hashSize should be a power of 2")

	// pidx is special as 0 means "none" and 1 is the first node.
	// For that reason we have 1 fewer nodes available than the
	// number of values it can contain.
	assert.True(np.maxNodes > 0 && np.maxNodes <= int32(nullIdx) &&
		np.maxNodes <= (1<<nodeParentBits)-1, "NodePool, max nodes check failed")

	np.nodes = make([]Node, np.maxNodes)
	for i := range np.nodes {
		np.nodes[i] = newNode()
	}
	np.next = make([]NodeIndex, np.maxNodes)
	np.first = make([]NodeIndex, hashSize)

	assert.True(len(np.nodes) > 0, "Nodes should not be empty")
	assert.True(len(np.next) > 0, "Next should not be empty")
	assert.True(len(np.first) > 0, "First should not be empty")

	for idx := range np.first {
		np.first[idx] = nullIdx
	}
	for idx := range np.next {
		np.next[idx] = nullIdx
	}
	return np
}

// Clear clears the node pool.
func (np *NodePool) Clear() {
	assert.True(int(np.hashSize) == len(np.first), "np.HashSize == len(np.First)")
	for idx := range np.first {
		np.first[idx] = nullIdx
	}
	np.nodeCount = 0
}

// Node returns a Node by ref and extra state information. If
// there is none then allocate. There can be more than one node
// for the same polyRef but with different extra state
// information
func (np *NodePool) Node(id PolyRef, state uint8) *Node {
	bucket := hashRef(id) & uint32(np.hashSize-1)

	var i NodeIndex
	i = np.first[bucket]
	var node *Node
	for i != nullIdx {
		if np.nodes[i].ID == id && np.nodes[i].State == state {
			return &np.nodes[i]
		}
		i = np.next[i]
	}

	if np.nodeCount >= np.maxNodes {
		return nil
	}

	i = NodeIndex(np.nodeCount)
	np.nodeCount++

	// Init node
	node = &np.nodes[i]
	node.PIdx = 0
	node.Cost = 0
	node.Total = 0
	node.ID = id
	node.State = state
	node.Flags = 0

	np.next[i] = np.first[bucket]
	np.first[bucket] = i

	return node
}

// FindNode finds the node that corresponds to the given polygon
// reference and having the given state
func (np *NodePool) FindNode(id PolyRef, state uint8) *Node {
	bucket := hashRef(id) & uint32(np.hashSize-1)
	i := np.first[bucket]
	for i != nullIdx {
		if np.nodes[i].ID == id && np.nodes[i].State == state {
			return &np.nodes[i]
		}
		i = np.next[i]
	}
	return nil
}

// FindNodes fills the provided nodes slices with nodes that
// correspond to the given polygon reference.
//
// Note: No more than maxNodes will be returned (nodes should
// have at least maxNodes elements)
func (np *NodePool) FindNodes(id PolyRef, nodes []*Node, maxNodes int32) uint32 {
	assert.True(len(nodes) >= int(maxNodes), "nodes should contain at least maxNodes elements")

	var n uint32
	bucket := hashRef(id) & uint32(np.hashSize-1)
	i := np.first[bucket]
	for i != nullIdx {
		if np.nodes[i].ID == id {
			if n >= uint32(maxNodes) {
				return n
			}
			nodes[n] = &np.nodes[i]
			n++
		}
		i = np.next[i]
	}
	return n
}

// NodeIdx returns the index of the given node.
func (np *NodePool) NodeIdx(node *Node) uint32 {
	if node == nil {
		return 0
	}

	e := uintptr(unsafe.Pointer(node)) - uintptr(unsafe.Pointer(&np.nodes[0]))
	ip := uint32(e / unsafe.Sizeof(*node))

	assert.True(ip < uint32(len(np.nodes)), "ip should be < len(np.Npdes), ip=%d, len(np.Nodes)=%d", ip, len(np.nodes))

	return ip + 1
}

// NodeAtIdx returns the node at given index.
func (np *NodePool) NodeAtIdx(idx int32) *Node {
	if idx == 0 {
		return nil
	}
	return &np.nodes[idx-1]
}

// MemUsed returns the number of bytes currently in use in this
// node pool.
func (np *NodePool) MemUsed() int32 {
	log.Fatal("use of unsafe in getMemUsed")
	return int32(unsafe.Sizeof(*np)) +
		int32(unsafe.Sizeof(Node{}))*np.maxNodes +
		int32(unsafe.Sizeof(NodeIndex(0)))*np.maxNodes +
		int32(unsafe.Sizeof(NodeIndex(0)))*np.hashSize
}

// MaxNodes returns the maximum number of nodes the pool can
// contain.
func (np *NodePool) MaxNodes() int32 {
	return np.maxNodes
}

// HashSize returns the hash size.
func (np *NodePool) HashSize() int32 {
	return np.hashSize
}

// First returns the index of the first node.
func (np *NodePool) First(bucket int32) NodeIndex {
	return np.first[bucket]
}

// Next returns the next node index after the given one.
func (np *NodePool) Next(i int32) NodeIndex {
	return np.next[i]
}

// NodeCount returns the current node count in the pool.
func (np *NodePool) NodeCount() int32 {
	return np.nodeCount
}
