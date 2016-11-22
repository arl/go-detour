package detour

import (
	"log"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

func dtHashRef(a DtPolyRef) uint32 {
	a += ^(a << 15)
	a ^= (a >> 10)
	a += (a << 3)
	a ^= (a >> 6)
	a += ^(a << 11)
	a ^= (a >> 16)
	return uint32(a)
}

// DtNodeFlags represent flags associated to a node.
type DtNodeFlags uint8

const (
	dtNodeOpen DtNodeFlags = 1 << iota
	dtNodeClosed
	dtNodeParentDetached // parent of the node is not adjacent. Found using raycast.
)

// DtNodeIndex is the index of a node inside the pool.
type DtNodeIndex uint16

const (
	dtNullIdx DtNodeIndex = ^DtNodeIndex(0)
)

const (
	dtNodeParentBits uint32 = 24
	dtNodeStateBits  uint32 = 2
)

// DtNode represents a node in a weighted graph.
type DtNode struct {
	Pos   d3.Vec3 // Position of the node.
	Cost  float32 // Cost from previous node to current node.
	Total float32 // Cost up to the node.
	//unsigned int pidx : DT_NODE_PARENT_BITS;	// Index to parent node.
	//unsigned int state : DT_NODE_STATE_BITS;	// extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	//unsigned int flags : 3;						// Node flags. A combination of dtNodeFlags.

	// fucking C bitfields!!! as we allocate the dtNode ourselves, we can always use:
	PIdx  uint32
	State uint8
	//Flags uint8
	Flags DtNodeFlags
	ID    DtPolyRef // Polygon ref the node corresponds to.
}

func newDtNode() DtNode {
	return DtNode{
		Pos: d3.NewVec3(),
	}
}

const (
	dtMaxStatesPerNode int32 = 1 << dtNodeStateBits // number of extra states per node. See dtNode::state
)

// DtNodePool is a pool of nodes, it allocated them and allows
// them to be reused.
type DtNodePool struct {
	nodes       []DtNode
	first, next []DtNodeIndex
	maxNodes    int32
	hashSize    int32
	nodeCount   int32
}

func newDtNodePool(maxNodes, hashSize int32) *DtNodePool {
	np := &DtNodePool{
		maxNodes: maxNodes,
		hashSize: hashSize,
	}
	assert.True(math32.NextPow2(uint32(np.hashSize)) == uint32(np.hashSize),
		"m_hashSize should be a power of 2")

	// pidx is special as 0 means "none" and 1 is the first node.
	// For that reason we have 1 fewer nodes available than the
	// number of values it can contain.
	assert.True(np.maxNodes > 0 && np.maxNodes <= int32(dtNullIdx) &&
		np.maxNodes <= (1<<dtNodeParentBits)-1, "DtNodePool, max nodes check failed")

	np.nodes = make([]DtNode, np.maxNodes)
	for i := range np.nodes {
		np.nodes[i] = newDtNode()
	}
	np.next = make([]DtNodeIndex, np.maxNodes)
	np.first = make([]DtNodeIndex, hashSize)

	assert.True(len(np.nodes) > 0, "Nodes should not be empty")
	assert.True(len(np.next) > 0, "Next should not be empty")
	assert.True(len(np.first) > 0, "First should not be empty")

	for idx := range np.first {
		np.first[idx] = dtNullIdx
	}
	for idx := range np.next {
		np.next[idx] = dtNullIdx
	}
	return np
}

// Clear clears the node pool.
func (np *DtNodePool) Clear() {
	assert.True(int(np.hashSize) == len(np.first), "np.HashSize == len(np.First)")
	for idx := range np.first {
		np.first[idx] = dtNullIdx
	}
	np.nodeCount = 0
}

// Node returns a dtNode by ref and extra state information. If
// there is none then allocate. There can be more than one node
// for the same polyRef but with different extra state
// information
func (np *DtNodePool) Node(id DtPolyRef, state uint8) *DtNode {
	bucket := dtHashRef(id) & uint32(np.hashSize-1)

	var i DtNodeIndex
	i = np.first[bucket]
	var node *DtNode
	for i != dtNullIdx {
		if np.nodes[i].ID == id && np.nodes[i].State == state {
			return &np.nodes[i]
		}
		i = np.next[i]
	}

	if np.nodeCount >= np.maxNodes {
		return nil
	}

	i = DtNodeIndex(np.nodeCount)
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
func (np *DtNodePool) FindNode(id DtPolyRef, state uint8) *DtNode {
	bucket := dtHashRef(id) & uint32(np.hashSize-1)
	i := np.first[bucket]
	for i != dtNullIdx {
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
func (np *DtNodePool) FindNodes(id DtPolyRef, nodes []*DtNode, maxNodes int32) uint32 {
	assert.True(len(nodes) >= int(maxNodes), "nodes should contain at least maxNodes elements")

	var n uint32
	bucket := dtHashRef(id) & uint32(np.hashSize-1)
	i := np.first[bucket]
	for i != dtNullIdx {
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
func (np *DtNodePool) NodeIdx(node *DtNode) uint32 {
	if node == nil {
		return 0
	}

	e := uintptr(unsafe.Pointer(node)) - uintptr(unsafe.Pointer(&np.nodes[0]))
	ip := uint32(e / unsafe.Sizeof(*node))

	assert.True(ip < uint32(len(np.nodes)), "ip should be < len(np.Npdes), ip=%d, len(np.Nodes)=%d", ip, len(np.nodes))

	return ip + 1
}

// NodeAtIdx returns the node at given index.
func (np *DtNodePool) NodeAtIdx(idx int32) *DtNode {
	if idx == 0 {
		return nil
	}
	return &np.nodes[idx-1]
}

// MemUsed returns the number of bytes currently in use in this
// node pool.
func (np *DtNodePool) MemUsed() int32 {
	log.Fatal("use of unsafe in getMemUsed")
	return int32(unsafe.Sizeof(*np)) +
		int32(unsafe.Sizeof(DtNode{}))*np.maxNodes +
		int32(unsafe.Sizeof(DtNodeIndex(0)))*np.maxNodes +
		int32(unsafe.Sizeof(DtNodeIndex(0)))*np.hashSize
}

// MaxNodes returns the maximum number of nodes the pool can
// contain.
func (np *DtNodePool) MaxNodes() int32 {
	return np.maxNodes
}

// HashSize returns the hash size.
func (np *DtNodePool) HashSize() int32 {
	return np.hashSize
}

// First returns the index of the first node.
func (np *DtNodePool) First(bucket int32) DtNodeIndex {
	return np.first[bucket]
}

// Next returns the next node index after the given one.
func (np *DtNodePool) Next(i int32) DtNodeIndex {
	return np.next[i]
}

// NodeCount returns the current node count in the pool.
func (np *DtNodePool) NodeCount() int32 {
	return np.nodeCount
}
