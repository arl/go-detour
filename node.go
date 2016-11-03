package detour

import (
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
)

func dtHashRef(a dtPolyRef) uint32 {
	a += ^(a << 15)
	a ^= (a >> 10)
	a += (a << 3)
	a ^= (a >> 6)
	a += ^(a << 11)
	a ^= (a >> 16)
	return uint32(a)
}

type dtNodeFlags uint32

const (
	DT_NODE_OPEN            dtNodeFlags = 0x01
	DT_NODE_CLOSED                      = 0x02
	DT_NODE_PARENT_DETACHED             = 0x04 // parent of the node is not adjacent. Found using raycast.
)

type dtNodeIndex uint16

const (
	DT_NULL_IDX dtNodeIndex = ^dtNodeIndex(0)
)

const (
	DT_NODE_PARENT_BITS uint32 = 24
	DT_NODE_STATE_BITS  uint32 = 2
)

type dtNode struct {
	pos   [3]float32 ///< Position of the node.
	cost  float32    ///< Cost from previous node to current node.
	total float32    ///< Cost up to the node.
	//unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
	//unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	//unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.

	// fucking C bitfields!!! as we allocate the dtNode ourselves, we can always use:
	pidx  uint32
	state uint8
	flags uint8
	id    dtPolyRef ///< Polygon ref the node corresponds to.
}

const (
	DT_MAX_STATES_PER_NODE int32 = 1 << DT_NODE_STATE_BITS // number of extra states per node. See dtNode::state
)

type dtNodePool struct {
	m_nodes         []dtNode
	m_first, m_next []dtNodeIndex
	m_maxNodes      int32
	m_hashSize      int32
	m_nodeCount     int32
}

func newDtNodePool(maxNodes, hashSize int32) *dtNodePool {
	np := &dtNodePool{
		m_maxNodes: maxNodes,
		m_hashSize: hashSize,
	}

	assert.True(dtNextPow2(uint32(np.m_hashSize)) == uint32(np.m_hashSize), "m_hashSize should be a power of 2")

	// pidx is special as 0 means "none" and 1 is the first node. For that reason
	// we have 1 fewer nodes available than the number of values it can contain.
	assert.True(np.m_maxNodes > 0 && np.m_maxNodes <= int32(DT_NULL_IDX) && np.m_maxNodes <= (1<<DT_NODE_PARENT_BITS)-1, "DtNodePool, max nodes check failed")

	//m_nodes = (dtNode*)dtAlloc(sizeof(dtNode)*m_maxNodes, DT_ALLOC_PERM);
	np.m_nodes = make([]dtNode, np.m_maxNodes)
	//m_next = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*m_maxNodes, DT_ALLOC_PERM);
	np.m_next = make([]dtNodeIndex, np.m_maxNodes)
	//m_first = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*hashSize, DT_ALLOC_PERM);
	np.m_first = make([]dtNodeIndex, hashSize)

	assert.True(len(np.m_nodes) > 0, "m_nodes should not be empty")
	assert.True(len(np.m_next) > 0, "m_next should not be empty")
	assert.True(len(np.m_first) > 0, "m_first should not be empty")

	for idx := range np.m_first {
		np.m_first[idx] = 0xff
	}
	for idx := range np.m_next {
		np.m_next[idx] = 0xff
	}
	///memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize);
	//memset(m_next, 0xff, sizeof(dtNodeIndex)*m_maxNodes);
	return np
}

func (np *dtNodePool) clear() {
	//memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize)
	for idx := range np.m_first {
		np.m_first[idx] = 0xff
	}
	np.m_nodeCount = 0
}

// Get a dtNode by ref and extra state information. If there is none then - allocate
// There can be more than one node for the same polyRef but with different extra state information
func (np *dtNodePool) getNode(id dtPolyRef, state uint8) *dtNode {
	bucket := dtHashRef(id) & uint32(np.m_hashSize-1)
	i := np.m_first[bucket]
	var node *dtNode
	for i != DT_NULL_IDX {
		if np.m_nodes[i].id == id && np.m_nodes[i].state == state {
			return &np.m_nodes[i]
		}
		i = np.m_next[i]
	}

	if np.m_nodeCount >= np.m_maxNodes {
		return nil
	}

	i = dtNodeIndex(np.m_nodeCount)
	np.m_nodeCount++

	// Init node
	node = &np.m_nodes[i]
	node.pidx = 0
	node.cost = 0
	node.total = 0
	node.id = id
	node.state = state
	node.flags = 0

	np.m_next[i] = np.m_first[bucket]
	np.m_first[bucket] = i

	return node
}

func (np *dtNodePool) findNode(id dtPolyRef, state uint8) *dtNode {
	bucket := dtHashRef(id) & uint32(np.m_hashSize-1)
	i := np.m_first[bucket]
	for i != DT_NULL_IDX {
		if np.m_nodes[i].id == id && np.m_nodes[i].state == state {
			return &np.m_nodes[i]
		}
		i = np.m_next[i]
	}
	return nil
}

func (np *dtNodePool) findNodes(id dtPolyRef, nodes *[]*dtNode, maxNodes int32) uint32 {
	var n uint32
	bucket := dtHashRef(id) & uint32(np.m_hashSize-1)
	i := np.m_first[bucket]
	for i != DT_NULL_IDX {
		if np.m_nodes[i].id == id {
			if n >= uint32(maxNodes) {
				return n
			}
			(*nodes)[n] = &np.m_nodes[i]
			n++
		}
		i = np.m_next[i]
	}
	return n
}

func (np *dtNodePool) getNodeIdx(node *dtNode) uint32 {
	if node == nil {
		return 0
	}
	//return uint32(node-np.m_nodes) + 1

	// TODO: use unsafe.Pointer here...
	e := uintptr(unsafe.Pointer(node)) - uintptr(unsafe.Pointer(&np.m_nodes[0]))
	ip := uint32(e / unsafe.Sizeof(node))
	return ip + 1
}

func (np *dtNodePool) getNodeAtIdx(idx int32) *dtNode {
	if idx == 0 {
		return nil
	}

	return &np.m_nodes[idx-1]
}

func (np *dtNodePool) getMemUsed() int32 {
	return int32(unsafe.Sizeof(*np)) +
		int32(unsafe.Sizeof(dtNode{}))*np.m_maxNodes +
		int32(unsafe.Sizeof(dtNodeIndex(0)))*np.m_maxNodes +
		int32(unsafe.Sizeof(dtNodeIndex(0)))*np.m_hashSize
}

func (np *dtNodePool) getMaxNodes() int32 {
	return np.m_maxNodes
}

func (np *dtNodePool) getHashSize() int32 {
	return np.m_hashSize
}

func (np *dtNodePool) getFirst(bucket int32) dtNodeIndex {
	return np.m_first[bucket]
}

func (np *dtNodePool) getNext(i int32) dtNodeIndex {
	return np.m_next[i]
}

func (np *dtNodePool) getNodeCount() int32 {
	return np.m_nodeCount
}
