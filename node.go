package detour

import (
	"log"
	"unsafe"

	"github.com/aurelien-rainone/assertgo"
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

type dtNodeFlags uint32

const (
	DT_NODE_OPEN            dtNodeFlags = 0x01
	DT_NODE_CLOSED                      = 0x02
	DT_NODE_PARENT_DETACHED             = 0x04 // parent of the node is not adjacent. Found using raycast.
)

type DtNodeIndex uint16

const (
	DT_NULL_IDX DtNodeIndex = ^DtNodeIndex(0)
)

const (
	DT_NODE_PARENT_BITS uint32 = 24
	DT_NODE_STATE_BITS  uint32 = 2
)

type DtNode struct {
	Pos   [3]float32 ///< Position of the node.
	Cost  float32    ///< Cost from previous node to current node.
	Total float32    ///< Cost up to the node.
	//unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
	//unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	//unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.

	// fucking C bitfields!!! as we allocate the dtNode ourselves, we can always use:
	PIdx  uint32
	State uint8
	Flags uint8
	ID    DtPolyRef ///< Polygon ref the node corresponds to.
}

const (
	DT_MAX_STATES_PER_NODE int32 = 1 << DT_NODE_STATE_BITS // number of extra states per node. See dtNode::state
)

type DtNodePool struct {
	Nodes       []DtNode
	First, Next []DtNodeIndex
	MaxNodes    int32
	HashSize    int32
	NodeCount   int32
}

func newDtNodePool(maxNodes, hashSize int32) *DtNodePool {
	np := &DtNodePool{
		MaxNodes: maxNodes,
		HashSize: hashSize,
	}
	assert.True(dtNextPow2(uint32(np.HashSize)) == uint32(np.HashSize), "m_hashSize should be a power of 2")

	// pidx is special as 0 means "none" and 1 is the first node. For that reason
	// we have 1 fewer nodes available than the number of values it can contain.
	assert.True(np.MaxNodes > 0 && np.MaxNodes <= int32(DT_NULL_IDX) && np.MaxNodes <= (1<<DT_NODE_PARENT_BITS)-1, "DtNodePool, max nodes check failed")

	//m_nodes = (dtNode*)dtAlloc(sizeof(dtNode)*m_maxNodes, DT_ALLOC_PERM);
	np.Nodes = make([]DtNode, np.MaxNodes)
	//m_next = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*m_maxNodes, DT_ALLOC_PERM);
	np.Next = make([]DtNodeIndex, np.MaxNodes)
	//m_first = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*hashSize, DT_ALLOC_PERM);
	np.First = make([]DtNodeIndex, hashSize)

	assert.True(len(np.Nodes) > 0, "Nodes should not be empty")
	assert.True(len(np.Next) > 0, "Next should not be empty")
	assert.True(len(np.First) > 0, "First should not be empty")

	for idx := range np.First {
		np.First[idx] = 0xff
	}
	for idx := range np.Next {
		np.Next[idx] = 0xff
	}
	return np
}

func (np *DtNodePool) clear() {
	//memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize)
	for idx := range np.First {
		np.First[idx] = 0xff
	}
	np.NodeCount = 0
}

// Get a dtNode by ref and extra state information. If there is none then - allocate
// There can be more than one node for the same polyRef but with different extra state information
func (np *DtNodePool) getNode(id DtPolyRef, state uint8) *DtNode {
	bucket := dtHashRef(id) & uint32(np.HashSize-1)
	i := np.First[bucket]
	var node *DtNode
	for i != DT_NULL_IDX {
		if np.Nodes[i].ID == id && np.Nodes[i].State == state {
			return &np.Nodes[i]
		}
		i = np.Next[i]
	}

	if np.NodeCount >= np.MaxNodes {
		return nil
	}

	i = DtNodeIndex(np.NodeCount)
	np.NodeCount++

	// Init node
	node = &np.Nodes[i]
	node.PIdx = 0
	node.Cost = 0
	node.Total = 0
	node.ID = id
	node.State = state
	node.Flags = 0

	np.Next[i] = np.First[bucket]
	np.First[bucket] = i

	return node
}

func (np *DtNodePool) getNode2(id DtPolyRef) *DtNode {
	return np.getNode(id, 0)
}

func (np *DtNodePool) findNode(id DtPolyRef, state uint8) *DtNode {
	bucket := dtHashRef(id) & uint32(np.HashSize-1)
	i := np.First[bucket]
	for i != DT_NULL_IDX {
		if np.Nodes[i].ID == id && np.Nodes[i].State == state {
			return &np.Nodes[i]
		}
		i = np.Next[i]
	}
	return nil
}

func (np *DtNodePool) findNodes(id DtPolyRef, nodes *[]*DtNode, maxNodes int32) uint32 {
	var n uint32
	bucket := dtHashRef(id) & uint32(np.HashSize-1)
	i := np.First[bucket]
	for i != DT_NULL_IDX {
		if np.Nodes[i].ID == id {
			if n >= uint32(maxNodes) {
				return n
			}
			(*nodes)[n] = &np.Nodes[i]
			n++
		}
		i = np.Next[i]
	}
	return n
}

func (np *DtNodePool) getNodeIdx(node *DtNode) uint32 {
	if node == nil {
		return 0
	}
	//return uint32(node-np.m_nodes) + 1

	// TODO: use unsafe.Pointer here...
	e := uintptr(unsafe.Pointer(node)) - uintptr(unsafe.Pointer(&np.Nodes[0]))
	ip := uint32(e / unsafe.Sizeof(node))
	log.Fatal("use of unsafe in getNodeIdx")
	return ip + 1
}

func (np *DtNodePool) getNodeAtIdx(idx int32) *DtNode {
	if idx == 0 {
		return nil
	}

	return &np.Nodes[idx-1]
}

func (np *DtNodePool) getMemUsed() int32 {
	log.Fatal("use of unsafe in getMemUsed")
	return int32(unsafe.Sizeof(*np)) +
		int32(unsafe.Sizeof(DtNode{}))*np.MaxNodes +
		int32(unsafe.Sizeof(DtNodeIndex(0)))*np.MaxNodes +
		int32(unsafe.Sizeof(DtNodeIndex(0)))*np.HashSize
}

func (np *DtNodePool) getMaxNodes() int32 {
	return np.MaxNodes
}

func (np *DtNodePool) getHashSize() int32 {
	return np.HashSize
}

func (np *DtNodePool) getFirst(bucket int32) DtNodeIndex {
	return np.First[bucket]
}

func (np *DtNodePool) getNext(i int32) DtNodeIndex {
	return np.Next[i]
}

func (np *DtNodePool) getNodeCount() int32 {
	return np.NodeCount
}
