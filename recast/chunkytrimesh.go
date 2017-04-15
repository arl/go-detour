package recast

import (
	"sort"

	"github.com/aurelien-rainone/math32"
)

type ChunkyTriMeshNode struct {
	BMin [2]float32
	BMax [2]float32
	I, N int32
}

type ChunkyTriMesh struct {
	Nodes           []ChunkyTriMeshNode
	Nnodes          int32
	Tris            []int32
	Ntris           int32
	MaxTrisPerChunk int32
}

type BoundsItem struct {
	bmin [2]float32
	bmax [2]float32
	i    int32
}

func calcExtends(items []BoundsItem, nitems, imin, imax int32, bmin []float32, bmax []float32) {
	bmin[0] = items[imin].bmin[0]
	bmin[1] = items[imin].bmin[1]

	bmax[0] = items[imin].bmax[0]
	bmax[1] = items[imin].bmax[1]

	for i := imin + 1; i < imax; i++ {
		it := items[i]
		if it.bmin[0] < bmin[0] {
			bmin[0] = it.bmin[0]
		}
		if it.bmin[1] < bmin[1] {
			bmin[1] = it.bmin[1]
		}

		if it.bmax[0] > bmax[0] {
			bmax[0] = it.bmax[0]
		}
		if it.bmax[1] > bmax[1] {
			bmax[1] = it.bmax[1]
		}
	}
}

func longestAxis(x, y float32) int {
	if y > x {
		return 1
	}
	return 0
}

func subdivide(items []BoundsItem, nitems, imin, imax, trisPerChunk int32,
	curNode *int32, nodes []ChunkyTriMeshNode, maxNodes int32,
	curTri *int32, outTris, inTris []int32) {

	inum := imax - imin
	icur := *curNode

	if *curNode > maxNodes {
		return
	}

	node := &nodes[*curNode]
	(*curNode)++

	if inum <= trisPerChunk {
		// Leaf
		calcExtends(items, nitems, imin, imax, node.BMin[:], node.BMax[:])

		// Copy triangles.
		node.I = *curTri
		node.N = inum

		for i := imin; i < imax; i++ {
			src := inTris[items[i].i*3:]
			dst := outTris[(*curTri)*3:]
			(*curTri)++
			copy(dst, src[:3])
		}
	} else {
		// Split
		calcExtends(items, nitems, imin, imax, node.BMin[:], node.BMax[:])

		axis := longestAxis(node.BMax[0]-node.BMin[0],
			node.BMax[1]-node.BMin[1])

		if axis == 0 {
			// Sort along x-axis
			sort.SliceStable(items[imin:imin+inum],
				func(i, j int) bool { return items[int(imin)+i].bmin[0] < items[int(imin)+j].bmin[0] })

		} else {
			// Sort along y-axis
			sort.SliceStable(items[imin:imin+inum],
				func(i, j int) bool { return items[int(imin)+i].bmin[1] < items[int(imin)+j].bmin[1] })
		}

		isplit := imin + inum/2

		// Left
		subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris)
		// Right
		subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris)

		iescape := (*curNode) - icur
		// Negative index means escape.
		node.I = -iescape
	}
}

// Creates partitioned triangle mesh (AABB tree),
// where each node contains at max trisPerChunk triangles.
func createChunkyTriMesh(verts []float32, tris []int32, ntris, trisPerChunk int32, cm *ChunkyTriMesh) bool {
	nchunks := (ntris + trisPerChunk - 1) / trisPerChunk
	cm.Nodes = make([]ChunkyTriMeshNode, nchunks*4)
	if len(cm.Nodes) == 0 {
		return false
	}

	cm.Tris = make([]int32, ntris*3)
	if len(cm.Tris) == 0 {
		return false
	}

	cm.Ntris = ntris

	// Build tree
	items := make([]BoundsItem, ntris)
	if len(items) == 0 {
		return false
	}

	for i := int32(0); i < ntris; i++ {
		t := tris[i*3 : i*3+3]
		it := &items[i]
		it.i = i
		// Calc triangle XZ bounds.
		it.bmax[0] = verts[t[0]*3+0]
		it.bmin[0] = it.bmax[0]
		it.bmax[1] = verts[t[0]*3+2]
		it.bmin[1] = it.bmax[1]

		for j := 1; j < 3; j++ {
			v := verts[t[j]*3 : 3+t[j]*3]
			if v[0] < it.bmin[0] {
				it.bmin[0] = v[0]
			}
			if v[2] < it.bmin[1] {
				it.bmin[1] = v[2]
			}

			if v[0] > it.bmax[0] {
				it.bmax[0] = v[0]
			}
			if v[2] > it.bmax[1] {
				it.bmax[1] = v[2]
			}
		}
	}

	var curTri, curNode int32
	subdivide(items, ntris, 0, ntris, trisPerChunk, &curNode, cm.Nodes, nchunks*4, &curTri, cm.Tris, tris)

	items = nil

	cm.Nnodes = curNode

	// Calc max tris per node.
	cm.MaxTrisPerChunk = 0
	for i := int32(0); i < cm.Nnodes; i++ {
		node := &cm.Nodes[i]
		isLeaf := node.I >= 0
		if !isLeaf {
			continue
		}
		if node.N > cm.MaxTrisPerChunk {
			cm.MaxTrisPerChunk = node.N
		}
	}

	return true
}

func checkOverlapRect(amin, amax, bmin, bmax [2]float32) bool {
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		return false
	}

	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		return false
	}
	return true
}

// Returns the chunk indices which overlap the input rectable.
func (cm *ChunkyTriMesh) ChunksOverlappingRect(bmin, bmax [2]float32, ids []int32) int {
	// Traverse tree
	var (
		i int32
		n int
	)
	for i < cm.Nnodes {
		node := &cm.Nodes[i]
		overlap := checkOverlapRect(bmin, bmax, node.BMin, node.BMax)
		isLeafNode := node.I >= 0

		if isLeafNode && overlap {
			if n < len(ids) {
				ids[n] = i
				n++
			}
		}

		if overlap || isLeafNode {
			i++
		} else {
			escapeIndex := -node.I
			i += escapeIndex
		}
	}

	return n
}

func checkOverlapSegment(p, q, bmin, bmax [2]float32) bool {
	EPSILON := float32(1e-6)
	tmin := float32(0)
	tmax := float32(1)
	var d [2]float32

	d[0] = q[0] - p[0]
	d[1] = q[1] - p[1]

	for i := 0; i < 2; i++ {
		if math32.Abs(d[i]) < EPSILON {
			// Ray is parallel to slab. No hit if origin not within slab
			if p[i] < bmin[i] || p[i] > bmax[i] {
				return false
			}
		} else {
			// Compute intersection t value of ray with near and far plane of slab
			ood := float32(1.0) / d[i]
			t1 := (bmin[i] - p[i]) * ood
			t2 := (bmax[i] - p[i]) * ood
			if t1 > t2 {
				t1, t2 = t2, t1
			}
			if t1 > tmin {
				tmin = t1
			}
			if t2 < tmax {
				tmax = t2
			}
			if tmin > tmax {
				return false
			}
		}
	}
	return true
}

// Returns the chunk indices which overlap the input segment.
func (cm *ChunkyTriMesh) chunksOverlappingSegment(p, q [2]float32, ids []int32, maxIds int32) int32 {
	// Traverse tree
	var i, n int32
	for i < cm.Nnodes {
		node := &cm.Nodes[i]
		overlap := checkOverlapSegment(p, q, node.BMin, node.BMax)
		isLeafNode := node.I >= 0

		if isLeafNode && overlap {
			if n < maxIds {
				ids[n] = i
				n++
			}
		}

		if overlap || isLeafNode {
			i++
		} else {
			escapeIndex := -node.I
			i += escapeIndex
		}
	}

	return n
}
