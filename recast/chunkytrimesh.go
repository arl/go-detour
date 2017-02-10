package recast

import (
	"sort"

	"github.com/aurelien-rainone/math32"
)

type chunkyTriMeshNode struct {
	bmin [2]float32
	bmax [2]float32
	i, n int32
}

type chunkyTriMesh struct {
	nodes           []chunkyTriMeshNode
	nnodes          int32
	tris            []int32
	ntris           int32
	maxTrisPerChunk int32
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

type alongXAxis []BoundsItem

// Len is the number of elements in the collection.
func (s alongXAxis) Len() int {
	return len(s)
}

// Less reports whether the element with
// index i should sort before the element with index j.
func (s alongXAxis) Less(i, j int) bool {
	a := s[i]
	b := s[j]

	switch {
	case a.bmin[0] < b.bmin[0]:
		return true
	case a.bmin[0] > b.bmin[0]:
		return false
	default:
		return false
	}
}

// Swap swaps the elements with indexes i and j.
func (s alongXAxis) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

type alongYAxis []BoundsItem

// Len is the number of elements in the collection.
func (s alongYAxis) Len() int {
	return len(s)
}

// Less reports whether the element with
// index i should sort before the element with index j.
func (s alongYAxis) Less(i, j int) bool {
	a := s[i]
	b := s[j]

	switch {
	case a.bmin[1] < b.bmin[1]:
		return true
	case a.bmin[1] > b.bmin[1]:
		return false
	default:
		return false
	}
}

// Swap swaps the elements with indexes i and j.
func (s alongYAxis) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

func subdivide(items []BoundsItem, nitems, imin, imax, trisPerChunk int32,
	curNode *int32, nodes []chunkyTriMeshNode, maxNodes int32,
	curTri *int32, outTris, inTris []int32) {

	inum := imax - imin
	icur := *curNode

	if *curNode > maxNodes {
		return
	}

	node := nodes[*curNode]
	(*curNode)++

	if inum <= trisPerChunk {
		// Leaf
		calcExtends(items, nitems, imin, imax, node.bmin[:], node.bmax[:])

		// Copy triangles.
		node.i = *curTri
		node.n = inum

		for i := imin; i < imax; i++ {
			src := inTris[items[i].i*3 : 3+items[i].i*3]
			dst := outTris[(*curTri)*3 : 3+(*curTri)*3]
			(*curTri)++
			dst[0] = src[0]
			dst[1] = src[1]
			dst[2] = src[2]
		}
	} else {
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin[:], node.bmax[:])

		axis := longestAxis(node.bmax[0]-node.bmin[0],
			node.bmax[1]-node.bmin[1])

		if axis == 0 {
			// Sort along x-axis
			sort.Sort(alongXAxis(items[imin : imin+inum]))
		} else if axis == 1 {
			// Sort along y-axis
			sort.Sort(alongYAxis(items[imin : imin+inum]))
		}

		isplit := imin + inum/2

		// Left
		subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris)
		// Right
		subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris)

		iescape := (*curNode) - icur
		// Negative index means escape.
		node.i = -iescape
	}
}

// Creates partitioned triangle mesh (AABB tree),
// where each node contains at max trisPerChunk triangles.
func createChunkyTriMesh(verts []float32, tris []int32, ntris, trisPerChunk int32, cm *chunkyTriMesh) bool {
	nchunks := (ntris + trisPerChunk - 1) / trisPerChunk
	cm.nodes = make([]chunkyTriMeshNode, nchunks*4)
	if len(cm.nodes) == 0 {
		return false
	}

	cm.tris = make([]int32, ntris*3)
	if len(cm.tris) == 0 {
		return false
	}

	cm.ntris = ntris

	// Build tree
	items := make([]BoundsItem, ntris)
	if len(items) == 0 {
		return false
	}

	for i := int32(0); i < ntris; i++ {
		t := tris[i*3 : i*3+3]
		it := items[i]
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
	subdivide(items, ntris, 0, ntris, trisPerChunk, &curNode, cm.nodes, nchunks*4, &curTri, cm.tris, tris)

	items = nil

	cm.nnodes = curNode

	// Calc max tris per node.
	cm.maxTrisPerChunk = 0
	for i := int32(0); i < cm.nnodes; i++ {
		node := cm.nodes[i]
		isLeaf := node.i >= 0
		if !isLeaf {
			continue
		}
		if node.n > cm.maxTrisPerChunk {
			cm.maxTrisPerChunk = node.n
		}
	}

	return true
}

func checkOverlapRect(amin, amax, bmin, bmax [2]float32) bool {
	overlap := true

	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}

	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	return overlap
}

// Returns the chunk indices which overlap the input rectable.
func (cm *chunkyTriMesh) chunksOverlappingRect(bmin, bmax [2]float32, ids []int32, maxIds int32) bool {
	// Traverse tree
	var i, n int32
	for i < cm.nnodes {
		node := &cm.nodes[i]
		overlap := checkOverlapRect(bmin, bmax, node.bmin, node.bmax)
		isLeafNode := node.i >= 0

		if isLeafNode && overlap {
			if n < maxIds {
				ids[n] = i
				n++
			}
		}

		if overlap || isLeafNode {
			i++
		} else {
			escapeIndex := -node.i
			i += escapeIndex
		}
	}

	return n != 0
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
func (cm *chunkyTriMesh) chunksOverlappingSegment(p, q [2]float32, ids []int32, maxIds int32) int32 {
	// Traverse tree
	var i, n int32
	for i < cm.nnodes {
		node := &cm.nodes[i]
		overlap := checkOverlapSegment(p, q, node.bmin, node.bmax)
		isLeafNode := node.i >= 0

		if isLeafNode && overlap {
			if n < maxIds {
				ids[n] = i
				n++
			}
		}

		if overlap || isLeafNode {
			i++
		} else {
			escapeIndex := -node.i
			i += escapeIndex
		}
	}

	return n
}
