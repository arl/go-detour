package recast

type Edge struct {
	vert     [2]uint16
	polyEdge [2]uint16
	poly     [2]uint16
}

func buildMeshAdjacency(polys []uint16, npolys int32, nverts, vertsPerPoly int32) bool {
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php

	maxEdgeCount := npolys * vertsPerPoly
	firstEdge := make([]uint16, nverts+maxEdgeCount)
	nextEdge := firstEdge[nverts:]
	var edgeCount int32

	edges := make([]*Edge, maxEdgeCount)
	for i := range edges {
		edges[i] = new(Edge)
	}

	for i := int32(0); i < nverts; i++ {
		firstEdge[i] = meshNullIdx
	}

	for i := int32(0); i < npolys; i++ {
		t := polys[i*vertsPerPoly*2:]
		for j := int32(0); j < vertsPerPoly; j++ {
			if t[j] == meshNullIdx {
				break
			}
			var v0, v1 uint16
			v0 = t[j]
			if j+1 >= vertsPerPoly || t[j+1] == meshNullIdx {
				v1 = t[0]
			} else {
				v1 = t[j+1]
			}

			if v0 < v1 {
				edge := edges[edgeCount]
				edge.vert[0] = v0
				edge.vert[1] = v1
				edge.poly[0] = uint16(i)
				edge.polyEdge[0] = uint16(j)
				edge.poly[1] = uint16(i)
				edge.polyEdge[1] = 0
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0]
				firstEdge[v0] = uint16(edgeCount)
				edgeCount++
			}
		}
	}

	for i := int32(0); i < npolys; i++ {
		t := polys[i*vertsPerPoly*2:]
		for j := int32(0); j < vertsPerPoly; j++ {
			if t[j] == meshNullIdx {
				break
			}
			var v0, v1 uint16
			v0 = t[j]
			if j+1 >= vertsPerPoly || t[j+1] == meshNullIdx {
				v1 = t[0]
			} else {
				v1 = t[j+1]
			}
			if v0 > v1 {
				for e := uint16(firstEdge[v1]); e != meshNullIdx; e = nextEdge[e] {
					edge := edges[e]
					if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
						edge.poly[1] = uint16(i)
						edge.polyEdge[1] = uint16(j)
						break
					}
				}
			}
		}
	}

	// Store adjacency
	for i := int32(0); i < edgeCount; i++ {
		e := edges[i]
		if e.poly[0] != e.poly[1] {
			p0 := polys[int32(e.poly[0])*vertsPerPoly*2:]
			p1 := polys[int32(e.poly[1])*vertsPerPoly*2:]
			p0[vertsPerPoly+int32(e.polyEdge[0])] = e.poly[1]
			p1[vertsPerPoly+int32(e.polyEdge[1])] = e.poly[0]
		}
	}

	return true
}

const VERTEX_BUCKET_COUNT int32 = 1 << 12

func computeVertexHash(x, y, z int32) int32 {
	const (
		h1 int64 = 0x8da6b343 // Large multiplicative constants;
		h2       = 0xd8163841 // here arbitrarily chosen primes
		h3       = 0xcb1ab31f
	)
	n := uint32(h1*int64(x) + h2*int64(y) + h3*int64(z))
	return int32(n & uint32(VERTEX_BUCKET_COUNT-1))
}

func addVertex(x, y, z uint16, verts []uint16, firstVert, nextVert []int32, nv *int32) uint16 {
	bucket := computeVertexHash(int32(x), 0, int32(z))
	i := firstVert[bucket]

	for i != -1 {
		v := verts[i*3:]
		if v[0] == x && (iAbs(int32(v[1]-y)) <= 2) && v[2] == z {
			return uint16(i)
		}
		i = nextVert[i] // next
	}

	// Could not find, create new.
	i = *nv
	*nv = *nv + 1
	v := verts[i*3:]
	v[0] = x
	v[1] = y
	v[2] = z
	nextVert[i] = firstVert[bucket]
	firstVert[bucket] = i

	return uint16(i)
}

// Returns true iff the diagonal (i,j) is strictly internal to the
// polygon P in the neighborhood of the i endpoint.
func inCone5(i, j, n int32, verts []int32, indices []int64) bool {
	pi := verts[(indices[i]&0x0fffffff)*4:]
	pj := verts[(indices[j]&0x0fffffff)*4:]
	pi1 := verts[(indices[next(i, n)]&0x0fffffff)*4:]
	pin1 := verts[(indices[prev(i, n)]&0x0fffffff)*4:]

	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if leftOn(pin1, pi, pi1) {
		return left(pi, pj, pin1) && left(pj, pi, pi1)
	}
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1))
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
func diagonal(i, j, n int32, verts []int32, indices []int64) bool {
	return inCone5(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices)
}

func diagonalieLoose(i, j, n int32, verts []int32, indices []int64) bool {
	d0 := verts[(indices[i]&0x0fffffff)*4:]
	d1 := verts[(indices[j]&0x0fffffff)*4:]

	// For each edge (k,k+1) of P
	for k := int32(0); k < n; k++ {
		k1 := next(k, n)
		// Skip edges incident to i or j
		if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
			p0 := verts[(indices[k]&0x0fffffff)*4:]
			p1 := verts[(indices[k1]&0x0fffffff)*4:]

			if vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1) {
				continue
			}

			if intersectProp(d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}

func inConeLoose(i, j, n int32, verts []int32, indices []int64) bool {
	pi := verts[(indices[i]&0x0fffffff)*4:]
	pj := verts[(indices[j]&0x0fffffff)*4:]
	pi1 := verts[(indices[next(i, n)]&0x0fffffff)*4:]
	pin1 := verts[(indices[prev(i, n)]&0x0fffffff)*4:]

	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if leftOn(pin1, pi, pi1) {
		return leftOn(pi, pj, pin1) && leftOn(pj, pi, pi1)
	}
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1))
}

func diagonalLoose(i, j, n int32, verts []int32, indices []int64) bool {
	return inConeLoose(i, j, n, verts, indices) && diagonalieLoose(i, j, n, verts, indices)
}

func triangulate(n int32, verts []int32, indices []int64, tris []int32) int32 {
	var ntris int32
	dst := tris

	// The last bit of the index is used to indicate if the vertex can be removed.
	for i := int32(0); i < n; i++ {
		i1 := next(i, n)
		i2 := next(i1, n)
		if diagonal(i, i2, n, verts, indices) {
			indices[i1] |= 0x80000000
		}
	}

	for n > 3 {
		minLen := int32(-1)
		mini := int32(-1)
		for i := int32(0); i < n; i++ {
			i1 := next(i, n)
			if (indices[i1] & 0x80000000) != 0 {
				p0 := verts[(indices[i]&0x0fffffff)*4:]
				p2 := verts[(indices[next(i1, n)]&0x0fffffff)*4:]

				dx := p2[0] - p0[0]
				dy := p2[2] - p0[2]
				length := dx*dx + dy*dy

				if minLen < 0 || length < minLen {
					minLen = length
					mini = i
				}
			}
		}

		if mini == -1 {
			// We might get here because the contour has overlapping segments, like this:
			//
			//  A o-o=====o---o B
			//   /  |C   D|    \
			//  o   o     o     o
			//  :   :     :     :
			// We'll try to recover by loosing up the inCone test a bit so that a diagonal
			// like A-B or C-D can be found and we can continue.
			minLen = -1
			mini = -1
			for i := int32(0); i < n; i++ {
				i1 := next(i, n)
				i2 := next(i1, n)
				if diagonalLoose(i, i2, n, verts, indices) {
					p0 := verts[(indices[i]&0x0fffffff)*4:]
					p2 := verts[(indices[next(i2, n)]&0x0fffffff)*4:]
					dx := p2[0] - p0[0]
					dy := p2[2] - p0[2]
					length := dx*dx + dy*dy

					if minLen < 0 || length < minLen {
						minLen = length
						mini = i
					}
				}
			}
			if mini == -1 {
				// The contour is messed up. This sometimes happens
				// if the contour simplification is too aggressive.
				return -ntris
			}
		}

		i := mini
		i1 := next(i, n)
		i2 := next(i1, n)

		dst[0] = int32(indices[i] & 0x0fffffff)
		dst[1] = int32(indices[i1] & 0x0fffffff)
		dst[2] = int32(indices[i2] & 0x0fffffff)
		dst = dst[3:]
		ntris++

		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--
		for k := i1; k < n; k++ {

			indices[k] = indices[k+1]
		}

		if i1 >= n {
			i1 = 0
		}
		i = prev(i1, n)
		// Update diagonal flags.
		if diagonal(prev(i, n), i1, n, verts, indices) {
			indices[i] |= 0x80000000
		} else {
			indices[i] &= 0x0fffffff
		}

		if diagonal(i, next(i1, n), n, verts, indices) {
			indices[i1] |= 0x80000000
		} else {
			indices[i1] &= 0x0fffffff
		}
	}

	// Append the remaining triangle.
	dst[0] = int32(indices[0] & 0x0fffffff)
	dst[1] = int32(indices[1] & 0x0fffffff)
	dst[2] = int32(indices[2] & 0x0fffffff)
	dst = dst[3:]

	ntris++
	return ntris
}

func countPolyVerts(p []uint16, nvp int32) int32 {
	for i := int32(0); i < nvp; i++ {
		if p[i] == meshNullIdx {
			return i
		}
	}
	return nvp
}

func uleft(a, b, c []uint16) bool {
	return (int32(b[0])-int32(a[0]))*(int32(c[2])-int32(a[2]))-
		(int32(c[0])-int32(a[0]))*(int32(b[2])-int32(a[2])) < 0
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
func diagonalie(i, j, n int32, verts []int32, indices []int64) bool {
	d0 := verts[(indices[i]&0x0fffffff)*4:]
	d1 := verts[(indices[j]&0x0fffffff)*4:]

	// For each edge (k,k+1) of P
	for k := int32(0); k < n; k++ {
		k1 := next(k, n)
		// Skip edges incident to i or j
		if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
			p0 := verts[(indices[k]&0x0fffffff)*4:]
			p1 := verts[(indices[k1]&0x0fffffff)*4:]

			if vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1) {
				continue
			}

			if intersect(d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}

func getPolyMergeValue(pa, pb []uint16,
	verts []uint16, ea, eb *int32,
	nvp int32) int32 {
	na := countPolyVerts(pa, nvp)
	nb := countPolyVerts(pb, nvp)

	// If the merged polygon would be too big, do not merge.
	if na+nb-2 > nvp {
		return -1
	}

	// Check if the polygons share an edge.
	*ea = -1
	*eb = -1

	for i := int32(0); i < na; i++ {
		va0 := pa[i]
		va1 := pa[(i+1)%na]
		if va0 > va1 {
			va0, va1 = va1, va0
		}
		for j := int32(0); j < nb; j++ {
			vb0 := pb[j]
			vb1 := pb[(j+1)%nb]
			if vb0 > vb1 {
				vb0, vb1 = vb1, vb0
			}
			if va0 == vb0 && va1 == vb1 {
				*ea = i
				*eb = j
				break
			}
		}
	}

	// No common edge, cannot merge.
	if *ea == -1 || *eb == -1 {
		return -1
	}

	// Check to see if the merged polygon would be convex.
	var va, vb, vc uint16

	va = pa[(*ea+na-1)%na]
	vb = pa[*ea]
	vc = pb[(*eb+2)%nb]
	if !uleft(verts[va*3:], verts[vb*3:], verts[vc*3:]) {
		return -1
	}

	va = pb[(*eb+nb-1)%nb]
	vb = pb[*eb]
	vc = pa[(*ea+2)%na]
	if !uleft(verts[va*3:], verts[vb*3:], verts[vc*3:]) {
		return -1
	}

	va = pa[*ea]
	vb = pa[(*ea+1)%na]

	dx := int32(verts[va*3+0]) - int32(verts[vb*3+0])
	dy := int32(verts[va*3+2]) - int32(verts[vb*3+2])

	return dx*dx + dy*dy
}

func mergePolyVerts(pa, pb []uint16, ea, eb int32,
	tmp []uint16, nvp int32) {
	na := countPolyVerts(pa, nvp)
	nb := countPolyVerts(pb, nvp)

	// Merge polygons.
	for i := int32(0); i < nvp; i++ {
		tmp[i] = 0xffff
	}
	var n int32
	// Add pa
	for i := int32(0); i < na-1; i++ {
		tmp[n] = pa[(ea+1+i)%na]
		n++
	}
	// Add pb
	for i := int32(0); i < nb-1; i++ {
		tmp[n] = pb[(eb+1+i)%nb]
		n++
	}

	copy(pa, tmp[:nvp])
}

func pushFront(v int32, arr []int32, an *int32) {
	(*an)++
	for i := (*an) - 1; i > 0; i-- {
		arr[i] = arr[i-1]
	}
	arr[0] = v
}

func pushBack(v int32, arr []int32, an *int32) {
	arr[*an] = v
	(*an)++
}

func canRemoveVertex(ctx *BuildContext, mesh *PolyMesh, rem uint16) bool {
	nvp := mesh.Nvp

	// Count number of polygons to remove.
	var (
		numRemovedVerts   int32
		numTouchedVerts   int32
		numRemainingEdges int32
	)
	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		var numRemoved, numVerts int32
		for j := int32(0); j < nv; j++ {
			if p[j] == rem {
				numTouchedVerts++
				numRemoved++
			}
			numVerts++
		}
		if numRemoved != 0 {
			numRemovedVerts += numRemoved
			numRemainingEdges += numVerts - (numRemoved + 1)
		}
	}

	// There would be too few edges remaining to create a polygon.
	// This can happen for example when a tip of a triangle is marked
	// as deletion, but there are no other polys that share the vertex.
	// In this case, the vertex should not be removed.
	if numRemainingEdges <= 2 {
		return false
	}

	// Find edges which share the removed vertex.
	maxEdges := numTouchedVerts * 2
	var nedges int32
	edges := make([]int32, maxEdges*3)

	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)

		// Collect edges which touches the removed vertex.
		var j int32
		for k := nv - 1; j < nv; k = j {
			if p[j] == rem || p[k] == rem {
				// Arrange edge so that a=rem.
				a, b := p[j], p[k]
				if b == rem {
					a, b = b, a
				}

				// Check if the edge exists
				exists := false
				for m := int32(0); m < nedges; m++ {
					e := edges[m*3:]
					if e[1] == int32(b) {
						// Exists, increment vertex share count.
						e[2]++
						exists = true
					}
				}
				// Add new edge.
				if !exists {
					e := edges[nedges*3:]
					e[0] = int32(a)
					e[1] = int32(b)
					e[2] = 1
					nedges++
				}
			}
			j++
		}
	}

	// There should be no more than 2 open edges.
	// This catches the case that two non-adjacent polygons
	// share the removed vertex. In that case, do not remove the vertex.
	var numOpenEdges int32
	for i := int32(0); i < nedges; i++ {
		if edges[i*3+2] < 2 {
			numOpenEdges++
		}
	}
	if numOpenEdges > 2 {
		return false
	}

	return true
}

func removeVertex(ctx *BuildContext, mesh *PolyMesh, rem uint16, maxTris int32) bool {
	nvp := mesh.Nvp

	// Count number of polygons to remove.
	var numRemovedVerts int32
	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		for j := int32(0); j < nv; j++ {
			if p[j] == rem {
				numRemovedVerts++
			}
		}
	}

	var nedges int32
	edges := make([]int32, numRemovedVerts*nvp*4)

	var nhole int32
	hole := make([]int32, numRemovedVerts*nvp)

	var nhreg int32
	hreg := make([]int32, numRemovedVerts*nvp)

	var nharea int32
	harea := make([]int32, numRemovedVerts*nvp)

	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		hasRem := false
		for j := int32(0); j < nv; j++ {
			if p[j] == rem {
				hasRem = true
			}
		}
		if hasRem {
			// Collect edges which does not touch the removed vertex.
			for j, k := int32(0), nv-1; j < nv; k, j = j, j+1 {
				if p[j] != rem && p[k] != rem {
					e := edges[nedges*4:]
					e[0] = int32(p[k])
					e[1] = int32(p[j])
					e[2] = int32(mesh.Regs[i])
					e[3] = int32(mesh.Areas[i])
					nedges++
				}
			}

			// Remove the polygon.
			p2 := mesh.Polys[(mesh.NPolys-1)*nvp*2:]
			if !compareSlicesUInt16(p, p2) {
				copy(p, p2[:nvp])
			}

			for idx := int32(nvp); idx < nvp; idx++ {
				p[idx] = 0xffff
			}

			mesh.Regs[i] = mesh.Regs[mesh.NPolys-1]
			mesh.Areas[i] = mesh.Areas[mesh.NPolys-1]
			mesh.NPolys--
			i--
		}
	}

	// Remove vertex.
	for i := int32(rem); i < mesh.NVerts-1; i++ {
		mesh.Verts[i*3+0] = mesh.Verts[(i+1)*3+0]
		mesh.Verts[i*3+1] = mesh.Verts[(i+1)*3+1]
		mesh.Verts[i*3+2] = mesh.Verts[(i+1)*3+2]
	}
	mesh.NVerts--

	// Adjust indices to match the removed vertex layout.
	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		for j := int32(0); j < nv; j++ {
			if p[j] > rem {
				p[j]--
			}
		}
	}
	for i := int32(0); i < nedges; i++ {
		if edges[i*4+0] > int32(rem) {
			edges[i*4+0]--
		}
		if edges[i*4+1] > int32(rem) {
			edges[i*4+1]--
		}
	}

	if nedges == 0 {
		return true
	}

	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	pushBack(edges[0], hole, &nhole)
	pushBack(edges[2], hreg, &nhreg)
	pushBack(edges[3], harea, &nharea)

	for nedges != 0 {
		var match bool

		for i := int32(0); i < nedges; i++ {
			ea := edges[i*4+0]
			eb := edges[i*4+1]
			r := edges[i*4+2]
			a := edges[i*4+3]
			var add bool
			if hole[0] == eb {
				// The segment matches the beginning of the hole boundary.
				pushFront(ea, hole, &nhole)
				pushFront(r, hreg, &nhreg)
				pushFront(a, harea, &nharea)
				add = true
			} else if hole[nhole-1] == ea {
				// The segment matches the end of the hole boundary.
				pushBack(eb, hole, &nhole)
				pushBack(r, hreg, &nhreg)
				pushBack(a, harea, &nharea)
				add = true
			}
			if add {
				// The edge segment was added, remove it.
				edges[i*4+0] = edges[(nedges-1)*4+0]
				edges[i*4+1] = edges[(nedges-1)*4+1]
				edges[i*4+2] = edges[(nedges-1)*4+2]
				edges[i*4+3] = edges[(nedges-1)*4+3]
				nedges--
				match = true
				i--
			}
		}

		if !match {
			break
		}
	}

	tris := make([]int32, nhole*3)
	tverts := make([]int32, nhole*4)
	thole := make([]int64, nhole)

	// Generate temp vertex array for triangulation.
	for i := int32(0); i < nhole; i++ {
		pi := hole[i]
		tverts[i*4+0] = int32(mesh.Verts[pi*3+0])
		tverts[i*4+1] = int32(mesh.Verts[pi*3+1])
		tverts[i*4+2] = int32(mesh.Verts[pi*3+2])
		tverts[i*4+3] = 0
		thole[i] = int64(i)
	}

	// Triangulate the hole.
	ntris := triangulate(nhole, tverts[:], thole[:], tris)
	if ntris < 0 {
		ntris = -ntris
		ctx.Warningf("removeVertex: triangulate() returned bad results.")
	}

	// Merge the hole triangles back to polygons.
	polys := make([]uint16, (ntris+1)*nvp)
	pregs := make([]uint16, ntris)
	pareas := make([]uint8, ntris)

	tmpPoly := polys[ntris*nvp:]

	// Build initial polygons.
	var npolys int32
	for i := int32(0); i < ntris*nvp; i++ {
		polys[i] = 0xffff
	}

	for j := int32(0); j < ntris; j++ {
		t := tris[j*3:]
		if t[0] != t[1] && t[0] != t[2] && t[1] != t[2] {
			polys[npolys*nvp+0] = uint16(hole[t[0]])
			polys[npolys*nvp+1] = uint16(hole[t[1]])
			polys[npolys*nvp+2] = uint16(hole[t[2]])

			// If this polygon covers multiple region types then
			// mark it as such
			if hreg[t[0]] != hreg[t[1]] || hreg[t[1]] != hreg[t[2]] {
				pregs[npolys] = multipleRegs
			} else {
				pregs[npolys] = uint16(hreg[t[0]])
			}

			pareas[npolys] = uint8(harea[t[0]])
			npolys++
		}
	}
	if npolys == 0 {
		return true
	}

	// Merge polygons.
	if nvp > 3 {
		for {
			// Find best polygons to merge.
			var (
				bestMergeVal                   int32
				bestPa, bestPb, bestEa, bestEb int32
			)

			for j := int32(0); j < npolys-1; j++ {
				pj := polys[j*nvp:]
				for k := j + 1; k < npolys; k++ {
					pk := polys[k*nvp:]
					var ea, eb int32
					v := getPolyMergeValue(pj, pk, mesh.Verts, &ea, &eb, nvp)
					if v > bestMergeVal {
						bestMergeVal = v
						bestPa = j
						bestPb = k
						bestEa = ea
						bestEb = eb
					}
				}
			}

			if bestMergeVal > 0 {
				// Found best, merge.
				pa := polys[bestPa*nvp:]
				pb := polys[bestPb*nvp:]
				mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp)
				if pregs[bestPa] != pregs[bestPb] {
					pregs[bestPa] = multipleRegs
				}

				last := polys[(npolys-1)*nvp:]
				if !compareSlicesUInt16(pb, last) {
					copy(pb, last[:nvp])
				}
				pregs[bestPb] = pregs[npolys-1]
				pareas[bestPb] = pareas[npolys-1]
				npolys--
			} else {
				// Could not merge any polygons, stop.
				break
			}
		}
	}

	// Store polygons.
	for i := int32(0); i < npolys; i++ {
		if mesh.NPolys >= maxTris {
			break
		}
		p := mesh.Polys[mesh.NPolys*nvp*2:]
		for idx := int32(0); idx < nvp; idx++ {
			p[idx] = 0xffff
		}

		for j := int32(0); j < nvp; j++ {
			p[j] = polys[i*nvp+j]
		}
		mesh.Regs[mesh.NPolys] = pregs[i]
		mesh.Areas[mesh.NPolys] = pareas[i]
		mesh.NPolys++
		if mesh.NPolys > maxTris {
			ctx.Errorf("removeVertex: Too many polygons %d (max:%d).", mesh.NPolys, maxTris)
			return false
		}
	}

	return true
}
