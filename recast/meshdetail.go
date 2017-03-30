package recast

import (
	"fmt"

	assert "github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

// Contains triangle meshes that represent detailed height data associated
// with the polygons in its associated polygon mesh object.
type PolyMeshDetail struct {
	Meshes  []int32   // The sub-mesh data. [Size: 4*#nmeshes]
	Verts   []float32 // The mesh vertices. [Size: 3*#nverts]
	Tris    []uint8   // The mesh triangles. [Size: 4*#ntris]
	NMeshes int32     // The number of sub-meshes defined by #meshes.
	NVerts  int32     // The number of vertices in #verts.
	NTris   int32     // The number of triangles in #tris.
}

func (pmd *PolyMeshDetail) Free() {
	if pmd == nil {
		return
	}
	pmd.Meshes = make([]int32, 0)
	pmd.Verts = make([]float32, 0)
	pmd.Tris = make([]uint8, 0)
	pmd = nil
}

const unsetHeight = 0xffff

type HeightPatch struct {
	data                      []uint16
	xmin, ymin, width, height int32
}

// TODO: AR, all this functions should be taken from Vec3D when possible (or
// added if inexistent)
func vdot2(a, b []float32) float32 {
	return a[0]*b[0] + a[2]*b[2]
}

func vdistSq2(p, q []float32) float32 {
	dx := q[0] - p[0]
	dy := q[2] - p[2]
	return dx*dx + dy*dy
}

func vdist2(p, q []float32) float32 {
	return math32.Sqrt(vdistSq2(p, q))
}

func vcross2(p1, p2, p3 []float32) float32 {
	u1 := p2[0] - p1[0]
	v1 := p2[2] - p1[2]
	u2 := p3[0] - p1[0]
	v2 := p3[2] - p1[2]
	return u1*v2 - v1*u2
}

// TODO: should use d3.Vec3 instead of plain slices
func circumCircle(p1, p2, p3 []float32, c []float32) (r float32, ret bool) {
	const EPS float32 = 1e-6
	// Calculate the circle relative to p1, to avoid some precision issues.
	var v1, v2, v3 [3]float32
	d3.Vec3Sub(v2[:], p2, p1)
	d3.Vec3Sub(v3[:], p3, p1)

	cp := vcross2(v1[:], v2[:], v3[:])
	if math32.Abs(cp) > EPS {
		v1Sq := vdot2(v1[:], v1[:])
		v2Sq := vdot2(v2[:], v2[:])
		v3Sq := vdot2(v3[:], v3[:])
		c[0] = (v1Sq*(v2[2]-v3[2]) + v2Sq*(v3[2]-v1[2]) + v3Sq*(v1[2]-v2[2])) / (2 * cp)
		c[1] = 0
		c[2] = (v1Sq*(v3[0]-v2[0]) + v2Sq*(v1[0]-v3[0]) + v3Sq*(v2[0]-v1[0])) / (2 * cp)
		r = vdist2(c, v1[:])
		d3.Vec3Add(c, c, p1)
		return r, true
	}

	copy(c, p1[:])
	r = 0
	return r, false
}

func distPtTri(p, a, b, c []float32) float32 {
	var v0, v1, v2 [3]float32

	d3.Vec3Sub(v0[:], c, a)
	d3.Vec3Sub(v1[:], b, a)
	d3.Vec3Sub(v2[:], p, a)

	dot00 := vdot2(v0[:], v0[:])
	dot01 := vdot2(v0[:], v1[:])
	dot02 := vdot2(v0[:], v2[:])
	dot11 := vdot2(v1[:], v1[:])
	dot12 := vdot2(v1[:], v2[:])

	// Compute barycentric coordinates
	invDenom := float32(1.0 / (dot00*dot11 - dot01*dot01))
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// If point lies inside the triangle, return interpolated y-coord.
	const EPS float32 = 1e-4
	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		y := a[1] + v0[1]*u + v1[1]*v
		return math32.Abs(y - p[1])
	}
	return math32.MaxFloat32
}

func distancePtSeg3Pt(pt, p, q []float32) float32 {
	pqx := q[0] - p[0]
	pqy := q[1] - p[1]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dy := pt[1] - p[1]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqy*pqy + pqz*pqz
	t := pqx*dx + pqy*dy + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	dx = p[0] + t*pqx - pt[0]
	dy = p[1] + t*pqy - pt[1]
	dz = p[2] + t*pqz - pt[2]

	return dx*dx + dy*dy + dz*dz
}

func distancePtSeg2d(pt, p, q []float32) float32 {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]

	return dx*dx + dz*dz
}

func distToTriMesh(p, verts []float32, nverts int32, tris []int32, ntris int32) float32 {
	dmin := math32.MaxFloat32
	for i := int32(0); i < ntris; i++ {
		va := verts[tris[i*4+0]*3:]
		vb := verts[tris[i*4+1]*3:]
		vc := verts[tris[i*4+2]*3:]
		d := distPtTri(p, va, vb, vc)
		if d < dmin {
			dmin = d
		}
	}
	if dmin == math32.MaxFloat32 {
		return -1
	}
	return dmin
}

func push3(queue *[]int32, v1, v2, v3 int32) {
	*queue = append(*queue, v1, v2, v3)
}

func getHeight(fx, fy, fz, cs, ics, ch float32, radius int32, hp *HeightPatch) uint16 {
	ix := int32(math32.Floor(fx*ics + 0.01))
	iz := int32(math32.Floor(fz*ics + 0.01))
	ix = int32Clamp(ix-hp.xmin, 0, hp.width-1)
	iz = int32Clamp(iz-hp.ymin, 0, hp.height-1)
	h := hp.data[ix+iz*hp.width]
	if uint32(h) == unsetHeight {
		// Special case when data might be bad.
		// Walk adjacent cells in a spiral up to 'radius', and look
		// for a pixel which has a valid height.

		x := int32(1)
		z := int32(0)
		dx := int32(1)
		dz := int32(0)
		maxSize := radius*2 + 1
		maxIter := maxSize*maxSize - 1

		nextRingIterStart := int32(8)
		nextRingIters := int32(16)

		dmin := math32.MaxFloat32
		for i := int32(0); i < maxIter; i++ {
			nx := ix + x
			nz := iz + z

			if nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height {
				nh := hp.data[nx+nz*hp.width]
				if uint32(nh) != unsetHeight {
					d := math32.Abs(float32(nh)*ch - fy)
					if d < dmin {
						h = nh
						dmin = d
					}
				}
			}

			// We are searching in a grid which looks approximately like this:
			//  __________
			// |2 ______ 2|
			// | |1 __ 1| |
			// | | |__| | |
			// | |______| |
			// |__________|
			// We want to find the best height as close to the center cell as
			// possible. This means that if we find a height in one of the
			// neighbor cells to the center, we don't want to expand further out
			// than the 8 neighbors - we want to limit our search to the closest
			// of these "rings", but the best height in the ring.
			// For example, the center is just 1 cell. We checked that at the
			// entrance to the function.
			// The next "ring" contains 8 cells (marked 1 above). Those are all
			// the neighbors to the center cell.
			// The next one again contains 16 cells (marked 2). In general each
			// ring has 8 additional cells, which can be thought of as adding 2
			// cells around the "center" of each side when we expand the ring.
			// Here we detect if we are about to enter the next ring, and if we
			// are and we have found a height, we abort the search.
			if i+1 == nextRingIterStart {
				if uint32(h) != unsetHeight {
					break
				}

				nextRingIterStart += nextRingIters
				nextRingIters += 8
			}

			if (x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1-z)) {
				tmp := dx
				dx = -dz
				dz = tmp
			}
			x += dx
			z += dz
		}
	}
	return h
}

// enum EdgeValues
const (
	EV_UNDEF int32 = -1
	EV_HULL  int32 = -2
)

func findEdge(edges []int32, nedges int32, s, t int32) int32 {
	for i := int32(0); i < nedges; i++ {
		e := edges[i*4:]
		if (e[0] == s && e[1] == t) || (e[0] == t && e[1] == s) {
			return i
		}
	}
	return EV_UNDEF
}

func addEdge(ctx *BuildContext, edges []int32, nedges *int32, maxEdges, s, t, l, r int32) int32 {
	if *nedges >= maxEdges {
		ctx.Errorf("addEdge: Too many edges (%d/%d).", *nedges, maxEdges)
		return EV_UNDEF
	}

	// Add edge if not already in the triangulation.
	e := findEdge(edges, *nedges, s, t)
	if e == EV_UNDEF {
		edge := edges[*nedges*4:]
		edge[0] = s
		edge[1] = t
		edge[2] = l
		edge[3] = r
		*nedges++
		return *nedges
	} else {
		return EV_UNDEF
	}
}

func getEdgeFlags(va, vb, vpoly []float32, npoly int32) uint8 {
	// Return true if edge (va,vb) is part of the polygon.
	thrSqr := float32(0.001 * 0.001)
	var i int32
	for j := int32(npoly - 1); i < npoly; i++ {
		if distancePtSeg2d(va, vpoly[j*3:], vpoly[i*3:]) < thrSqr &&
			distancePtSeg2d(vb, vpoly[j*3:], vpoly[i*3:]) < thrSqr {
			return 1
		}
		j = i
	}
	return 0
}

func getTriFlags(va, vb, vc, vpoly []float32, npoly int32) uint8 {
	var flags uint8
	flags |= getEdgeFlags(va, vb, vpoly, npoly) << 0
	flags |= getEdgeFlags(vb, vc, vpoly, npoly) << 2
	flags |= getEdgeFlags(vc, va, vpoly, npoly) << 4
	return flags
}

// BuildPolyMeshDetail builds a detail mesh from the provided polygon mesh.
//
//  Arguments:
//  ctx             The build context to use during the operation.
//  meshs           A fully built polygon mesh.
//  chf             The compact heightfield used to build the polygon mesh.
//  sampleDist      Sets the distance to use when samping the heightfield.
//                  [Limit: >=0] [Units: wu]
//  sampleMaxError  The maximum distance the detail mesh surface should deviate
//                  from heightfield data. [Limit: >=0] [Units: wu]
//  dmesh           The resulting detail mesh. (Must be pre-allocated.)
//
// Returns True if the operation completed successfully.
// See the Config documentation for more information on the configuration
// parameters.
//
// see AllocPolyMeshDetail, PolyMesh, CompactHeightfield, PolyMeshDetail, Config
func BuildPolyMeshDetail(ctx *BuildContext, mesh *PolyMesh, chf *CompactHeightfield, sampleDist, sampleMaxError float32) (*PolyMeshDetail, bool) {
	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerBuildPolyMeshDetail)
	defer ctx.StopTimer(TimerBuildPolyMeshDetail)

	var dmesh PolyMeshDetail
	if mesh.NVerts == 0 || mesh.NPolys == 0 {
		return &dmesh, true
	}

	nvp := mesh.Nvp
	cs := mesh.Cs
	ch := mesh.Ch
	orig := mesh.BMin
	borderSize := mesh.BorderSize
	heightSearchRadius := iMax(1, int32(math32.Ceil(mesh.MaxEdgeError)))

	edges := make([]int32, 64)
	tris := make([]int32, 512)
	arr := make([]int32, 512)
	samples := make([]int32, 512)

	var (
		verts        []float32
		hp           HeightPatch
		nPolyVerts   int32
		maxhw, maxhh int32
	)
	verts = make([]float32, 256*3)

	bounds := make([]int32, mesh.NPolys*4)
	poly := make([]float32, nvp*3)

	// Find max size for a polygon area.
	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		xmin := &bounds[i*4+0]
		xmax := &bounds[i*4+1]
		ymin := &bounds[i*4+2]
		ymax := &bounds[i*4+3]

		*xmin = chf.Width
		*xmax = 0
		*ymin = chf.Height
		*ymax = 0
		for j := int32(0); j < nvp; j++ {
			if p[j] == meshNullIdx {
				break
			}
			v := mesh.Verts[p[j]*3:]
			*xmin = iMin(*xmin, int32(v[0]))
			*xmax = iMax(*xmax, int32(v[0]))
			*ymin = iMin(*ymin, int32(v[2]))
			*ymax = iMax(*ymax, int32(v[2]))
			nPolyVerts++
		}
		*xmin = iMax(0, *xmin-1)
		*xmax = iMin(chf.Width, *xmax+1)
		*ymin = iMax(0, *ymin-1)
		*ymax = iMin(chf.Height, *ymax+1)
		if *xmin >= *xmax || *ymin >= *ymax {
			continue
		}
		maxhw = iMax(maxhw, *xmax-*xmin)
		maxhh = iMax(maxhh, *ymax-*ymin)
	}

	hp.data = make([]uint16, maxhw*maxhh)

	dmesh.NMeshes = mesh.NPolys
	dmesh.NVerts = 0
	dmesh.NTris = 0
	dmesh.Meshes = make([]int32, dmesh.NMeshes*4)

	vcap := nPolyVerts + nPolyVerts/2
	tcap := vcap * 2

	dmesh.NVerts = 0
	dmesh.Verts = make([]float32, vcap*3)
	dmesh.Tris = make([]uint8, tcap*4)

	for i := int32(0); i < mesh.NPolys; i++ {
		p := mesh.Polys[i*nvp*2:]

		// Store polygon vertices for processing.
		var npoly int32
		for j := int32(0); j < nvp; j++ {
			if p[j] == meshNullIdx {
				break
			}
			v := mesh.Verts[p[j]*3:]
			poly[j*3+0] = float32(v[0]) * cs
			poly[j*3+1] = float32(v[1]) * ch
			poly[j*3+2] = float32(v[2]) * cs
			npoly++
		}

		// Get the height data from the area of the polygon.
		hp.xmin = bounds[i*4+0]
		hp.ymin = bounds[i*4+2]
		hp.width = bounds[i*4+1] - bounds[i*4+0]
		hp.height = bounds[i*4+3] - bounds[i*4+2]
		getHeightData(ctx, chf, p, npoly, mesh.Verts, borderSize, &hp, &arr, int32(mesh.Regs[i]))

		// Build detail mesh.
		var nverts int32
		if !buildPolyDetail(ctx, poly, npoly,
			sampleDist, sampleMaxError,
			heightSearchRadius, chf, &hp,
			verts, &nverts, &tris,
			&edges, &samples) {
			return nil, false
		}

		// Move detail verts to world space.
		for j := int32(0); j < nverts; j++ {
			verts[j*3+0] += orig[0]
			verts[j*3+1] += orig[1] + chf.Ch // Is this offset necessary?
			verts[j*3+2] += orig[2]
		}
		// Offset poly too, will be used to flag checking.
		for j := int32(0); j < npoly; j++ {
			poly[j*3+0] += orig[0]
			poly[j*3+1] += orig[1]
			poly[j*3+2] += orig[2]
		}

		// Store detail submesh.
		ntris := int32(len(tris) / 4)

		dmesh.Meshes[i*4+0] = dmesh.NVerts
		dmesh.Meshes[i*4+1] = nverts
		dmesh.Meshes[i*4+2] = dmesh.NTris
		dmesh.Meshes[i*4+3] = ntris

		// Store vertices, allocate more memory if necessary.
		if dmesh.NVerts+nverts > vcap {
			for dmesh.NVerts+nverts > vcap {
				vcap += 256
			}

			newv := make([]float32, vcap*3)
			if dmesh.NVerts != 0 {
				copy(newv, dmesh.Verts[:3*dmesh.NVerts])
			}
			dmesh.Verts = newv
		}
		for j := int32(0); j < nverts; j++ {
			dmesh.Verts[dmesh.NVerts*3+0] = verts[j*3+0]
			dmesh.Verts[dmesh.NVerts*3+1] = verts[j*3+1]
			dmesh.Verts[dmesh.NVerts*3+2] = verts[j*3+2]
			dmesh.NVerts++
		}

		// Store triangles, allocate more memory if necessary.
		if dmesh.NTris+ntris > tcap {
			for dmesh.NTris+ntris > tcap {
				tcap += 256
			}
			newt := make([]uint8, tcap*4)
			if dmesh.NTris != 0 {
				copy(newt, dmesh.Tris[:4*dmesh.NTris])
			}
			dmesh.Tris = newt
		}
		for j := int32(0); j < ntris; j++ {
			t := tris[j*4:]
			dmesh.Tris[dmesh.NTris*4+0] = uint8(t[0])
			dmesh.Tris[dmesh.NTris*4+1] = uint8(t[1])
			dmesh.Tris[dmesh.NTris*4+2] = uint8(t[2])
			dmesh.Tris[dmesh.NTris*4+3] = getTriFlags(verts[t[0]*3:], verts[t[1]*3:], verts[t[2]*3:], poly, npoly)
			dmesh.NTris++
		}
	}

	return &dmesh, true
}

func updateLeftFace(e []int32, s, t, f int32) {
	if e[0] == s && e[1] == t && e[2] == EV_UNDEF {
		e[2] = f
	} else if e[1] == s && e[0] == t && e[3] == EV_UNDEF {
		e[3] = f
	}
}

func overlapSegSeg2d(a, b, c, d []float32) bool {
	a1 := vcross2(a, b, d)
	a2 := vcross2(a, b, c)
	if a1*a2 < 0.0 {
		a3 := vcross2(c, d, a)
		a4 := a3 + a2 - a1
		if a3*a4 < 0.0 {
			return true
		}
	}
	return false
}

func overlapEdges(pts []float32, edges []int32, nedges, s1, t1 int32) bool {
	for i := int32(0); i < nedges; i++ {
		s0 := edges[i*4+0]
		t0 := edges[i*4+1]
		// Same or connected edges do not overlap.
		if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
			continue
		}
		if overlapSegSeg2d(pts[s0*3:], pts[t0*3:], pts[s1*3:], pts[t1*3:]) {
			return true
		}
	}
	return false
}

func completeFacet(ctx *BuildContext, pts []float32, npts int32, edges []int32, nedges *int32, maxEdges int32, nfaces *int32, e int32) {
	const EPS float32 = 1e-5

	edge := edges[e*4:]

	// Cache s and t.
	var s, t int32
	if edge[2] == EV_UNDEF {
		s = edge[0]
		t = edge[1]
	} else if edge[3] == EV_UNDEF {
		s = edge[1]
		t = edge[0]
	} else {
		// Edge already completed.
		return
	}

	// Find best point on left of edge.
	pt := npts
	var c [3]float32
	r := float32(-1)
	for u := int32(0); u < npts; u++ {
		if u == s || u == t {
			continue
		}
		if vcross2(pts[s*3:], pts[t*3:], pts[u*3:]) > EPS {
			if r < 0 {
				// The circle is not updated yet, do it now.
				pt = u
				r, _ = circumCircle(pts[s*3:], pts[t*3:], pts[u*3:], c[:])
				continue
			}
			d := vdist2(c[:], pts[u*3:])
			tol := float32(0.001)
			if d > r*(1+tol) {
				// Outside current circumcircle, skip.
				continue
			} else if d < r*(1-tol) {
				// Inside safe circumcircle, update circle.
				pt = u
				r, _ = circumCircle(pts[s*3:], pts[t*3:], pts[u*3:], c[:])
			} else {
				// Inside epsilon circum circle, do extra tests to make sure the edge is valid.
				// s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
				if overlapEdges(pts, edges, *nedges, s, u) {
					continue
				}
				if overlapEdges(pts, edges, *nedges, t, u) {
					continue
				}
				// Edge is valid.
				pt = u
				r, _ = circumCircle(pts[s*3:], pts[t*3:], pts[u*3:], c[:])
			}
		}
	}

	// Add new triangle or update edge info if s-t is on hull.
	if pt < npts {
		// Update face information of edge being completed.
		updateLeftFace(edges[e*4:], s, t, *nfaces)

		// Add new edge or update face info of old edge.
		e = findEdge(edges, *nedges, pt, s)
		if e == EV_UNDEF {
			addEdge(ctx, edges, nedges, maxEdges, pt, s, *nfaces, EV_UNDEF)
		} else {
			updateLeftFace(edges[e*4:], pt, s, *nfaces)
		}

		// Add new edge or update face info of old edge.
		e = findEdge(edges, *nedges, t, pt)
		if e == EV_UNDEF {
			addEdge(ctx, edges, nedges, maxEdges, t, pt, *nfaces, EV_UNDEF)
		} else {
			updateLeftFace(edges[e*4:], t, pt, *nfaces)
		}

		*nfaces++
	} else {
		updateLeftFace(edges[e*4:], s, t, EV_HULL)
	}
}

func delaunayHull(ctx *BuildContext, npts int32, pts []float32,
	nhull int32, hull []int32,
	tris, edges *[]int32) {
	var (
		nfaces, nedges int32
	)
	maxEdges := npts * 10
	*edges = make([]int32, maxEdges*4)

	var i int32
	for j := nhull - 1; i < nhull; i++ {
		addEdge(ctx, (*edges)[:], &nedges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF)
		j = i
	}

	var currentEdge int32
	for currentEdge < nedges {
		if (*edges)[currentEdge*4+2] == EV_UNDEF {
			completeFacet(ctx, pts, npts, *edges, &nedges, maxEdges, &nfaces, currentEdge)
		}
		if (*edges)[currentEdge*4+3] == EV_UNDEF {
			completeFacet(ctx, pts, npts, *edges, &nedges, maxEdges, &nfaces, currentEdge)
		}
		currentEdge++
	}

	// Create tris
	*tris = make([]int32, nfaces*4)
	for i := int32(0); i < nfaces*4; i++ {
		(*tris)[i] = -1
	}

	for i := int32(0); i < nedges; i++ {
		e := (*edges)[i*4:]
		if e[3] >= 0 {
			// Left face
			t := (*tris)[e[3]*4:]
			if t[0] == -1 {
				t[0] = e[0]
				t[1] = e[1]
			} else if t[0] == e[1] {
				t[2] = e[0]
			} else if t[1] == e[0] {
				t[2] = e[1]
			}
		}
		if e[2] >= 0 {
			// Right
			t := (*tris)[e[2]*4:]
			if t[0] == -1 {
				t[0] = e[1]
				t[1] = e[0]
			} else if t[0] == e[0] {
				t[2] = e[1]
			} else if t[1] == e[1] {
				t[2] = e[0]
			}
		}
	}

	for i := 0; i < len(*tris)/4; i++ {
		t := (*tris)[i*4:]
		if t[0] == -1 || t[1] == -1 || t[2] == -1 {
			ctx.Warningf("delaunayHull: Removing dangling face %d [%d,%d,%d].", i, t[0], t[1], t[2])
			panic("untested")
			// TODO: simplify
			t[0] = (*tris)[len(*tris)-4]
			t[1] = (*tris)[len(*tris)-3]
			t[2] = (*tris)[len(*tris)-2]
			t[3] = (*tris)[len(*tris)-1]
			*tris = append([]int32{}, (*tris)[:len(*tris)-4]...)
			i--
		}
	}
}

// Calculate minimum extend of the polygon.
func polyMinExtent(verts []float32, nverts int32) float32 {
	minDist := math32.MaxFloat32
	for i := int32(0); i < nverts; i++ {
		ni := (i + 1) % nverts
		p1 := verts[i*3:]
		p2 := verts[ni*3:]
		var maxEdgeDist float32
		for j := int32(0); j < nverts; j++ {
			if j == i || j == ni {
				continue
			}
			d := distancePtSeg2d(verts[j*3:], p1, p2)
			if d > maxEdgeDist {
				maxEdgeDist = d
			}
		}
		if maxEdgeDist < minDist {
			minDist = maxEdgeDist
		}
	}
	return math32.Sqrt(minDist)
}

func triangulateHull(nverts int32, verts []float32, nhull int32, hull []int32, tris *[]int32) {
	start := int32(0)
	left := int32(1)
	right := nhull - 1

	// Start from an ear with shortest perimeter.
	// This tends to favor well formed triangles as starting point.
	var dmin float32
	for i := int32(0); i < nhull; i++ {
		pi := prev(i, nhull)
		ni := next(i, nhull)
		pv := verts[hull[pi]*3:]
		cv := verts[hull[i]*3:]
		nv := verts[hull[ni]*3:]
		d := vdist2(pv, cv) + vdist2(cv, nv) + vdist2(nv, pv)
		if d < dmin {
			start = i
			left = ni
			right = pi
			dmin = d
		}
	}

	// Add first triangle
	*tris = append(*tris, hull[start])
	*tris = append(*tris, hull[left])
	*tris = append(*tris, hull[right])
	*tris = append(*tris, 0)

	// Triangulate the polygon by moving left or right,
	// depending on which triangle has shorter perimeter.
	// This heuristic was chose emprically, since it seems
	// handle tesselated straight edges well.
	for next(left, nhull) != right {
		// Check to see if se should advance left or right.
		nleft := next(left, nhull)
		nright := prev(right, nhull)

		cvleft := verts[hull[left]*3:]
		nvleft := verts[hull[nleft]*3:]
		cvright := verts[hull[right]*3:]
		nvright := verts[hull[nright]*3:]
		dleft := vdist2(cvleft, nvleft) + vdist2(nvleft, cvright)
		dright := vdist2(cvright, nvright) + vdist2(cvleft, nvright)

		if dleft < dright {
			*tris = append(*tris, hull[left], hull[nleft], hull[right], 0)
			left = nleft
		} else {
			*tris = append(*tris, hull[left], hull[nright], hull[right], 0)
			right = nright
		}
	}
}

func jitterX(i int64) float32 {
	return (float32((i*0x8da6b343)&0xffff) / float32(65535.0) * float32(2.0)) - float32(1.0)
}

func jitterY(i int64) float32 {
	return (float32((i*0xd8163841)&0xffff) / float32(65535.0) * float32(2.0)) - float32(1.0)
}

func buildPolyDetail(ctx *BuildContext, in []float32, nin int32,
	sampleDist, sampleMaxError float32,
	heightSearchRadius int32, chf *CompactHeightfield,
	hp *HeightPatch, verts []float32, nverts *int32,
	tris, edges, samples *[]int32) bool {
	const (
		MAX_VERTS          = 127
		MAX_TRIS           = 255 // Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
		MAX_VERTS_PER_EDGE = 32
	)

	var edge [(MAX_VERTS_PER_EDGE + 1) * 3]float32
	var hull [MAX_VERTS]int32
	var nhull int32

	*nverts = nin

	for i := int32(0); i < nin; i++ {
		copy(verts[i*3:], in[i*3:3+i*3])
	}

	*edges = make([]int32, 0)
	*tris = make([]int32, 0)

	cs := chf.Cs
	ics := 1.0 / cs

	// Calculate minimum extents of the polygon based on input data.
	minExtent := polyMinExtent(verts, *nverts)

	// Tessellate outlines.
	// This is done in separate pass in order to ensure
	// seamless height values across the ply boundaries.
	if sampleDist > 0 {
		i := int32(0)
		for j := nin - 1; i < nin; i++ {
			vj := in[j*3:]
			vi := in[i*3:]
			var swapped bool
			// Make sure the segments are always handled in same order
			// using lexological sort or else there will be seams.
			if math32.Abs(vj[0]-vi[0]) < 1e-6 {
				if vj[2] > vi[2] {
					vj, vi = vi, vj
					swapped = true
				}
			} else {
				if vj[0] > vi[0] {
					vj, vi = vi, vj
					swapped = true
				}
			}
			// Create samples along the edge.
			dx := vi[0] - vj[0]
			dy := vi[1] - vj[1]
			dz := vi[2] - vj[2]
			d := math32.Sqrt(dx*dx + dz*dz)
			nn := 1 + int32(math32.Floor(d/sampleDist))
			if nn >= MAX_VERTS_PER_EDGE {
				nn = MAX_VERTS_PER_EDGE - 1
			}
			if *nverts+nn >= MAX_VERTS {
				nn = MAX_VERTS - 1 - *nverts
			}

			for k := int32(0); k <= nn; k++ {
				u := float32(k) / float32(nn)
				pos := edge[k*3:]
				pos[0] = vj[0] + dx*u
				pos[1] = vj[1] + dy*u
				pos[2] = vj[2] + dz*u
				pos[1] = float32(getHeight(pos[0], pos[1], pos[2], cs, ics, chf.Ch, heightSearchRadius, hp)) * chf.Ch
			}
			// Simplify samples.
			idx := [MAX_VERTS_PER_EDGE]int32{0, nn}
			nidx := int32(2)
			for k := int32(0); k < nidx-1; {
				a := idx[k]
				b := idx[k+1]
				va := edge[a*3:]
				vb := edge[b*3:]
				// Find maximum deviation along the segment.
				var maxd float32
				maxi := int32(-1)
				for m := a + 1; m < b; m++ {
					dev := distancePtSeg3Pt(edge[m*3:], va, vb)
					if dev > maxd {
						maxd = dev
						maxi = m
					}
				}
				// If the max deviation is larger than accepted error,
				// add new point, else continue to next segment.
				if maxi != -1 && maxd > math32.Sqr(sampleMaxError) {
					for m := nidx; m > k; m-- {
						idx[m] = idx[m-1]
					}
					idx[k+1] = maxi
					nidx++
				} else {
					k++
				}
			}

			hull[nhull] = j
			nhull++
			// Add new vertices.
			if swapped {
				for k := nidx - 2; k > 0; k-- {
					copy(verts[*nverts*3:], edge[idx[k]*3:3+idx[k]*3])
					hull[nhull] = *nverts
					nhull++
					*nverts++
				}
			} else {
				for k := int32(1); k < nidx-1; k++ {
					copy(verts[*nverts*3:], edge[idx[k]*3:3+(idx[k]*3)])
					hull[nhull] = *nverts
					nhull++
					*nverts++
				}
			}
			j = i
		}
	}

	// If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
	if minExtent < sampleDist*2 {
		triangulateHull(*nverts, verts, nhull, hull[:], tris)
		return true
	}

	// Tessellate the base mesh.
	// We're using the triangulateHull instead of delaunayHull as it tends to
	// create a bit better triangulation for long thin triangles when there
	// are no internal points.
	triangulateHull(*nverts, verts, nhull, hull[:], tris)

	if len(*tris) == 0 {
		// Could not triangulate the poly, make sure there is some valid data there.
		ctx.Warningf("buildPolyDetail: Could not triangulate polygon (%d verts).", nverts)
		return true
	}

	if sampleDist > 0 {
		// Create sample locations in a grid.
		var bmin, bmax [3]float32
		copy(bmin[:], in)
		copy(bmax[:], in)
		for i := int32(1); i < nin; i++ {
			d3.Vec3Min(bmin[:], in[i*3:])
			d3.Vec3Max(bmax[:], in[i*3:])
		}
		x0 := int32(math32.Floor(bmin[0] / sampleDist))
		x1 := int32(math32.Ceil(bmax[0] / sampleDist))
		z0 := int32(math32.Floor(bmin[2] / sampleDist))
		z1 := int32(math32.Ceil(bmax[2] / sampleDist))
		*samples = make([]int32, 0)
		for z := z0; z < z1; z++ {
			for x := x0; x < x1; x++ {
				var pt [3]float32
				pt[0] = float32(x) * sampleDist
				pt[1] = (bmax[1] + bmin[1]) * 0.5
				pt[2] = float32(z) * sampleDist
				// Make sure the samples are not too close to the edges.
				if distToPoly(nin, in, pt[:]) > -sampleDist/2 {
					continue
				}
				*samples = append(*samples,
					x,
					int32(getHeight(pt[0], pt[1], pt[2], cs, ics, chf.Ch, heightSearchRadius, hp)),
					z,
					0)
			}
		}

		// Add the samples starting from the one that has the most
		// error. The procedure stops when all samples are added
		// or when the max error is within treshold.
		nsamples := int32(len(*samples) / 4)
		for iter := int32(0); iter < nsamples; iter++ {
			if *nverts >= MAX_VERTS {
				break
			}

			// Find sample with most error.
			var (
				bestpt [3]float32
				bestd  float32
			)
			besti := int32(-1)
			for i := int32(0); i < nsamples; i++ {
				s := (*samples)[i*4:]
				if s[3] != 0 {
					continue // skip added.
				}
				var pt [3]float32
				// The sample location is jittered to get rid of some bad triangulations
				// which are cause by symmetrical data from the grid structure.
				pt[0] = float32(s[0])*sampleDist + jitterX(int64(i))*cs*0.1
				pt[1] = float32(s[1]) * chf.Ch
				pt[2] = float32(s[2])*sampleDist + jitterY(int64(i))*cs*0.1
				d := distToTriMesh(pt[:], verts, *nverts, *tris, int32(len(*tris)/4))
				if d < 0 {
					continue // did not hit the mesh.
				}
				if d > bestd {
					bestd = d
					besti = i
					copy(bestpt[:], pt[:])
				}
			}

			// If the max error is within accepted threshold, stop tesselating.
			if bestd <= sampleMaxError || besti == -1 {
				break
			}
			// Mark sample as added.
			(*samples)[besti*4+3] = 1
			// Add the new sample point.
			copy(verts[*nverts*3:], bestpt[:])
			*nverts++

			// Create new triangulation.
			// TODO: Incremental add instead of full rebuild.
			*edges = make([]int32, 0)
			*tris = make([]int32, 0)
			delaunayHull(ctx, *nverts, verts, nhull, hull[:], tris, edges)
		}
	}

	ntris := len(*tris) / 4
	if ntris > MAX_TRIS {
		panic("untested")
		//tris.resize(MAX_TRIS*4);
		*tris = make([]int32, MAX_TRIS*4)
		ctx.Errorf("rcBuildPolyMeshDetail: Shrinking triangle count from %d to max %d.", ntris, MAX_TRIS)
	}

	return true
}

var bsOffset [9 * 2]int32

func init() {
	// Note: Reads to the compact heightfield are offset by border size (bs)
	// since border size offset is already removed from the polymesh vertices.
	bsOffset = [9 * 2]int32{0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0}
}

func seedArrayWithPolyCenter(ctx *BuildContext, chf *CompactHeightfield,
	poly []uint16, npoly int32,
	verts []uint16, bs int32,
	hp *HeightPatch, array *[]int32) {

	// Find cell closest to a poly vertex
	var (
		startCellX, startCellY int32
		startSpanIndex         int32 = -1
		dmin                   int32 = unsetHeight
	)
	for j := int32(0); j < npoly && dmin > 0; j++ {
		for k := int32(0); k < 9 && dmin > 0; k++ {
			ax := int32(verts[poly[j]*3+0]) + bsOffset[k*2+0]
			ay := int32(verts[poly[j]*3+1])
			az := int32(verts[poly[j]*3+2]) + bsOffset[k*2+1]
			if ax < hp.xmin || ax >= hp.xmin+hp.width ||
				az < hp.ymin || az >= hp.ymin+hp.height {
				continue
			}

			c := &chf.Cells[(ax+bs)+(az+bs)*chf.Width]
			for i, ni := int32(c.Index), int32(c.Index)+int32(c.Count); i < ni && dmin > 0; i++ {
				d := iAbs(ay - int32(chf.Spans[i].Y))
				if d < dmin {
					startCellX = ax
					startCellY = az
					startSpanIndex = i
					dmin = d
				}
			}
		}
	}

	if startSpanIndex == -1 {
		panic(fmt.Sprintf("startSpanIndex should be != 1, got %v", startSpanIndex))
	}

	// Find center of the polygon
	var pcx, pcy int32
	for j := int32(0); j < npoly; j++ {
		pcx += int32(verts[poly[j]*3+0])
		pcy += int32(verts[poly[j]*3+2])
	}
	pcx /= npoly
	pcy /= npoly

	// Use seeds array as a stack for DFS
	*array = append([]int32{}, startCellX, startCellY, startSpanIndex)

	dirs := []int32{0, 1, 2, 3}
	for i := int32(0); i < hp.width*hp.height; i++ {
		hp.data[i] = 0
	}

	// DFS to move to the center. Note that we need a DFS here and can not just move
	// directly towards the center without recording intermediate nodes, even though the polygons
	// are convex. In very rare we can get stuck due to contour simplification if we do not
	// record nodes.
	cx := int32(-1)
	cy := int32(-1)
	ci := int32(-1)
	for {
		if len(*array) < 3 {
			ctx.Warningf("Walk towards polygon center failed to reach center")
			break
		}

		// pop last 3 elements from the slice
		ci, *array = (*array)[len(*array)-1], (*array)[:len(*array)-1]
		cy, *array = (*array)[len(*array)-1], (*array)[:len(*array)-1]
		cx, *array = (*array)[len(*array)-1], (*array)[:len(*array)-1]

		if cx == pcx && cy == pcy {
			break
		}

		// If we are already at the correct X-position, prefer direction
		// directly towards the center in the Y-axis; otherwise prefer
		// direction in the X-axis
		var directDir, off int32
		if cx == pcx {
			if pcy > cy {
				off = 1
			} else {
				off = -1
			}
			directDir = GetDirForOffset(0, off)
		} else {
			if pcx > cx {
				off = 1
			} else {
				off = -1
			}
			directDir = GetDirForOffset(off, 0)
		}

		// Push the direct dir last so we start with this on next iteration
		//rcSwap(dirs[directDir], dirs[3]);
		dirs[directDir], dirs[3] = dirs[3], dirs[directDir]

		cs := &chf.Spans[ci]
		for i := int32(0); i < int32(4); i++ {
			dir := dirs[i]
			if GetCon(cs, dir) == notConnected {
				continue
			}

			newX := cx + GetDirOffsetX(dir)
			newY := cy + GetDirOffsetY(dir)

			hpx := newX - hp.xmin
			hpy := newY - hp.ymin
			if hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height {
				continue
			}

			if hp.data[hpx+hpy*hp.width] != 0 {
				continue
			}

			hp.data[hpx+hpy*hp.width] = 1
			*array = append(*array,
				newX,
				newY,
				int32(chf.Cells[(newX+bs)+(newY+bs)*chf.Width].Index)+GetCon(cs, dir))
		}

		dirs[directDir], dirs[3] = dirs[3], dirs[directDir]
	}

	// getHeightData seeds are given in coordinates with borders
	*array = append([]int32{}, cx+bs, cy+bs, ci)

	for i := int32(0); i < hp.width*hp.height; i++ {
		hp.data[i] = 0xffff
	}

	hp.data[cx-hp.xmin+(cy-hp.ymin)*hp.width] = chf.Spans[ci].Y
}

func distToPoly(nvert int32, verts, p []float32) float32 {
	dmin := math32.MaxFloat32
	var (
		i, j int32
		c    bool
	)

	for j = nvert - 1; i < nvert; i++ {
		vi := verts[i*3:]
		vj := verts[j*3:]
		if ((vi[2] > p[2]) != (vj[2] > p[2])) &&
			(p[0] < (vj[0]-vi[0])*(p[2]-vi[2])/(vj[2]-vi[2])+vi[0]) {
			c = !c
		}
		//dmin = rcMin(dmin, distancePtSeg2d(p, vj, vi));
		d := distancePtSeg2d(p, vj, vi)
		if d < dmin {
			dmin = d
		}
		j = i
	}
	if c {
		return -dmin
	}
	return dmin
}

func getHeightData(ctx *BuildContext, chf *CompactHeightfield,
	poly []uint16, npoly int32,
	verts []uint16, bs int32,
	hp *HeightPatch, queue *[]int32,
	region int32) {
	// Note: Reads to the compact heightfield are offset by border size (bs)
	// since border size offset is already removed from the polymesh vertices.
	*queue = make([]int32, 0)
	// Set all heights to RC_UNSET_HEIGHT.
	//memset(hp.data, 0xff, sizeof(unsigned short)*hp.width*hp.height);
	for i := int32(0); i < hp.width*hp.height; i++ {
		hp.data[i] = 0xffff
	}

	empty := true

	// We cannot sample from this poly if it was created from polys of different
	// regions. If it was then it could potentially be overlapping with polys of
	// that region and the heights sampled here could be wrong.
	if region != int32(multipleRegs) {
		// Copy the height from the same region, and mark region borders
		// as seed points to fill the rest.
		for hy := int32(0); hy < hp.height; hy++ {
			y := hp.ymin + hy + bs
			for hx := int32(0); hx < hp.width; hx++ {
				x := hp.xmin + hx + bs
				c := &chf.Cells[x+y*chf.Width]

				for i, ni := int32(c.Index), int32(c.Index+uint32(c.Count)); i < ni; i++ {
					s := &chf.Spans[i]
					if int32(s.Reg) == region {
						// Store height
						hp.data[hx+hy*hp.width] = s.Y
						empty = false

						// If any of the neighbours is not in same region,
						// add the current location as flood fill start
						var border bool
						for dir := int32(0); dir < 4; dir++ {
							if GetCon(s, dir) != notConnected {
								ax := x + GetDirOffsetX(dir)
								ay := y + GetDirOffsetY(dir)
								ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(s, dir)
								if int32(chf.Spans[ai].Reg) != region {
									border = true
									break
								}
							}
						}
						if border {
							push3(queue, x, y, i)
						}
						break
					}
				}
			}
		}
	}

	// if the polygon does not contain any points from the current region (rare,
	// but happens) or if it could potentially be overlapping polygons of the
	// same region, then use the center as the seed point.
	if empty {
		seedArrayWithPolyCenter(ctx, chf, poly, npoly, verts, bs, hp, queue)
	}

	const RETRACT_SIZE = 256
	var head int

	// We assume the seed is centered in the polygon, so a BFS to collect height
	// data will ensure we do not move onto overlapping polygons and sample
	// wrong heights.
	for head*3 < len(*queue) {
		cx := (*queue)[head*3+0]
		cy := (*queue)[head*3+1]
		ci := (*queue)[head*3+2]
		head++
		if head >= RETRACT_SIZE {
			head = 0
			if len(*queue) > RETRACT_SIZE*3 {
				copy((*queue)[:], (*queue)[RETRACT_SIZE*3:(RETRACT_SIZE*3)+len(*queue)-RETRACT_SIZE*3])
			}
			*queue = (*queue)[:len(*queue)-RETRACT_SIZE*3]
		}

		cs := &chf.Spans[ci]
		for dir := int32(0); dir < 4; dir++ {
			if GetCon(cs, dir) == notConnected {
				continue
			}

			ax := cx + GetDirOffsetX(dir)
			ay := cy + GetDirOffsetY(dir)
			hx := ax - hp.xmin - bs
			hy := ay - hp.ymin - bs

			if uint(hx) >= uint(hp.width) || uint(hy) >= uint(hp.height) {
				continue
			}

			if uint32(hp.data[hx+hy*hp.width]) != unsetHeight {
				continue
			}

			ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(cs, dir)

			hp.data[hx+hy*hp.width] = chf.Spans[ai].Y

			push3(queue, ax, ay, ai)
		}
	}
}
