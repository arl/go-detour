package detour

import (
	"github.com/arl/gogeo/f32/d3"
	"github.com/arl/math32"
)

// Computational geometry helper functions.

// TriArea2D derives the signed xz-plane area of the triangle ABC, or the
// relationship of line AB to point C.
//
//  a   Vertex A. [(x, y, z)]
//  b   Vertex B. [(x, y, z)]
//  c   Vertex C. [(x, y, z)]
// return The signed xz-plane area of the triangle.
func TriArea2D(a, b, c d3.Vec3) float32 {
	abx := b[0] - a[0]
	abz := b[2] - a[2]
	acx := c[0] - a[0]
	acz := c[2] - a[2]
	return acx*abz - abx*acz
}

// IntersectSegSeg2D returns wether two segments intersect, and their
// intersection point.
func IntersectSegSeg2D(ap, aq, bp, bq d3.Vec3) (hit bool, s, t float32) {
	u := aq.Sub(ap)
	v := bq.Sub(bp)
	w := ap.Sub(bp)

	d := u.Perp2D(v)
	if math32.Abs(d) < 1e-6 {
		return false, s, t
	}
	return true, v.Perp2D(w) / d, u.Perp2D(w) / d
}

// OverlapQuantBounds determines if two axis-aligned bounding boxes overlap.
//
//  amin   Minimum bounds of box A. [(x, y, z)]
//  amax   Maximum bounds of box A. [(x, y, z)]
//  bmin   Minimum bounds of box B. [(x, y, z)]
//  bmax   Maximum bounds of box B. [(x, y, z)]
//  return True if the two AABB's overlap.
// see overlapBounds
func OverlapQuantBounds(amin, amax, bmin, bmax []uint16) bool {
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		return false
	}

	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		return false
	}

	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		return false
	}
	return true
}

// OverlapBounds determines if two axis-aligned bounding boxes overlap.
//
//  Arguments:
//   amin     Minimum bounds of box A. [(x, y, z)]
//   amax     Maximum bounds of box A. [(x, y, z)]
//   bmin     Minimum bounds of box B. [(x, y, z)]
//   bmax     Maximum bounds of box B. [(x, y, z)]
//
// Return True if the two AABB's overlap.
// see overlapQuantBounds
func OverlapBounds(amin, amax, bmin, bmax []float32) bool {
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		return false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		return false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		return false
	}
	return true
}

func distancePtPolyEdgesSqr(pt, verts []float32, nverts int32, ed, et []float32) bool {
	// TODO: Replace pnpoly with triArea2D tests?
	c := false
	for i, j := 0, (nverts - 1); i < int(nverts); i++ {
		vi := verts[i*3 : i*3+3]
		vj := verts[j*3 : j*3+3]
		if ((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0])*(pt[2]-vi[2])/(vj[2]-vi[2])+vi[0]) {
			c = !c
		}
		ed[j] = distancePtSegSqr2D(pt, vj, vi, &et[j])
		j = int32(i)
	}
	return c
}

func projectPoly(axis d3.Vec3, poly []float32, npoly int32) (rmin, rmax float32) {
	rmin = axis.Dot2D(poly[:3])
	rmax = rmin
	for i := int32(1); i < npoly; i++ {
		d := axis.Dot2D(poly[i*3:])
		if d < rmin {
			rmin = d
		}
		if d > rmax {
			rmax = d
		}
	}
	return
}

func overlapRange(amin, amax, bmin, bmax, eps float32) bool {
	return !((amin+eps) > bmax || (amax-eps) < bmin)
}

// OverlapPolyPoly2 determines if the two convex polygons overlap on the
// xz-plane.
//
//  Arguments:
//   polya    Polygon A vertices. [(x, y, z) * @p npolya]
//   npolya   The number of vertices in polygon A.
//   polyb    Polygon B vertices. [(x, y, z) * @p npolyb]
//   npolyb   The number of vertices in polygon B.
//
//  Returns true if the two polygons overlap.
//
// All vertices are projected onto the xz-plane, so the y-values are ignored.
func OverlapPolyPoly2D(polya []float32, npolya int32,
	polyb []float32, npolyb int32) bool {
	const eps = 1e-4

	for i, j := int32(0), npolya-1; i < npolya; j, i = i, i+1 {
		va := polya[j*3:]
		vb := polya[i*3:]
		n := []float32{vb[2] - va[2], 0, -(vb[0] - va[0])}
		var amin, amax, bmin, bmax float32
		amin, amax = projectPoly(n, polya, npolya)
		bmin, bmax = projectPoly(n, polyb, npolyb)
		if !overlapRange(amin, amax, bmin, bmax, eps) {
			// Found separating axis
			return false
		}
	}
	for i, j := int32(0), npolyb-1; i < npolyb; j, i = i, i+1 {
		va := polyb[j*3:]
		vb := polyb[i*3:]
		n := []float32{vb[2] - va[2], 0, -(vb[0] - va[0])}
		var amin, amax, bmin, bmax float32
		amin, amax = projectPoly(n, polya, npolya)
		bmin, bmax = projectPoly(n, polyb, npolyb)
		if !overlapRange(amin, amax, bmin, bmax, eps) {
			// Found separating axis
			return false
		}
	}
	return true
}

func IntersectSegmentPoly2D(p0, p1 d3.Vec3, verts []float32, nverts int) (tmin, tmax float32, segMin, segMax int, res bool) {
	const eps float32 = 0.00000001

	tmin = 0
	tmax = 1
	segMin = -1
	segMax = -1

	var dir d3.Vec3 = p1.Sub(p0)
	j := nverts - 1
	for i := 0; i < nverts; i++ {
		edge := d3.Vec3(verts[i*3:]).Sub(d3.Vec3(verts[j*3:]))
		diff := p0.Sub(d3.Vec3(verts[j*3:]))
		n := edge.Perp2D(diff)
		d := dir.Perp2D(edge)
		if math32.Abs(d) < eps {
			// S is nearly parallel to this edge
			if n < 0 {
				return
			}
			j = i
			continue
		}
		t := n / d
		if d < 0 {
			// segment S is entering across this edge
			if t > tmin {
				tmin = t
				segMin = j
				// S enters after leaving polygon
				if tmin > tmax {
					return
				}
			}
		} else {
			// segment S is leaving across this edge
			if t < tmax {
				tmax = t
				segMax = j
				// S leaves before entering polygon
				if tmax < tmin {
					return
				}
			}
		}
		j = i
	}

	res = true
	return
}

// TODO: go-ify (return 2 params)
func distancePtSegSqr2D(pt, p, q d3.Vec3, t *float32) float32 {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqz*pqz
	*t = pqx*dx + pqz*dz
	if d > 0 {
		*t /= d
	}
	if *t < float32(0) {
		*t = 0
	} else if *t > 1 {
		*t = 1
	}
	dx = p[0] + *t*pqx - pt[0]
	dz = p[2] + *t*pqz - pt[2]
	return dx*dx + dz*dz
}

// TODO: go-ify (return 2 params)
func closestHeightPointTriangle(p, a, b, c d3.Vec3, h *float32) bool {
	v0 := c.Sub(a)
	v1 := b.Sub(a)
	v2 := p.Sub(a)

	dot00 := v0.Dot2D(v0)
	dot01 := v0.Dot2D(v1)
	dot02 := v0.Dot2D(v2)
	dot11 := v1.Dot2D(v1)
	dot12 := v1.Dot2D(v2)

	// Compute barycentric coordinates
	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	EPS := float32(1e-4)

	// If point lies inside the triangle, return interpolated ycoord.
	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		*h = a[1] + v0[1]*u + v1[1]*v
		return true
	}

	return false
}

func oppositeTile(side int32) int32 {
	return (side + 4) & 0x7
}
