package detour

import (
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
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
