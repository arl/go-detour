package detour

import (
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

func dtNextPow2(v uint32) uint32 {
	v--
	v |= v >> 1
	v |= v >> 2
	v |= v >> 4
	v |= v >> 8
	v |= v >> 16
	v++
	return v
}

// dtiMin returns the minimum of two int32 values.
func dtiMin(a, b int32) int32 {
	if a < b {
		return a
	}
	return b
}

func dtIlog2(v uint32) uint32 {
	boolToUInt32 := func(b bool) uint32 {
		if b {
			return 1
		}
		return 0
	}

	var r, shift uint32
	r = boolToUInt32(v > 0xffff) << 4
	v >>= r
	shift = boolToUInt32(v > 0xff) << 3
	v >>= shift
	r |= shift
	shift = boolToUInt32(v > 0xf) << 2
	v >>= shift
	r |= shift
	shift = boolToUInt32(v > 0x3) << 1
	v >>= shift
	r |= shift
	r |= (v >> 1)
	return r
}

/// @}
/// @name Computational geometry helper functions.
/// @{

/// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
///  @param[in]		a		Vertex A. [(x, y, z)]
///  @param[in]		b		Vertex B. [(x, y, z)]
///  @param[in]		c		Vertex C. [(x, y, z)]
/// @return The signed xz-plane area of the triangle.
func dtTriArea2D(a, b, c d3.Vec3) float32 {
	abx := b[0] - a[0]
	abz := b[2] - a[2]
	acx := c[0] - a[0]
	acz := c[2] - a[2]
	return acx*abz - abx*acz
}

func vperpXZ(a, b d3.Vec3) float32 {
	return a[0]*b[2] - a[2]*b[0]
}

func DtIntersectSegSeg2D(ap, aq, bp, bq d3.Vec3) (hit bool, s, t float32) {
	u := aq.Sub(ap)
	v := bq.Sub(bp)
	w := ap.Sub(bp)

	d := vperpXZ(u, v)
	if math32.Abs(d) < 1e-6 {
		return hit, s, t
	}
	return true, vperpXZ(v, w) / d, vperpXZ(u, w) / d
}

// Determines if two axis-aligned bounding boxes overlap.
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
// @see dtOverlapBounds
func dtOverlapQuantBounds(amin, amax, bmin, bmax []uint16) bool {
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

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapQuantBounds
func dtOverlapBounds(amin, amax, bmin, bmax []float32) bool {
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

func dtDistancePtPolyEdgesSqr(pt, verts []float32, nverts int32, ed, et []float32) bool {
	// TODO: Replace pnpoly with triArea2D tests?
	c := false
	for i, j := 0, (nverts - 1); i < int(nverts); i++ {
		vi := verts[i*3 : i*3+3]
		vj := verts[j*3 : j*3+3]
		if ((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0])*(pt[2]-vi[2])/(vj[2]-vi[2])+vi[0]) {
			c = !c
		}
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, &et[j])
		j = int32(i)
	}
	return c
}

func dtDistancePtSegSqr2D(pt, p, q []float32, t *float32) float32 {
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

func dtClosestHeightPointTriangle(p, a, b, c d3.Vec3, h *float32) bool {
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

func dtOppositeTile(side int32) int32 {
	return (side + 4) & 0x7
}
