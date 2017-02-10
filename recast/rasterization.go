package recast

import (
	"github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

// RasterizeTriangle rasterizes a triangle into the specified heightfield.
//
//  Arguments:
//  ctx           The build context to use during the operation.
//  v0            Triangle vertex 0 [(x, y, z)]
//  v1            Triangle vertex 1 [(x, y, z)]
//  v2            Triangle vertex 2 [(x, y, z)]
//  area          The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
//  solid         An initialized heightfield.
//  flagMergeThr  The distance where the walkable flag is favored over the
//                non-walkable flag.[Limit: >= 0] [Units: vx]
//
// Returns True if the operation completed successfully.
// No spans will be added if the triangle does not overlap the heightfield grid.
// see Heightfield
func RasterizeTriangle(ctx *BuildContext, v0, v1, v2 d3.Vec3,
	area uint8, solid *Heightfield,
	flagMergeThr int32) bool {
	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerRasterizeTriangles)
	defer ctx.StopTimer(TimerRasterizeTriangles)

	ics := 1.0 / solid.Cs
	ich := 1.0 / solid.Ch
	if !rasterizeTri(v0, v1, v2, area, solid, solid.BMin[:], solid.BMax[:], solid.Cs, ics, ich, flagMergeThr) {
		ctx.Errorf("RasterizeTriangle: Out of memory.")
		return false
	}

	return true
}

// RasterizeTriangles rasterizes an indexed triangle mesh into the specified
// heightfield.
//
//  Arguments:
//  ctx           The build context to use during the operation.
//  verts         The vertices. [(x, y, z) * @p nv]
//  nv            The number of vertices.
//  tris          The triangle indices. [(vertA, vertB, vertC) * @p nt]
//  areas         The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA]
//                [Size: @p nt]
//  nt            The number of triangles.
//  solid         An initialized heightfield.
//  flagMergeThr  The distance where the walkable flag is favored over the
//                non-walkable flag. [Limit: >= 0] [Units: vx]
//
//  Returns True if the operation completed successfully.
//
// Spans will only be added for triangles that overlap the heightfield grid.
//
// see Heightfield
func RasterizeTriangles(ctx *BuildContext, verts []float32, nv int32,
	tris []int32, areas []uint8, nt int32,
	solid *Heightfield, flagMergeThr int32) bool {

	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerRasterizeTriangles)
	defer ctx.StopTimer(TimerRasterizeTriangles)

	ics := 1.0 / solid.Cs
	ich := 1.0 / solid.Ch
	// Rasterize triangles.
	for i := int32(0); i < nt; i++ {
		v0 := verts[tris[i*3+0]*3:]
		v1 := verts[tris[i*3+1]*3:]
		v2 := verts[tris[i*3+2]*3:]
		// Rasterize.
		if !rasterizeTri(v0, v1, v2, areas[i], solid, solid.BMin[:], solid.BMax[:], solid.Cs, ics, ich, flagMergeThr) {
			ctx.Errorf("RasterizeTriangles: Out of memory.")
			return false
		}
	}

	return true
}

// RasterizeTriangles2 rasterizes triangles into the specified heightfield.
//
//  Arguments:
//  ctx           The build context to use during the operation.
//  verts         The triangle vertices.
//                [(ax, ay, az, bx, by, bz, cx, by, cx) * nt]
//  areas         The area id's of the triangles.
//                [Limit: <= #RC_WALKABLE_AREA] [Size: nt]
//  nt            The number of triangles.
//  solid         An initialized heightfield.
//  flagMergeThr  The distance where the walkable flag is favored over the
//                non-walkable flag. [Limit: >= 0] [Units: vx]
//
// Returns True if the operation completed successfully.
// Spans will only be added for triangles that overlap the heightfield grid.
//
// see Heightfield
func RasterizeTriangles2(ctx *BuildContext, verts []float32, areas []uint8, nt int32,
	solid *Heightfield, flagMergeThr int32) bool {

	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerRasterizeTriangles)
	defer ctx.StopTimer(TimerRasterizeTriangles)

	ics := float32(1.0 / solid.Cs)
	ich := float32(1.0 / solid.Ch)
	// Rasterize triangles.
	for i := int32(0); i < nt; i++ {
		v0 := verts[(i*3+0)*3:]
		v1 := verts[(i*3+1)*3:]
		v2 := verts[(i*3+2)*3:]
		// Rasterize.
		if !rasterizeTri(v0, v1, v2, areas[i], solid, solid.BMin[:], solid.BMax[:], solid.Cs, ics, ich, flagMergeThr) {
			ctx.Errorf("rcRasterizeTriangles: Out of memory.")
			return false
		}
	}

	return true
}

func rasterizeTri(v0, v1, v2 []float32,
	area uint8, hf *Heightfield,
	bmin, bmax []float32,
	cs, ics, ich float32,
	flagMergeThr int32) bool {

	w := hf.Width
	h := hf.Height
	var tmin, tmax [3]float32
	by := bmax[1] - bmin[1]

	// Calculate the bounding box of the triangle.
	copy(tmin[:], v0)
	copy(tmax[:], v0)
	d3.Vec3Min(tmin[:], v1)
	d3.Vec3Min(tmin[:], v2)
	d3.Vec3Max(tmax[:], v1)
	d3.Vec3Max(tmax[:], v2)

	// If the triangle does not touch the bbox of the heightfield, skip the triagle.
	if !overlapBounds(bmin, bmax, tmin[:], tmax[:]) {
		return true
	}

	// Calculate the footprint of the triangle on the grid's y-axis
	y0 := int32((tmin[2] - bmin[2]) * ics)
	y1 := int32((tmax[2] - bmin[2]) * ics)
	y0 = int32Clamp(y0, 0, h-1)
	y1 = int32Clamp(y1, 0, h-1)

	// Clip the triangle into all grid cells it touches.
	var buf [7 * 3 * 4]float32

	in := buf[:]
	inrow := buf[7*3:]
	p1 := inrow[7*3:]
	p2 := p1[7*3:]

	copy(in, v0)
	copy(in[3:6], v1)
	copy(in[6:9], v2)

	var nvrow, nvIn int32
	nvIn = 3

	for y := y0; y <= y1; y++ {
		// Clip polygon to row. Store the remaining polygon as well
		cz := bmin[2] + float32(y)*cs
		dividePoly(in, nvIn, inrow, &nvrow, p1, &nvIn, cz+cs, 2)
		in, p1 = p1, in
		if nvrow < 3 {
			continue
		}

		// find the horizontal bounds in the row
		minX, maxX := inrow[0], inrow[0]
		for i := int32(1); i < nvrow; i++ {
			if minX > inrow[i*3] {
				minX = inrow[i*3]
			}
			if maxX < inrow[i*3] {
				maxX = inrow[i*3]
			}
		}
		x0 := int32((minX - bmin[0]) * ics)
		x1 := int32((maxX - bmin[0]) * ics)
		x0 = int32Clamp(x0, 0, w-1)
		x1 = int32Clamp(x1, 0, w-1)

		var nv, nv2 int32
		nv2 = nvrow

		for x := x0; x <= x1; x++ {
			// Clip polygon to column. store the remaining polygon as well
			cx := bmin[0] + float32(x)*cs
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx+cs, 0)
			inrow, p2 = p2, inrow
			if nv < 3 {
				continue
			}

			// Calculate min and max of the span.
			smin, smax := p1[1], p1[1]
			for i := int32(1); i < nv; i++ {
				smin = math32.Min(smin, p1[i*3+1])
				smax = math32.Max(smax, p1[i*3+1])
			}
			smin -= bmin[1]
			smax -= bmin[1]

			// Skip the span if it is outside the heightfield bbox
			if smax < 0.0 {
				continue
			}
			if smin > by {
				continue
			}
			// Clamp the span to the heightfield bbox.
			if smin < 0.0 {
				smin = 0
			}
			if smax > by {
				smax = by
			}

			// Snap the span to the heightfield height grid.
			ismin := uint16(int32Clamp(int32(math32.Floor(smin*ich)), 0, RC_SPAN_MAX_HEIGHT))
			ismax := uint16(int32Clamp(int32(math32.Ceil(smax*ich)), int32(ismin+1), RC_SPAN_MAX_HEIGHT))

			if !hf.addSpan(x, y, ismin, ismax, area, flagMergeThr) {
				return false
			}
		}
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
// Return true if the two AABB's overlap.
func overlapBounds(amin, amax, bmin, bmax []float32) bool {
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

func int32Clamp(a, low, high int32) int32 {
	if a < low {
		return low
	} else if a > high {
		return high
	}

	return a
}

// divides a convex polygons into two convex polygons on both sides of a line
func dividePoly(in []float32, nin int32,
	out1 []float32, nout1 *int32,
	out2 []float32, nout2 *int32,
	x float32, axis int32) {
	var d [12]float32
	for i := int32(0); i < nin; i++ {
		d[i] = x - in[i*3+axis]
	}

	var m, n int32
	j := nin - 1
	for i := int32(0); i < nin; /*j=i, */ i++ {
		ina := d[j] >= 0
		inb := d[i] >= 0
		if ina != inb {
			s := d[j] / (d[j] - d[i])
			out1[m*3+0] = in[j*3+0] + (in[i*3+0]-in[j*3+0])*s
			out1[m*3+1] = in[j*3+1] + (in[i*3+1]-in[j*3+1])*s
			out1[m*3+2] = in[j*3+2] + (in[i*3+2]-in[j*3+2])*s

			copy(out2[n*3:n*3+3], out1[m*3:m*3+3])
			m++
			n++
			// add the i'th point to the right polygon. Do NOT add points that
			// are on the dividing line since these were already added above
			if d[i] > 0 {
				copy(out1[m*3:m*3+3], in[i*3:i*3+3])
				m++
			} else if d[i] < 0 {
				copy(out2[n*3:n*3+3], in[i*3:i*3+3])
				n++
			}
		} else {
			// same side add the i'th point to the right polygon. Addition is
			// done even for points on the dividing line
			if d[i] >= 0 {
				copy(out1[m*3:m*3+3], in[i*3:i*3+3])
				m++
				if d[i] != 0 {
					j = i
					continue
				}
			}
			copy(out2[n*3:n*3+3], in[i*3:i*3+3])
			n++
		}
		j = i
	}

	*nout1 = m
	*nout2 = n
}
