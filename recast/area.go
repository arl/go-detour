package recast

import (
	"github.com/aurelien-rainone/assertgo"
	"github.com/aurelien-rainone/gogeo/f32/d3"
)

// ErodeWalkableArea erodes the walkable area within the heightfield by the
// specified radius.
//
//  Arguments:
//   ctx     The build context to use during the operation.
//   radius  The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
//   chf     The populated compact heightfield to erode.
//
// Returns true if the operation completed successfully.
//
// Basically, any spans that are closer to a boundary or obstruction than the
// specified radius are marked as unwalkable.
//
// This method is usually called immediately after the heightfield has been
// built.
// see CompactHeightfield, BuildCompactHeightfield, Config.WalkableRadius
func ErodeWalkableArea(ctx *BuildContext, radius int32, chf *CompactHeightfield) bool {
	assert.True(ctx != nil, "ctx should not be nil")

	w := chf.Width
	h := chf.Height

	ctx.StartTimer(TimerErodeArea)
	defer ctx.StopTimer(TimerErodeArea)

	dist := make([]uint8, chf.SpanCount)

	// Init distance.
	for i := range dist {
		dist[i] = 0xff
	}

	// Mark boundary cells.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			ni := int32(c.Index) + int32(c.Count)
			for i := int32(c.Index); i < ni; i++ {
				if chf.Areas[i] == nullArea {
					dist[i] = 0
				} else {
					s := &chf.Spans[i]
					nc := int32(0)
					for dir := int32(0); dir < 4; dir++ {
						if GetCon(s, dir) != notConnected {
							nx := x + GetDirOffsetX(dir)
							ny := y + GetDirOffsetY(dir)
							nidx := int32(chf.Cells[nx+ny*w].Index) + GetCon(s, dir)
							if chf.Areas[nidx] != nullArea {
								nc++
							}
						}
					}
					// At least one missing neighbour.
					if nc != 4 {
						dist[i] = 0
					}
				}
			}
		}
	}

	var nd uint8

	// Pass 1
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			ni := int32(c.Index) + int32(c.Count)
			for i := int32(c.Index); i < ni; i++ {
				s := &chf.Spans[i]

				if GetCon(s, 0) != notConnected {
					// (-1,0)
					ax := x + GetDirOffsetX(0)
					ay := y + GetDirOffsetY(0)
					ai := int32(chf.Cells[ax+ay*w].Index) + int32(GetCon(s, 0))
					as := &chf.Spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (-1,-1)
					if GetCon(as, 3) != notConnected {
						aax := ax + GetDirOffsetX(3)
						aay := ay + GetDirOffsetY(3)
						aai := int32(chf.Cells[aax+aay*w].Index) + int32(GetCon(as, 3))
						nd = uint8(iMin(int32(dist[aai]+3), int32(255)))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}

				if GetCon(s, 3) != notConnected {
					// (0,-1)
					ax := x + GetDirOffsetX(3)
					ay := y + GetDirOffsetY(3)
					ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, 3)
					as := &chf.Spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (1,-1)
					if GetCon(as, 2) != notConnected {
						aax := ax + GetDirOffsetX(2)
						aay := ay + GetDirOffsetY(2)
						aai := int32(chf.Cells[aax+aay*w].Index) + GetCon(as, 2)
						nd = uint8(iMin(int32(dist[aai]+3), int32(255)))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
			}
		}
	}

	// Pass 2
	for y := int32(h - 1); y >= 0; y-- {
		for x := int32(w - 1); x >= 0; x-- {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				s := &chf.Spans[i]

				if GetCon(s, 2) != notConnected {
					// (1,0)
					ax := x + GetDirOffsetX(2)
					ay := y + GetDirOffsetY(2)
					ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, 2)
					as := &chf.Spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (1,1)
					if GetCon(as, 1) != notConnected {
						aax := ax + GetDirOffsetX(1)
						aay := ay + GetDirOffsetY(1)
						aai := int32(chf.Cells[aax+aay*w].Index) + GetCon(as, 1)
						nd = uint8(iMin(int32(dist[aai]+3), int32(255)))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
				if GetCon(s, 1) != notConnected {
					// (0,1)
					ax := x + GetDirOffsetX(1)
					ay := y + GetDirOffsetY(1)
					ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, 1)
					as := &chf.Spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (-1,1)
					if GetCon(as, 0) != notConnected {
						aax := ax + GetDirOffsetX(0)
						aay := ay + GetDirOffsetY(0)
						aai := int32(chf.Cells[aax+aay*w].Index) + GetCon(as, 0)
						nd = uint8(iMin(int32(dist[aai]+3), int32(255)))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
			}
		}
	}

	thr := uint8(radius * 2)
	for i := int32(0); i < chf.SpanCount; i++ {
		if dist[i] < thr {
			chf.Areas[i] = nullArea
		}
	}

	dist = nil

	return true
}

// MarkConvexPolyArea applies the area id to the all spans within the specified
// convex polygon.
//
//  Arguments:
//   ctx     The build context to use during the operation.
//   verts   The vertices of the polygon [Fomr: (x, y, z) * @p nverts]
//   nverts  The number of vertices in the polygon.
//   hmin    The height of the base of the polygon.
//   hmax    The height of the top of the polygon.
//   areaID  The area id to apply. [Limit: <= RC_WALKABLE_AREA]
//   chf     A populated compact heightfield.
//
// The value of spacial parameters are in world units.
//
// The y-values of the polygon vertices are ignored. So the polygon is
// effectively projected onto the xz-plane at hmin, then extruded to hmax.
//
// See CompactHeightfield, MedianFilterWalkableArea
func MarkConvexPolyArea(ctx *BuildContext, verts []float32, nverts int32,
	hmin, hmax float32, areaID uint8, chf *CompactHeightfield) {

	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerMarkConvexPolyArea)
	defer ctx.StopTimer(TimerMarkConvexPolyArea)
	var bmin, bmax [3]float32
	copy(bmin[:], verts[:3])
	copy(bmax[:], verts[:3])
	for i := int32(1); i*3 < nverts; i++ {
		v := verts[i*3:]
		d3.Vec3Min(bmin[:], v)
		d3.Vec3Max(bmax[:], v)
	}
	bmin[1] = hmin
	bmax[1] = hmax

	minx := int32(((bmin[0] - chf.BMin[0]) / chf.Cs))
	miny := int32(((bmin[1] - chf.BMin[1]) / chf.Ch))
	minz := int32(((bmin[2] - chf.BMin[2]) / chf.Cs))
	maxx := int32(((bmax[0] - chf.BMin[0]) / chf.Cs))
	maxy := int32(((bmax[1] - chf.BMin[1]) / chf.Ch))
	maxz := int32(((bmax[2] - chf.BMin[2]) / chf.Cs))

	if maxx < 0 {
		return
	}
	if minx >= chf.Width {
		return
	}
	if maxz < 0 {
		return
	}
	if minz >= chf.Height {
		return
	}

	if minx < 0 {
		minx = 0
	}
	if maxx >= chf.Width {
		maxx = chf.Width - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= chf.Height {
		maxz = chf.Height - 1
	}

	// TODO: Optimize.
	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			c := &chf.Cells[x+z*chf.Width]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				if chf.Areas[i] == nullArea {
					continue
				}
				s := &chf.Spans[i]
				if int32(s.Y) >= miny && int32(s.Y) <= maxy {
					var p [3]float32
					p[0] = chf.BMin[0] + (float32(x)+0.5)*chf.Cs
					p[1] = 0
					p[2] = chf.BMin[2] + (float32(z)+0.5)*chf.Cs

					if pointInPoly(nverts, verts, p[:]) {
						chf.Areas[i] = areaID
					}
				}
			}
		}
	}
}

func pointInPoly(nvert int32, verts, p []float32) bool {
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
		j = i
	}
	return c
}
