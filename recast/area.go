package recast

import "github.com/aurelien-rainone/assertgo"

/// @par
///
/// Basically, any spans that are closer to a boundary or obstruction than the specified radius
/// are marked as unwalkable.
///
/// This method is usually called immediately after the heightfield has been built.
///
/// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
func ErodeWalkableArea(ctx *Context, radius int32, chf *CompactHeightfield) bool {
	assert.True(ctx != nil, "ctx should not be nil")

	w := chf.width
	h := chf.height

	ctx.StartTimer(RC_TIMER_ERODE_AREA)
	defer ctx.StopTimer(RC_TIMER_ERODE_AREA)

	dist := make([]uint8, chf.spanCount)

	// Init distance.
	for i := range dist {
		dist[i] = 0xff
	}

	// Mark boundary cells.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := chf.cells[x+y*w]
			ni := int32(c.index) + int32(c.count)
			for i := int32(c.index); i < ni; i++ {
				if chf.areas[i] == RC_NULL_AREA {
					dist[i] = 0
				} else {
					s := &chf.spans[i]
					nc := int32(0)
					for dir := int32(0); dir < 4; dir++ {
						if GetCon(s, dir) != RC_NOT_CONNECTED {
							nx := x + GetDirOffsetX(dir)
							ny := y + GetDirOffsetY(dir)
							nidx := int32(chf.cells[nx+ny*w].index) + GetCon(s, dir)
							if chf.areas[nidx] != RC_NULL_AREA {
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
			c := chf.cells[x+y*w]
			ni := int32(c.index) + int32(c.count)
			for i := int32(c.index); i < ni; i++ {
				s := &chf.spans[i]

				if GetCon(s, 0) != RC_NOT_CONNECTED {
					// (-1,0)
					ax := x + GetDirOffsetX(0)
					ay := y + GetDirOffsetY(0)
					ai := int32(chf.cells[ax+ay*w].index) + int32(GetCon(s, 0))
					as := &chf.spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (-1,-1)
					if GetCon(as, 3) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(3)
						aay := ay + GetDirOffsetY(3)
						aai := int32(chf.cells[aax+aay*w].index) + int32(GetCon(as, 3))
						nd = uint8(iMin(int32(dist[aai]+3), int32(255)))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}

				if GetCon(s, 3) != RC_NOT_CONNECTED {
					// (0,-1)
					ax := x + GetDirOffsetX(3)
					ay := y + GetDirOffsetY(3)
					ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, 3)
					as := &chf.spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (1,-1)
					if GetCon(as, 2) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(2)
						aay := ay + GetDirOffsetY(2)
						aai := int32(chf.cells[aax+aay*w].index) + GetCon(as, 2)
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
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				s := &chf.spans[i]

				if GetCon(s, 2) != RC_NOT_CONNECTED {
					// (1,0)
					ax := x + GetDirOffsetX(2)
					ay := y + GetDirOffsetY(2)
					ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, 2)
					as := &chf.spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (1,1)
					if GetCon(as, 1) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(1)
						aay := ay + GetDirOffsetY(1)
						aai := int32(chf.cells[aax+aay*w].index) + GetCon(as, 1)
						nd = uint8(iMin(int32(dist[aai]+3), int32(255)))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
				if GetCon(s, 1) != RC_NOT_CONNECTED {
					// (0,1)
					ax := x + GetDirOffsetX(1)
					ay := y + GetDirOffsetY(1)
					ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, 1)
					as := &chf.spans[ai]
					nd = uint8(iMin(int32(dist[ai]+2), int32(255)))
					if nd < dist[i] {
						dist[i] = nd
					}

					// (-1,1)
					if GetCon(as, 0) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(0)
						aay := ay + GetDirOffsetY(0)
						aai := int32(chf.cells[aax+aay*w].index) + GetCon(as, 0)
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
	for i := int32(0); i < chf.spanCount; i++ {
		if dist[i] < thr {
			chf.areas[i] = RC_NULL_AREA
		}
	}

	dist = nil

	return true
}
