package recast

import "github.com/aurelien-rainone/assertgo"

// Allows the formation of walkable regions that will flow over low lying
// objects such as curbs, and up structures such as stairways.
//
// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
//
// @warning Will override the effect of #rcFilterLedgeSpans.  So if both filters are used, call
// #rcFilterLedgeSpans after calling this filter.
//
// @see rcHeightfield, rcConfig
func FilterLowHangingWalkableObstacles(ctx *Context, walkableClimb int32, solid *Heightfield) {
	assert.True(ctx != nil, "ctx should not be nil")
	ctx.StartTimer(RC_TIMER_FILTER_LOW_OBSTACLES)

	w := solid.Width
	h := solid.Height

	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			var ps *rcSpan
			previousWalkable := false
			previousArea := uint8(RC_NULL_AREA)

			for s := solid.Spans[x+y*w]; s != nil; s = s.next {
				walkable := s.area != RC_NULL_AREA
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if !walkable && previousWalkable {
					if iAbs(int32(s.smax)-int32(ps.smax)) <= walkableClimb {
						s.area = previousArea
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWalkable = walkable
				previousArea = s.area
				ps = s
			}
		}
	}
}

// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
// from the current span's maximum.
// This method removes the impact of the overestimation of conservative voxelization
// so the resulting mesh will not have regions hanging in the air over ledges.
//
// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
//
// @see rcHeightfield, rcConfig
func FilterLedgeSpans(ctx *Context, walkableHeight, walkableClimb int32,
	solid *Heightfield) {
	assert.True(ctx != nil, "ctx should not be nil")
	ctx.StartTimer(RC_TIMER_FILTER_BORDER)

	w := solid.Width
	h := solid.Height
	MAX_HEIGHT := 0xffff

	// Mark border spans.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			for s := solid.Spans[x+y*w]; s != nil; s = s.next {
				// Skip non walkable spans.
				if s.area == RC_NULL_AREA {
					continue
				}

				bot := int32(s.smax)
				var top int32
				if s.next != nil {
					top = int32(s.next.smin)
				} else {
					top = int32(MAX_HEIGHT)
				}

				// Find neighbours minimum height.
				minh := int32(MAX_HEIGHT)

				// Min and max height of accessible neighbours.
				asmin := s.smax
				asmax := s.smax

				for dir := 0; dir < 4; dir++ {
					dx := x + GetDirOffsetX(dir)
					dy := y + GetDirOffsetY(dir)
					// Skip neighbours which are out of bounds.
					if dx < 0 || dy < 0 || dx >= w || dy >= h {
						minh = iMin(minh, -walkableClimb-bot)
						continue
					}

					// From minus infinity to the first span.
					ns := solid.Spans[dx+dy*w]
					nbot := -walkableClimb
					var ntop int32
					if ns != nil {
						ntop = int32(ns.smin)
					} else {
						ntop = int32(MAX_HEIGHT)
					}

					// Skip neightbour if the gap between the spans is too small.
					if iMin(top, ntop)-iMax(bot, nbot) > walkableHeight {
						minh = iMin(minh, nbot-bot)
					}

					// Rest of the spans.
					for ns = solid.Spans[dx+dy*w]; ns != nil; ns = ns.next {
						nbot = int32(ns.smax)
						if ns.next != nil {
							ntop = int32(ns.next.smin)
						} else {
							ntop = int32(MAX_HEIGHT)
						}
						// Skip neightbour if the gap between the spans is too small.
						if iMin(top, ntop)-iMax(bot, nbot) > walkableHeight {
							minh = iMin(minh, nbot-bot)

							// Find min/max accessible neighbour height.
							if iAbs(nbot-bot) <= walkableClimb {
								if nbot < int32(asmin) {
									asmin = uint16(nbot)
								}
								if nbot > int32(asmax) {
									asmax = uint16(nbot)
								}
							}

						}
					}
				}

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if minh < -walkableClimb {
					s.area = RC_NULL_AREA
				} else if int32(asmax-asmin) > walkableClimb {
					// If the difference between all neighbours is too large,
					// we are at steep slope, mark the span as ledge.
					s.area = RC_NULL_AREA
				}
			}
		}
	}
}
