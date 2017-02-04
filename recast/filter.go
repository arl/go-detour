package recast

import "github.com/aurelien-rainone/assertgo"

// FilterLowHangingWalkableObstacles marks non-walkable spans as walkable if
// their maximum is within p walkableClimb of a walkable neighbor.
//
//  Arguments:
//   ctx           The build context to use during the operation.
//   walkableClimb Maximum ledge height that is considered to still be
//                 traversable. [Limit: >=0] [Units: vx]
//   solid         A fully built heightfield.  (All spans have been added.)
//
// Allows the formation of walkable regions that will flow over low lying
// objects such as curbs, and up structures such as stairways.
//
// Two neighboring spans are walkable if:
// Abs(currentSpan.smax - neighborSpan.smax) < waklableClimb
//
// Warning Will override the effect of FilterLedgeSpans. So if both filters are
// used, call FilterLedgeSpans after calling this filter.
//
// see Heightfield, Config
func FilterLowHangingWalkableObstacles(ctx *BuildContext, walkableClimb int32, solid *Heightfield) {
	assert.True(ctx != nil, "ctx should not be nil")
	ctx.StartTimer(TimerFilterLowObstacles)

	w := solid.Width
	h := solid.Height

	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			var ps *Span
			previousWalkable := false
			previousArea := uint8(nullArea)

			for s := solid.Spans[x+y*w]; s != nil; s = s.next {
				walkable := s.area != nullArea
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

// FilterLedgeSpans marks spans that are ledges as not-walkable.
//
//  Arguments:
//  ctx             The build context to use during the operation.
//  walkableHeight  Minimum floor to 'ceiling' height that will still allow the
//                  floor area to be considered walkable.
//                  [Limit: >= 3] [Units: vx]
//  walkableClimb   Maximum ledge height that is considered to still be
//                  traversable. [Limit: >=0] [Units: vx]
//  solid           A fully built heightfield.  (All spans have been added.)
//
// A ledge is a span with one or more neighbors whose maximum is further away
// than walkableClimb from the current span's maximum.
// This method removes the impact of the overestimation of conservative
// voxelization so the resulting mesh will not have regions hanging in the air
// over ledges.
//
// A span is a ledge if:
// Abs(currentSpan.smax - neighborSpan.smax) > walkableClimb
//
// see Heightfield, Config
func FilterLedgeSpans(ctx *BuildContext, walkableHeight, walkableClimb int32, solid *Heightfield) {
	assert.True(ctx != nil, "ctx should not be nil")
	ctx.StartTimer(TimerFilterBorder)

	w := solid.Width
	h := solid.Height
	MAX_HEIGHT := 0xffff

	// Mark border spans.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			for s := solid.Spans[x+y*w]; s != nil; s = s.next {
				// Skip non walkable spans.
				if s.area == nullArea {
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

				for dir := int32(0); dir < 4; dir++ {
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
					s.area = nullArea
				} else if int32(asmax-asmin) > walkableClimb {
					// If the difference between all neighbours is too large,
					// we are at steep slope, mark the span as ledge.
					s.area = nullArea
				}
			}
		}
	}
}

// FilterWalkableLowHeightSpans marks walkable spans as not walkable if the
// clearance above the span is less than the specified height.
//
//  Arguments:
//  ctx             The build context to use during the operation.
//  walkableHeight  Minimum floor to 'ceiling' height that will still allow the
//                  floor area to be considered walkable.
//                  [Limit: >= 3] [Units: vx]
//  solid           A fully built heightfield. (All spans have been added.)
//
// For this filter, the clearance above the span is the distance from the span's
// maximum to the next higher span's minimum. (Same grid column.)
//
// see Heightfield, Config
func FilterWalkableLowHeightSpans(ctx *BuildContext, walkableHeight int32, solid *Heightfield) {
	assert.True(ctx != nil, "ctx should not be nil")
	ctx.StartTimer(TimerFilterWalkable)

	w := solid.Width
	h := solid.Height
	MAX_HEIGHT := int32(0xffff)

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			for s := solid.Spans[x+y*w]; s != nil; s = s.next {
				bot := int32(s.smax)
				var top int32
				if s.next != nil {
					top = int32(s.next.smin)
				} else {
					top = int32(MAX_HEIGHT)
				}
				if (top - bot) <= walkableHeight {
					s.area = nullArea
				}
			}
		}
	}
}
