package recast

import "github.com/aurelien-rainone/assertgo"

// Defines the number of bits allocated to rcSpan::smin and rcSpan::smax.
const (
	RC_SPAN_HEIGHT_BITS uint = 16

	// Defines the maximum value for Span.smin and Span.smax.
	RC_SPAN_MAX_HEIGHT int32 = (1 << RC_SPAN_HEIGHT_BITS) - 1

	// The number of spans allocated per span spool.
	RC_SPANS_PER_POOL int32 = 2048
)

// A Span represents a span in a heightfield.
// see Heightfield
type Span struct {
	smin uint16 // The lower limit of the span. [Limit: < smax]
	smax uint16 // The upper limit of the span. [Limit: <= RC_SPAN_MAX_HEIGHT]
	area uint8  // The area id assigned to the span.
	next *Span  // The next span higher up in column.
}

// A memory pool used for quick allocation of spans within a heightfield.
//
// see Heightfield
type spanPool struct {
	next  *spanPool               // The next span pool.
	items [RC_SPANS_PER_POOL]Span // Array of spans in the pool.
}

// Heightfield is a dynamic heightfield representing obstructed space.
type Heightfield struct {
	Width    int32      // The width of the heightfield. (Along the x-axis in cell units.)
	Height   int32      // The height of the heightfield. (Along the z-axis in cell units.)
	BMin     [3]float32 // The minimum bounds in world space. [(x, y, z)]
	BMax     [3]float32 // The maximum bounds in world space. [(x, y, z)]
	Cs       float32    // The size of each cell. (On the xz-plane.)
	Ch       float32    // The height of each cell. (The minimum increment along the y-axis.)
	Spans    []*Span    // Heightfield of spans (width*height).
	Pools    *spanPool  // Linked list of span pools.
	Freelist *Span      // The next free span.
}

// See the Config documentation for more information on the configuration parameters.
func NewHeightfield(width, height int32, bmin, bmax []float32, cs, ch float32) *Heightfield {
	hf := &Heightfield{
		Width:  width,
		Height: height,
		Cs:     cs,
		Ch:     ch,
		Spans:  make([]*Span, width*height),
	}
	copy(hf.BMin[:], bmin)
	copy(hf.BMax[:], bmax)
	return hf
}

func (hf *Heightfield) Free() {
	if hf == nil {
		return
	}

	// Delete span array.
	hf.Spans = make([]*Span, 0)
	// Delete span pools.
	for hf.Pools != nil {
		next := hf.Pools.next
		hf.Pools = nil
		hf.Pools = next
	}
}

func (hf *Heightfield) allocSpan() *Span {
	// If running out of memory, allocate new page and update the freelist.
	if hf.Freelist == nil || hf.Freelist.next == nil {
		// Create new page.
		// Allocate memory for the new pool.
		pool := &spanPool{}
		if pool == nil {
			return nil
		}

		// Add the pool into the list of pools.
		pool.next = hf.Pools
		hf.Pools = pool
		// Add new items to the free list.
		freelist := hf.Freelist
		var it *Span
		for i := len(pool.items) - 1; i > 0; i-- {
			it = &pool.items[i]
			it.next = freelist
			freelist = it

			if i == 0 {
				break
			}
		}
		hf.Freelist = it
	}

	// Pop item from in front of the free list.
	it := hf.Freelist
	hf.Freelist = hf.Freelist.next
	return it
}

func (hf *Heightfield) freeSpan(ptr *Span) {
	if ptr == nil {
		return
	}
	// Add the node in front of the free list.
	ptr.next = hf.Freelist
	hf.Freelist = ptr
}

func (hf *Heightfield) addSpan(x, y int32, smin, smax uint16,
	area uint8, flagMergeThr int32) bool {

	idx := x + y*hf.Width
	s := hf.allocSpan()
	if s == nil {
		return false
	}
	s.smin = smin
	s.smax = smax
	s.area = area
	s.next = nil

	// Empty cell, add the first span.
	if hf.Spans[idx] == nil {
		hf.Spans[idx] = s
		return true
	}
	var prev *Span
	cur := hf.Spans[idx]

	// Insert and merge spans.
	for cur != nil {
		if cur.smin > s.smax {
			// Current span is further than the new span, break.
			break
		} else if cur.smax < s.smin {
			// Current span is before the new span advance.
			prev = cur
			cur = cur.next
		} else {
			// Merge spans.
			if cur.smin < s.smin {
				s.smin = cur.smin
			}
			if cur.smax > s.smax {
				s.smax = cur.smax
			}

			// Merge flags.
			mergeFlags := int32(s.smax) - int32(cur.smax)
			if mergeFlags < 0 {
				mergeFlags = -mergeFlags
			}
			if mergeFlags <= flagMergeThr {
				if cur.area > s.area {
					s.area = cur.area
				}
			}

			// Remove current span.
			next := cur.next
			hf.freeSpan(cur)
			if prev != nil {
				prev.next = next
			} else {
				hf.Spans[idx] = next
			}
			cur = next
		}
	}

	// Insert new span.
	if prev != nil {
		s.next = prev.next
		prev.next = s
	} else {
		s.next = hf.Spans[idx]
		hf.Spans[idx] = s
	}

	return true
}

// A CompactCell provides information on the content of a cell column in a
// compact heightfield.
type CompactCell struct {
	Index uint32 // Index to the first span in the column.
	Count uint8  // Number of spans in the column.
}

// CompactSpan represents a span of unobstructed space within a compact
// heightfield.
type CompactSpan struct {
	Y   uint16 // The lower extent of the span. (Measured from the heightfield's base.)
	Reg uint16 // The id of the region the span belongs to. (Or zero if not in a region.)
	Con uint32 // Packed neighbor connection data.
	H   uint8  // The height of the span.  (Measured from #y.)
}

// A CompactHeightfield is a compact, static heightfield representing
// unobstructed space.
type CompactHeightfield struct {
	Width          int32         // The width of the heightfield. (Along the x-axis in cell units.)
	Height         int32         // The height of the heightfield. (Along the z-axis in cell units.)
	SpanCount      int32         // The number of spans in the heightfield.
	WalkableHeight int32         // The walkable height used during the build of the field.  (See: Config.walkableHeight)
	WalkableClimb  int32         // The walkable climb used during the build of the field. (See: Config.walkableClimb)
	BorderSize     int32         // The AABB border size used during the build of the field. (See: Config.borderSize)
	MaxDistance    uint16        // The maximum distance value of any span within the field.
	MaxRegions     uint16        // The maximum region id of any span within the field.
	BMin           [3]float32    // The minimum bounds in world space. [(x, y, z)]
	BMax           [3]float32    // The maximum bounds in world space. [(x, y, z)]
	Cs             float32       // The size of each cell. (On the xz-plane.)
	Ch             float32       // The height of each cell. (The minimum increment along the y-axis.)
	Cells          []CompactCell // Array of cells. [Size: width*height]
	Spans          []CompactSpan // Array of spans. [Size: SpanCount]
	Dist           []uint16      // Array containing border distance data. [Size: SpanCount]
	Areas          []uint8       // Array containing area id data. [Size: SpanCount]
}

func (hf *Heightfield) GetHeightFieldSpanCount(ctx *BuildContext) int32 {
	w := hf.Width
	h := hf.Height
	spanCount := int32(0)
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			for s := hf.Spans[x+y*w]; s != nil; s = s.next {
				if s.area != nullArea {
					spanCount++
				}
			}
		}
	}
	return spanCount
}

// BuildCompactHeightfield builds a compact heightfield representing open space,
// from a heightfield representing solid space.
//
//  Arguments:
//   ctx             The build context to use during the operation.
//   walkableHeight  Minimum floor to 'ceiling' height that will still allow the
//                   floor area to be considered walkable.
//                   [Limit: >= 3] [Units: vx]
//   walkableClimb   Maximum ledge height that is considered to still be
//                   traversable. [Limit: >=0] [Units: vx]
//   hf              The heightfield to be compacted.
//   chf             The resulting compact heightfield. (Must be pre-allocated.)
//
//  Returns true if the operation completed successfully.
//
// This is just the beginning of the process of fully building a compact
// heightfield.  Various filters may be applied, then the distance field and
// regions built.  E.g: BuildDistanceField and BuildRegions
//
// See the Config documentation for more information on the configuration parameters.
// see Heightfield, CompactHeightfield, Config
func BuildCompactHeightfield(ctx *BuildContext, walkableHeight, walkableClimb int32,
	hf *Heightfield, chf *CompactHeightfield) bool {

	assert.True(ctx != nil, "ctx should not be nil")
	ctx.StartTimer(TimerBuildCompactHeightfield)

	w := hf.Width
	h := hf.Height
	spanCount := hf.GetHeightFieldSpanCount(ctx)

	// Fill in header.
	chf.Width = w
	chf.Height = h
	chf.SpanCount = spanCount
	chf.WalkableHeight = walkableHeight
	chf.WalkableClimb = walkableClimb
	chf.MaxRegions = 0
	copy(chf.BMin[:], hf.BMin[:])
	copy(chf.BMax[:], hf.BMax[:])
	chf.BMax[1] += float32(walkableHeight) * hf.Ch
	chf.Cs = hf.Cs
	chf.Ch = hf.Ch
	chf.Cells = make([]CompactCell, w*h)
	chf.Spans = make([]CompactSpan, spanCount)
	chf.Areas = make([]uint8, spanCount)
	for i := range chf.Areas {
		chf.Areas[i] = nullArea
	}

	MAX_HEIGHT := int32(0xffff)

	// Fill in cells and spans.
	var idx uint32
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			s := hf.Spans[x+y*w]
			// If there are no spans at this cell, just leave the data to index=0, count=0.
			if s == nil {
				continue
			}
			c := &chf.Cells[x+y*w]
			c.Index = idx
			c.Count = 0
			for s != nil {
				if s.area != nullArea {
					bot := int32(s.smax)
					var top int32
					if s.next != nil {
						top = int32(s.next.smin)
					} else {
						top = MAX_HEIGHT
					}
					chf.Spans[idx].Y = uint16(int32Clamp(bot, 0, 0xffff))
					chf.Spans[idx].H = uint8(int32Clamp(top-bot, 0, 0xff))
					chf.Areas[idx] = s.area
					idx++
					c.Count++
				}
				s = s.next
			}
		}
	}

	// Find neighbour connections.
	const MAX_LAYERS = notConnected - 1
	tooHighNeighbour := int32(0)
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				s := &chf.Spans[i]

				for dir := int32(0); dir < 4; dir++ {
					SetCon(s, dir, notConnected)
					nx := x + GetDirOffsetX(dir)
					ny := y + GetDirOffsetY(dir)
					// First check that the neighbour cell is in bounds.
					if nx < 0 || ny < 0 || nx >= w || ny >= h {
						continue
					}

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					nc := &chf.Cells[nx+ny*w]
					k := int32(nc.Index)
					for nk := int32(nc.Index + uint32(nc.Count)); k < nk; k++ {
						ns := &chf.Spans[k]
						bot := iMax(int32(s.Y), int32(ns.Y))
						top := iMin(int32(s.Y)+int32(s.H), int32(ns.Y)+int32(ns.H))

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if (top-bot) >= walkableHeight && iAbs(int32(ns.Y)-int32(s.Y)) <= walkableClimb {
							// Mark direction as walkable.
							lidx := k - int32(nc.Index)
							if lidx < 0 || lidx > MAX_LAYERS {
								tooHighNeighbour = iMax(tooHighNeighbour, lidx)
								continue
							}
							SetCon(s, dir, lidx)
							break
						}
					}

				}
			}
		}
	}

	if tooHighNeighbour > MAX_LAYERS {
		ctx.Errorf("rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
			tooHighNeighbour, MAX_LAYERS)
	}

	return true
}
