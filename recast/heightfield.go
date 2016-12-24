package recast

/// Defines the number of bits allocated to rcSpan::smin and rcSpan::smax.
const (
	RC_SPAN_HEIGHT_BITS uint = 16
	/// Defines the maximum value for rcSpan::smin and rcSpan::smax.
	RC_SPAN_MAX_HEIGHT int32 = (1 << RC_SPAN_HEIGHT_BITS) - 1
	/// The number of spans allocated per span spool.
	RC_SPANS_PER_POOL int32 = 2048
)

/// Represents a span in a heightfield.
/// @see rcHeightfield
type rcSpan struct {
	//unsigned int smin : RC_SPAN_HEIGHT_BITS; ///< The lower limit of the span. [Limit: < #smax]
	//unsigned int smax : RC_SPAN_HEIGHT_BITS; ///< The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT]
	//unsigned int area : 6;                   ///< The area id assigned to the span.

	smin uint16  ///< The lower limit of the span. [Limit: < #smax]
	smax uint16  ///< The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT]
	area uint8   ///< The area id assigned to the span.
	next *rcSpan ///< The next span higher up in column.
}

/// A memory pool used for quick allocation of spans within a heightfield.
/// @see rcHeightfield
type rcSpanPool struct {
	next  *rcSpanPool               ///< The next span pool.
	items [RC_SPANS_PER_POOL]rcSpan ///< Array of spans in the pool.
}

// A dynamic heightfield representing obstructed space.
type Heightfield struct {
	Width    int32       // The width of the heightfield. (Along the x-axis in cell units.)
	Height   int32       // The height of the heightfield. (Along the z-axis in cell units.)
	BMin     [3]float32  // The minimum bounds in world space. [(x, y, z)]
	BMax     [3]float32  // The maximum bounds in world space. [(x, y, z)]
	Cs       float32     // The size of each cell. (On the xz-plane.)
	Ch       float32     // The height of each cell. (The minimum increment along the y-axis.)
	Spans    []*rcSpan   // Heightfield of spans (width*height).
	Pools    *rcSpanPool // Linked list of span pools.
	Freelist *rcSpan     // The next free span.
}

func NewHeightfield() *Heightfield {
	return &Heightfield{}
}

/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocHeightfield, rcHeightfield
func (hf *Heightfield) Create(ctx *Context, width, height int32,
	bmin, bmax []float32, cs, ch float32) bool {
	hf.Width = width
	hf.Height = height
	copy(hf.BMin[:], bmin)
	copy(hf.BMax[:], bmax)
	hf.Cs = cs
	hf.Ch = ch
	hf.Spans = make([]*rcSpan, hf.Width*hf.Height)
	if len(hf.Spans) == 0 {
		// NOTE: since in Go, we don't have (for now) any way to check that we
		// are out of memory, this check doesn't have any sense, but for
		// completeness with the originla C code, I'll let it, hopefully it will
		// be replaced one day by a proper "allocation done check"
		return false
	}
	return true
}

func (hf *Heightfield) allocSpan() *rcSpan {
	// If running out of memory, allocate new page and update the freelist.
	if hf.Freelist == nil || hf.Freelist.next == nil {
		// Create new page.
		// Allocate memory for the new pool.
		pool := &rcSpanPool{}
		if pool == nil {
			return nil
		}

		// Add the pool into the list of pools.
		pool.next = hf.Pools
		hf.Pools = pool
		// Add new items to the free list.
		freelist := hf.Freelist
		//head := &pool.items[0]
		//it := pool.items[RC_SPANS_PER_POOL]
		var it *rcSpan
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

func (hf *Heightfield) freeSpan(ptr *rcSpan) {
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
	var prev *rcSpan
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

/// Provides information on the content of a cell column in a compact heightfield.
type CompactCell struct {
	index uint32 ///< Index to the first span in the column.
	count uint8  ///< Number of spans in the column.
}

/// Represents a span of unobstructed space within a compact heightfield.
type CompactSpan struct {
	y   uint16 ///< The lower extent of the span. (Measured from the heightfield's base.)
	reg uint16 ///< The id of the region the span belongs to. (Or zero if not in a region.)
	con uint32 ///< Packed neighbor connection data.
	h   uint8  ///< The height of the span.  (Measured from #y.)
}

/// A compact, static heightfield representing unobstructed space.
/// @ingroup recast
type CompactHeightfield struct {
	width          int32         ///< The width of the heightfield. (Along the x-axis in cell units.)
	height         int32         ///< The height of the heightfield. (Along the z-axis in cell units.)
	spanCount      int32         ///< The number of spans in the heightfield.
	walkableHeight int32         ///< The walkable height used during the build of the field.  (See: rcConfig::walkableHeight)
	walkableClimb  int32         ///< The walkable climb used during the build of the field. (See: rcConfig::walkableClimb)
	borderSize     int32         ///< The AABB border size used during the build of the field. (See: rcConfig::borderSize)
	maxDistance    uint16        ///< The maximum distance value of any span within the field.
	maxRegions     uint16        ///< The maximum region id of any span within the field.
	bmin           [3]float32    ///< The minimum bounds in world space. [(x, y, z)]
	bmax           [3]float32    ///< The maximum bounds in world space. [(x, y, z)]
	cs             float32       ///< The size of each cell. (On the xz-plane.)
	ch             float32       ///< The height of each cell. (The minimum increment along the y-axis.)
	cells          []CompactCell ///< Array of cells. [Size: #width*#height]
	spans          []CompactSpan ///< Array of spans. [Size: #spanCount]
	dist           []uint16      ///< Array containing border distance data. [Size: #spanCount]
	areas          []uint8       ///< Array containing area id data. [Size: #spanCount]
}
