package crowd

import (
	"math"

	"github.com/arl/go-detour/detour"
	"github.com/arl/gogeo/f32/d3"
)

const (
	maxLocalSegs  = 8
	maxLocalPolys = 16
)

type segment struct {
	s [6]float32 // Segment start/end
	d float32    // Distance for pruning.
}

type LocalBoundary struct {
	center [3]float32
	segs   [maxLocalSegs]segment
	nsegs  int

	polys  [maxLocalPolys]detour.PolyRef
	npolys int
}

func NewLocalBoundary() *LocalBoundary {
	lb := &LocalBoundary{}
	lb.reset()
	return lb
}

func (lb *LocalBoundary) addSegment(dist float32, s []float32) {
	// Insert neighbour based on the distance.
	var seg *segment = nil
	if lb.nsegs == 0 {
		// First, trivial accept.
		seg = &lb.segs[0]
	} else if dist >= lb.segs[lb.nsegs-1].d {
		// Further than the last segment, skip.
		if lb.nsegs >= maxLocalSegs {
			return
		}
		// Last, trivial accept.
		seg = &lb.segs[lb.nsegs]
	} else {
		// Insert inbetween.
		var i int
		for i = 0; i < lb.nsegs; i++ {
			if dist <= lb.segs[i].d {
				break
			}
		}
		var (
			tgt = i + 1
			n   int
		)

		if lb.nsegs-i < maxLocalSegs-tgt {
			n = lb.nsegs - 1
		} else {
			n = maxLocalSegs - tgt
		}
		if tgt+n > maxLocalSegs {
			panic("should have tgt+n <= maxLocalSegs")
		}
		if n > 0 {
			copy(lb.segs[tgt:], lb.segs[i:i+n])
		}
		seg = &lb.segs[i]
	}

	seg.d = dist
	copy(seg.s[:], s[:6])

	if lb.nsegs < maxLocalSegs {
		lb.nsegs++
	}
}

func (lb *LocalBoundary) reset() {
	lb.center[0] = math.MaxFloat32
	lb.center[1] = math.MaxFloat32
	lb.center[2] = math.MaxFloat32
	lb.npolys = 0
	lb.nsegs = 0
}

func (lb *LocalBoundary) update(ref detour.PolyRef, pos d3.Vec3, collisionQueryRange float32, navquery *detour.NavMeshQuery, filter detour.QueryFilter) {
	const maxSegsPerPoly = detour.VertsPerPolygon * 3

	if ref == 0 {
		lb.center[0] = math.MaxFloat32
		lb.center[1] = math.MaxFloat32
		lb.center[2] = math.MaxFloat32
		lb.nsegs = 0
		lb.npolys = 0
		return
	}

	copy(lb.center[:], pos)

	// First query non-overlapping polygons.
	navquery.FindLocalNeighbourhood(ref, pos, collisionQueryRange,
		filter, lb.polys[:], nil, &lb.npolys, maxLocalPolys)

	// Secondly, store all polygon edges.
	lb.nsegs = 0
	var (
		segs  [maxSegsPerPoly * 6]float32
		nsegs = 0
	)
	for j := 0; j < lb.npolys; j++ {
		navquery.PolyWallSegments(lb.polys[j], filter, segs[:], nil, &nsegs, maxSegsPerPoly)
		for k := 0; k < nsegs; k++ {
			s := segs[k*6:]
			// Skip too distant segments.
			distSqr, _ := detour.DistancePtSegSqr2D(pos, s, s[3:])
			if distSqr > collisionQueryRange*collisionQueryRange {
				continue
			}
			lb.addSegment(distSqr, s)
		}
	}
}

func (lb *LocalBoundary) isValid(navquery *detour.NavMeshQuery, filter detour.QueryFilter) bool {
	if lb.npolys == 0 {
		return false
	}

	// Check that all polygons still pass query filter.
	for i := 0; i < lb.npolys; i++ {
		if !navquery.IsValidPolyRef(lb.polys[i], filter) {
			return false
		}
	}
	return true
}

func (lb *LocalBoundary) Center() d3.Vec3 {
	return lb.center[:]
}

func (lb *LocalBoundary) SegmentCount() int {
	return lb.nsegs
}

func (lb *LocalBoundary) Segment(i int) []float32 {
	return lb.segs[i].s[:]
}
