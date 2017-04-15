package recast

import "github.com/aurelien-rainone/assertgo"

// BuildRegionsMonotone builds region data for the heightfield using simple
// monotone partitioning.
//
//  Arguments:
//   ctx             The build context to use during the operation.
//   chf             A populated compact heightfield.
//   borderSize      The size of the non-navigable border around the
//                   heightfield. [Limit: >=0] [Units: vx]
//   minRegionArea   The minimum number of cells allowed to form isolated island
//                   areas. [Limit: >=0] [Units: vx].
//   mergeRegionArea Any regions with a span count smaller than this value will,
//                   if possible, be merged with larger regions.
//                   [Limit: >=0] [Units: vx]
//
// Returns true if the operation completed successfully.
// Non-null regions will consist of connected, non-overlapping walkable spans
// that form a single contour.  Contours will form simple polygons.
//
// If multiple regions form an area that is smaller than `minRegionArea`, then
// all spans will be re-assigned to the zero (null) region.
//
// Partitioning can result in smaller than necessary regions. `mergeRegionArea`
// helps reduce unecessarily small regions.
//
// See the Config documentation for more information on the configuration
// parameters.
//
// The region data will be available via the CompactHeightfield.maxRegions and
// CompactSpan.reg fields.
//
// Warning: the distance field must be created using BuildDistanceField before
// attempting to build regions.
//
// see CompactHeightfield, CompactSpan, BuildDistanceField,
// BuildRegionsMonotone, Config
func BuildRegionsMonotone(ctx *BuildContext, chf *CompactHeightfield,
	borderSize, minRegionArea, mergeRegionArea int32) bool {
	ctx.StartTimer(TimerBuildRegions)
	defer ctx.StopTimer(TimerBuildRegions)

	w := chf.Width
	h := chf.Height
	id := uint16(1)

	srcReg := make([]uint16, chf.SpanCount)
	nsweeps := iMax(chf.Width, chf.Height)
	sweeps := make([]sweepSpan, nsweeps)

	// Mark border regions.
	if borderSize > 0 {
		// Make sure border will not overflow.
		bw := iMin(w, borderSize)
		bh := iMin(h, borderSize)
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(w-bw, w, 0, h, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(0, w, 0, bh, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(0, w, h-bh, h, id|borderReg, chf, srcReg)
		id++

		chf.BorderSize = borderSize
	}

	prev := make([]int32, 256)

	// Sweep one line at a time.
	for y := borderSize; y < h-borderSize; y++ {
		// Collect spans from this row.
		prev = make([]int32, id+1)
		//memset(&prev[0],0,sizeof(int)*id);
		rid := uint16(1)

		for x := borderSize; x < w-borderSize; x++ {
			c := &chf.Cells[x+y*w]

			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				s := &chf.Spans[i]
				if chf.Areas[i] == nullArea {
					continue
				}

				// -x
				previd := uint16(0)
				if GetCon(s, 0) != notConnected {
					ax := x + GetDirOffsetX(0)
					ay := y + GetDirOffsetY(0)
					ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, 0)
					if (srcReg[ai]&borderReg) == 0 && chf.Areas[i] == chf.Areas[ai] {

						previd = srcReg[ai]
					}
				}

				if previd == 0 {
					previd = rid
					rid++
					sweeps[previd].rid = previd
					sweeps[previd].ns = 0
					sweeps[previd].nei = 0
				}

				// -y
				if GetCon(s, 3) != notConnected {
					ax := x + GetDirOffsetX(3)
					ay := y + GetDirOffsetY(3)
					ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, 3)
					if (srcReg[ai] != 0) && (srcReg[ai]&borderReg) == 0 && chf.Areas[i] == chf.Areas[ai] {
						nr := uint16(srcReg[ai])
						if (sweeps[previd].nei == 0) || sweeps[previd].nei == nr {
							sweeps[previd].nei = nr
							sweeps[previd].ns++
							prev[nr]++
						} else {
							sweeps[previd].nei = RC_NULL_NEI
						}
					}
				}

				srcReg[i] = previd
			}
		}

		// Create unique ID.
		for i := uint16(1); i < rid; i++ {
			if sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 && prev[sweeps[i].nei] == int32(sweeps[i].ns) {
				sweeps[i].id = sweeps[i].nei
			} else {
				sweeps[i].id = id
				id++
			}
		}

		// Remap IDs
		for x := borderSize; x < w-borderSize; x++ {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				if srcReg[i] > 0 && srcReg[i] < rid {
					srcReg[i] = sweeps[srcReg[i]].id
				}
			}
		}
	}

	{
		ctx.StartTimer(TimerBuildRegionsFilter)

		// Merge regions and filter out small regions.
		overlaps := make([]int32, 0)
		chf.MaxRegions = id
		if !mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, &chf.MaxRegions, chf, srcReg, &overlaps) {
			return false
		}
		// Monotone partitioning does not generate overlapping regions.
		ctx.StopTimer(TimerBuildRegionsFilter)
	}

	// Store the result out.
	for i := int32(0); i < chf.SpanCount; i++ {
		chf.Spans[i].Reg = srcReg[i]
	}

	return true
}

// BuildRegions builds region data for the heightfield using watershed
// partitioning.
//
//  Arguments:
//   ctx             The build context to use during the operation.
//   chf             A populated compact heightfield.
//   borderSize      The size of the non-navigable border around the
//                   heightfield. [Limit: >=0] [Units: vx]
//   minRegionArea   The minimum number of cells allowed to form isolated island
//                   areas. [Limit: >=0] [Units: vx].
//   mergeRegionArea Any regions with a span count smaller than this value will,
//                   if possible, be merged with larger regions.
//                   [Limit: >=0] [Units: vx]
//
// Returns True if the operation completed successfully.
//
// Non-null regions will consist of connected, non-overlapping walkable spans
// that form a single contour.
// Contours will form simple polygons.
//
// If multiple regions form an area that is smaller than @p minRegionArea, then
// all spans will be re-assigned to the zero (null) region.
//
// Watershed partitioning can result in smaller than necessary regions,
// especially in diagonal corridors. mergeRegionArea helps reduce
// unecessarily small regions.
//
// See the Config documentation for more information on the configuration
// parameters.
//
// The region data will be available via the CompactHeightfield::maxRegions and
// CompactSpan::reg fields.
//
// Warning: The distance field must be created using BuildDistanceField
// before attempting to build regions.
//
// see CompactHeightfield, CompactSpan, BuildDistanceField, BuildRegionsMonotone, Config
func BuildRegions(ctx *BuildContext, chf *CompactHeightfield,
	borderSize, minRegionArea, mergeRegionArea int32) bool {

	panic("untested")
	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerBuildRegions)
	defer ctx.StopTimer(TimerBuildRegions)

	w := chf.Width
	h := chf.Height

	buf := make([]uint16, chf.SpanCount*4)
	ctx.StartTimer(TimerBuildRegionsWatershed)

	const (
		LOG_NB_STACKS = 3
		NB_STACKS     = 1 << LOG_NB_STACKS
	)
	var (
		lvlStacks [][]int32
		stack     []int32
		//visited []int32
	)

	lvlStacks = make([][]int32, NB_STACKS)
	for i := range lvlStacks {
		lvlStacks[i] = make([]int32, 1024)
	}
	stack = make([]int32, 2024)
	//visited = make([]int32, 2024)

	srcReg := buf[:]
	srcDist := buf[chf.SpanCount:]
	dstReg := buf[chf.SpanCount*2:]
	dstDist := buf[chf.SpanCount*3:]

	//memset(srcReg, 0, sizeof(unsigned short)*chf.spanCount);
	//memset(srcDist, 0, sizeof(unsigned short)*chf.spanCount);

	regionID := uint16(1)
	// original C code:
	// level := uint16(chf.maxDistance+1) & ~1;
	level := uint16(int(chf.MaxDistance+1) & int(^1))

	// TODO: Figure better formula, expandIters defines how much the
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
	//	const int expandIters = 4 + walkableRadius * 2;
	const expandIters int = 8

	if borderSize > 0 {
		// Make sure border will not overflow.
		bw := iMin(w, borderSize)
		bh := iMin(h, borderSize)

		// Paint regions
		paintRectRegion(0, bw, 0, h, regionID|borderReg, chf, srcReg)
		regionID++
		paintRectRegion(w-bw, w, 0, h, regionID|borderReg, chf, srcReg)
		regionID++
		paintRectRegion(0, w, 0, bh, regionID|borderReg, chf, srcReg)
		regionID++
		paintRectRegion(0, w, h-bh, h, regionID|borderReg, chf, srcReg)
		regionID++

		chf.BorderSize = borderSize
	}

	sID := -1
	for level > 0 {
		if level >= 2 {
			level = level - 2
		} else {
			level = 0
		}
		sID = (sID + 1) & (NB_STACKS - 1)

		//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if sID == 0 {
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks[:], 1)
		} else {
			// copy left overs from last level
			appendStacks(lvlStacks[sID-1], lvlStacks[sID], srcReg)
		}

		//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			ctx.StartTimer(TimerBuildRegionsExpand)

			// Expand current regions until no empty connected cells found.
			// TODO: CHECK THIS
			if swapped := expandRegions(expandIters, level, chf, &srcReg, &srcDist, &dstReg, &dstDist, &lvlStacks[sID], false); swapped {
				srcReg, dstReg = dstReg, srcReg
				srcDist, dstDist = dstDist, srcDist
			}
			ctx.StopTimer(TimerBuildRegionsExpand)
		}

		{
			ctx.StartTimer(TimerBuildRegionsFlood)

			// Mark new regions with IDs.
			for j := 0; j < len(lvlStacks[sID]); j += 3 {
				x := lvlStacks[sID][j]
				y := lvlStacks[sID][j+1]
				i := lvlStacks[sID][j+2]
				if i >= 0 && srcReg[i] == 0 {
					if floodRegion(x, y, i, level, regionID, chf, srcReg, srcDist, &stack) {
						if regionID == 0xFFFF {
							ctx.Errorf("rcBuildRegions: Region ID overflow")
							return false
						}

						regionID++
					}
				}
			}
			ctx.StopTimer(TimerBuildRegionsFlood)
		}
	}

	// Expand current regions until no empty connected cells found.
	if swapped := expandRegions(expandIters*8, 0, chf, &srcReg, &srcDist, &dstReg, &dstDist, &stack, true); swapped {
		srcReg, dstReg = dstReg, srcReg
		srcDist, dstDist = dstDist, srcDist
	}

	ctx.StartTimer(TimerBuildRegionsWatershed)

	{
		ctx.StartTimer(TimerBuildRegionsFilter)

		// Merge regions and filter out smalle regions.
		var overlaps []int32
		chf.MaxRegions = regionID
		if !mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, &chf.MaxRegions, chf, srcReg, &overlaps) {
			return false
		}

		// If overlapping regions were found during merging, split those regions.
		if len(overlaps) > 0 {
			ctx.Errorf("rcBuildRegions: %d overlapping regions.", len(overlaps))
		}
		ctx.StopTimer(TimerBuildRegionsFilter)
	}

	// Write the result out.
	for i := int32(0); i < chf.SpanCount; i++ {
		chf.Spans[i].Reg = srcReg[i]
	}

	return true
}

func paintRectRegion(minx, maxx, miny, maxy int32, regID uint16, chf *CompactHeightfield, srcReg []uint16) {
	w := chf.Width
	for y := miny; y < maxy; y++ {
		for x := minx; x < maxx; x++ {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				if chf.Areas[i] != nullArea {
					srcReg[i] = regID
				}
			}
		}
	}
}

func floodRegion(x, y, i int32,
	level, r uint16,
	chf *CompactHeightfield,
	srcReg, srcDist []uint16,
	stack *[]int32) bool {
	w := chf.Width
	area := chf.Areas[i]

	// Flood fill mark region.
	if stack == nil {
		(*stack) = make([]int32, 0)
	}
	*stack = append(*stack, x, y, i)
	srcReg[i] = r
	srcDist[i] = 0

	var (
		ci, cy, cx int32
		lev        uint16
		count      int32
	)
	if level >= 2 {
		lev = lev - 2
	}

	for len(*stack) > 0 {

		// pop 3 last elements
		ci, *stack = (*stack)[len(*stack)-1], (*stack)[:len(*stack)-1]
		cy, *stack = (*stack)[len(*stack)-1], (*stack)[:len(*stack)-1]
		cx, *stack = (*stack)[len(*stack)-1], (*stack)[:len(*stack)-1]

		cs := &chf.Spans[ci]

		// Check if any of the neighbours already have a valid region set.
		var (
			ar  uint16
			dir int32
		)
		dir = 0
		for ; dir < 4; dir++ {
			// 8 connected
			if GetCon(cs, dir) != notConnected {
				ax := cx + GetDirOffsetX(dir)
				ay := cy + GetDirOffsetY(dir)
				ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(cs, dir)
				if chf.Areas[ai] != area {
					continue
				}
				nr := srcReg[ai]
				if (nr & borderReg) != 0 {
					// Do not take borders into account.
					continue
				}
				if nr != 0 && nr != r {
					ar = nr
					break
				}

				as := &chf.Spans[ai]

				dir2 := int32((dir + 1) & 0x3)
				if GetCon(as, dir2) != notConnected {
					ax2 := ax + GetDirOffsetX(dir2)
					ay2 := ay + GetDirOffsetY(dir2)
					ai2 := int32(chf.Cells[ax2+ay2*w].Index) + GetCon(as, dir2)
					if chf.Areas[ai2] != area {
						continue
					}
					nr2 := srcReg[ai2]
					if nr2 != 0 && nr2 != r {
						ar = nr2
						break
					}
				}
			}
		}
		if ar != 0 {
			srcReg[ci] = 0
			continue
		}

		count++

		// Expand neighbours.
		for dir = 0; dir < 4; dir++ {
			if GetCon(cs, dir) != notConnected {
				ax := cx + GetDirOffsetX(dir)
				ay := cy + GetDirOffsetY(dir)
				ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(cs, dir)
				if chf.Areas[ai] != area {
					continue
				}
				if chf.Dist[ai] >= lev && srcReg[ai] == 0 {
					srcReg[ai] = r
					srcDist[ai] = 0
					*stack = append(*stack, ax, ay, ai)
				}
			}
		}
	}

	return count > 0
}

func expandRegions(maxIter int, level uint16,
	chf *CompactHeightfield,
	srcReg, srcDist, dstReg, dstDist *[]uint16,
	stack *[]int32, fillStack bool) (swapped bool) {

	panic("untested")
	swapped = false
	w := chf.Width
	h := chf.Height

	if fillStack {
		// Find cells revealed by the raised level.
		*stack = make([]int32, 0)
		for y := int32(0); y < h; y++ {
			for x := int32(0); x < w; x++ {
				c := &chf.Cells[x+y*w]
				i := int32(c.Index)
				for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
					if chf.Dist[i] >= level && (*srcReg)[i] == 0 && chf.Areas[i] != nullArea {
						*stack = append(*stack, x, y, i)
					}
				}
			}
		}
	} else {
		// use cells in the input stack
		// mark all cells which already have a region
		for j := 0; j < len(*stack); j += 3 {
			i := (*stack)[j+2]
			if (*srcReg)[i] != 0 {
				(*stack)[j+2] = -1
			}
		}
	}

	var iter int
	for len(*stack) > 0 {
		failed := 0

		copy(*dstReg, (*srcReg)[:chf.SpanCount])
		copy(*dstDist, (*srcDist)[:chf.SpanCount])

		for j := 0; j < len(*stack); j += 3 {
			x := (*stack)[j+0]
			y := (*stack)[j+1]
			i := (*stack)[j+2]
			if i < 0 {
				failed++
				continue
			}

			r := (*srcReg)[i]
			d2 := int32(0xffff)
			area := chf.Areas[i]
			s := &chf.Spans[i]
			var dir int32
			for dir = 0; dir < 4; dir++ {
				if GetCon(s, dir) == notConnected {
					continue
				}
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, dir)
				if chf.Areas[ai] != area {
					continue
				}
				if (*srcReg)[ai] > 0 && ((*srcReg)[ai]&borderReg) == 0 {
					if int32((*srcDist)[ai]+2) < int32(d2) {
						r = (*srcReg)[ai]
						d2 = int32((*srcDist)[ai] + 2)
					}
				}
			}
			if r != 0 {
				(*stack)[j+2] = -1 // mark as used
				(*dstReg)[i] = r
				(*dstDist)[i] = uint16(d2)
			} else {
				failed++
			}
		}

		*srcReg, *dstReg = *dstReg, *srcReg
		*srcDist, *dstDist = *dstDist, *srcDist
		swapped = !swapped

		if failed*3 == len(*stack) {
			break
		}

		if level > 0 {
			iter++
			if iter >= maxIter {
				break
			}
		}
	}

	return swapped
}

func sortCellsByLevel(startLevel uint16,
	chf *CompactHeightfield,
	srcReg []uint16,
	nbStacks uint32, stacks [][]int32,
	// the levels per stack (2 in our case) as a bit shift
	loglevelsPerStack uint16) {
	w := chf.Width
	h := chf.Height
	startLevel = startLevel >> loglevelsPerStack

	for j := uint32(0); j < nbStacks; j++ {
		stacks[j] = make([]int32, 0)
	}

	// put all cells in the level range into the appropriate stacks
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				if chf.Areas[i] == nullArea || srcReg[i] != 0 {
					continue
				}

				level := chf.Dist[i] >> loglevelsPerStack
				sID := startLevel - level
				if uint32(sID) >= nbStacks {
					continue
				}
				if sID < 0 {
					sID = 0
				}

				stacks[sID] = append(stacks[sID], x)
				stacks[sID] = append(stacks[sID], y)
				stacks[sID] = append(stacks[sID], i)
			}
		}
	}
}

func appendStacks(srcStack, dstStack []int32, srcReg []uint16) {
	for j := 0; j < len(srcStack); j += 3 {
		i := srcStack[j+2]
		if (i < 0) || (srcReg[i] != 0) {
			continue
		}
		dstStack = append(dstStack, srcStack[j:j+3]...)
	}
}

type Region struct {
	SpanCount        int32  // Number of spans belonging to this region
	ID               uint16 // ID of the region
	AreaType         uint8  // Are type.
	Remap, Visited   bool
	Overlap          bool
	ConnectsToBorder bool
	YMin, YMax       uint16
	Connections      []int32
	Floors           []int32
}

func newRegion(i int) *Region {
	return &Region{
		SpanCount:        0,
		ID:               uint16(i),
		AreaType:         0,
		Remap:            false,
		Visited:          false,
		Overlap:          false,
		ConnectsToBorder: false,
		YMin:             uint16(0xffff),
		YMax:             uint16(0),
	}
}

func (reg *Region) removeAdjacentNeighbours() {
	// Remove adjacent duplicates.
	for i := 0; i < len(reg.Connections) && len(reg.Connections) > 1; {
		ni := (i + 1) % len(reg.Connections)
		if reg.Connections[i] == reg.Connections[ni] {
			// Remove duplicate
			for j := i; j < len(reg.Connections)-1; j++ {
				reg.Connections[j] = reg.Connections[j+1]
			}
			// pop
			reg.Connections = reg.Connections[:len(reg.Connections)-1]
		} else {
			i++
		}
	}
}

func (reg *Region) replaceNeighbour(oldID, newID uint16) {
	var neiChanged bool

	for i := range reg.Connections {
		if reg.Connections[i] == int32(oldID) {
			reg.Connections[i] = int32(newID)
			neiChanged = true
		}
	}
	for i := range reg.Floors {
		if reg.Floors[i] == int32(oldID) {
			reg.Floors[i] = int32(newID)
		}
	}

	if neiChanged {
		reg.removeAdjacentNeighbours()
	}
}

func (reg *Region) canMergeWithRegion(reg2 *Region) bool {
	if reg.AreaType != reg2.AreaType {
		return false
	}
	var n int
	for i := 0; i < len(reg.Connections); i++ {
		if reg.Connections[i] == int32(reg2.ID) {
			n++
		}
	}
	if n > 1 {
		return false
	}
	for i := 0; i < len(reg.Floors); i++ {
		if reg.Floors[i] == int32(reg2.ID) {
			return false
		}
	}
	return true
}

func (reg *Region) addUniqueFloorRegion(n int32) {
	for i := 0; i < len(reg.Floors); i++ {
		if reg.Floors[i] == n {
			return
		}
	}
	reg.Floors = append(reg.Floors, n)
}

func mergeRegions(rega, regb *Region) bool {
	aid := rega.ID
	bid := regb.ID

	// Duplicate current neighbourhood.
	var acon []int32
	acon = make([]int32, len(rega.Connections))
	for i := 0; i < len(rega.Connections); i++ {
		acon[i] = rega.Connections[i]
	}
	bcon := regb.Connections

	// Find insertion point on A.
	insa := int32(-1)
	for i := 0; i < len(acon); i++ {
		if acon[i] == int32(bid) {
			insa = int32(i)
			break
		}
	}
	if insa == -1 {
		return false
	}

	// Find insertion point on B.
	insb := int32(-1)
	for i := 0; i < len(bcon); i++ {
		if bcon[i] == int32(aid) {
			insb = int32(i)
			break
		}
	}
	if insb == -1 {
		return false
	}

	// Merge neighbours.
	rega.Connections = make([]int32, 0)
	var i int32
	for ni := int32(len(acon)); i < ni-1; i++ {
		rega.Connections = append(rega.Connections, acon[(insa+1+int32(i))%ni])
	}

	i = 0
	for ni := int32(len(bcon)); i < ni-1; i++ {
		rega.Connections = append(rega.Connections, bcon[(insb+1+int32(i))%ni])
	}

	rega.removeAdjacentNeighbours()

	for j := 0; j < len(regb.Floors); j++ {
		rega.addUniqueFloorRegion(regb.Floors[j])
	}
	rega.SpanCount += regb.SpanCount
	regb.SpanCount = 0
	regb.Connections = make([]int32, 0)

	return true
}

func (reg *Region) isConnectedToBorder() bool {
	// Region is connected to border if
	// one of the neighbours is null id.
	for _, conn := range reg.Connections {
		if conn == 0 {
			return true
		}
	}
	return false
}

func isSolidEdge(chf *CompactHeightfield, srcReg []uint16,
	x, y, i, dir int32) bool {
	s := &chf.Spans[i]
	var r uint16
	if GetCon(s, dir) != notConnected {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(s, dir)
		r = srcReg[ai]
	}
	if r == srcReg[i] {
		return false
	}
	return true
}

func walkContour(x, y, i, dir int32,
	chf *CompactHeightfield,
	srcReg []uint16,
	cont *[]int32) {
	startDir := dir
	starti := i

	ss := &chf.Spans[i]
	var curReg uint16
	if GetCon(ss, dir) != notConnected {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(ss, dir)
		curReg = srcReg[ai]
	}
	*cont = append(*cont, int32(curReg))

	for iter := int32(1); iter < 39999; iter++ {
		s := &chf.Spans[i]

		if isSolidEdge(chf, srcReg, x, y, i, dir) {
			// Choose the edge corner
			var r uint16
			if GetCon(s, dir) != notConnected {
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(s, dir)
				r = srcReg[ai]
			}
			if r != curReg {
				curReg = r
				*cont = append(*cont, int32(curReg))
			}

			dir = (dir + 1) & 0x3 // Rotate CW
		} else {
			ni := int32(-1)
			nx := x + GetDirOffsetX(dir)
			ny := y + GetDirOffsetY(dir)
			if GetCon(s, dir) != notConnected {
				ni = int32(chf.Cells[nx+ny*chf.Width].Index) + GetCon(s, dir)
			}
			if ni == -1 {
				// Should not happen.
				return
			}
			x = nx
			y = ny
			i = ni
			dir = (dir + 3) & 0x3 // Rotate CCW
		}

		if starti == i && startDir == dir {
			break
		}
	}

	// Remove adjacent duplicates.
	if len(*cont) > 1 {
		for j := 0; j < len(*cont); {
			nj := (j + 1) % len(*cont)
			if (*cont)[j] == (*cont)[nj] {
				for k := j; k < len(*cont)-1; k++ {
					(*cont)[k] = (*cont)[k+1]
				}
				*cont = (*cont)[:len(*cont)-1]
			} else {
				j++
			}
		}
	}
}

func mergeAndFilterRegions(ctx *BuildContext,
	minRegionArea, mergeRegionSize int32,
	maxRegionID *uint16,
	chf *CompactHeightfield,
	srcReg []uint16,
	overlaps *[]int32) bool {

	w := chf.Width
	h := chf.Height

	nreg := (*maxRegionID) + 1
	regions := make([]*Region, nreg)

	// Construct regions
	for ridx := range regions {
		regions[ridx] = newRegion(ridx)
	}

	// Find edge of a region and find connections around the contour.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			i2 := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i2 < ni; i2++ {
				r := srcReg[i2]
				if r == 0 || r >= nreg {
					continue
				}

				reg := regions[r]
				reg.SpanCount++

				// Update floors.
				for j0 := int32(c.Index); j0 < ni; j0++ {
					if i2 == j0 {
						continue
					}
					floorID := srcReg[j0]
					if floorID == 0 || floorID >= nreg {
						continue
					}
					if floorID == r {
						reg.Overlap = true
					}
					reg.addUniqueFloorRegion(int32(floorID))
				}

				// Have found contour
				if len(reg.Connections) > 0 {
					continue
				}

				reg.AreaType = chf.Areas[i2]

				// Check if this cell is next to a border.
				ndir := int32(-1)
				for dir := int32(0); dir < 4; dir++ {
					if isSolidEdge(chf, srcReg, x, y, i2, dir) {
						ndir = dir
						break
					}
				}

				if ndir != -1 {
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i2, ndir, chf, srcReg, &reg.Connections)
				}
			}
		}
	}

	// Remove too small regions.
	stack := make([]int32, 32)
	trace := make([]int32, 32)
	for i3 := uint16(0); i3 < nreg; i3++ {
		reg := regions[i3]
		if reg.ID == 0 || ((reg.ID & borderReg) != 0) {
			continue
		}
		if reg.SpanCount == 0 {
			continue
		}
		if reg.Visited {
			continue
		}

		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		connectsToBorder := false
		spanCount := int32(0)
		stack = stack[:0]
		trace = trace[:0]

		reg.Visited = true
		stack = append(stack, int32(i3))

		for len(stack) > 0 {

			// Pop
			ri := stack[len(stack)-1]
			stack = stack[:len(stack)-1]

			creg := regions[ri]

			spanCount += creg.SpanCount
			trace = append(trace, ri)

			for j1 := 0; j1 < len(creg.Connections); j1++ {
				if (creg.Connections[j1] & int32(borderReg)) != 0 {
					connectsToBorder = true
					continue
				}
				neireg := regions[creg.Connections[j1]]
				if neireg.Visited {
					continue
				}
				if neireg.ID == 0 || ((neireg.ID & borderReg) != 0) {
					continue
				}
				// Visit
				stack = append(stack, int32(neireg.ID))
				neireg.Visited = true
			}
		}

		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if spanCount < minRegionArea && !connectsToBorder {
			// Kill all visited regions.
			for j2 := 0; j2 < len(trace); j2++ {
				regions[trace[j2]].SpanCount = 0
				regions[trace[j2]].ID = 0
			}
		}
	}

	// Merge too small regions to neighbour regions.
	mergeCount := 0
	for {
		mergeCount = 0
		for i4 := uint16(0); i4 < nreg; i4++ {
			reg := regions[i4]
			if reg.ID == 0 || ((reg.ID & borderReg) != 0) {
				continue
			}
			if reg.Overlap {
				continue
			}
			if reg.SpanCount == 0 {
				continue
			}

			// Check to see if the region should be merged.
			if reg.SpanCount > mergeRegionSize && reg.isConnectedToBorder() {
				continue
			}

			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			smallest := int32(0xfffffff)
			mergeID := uint16(reg.ID)
			for j3 := 0; j3 < len(reg.Connections); j3++ {
				if (reg.Connections[j3] & int32(borderReg)) != 0 {
					continue
				}
				mreg := regions[reg.Connections[j3]]
				if mreg.ID == 0 || ((mreg.ID & borderReg) != 0) || mreg.Overlap {
					continue
				}
				if mreg.SpanCount < smallest &&
					reg.canMergeWithRegion(mreg) &&
					mreg.canMergeWithRegion(reg) {
					smallest = mreg.SpanCount
					mergeID = mreg.ID
				}
			}
			// Found new id.
			if mergeID != reg.ID {
				oldID := reg.ID
				target := regions[mergeID]

				// Merge neighbours.
				if mergeRegions(target, reg) {

					// Fixup regions pointing to current region.
					for j4 := uint16(0); j4 < nreg; j4++ {
						if regions[j4].ID == 0 || ((regions[j4].ID & borderReg) != 0) {
							continue
						}
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if regions[j4].ID == oldID {
							regions[j4].ID = mergeID
						}
						// Replace the current region with the new one if the
						// current regions is neighbour.
						regions[j4].replaceNeighbour(oldID, mergeID)
					}
					mergeCount++
				}
			}
		}
		if mergeCount == 0 {
			break
		}
	}

	// Compress region Ids.
	for i5 := uint16(0); i5 < nreg; i5++ {
		regions[i5].Remap = false
		if regions[i5].ID == 0 {
			continue // Skip nil regions.
		}
		if (regions[i5].ID & borderReg) != 0 {
			continue // Skip external regions.
		}
		regions[i5].Remap = true
	}

	var regIDGen uint16
	for i6 := uint16(0); i6 < nreg; i6++ {
		if !regions[i6].Remap {
			continue
		}
		oldID := regions[i6].ID
		regIDGen++
		newID := regIDGen
		for j5 := i6; j5 < nreg; j5++ {
			if regions[j5].ID == oldID {
				regions[j5].ID = newID
				regions[j5].Remap = false
			}
		}
	}
	*maxRegionID = regIDGen

	// Remap regions.
	for i7 := int32(0); i7 < chf.SpanCount; i7++ {
		if (srcReg[i7] & borderReg) == 0 {
			srcReg[i7] = regions[srcReg[i7]].ID
		}
	}

	// Return regions that we found to be overlapping.
	for i8 := uint16(0); i8 < nreg; i8++ {
		if regions[i8].Overlap {
			*overlaps = append(*overlaps, int32(regions[i8].ID))
		}
	}

	for i9 := uint16(0); i9 < nreg; i9++ {
		regions[i9] = nil
	}
	return true
}

const RC_NULL_NEI uint16 = 0xffff

type sweepSpan struct {
	rid uint16 // row id
	id  uint16 // region id
	ns  uint16 // number samples
	nei uint16 // neighbour id
}
