package recast

import "github.com/aurelien-rainone/assertgo"

/// @par
///
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
///
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
///
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps
/// reduce unecessarily small regions.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
///
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
func BuildRegionsMonotone(ctx *Context, chf *CompactHeightfield,
	borderSize, minRegionArea, mergeRegionArea int32) bool {
	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(RC_TIMER_BUILD_REGIONS)
	defer ctx.StopTimer(RC_TIMER_BUILD_REGIONS)

	w := chf.width
	h := chf.height
	id := uint16(1)

	srcReg := make([]uint16, chf.spanCount)
	//if (!srcReg)
	//{
	//ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
	//return false;
	//}
	//memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

	nsweeps := iMax(chf.width, chf.height)
	sweeps := make([]sweepSpan, nsweeps)
	//rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	//if (!sweeps)
	//{
	//ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
	//return false;
	//}

	// Mark border regions.
	if borderSize > 0 {
		// Make sure border will not overflow.
		bw := iMin(w, borderSize)
		bh := iMin(h, borderSize)
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg)
		id++
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg)
		id++
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg)
		id++
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg)
		id++

		chf.borderSize = borderSize
	}

	prev := make([]int32, 256)

	// Sweep one line at a time.
	for y := borderSize; y < h-borderSize; y++ {
		// Collect spans from this row.
		prev = make([]int32, id+1)
		//memset(&prev[0],0,sizeof(int)*id);
		rid := uint16(1)

		for x := borderSize; x < w-borderSize; x++ {
			c := chf.cells[x+y*w]

			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				s := chf.spans[i]
				if chf.areas[i] == RC_NULL_AREA {
					continue
				}

				// -x
				previd := uint16(0)
				if GetCon(s, 0) != RC_NOT_CONNECTED {
					ax := x + GetDirOffsetX(0)
					ay := y + GetDirOffsetY(0)
					ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, 0)
					if (srcReg[ai]&RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai] {

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
				if GetCon(s, 3) != RC_NOT_CONNECTED {
					ax := x + GetDirOffsetX(3)
					ay := y + GetDirOffsetY(3)
					ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, 3)
					if (srcReg[ai] != 0) && (srcReg[ai]&RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai] {
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
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				if srcReg[i] > 0 && srcReg[i] < rid {
					srcReg[i] = sweeps[srcReg[i]].id
				}
			}
		}
	}

	{
		ctx.StartTimer(RC_TIMER_BUILD_REGIONS_FILTER)

		// Merge regions and filter out small regions.
		overlaps := make([]int32, 0)
		chf.maxRegions = id
		if !mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, &chf.maxRegions, chf, srcReg, &overlaps) {
			return false
		}

		// Monotone partitioning does not generate overlapping regions.
		ctx.StopTimer(RC_TIMER_BUILD_REGIONS_FILTER)
	}

	// Store the result out.
	for i := int32(0); i < chf.spanCount; i++ {
		chf.spans[i].reg = srcReg[i]
	}

	return true
}

/// @par
///
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
///
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
///
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors.
/// @p mergeRegionArea helps reduce unecessarily small regions.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
///
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
func BuildRegions(ctx *Context, chf *CompactHeightfield,
	borderSize, minRegionArea, mergeRegionArea int32) bool {

	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(RC_TIMER_BUILD_REGIONS)
	defer ctx.StopTimer(RC_TIMER_BUILD_REGIONS)

	w := chf.width
	h := chf.height

	buf := make([]uint16, chf.spanCount*4)
	ctx.StartTimer(RC_TIMER_BUILD_REGIONS_WATERSHED)

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
	srcDist := buf[chf.spanCount:]
	dstReg := buf[chf.spanCount*2:]
	dstDist := buf[chf.spanCount*3:]

	//memset(srcReg, 0, sizeof(unsigned short)*chf.spanCount);
	//memset(srcDist, 0, sizeof(unsigned short)*chf.spanCount);

	regionId := uint16(1)
	// original C code:
	// level := uint16(chf.maxDistance+1) & ~1;
	level := uint16(int(chf.maxDistance+1) & int(^1))

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
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++

		chf.borderSize = borderSize
	}

	sId := -1
	for level > 0 {
		if level >= 2 {
			level = level - 2
		} else {
			level = 0
		}
		sId = (sId + 1) & (NB_STACKS - 1)

		//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if sId == 0 {
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks[:], 1)
		} else {
			// copy left overs from last level
			appendStacks(lvlStacks[sId-1], lvlStacks[sId], srcReg)
		}

		//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			ctx.StartTimer(RC_TIMER_BUILD_REGIONS_EXPAND)

			// Expand current regions until no empty connected cells found.
			// TODO: CHECK THIS
			if swapped := expandRegions(expandIters, level, chf, &srcReg, &srcDist, &dstReg, &dstDist, &lvlStacks[sId], false); swapped {
				srcReg, dstReg = dstReg, srcReg
				srcDist, dstDist = dstDist, srcDist
			}
			ctx.StopTimer(RC_TIMER_BUILD_REGIONS_EXPAND)
		}

		{
			ctx.StartTimer(RC_TIMER_BUILD_REGIONS_FLOOD)

			// Mark new regions with IDs.
			for j := 0; j < len(lvlStacks[sId]); j += 3 {
				x := lvlStacks[sId][j]
				y := lvlStacks[sId][j+1]
				i := lvlStacks[sId][j+2]
				if i >= 0 && srcReg[i] == 0 {
					if floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, &stack) {
						if regionId == 0xFFFF {
							ctx.Errorf("rcBuildRegions: Region ID overflow")
							return false
						}

						regionId++
					}
				}
			}
			ctx.StopTimer(RC_TIMER_BUILD_REGIONS_FLOOD)
		}
	}

	// Expand current regions until no empty connected cells found.
	if swapped := expandRegions(expandIters*8, 0, chf, &srcReg, &srcDist, &dstReg, &dstDist, &stack, true); swapped {
		srcReg, dstReg = dstReg, srcReg
		srcDist, dstDist = dstDist, srcDist
	}

	ctx.StartTimer(RC_TIMER_BUILD_REGIONS_WATERSHED)

	{
		ctx.StartTimer(RC_TIMER_BUILD_REGIONS_FILTER)

		// Merge regions and filter out smalle regions.
		var overlaps []int32
		chf.maxRegions = regionId
		if !mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, &chf.maxRegions, chf, srcReg, &overlaps) {
			return false
		}

		// If overlapping regions were found during merging, split those regions.
		if len(overlaps) > 0 {
			ctx.Errorf("rcBuildRegions: %d overlapping regions.", len(overlaps))
		}
		ctx.StopTimer(RC_TIMER_BUILD_REGIONS_FILTER)
	}

	// Write the result out.
	for i := int32(0); i < chf.spanCount; i++ {
		chf.spans[i].reg = srcReg[i]
	}

	return true
}

func paintRectRegion(minx, maxx, miny, maxy int32, regId uint16, chf *CompactHeightfield, srcReg []uint16) {
	w := chf.width
	for y := miny; y < maxy; y++ {
		for x := minx; x < maxx; x++ {
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				if chf.areas[i] != RC_NULL_AREA {
					srcReg[i] = regId
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
	w := chf.width
	area := chf.areas[i]

	// Flood fill mark region.
	if stack == nil {
		(*stack) = make([]int32, 0)
	}
	*stack = append(*stack, x, y, i)
	srcReg[i] = r
	srcDist[i] = 0

	var (
		lev   uint16
		count int32
	)
	if level >= 2 {
		lev = lev - 2
	}

	for len(*stack) > 0 {

		// pop
		ci := (*stack)[len(*stack)-1]
		*stack = (*stack)[:len(*stack)-1]

		// pop
		cy := (*stack)[len(*stack)-1]
		*stack = (*stack)[:len(*stack)-1]

		// pop
		cx := (*stack)[len(*stack)-1]
		*stack = (*stack)[:len(*stack)-1]

		cs := chf.spans[ci]

		// Check if any of the neighbours already have a valid region set.
		var (
			ar  uint16
			dir int32
		)
		dir = 0
		for ; dir < 4; dir++ {
			// 8 connected
			if GetCon(cs, dir) != RC_NOT_CONNECTED {
				ax := cx + GetDirOffsetX(dir)
				ay := cy + GetDirOffsetY(dir)
				ai := int32(chf.cells[ax+ay*w].index) + GetCon(cs, dir)
				if chf.areas[ai] != area {
					continue
				}
				nr := srcReg[ai]
				if (nr & RC_BORDER_REG) != 0 {
					// Do not take borders into account.
					continue
				}
				if nr != 0 && nr != r {
					ar = nr
					break
				}

				as := chf.spans[ai]

				dir2 := int32((dir + 1) & 0x3)
				if GetCon(as, dir2) != RC_NOT_CONNECTED {
					ax2 := ax + GetDirOffsetX(dir2)
					ay2 := ay + GetDirOffsetY(dir2)
					ai2 := int32(chf.cells[ax2+ay2*w].index) + GetCon(as, dir2)
					if chf.areas[ai2] != area {
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
			if GetCon(cs, dir) != RC_NOT_CONNECTED {
				ax := cx + GetDirOffsetX(dir)
				ay := cy + GetDirOffsetY(dir)
				ai := int32(chf.cells[ax+ay*w].index) + GetCon(cs, dir)
				if chf.areas[ai] != area {
					continue
				}
				if chf.dist[ai] >= lev && srcReg[ai] == 0 {
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

	swapped = false
	w := chf.width
	h := chf.height

	if fillStack {
		// Find cells revealed by the raised level.
		*stack = make([]int32, 0)
		for y := int32(0); y < h; y++ {
			for x := int32(0); x < w; x++ {
				c := chf.cells[x+y*w]
				i := int32(c.index)
				for ni := int32(c.index) + int32(c.count); i < ni; i++ {
					if chf.dist[i] >= level && (*srcReg)[i] == 0 && chf.areas[i] != RC_NULL_AREA {
						*stack = append(*stack, x, y, i)
						//stack.push(x);
						//stack.push(y);
						//stack.push(i);
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

		copy(*dstReg, (*srcReg)[:chf.spanCount])
		copy(*dstDist, (*srcDist)[:chf.spanCount])
		//memcpy(dstReg, srcReg, sizeof(unsigned short)*chf.spanCount);
		//memcpy(dstDist, srcDist, sizeof(unsigned short)*chf.spanCount);

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
			area := chf.areas[i]
			s := chf.spans[i]
			var dir int32
			for dir = 0; dir < 4; dir++ {
				if GetCon(s, dir) == RC_NOT_CONNECTED {
					continue
				}
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, dir)
				if chf.areas[ai] != area {
					continue
				}
				if (*srcReg)[ai] > 0 && ((*srcReg)[ai]&RC_BORDER_REG) == 0 {
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

		// rcSwap source and dest.
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
	w := chf.width
	h := chf.height
	startLevel = startLevel >> loglevelsPerStack

	for j := uint32(0); j < nbStacks; j++ {
		stacks[j] = make([]int32, 0)
	}

	// put all cells in the level range into the appropriate stacks
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				if chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0 {
					continue
				}

				level := chf.dist[i] >> loglevelsPerStack
				sId := startLevel - level
				if uint32(sId) >= nbStacks {
					continue
				}
				if sId < 0 {
					sId = 0
				}

				stacks[sId] = append(stacks[sId], x)
				stacks[sId] = append(stacks[sId], y)
				stacks[sId] = append(stacks[sId], i)
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
		//dstStack.push(srcStack[j+1]);
		//dstStack.push(srcStack[j+2]);
	}
}

type Region struct {
	spanCount        int32  // Number of spans belonging to this region
	id               uint16 // ID of the region
	areaType         uint8  // Are type.
	remap, visited   bool
	overlap          bool
	connectsToBorder bool
	ymin, ymax       uint16
	connections      []int32
	floors           []int32
}

func newRegion(i uint16) *Region {
	return &Region{
		spanCount:        0,
		id:               i,
		areaType:         0,
		remap:            false,
		visited:          false,
		overlap:          false,
		connectsToBorder: false,
		ymin:             uint16(0xffff),
		ymax:             uint16(0),
	}
}

func (reg *Region) removeAdjacentNeighbours() {
	// Remove adjacent duplicates.
	for i := 0; i < len(reg.connections) && len(reg.connections) > 1; {
		ni := (i + 1) % len(reg.connections)
		if reg.connections[i] == reg.connections[ni] {
			// Remove duplicate
			for j := i; j < len(reg.connections)-1; j++ {
				reg.connections[j] = reg.connections[j+1]
			}
			// pop
			reg.connections = reg.connections[:len(reg.connections)-1]
		} else {
			i++
		}
	}
}

func (reg *Region) replaceNeighbour(oldId, newId uint16) {
	var neiChanged bool

	for i := range reg.connections {
		if reg.connections[i] == int32(oldId) {
			reg.connections[i] = int32(newId)
			neiChanged = true
		}
	}
	for i := range reg.floors {
		if reg.floors[i] == int32(oldId) {
			reg.floors[i] = int32(newId)
		}
	}

	if neiChanged {
		reg.removeAdjacentNeighbours()
	}
}

func (rega *Region) canMergeWithRegion(regb *Region) bool {
	if rega.areaType != regb.areaType {
		return false
	}
	var n int
	for i := 0; i < len(rega.connections); i++ {
		if rega.connections[i] == int32(regb.id) {
			n++
		}
	}
	if n > 1 {
		return false
	}
	for i := 0; i < len(rega.floors); i++ {
		if rega.floors[i] == int32(regb.id) {
			return false
		}
	}
	return true
}

func (reg *Region) addUniqueFloorRegion(n int32) {
	for i := 0; i < len(reg.floors); i++ {
		if reg.floors[i] == n {
			return
		}
	}
	reg.floors = append(reg.floors, n)
}

func (rega *Region) mergeRegions(regb *Region) bool {
	aid := rega.id
	bid := regb.id

	// Duplicate current neighbourhood.
	var acon []int32
	acon = make([]int32, len(rega.connections))
	// TODO: could use copy builtin here
	for i := 0; i < len(rega.connections); i++ {
		acon[i] = rega.connections[i]
	}
	bcon := regb.connections

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
	rega.connections = make([]int32, 0)
	var i int32
	for ni := int32(len(acon)); i < ni-1; i++ {
		//rega.connections.push(acon[(insa+1+i) % ni]);
		rega.connections = append(rega.connections, acon[(insa+1+int32(i))%ni])
	}

	i = 0
	for ni := int32(len(bcon)); i < ni-1; i++ {
		//rega.connections.push(bcon[(insb+1+i) % ni]);
		rega.connections = append(rega.connections, bcon[(insb+1+int32(i))%ni])
	}

	rega.removeAdjacentNeighbours()

	for j := 0; j < len(regb.floors); j++ {
		rega.addUniqueFloorRegion(regb.floors[j])
	}
	rega.spanCount += regb.spanCount
	regb.spanCount = 0
	regb.connections = make([]int32, 0)

	return true
}

func (reg *Region) isConnectedToBorder() bool {
	// Region is connected to border if
	// one of the neighbours is null id.
	for _, conn := range reg.connections {
		if conn == 0 {
			return true
		}
	}
	return false
}

func isSolidEdge(chf *CompactHeightfield, srcReg []uint16,
	x, y, i, dir int32) bool {
	s := chf.spans[i]
	var r uint16
	if GetCon(s, dir) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := int32(chf.cells[ax+ay*chf.width].index) + GetCon(s, dir)
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

	ss := chf.spans[i]
	var curReg uint16
	if GetCon(ss, dir) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := int32(chf.cells[ax+ay*chf.width].index) + GetCon(ss, dir)
		curReg = srcReg[ai]
	}
	*cont = append(*cont, int32(curReg))

	// TODO: check!
	// oriinal code: while (++iter < 40000)
	for iter := int32(0); iter+1 < 40000; {
		iter++
		s := chf.spans[i]

		if isSolidEdge(chf, srcReg, x, y, i, dir) {
			// Choose the edge corner
			var r uint16
			if GetCon(s, dir) != RC_NOT_CONNECTED {
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := int32(chf.cells[ax+ay*chf.width].index) + GetCon(s, dir)
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
			if GetCon(s, dir) != RC_NOT_CONNECTED {
				nc := chf.cells[nx+ny*chf.width]
				ni = int32(nc.index) + GetCon(s, dir)
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
				//cont.pop()
				*cont = (*cont)[:len(*cont)-1]
			} else {
				j++
			}
		}
	}
}

func mergeAndFilterRegions(ctx *Context, minRegionArea, mergeRegionSize int32,
	maxRegionId *uint16,
	chf *CompactHeightfield,
	srcReg []uint16, overlaps *[]int32) bool {
	w := chf.width
	h := chf.height

	nreg := (*maxRegionId) + 1
	regions := make([]*Region, nreg)
	//if (!regions)
	//{
	//ctx->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
	//return false;
	//}

	// Construct regions
	for i := uint16(0); i < nreg; i++ {
		regions[i] = newRegion(i)
	}

	// Find edge of a region and find connections around the contour.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				r := srcReg[i]
				if r == 0 || r >= nreg {
					continue
				}

				reg := regions[r]
				reg.spanCount++

				// Update floors.
				for j := int32(c.index); j < ni; j++ {
					if i == j {
						continue
					}
					floorId := srcReg[j]
					if floorId == 0 || floorId >= nreg {
						continue
					}
					if floorId == r {
						reg.overlap = true
					}
					reg.addUniqueFloorRegion(int32(floorId))
				}

				// Have found contour
				if len(reg.connections) > 0 {
					continue
				}

				reg.areaType = chf.areas[i]

				// Check if this cell is next to a border.
				ndir := int32(-1)
				for dir := int32(0); dir < 4; dir++ {
					if isSolidEdge(chf, srcReg, x, y, i, dir) {
						ndir = dir
						break
					}
				}

				if ndir != -1 {
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i, ndir, chf, srcReg, &reg.connections)
				}
			}
		}
	}

	// Remove too small regions.
	stack := make([]int32, 32)
	trace := make([]int32, 32)
	for i := uint16(0); i < nreg; i++ {
		reg := regions[i]
		if reg.id == 0 || ((reg.id & RC_BORDER_REG) != 0) {
			continue
		}
		if reg.spanCount == 0 {
			continue
		}
		if reg.visited {
			continue
		}

		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		connectsToBorder := false
		spanCount := int32(0)
		stack = stack[:0]
		trace = stack[:0]

		reg.visited = true
		stack = append(stack, int32(i))

		for len(stack) > 0 {

			// Pop
			ri := stack[len(stack)-1]
			stack = stack[:len(stack)-1]

			creg := regions[ri]

			spanCount += creg.spanCount
			trace = append(trace, ri)

			for j := 0; j < len(creg.connections); j++ {
				if (creg.connections[j] & int32(RC_BORDER_REG)) != 0 {
					connectsToBorder = true
					continue
				}
				neireg := regions[creg.connections[j]]
				if neireg.visited {
					continue
				}
				if neireg.id == 0 || ((neireg.id & RC_BORDER_REG) != 0) {
					continue
				}
				// Visit
				stack = append(stack, int32(neireg.id))
				neireg.visited = true
			}
		}

		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if spanCount < minRegionArea && !connectsToBorder {
			// Kill all visited regions.
			for j := 0; j < len(trace); j++ {
				regions[trace[j]].spanCount = 0
				regions[trace[j]].id = 0
			}
		}
	}

	// Merge too small regions to neighbour regions.
	mergeCount := 0
	for {
		mergeCount = 0
		for i := uint16(0); i < nreg; i++ {
			reg := regions[i]
			if reg.id == 0 || ((reg.id & RC_BORDER_REG) != 0) {
				continue
			}
			if reg.overlap {
				continue
			}
			if reg.spanCount == 0 {
				continue
			}

			// Check to see if the region should be merged.
			if reg.spanCount > mergeRegionSize && reg.isConnectedToBorder() {
				continue
			}

			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			smallest := int32(0xfffffff)
			mergeId := uint16(reg.id)
			for j := 0; j < len(reg.connections); j++ {
				if (reg.connections[j] & int32(RC_BORDER_REG)) != 0 {
					continue
				}
				mreg := regions[reg.connections[j]]
				if mreg.id == 0 || ((mreg.id & RC_BORDER_REG) != 0) || mreg.overlap {
					continue
				}
				if mreg.spanCount < smallest &&
					reg.canMergeWithRegion(mreg) &&
					mreg.canMergeWithRegion(reg) {
					smallest = mreg.spanCount
					mergeId = mreg.id
				}
			}
			// Found new id.
			if mergeId != reg.id {
				oldId := reg.id
				target := regions[mergeId]

				// Merge neighbours.
				if target.mergeRegions(reg) {
					// Fixup regions pointing to current region.
					for j := uint16(0); j < nreg; j++ {
						if regions[j].id == 0 || ((regions[j].id & RC_BORDER_REG) != 0) {
							continue
						}
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if regions[j].id == oldId {
							regions[j].id = mergeId
						}
						// Replace the current region with the new one if the
						// current regions is neighbour.
						regions[j].replaceNeighbour(oldId, mergeId)
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
	for i := uint16(0); i < nreg; i++ {
		regions[i].remap = false
		if regions[i].id == 0 {
			continue // Skip nil regions.
		}
		if (regions[i].id & RC_BORDER_REG) != 0 {
			continue // Skip external regions.
		}
		regions[i].remap = true
	}

	var regIdGen uint16
	for i := uint16(0); i < nreg; i++ {
		if !regions[i].remap {
			continue
		}
		oldId := regions[i].id
		regIdGen++
		newId := regIdGen
		for j := i; j < nreg; j++ {
			if regions[j].id == oldId {
				regions[j].id = newId
				regions[j].remap = false
			}
		}
	}
	*maxRegionId = regIdGen

	// Remap regions.
	for i := int32(0); i < chf.spanCount; i++ {
		if (srcReg[i] & RC_BORDER_REG) == 0 {
			srcReg[i] = regions[srcReg[i]].id
		}
	}

	// Return regions that we found to be overlapping.
	for i := uint16(0); i < nreg; i++ {
		if regions[i].overlap {
			*overlaps = append(*overlaps, int32(regions[i].id))
		}
	}

	for i := uint16(0); i < nreg; i++ {
		regions[i] = nil
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
