package recast

import (
	"sort"

	"github.com/aurelien-rainone/assertgo"
)

func getCornerHeight(x, y, i, dir int32, chf *CompactHeightfield) (ch int32, isBorderVertex bool) {
	s := chf.spans[i]
	ch = int32(s.y)
	dirp := (dir + 1) & 0x3

	regs := [4]uint16{0, 0, 0, 0}

	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed.
	regs[0] = chf.spans[i].reg | uint16(chf.areas[i]<<16)

	if GetCon(s, dir) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := int32(chf.cells[ax+ay*chf.width].index) + GetCon(s, dir)
		as := chf.spans[ai]
		ch = iMax(ch, int32(as.y))
		regs[1] = chf.spans[ai].reg | uint16(chf.areas[ai]<<16)
		if GetCon(as, dirp) != RC_NOT_CONNECTED {
			ax2 := ax + GetDirOffsetX(dirp)
			ay2 := ay + GetDirOffsetY(dirp)
			ai2 := int32(chf.cells[ax2+ay2*chf.width].index) + GetCon(as, dirp)
			as2 := chf.spans[ai2]
			ch = iMax(ch, int32(as2.y))
			regs[2] = chf.spans[ai2].reg | uint16(chf.areas[ai2]<<16)
		}
	}
	if GetCon(s, dirp) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dirp)
		ay := y + GetDirOffsetY(dirp)
		ai := int32(chf.cells[ax+ay*chf.width].index) + GetCon(s, dirp)
		as := chf.spans[ai]
		ch = iMax(ch, int32(as.y))
		regs[3] = chf.spans[ai].reg | uint16(chf.areas[ai]<<16)
		if GetCon(as, dir) != RC_NOT_CONNECTED {
			ax2 := ax + GetDirOffsetX(dir)
			ay2 := ay + GetDirOffsetY(dir)
			ai2 := int32(chf.cells[ax2+ay2*chf.width].index) + GetCon(as, dir)
			as2 := chf.spans[ai2]
			ch = iMax(ch, int32(as2.y))
			regs[2] = chf.spans[ai2].reg | uint16(chf.areas[ai2]<<16)
		}
	}

	// Check if the vertex is special edge vertex, these vertices will be removed later.
	for j := int32(0); j < 4; j++ {
		a := j
		b := (j + 1) & 0x3
		c := (j + 2) & 0x3
		d := (j + 3) & 0x3

		// The vertex is a border vertex there are two same exterior cells in a row,
		// followed by two interior cells and none of the regions are out of bounds.
		twoSameExts := (regs[a]&regs[b]&RC_BORDER_REG) != 0 && regs[a] == regs[b]
		twoInts := ((regs[c] | regs[d]) & RC_BORDER_REG) == 0
		intsSameArea := (regs[c] >> 16) == (regs[d] >> 16)
		noZeros := regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0
		if twoSameExts && twoInts && intsSameArea && noZeros {
			isBorderVertex = true
			break
		}
	}

	return ch, isBorderVertex
}

/// Represents a simple, non-overlapping contour in field space.
type Contour struct {
	verts   []int32 ///< Simplified contour vertex and connection data. [Size: 4 * #nverts]
	nverts  int32   ///< The number of vertices in the simplified contour.
	rverts  []int32 ///< Raw contour vertex and connection data. [Size: 4 * #nrverts]
	nrverts int32   ///< The number of vertices in the raw contour.
	reg     uint16  ///< The region id of the contour.
	area    uint8   ///< The area id of the contour.
}

/// Represents a group of related contours.
/// @ingroup recast
type ContourSet struct {
	conts      []*Contour ///< An array of the contours in the set. [Size: #nconts]
	nconts     int32      ///< The number of contours in the set.
	bmin       [3]float32 ///< The minimum bounds in world space. [(x, y, z)]
	bmax       [3]float32 ///< The maximum bounds in world space. [(x, y, z)]
	cs         float32    ///< The size of each cell. (On the xz-plane.)
	ch         float32    ///< The height of each cell. (The minimum increment along the y-axis.)
	width      int32      ///< The width of the set. (Along the x-axis in cell units.)
	height     int32      ///< The height of the set. (Along the z-axis in cell units.)
	borderSize int32      ///< The AABB border size used to generate the source data from which the contours were derived.
	maxError   float32    ///< The max edge error that this contour set was simplified with.
}

func mergeRegionHoles(ctx *Context, region *ContourRegion) {
	// Sort holes from left to right.
	for i := int32(0); i < region.nholes; i++ {

		region.holes[i].minx, region.holes[i].minz, region.holes[i].leftmost = findLeftMostVertex(region.holes[i].contour)
	}

	//sort.Sort(region.holes, region.nholes, sizeof(rcContourHole), compareHoles);
	sort.Sort(compareHoles(region.holes))

	maxVerts := region.outline.nverts
	for i := int32(0); i < region.nholes; i++ {
		maxVerts += region.holes[i].contour.nverts
	}

	diags := make([]PotentialDiagonal, maxVerts)
	//if (!diags)
	//{
	//ctx.log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
	//return;
	//}

	outline := region.outline

	// Merge holes into the outline one by one.
	for i := int32(0); i < region.nholes; i++ {
		hole := region.holes[i].contour

		index := int32(-1)
		bestVertex := region.holes[i].leftmost
		for iter := int32(0); iter < hole.nverts; iter++ {
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			var ndiags int32
			corner := hole.verts[bestVertex*4:]
			for j := int32(0); j < outline.nverts; j++ {
				if inCone4(j, outline.nverts, outline.verts, corner) {
					dx := outline.verts[j*4+0] - corner[0]
					dz := outline.verts[j*4+2] - corner[2]
					diags[ndiags].vert = j
					diags[ndiags].dist = dx*dx + dz*dz
					ndiags++
				}
			}

			// Sort potential diagonals by distance, we want to make the connection as short as possible.
			//qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);
			sort.Sort(compareDiagDist(diags))

			// Find a diagonal that is not intersecting the outline not the remaining holes.
			index = -1
			for j := int32(0); j < ndiags; j++ {
				pt := outline.verts[diags[j].vert*4:]
				intersect := intersectSegCountour(pt, corner, diags[i].vert, outline.nverts, outline.verts)
				for k := i; k < region.nholes && !intersect; k++ {
					intersect = intersect || intersectSegCountour(pt, corner, -1, region.holes[k].contour.nverts, region.holes[k].contour.verts)
				}
				if !intersect {
					index = diags[j].vert
					break
				}
			}
			// If found non-intersecting diagonal, stop looking.
			if index != -1 {
				break
			}
			// All the potential diagonals for the current vertex were intersecting, try next vertex.
			bestVertex = (bestVertex + 1) % hole.nverts
		}

		if index == -1 {
			ctx.Warningf("mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole)
			continue
		}
		if !mergeContours(region.outline, hole, index, bestVertex) {
			ctx.Warningf("mergeHoles: Failed to merge contours %p and %p.", region.outline, hole)
			continue
		}
	}
}

/// @par
///
/// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
/// parameters control how closely the simplified contours will match the raw contours.
///
/// Simplified contours are generated such that the vertices for portals between areas match up.
/// (They are considered mandatory vertices.)
///
/// Setting @p maxEdgeLength to zero will disabled the edge length feature.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
func BuildContours(ctx *Context, chf *CompactHeightfield,
	maxError float32, maxEdgeLen int32,
	cset *ContourSet, buildFlags int32) bool {
	assert.True(ctx != nil, "ctx should not be nil")

	w := chf.width
	h := chf.height
	borderSize := chf.borderSize

	ctx.StartTimer(RC_TIMER_BUILD_CONTOURS)
	defer ctx.StopTimer(RC_TIMER_BUILD_CONTOURS)

	copy(cset.bmin[:], chf.bmin[:])
	copy(cset.bmax[:], chf.bmax[:])
	if borderSize > 0 {
		// If the heightfield was build with bordersize, remove the offset.
		pad := float32(borderSize) * chf.cs
		cset.bmin[0] += pad
		cset.bmin[2] += pad
		cset.bmax[0] -= pad
		cset.bmax[2] -= pad
	}
	cset.cs = chf.cs
	cset.ch = chf.ch
	cset.width = chf.width - chf.borderSize*2
	cset.height = chf.height - chf.borderSize*2
	cset.borderSize = chf.borderSize
	cset.maxError = maxError

	maxContours := iMax(int32(chf.maxRegions), 8)
	cset.conts = make([]*Contour, maxContours)
	for i := range cset.conts {
		cset.conts[i] = new(Contour)
	}
	//if (!cset.conts) {
	//return false;
	//}
	cset.nconts = 0

	flags := make([]uint8, chf.spanCount)
	//if (!flags)
	//{
	//ctx.log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount);
	//return false;
	//}

	ctx.StartTimer(RC_TIMER_BUILD_CONTOURS_TRACE)

	// Mark boundaries.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				var res uint8
				s := chf.spans[i]
				if (chf.spans[i].reg != 0) || ((chf.spans[i].reg & RC_BORDER_REG) != 0) {
					flags[i] = 0
					continue
				}
				for dir := int32(0); dir < 4; dir++ {
					var r uint16
					if GetCon(s, dir) != RC_NOT_CONNECTED {
						ax := x + GetDirOffsetX(dir)
						ay := y + GetDirOffsetY(dir)
						ai := int32(chf.cells[ax+ay*w].index) + GetCon(s, dir)
						r = chf.spans[ai].reg
					}
					if r == chf.spans[i].reg {
						res |= (1 << uint(dir))
					}
				}
				flags[i] = res ^ 0xf // Inverse, mark non connected edges.
			}
		}
	}

	ctx.StopTimer(RC_TIMER_BUILD_CONTOURS_TRACE)

	verts := make([]int32, 256)
	simplified := make([]int32, 64)

	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := chf.cells[x+y*w]
			i := int32(c.index)
			for ni := int32(c.index) + int32(c.count); i < ni; i++ {
				if flags[i] == 0 || flags[i] == 0xf {
					flags[i] = 0
					continue
				}
				reg := chf.spans[i].reg
				if (reg != 0) || ((reg & RC_BORDER_REG) != 0) {
					continue
				}
				area := chf.areas[i]

				verts = make([]int32, 0)
				simplified = make([]int32, 0)

				ctx.StartTimer(RC_TIMER_BUILD_CONTOURS_TRACE)
				walkContour2(x, y, i, chf, flags, &verts)
				ctx.StopTimer(RC_TIMER_BUILD_CONTOURS_TRACE)

				ctx.StartTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY)
				simplifyContour(&verts, &simplified, maxError, maxEdgeLen, buildFlags)
				removeDegenerateSegments(&simplified)
				ctx.StopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY)

				// Store region.contour remap info.
				// Create contour.
				if len(simplified)/4 >= 3 {
					if cset.nconts >= maxContours {
						// Allocate more contours.
						// This happens when a region has holes.
						oldMax := maxContours
						maxContours *= 2
						newConts := make([]*Contour, maxContours)
						for j := int32(0); j < cset.nconts; j++ {
							newConts[j] = cset.conts[j]
							// Reset source pointers to prevent data deletion.
							cset.conts[j].verts = make([]int32, 0)
							cset.conts[j].rverts = make([]int32, 0)
						}
						cset.conts = newConts

						ctx.Warningf("rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours)
					}

					cont := cset.conts[cset.nconts]
					cset.nconts++
					cont.nverts = int32(len(simplified) / 4)
					cont.verts = make([]int32, cont.nverts*4)
					//if (!cont.verts) {
					//ctx.Errorf("BuildContours: Out of memory 'verts' (%d).", cont.nverts);
					//return false;
					//}
					//memcpy(cont.verts, &simplified[0], sizeof(int)*cont.nverts*4);
					if borderSize > 0 {
						// If the heightfield was build with bordersize, remove the offset.
						for j := int32(0); j < cont.nverts; j++ {
							v := cont.verts[j*4:]
							v[0] -= borderSize
							v[2] -= borderSize
						}
					}

					cont.nrverts = int32(len(verts) / 4)
					cont.rverts = make([]int32, cont.nrverts*4)
					//if (!cont.rverts) {
					//ctx.Errorf("rcBuildContours: Out of memory 'rverts' (%d).", cont.nrverts);
					//return false;
					//}
					//memcpy(cont.rverts, &verts[0], sizeof(int)*cont.nrverts*4);
					if borderSize > 0 {
						// If the heightfield was build with bordersize, remove the offset.
						for j := int32(0); j < cont.nrverts; j++ {
							v := cont.rverts[j*4:]
							v[0] -= borderSize
							v[2] -= borderSize
						}
					}

					cont.reg = reg
					cont.area = area
				}
			}
		}
	}

	// Merge holes if needed.
	if cset.nconts > 0 {
		// Calculate winding of all polygons.
		//rcScopedDelete<char> winding((char*)rcAlloc(sizeof(char)*cset.nconts, RC_ALLOC_TEMP));
		winding := make([]uint8, cset.nconts)
		//if (!winding)
		//{
		//ctx.log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'hole' (%d).", cset.nconts);
		//return false;
		//}
		var nholes int32
		for i := int32(0); i < cset.nconts; i++ {
			cont := cset.conts[i]
			// If the contour is wound backwards, it is a hole.
			if calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 {
				// TODO: check that!
				//winding[i] = uint8(-1)
				winding[i] = 0xff
			} else {
				winding[i] = 1
			}
			if winding[i] < 0 {
				nholes++
			}
		}

		if nholes > 0 {
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			nregions := chf.maxRegions + 1

			regions := make([]ContourRegion, nregions)
			//if (!regions)
			//{
			//ctx.log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'regions' (%d).", nregions);
			//return false;
			//}
			//memset(regions, 0, sizeof(rcContourRegion)*nregions);

			holes := make([]*ContourHole, cset.nconts)
			//rcScopedDelete<rcContourHole> holes((rcContourHole*)rcAlloc(sizeof(rcContourHole)*cset.nconts, RC_ALLOC_TEMP));
			//if (!holes)
			//{
			//ctx.log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'holes' (%d).", cset.nconts);
			//return false;
			//}
			//memset(holes, 0, sizeof(rcContourHole)*cset.nconts);

			for i := int32(0); i < cset.nconts; i++ {
				cont := &cset.conts[i]
				// Positively would contours are outlines, negative holes.
				if winding[i] > 0 {
					if regions[(*cont).reg].outline != nil {
						ctx.Errorf("BuildContours: Multiple outlines for region %d.", (*cont).reg)
					}
					regions[(*cont).reg].outline = *cont
				} else {
					regions[(*cont).reg].nholes++
				}
			}
			index := int32(0)
			for i := uint16(0); i < nregions; i++ {
				if regions[i].nholes > 0 {
					regions[i].holes = holes[index:]
					index += regions[i].nholes
					regions[i].nholes = 0
				}
			}
			for i := int32(0); i < cset.nconts; i++ {
				cont := &cset.conts[i]
				reg := &regions[(*cont).reg]
				if winding[i] < 0 {
					reg.holes[reg.nholes].contour = *cont
					reg.nholes++
				}
			}

			// Finally merge each regions holes into the outline.
			for i := uint16(0); i < nregions; i++ {
				reg := &regions[i]
				if reg.nholes == 0 {
					continue
				}

				if reg.outline != nil {
					mergeRegionHoles(ctx, reg)
				} else {
					// The region does not have an outline.
					// This can happen if the contour becaomes selfoverlapping because of
					// too aggressive simplification settings.
					ctx.Errorf("BuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i)
				}
			}
		}

	}

	return true
}

func walkContour2(x, y, i int32,
	chf *CompactHeightfield,
	flags []uint8, points *[]int32) {
	// Choose the first non-connected edge
	var dir uint8
	for (flags[i] & (1 << dir)) == 0 {
		dir++
	}

	startDir := dir
	starti := i

	area := chf.areas[i]

	iter := int32(0)
	for iter+1 < 40000 {
		iter++
		if (flags[i] & (1 << uint(dir))) != 0 {
			// Choose the edge corner
			isBorderVertex := false
			isAreaBorder := false
			px := x
			py, isBorderVertex := getCornerHeight(x, y, i, int32(dir), chf)
			pz := y
			switch dir {
			case 0:
				pz++
				break
			case 1:
				px++
				pz++
				break
			case 2:
				px++
				break
			}
			r := int32(0)
			s := chf.spans[i]
			if GetCon(s, int32(dir)) != RC_NOT_CONNECTED {
				ax := x + GetDirOffsetX(int32(dir))
				ay := y + GetDirOffsetY(int32(dir))
				ai := int32(chf.cells[ax+ay*chf.width].index) + GetCon(s, int32(dir))
				r = int32(chf.spans[ai].reg)
				if area != chf.areas[ai] {
					isAreaBorder = true
				}
			}
			if isBorderVertex {
				r |= RC_BORDER_VERTEX
			}
			if isAreaBorder {
				r |= RC_AREA_BORDER
			}
			*points = append(*points, px, py, pz, r)

			flags[i] &= ^(1 << dir) // Remove visited edges
			dir = (dir + 1) & 0x3   // Rotate CW
		} else {
			ni := int32(-1)
			nx := x + GetDirOffsetX(int32(dir))
			ny := y + GetDirOffsetY(int32(dir))
			s := chf.spans[i]
			if GetCon(s, int32(dir)) != RC_NOT_CONNECTED {
				nc := chf.cells[nx+ny*chf.width]
				ni = int32(nc.index) + GetCon(s, int32(dir))
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
}

func distancePtSeg(x, z int32, px, pz, qx, qz int32) float32 {
	pqx := float32(qx - px)
	pqz := float32(qz - pz)
	dx := float32(x - px)
	dz := float32(z - pz)
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	dx = float32(px) + t*pqx - float32(x)
	dz = float32(pz) + t*pqz - float32(z)

	return dx*dx + dz*dz
}

func simplifyContour(points, simplified *[]int32,
	maxError float32, maxEdgeLen, buildFlags int32) {
	// Add initial points.
	hasConnections := false
	for i := 0; i < len(*points); i += 4 {
		if ((*points)[i+3] & RC_CONTOUR_REG_MASK) != 0 {
			hasConnections = true
			break
		}
	}

	if hasConnections {
		// The contour has some portals to other regions.
		// Add a new point to every location where the region changes.
		var i int
		for ni := len(*points) / 4; i < ni; i++ {
			ii := (i + 1) % ni
			differentRegs := ((*points)[i*4+3] & RC_CONTOUR_REG_MASK) != ((*points)[ii*4+3] & RC_CONTOUR_REG_MASK)
			areaBorders := ((*points)[i*4+3] & RC_AREA_BORDER) != ((*points)[ii*4+3] & RC_AREA_BORDER)
			if differentRegs || areaBorders {
				*simplified = append(*simplified, (*points)[i*4+0])
				*simplified = append(*simplified, (*points)[i*4+1])
				*simplified = append(*simplified, (*points)[i*4+2])
				*simplified = append(*simplified, int32(i))
			}
		}
	}

	if len(*simplified) == 0 {
		// If there is no connections at all,
		// create some initial points for the simplification process.
		// Find lower-left and upper-right vertices of the contour.
		llx := (*points)[0]
		lly := (*points)[1]
		llz := (*points)[2]
		lli := int32(0)
		urx := (*points)[0]
		ury := (*points)[1]
		urz := (*points)[2]
		uri := int32(0)
		for i := 0; i < len(*points); i += 4 {
			x := (*points)[i+0]
			y := (*points)[i+1]
			z := (*points)[i+2]
			if x < llx || (x == llx && z < llz) {
				llx = x
				lly = y
				llz = z
				lli = int32(i / 4)
			}
			if x > urx || (x == urx && z > urz) {
				urx = x
				ury = y
				urz = z
				uri = int32(i / 4)
			}
		}
		*simplified = append(*simplified, llx)
		*simplified = append(*simplified, lly)
		*simplified = append(*simplified, llz)
		*simplified = append(*simplified, lli)

		*simplified = append(*simplified, urx)
		*simplified = append(*simplified, ury)
		*simplified = append(*simplified, urz)
		*simplified = append(*simplified, uri)
	}

	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	pn := int32(len(*points) / 4)
	for i := 0; i < len(*simplified)/4; {
		ii := (i + 1) % (len(*simplified) / 4)

		ax := (*simplified)[i*4+0]
		az := (*simplified)[i*4+2]
		ai := (*simplified)[i*4+3]

		bx := (*simplified)[ii*4+0]
		bz := (*simplified)[ii*4+2]
		bi := (*simplified)[ii*4+3]

		// Find maximum deviation from the segment.
		var maxd float32
		maxi := int32(-1)
		var ci, cinc, endi int32

		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if bx > ax || (bx == ax && bz > az) {
			cinc = 1
			ci = (ai + cinc) % pn
			endi = bi
		} else {
			cinc = pn - 1
			ci = (bi + cinc) % pn
			endi = ai
			// TODO: check that
			//rcSwap(ax, bx)
			//rcSwap(az, bz)
			ax, bx = bx, ax
			az, bz = bz, az
		}

		// Tessellate only outer edges or edges between areas.
		if ((*points)[ci*4+3]&RC_CONTOUR_REG_MASK) == 0 ||
			(((*points)[ci*4+3] & RC_AREA_BORDER) != 0) {
			for ci != endi {
				d := distancePtSeg((*points)[ci*4+0], (*points)[ci*4+2], ax, az, bx, bz)
				if d > maxd {
					maxd = d
					maxi = ci
				}
				ci = (ci + cinc) % pn
			}
		}

		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if maxi != -1 && maxd > (maxError*maxError) {
			// Add space for the new point.
			// TODO: check that, original code:
			// simplified.resize(simplified.size()+4);
			*simplified = append(*simplified, make([]int32, 4)...)
			n := len(*simplified) / 4
			for j := n - 1; j > i; j-- {
				(*simplified)[j*4+0] = (*simplified)[(j-1)*4+0]
				(*simplified)[j*4+1] = (*simplified)[(j-1)*4+1]
				(*simplified)[j*4+2] = (*simplified)[(j-1)*4+2]
				(*simplified)[j*4+3] = (*simplified)[(j-1)*4+3]
			}
			// Add the point.
			(*simplified)[(i+1)*4+0] = (*points)[maxi*4+0]
			(*simplified)[(i+1)*4+1] = (*points)[maxi*4+1]
			(*simplified)[(i+1)*4+2] = (*points)[maxi*4+2]
			(*simplified)[(i+1)*4+3] = maxi
		} else {
			i++
		}
	}

	// Split too long edges.
	if maxEdgeLen > 0 && (buildFlags&(RC_CONTOUR_TESS_WALL_EDGES|RC_CONTOUR_TESS_AREA_EDGES)) != 0 {
		for i := 0; i < len(*simplified)/4; {
			ii := (i + 1) % (len(*simplified) / 4)

			ax := (*simplified)[i*4+0]
			az := (*simplified)[i*4+2]
			ai := (*simplified)[i*4+3]

			bx := (*simplified)[ii*4+0]
			bz := (*simplified)[ii*4+2]
			bi := (*simplified)[ii*4+3]

			// Find maximum deviation from the segment.
			maxi := int32(-1)
			ci := (ai + 1) % pn

			// Tessellate only outer edges or edges between areas.
			tess := false
			// Wall edges.
			if ((buildFlags & RC_CONTOUR_TESS_WALL_EDGES) != 0) && ((*points)[ci*4+3]&RC_CONTOUR_REG_MASK) == 0 {
				tess = true
			}
			// Edges between areas.
			if ((buildFlags & RC_CONTOUR_TESS_AREA_EDGES) != 0) && (((*points)[ci*4+3] & RC_AREA_BORDER) != 0) {
				tess = true
			}

			if tess {
				dx := bx - ax
				dz := bz - az
				if dx*dx+dz*dz > maxEdgeLen*maxEdgeLen {
					// Round based on the segments in lexilogical order so that the
					// max tesselation is consistent regardles in which direction
					// segments are traversed.
					var n int32
					if bi < ai {
						n = (bi + pn - ai)
					} else {
						n = (bi - ai)
					}
					if n > 1 {
						if bx > ax || (bx == ax && bz > az) {
							maxi = (ai + n/2) % pn
						} else {
							maxi = (ai + (n+1)/2) % pn
						}
					}
				}
			}

			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if maxi != -1 {
				// Add space for the new point.
				// TODO: check that, original code:
				// simplified.resize(simplified.size()+4);
				*simplified = append(*simplified, make([]int32, 4)...)

				n := len(*simplified) / 4
				for j := n - 1; j > i; j-- {
					(*simplified)[j*4+0] = (*simplified)[(j-1)*4+0]
					(*simplified)[j*4+1] = (*simplified)[(j-1)*4+1]
					(*simplified)[j*4+2] = (*simplified)[(j-1)*4+2]
					(*simplified)[j*4+3] = (*simplified)[(j-1)*4+3]
				}
				// Add the point.
				(*simplified)[(i+1)*4+0] = (*points)[maxi*4+0]
				(*simplified)[(i+1)*4+1] = (*points)[maxi*4+1]
				(*simplified)[(i+1)*4+2] = (*points)[maxi*4+2]
				(*simplified)[(i+1)*4+3] = maxi
			} else {
				i++
			}
		}
	}

	for i := 0; i < len(*simplified)/4; i++ {
		// The edge vertex flag is take from the current raw point,
		// and the neighbour region is take from the next raw point.
		ai := ((*simplified)[i*4+3] + 1) % pn
		bi := (*simplified)[i*4+3]
		(*simplified)[i*4+3] = ((*points)[ai*4+3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER)) | ((*points)[bi*4+3] & RC_BORDER_VERTEX)
	}

}

func calcAreaOfPolygon2D(verts []int32, nverts int32) int32 {
	var area, i int32
	for j := nverts - 1; i < nverts; i++ {
		vi := verts[i*4:]
		vj := verts[j*4:]
		area += vi[0]*vj[2] - vj[0]*vi[2]
		j = i
	}
	return (area + 1) / 2
}

func prev(i, n int32) int32 {
	if i-1 >= 0 {
		return i - 1
	}
	return n - 1
}

func next(i, n int32) int32 {
	if i+1 < n {
		return i + 1
	}
	return 0
}

func area2(a, b, c []int32) int32 {
	return (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
func xorb(x, y bool) bool {
	return x != y
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
func left(a, b, c []int32) bool {
	return area2(a, b, c) < 0
}

func leftOn(a, b, c []int32) bool {
	return area2(a, b, c) <= 0
}

func collinear(a, b, c []int32) bool {
	return area2(a, b, c) == 0
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
func intersectProp(a, b, c, d []int32) bool {
	// Eliminate improper cases.
	if collinear(a, b, c) || collinear(a, b, d) ||
		collinear(c, d, a) || collinear(c, d, b) {
		return false
	}

	return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b))
}

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segement ab.
func between(a, b, c []int32) bool {
	if !collinear(a, b, c) {
		return false
	}
	// If ab not vertical, check betweenness on x; else on y.
	if a[0] != b[0] {
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]))
	} else {
		return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]))
	}
}

// Returns true iff segments ab and cd intersect, properly or improperly.
func intersect(a, b, c, d []int32) bool {
	if intersectProp(a, b, c, d) {
		return true
	} else if between(a, b, c) || between(a, b, d) || between(c, d, a) || between(c, d, b) {
		return true
	}
	return false
}

func vequal(a, b []int32) bool {
	return a[0] == b[0] && a[2] == b[2]
}

func removeDegenerateSegments(simplified *[]int32) {
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	npts := int32(len(*simplified) / 4)
	for i := int32(0); i < npts; i++ {
		ni := next(i, npts)

		if vequal((*simplified)[i*4:], (*simplified)[ni*4:]) {
			// Degenerate segment, remove.
			for j := i; j < int32(len(*simplified)/4-1); j++ {
				(*simplified)[j*4+0] = (*simplified)[(j+1)*4+0]
				(*simplified)[j*4+1] = (*simplified)[(j+1)*4+1]
				(*simplified)[j*4+2] = (*simplified)[(j+1)*4+2]
				(*simplified)[j*4+3] = (*simplified)[(j+1)*4+3]
			}

			// TODO: check that, original code:
			// simplified.resize(simplified.size()-4);
			*simplified = (*simplified)[:len(*simplified)-4]
			npts--
		}
	}
}

// Finds the lowest leftmost vertex of a contour.
func findLeftMostVertex(contour *Contour) (minx, minz, leftmost int32) {
	minx = contour.verts[0]
	minz = contour.verts[2]
	leftmost = 0
	for i := int32(1); i < contour.nverts; i++ {
		x := contour.verts[i*4+0]
		z := contour.verts[i*4+2]
		if x < minx || (x == minx && z < minz) {
			minx = x
			minz = z
			leftmost = i
		}
	}
	return
}

func mergeContours(ca, cb *Contour, ia, ib int32) bool {
	maxVerts := ca.nverts + cb.nverts + 2
	verts := make([]int32, maxVerts*4)
	//if !verts {
	//return false
	//}

	var nv int32

	// Copy contour A.
	for i := int32(0); i <= ca.nverts; i++ {
		dst := verts[nv*4:]
		src := ca.verts[((ia+i)%ca.nverts)*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		nv++
	}

	// Copy contour B
	for i := int32(0); i <= cb.nverts; i++ {
		dst := verts[nv*4:]
		src := cb.verts[((ib+i)%cb.nverts)*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		nv++
	}

	//rcFree(ca.verts);
	ca.verts = verts
	ca.nverts = nv

	//rcFree(cb.verts);
	cb.verts = make([]int32, 0)
	cb.nverts = 0

	return true
}

type ContourRegion struct {
	outline *Contour
	holes   []*ContourHole
	nholes  int32
}

type ContourHole struct {
	contour              *Contour
	minx, minz, leftmost int32
}

type compareHoles []*ContourHole

// Len is the number of elements in the collection.
func (s compareHoles) Len() int {
	return len(s)
}

// Less reports whether the element with
// index a should sort before the element with index b.
func (s compareHoles) Less(i, j int) bool {

	a := s[i]
	b := s[j]
	if a.minx == b.minx {
		switch {
		case a.minz < b.minz:
			return true
		case a.minz > b.minz:
			return false
		}
	} else {
		switch {
		case a.minx < b.minx:
			return true
		case a.minx > b.minx:
			return false
		}
	}
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareHoles) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

type PotentialDiagonal struct {
	vert, dist int32
}

type compareDiagDist []PotentialDiagonal

// Len is the number of elements in the collection.
func (s compareDiagDist) Len() int {
	return len(s)
}

// Less reports whether the element with
// index a should sort before the element with index b.
func (s compareDiagDist) Less(i, j int) bool {
	a := s[i]
	b := s[j]

	switch {
	case a.dist < b.dist:
		return true
	case a.dist > b.dist:
		return false
	default:
		return false
	}
}

// Swap swaps the elements with indexes i and j.
func (s compareDiagDist) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

func intersectSegCountour(d0, d1 []int32, i, n int32, verts []int32) bool {
	// For each edge (k,k+1) of P
	for k := int32(0); k < n; k++ {
		k1 := next(k, n)
		// Skip edges incident to i.
		if i == k || i == k1 {
			continue
		}
		p0 := verts[k*4:]
		p1 := verts[k1*4:]
		if vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1) {
			continue
		}

		if intersect(d0, d1, p0, p1) {
			return true
		}
	}
	return false
}

func inCone4(i, n int32, verts, pj []int32) bool {
	pi := verts[i*4:]
	pi1 := verts[next(i, n)*4:]
	pin1 := verts[prev(i, n)*4:]

	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if leftOn(pin1, pi, pi1) {
		return left(pi, pj, pin1) && left(pj, pi, pi1)
	}
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1))
}
