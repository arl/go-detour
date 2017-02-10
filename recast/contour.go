package recast

import (
	"sort"

	"github.com/aurelien-rainone/assertgo"
)

func cornerHeight(x, y, i, dir int32, chf *CompactHeightfield) (ch int32, isBorderVertex bool) {
	s := &chf.Spans[i]
	ch = int32(s.Y)
	dirp := (dir + 1) & 0x3

	regs := [4]uint16{0, 0, 0, 0}

	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed.
	regs[0] = uint16(uint32(chf.Spans[i].Reg) | (uint32(chf.Areas[i]) << 16))

	if GetCon(s, dir) != notConnected {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(s, dir)
		as := &chf.Spans[ai]
		ch = iMax(ch, int32(as.Y))
		regs[1] = uint16(uint32(chf.Spans[ai].Reg) | (uint32(chf.Areas[ai]) << 16))
		if GetCon(as, dirp) != notConnected {
			ax2 := ax + GetDirOffsetX(dirp)
			ay2 := ay + GetDirOffsetY(dirp)
			ai2 := int32(chf.Cells[ax2+ay2*chf.Width].Index) + GetCon(as, dirp)
			ch = iMax(ch, int32(chf.Spans[ai2].Y))
			regs[2] = uint16(uint32(chf.Spans[ai2].Reg) | (uint32(chf.Areas[ai2]) << 16))
		}
	}
	if GetCon(s, dirp) != notConnected {
		ax := x + GetDirOffsetX(dirp)
		ay := y + GetDirOffsetY(dirp)
		ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(s, dirp)
		as := &chf.Spans[ai]
		ch = iMax(ch, int32(as.Y))
		regs[3] = uint16(uint32(chf.Spans[ai].Reg) | (uint32(chf.Areas[ai]) << 16))
		if GetCon(as, dir) != notConnected {
			ax2 := ax + GetDirOffsetX(dir)
			ay2 := ay + GetDirOffsetY(dir)
			ai2 := int32(chf.Cells[ax2+ay2*chf.Width].Index) + GetCon(as, dir)
			ch = iMax(ch, int32(chf.Spans[ai2].Y))
			regs[2] = uint16(uint32(chf.Spans[ai2].Reg) | (uint32(chf.Areas[ai2]) << 16))
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
		twoSameExts := (regs[a]&regs[b]&borderReg) != 0 && regs[a] == regs[b]
		twoInts := ((regs[c] | regs[d]) & borderReg) == 0
		intsSameArea := (uint32(regs[c]) >> 16) == (uint32(regs[d]) >> 16)
		noZeros := regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0
		if twoSameExts && twoInts && intsSameArea && noZeros {
			isBorderVertex = true
			break
		}
	}

	return ch, isBorderVertex
}

// Contour represents a simple, non-overlapping contour in field space.
type Contour struct {
	Verts   []int32 // Simplified contour vertex and connection data. [Size: 4 * #nverts]
	NVerts  int32   // The number of vertices in the simplified contour.
	RVerts  []int32 // Raw contour vertex and connection data. [Size: 4 * #nrverts]
	NRVerts int32   // The number of vertices in the raw contour.
	Reg     uint16  // The region id of the contour.
	Area    uint8   // The area id of the contour.
}

// ContourSet represents a group of related contours.
type ContourSet struct {
	Conts      []Contour  // An array of the contours in the set. [Size: #nconts]
	NConts     int32      // The number of contours in the set.
	BMin       [3]float32 // The minimum bounds in world space. [(x, y, z)]
	BMax       [3]float32 // The maximum bounds in world space. [(x, y, z)]
	Cs         float32    // The size of each cell. (On the xz-plane.)
	Ch         float32    // The height of each cell. (The minimum increment along the y-axis.)
	Width      int32      // The width of the set. (Along the x-axis in cell units.)
	Height     int32      // The height of the set. (Along the z-axis in cell units.)
	BorderSize int32      // The AABB border size used to generate the source data from which the contours were derived.
	MaxError   float32    // The max edge error that this contour set was simplified with.
}

func mergeRegionHoles(ctx *BuildContext, region *contourRegion) {
	// Sort holes from left to right.
	for i := int32(0); i < region.nholes; i++ {
		region.holes[i].minx, region.holes[i].minz, region.holes[i].leftmost = findLeftMostVertex(region.holes[i].contour)
	}

	sort.Sort(compareHoles(region.holes))

	maxVerts := region.outline.NVerts
	for i := int32(0); i < region.nholes; i++ {
		maxVerts += region.holes[i].contour.NVerts
	}

	diags := make([]potentionalDiagonal, maxVerts)

	outline := region.outline

	// Merge holes into the outline one by one.
	for i := int32(0); i < region.nholes; i++ {
		hole := region.holes[i].contour

		index := int32(-1)
		bestVertex := region.holes[i].leftmost
		for iter := int32(0); iter < hole.NVerts; iter++ {
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			var ndiags int32
			corner := hole.Verts[bestVertex*4:]
			for j := int32(0); j < outline.NVerts; j++ {
				if inCone4(j, outline.NVerts, outline.Verts, corner) {
					dx := outline.Verts[j*4+0] - corner[0]
					dz := outline.Verts[j*4+2] - corner[2]
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
				pt := outline.Verts[diags[j].vert*4:]
				intersect := intersectSegCountour(pt, corner, diags[i].vert, outline.NVerts, outline.Verts)
				for k := i; k < region.nholes && !intersect; k++ {
					intersect = intersect || intersectSegCountour(pt, corner, -1, region.holes[k].contour.NVerts, region.holes[k].contour.Verts)
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
			bestVertex = (bestVertex + 1) % hole.NVerts
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

// BuildContours builds a contour set from the region outlines in the provided
// compact heightfield.
//
//  Arguments:
//   ctx         The build context to use during the operation.
//   chf         A fully built compact heightfield.
//   maxError    The maximum distance a simplfied contour's border edges should
//               deviate the original raw contour. [Limit: >=0] [Units: wu]
//   maxEdgeLen  The maximum allowed length for contour edges along the border
//               of the mesh. [Limit: >=0] [Units: vx]
//   cset        The resulting contour set. (Must be pre-allocated.)
//   buildFlags  The build flags. (See: BuildContoursFlags)
//
//  Returns true if the operation completed successfully.
//
// The raw contours will match the region outlines exactly. The maxError and
// maxEdgeLen parameters control how closely the simplified contours will match
// the raw contours.
// Simplified contours are generated such that the vertices for portals between
// areas match up. (They are considered mandatory vertices.)
//
// Setting maxEdgeLength to zero will disabled the edge length feature.
//
// See the Config documentation for more information on the configuration
// parameters.
//
// see CompactHeightfield, ContourSet, Config
func BuildContours(ctx *BuildContext, chf *CompactHeightfield,
	maxError float32, maxEdgeLen int32,
	cset *ContourSet, buildFlags int32) bool {
	assert.True(ctx != nil, "ctx should not be nil")

	w := chf.Width
	h := chf.Height
	borderSize := chf.BorderSize

	ctx.StartTimer(TimerBuildContours)
	defer ctx.StopTimer(TimerBuildContours)

	copy(cset.BMin[:], chf.BMin[:])
	copy(cset.BMax[:], chf.BMax[:])
	if borderSize > 0 {
		// If the heightfield was build with bordersize, remove the offset.
		pad := float32(borderSize) * chf.Cs
		cset.BMin[0] += pad
		cset.BMin[2] += pad
		cset.BMax[0] -= pad
		cset.BMax[2] -= pad
	}
	cset.Cs = chf.Cs
	cset.Ch = chf.Ch
	cset.Width = chf.Width - chf.BorderSize*2
	cset.Height = chf.Height - chf.BorderSize*2
	cset.BorderSize = chf.BorderSize
	cset.MaxError = maxError

	maxContours := iMax(int32(chf.MaxRegions), 8)
	cset.Conts = make([]Contour, maxContours)
	cset.NConts = 0

	flags := make([]uint8, chf.SpanCount)

	ctx.StartTimer(TimerBuildContoursTrace)

	// Mark boundaries.
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				var res uint8
				s := &chf.Spans[i]
				if (s.Reg == 0) || ((s.Reg & borderReg) != 0) {
					flags[i] = 0
					continue
				}
				for dir := int32(0); dir < 4; dir++ {
					var r uint16
					if GetCon(s, dir) != notConnected {
						ax := x + GetDirOffsetX(dir)
						ay := y + GetDirOffsetY(dir)
						ai := int32(chf.Cells[ax+ay*w].Index) + GetCon(s, dir)
						r = chf.Spans[ai].Reg
					}
					if r == chf.Spans[i].Reg {
						res |= (1 << uint(dir))
					}
				}
				flags[i] = res ^ 0xf // Inverse, mark non connected edges.
			}
		}
	}

	ctx.StopTimer(TimerBuildContoursTrace)

	verts := make([]int32, 256)
	simplified := make([]int32, 64)

	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			c := &chf.Cells[x+y*w]
			i := int32(c.Index)
			for ni := int32(c.Index) + int32(c.Count); i < ni; i++ {
				if flags[i] == 0 || flags[i] == 0xf {
					flags[i] = 0
					continue
				}
				reg := chf.Spans[i].Reg
				if (reg == 0) || ((reg & borderReg) != 0) {
					continue
				}
				area := chf.Areas[i]

				verts = make([]int32, 0)
				simplified = make([]int32, 0)

				ctx.StartTimer(TimerBuildContoursTrace)
				walkContour2(x, y, i, chf, flags, &verts)
				ctx.StopTimer(TimerBuildContoursTrace)

				ctx.StartTimer(TimerBuildContoursSimplify)
				simplifyContour(&verts, &simplified, maxError, maxEdgeLen, buildFlags)
				removeDegenerateSegments(&simplified)
				ctx.StopTimer(TimerBuildContoursSimplify)

				// Store region.contour remap info.
				// Create contour.
				if len(simplified)/4 >= 3 {
					if cset.NConts >= maxContours {
						panic("ENTERING")
						// Allocate more contours.
						// This happens when a region has holes.
						oldMax := maxContours
						maxContours *= 2
						newConts := make([]Contour, maxContours)
						for j := int32(0); j < cset.NConts; j++ {
							newConts[j] = cset.Conts[j]
							// Reset source pointers to prevent data deletion.
							cset.Conts[j].Verts = make([]int32, 0)
							cset.Conts[j].RVerts = make([]int32, 0)
						}
						cset.Conts = newConts

						ctx.Warningf("BuildContours: Expanding max contours from %d to %d.", oldMax, maxContours)
					}

					cont := &cset.Conts[cset.NConts]
					cset.NConts++
					cont.NVerts = int32(len(simplified) / 4)
					cont.Verts = make([]int32, cont.NVerts*4)
					copy(cont.Verts, simplified[:cont.NVerts*4])
					if borderSize > 0 {
						// If the heightfield was build with bordersize, remove the offset.
						for j := int32(0); j < cont.NVerts; j++ {
							v := cont.Verts[j*4:]
							v[0] -= borderSize
							v[2] -= borderSize
						}
					}

					cont.NRVerts = int32(len(verts) / 4)
					cont.RVerts = make([]int32, cont.NRVerts*4)
					copy(cont.RVerts, verts[:cont.NRVerts*4])
					if borderSize > 0 {
						// If the heightfield was build with bordersize, remove the offset.
						for j := int32(0); j < cont.NRVerts; j++ {
							v := cont.RVerts[j*4:]
							v[0] -= borderSize
							v[2] -= borderSize
						}
					}

					cont.Reg = reg
					cont.Area = area
				}
			}
		}
	}

	// Merge holes if needed.
	if cset.NConts > 0 {
		// Calculate winding of all polygons.
		winding := make([]uint8, cset.NConts)
		var nholes int32
		for i := int32(0); i < cset.NConts; i++ {
			cont := &cset.Conts[i]
			// If the contour is wound backwards, it is a hole.
			if calcAreaOfPolygon2D(cont.Verts, cont.NVerts) < 0 {
				winding[i] = 0xff
			} else {
				winding[i] = 1
			}
			if winding[i] < 0 {
				nholes++
			}
		}

		if nholes > 0 {
			panic("UNTESTED")
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			nregions := chf.MaxRegions + 1
			regions := make([]contourRegion, nregions)
			holes := make([]contourHole, cset.NConts)

			for i := int32(0); i < cset.NConts; i++ {
				cont := &cset.Conts[i]
				// Positively would contours are outlines, negative holes.
				if winding[i] > 0 {
					if regions[cont.Reg].outline != nil {
						ctx.Errorf("BuildContours: Multiple outlines for region %d.", cont.Reg)
					}
					regions[cont.Reg].outline = cont
				} else {
					regions[cont.Reg].nholes++
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
			for i := int32(0); i < cset.NConts; i++ {
				cont := &cset.Conts[i]
				reg := &regions[cont.Reg]
				if winding[i] < 0 {
					reg.holes[reg.nholes].contour = cont
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

	area := chf.Areas[i]

	iter := int32(0)
	for iter+1 < 40000 {
		iter++
		if (flags[i] & (1 << uint(dir))) != 0 {
			// Choose the edge corner
			isBorderVertex := false
			isAreaBorder := false
			px := x
			py, isBorderVertex := cornerHeight(x, y, i, int32(dir), chf)
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
			s := &chf.Spans[i]
			if GetCon(s, int32(dir)) != notConnected {
				ax := x + GetDirOffsetX(int32(dir))
				ay := y + GetDirOffsetY(int32(dir))
				ai := int32(chf.Cells[ax+ay*chf.Width].Index) + GetCon(s, int32(dir))
				r = int32(chf.Spans[ai].Reg)
				if area != chf.Areas[ai] {
					isAreaBorder = true
				}
			}
			if isBorderVertex {
				r |= borderVertex
			}
			if isAreaBorder {
				r |= areaBorder
			}
			*points = append(*points, px, py, pz, r)

			flags[i] &= ^(1 << dir) // Remove visited edges
			dir = (dir + 1) & 0x3   // Rotate CW
		} else {
			ni := int32(-1)
			nx := x + GetDirOffsetX(int32(dir))
			ny := y + GetDirOffsetY(int32(dir))
			s := &chf.Spans[i]
			if GetCon(s, int32(dir)) != notConnected {
				ni = int32(chf.Cells[nx+ny*chf.Width].Index) + GetCon(s, int32(dir))
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
		if ((*points)[i+3] & contourRegMask) != 0 {
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
			differentRegs := ((*points)[i*4+3] & contourRegMask) != ((*points)[ii*4+3] & contourRegMask)
			areaBorders := ((*points)[i*4+3] & areaBorder) != ((*points)[ii*4+3] & areaBorder)
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
			ax, bx = bx, ax
			az, bz = bz, az
		}

		// Tessellate only outer edges or edges between areas.
		if ((*points)[ci*4+3]&contourRegMask) == 0 ||
			(((*points)[ci*4+3] & areaBorder) != 0) {
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
	if maxEdgeLen > 0 && (buildFlags&(ContourTessWallEdges|ContourTessAreaEdges)) != 0 {
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
			if ((buildFlags & ContourTessWallEdges) != 0) && ((*points)[ci*4+3]&contourRegMask) == 0 {
				tess = true
			}
			// Edges between areas.
			if ((buildFlags & ContourTessAreaEdges) != 0) && (((*points)[ci*4+3] & areaBorder) != 0) {
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
		(*simplified)[i*4+3] = ((*points)[ai*4+3] & (contourRegMask | areaBorder)) | ((*points)[bi*4+3] & borderVertex)
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
	}
	return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]))
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
	minx = contour.Verts[0]
	minz = contour.Verts[2]
	leftmost = 0
	for i := int32(1); i < contour.NVerts; i++ {
		x := contour.Verts[i*4+0]
		z := contour.Verts[i*4+2]
		if x < minx || (x == minx && z < minz) {
			minx = x
			minz = z
			leftmost = i
		}
	}
	return
}

func mergeContours(ca, cb *Contour, ia, ib int32) bool {
	maxVerts := ca.NVerts + cb.NVerts + 2
	verts := make([]int32, maxVerts*4)

	var nv int32

	// Copy contour A.
	for i := int32(0); i <= ca.NVerts; i++ {
		dst := verts[nv*4:]
		src := ca.Verts[((ia+i)%ca.NVerts)*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		nv++
	}

	// Copy contour B
	for i := int32(0); i <= cb.NVerts; i++ {
		dst := verts[nv*4:]
		src := cb.Verts[((ib+i)%cb.NVerts)*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		nv++
	}

	ca.Verts = verts
	ca.NVerts = nv

	cb.Verts = make([]int32, 0)
	cb.NVerts = 0

	return true
}

type contourRegion struct {
	outline *Contour
	holes   []contourHole
	nholes  int32
}

type contourHole struct {
	contour              *Contour
	minx, minz, leftmost int32
}

type compareHoles []contourHole

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

type potentionalDiagonal struct {
	vert, dist int32
}

type compareDiagDist []potentionalDiagonal

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
