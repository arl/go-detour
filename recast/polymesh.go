package recast

import "github.com/aurelien-rainone/assertgo"

// PolyMesh represents a polygon mesh suitable for use in building a navigation
// mesh.
type PolyMesh struct {
	Verts        []uint16   // The mesh vertices. [Form: (x, y, z) * #nverts]
	Polys        []uint16   // Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
	Regs         []uint16   // The region id assigned to each polygon. [Length: #maxpolys]
	Flags        []uint16   // The user defined flags for each polygon. [Length: #maxpolys]
	Areas        []uint8    // The area id assigned to each polygon. [Length: #maxpolys]
	NVerts       int32      // The number of vertices.
	NPolys       int32      // The number of polygons.
	MaxPolys     int32      // The number of allocated polygons.
	Nvp          int32      // The maximum number of vertices per polygon.
	BMin         [3]float32 // The minimum bounds in world space. [(x, y, z)]
	BMax         [3]float32 // The maximum bounds in world space. [(x, y, z)]
	Cs           float32    // The size of each cell. (On the xz-plane.)
	Ch           float32    // The height of each cell. (The minimum increment along the y-axis.)
	BorderSize   int32      // The AABB border size used to generate the source data from which the mesh was derived.
	MaxEdgeError float32    // The max error of the polygon edges in the mesh.
}

func (pm *PolyMesh) Free() {
	if pm == nil {
		return
	}
	pm.Verts = make([]uint16, 0)
	pm.Polys = make([]uint16, 0)
	pm.Regs = make([]uint16, 0)
	pm.Flags = make([]uint16, 0)
	pm.Areas = make([]uint8, 0)
	pm = nil
}

// BuildPolyMesh builds a polygon mesh from the provided contours.
//
//  Arguments:
//  ctx     The build context to use during the operation.
//  cset    A fully built contour set.
//  nvp     The maximum number of vertices allowed for polygons generated during
//          the contour to polygon conversion process. [Limit: >= 3]
//  mesh    The resulting polygon mesh. (Must be re-allocated.)
//
// Returns True if the operation completed successfully.
//
// Note: If the mesh data is to be used to construct a Detour navigation mesh,
// then the upper
// limit must be retricted to <= VertsPerPolygon.
//
// see ContourSet, PolyMesh, Config
func BuildPolyMesh(ctx *BuildContext, cset *ContourSet, nvp int32) (*PolyMesh, bool) {
	assert.True(ctx != nil, "ctx should not be nil")

	ctx.StartTimer(TimerBuildPolymesh)
	defer ctx.StopTimer(TimerBuildPolymesh)

	var (
		maxVertices     int32
		maxTris         int32
		maxVertsPerCont int32
		mesh            *PolyMesh
	)

	for i := int32(0); i < cset.NConts; i++ {
		// Skip null contours.
		if cset.Conts[i].NVerts < 3 {
			continue
		}
		maxVertices += cset.Conts[i].NVerts
		maxTris += cset.Conts[i].NVerts - 2
		maxVertsPerCont = iMax(maxVertsPerCont, cset.Conts[i].NVerts)
	}

	if maxVertices >= 0xfffe {
		ctx.Errorf("BuildPolyMesh: Too many vertices %d.", maxVertices)
		return nil, false
	}

	mesh = &PolyMesh{
		Cs:           cset.Cs,
		Ch:           cset.Ch,
		BorderSize:   cset.BorderSize,
		MaxEdgeError: cset.MaxError,
		Verts:        make([]uint16, maxVertices*3),
		Polys:        make([]uint16, maxTris*nvp*2),
		Regs:         make([]uint16, maxTris),
		Areas:        make([]uint8, maxTris),
		NVerts:       0,
		NPolys:       0,
		Nvp:          nvp,
		MaxPolys:     maxTris,
	}

	copy(mesh.BMin[:], cset.BMin[:])
	copy(mesh.BMax[:], cset.BMax[:])

	vflags := make([]uint8, maxVertices)

	for i := range mesh.Polys {
		mesh.Polys[i] = 0xffff
	}

	nextVert := make([]int32, maxVertices)
	firstVert := make([]int32, VERTEX_BUCKET_COUNT)

	for i := range firstVert {
		firstVert[i] = -1
	}

	indices := make([]int64, maxVertsPerCont)
	tris := make([]int32, maxVertsPerCont*3)
	polys := make([]uint16, (maxVertsPerCont+1)*nvp)
	tmpPoly := polys[maxVertsPerCont*nvp:]

	for i := int32(0); i < cset.NConts; i++ {
		cont := cset.Conts[i]

		// Skip null contours.
		if cont.NVerts < 3 {
			continue
		}

		// Triangulate contour
		for j := int32(0); j < cont.NVerts; j++ {
			indices[j] = int64(j)
		}

		ntris := triangulate(cont.NVerts, cont.Verts, indices[:], tris[:])
		if ntris <= 0 {
			// Bad triangulation, should not happen.
			/*			printf("\tconst float bmin[3] = {%ff,%ff,%ff};\n", cset.bmin[0], cset.bmin[1], cset.bmin[2]);
						printf("\tconst float cs = %ff;\n", cset.cs);
						printf("\tconst float ch = %ff;\n", cset.ch);
						printf("\tconst int verts[] = {\n");
						for (int k = 0; k < cont.nverts; ++k)
						{
							const int* v = &cont.verts[k*4];
							printf("\t\t%d,%d,%d,%d,\n", v[0], v[1], v[2], v[3]);
						}
						printf("\t};\n\tconst int nverts = sizeof(verts)/(sizeof(int)*4);\n");*/
			ctx.Warningf("rcBuildPolyMesh: Bad triangulation Contour %d.", i)
			ntris = -ntris
		}

		// Add and merge vertices.
		for j := int32(0); j < cont.NVerts; j++ {
			v := cont.Verts[j*4:]
			indices[j] = int64(addVertex(uint16(v[0]), uint16(v[1]), uint16(v[2]),
				mesh.Verts, firstVert, nextVert, &mesh.NVerts))
			if (v[3] & borderVertex) != 0 {
				// This vertex should be removed.
				vflags[indices[j]] = 1
			}
		}

		// Build initial polygons.
		var npolys int32
		//memset(polys, 0xff, maxVertsPerCont*nvp*sizeof(unsigned short));
		for i := int32(0); i < maxVertsPerCont*nvp; i++ {
			polys[i] = 0xffff
		}

		for j := int32(0); j < ntris; j++ {
			t := tris[j*3:]
			if t[0] != t[1] && t[0] != t[2] && t[1] != t[2] {
				polys[npolys*nvp+0] = uint16(indices[t[0]])
				polys[npolys*nvp+1] = uint16(indices[t[1]])
				polys[npolys*nvp+2] = uint16(indices[t[2]])
				npolys++
			}
		}
		if npolys == 0 {
			continue
		}

		// Merge polygons.
		if nvp > 3 {
			for {
				// Find best polygons to merge.
				var (
					bestMergeVal                   int32
					bestPa, bestPb, bestEa, bestEb int32
				)

				for j := int32(0); j < npolys-1; j++ {
					pj := polys[j*nvp:]
					for k := j + 1; k < npolys; k++ {
						pk := polys[k*nvp:]
						var ea, eb int32
						v := getPolyMergeValue(pj, pk, mesh.Verts, &ea, &eb, nvp)
						if v > bestMergeVal {
							bestMergeVal = v
							bestPa = j
							bestPb = k
							bestEa = ea
							bestEb = eb
						}
					}
				}

				if bestMergeVal > 0 {
					// Found best, merge.
					pa := polys[bestPa*nvp:]
					pb := polys[bestPb*nvp:]
					mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp)
					lastPoly := polys[(npolys-1)*nvp:]

					if !compareSlicesUInt16(pb, lastPoly) {
						copy(pb, lastPoly[:nvp])
					}
					npolys--
				} else {
					// Could not merge any polygons, stop.
					break
				}
			}
		}

		// Store polygons.
		for j := int32(0); j < npolys; j++ {
			p := mesh.Polys[mesh.NPolys*nvp*2:]
			q := polys[j*nvp:]
			for k := int32(0); k < nvp; k++ {
				p[k] = q[k]
			}
			mesh.Regs[mesh.NPolys] = cont.Reg
			mesh.Areas[mesh.NPolys] = cont.Area
			mesh.NPolys++
			if mesh.NPolys > maxTris {
				ctx.Errorf("rcBuildPolyMesh: Too many polygons %d (max:%d).", mesh.NPolys, maxTris)
				return nil, false
			}
		}
	}

	// Remove edge vertices.
	for i := int32(0); i < mesh.NVerts; i++ {
		if vflags[i] != 0 {
			if !canRemoveVertex(ctx, mesh, uint16(i)) {
				continue
			}
			if !removeVertex(ctx, mesh, uint16(i), maxTris) {
				// Failed to remove vertex
				ctx.Errorf("rcBuildPolyMesh: Failed to remove edge vertex %d.", i)
				return nil, false
			}
			// Remove vertex
			// Note: mesh.nverts is already decremented inside removeVertex()!
			// Fixup vertex flags
			for j := i; j < mesh.NVerts; j++ {
				vflags[j] = vflags[j+1]
			}
			i--
		}
	}

	// Calculate adjacency.
	if !buildMeshAdjacency(mesh.Polys, mesh.NPolys, mesh.NVerts, nvp) {
		ctx.Errorf("rcBuildPolyMesh: Adjacency failed.")
		return nil, false
	}

	// Find portal edges
	if mesh.BorderSize > 0 {
		w := cset.Width
		h := cset.Height
		for i := int32(0); i < mesh.NPolys; i++ {
			p := mesh.Polys[i*2*nvp:]
			for j := int32(0); j < nvp; j++ {
				if p[j] == meshNullIdx {
					break
				}
				// Skip connected edges.
				if p[nvp+j] != meshNullIdx {
					continue
				}
				nj := j + 1
				if nj >= nvp || p[nj] == meshNullIdx {
					nj = 0
				}
				va := mesh.Verts[p[j]*3:]
				vb := mesh.Verts[p[nj]*3:]

				if int32(va[0]) == 0 && int32(vb[0]) == 0 {
					p[nvp+j] = 0x8000 | 0
				} else if int32(va[2]) == h && int32(vb[2]) == h {
					p[nvp+j] = 0x8000 | 1
				} else if int32(va[0]) == w && int32(vb[0]) == w {
					p[nvp+j] = 0x8000 | 2
				} else if int32(va[2]) == 0 && int32(vb[2]) == 0 {
					p[nvp+j] = 0x8000 | 3
				}
			}
		}
	}

	// Just allocate the mesh flags array. The user is resposible to fill it.
	mesh.Flags = make([]uint16, mesh.NPolys)
	if mesh.NVerts > 0xffff {
		ctx.Errorf("rcBuildPolyMesh: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.", mesh.NVerts, 0xffff)
	}
	if mesh.NPolys > 0xffff {
		ctx.Errorf("rcBuildPolyMesh: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.", mesh.NPolys, 0xffff)
	}

	return mesh, true
}
