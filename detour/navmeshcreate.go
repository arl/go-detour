package detour

import (
	"fmt"
	"sort"
	"unsafe"

	"github.com/aurelien-rainone/gogeo/f32"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

type bvItem struct {
	BMin, BMax [3]uint16
	i          int32
}

type compareItemX []bvItem

// Len is the number of elements in the collection.
func (s compareItemX) Len() int {
	return len(s)
}

// Less reports whether the element with
// index i should sort before the element with index j.
func (s compareItemX) Less(i, j int) bool {
	a := s[i]
	b := s[j]

	if a.BMin[0] < b.BMin[0] {
		return true
	}
	if a.BMin[0] > b.BMin[0] {
		return false
	}

	// if x are equal, compare y
	if a.BMin[1] < b.BMin[1] {
		return true
	}
	if a.BMin[1] > b.BMin[1] {
		return false
	}

	// if y are equal, compare z
	if a.BMin[2] < b.BMin[2] {
		return true
	}
	if a.BMin[2] > b.BMin[2] {
		return false
	}

	// compare bmax
	if a.BMax[0] < b.BMax[0] {
		return true
	}
	if a.BMax[0] > b.BMax[0] {
		return false
	}
	if a.BMax[1] < b.BMax[1] {
		return true
	}
	if a.BMax[1] > b.BMax[1] {
		return false
	}
	if a.BMax[2] < b.BMax[2] {
		return true
	}
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareItemX) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

type compareItemY []bvItem

// Len is the number of elements in the collection.
func (s compareItemY) Len() int {
	return len(s)
}

// Less reports whether the element with
// index i should sort before the element with index j.
func (s compareItemY) Less(i, j int) bool {
	a := s[i]
	b := s[j]

	if a.BMin[1] < b.BMin[1] {
		return true
	}
	if a.BMin[1] > b.BMin[1] {
		return false
	}

	// if y are equal, compare z
	if a.BMin[2] < b.BMin[2] {
		return true
	}
	if a.BMin[2] > b.BMin[2] {
		return false
	}

	// if z are equal, compare x
	if a.BMin[0] < b.BMin[0] {
		return true
	}
	if a.BMin[0] > b.BMin[0] {
		return false
	}

	// compare bmax
	if a.BMax[0] < b.BMax[0] {
		return true
	}
	if a.BMax[0] > b.BMax[0] {
		return false
	}
	if a.BMax[1] < b.BMax[1] {
		return true
	}
	if a.BMax[1] > b.BMax[1] {
		return false
	}
	if a.BMax[2] < b.BMax[2] {
		return true
	}
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareItemY) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

type compareItemZ []bvItem

// Len is the number of elements in the collection.
func (s compareItemZ) Len() int {
	return len(s)
}

// Less reports whether the element with
// index i should sort before the element with index j.
func (s compareItemZ) Less(i, j int) bool {
	a := s[i]
	b := s[j]

	if a.BMin[2] < b.BMin[2] {
		return true
	}
	if a.BMin[2] > b.BMin[2] {
		return false
	}
	// if z are equal, compare x
	if a.BMin[0] < b.BMin[0] {
		return true
	}
	if a.BMin[0] > b.BMin[0] {
		return false
	}
	// if x are equal, compare y
	if a.BMin[1] < b.BMin[1] {
		return true
	}
	if a.BMin[1] > b.BMin[1] {
		return false
	}

	// compare bmax
	if a.BMax[0] < b.BMax[0] {
		return true
	}
	if a.BMax[0] > b.BMax[0] {
		return false
	}
	if a.BMax[1] < b.BMax[1] {
		return true
	}
	if a.BMax[1] > b.BMax[1] {
		return false
	}
	if a.BMax[2] < b.BMax[2] {
		return true
	}
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareItemZ) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

func calcExtends(items []bvItem, imin, imax int32, bmin, bmax []uint16) {
	bmin[0] = items[imin].BMin[0]
	bmin[1] = items[imin].BMin[1]
	bmin[2] = items[imin].BMin[2]

	bmax[0] = items[imin].BMax[0]
	bmax[1] = items[imin].BMax[1]
	bmax[2] = items[imin].BMax[2]

	for i := imin + 1; i < imax; i++ {
		it := &items[i]
		if it.BMin[0] < bmin[0] {
			bmin[0] = it.BMin[0]
		}
		if it.BMin[1] < bmin[1] {
			bmin[1] = it.BMin[1]
		}
		if it.BMin[2] < bmin[2] {
			bmin[2] = it.BMin[2]
		}

		if it.BMax[0] > bmax[0] {
			bmax[0] = it.BMax[0]
		}
		if it.BMax[1] > bmax[1] {
			bmax[1] = it.BMax[1]
		}
		if it.BMax[2] > bmax[2] {
			bmax[2] = it.BMax[2]
		}
	}
}

func longestAxis(x, y, z uint16) int {
	var axis int
	maxVal := x
	if y > maxVal {
		axis = 1
		maxVal = y
	}
	if z > maxVal {
		axis = 2
	}
	return axis
}

func subdivide(items []bvItem, nitems, imin, imax int32, curNode *int32, nodes []BvNode) {
	inum := imax - imin
	icur := *curNode

	node := &nodes[*curNode]
	*curNode++

	if inum == 1 {
		// Leaf
		node.Bmin[0] = items[imin].BMin[0]
		node.Bmin[1] = items[imin].BMin[1]
		node.Bmin[2] = items[imin].BMin[2]

		node.Bmax[0] = items[imin].BMax[0]
		node.Bmax[1] = items[imin].BMax[1]
		node.Bmax[2] = items[imin].BMax[2]

		node.I = items[imin].i
	} else {
		// Split
		calcExtends(items, imin, imax, node.Bmin[:], node.Bmax[:])

		axis := longestAxis(
			node.Bmax[0]-node.Bmin[0],
			node.Bmax[1]-node.Bmin[1],
			node.Bmax[2]-node.Bmin[2])

		if axis == 0 {
			// Sort along x-axis
			sort.Sort(compareItemX(items[imin : imin+inum]))
		} else if axis == 1 {
			// Sort along y-axis
			sort.Sort(compareItemY(items[imin : imin+inum]))
		} else {
			// Sort along z-axis
			sort.Sort(compareItemZ(items[imin : imin+inum]))
		}

		isplit := imin + inum/2

		// Left
		subdivide(items, nitems, imin, isplit, curNode, nodes)
		// Right
		subdivide(items, nitems, isplit, imax, curNode, nodes)

		iescape := *curNode - icur

		// Negative index means escape.
		node.I = -iescape
	}
}

func int32Clamp(a, low, high int32) int32 {
	if a < low {
		return low
	} else if a > high {
		return high
	}

	return a
}

func createBVTree(params *NavMeshCreateParams, nodes []BvNode) int32 {
	// Build tree
	quantFactor := float32(1.0) / params.Cs
	items := make([]bvItem, params.PolyCount)
	for i := int32(0); i < params.PolyCount; i++ {
		it := &items[i]
		it.i = i
		// Calc polygon bounds. Use detail meshes if available.
		if len(params.DetailMeshes) > 0 {

			vb := int32(params.DetailMeshes[i*4+0])
			ndv := int32(params.DetailMeshes[i*4+1])
			var bmin, bmax [3]float32

			dv := params.DetailVerts[vb*3:]
			copy(bmin[:], dv[:3])
			copy(bmax[:], dv[:3])

			for j := int32(1); j < ndv; j++ {
				d3.Vec3Min(bmin[:], dv[j*3:])
				d3.Vec3Max(bmax[:], dv[j*3:])
			}

			// BV-tree uses cs for all dimensions
			it.BMin[0] = uint16(int32Clamp(int32((bmin[0]-params.BMin[0])*quantFactor), 0, 0xffff))
			it.BMin[1] = uint16(int32Clamp(int32((bmin[1]-params.BMin[1])*quantFactor), 0, 0xffff))
			it.BMin[2] = uint16(int32Clamp(int32((bmin[2]-params.BMin[2])*quantFactor), 0, 0xffff))

			it.BMax[0] = uint16(int32Clamp(int32((bmax[0]-params.BMin[0])*quantFactor), 0, 0xffff))
			it.BMax[1] = uint16(int32Clamp(int32((bmax[1]-params.BMin[1])*quantFactor), 0, 0xffff))
			it.BMax[2] = uint16(int32Clamp(int32((bmax[2]-params.BMin[2])*quantFactor), 0, 0xffff))
		} else {
			panic("UNTESTED")
			p := params.Polys[i*params.Nvp*2:]
			it.BMin[0] = params.Verts[p[0]*3+0]
			it.BMin[1] = params.Verts[p[0]*3+1]
			it.BMin[2] = params.Verts[p[0]*3+2]

			it.BMax[0] = it.BMin[0]
			it.BMax[1] = it.BMin[1]
			it.BMax[2] = it.BMin[2]

			for j := int32(1); j < params.Nvp; j++ {
				if p[j] == meshNullIdx {
					break
				}
				x := params.Verts[p[j]*3+0]
				y := params.Verts[p[j]*3+1]
				z := params.Verts[p[j]*3+2]

				if x < it.BMin[0] {
					it.BMin[0] = x
				}
				if y < it.BMin[1] {
					it.BMin[1] = y
				}
				if z < it.BMin[2] {
					it.BMin[2] = z
				}

				if x > it.BMax[0] {
					it.BMax[0] = x
				}
				if y > it.BMax[1] {
					it.BMax[1] = y
				}
				if z > it.BMax[2] {
					it.BMax[2] = z
				}
			}
			// Remap y
			it.BMin[1] = uint16(math32.Floor(float32(it.BMin[1]) * params.Ch / params.Cs))
			it.BMax[1] = uint16(math32.Ceil(float32(it.BMax[1]) * params.Ch / params.Cs))
		}
	}

	var curNode int32
	subdivide(items, params.PolyCount, 0, params.PolyCount, &curNode, nodes)
	return curNode
}

func classifyOffMeshPoint(pt, bmin, bmax d3.Vec3) uint8 {
	const (
		XP uint8 = 1 << 0
		ZP       = 1 << 1
		XM       = 1 << 2
		ZM       = 1 << 3
	)

	var outcode uint8

	if pt[0] >= bmax[0] {
		outcode |= XP
	}

	if pt[2] >= bmax[2] {
		outcode |= ZP
	}
	if pt[0] < bmin[0] {
		outcode |= XM
	}
	if pt[2] < bmin[2] {
		outcode |= ZM
	}

	switch outcode {
	case XP:
		return 0
	case XP | ZP:
		return 1
	case ZP:
		return 2
	case XM | ZP:
		return 3
	case XM:
		return 4
	case XM | ZM:
		return 5
	case ZM:
		return 6
	case XP | ZM:
		return 7
	}

	return 0xff
}

const meshNullIdx uint16 = 0xffff

// NavMeshCreateParams represents the source data used to build an navigation
// mesh tile.
type NavMeshCreateParams struct {

	//
	// Polygon Mesh Attributes
	// Used to create the base navigation graph.
	// See recast.PolyMesh for details related to these attributes.
	//

	// The polygon mesh vertices.
	// [(x, y, z) * vertCount] [Unit: vx]
	Verts []uint16

	// The number vertices in the polygon mesh.
	// [Limit: >= 3]
	VertCount int32

	// The polygon data.
	// [Size: polyCount * 2 * nvp]
	Polys []uint16

	// The user defined flags assigned to each polygon.
	// [Size: polyCount]
	PolyFlags []uint16

	// The user defined area ids assigned to each polygon.
	// [Size: polyCount]
	PolyAreas []uint8

	// Number of polygons in the mesh.
	// [Limit: >= 1]
	PolyCount int32

	// Number maximum number of vertices per polygon.
	// [Limit: >= 3]
	Nvp int32

	//
	// Height Detail Attributes (Optional)
	// See recast.PolyMeshDetail for details related to these attributes.
	//

	// The height detail sub-mesh data.
	// [Size: 4 * polyCount]
	DetailMeshes []int32

	// The detail mesh vertices.
	// [Size: 3 * detailVertsCount] [Unit: wu]
	DetailVerts []float32

	// The number of vertices in the detail mesh.
	DetailVertsCount int32

	// The detail mesh triangles.
	// [Size: 4 * detailTriCount]
	DetailTris []uint8

	// The number of triangles in the detail mesh.
	DetailTriCount int32

	//
	// Off-Mesh Connections Attributes (Optional)
	// Used to define a custom point-to-point edge within the navigation graph,
	// an off-mesh connection is a user defined traversable connection made up
	// to two vertices, at least one of which resides within a navigation mesh
	// polygon.
	//

	// Off-mesh connection vertices.
	// [(ax, ay, az, bx, by, bz) * offMeshConCount] [Unit: wu]
	OffMeshConVerts []float32

	// Off-mesh connection radii.
	// [Size: offMeshConCount] [Unit: wu]
	OffMeshConRad []float32

	// User defined flags assigned to the off-mesh connections.
	// [Size: offMeshConCount]
	OffMeshConFlags []uint16

	// User defined area ids assigned to the off-mesh connections.
	// [Size: offMeshConCount]
	OffMeshConAreas []uint8

	// The permitted travel direction of the off-mesh connections.
	// [Size: offMeshConCount]
	//
	// 0 = Travel only from endpoint A to endpoint B.<br/>
	// OffMeshConBiDir = Bidirectional travel.
	OffMeshConDir []uint8

	// The user defined ids of the off-mesh connection.
	// [Size: offMeshConCount]
	OffMeshConUserID []uint32

	// The number of off-mesh connections.
	// [Limit: >= 0]
	OffMeshConCount int32

	//
	// Tile Attributes
	// note The tile grid/layer data can be left at zero
	// if the destination is a single tile mesh.
	//

	// The user defined id of the tile.
	UserID uint32

	// The tile's x-grid location within the multi-tile destination mesh.
	// (Along the x-axis.)
	TileX int32

	// The tile's y-grid location within the multi-tile desitation mesh.
	// (Along the z-axis.)
	TileY int32

	// The tile's layer within the layered destination mesh. [Limit: >= 0]
	// (Along the y-axis.)
	TileLayer int32

	// The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
	BMin [3]float32

	// The maximum bounds of the tile. [(x, y, z)] [Unit: wu]
	BMax [3]float32

	//
	// General Configuration Attributes
	//

	// The agent height. [Unit: wu]
	WalkableHeight float32
	// The agent radius. [Unit: wu]
	WalkableRadius float32
	// The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	WalkableClimb float32
	// The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	Cs float32
	// The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]
	Ch float32

	// True if a bounding volume tree should be built for the tile.
	// Note The BVTree is not normally needed for layered navigation meshes.
	BuildBvTree bool
}

// CreateNavMeshData builds navigation mesh tile data from the provided tile
// creation data.
//
//  Arguments:
//  paramsi     Tile creation data.
//  outData     The resulting tile data.
//  outDataSize The size of the tile data array.
//
// Return true if the tile data was successfully created.
//
// see NavMesh, NavMesh.addTile()
func CreateNavMeshData(params *NavMeshCreateParams) ([]uint8, error) {
	if params.Nvp > int32(VertsPerPolygon) {
		return nil, fmt.Errorf("wrong value for params.Nvp")
	}
	if params.VertCount >= 0xffff {
		return nil, fmt.Errorf("wrong value for params.VertCount")
	}
	if params.VertCount == 0 || params.Verts == nil {
		return nil, fmt.Errorf("wrong value for params.VertCount or params.Verts")
	}
	if params.PolyCount == 0 || params.Polys == nil {
		return nil, fmt.Errorf("wrong value for params.PolyCount or params.Polys")
	}

	nvp := params.Nvp

	// Classify off-mesh connection points. We store only the connections
	// whose start point is inside the tile.
	var (
		offMeshConClass       []uint8
		storedOffMeshConCount int32
		offMeshConLinkCount   int32
	)

	if params.OffMeshConCount > 0 {
		offMeshConClass = make([]uint8, params.OffMeshConCount*2)

		// Find tight heigh bounds, used for culling out off-mesh start locations.
		hmin := math32.MaxFloat32
		hmax := -math32.MaxFloat32

		if params.DetailVerts != nil && params.DetailVertsCount != 0 {
			for i := int32(0); i < params.DetailVertsCount; i++ {
				h := params.DetailVerts[i*3+1]
				f32.SetMin(&hmin, h)
				f32.SetMax(&hmax, h)
			}
		} else {
			for i := int32(0); i < params.VertCount; i++ {
				iv := params.Verts[i*3:]
				h := params.BMin[1] + float32(iv[1])*params.Ch
				f32.SetMin(&hmin, h)
				f32.SetMax(&hmax, h)
			}
		}
		hmin -= params.WalkableClimb
		hmax += params.WalkableClimb
		var bmin, bmax [3]float32
		copy(bmin[:], params.BMin[:])
		copy(bmax[:], params.BMax[:])
		bmin[1] = hmin
		bmax[1] = hmax

		for i := int32(0); i < params.OffMeshConCount; i++ {
			p0 := params.OffMeshConVerts[(i*2+0)*3:]
			p1 := params.OffMeshConVerts[(i*2+1)*3:]
			offMeshConClass[i*2+0] = classifyOffMeshPoint(p0, bmin[:], bmax[:])
			offMeshConClass[i*2+1] = classifyOffMeshPoint(p1, bmin[:], bmax[:])

			// Zero out off-mesh start positions which are not even potentially touching the mesh.
			if offMeshConClass[i*2+0] == 0xff {
				if p0[1] < bmin[1] || p0[1] > bmax[1] {
					offMeshConClass[i*2+0] = 0
				}
			}

			// Cound how many links should be allocated for off-mesh connections.
			if offMeshConClass[i*2+0] == 0xff {
				offMeshConLinkCount++
			}
			if offMeshConClass[i*2+1] == 0xff {
				offMeshConLinkCount++
			}

			if offMeshConClass[i*2+0] == 0xff {
				storedOffMeshConCount++
			}
		}
	}

	// Off-mesh connectionss are stored as polygons, adjust values.
	totPolyCount := int(params.PolyCount + storedOffMeshConCount)
	totVertCount := int(params.VertCount + storedOffMeshConCount*2)

	// Find portal edges which are at tile borders.
	var (
		edgeCount   int32
		portalCount int32
	)
	for i := int32(0); i < params.PolyCount; i++ {
		p := params.Polys[i*2*nvp:]
		for j := int32(0); j < nvp; j++ {
			if p[j] == meshNullIdx {
				break
			}
			edgeCount++

			if (p[nvp+j] & 0x8000) != 0 {
				dir := p[nvp+j] & 0xf
				if dir != 0xf {
					portalCount++
				}
			}
		}
	}

	maxLinkCount := edgeCount + portalCount*2 + offMeshConLinkCount*2

	// Find unique detail vertices.
	var (
		uniqueDetailVertCount int32
		detailTriCount        int32
	)
	if params.DetailMeshes != nil {
		// Has detail mesh, count unique detail vertex count and use input detail tri count.
		detailTriCount = params.DetailTriCount
		for i := int32(0); i < params.PolyCount; i++ {
			p := params.Polys[i*nvp*2:]
			ndv := params.DetailMeshes[i*4+1]
			var nv int32
			for j := int32(0); j < nvp; j++ {
				if p[j] == meshNullIdx {
					break
				}
				nv++
			}
			ndv -= nv
			uniqueDetailVertCount += ndv
		}
	} else {
		// No input detail mesh, build detail mesh from nav polys.
		uniqueDetailVertCount = 0 // No extra detail verts.
		detailTriCount = 0
		for i := int32(0); i < params.PolyCount; i++ {
			p := params.Polys[i*nvp*2:]
			var nv int32
			for j := int32(0); j < nvp; j++ {
				if p[j] == meshNullIdx {
					break
				}
				nv++
			}
			detailTriCount += nv - 2
		}
	}

	var (
		hdr        MeshHeader
		bvTreeSize int
	)

	// Calculate data size in order to allocate buffer
	headerSize := hdr.size()
	vertsSize := 4 * 3 * totVertCount
	polysSize := int(unsafe.Sizeof(Poly{})) * totPolyCount
	linksSize := int(unsafe.Sizeof(Link{})) * int(maxLinkCount)
	detailMeshesSize := int(unsafe.Sizeof(PolyDetail{})) * int(params.PolyCount)
	detailVertsSize := 4 * 3 * int(uniqueDetailVertCount)
	detailTrisSize := 4 * int(detailTriCount)
	if params.BuildBvTree {
		bvTreeSize = int(unsafe.Sizeof(BvNode{})) * int(params.PolyCount*2)
	}
	offMeshConsSize := int(unsafe.Sizeof(OffMeshConnection{})) * int(storedOffMeshConCount)

	dataSize := headerSize + vertsSize + polysSize + linksSize +
		detailMeshesSize + detailVertsSize + detailTrisSize +
		bvTreeSize + offMeshConsSize

	// create the variables that will hold the values to serialize
	navVerts := make([]float32, 3*totVertCount)
	navPolys := make([]Poly, totPolyCount)

	navDMeshes := make([]PolyDetail, params.PolyCount)
	navDVerts := make([]float32, 3*uniqueDetailVertCount)
	navDTris := make([]uint8, 4*detailTriCount)
	navBvtree := make([]BvNode, params.PolyCount*2)
	offMeshCons := make([]OffMeshConnection, storedOffMeshConCount)

	// Fill header
	hdr.Magic = navMeshMagic
	hdr.Version = navMeshVersion
	hdr.X = params.TileX
	hdr.Y = params.TileY
	hdr.Layer = params.TileLayer
	hdr.UserID = params.UserID
	hdr.PolyCount = int32(totPolyCount)
	hdr.VertCount = int32(totVertCount)
	hdr.MaxLinkCount = maxLinkCount
	copy(hdr.Bmin[:], params.BMin[:])
	copy(hdr.Bmax[:], params.BMax[:])
	hdr.DetailMeshCount = params.PolyCount
	hdr.DetailVertCount = uniqueDetailVertCount
	hdr.DetailTriCount = detailTriCount
	hdr.BvQuantFactor = 1.0 / params.Cs
	hdr.OffMeshBase = params.PolyCount
	hdr.WalkableHeight = params.WalkableHeight
	hdr.WalkableRadius = params.WalkableRadius
	hdr.WalkableClimb = params.WalkableClimb
	hdr.OffMeshConCount = storedOffMeshConCount
	hdr.BvNodeCount = 0
	if params.BuildBvTree {
		hdr.BvNodeCount = params.PolyCount * 2
	}

	offMeshVertsBase := params.VertCount
	offMeshPolyBase := params.PolyCount

	// Fill vertices
	// Mesh vertices
	for i := int32(0); i < params.VertCount; i++ {
		iv := params.Verts[i*3 : i*3+3]
		v := navVerts[i*3 : i*3+3]
		v[0] = params.BMin[0] + float32(iv[0])*params.Cs
		v[1] = params.BMin[1] + float32(iv[1])*params.Ch
		v[2] = params.BMin[2] + float32(iv[2])*params.Cs
	}
	// Off-mesh link vertices.
	var n int32
	for i := int32(0); i < params.OffMeshConCount; i++ {
		// Only store connections which start from this tile.
		if offMeshConClass[i*2+0] == 0xff {
			linkv := params.OffMeshConVerts[i*2*3:]
			v := navVerts[(offMeshVertsBase+n*2)*3:]
			copy(v[0:3], linkv[0:3])
			copy(v[3:6], linkv[3:6])
			n++
		}
	}

	// Fill polygons
	// Mesh polys
	src := params.Polys[:]
	for i := int32(0); i < params.PolyCount; i++ {
		p := &navPolys[i]
		p.VertCount = 0
		p.Flags = params.PolyFlags[i]
		p.SetArea(params.PolyAreas[i])
		p.SetType(uint8(polyTypeGround))
		for j := int32(0); j < nvp; j++ {
			if src[j] == meshNullIdx {
				break
			}
			p.Verts[j] = src[j]
			if (src[nvp+j] & 0x8000) != 0 {
				// Border or portal edge.
				dir := src[nvp+j] & 0xf
				if dir == 0xf {
					// Border
					p.Neis[j] = 0

				} else if dir == 0 {
					// Portal x-
					p.Neis[j] = extLink | 4

				} else if dir == 1 {
					// Portal z+
					p.Neis[j] = extLink | 2

				} else if dir == 2 {
					// Portal x+
					p.Neis[j] = extLink | 0

				} else if dir == 3 {
					// Portal z-
					p.Neis[j] = extLink | 6

				}
			} else {
				// Normal connection
				p.Neis[j] = src[nvp+j] + 1
			}

			p.VertCount++
		}
		src = src[nvp*2:]
	}

	// Off-mesh connection vertices.
	n = 0
	for i := int32(0); i < params.OffMeshConCount; i++ {
		// Only store connections which start from this tile.
		if offMeshConClass[i*2+0] == 0xff {
			p := &navPolys[offMeshPolyBase+n]
			p.VertCount = 2
			p.Verts[0] = uint16(offMeshVertsBase + n*2 + 0)
			p.Verts[1] = uint16(offMeshVertsBase + n*2 + 1)
			p.Flags = params.OffMeshConFlags[i]
			p.SetArea(params.OffMeshConAreas[i])
			p.SetType(polyTypeOffMeshConnection)
			n++
		}
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	if params.DetailMeshes != nil && len(params.DetailMeshes) > 0 {
		var vbase uint16
		for i2 := int32(0); i2 < params.PolyCount; i2++ {
			dtl := &navDMeshes[i2]
			vb := params.DetailMeshes[i2*4+0]
			ndv := params.DetailMeshes[i2*4+1]
			nv := int32(navPolys[i2].VertCount)
			dtl.VertBase = uint32(vbase)
			dtl.VertCount = uint8(ndv - nv)
			dtl.TriBase = uint32(params.DetailMeshes[i2*4+2])
			dtl.TriCount = uint8(params.DetailMeshes[i2*4+3])
			// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
			if ndv-nv != 0 {
				start, length := int32(vb+nv)*3, 3*int32(ndv-nv)
				copy(navDVerts[vbase*3:], params.DetailVerts[start:start+length])
				vbase += uint16(ndv - nv)
			}
		}
		// Store triangles.
		copy(navDTris, params.DetailTris[:4*params.DetailTriCount])
	} else {
		// Create dummy detail mesh by triangulating polys.
		var tbase int32
		for i := int32(0); i < params.PolyCount; i++ {
			dtl := &navDMeshes[i]
			nv := navPolys[i].VertCount
			dtl.VertBase = 0
			dtl.VertCount = 0
			dtl.TriBase = uint32(tbase)
			dtl.TriCount = uint8(nv - 2)
			// Triangulate polygon (local indices).
			for j := uint8(2); j < nv; j++ {
				t := navDTris[tbase*4:]
				t[0] = 0
				t[1] = uint8(j - 1)
				t[2] = uint8(j)
				// Bit for each edge that belongs to poly boundary.
				t[3] = (1 << 2)
				if j == 2 {
					t[3] |= (1 << 0)
				}
				if j == nv-1 {
					t[3] |= (1 << 4)
				}
				tbase++
			}
		}
	}

	// Store and create BVtree.
	if params.BuildBvTree {
		createBVTree(params, navBvtree) //, 2*params.PolyCount)
	}

	// Store Off-Mesh connections.
	n = 0
	for i := int32(0); i < params.OffMeshConCount; i++ {
		// Only store connections which start from this tile.
		if offMeshConClass[i*2+0] == 0xff {
			con := &offMeshCons[n]
			con.Poly = uint16(offMeshPolyBase + n)
			// Copy connection end-points.
			endPts := params.OffMeshConVerts[i*2*3:]
			copy(con.Pos[0:], endPts[:3])
			copy(con.Pos[3:], endPts[3:])
			con.Rad = params.OffMeshConRad[i]
			if params.OffMeshConDir[i] != 0 {
				con.Flags = uint8(offMeshConBidir)
			} else {
				con.Flags = 0
			}
			con.Side = offMeshConClass[i*2+1]
			if len(params.OffMeshConUserID) != 0 {
				con.UserID = params.OffMeshConUserID[i]
			}
			n++
		}
	}

	buf := make([]byte, dataSize)
	hdr.serialize(buf)
	err := serializeTileData(buf[hdr.size():],
		navVerts,
		navPolys,
		make([]Link, maxLinkCount),
		navDMeshes,
		navDVerts,
		navDTris,
		navBvtree,
		offMeshCons)

	return buf, err
}
