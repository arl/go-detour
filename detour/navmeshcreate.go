package detour

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"sort"
	"unsafe"

	"github.com/aurelien-rainone/aligned"
	"github.com/aurelien-rainone/gogeo/f32"
	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
	"github.com/fatih/structs"
)

type BVItem struct {
	BMin, BMax [3]uint16
	i          int32
}

type compareItemX []BVItem

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
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareItemX) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

type compareItemY []BVItem

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
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareItemY) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

type compareItemZ []BVItem

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
	return false
}

// Swap swaps the elements with indexes i and j.
func (s compareItemZ) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}

func calcExtends(items []BVItem, imin, imax int32, bmin, bmax []uint16) {
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

func subdivide(items []BVItem, nitems, imin, imax int32, curNode *int32, nodes []bvNode) {
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

		axis := longestAxis(node.Bmax[0]-node.Bmin[0],
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

func createBVTree(params *NavMeshCreateParams, nodes []bvNode) int32 {
	// Build tree
	quantFactor := 1.0 / params.Cs
	items := make([]BVItem, params.PolyCount)
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
				d3.Vec3Min(bmax[:], dv[j*3:])
			}

			// BV-tree uses cs for all dimensions
			it.BMin[0] = uint16(int32Clamp(int32((bmin[0]-params.BMin[0])*quantFactor), 0, 0xffff))
			it.BMin[1] = uint16(int32Clamp(int32((bmin[1]-params.BMin[1])*quantFactor), 0, 0xffff))
			it.BMin[2] = uint16(int32Clamp(int32((bmin[2]-params.BMin[2])*quantFactor), 0, 0xffff))

			it.BMax[0] = uint16(int32Clamp(int32((bmax[0]-params.BMin[0])*quantFactor), 0, 0xffff))
			it.BMax[1] = uint16(int32Clamp(int32((bmax[1]-params.BMin[1])*quantFactor), 0, 0xffff))
			it.BMax[2] = uint16(int32Clamp(int32((bmax[2]-params.BMin[2])*quantFactor), 0, 0xffff))
		} else {
			p := params.Polys[i*params.Nvp*2:]
			it.BMin[0] = params.Verts[p[0]*3+0]
			it.BMin[1] = params.Verts[p[0]*3+1]
			it.BMin[2] = params.Verts[p[0]*3+2]

			it.BMax[0] = it.BMin[0]
			it.BMax[1] = it.BMin[1]
			it.BMax[2] = it.BMin[2]

			for j := int32(1); j < params.Nvp; j++ {
				if p[j] == MESH_NULL_IDX {
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

const MESH_NULL_IDX uint16 = 0xffff

// Represents the source data used to build an navigation mesh tile.
type NavMeshCreateParams struct {

	// Polygon Mesh Attributes
	// Used to create the base navigation graph.
	// See recast.PolyMesh for details related to these attributes.

	Verts     []uint16 // The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
	VertCount int32    // The number vertices in the polygon mesh. [Limit: >= 3]
	Polys     []uint16 // The polygon data. [Size: #polyCount * 2 * #nvp]
	PolyFlags []uint16 // The user defined flags assigned to each polygon. [Size: #polyCount]
	PolyAreas []uint8  // The user defined area ids assigned to each polygon. [Size: #polyCount]
	PolyCount int32    // Number of polygons in the mesh. [Limit: >= 1]
	Nvp       int32    // Number maximum number of vertices per polygon. [Limit: >= 3]

	// Height Detail Attributes (Optional)
	// See #recast.PolyMeshDetail for details related to these attributes.
	DetailMeshes     []int32   // The height detail sub-mesh data. [Size: 4 * #polyCount]
	DetailVerts      []float32 // The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
	DetailVertsCount int32     // The number of vertices in the detail mesh.
	DetailTris       []uint8   // The detail mesh triangles. [Size: 4 * #detailTriCount]
	DetailTriCount   int32     // The number of triangles in the detail mesh.

	// Off-Mesh Connections Attributes (Optional)
	// Used to define a custom point-to-point edge within the navigation graph, an
	// off-mesh connection is a user defined traversable connection made up to two vertices,
	// at least one of which resides within a navigation mesh polygon.

	// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	OffMeshConVerts []float32
	// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	OffMeshConRad []float32
	// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	OffMeshConFlags []uint16
	// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	OffMeshConAreas []uint8
	// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	//
	// 0 = Travel only from endpoint A to endpoint B.<br/>
	// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	OffMeshConDir []uint8
	// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	OffMeshConUserID []uint32
	// The number of off-mesh connections. [Limit: >= 0]
	OffMeshConCount int32

	// Tile Attributes
	// note The tile grid/layer data can be left at zero if the destination is a single tile mesh.
	// @{

	UserID    uint32     ///< The user defined id of the tile.
	TileX     int32      ///< The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
	TileY     int32      ///< The tile's y-grid location within the multi-tile desitation mesh. (Along the z-axis.)
	TileLayer int32      ///< The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
	BMin      [3]float32 ///< The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
	BMax      [3]float32 ///< The maximum bounds of the tile. [(x, y, z)] [Unit: wu]

	// General Configuration Attributes

	WalkableHeight float32 // The agent height. [Unit: wu]
	WalkableRadius float32 // The agent radius. [Unit: wu]
	WalkableClimb  float32 // The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	Cs             float32 // The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	Ch             float32 // The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]

	// True if a bounding volume tree should be built for the tile.
	// note The BVTree is not normally needed for layered navigation meshes.
	BuildBvTree bool
}

// TODO: Better error handling.

// The output data array is allocated using the detour allocator (dtAlloc()).  The method
// used to free the memory will be determined by how the tile is added to the navigation
// mesh.
//
// see NavMesh, NavMesh::addTile()
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
	totPolyCount := params.PolyCount + storedOffMeshConCount
	totVertCount := params.VertCount + storedOffMeshConCount*2

	// Find portal edges which are at tile borders.
	var (
		edgeCount   int32
		portalCount int32
	)
	for i := int32(0); i < params.PolyCount; i++ {
		p := params.Polys[i*2*nvp:]
		for j := int32(0); j < nvp; j++ {
			if p[j] == MESH_NULL_IDX {
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
				if p[j] == MESH_NULL_IDX {
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
				if p[j] == MESH_NULL_IDX {
					break
				}
				nv++
			}
			detailTriCount += nv - 2
		}
	}

	// Calculate data size
	// TODO: to be removed once writing is working and checked
	// we don't need the size, we just write
	headerSize := aligned.AlignN(int(unsafe.Sizeof(MeshHeader{})), 4)
	vertsSize := aligned.AlignN(int(4*3*totVertCount), 4)
	polysSize := aligned.AlignN(int(unsafe.Sizeof(Poly{})*uintptr(totPolyCount)), 4)
	linksSize := aligned.AlignN(int(unsafe.Sizeof(link{})*uintptr(maxLinkCount)), 4)
	detailMeshesSize := aligned.AlignN(int(unsafe.Sizeof(polyDetail{})*uintptr(params.PolyCount)), 4)
	detailVertsSize := aligned.AlignN(int(4*3*uintptr(uniqueDetailVertCount)), 4)
	detailTrisSize := aligned.AlignN(int(1*4*uintptr(detailTriCount)), 4)
	var bvTreeSize int
	if params.BuildBvTree {
		bvTreeSize = aligned.AlignN(int(unsafe.Sizeof(bvNode{})*uintptr(params.PolyCount*2)), 4)
	}
	offMeshConsSize := aligned.AlignN(int(unsafe.Sizeof(OffMeshConnection{})*uintptr(storedOffMeshConCount)), 4)

	// TODO: dataSize will be used to check that the length of the written
	// buffer is what we expect
	dataSize := headerSize + vertsSize + polysSize + linksSize +
		detailMeshesSize + detailVertsSize + detailTrisSize +
		bvTreeSize + offMeshConsSize

	// create a buffer of the total required size
	var data []uint8
	data = make([]uint8, dataSize)

	// create the variable that will hold the values to serialize
	var hdr MeshHeader
	navVerts := make([]float32, 3*totVertCount)
	navPolys := make([]Poly, 3*totVertCount)

	navDMeshes := make([]polyDetail, params.PolyCount)
	navDVerts := make([]float32, 3*uniqueDetailVertCount)
	navDTris := make([]uint8, 4*detailTriCount)
	navBvtree := make([]bvNode, params.PolyCount*2)
	offMeshCons := make([]OffMeshConnection, storedOffMeshConCount)

	// Fill header
	hdr.Magic = navMeshMagic
	hdr.Version = navMeshVersion
	hdr.X = params.TileX
	hdr.Y = params.TileY
	hdr.Layer = params.TileLayer
	hdr.UserID = params.UserID
	hdr.PolyCount = totPolyCount
	hdr.VertCount = totVertCount
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

	log.Println("header", structs.Map(hdr))

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
			if src[j] == MESH_NULL_IDX {
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

	//d += linksSize; // Ignore links; just leave enough space for them. They'll be created on load.

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	if params.DetailMeshes != nil && len(params.DetailMeshes) > 0 {
		var vbase uint16
		for i := int32(0); i < params.PolyCount; i++ {
			dtl := &navDMeshes[i]
			vb := uint8(params.DetailMeshes[i*4+0])
			ndv := uint8(params.DetailMeshes[i*4+1])
			nv := navPolys[i].VertCount
			dtl.VertBase = uint32(vbase)
			dtl.VertCount = ndv - nv
			dtl.TriBase = uint32(params.DetailMeshes[i*4+2])
			dtl.TriCount = uint8(params.DetailMeshes[i*4+3])
			// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
			if ndv-nv != 0 {
				//memcpy(&navDVerts[vbase*3], &params.detailVerts[(vb+nv)*3], sizeof(float)*3*(ndv-nv));
				start, length := (vb+nv)*3, 3*(ndv-nv)
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

	buf := bytes.NewBuffer(data)
	w := aligned.NewWriter(buf, 4, binary.LittleEndian)
	w.WriteVal(hdr)
	w.WriteSlice(navVerts)
	w.WriteSlice(navPolys)
	// TODO: could use a function like Truncate in bytes.Buffer instead of creating an empty buffer
	w.WriteSlice(make([]uint8, linksSize)) // Ignore links; just leave enough space for them. They'll be created on load.

	w.WriteSlice(navDMeshes)
	w.WriteSlice(navDVerts)
	w.WriteSlice(navDTris)
	w.WriteSlice(navBvtree)
	w.WriteSlice(offMeshCons)

	//dtFree(offMeshConClass);

	//*outData = data;
	//*outDataSize = dataSize;

	fmt.Println("data bytes", len(buf.Bytes()))
	return buf.Bytes(), nil
}
