package detour

import "github.com/aurelien-rainone/gogeo/f32/d3"

// DtPoly defines a polygon within a DtMeshTile object.
type DtPoly struct {
	// FirstLink is the index to first link in linked list.
	// (Or DT_NULL_LINK if there is no link.)
	FirstLink uint32

	// Verts are the indices of the polygon's vertices.
	// The actual vertices are located in DtMeshTile.Verts.
	Verts [DT_VERTS_PER_POLYGON]uint16

	// Neis is packed data representing neighbor polygons
	// references and flags for each edge.
	Neis [DT_VERTS_PER_POLYGON]uint16

	// Flags is an user-defined polygon flags.
	Flags uint16

	// VertCount is the number of vertices in the polygon.
	VertCount uint8

	// bit-packed area id and polygon type.
	//
	// Note: use SetArea/SetType/Area/Type functions to access those values.
	// This value is exported in order to be accessible from reflect during
	// unmarshalling from binary data.
	AreaAndType uint8
}

// SetArea sets the user defined area id. (limit: < DT_MAX_AREAS)
func (p *DtPoly) SetArea(a uint8) {
	p.AreaAndType = (p.AreaAndType & 0xc0) | (a & 0x3f)
}

// SetType sets the polygon type. (see: DtPolyTypes.)
func (p *DtPoly) SetType(t uint8) {
	p.AreaAndType = (p.AreaAndType & 0x3f) | (t << 6)
}

// Area returns the user defined area id.
func (p *DtPoly) Area() uint8 {
	return p.AreaAndType & 0x3f
}

// Type returns the polygon type. (see: DtPolyTypes)
func (p *DtPoly) Type() uint8 {
	return p.AreaAndType >> 6
}

// DtCalcPolyCenter derives and returns the centroid of a convex polygon.
//  idx     polygon indices. [(vertIndex) * nidx]
//  nidx    number of indices in the polygon. (limit: >= 3)
//  verts   polygon vertices. [(x, y, z) * vertCount]
func DtCalcPolyCenter(idx []uint16, nidx int32, verts []float32) d3.Vec3 {
	tc := d3.NewVec3()
	var j int32
	for j = 0; j < nidx; j++ {
		start := idx[j] * 3
		v := verts[start : start+3]
		tc[0] += v[0]
		tc[1] += v[1]
		tc[2] += v[2]
	}
	return tc.Scale(1 / float32(nidx))
}
