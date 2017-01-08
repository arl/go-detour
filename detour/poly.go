package detour

import "github.com/aurelien-rainone/gogeo/f32/d3"

// Poly defines a polygon within a MeshTile object.
type Poly struct {
	// FirstLink is the index to first link in linked list.
	// (Or nullLink if there is no link.)
	FirstLink uint32

	// Verts are the indices of the polygon's vertices.
	// The actual vertices are located in MeshTile.Verts.
	Verts [VertsPerPolygon]uint16

	// Neis is packed data representing neighbor polygons
	// references and flags for each edge.
	Neis [VertsPerPolygon]uint16

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

// SetArea sets the user defined area id. (limit: < maxAreas)
func (p *Poly) SetArea(a uint8) {
	p.AreaAndType = (p.AreaAndType & 0xc0) | (a & 0x3f)
}

// SetType sets the polygon type. (see: polyTypes.)
func (p *Poly) SetType(t uint8) {
	p.AreaAndType = (p.AreaAndType & 0x3f) | (t << 6)
}

// Area returns the user defined area id.
func (p *Poly) Area() uint8 {
	return p.AreaAndType & 0x3f
}

// Type returns the polygon type. (see: polyTypes)
func (p *Poly) Type() uint8 {
	return p.AreaAndType >> 6
}

// CalcPolyCenter derives and returns the centroid of a convex polygon.
//  idx     polygon indices. [(vertIndex) * nidx]
//  nidx    number of indices in the polygon. (limit: >= 3)
//  verts   polygon vertices. [(x, y, z) * vertCount]
func CalcPolyCenter(idx []uint16, nidx int32, verts []float32) d3.Vec3 {
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
