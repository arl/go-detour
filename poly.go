package detour

/// Defines a polygon within a dtMeshTile object.
type DtPoly struct {
	/// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	FirstLink uint32

	/// The indices of the polygon's vertices.
	/// The actual vertices are located in dtMeshTile::verts.
	Verts [DT_VERTS_PER_POLYGON]uint16

	/// Packed data representing neighbor polygons references and flags for each edge.
	Neis [DT_VERTS_PER_POLYGON]uint16

	/// The user defined polygon flags.
	Flags uint16

	/// The number of vertices in the polygon.
	VertCount uint8

	/// The bit packed area id and polygon type.
	/// @note Use the structure's set and get methods to acess this value.
	AreaAndtype uint8
}

/// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
func (p *DtPoly) SetArea(a uint8) {
	p.AreaAndtype = (p.AreaAndtype & 0xc0) | (a & 0x3f)
}

/// Sets the polygon type. (See: #dtPolyTypes.)
func (p *DtPoly) SetType(t uint8) {
	p.AreaAndtype = (p.AreaAndtype & 0x3f) | (t << 6)
}

/// Gets the user defined area id.
func (p *DtPoly) Area() uint8 {
	return p.AreaAndtype & 0x3f
}

/// Gets the polygon type. (See: #dtPolyTypes)
func (p *DtPoly) Type() uint8 {
	return p.AreaAndtype >> 6
}

/// Derives the centroid of a convex polygon.
///  @param[out]	tc		The centroid of the polgyon. [(x, y, z)]
///  @param[in]		idx		The polygon indices. [(vertIndex) * @p nidx]
///  @param[in]		nidx	The number of indices in the polygon. [Limit: >= 3]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * vertCount]
func DtCalcPolyCenter(tc []float32, idx []uint16, nidx int32, verts []float32) {
	tc[0] = 0.0
	tc[1] = 0.0
	tc[2] = 0.0
	var j int32
	for j = 0; j < nidx; j++ {
		start := idx[j] * 3
		v := verts[start : start+3]
		tc[0] += v[0]
		tc[1] += v[1]
		tc[2] += v[2]
	}
	s := float32(1.0 / float64(nidx))
	tc[0] *= s
	tc[1] *= s
	tc[2] *= s
}
