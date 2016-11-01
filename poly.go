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
