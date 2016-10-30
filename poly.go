package detour

/// Defines a polygon within a dtMeshTile object.
type dtPoly struct {
	/// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	firstLink uint

	/// The indices of the polygon's vertices.
	/// The actual vertices are located in dtMeshTile::verts.
	verts [DT_VERTS_PER_POLYGON]uint16

	/// Packed data representing neighbor polygons references and flags for each edge.
	neis [DT_VERTS_PER_POLYGON]uint16

	/// The user defined polygon flags.
	flags uint16

	/// The number of vertices in the polygon.
	vertCount uint8

	/// The bit packed area id and polygon type.
	/// @note Use the structure's set and get methods to acess this value.
	areaAndtype uint8
}

/// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
func (p *dtPoly) setArea(a uint8) {
	p.areaAndtype = (p.areaAndtype & 0xc0) | (a & 0x3f)
}

/// Sets the polygon type. (See: #dtPolyTypes.)
func (p *dtPoly) setType(t uint8) {
	p.areaAndtype = (p.areaAndtype & 0x3f) | (t << 6)
}

/// Gets the user defined area id.
func (p *dtPoly) getArea() uint8 {
	return p.areaAndtype & 0x3f
}

/// Gets the polygon type. (See: #dtPolyTypes)
func (p *dtPoly) getType() uint8 {
	return p.areaAndtype >> 6
}
