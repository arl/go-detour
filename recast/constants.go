package recast

// Contour build flags.
// @see rcBuildContours
//enum rcBuildContoursFlags
const (
	RC_CONTOUR_TESS_WALL_EDGES int32 = 0x01 ///< Tessellate solid (impassable) edges during contour simplification.
	RC_CONTOUR_TESS_AREA_EDGES int32 = 0x02 ///< Tessellate edges between areas during contour simplification.
)

// Applied to the region id field of contour vertices in order to extract the region id.
// The region id field of a vertex may have several flags applied to it.  So the
// fields value can't be used directly.
// @see rcContour::verts, rcContour::rverts
const RC_CONTOUR_REG_MASK int32 = 0xffff

// An value which indicates an invalid index within a mesh.
// @note This does not necessarily indicate an error.
// @see rcPolyMesh::polys
const RC_MESH_NULL_IDX uint16 = 0xffff

// Represents the null area.
// When a data element is given this value it is considered to no longer be
// assigned to a usable area.  (E.g. It is unwalkable.)
const RC_NULL_AREA uint8 = 0

// The default area id used to indicate a walkable polygon.
// This is also the maximum allowed area id, and the only non-null area id
// recognized by some steps in the build process.
const RC_WALKABLE_AREA uint8 = 63

// The value returned by #rcGetCon if the specified direction is not connected
// to another span. (Has no neighbor.)
const RC_NOT_CONNECTED int32 = 0x3f