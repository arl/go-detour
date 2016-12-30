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

// Heighfield border flag.
// If a heightfield region ID has this bit set, then the region is a border
// region and its spans are considered unwalkable.
// (Used during the region and contour build process.)
// @see rcCompactSpan::reg
const RC_BORDER_REG uint16 = 0x8000

// Polygon touches multiple regions.
// If a polygon has this region ID it was merged with or created
// from polygons of different regions during the polymesh
// build step that removes redundant border vertices.
// (Used during the polymesh and detail polymesh build processes)
// @see rcPolyMesh::regs
const RC_MULTIPLE_REGS uint16 = 0

// Border vertex flag.
// If a region ID has this bit set, then the associated element lies on
// a tile border. If a contour vertex's region ID has this bit set, the
// vertex will later be removed in order to match the segments and vertices
// at tile boundaries.
// (Used during the build process.)
// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
const RC_BORDER_VERTEX int32 = 0x10000

// Area border flag.
// If a region ID has this bit set, then the associated element lies on
// the border of an area.
// (Used during the region and contour build process.)
// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
const RC_AREA_BORDER int32 = 0x20000
