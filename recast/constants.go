package recast

// Contour build flags.
// see BuildContours
const (
	// Tessellate solid (impassable) edges during contour simplification.
	ContourTessWallEdges int32 = 0x01
	// Tessellate edges between areas during contour simplification.
	ContourTessAreaEdges int32 = 0x02
)

// Applied to the region id field of contour vertices in order to extract the region id.
// The region id field of a vertex may have several flags applied to it.  So the
// fields value can't be used directly.
// see Contour.verts, Contour.rverts
const contourRegMask int32 = 0xffff

// A value which indicates an invalid index within a mesh.
// note This does not necessarily indicate an error.
// see PolyMesh.polys
const meshNullIdx uint16 = 0xffff

// Represents the null area.
// When a data element is given this value it is considered to no longer be
// assigned to a usable area. (E.g. It is unwalkable.)
const nullArea uint8 = 0

// WalkableArea is he default area id used to indicate a walkable polygon.
// This is also the maximum allowed area id, and the only non-null area id
// recognized by some steps in the build process.
const WalkableArea uint8 = 63

// The value returned by GetCon if the specified direction is not connected to
// another span. (Has no neighbor.)
const notConnected int32 = 0x3f

// Heighfield border flag.
// If a heightfield region ID has this bit set, then the region is a border
// region and its spans are considered unwalkable.
// (Used during the region and contour build process.)
// see CompactSpan.reg
const borderReg uint16 = 0x8000

// Polygon touches multiple regions.
// If a polygon has this region ID it was merged with or created
// from polygons of different regions during the polymesh
// build step that removes redundant border vertices.
// (Used during the polymesh and detail polymesh build processes)
// see PolyMesh.regs
const multipleRegs uint16 = 0

// Border vertex flag.
// If a region ID has this bit set, then the associated element lies on a tile
// border. If a contour vertex's region ID has this bit set, the vertex will
// later be removed in order to match the segments and vertices at tile
// boundaries.
// (Used during the build process.)
// see CompactSpan.reg, Contour.verts, Contour.rverts
const borderVertex int32 = 0x10000

// Area border flag.
// If a region ID has this bit set, then the associated element lies on
// the border of an area.
// (Used during the region and contour build process.)
// see CompactSpan.reg, Contour.verts, Contour.rverts
const areaBorder int32 = 0x20000
