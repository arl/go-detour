package recast

// Config specifies a configuration to use when performing Recast builds.
type Config struct {
	// The width of the field along the x-axis.
	// [Limit: >= 0] [Units: vx]
	Width int32

	// The height of the field along the z-axis.
	// [Limit: >= 0] [Units: vx]
	Height int32

	// The width/height size of tile's on the xz-plane.
	// [Limit: >= 0] [Units: vx]
	TileSize int32

	// The size of the non-navigable border around the heightfield.
	// [Limit: >=0] [Units: vx]
	BorderSize int32

	// The xz-plane cell size to use for fields.
	// [Limit: > 0] [Units: wu]
	Cs float32

	// The y-axis cell size to use for fields.
	// [Limit: > 0] [Units: wu]
	Ch float32

	// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	BMin [3]float32

	// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	BMax [3]float32

	// The maximum slope that is considered walkable.
	// [Limits: 0 <= value < 90] [Units: Degrees]
	WalkableSlopeAngle float32

	// Minimum floor to 'ceiling' height that will still allow the
	// floor area to be considered walkable. [Limit: >= 3] [Units: vx]
	WalkableHeight int32

	// Maximum ledge height that is considered to still be
	// traversable. [Limit: >=0] [Units: vx]
	WalkableClimb int32

	// The distance to erode/shrink the walkable area of the
	// heightfield away from obstructions. [Limit: >=1] [Units: vx]
	WalkableRadius int32

	// The maximum allowed length for contour edges along the border
	// of the mesh. [Limit: >=0] [Units: vx]
	MaxEdgeLen int32

	// The maximum distance a simplfied contour's border edges should
	// deviate the original raw contour. [Limit: >=0] [Units: vx]
	MaxSimplificationError float32

	// The minimum number of cells allowed to form isolated island
	// areas.  [Limit: >=0] [Units: vx]
	MinRegionArea int32

	// Any regions with a span count smaller than this value will, if
	// possible, be merged with larger regions.
	// [Limit: >=0] [Units: vx]
	MergeRegionArea int32

	// The maximum number of vertices allowed for polygons generated
	// during the contour to polygon conversion process. [Limit: >= 3]
	MaxVertsPerPoly int32

	// Sets the sampling distance to use when generating the detail
	// mesh. (For height detail only.)
	// [Limits: 0 or >= 0.9] [Units: wu]
	DetailSampleDist float32

	// The maximum distance the detail mesh surface should deviate
	// from heightfield data. (For height detail only.)
	// [Limit: >=0] [Units: wu]
	DetailSampleMaxError float32
}
