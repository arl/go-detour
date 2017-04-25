package sample

// PartitionType represents a specific heightfield partitioning method.
type PartitionType int

const (
	// PartitionWatershed uses the watershed partitioning method
	PartitionWatershed PartitionType = iota
	// PartitionMonotone uses the monotone partitioning method
	PartitionMonotone
	// PartitionLayers uses the layer partitioning method
	PartitionLayers
)

// These are just sample areas to use consistent values across the samples.
// The user should specify these based on his needs.
const (
	PolyAreaGround = iota
	PolyAreaWater
	PolyAreaRoad
	PolyAreaDoor
	PolyAreaGrass
	PolyAreaJump
)

const (
	PolyFlagsWalk     = 0x01   // Ability to walk (ground, grass, road)
	PolyFlagsSwim     = 0x02   // Ability to swim (water).
	PolyFlagsDoor     = 0x04   // Ability to move through doors.
	PolyFlagsJump     = 0x08   // Ability to jump.
	PolyFlagsDisabled = 0x10   // Disabled polygon
	PolyFlagsAll      = 0xffff // All abilities.
)
