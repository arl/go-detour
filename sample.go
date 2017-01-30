package recast

// These are just sample areas to use consistent values across the samples.
// The user should specify these based on his needs.
const (
	samplePolyAreaGround uint8 = iota
	samplePolyAreaWater
	samplePolyAreaRoad
	samplePolyAreaDoor
	samplePolyAreaGrass
	samplePolyAreaJump
)

const (
	samplePolyFlagsWalk     uint16 = 0x01 // Ability to walk (ground, grass, road)
	samplePolyFlagsSwim     uint16 = 0x02 // Ability to swim (water).
	samplePolyFlagsDoor     uint16 = 0x04 // Ability to move through doors.
	samplePolyFlagsJump     uint16 = 0x08 // Ability to jump.
	samplePolyFlagsDisabled uint16 = 0x10 // Disabled polygon
	samplePolyFlagsAll      uint16 = 0xff // All abilities.
)
