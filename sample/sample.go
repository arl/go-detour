package sample

// These are just sample areas to use consistent values across the samples.
// The user should specify these based on his needs.
const (
	SamplePolyAreaGround uint8 = iota
	SamplePolyAreaWater
	SamplePolyAreaRoad
	SamplePolyAreaDoor
	SamplePolyAreaGrass
	SamplePolyAreaJump
)

const (
	SamplePolyFlagsWalk     uint16 = 0x01 // Ability to walk (ground, grass, road)
	SamplePolyFlagsSwim     uint16 = 0x02 // Ability to swim (water).
	SamplePolyFlagsDoor     uint16 = 0x04 // Ability to move through doors.
	SamplePolyFlagsJump     uint16 = 0x08 // Ability to jump.
	SamplePolyFlagsDisabled uint16 = 0x10 // Disabled polygon
	SamplePolyFlagsAll      uint16 = 0xff // All abilities.
)
