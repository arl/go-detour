package recast

// These are just sample areas to use consistent values across the samples.
// The use should specify these base on his needs.
// SamplePolyAreas
const (
	SAMPLE_POLYAREA_GROUND uint8 = iota
	SAMPLE_POLYAREA_WATER
	SAMPLE_POLYAREA_ROAD
	SAMPLE_POLYAREA_DOOR
	SAMPLE_POLYAREA_GRASS
	SAMPLE_POLYAREA_JUMP
)

//enum SamplePolyFlags
const (
	SAMPLE_POLYFLAGS_WALK     uint16 = 0x01 // Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM     uint16 = 0x02 // Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR     uint16 = 0x04 // Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP     uint16 = 0x08 // Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED uint16 = 0x10 // Disabled polygon
	SAMPLE_POLYFLAGS_ALL      uint16 = 0xff // All abilities.
)
