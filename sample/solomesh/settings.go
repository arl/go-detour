package solomesh

import "github.com/aurelien-rainone/go-detour/sample"

// Settings contains all the settings required for a SoloMesh.
type Settings struct {
	// Rasterization settings
	CellSize   float32
	CellHeight float32

	// Agent properties
	AgentHeight   float32
	AgentMaxClimb float32
	AgentRadius   float32

	// Region
	RegionMinSize   int32
	RegionMergeSize int32

	// Polygonization
	EdgeMaxLen   int32
	EdgeMaxError float32
	VertsPerPoly int32

	// Detail Mesh
	DetailSampleDist     float32
	DetailSampleMaxError float32

	WalkableSlopeAngle float32

	Partition sample.PartitionType
}

// NewSettings returns a new Settings struct filled with default values.
func NewSettings() Settings {
	return Settings{
		CellSize:             float32(0.3),
		CellHeight:           float32(0.2),
		AgentHeight:          float32(2.0),
		AgentMaxClimb:        float32(0.9),
		AgentRadius:          float32(0.6),
		RegionMinSize:        int32(8),
		RegionMergeSize:      int32(20),
		EdgeMaxLen:           int32(12),
		EdgeMaxError:         float32(1.3),
		VertsPerPoly:         int32(6),
		DetailSampleDist:     float32(6),
		DetailSampleMaxError: float32(1),
		WalkableSlopeAngle:   float32(45),
		Partition:            sample.PartitionMonotone,
	}
}
