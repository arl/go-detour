package solomesh

// Settings contains all the settings required for a SoloMesh.
type Settings struct {
	// Rasterization settings
	cellSize   float32
	cellHeight float32

	// Agent properties
	agentHeight   float32
	agentMaxClimb float32
	agentRadius   float32

	// Region
	regionMinSize   int32
	regionMergeSize int32

	// Polygonization
	edgeMaxLen   int32
	edgeMaxError float32
	vertsPerPoly int32

	// Detail Mesh
	detailSampleDist     float32
	detailSampleMaxError float32

	walkableSlopeAngle float32
}

// NewSettings returns a new Settings struct filled with default values.
func NewSettings() Settings {
	return Settings{
		cellSize:             float32(0.3),
		cellHeight:           float32(0.2),
		agentHeight:          float32(2.0),
		agentMaxClimb:        float32(0.9),
		agentRadius:          float32(0.6),
		regionMinSize:        int32(8),
		regionMergeSize:      int32(20),
		edgeMaxLen:           int32(12),
		edgeMaxError:         float32(1.3),
		vertsPerPoly:         int32(6),
		detailSampleDist:     float32(6),
		detailSampleMaxError: float32(1),
		walkableSlopeAngle:   float32(45),
	}
}
