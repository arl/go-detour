package tilemesh

import (
	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/go-detour/sample"
)

// DefaultSettings returns a recast.BuildSettings with default values for tile
// mesh sample.
func DefaultSettings() recast.BuildSettings {
	return recast.BuildSettings{
		CellSize:             float32(0.3),
		CellHeight:           float32(0.2),
		AgentHeight:          float32(2.0),
		AgentMaxClimb:        float32(0.9),
		AgentMaxSlope:        float32(45),
		AgentRadius:          float32(0.6),
		RegionMinSize:        8,
		RegionMergeSize:      20,
		EdgeMaxLen:           12,
		EdgeMaxError:         float32(1.3),
		VertsPerPoly:         6,
		DetailSampleDist:     float32(6),
		DetailSampleMaxError: float32(1),
		PartitionType:        int32(sample.PartitionMonotone),
		TileSize:             32,
	}
}
