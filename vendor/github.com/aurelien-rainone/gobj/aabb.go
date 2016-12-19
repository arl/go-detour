package gobj

import (
	"fmt"
	"math"
)

// AABB is an axis-aligned bounding box.
type AABB struct {
	MinX, MaxX float64
	MinY, MaxY float64
	MinZ, MaxZ float64
}

// NewAABB initializes the bounding box.
//
// The bounding box will be valid after the first call to extend.
func NewAABB() AABB {
	return AABB{
		MinX: math.Inf(1),
		MinY: math.Inf(1),
		MinZ: math.Inf(1),
		MaxX: math.Inf(-1),
		MaxY: math.Inf(-1),
		MaxZ: math.Inf(-1),
	}
}

func (bb AABB) String() string {
	return fmt.Sprintf("x[%f, %f], y[%f, %f], z[%f, %f]",
		bb.MinX, bb.MaxX,
		bb.MinY, bb.MaxY,
		bb.MinZ, bb.MaxZ)
}
