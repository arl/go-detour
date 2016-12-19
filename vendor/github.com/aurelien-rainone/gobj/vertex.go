package gobj

import (
	"errors"
	"fmt"
	"strconv"
)

// Vertex represents a 4D vertex.
type Vertex [4]float64

// NewVertex2D returns a 2D Vertex.
func NewVertex2D(x, y float64) Vertex {
	return Vertex{x, y, 0, 0}
}

// NewVertex3D returns a 3D Vertex.
func NewVertex3D(x, y, z float64) Vertex {
	return Vertex{x, y, z, 0}
}

// NewVertex4D returns a 4D Vertex.
func NewVertex4D(x, y, z, w float64) Vertex {
	return Vertex{x, y, z, w}
}

// Scale scales every coordinates by a given scale factor.
func (v *Vertex) Scale(f float64) {
	for i := range v {
		v[i] *= f
	}
}

// X returns the vertex X component.
func (v Vertex) X() float64 {
	return v[0]
}

// Y returns the vertex Y component.
func (v Vertex) Y() float64 {
	return v[1]
}

// Z returns the vertex Z component.
func (v Vertex) Z() float64 {
	return v[2]
}

// W returns the vertex W component.
func (v Vertex) W() float64 {
	return v[3]
}

// Set initializes a vertex from a string array where every string represents a
// vertex component.
func (v *Vertex) Set(s []string) error {
	var (
		err error
	)

	if len(s) > 4 {
		return errors.New("Vertex.Set: invalid string length")
	}

	for i := range s {
		if v[i], err = strconv.ParseFloat(s[i], 64); err != nil {
			return fmt.Errorf("invalid syntax \"%v\": %s", s[i], err)
		}
	}

	return nil
}
