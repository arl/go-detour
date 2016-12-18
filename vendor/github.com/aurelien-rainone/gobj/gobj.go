package gobj

import (
	"errors"
	"fmt"
	"math"
	"strconv"
	"strings"
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

// Polygon represents a polygonal face-element.
type Polygon []Vertex

// Scale applies f scale factor to every coord of the polygon vertices.
func (p *Polygon) Scale(f float64) {
	for i := range *p {
		(*p)[i].Scale(f)
	}
}

// AABB computes and returns the axis-aligned bounding-box
// of the polygon.
func (p *Polygon) AABB() AABB {
	bb := NewAABB()
	for _, v := range *p {
		updateMin(&bb.MinX, v.X())
		updateMin(&bb.MinY, v.Y())
		updateMin(&bb.MinZ, v.Z())
		updateMax(&bb.MaxX, v.X())
		updateMax(&bb.MaxY, v.Y())
		updateMax(&bb.MaxZ, v.Z())
	}
	return bb
}

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

func (bb *AABB) extend(other AABB) {
	// update the min and max for each coord
	updateMin(&bb.MinX, other.MinX)
	updateMin(&bb.MinY, other.MinY)
	updateMin(&bb.MinZ, other.MinZ)
	updateMax(&bb.MaxX, other.MaxX)
	updateMax(&bb.MaxY, other.MaxY)
	updateMax(&bb.MaxZ, other.MaxZ)
}

// Scale scales the axis aligned bounding box.
func (bb *AABB) Scale(f float64) {
	bb.MinX *= f
	bb.MinY *= f
	bb.MinZ *= f
	bb.MaxX *= f
	bb.MaxY *= f
	bb.MaxZ *= f
}

func (bb AABB) String() string {
	return fmt.Sprintf("x[%f, %f], y[%f, %f], z[%f, %f]",
		bb.MinX, bb.MaxX,
		bb.MinY, bb.MaxY,
		bb.MinZ, bb.MaxZ)
}

// OBJFile describes the content of an OBJ geometry definition file.
type OBJFile struct {
	verts []Vertex
	polys []Polygon
	aabb  AABB
}

// Verts returns the slice of vertices contained in the OBJ file.
func (of OBJFile) Verts() []Vertex {
	return of.verts
}

// Polys returns the slice of polygons contained in the OBJ file.
func (of OBJFile) Polys() []Polygon {
	return of.polys
}

// AABB returns the minimum axis-aligned bouding box containing every vertices
// contained in the OBJ file.
func (of OBJFile) AABB() AABB {
	return of.aabb
}

func (of *OBJFile) parseVertex(kw string, data []string) error {
	v := Vertex{}
	err := v.Set(data)
	if err != nil {
		return err
	}
	// discard the Z coordinate
	of.verts = append(of.verts, v)
	return nil
}

func (of *OBJFile) parseFace(kw string, data []string) error {
	var p Polygon // polygonal face currently filled
	for _, s := range data {
		// read the indices of the face vertices
		sidx := strings.Split(s, "/")[0]
		vidx, err := strconv.Atoi(sidx)
		if err != nil {
			return fmt.Errorf("invalid vertex coordinate value \"%s\"", s)
		}
		p = append(p, of.verts[vidx-1])
	}

	// extend the mesh bounding box with the polygon's one
	of.aabb.extend(p.AABB())
	of.polys = append(of.polys, p)
	return nil
}

// DumpInfo dumps some informations about the OBJ file.
func (of *OBJFile) DumpInfo() string {
	nfo := fmt.Sprintln("num verts:", len(of.verts))
	nfo += fmt.Sprintln("num polys:", len(of.polys))
	nfo += fmt.Sprintln("aabb     :", of.aabb)
	return nfo
}

// updateMin checks if a > b, then a will be set to the value of b.
func updateMin(a *float64, b float64) {
	if b < *a {
		*a = b
	}
}

// updateMax checks if a < b, then a will be set to the value of b.
func updateMax(a *float64, b float64) {
	if *a < b {
		*a = b
	}
}
