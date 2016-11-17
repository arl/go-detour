package d3

import (
	"fmt"

	"github.com/aurelien-rainone/math32"
)

// Vec3 is a 3 dimensions vector. It is made up of a slice of 32 bits floating
// points numbers.
//
// Depending on the context, a Vec3 can also represent a point in 3D space.
type Vec3 []float32

// NewVec3 allocates and returns a new Vec3 where each component has its zero
// value.
func NewVec3() Vec3 {
	return make(Vec3, 3)
}

// NewVec3From allocates and returns a new Vec3 that is the copy of v1.
func NewVec3From(v1 Vec3) Vec3 {
	return Vec3{v1[0], v1[1], v1[2]}
}

// NewVec3XYZ allocates and returns Vec3{x, y, z}.
func NewVec3XYZ(x, y, z float32) Vec3 {
	return Vec3{x, y, z}
}

// component access

// X returns the X component of v.
func (v Vec3) X() float32 {
	return v[0]
}

// Y returns the Y component of v.
func (v Vec3) Y() float32 {
	return v[1]
}

// Z returns the Z component of v.
func (v Vec3) Z() float32 {
	return v[2]
}

// X sets the X component of v.
func (v Vec3) SetX(x float32) {
	v[0] = x
}

// Y sets the Y component of v.
func (v Vec3) SetY(y float32) {
	v[1] = y
}

// Z sets the Z component of v.
func (v Vec3) SetZ(z float32) {
	v[2] = z
}

// Vec3 functions

// Vec3Add performs a vector addition. dest = v1 + v2
//
//     dest   [out] The result vector.
//     v1     [in]  The base vector.
//     v2     [in]  The vector to add to v1.
func Vec3Add(dest, v1, v2 Vec3) {
	dest[0] = v1[0] + v2[0]
	dest[1] = v1[1] + v2[1]
	dest[2] = v1[2] + v2[2]
}

// Vec3SAdd performs a scaled vector addition. dest = v1 + (v2 * s)
//
//     dest   [out] The result vector.
//     v1     [in]  The base vector.
//     v1     [in]  The vector to scale and add to v1.
//     s      [in]  The amount to scale v2 by before adding to v1.
func Vec3SAdd(dest, v1, v2 Vec3, s float32) {
	dest[0] = v1[0] + v2[0]*s
	dest[1] = v1[1] + v2[1]*s
	dest[2] = v1[2] + v2[2]*s
}

// Vec3Sub performs a vector subtraction. dest = v1 - v2.
//
//     dest  [out]  The result vector.
//     v1    [in]   The base vector.
//     v2    [in]   The vector to subtract from v1.
func Vec3Sub(dest, v1, v2 Vec3) {
	dest[0] = v1[0] - v2[0]
	dest[1] = v1[1] - v2[1]
	dest[2] = v1[2] - v2[2]
}

// Vec3Scale scales a vector by value t. dest = v * t
//
//     dest  [out]  The result vector.
//     v     [in]   The vector to scale.
//     t     [in]   The scaling factor.
func Vec3Scale(dest, v Vec3, t float32) {
	dest[0] = v[0] * t
	dest[1] = v[1] * t
	dest[2] = v[2] * t
}

// Vec3Min selects the minimum value of each element from the specified vectors.
//
//     mn  [in,out] A vector, will be updated with the result.
//     v   [in]     A vector.
func Vec3Min(mn, v Vec3) {
	mn[0] = math32.Min(mn[0], v[0])
	mn[1] = math32.Min(mn[1], v[1])
	mn[2] = math32.Min(mn[2], v[2])
}

// Vec3Max selects the maximum value of each element from the specified vectors.
//
//     mx  [in,out] A vector, will be updated with the result.
//     v   [in]     A vector.
func Vec3Max(mx, v Vec3) {
	mx[0] = math32.Max(mx[0], v[0])
	mx[1] = math32.Max(mx[1], v[1])
	mx[2] = math32.Max(mx[2], v[2])
}

// Vec3Lerp performs a linear interpolation between two vectors. v1 toward v2
//
//     dest  [out]  The result vector.
//     v1    [in]   The starting vector.
//     v2    [in]   The destination vector.
//     t     [in]   The interpolation factor. [Limits: 0 <= value <= 1.0]
func Vec3Lerp(dest, v1, v2 Vec3, t float32) {
	dest[0] = v1[0] + (v2[0]-v1[0])*t
	dest[1] = v1[1] + (v2[1]-v1[1])*t
	dest[2] = v1[2] + (v2[2]-v1[2])*t
}

// Vec3Cross derives the cross product of two vectors. dst = v1 x v2
//
//     dest   [out] The cross product.
//     v1     [in]  A Vector.
//     v2     [in]  A vector.
func Vec3Cross(dest, v1, v2 Vec3) {
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]
}

// Vec3 methods

// Add returns a new vector that is the result of v + v1.
//
// It allocates a new vector/slice.
func (v Vec3) Add(v1 Vec3) Vec3 {
	return NewVec3XYZ(
		v[0]+v1[0],
		v[1]+v1[1],
		v[2]+v1[2],
	)
}

// SAdd returns a new vector that is the result of v + (v1 * s).
//
// It allocates a new vector/slice.
func (v Vec3) SAdd(v1 Vec3, s float32) Vec3 {
	return NewVec3XYZ(
		v[0]+v1[0]*s,
		v[1]+v1[1]*s,
		v[2]+v1[2]*s,
	)
}

// Sub returns a new vector that is the result of v - v1.
//
// It allocates a new vector/slice.
func (v Vec3) Sub(v1 Vec3) Vec3 {
	return NewVec3XYZ(
		v[0]-v1[0],
		v[1]-v1[1],
		v[2]-v1[2],
	)
}

// Scale returns a new vector that is the result of v * t
func (v Vec3) Scale(t float32) Vec3 {
	return NewVec3XYZ(
		v[0]*t,
		v[1]*t,
		v[2]*t,
	)
}

// Assign assign the component of v1 to v. v = v1
func (v Vec3) Assign(v1 Vec3) {
	v[0] = v1[0]
	v[1] = v1[1]
	v[2] = v1[2]
}

// LenSqr derives the scalar scalar length of the vector. (len)
func (v Vec3) Len() float32 {
	return math32.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
}

// LenSqr derives the square of the scalar length of the vector. (len * len)
func (v Vec3) LenSqr() float32 {
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
}

// Dist returns the distance between v and v1. d = dist(v, v2)
func (v Vec3) Dist(v1 Vec3) float32 {
	dx := v1[0] - v[0]
	dy := v1[1] - v[1]
	dz := v1[2] - v[2]
	return math32.Sqrt(dx*dx + dy*dy + dz*dz)
}

// DistSqr returns the square of the distance between two points.
func (v Vec3) DistSqr(v1 Vec3) float32 {
	dx := v1[0] - v[0]
	dy := v1[1] - v[1]
	dz := v1[2] - v[2]
	return dx*dx + dy*dy + dz*dz
}

// Dist2D derives the distance between v and v2 on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
func (v Vec3) Dist2D(v1 Vec3) float32 {
	dx := v1[0] - v[0]
	dz := v1[2] - v[2]
	return math32.Sqrt(dx*dx + dz*dz)
}

// Dist2DSqr derives the square of the distance between v and v2 on the
// xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
func (v Vec3) Dist2DSqr(v1 Vec3) float32 {
	dx := v1[0] - v[0]
	dz := v1[2] - v[2]
	return dx*dx + dz*dz
}

// Normalize normalizes the vector.
func (v Vec3) Normalize() {
	d := 1.0 / math32.Sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
	v[0] *= d
	v[1] *= d
	v[2] *= d
}

// Lerp returns the result vector of a linear interpolation between two
// vectors. v toward v1.
//
// The interpolation factor t should be comprised betwenn 0 and 1.0.
// [Limits: 0 <= value <= 1.0]
func (v Vec3) Lerp(v1 Vec3, t float32) Vec3 {
	return Vec3{
		v[0] + (v1[0]-v[0])*t,
		v[1] + (v1[1]-v[1])*t,
		v[2] + (v1[2]-v[2])*t,
	}
}

// Cross returns the cross product of two vectors. v x v1
func (v Vec3) Cross(v1 Vec3) Vec3 {
	return Vec3{
		v[1]*v1[2] - v[2]*v1[1],
		v[2]*v1[0] - v[0]*v1[2],
		v[0]*v1[1] - v[1]*v1[0],
	}
}

// Dot derives the dot product of two vectors. v . v1
func (v Vec3) Dot(v1 Vec3) float32 {
	return v[0]*v1[0] + v[1]*v1[1] + v[2]*v1[2]
}

// Vec3Dot2D derives the dot product of two vectors on the xz-plane. u . v
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
func (v Vec3) Dot2D(u Vec3) float32 {
	return v[0]*u[0] + v[2]*u[2]
}

// Perp2D derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
//
// u is the LHV vector [(x, y, z)]
// v is the RHV vector [(x, y, z)]
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
func (v Vec3) Perp2D(u Vec3) float32 {
	return v[2]*u[0] - v[0]*u[2]
}

// Approx reports wether v and v1 are approximately equal.
//
// Element-wise approximation uses math32.Approx()
func (v Vec3) Approx(v1 Vec3) bool {
	return math32.Approx(v[0], v1[0]) &&
		math32.Approx(v[1], v1[1]) &&
		math32.Approx(v[2], v1[2])
}

// String returns a string representation of v like "(3,4)".
func (v Vec3) String() string {
	return fmt.Sprintf("(%f,%f,%f)", v[0], v[1], v[2])
}

func (v *Vec3) Set(s string) error {
	if _, err := fmt.Sscanf(s, "(%f,%f,%f)", (*v)[0], (*v)[1], (*v)[2]); err != nil {
		return fmt.Errorf("invalid syntax \"%s\"", s)
	}
	return nil
}
