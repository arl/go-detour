package detour

import "math"

type Vec3 []float32

func NewVec3() Vec3 {
	v := make(Vec3, 3)
	return v
}

func NewVec3xyz(x, y, z float32) Vec3 {
	return Vec3{x, y, z}
}

// dtVadd performs a vector addition. dest = v1 + v2
//
//     dest   [out] The result vector.
//     v1     [in]  The base vector.
//     v2     [in]  The vector to add to v1.
func dtVadd(dest, v1, v2 Vec3) {
	dest[0] = v1[0] + v2[0]
	dest[1] = v1[1] + v2[1]
	dest[2] = v1[2] + v2[2]
}

// Performs a scaled vector addition. dest = v1 + (v2 * s)
//
//     dest   [out] The result vector.
//     v1     [in]  The base vector.
//     v1     [in]  The vector to scale and add to v1.
//     s      [in]  The amount to scale v2 by before adding to v1.
func dtVmad(dest, v1, v2 Vec3, s float32) {
	dest[0] = v1[0] + v2[0]*s
	dest[1] = v1[1] + v2[1]*s
	dest[2] = v1[2] + v2[2]*s
}

// Performs a vector subtraction. dest = v1 - v2.
//
//     dest  [out]  The result vector.
//     v1    [in]   The base vector.
//     v2    [in]   The vector to subtract from v1.
func dtVsub(dest, v1, v2 Vec3) {
	dest[0] = v1[0] - v2[0]
	dest[1] = v1[1] - v2[1]
	dest[2] = v1[2] - v2[2]
}

// Performs a vector copy. dest = a
//
//     dest [out]   The result.
//     a    [in]    The vector to copy.
func dtVcopy(dest, a Vec3) {
	dest[0] = a[0]
	dest[1] = a[1]
	dest[2] = a[2]
}

// Selects the minimum value of each element from the specified vectors.
//
//     mn  [in,out] A vector, will be updated with the result.
//     v   [in]     A vector.
func dtVmin(mn, v Vec3) {
	mn[0] = dtMin(mn[0], v[0])
	mn[1] = dtMin(mn[1], v[1])
	mn[2] = dtMin(mn[2], v[2])
}

// Selects the maximum value of each element from the specified vectors.
//
//     mx  [in,out] A vector, will be updated with the result.
//     v   [in]     A vector.
func dtVmax(mx, v Vec3) {
	mx[0] = dtMax(mx[0], v[0])
	mx[1] = dtMax(mx[1], v[1])
	mx[2] = dtMax(mx[2], v[2])
}

// Derives the square of the scalar length of the vector. (len * len)
func dtVlenSqr(v Vec3) float32 {
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
}

// Returns the distance between two points.
func dtVdist(v1, v2 Vec3) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
}

// Performs a linear interpolation between two vectors. v1 toward v2
//
//     dest  [out]  The result vector.
//     v1    [in]   The starting vector.
//     v2    [in]   The destination vector.
//     t     [in]   The interpolation factor. [Limits: 0 <= value <= 1.0]
func dtVlerp(dest, v1, v2 Vec3, t float32) {
	dest[0] = v1[0] + (v2[0]-v1[0])*t
	dest[1] = v1[1] + (v2[1]-v1[1])*t
	dest[2] = v1[2] + (v2[2]-v1[2])*t
}

// Derives the cross product of two vectors. dst = v1 x v2
//
//     dest   [out] The cross product.
//     v1     [in]  A Vector.
//     v2     [in]  A vector.
func dtVcross(dest, v1, v2 Vec3) {
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]
}

// Derives the dot product of two vectors. dest = v1 . v2
//
//     v1     [in]  A Vector.
//     v2     [in]  A vector.
func dtVdot(v1, v2 Vec3) float32 {
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
}

// Derives the dot product of two vectors on the xz-plane. dest = u . v
//
//     u      [in]  A Vector.
//     v      [in]  A vector.
// The vectors are projected onto the xz-plane, so the y-values are ignored.
func dtVdot2D(u, v Vec3) float32 {
	return u[0]*v[0] + u[2]*v[2]
}

/// Returns the minimum of two float32 values.
///  @param[in]        a    Value A
///  @param[in]        b    Value B
///  @return The minimum of the two values.
func dtMin(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

/// Returns the minimum of two int32 values.
///  @param[in]        a    Value A
///  @param[in]        b    Value B
///  @return The minimum of the two values.
func dtiMin(a, b int32) int32 {
	if a < b {
		return a
	}
	return b
}

/// Returns the maximum of two values.
///  @param[in]        a    Value A
///  @param[in]        b    Value B
///  @return The maximum of the two values.
func dtMax(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
