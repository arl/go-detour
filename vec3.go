package detour

import "math"

/// Performs a vector addition. (@p v1 + @p v2)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
func dtVadd(dest, v1, v2 []float32) {
	dest[0] = v1[0] + v2[0]
	dest[1] = v1[1] + v2[1]
	dest[2] = v1[2] + v2[2]
}

/// Performs a vector subtraction. (@p v1 - @p v2)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
func dtVsub(dest, v1, v2 []float32) {
	dest[0] = v1[0] - v2[0]
	dest[1] = v1[1] - v2[1]
	dest[2] = v1[2] - v2[2]
}

/// Performs a vector copy.
///  @param[out]	dest	The result. [(x, y, z)]
///  @param[in]		a		The vector to copy. [(x, y, z)]
func dtVcopy(dest, a []float32) {
	dest[0] = a[0]
	dest[1] = a[1]
	dest[2] = a[2]
}

/// Selects the minimum value of each element from the specified vectors.
///  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]	v	A vector. [(x, y, z)]
func dtVmin(mn, v []float32) {
	mn[0] = dtMin(mn[0], v[0])
	mn[1] = dtMin(mn[1], v[1])
	mn[2] = dtMin(mn[2], v[2])
}

/// Selects the maximum value of each element from the specified vectors.
///  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]		v	A vector. [(x, y, z)]
func dtVmax(mx, v []float32) {
	mx[0] = dtMax(mx[0], v[0])
	mx[1] = dtMax(mx[1], v[1])
	mx[2] = dtMax(mx[2], v[2])
}

/// Returns the minimum of two float32 values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The minimum of the two values.
func dtMin(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

/// Returns the minimum of two int32 values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The minimum of the two values.
func dtiMin(a, b int32) int32 {
	if a < b {
		return a
	}
	return b
}

/// Returns the maximum of two values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The maximum of the two values.
func dtMax(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

/// Derives the square of the scalar length of the vector. (len * len)
///  @param[in]		v The vector. [(x, y, z)]
/// @return The square of the scalar length of the vector.
func dtVlenSqr(v []float32) float32 {
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
}

/// Returns the distance between two points.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The distance between the two points.
func dtVdist(v1, v2 []float32) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
}

/// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
///  @param[out]	dest	The result vector. [(x, y, x)]
///  @param[in]		v1		The starting vector.
///  @param[in]		v2		The destination vector.
///	 @param[in]		t		The interpolation factor. [Limits: 0 <= value <= 1.0]
func dtVlerp(dest, v1, v2 []float32, t float32) {
	dest[0] = v1[0] + (v2[0]-v1[0])*t
	dest[1] = v1[1] + (v2[1]-v1[1])*t
	dest[2] = v1[2] + (v2[2]-v1[2])*t
}

/// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
///  @param[in]		u		A vector [(x, y, z)]
///  @param[in]		v		A vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
func dtVdot2D(u, v []float32) float32 {
	return u[0]*v[0] + u[2]*v[2]
}
