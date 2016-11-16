package math32

import "math"

// Atanh returns the inverse hyperbolic tangent of x.
//
// Special cases are:
//	Atanh(1) = +Inf
//	Atanh(±0) = ±0
//	Atanh(-1) = -Inf
//	Atanh(x) = NaN if x < -1 or x > 1
//	Atanh(NaN) = NaN
func Atanh(x float32) float32 {
	return float32(math.Atanh(float64(x)))
}
