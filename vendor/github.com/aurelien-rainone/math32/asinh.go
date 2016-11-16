package math32

import "math"

// Asinh returns the inverse hyperbolic sine of x.
//
// Special cases are:
//	Asinh(±0) = ±0
//	Asinh(±Inf) = ±Inf
//	Asinh(NaN) = NaN
func Asinh(x float32) float32 {
	return float32(math.Asinh(float64(x)))
}
