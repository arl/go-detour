package math32

import "math"

// Log returns the natural logarithm of x.
//
// Special cases are:
//	Log(+Inf) = +Inf
//	Log(0) = -Inf
//	Log(x < 0) = NaN
//	Log(NaN) = NaN
func Log(x float32) float32 {
	return float32(math.Log(float64(x)))
}
