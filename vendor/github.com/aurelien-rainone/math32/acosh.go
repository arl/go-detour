package math32

import "math"

// Acosh returns the inverse hyperbolic cosine of x.
//
// Special cases are:
//	Acosh(+Inf) = +Inf
//	Acosh(x) = NaN if x < 1
//	Acosh(NaN) = NaN
func Acosh(x float32) float32 {
	return float32(math.Acosh(float64(x)))
}
