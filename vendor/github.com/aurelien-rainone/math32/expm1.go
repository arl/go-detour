package math32

import "math"

// Expm1 returns e**x - 1, the base-e exponential of x minus 1.
// It is more accurate than Exp(x) - 1 when x is near zero.
//
// Special cases are:
//	Expm1(+Inf) = +Inf
//	Expm1(-Inf) = -1
//	Expm1(NaN) = NaN
// Very large values overflow to -1 or +Inf.
func Expm1(x float32) float32 {
	return float32(math.Expm1(float64(x)))
}
