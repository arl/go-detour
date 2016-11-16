package math32

import "math"

// Exp returns e**x, the base-e exponential of x.
//
// Special cases are:
//	Exp(+Inf) = +Inf
//	Exp(NaN) = NaN
// Very large values overflow to 0 or +Inf.
// Very small values underflow to 1.
func Exp(x float32) float32 {
	return float32(math.Exp(float64(x)))
}

// Exp2 returns 2**x, the base-2 exponential of x.
//
// Special cases are the same as Exp.
func Exp2(x float32) float32 {
	return float32(math.Exp2(float64(x)))
}
