package math32

import "math"

// Erf returns the error function of x.
//
// Special cases are:
//	Erf(+Inf) = 1
//	Erf(-Inf) = -1
//	Erf(NaN) = NaN
func Erf(x float32) float32 {
	return float32(math.Erf(float64(x)))
}

// Erfc returns the complementary error function of x.
//
// Special cases are:
//	Erfc(+Inf) = 0
//	Erfc(-Inf) = 2
//	Erfc(NaN) = NaN
func Erfc(x float32) float32 {
	return float32(math.Erfc(float64(x)))
}
