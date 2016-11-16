package math32

import "math"

// Frexp breaks f into a normalized fraction
// and an integral power of two.
// It returns frac and exp satisfying f == frac × 2**exp,
// with the absolute value of frac in the interval [½, 1).
//
// Special cases are:
//	Frexp(±0) = ±0, 0
//	Frexp(±Inf) = ±Inf, 0
//	Frexp(NaN) = NaN, 0
func Frexp(f float32) (frac float32, exp int) {
	frac64, e := math.Frexp(float64(f))
	return float32(frac64), e
}
