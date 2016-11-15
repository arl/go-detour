package math32

// Sqr returns the square of x.
//
// Special cases are:
//	Sqr(+Inf) = +Inf
//	Sqr(±0) = ±0
//	Sqrt(NaN) = NaN
func Sqr(x float32) float32 {
	return x * x
}
