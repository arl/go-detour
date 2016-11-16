package math32

import "math"

// Copysign returns a value with the magnitude
// of x and the sign of y.
func Copysign(x, y float32) float32 {
	return float32(math.Copysign(float64(x), float64(y)))
}
