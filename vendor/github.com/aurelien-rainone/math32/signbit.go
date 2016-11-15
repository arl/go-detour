package math32

import "math"

// Signbit returns true if x is negative or negative zero.
func Signbit(x float32) bool {
	return math.Signbit(float64(x))
}
