package math32

import "math"

var (
	epsilon32 float32
)

func init() {
	epsilon32 = math.Nextafter32(1, 2) - 1
}

// Approx returns true if x ~= y
func Approx(x, y float32) bool {
	eps := epsilon32 * 100
	return Abs(x-y) < eps*(1.0+Max(Abs(x), Abs(y)))
}

// ApproxEpsilon returns true if x ~= y, using provided epsilon value.
func ApproxEpsilon(x, y float32, eps float32) bool {
	return Abs(x-y) < eps*(1.0+Max(Abs(x), Abs(y)))
}
