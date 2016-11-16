// Package math32 provides basic constants and mathematical functions for 32
// bits floating point, based off the standard Go math package.
package math32

import "math"

// Mathematical constants.
const (
	E   = float32(math.E)
	Pi  = float32(math.Pi)
	Phi = float32(math.Phi)

	Sqrt2   = float32(math.Sqrt2)
	SqrtE   = float32(math.SqrtE)
	SqrtPi  = float32(math.SqrtPi)
	SqrtPhi = float32(math.SqrtPhi)

	Ln2    = float32(math.Ln2)
	Log2E  = float32(math.Log2E)
	Ln10   = float32(math.Ln10)
	Log10E = float32(math.Log10E)
)

// Floating-point limit values.
// Max is the largest finite value representable by the type.
// SmallestNonzero is the smallest positive, non-zero value representable by the type.
const (
	MaxFloat32             = float32(math.MaxFloat32)
	SmallestNonzeroFloat32 = float32(math.SmallestNonzeroFloat32)
)
