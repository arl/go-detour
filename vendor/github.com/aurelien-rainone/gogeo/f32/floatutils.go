// Copyright 2016 Aurélien Rainone. All rights reserved.
// Use of this source code is governed by MIT license.
// license that can be found in the LICENSE file.
//
// Part of this code has been inspired from golang/image/math/f32

package f32

import "github.com/aurelien-rainone/math32"

// Clamp takes in a value and two thresholds. If the value is smaller than the
// low threshold, it returns the low threshold. If it's bigger than the high
// threshold it returns the high threshold. Otherwise it returns the value.
//
// Useful to prevent some functions from freaking out because a value was
// technically out of range.
func Clamp(a, low, high float32) float32 {
	if a < low {
		return low
	} else if a > high {
		return high
	}

	return a
}

// ClampFunc generates a closure that returns its parameter
// clamped to the range [low,high].
func ClampFunc(low, high float32) func(float32) float32 {
	return func(a float32) float32 {
		return Clamp(a, low, high)
	}
}

// IsClamped uses strict equality (meaning: not the Approx function) there
// shouldn't be any major issues with this since clamp is often used to fix
// minor errors.
//
// Checks if a is clamped between low and high as if
// Clamp(a, low, high) had been called.
// In most cases it's probably better to just call Clamp
// without checking this since it's relatively cheap.
func IsClamped(a, low, high float32) bool {
	return a >= low && a <= high
}

// SetMin checks if a > b, then a will be set to the value of b.
func SetMin(a *float32, b float32) {
	if b < *a {
		*a = b
	}
}

// SetMax checks if a < b, then a will be set to the value of b.
func SetMax(a *float32, b float32) {
	if *a < b {
		*a = b
	}
}

// Round shortens a float32 value to a specified precision (number of digits
// after the decimal point) with "round half up" tie-breaking rule. Half-way
// values (23.5) are always rounded up (24).
func Round(v float32, prec int) float32 {
	p := float32(prec)
	t := v * math32.Pow(10, p)
	if t > 0 {
		return math32.Floor(t+0.5) / math32.Pow(10, p)
	}
	return math32.Ceil(t-0.5) / math32.Pow(10, p)
}
