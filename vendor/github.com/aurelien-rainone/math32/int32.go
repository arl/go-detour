package math32

// NextPow2 returns the next power of 2 of v, or v if v is a power of 2.
func NextPow2(v uint32) uint32 {
	v--
	v |= v >> 1
	v |= v >> 2
	v |= v >> 4
	v |= v >> 8
	v |= v >> 16
	v++
	return v
}

// MinInt32 returns the minimum of two int32 values.
func MinInt32(a, b int32) int32 {
	if a < b {
		return a
	}
	return b
}

// Ilog2 returns the uint32 value that is log2(v).
func Ilog2(v uint32) uint32 {
	boolToUInt32 := func(b bool) uint32 {
		if b {
			return 1
		}
		return 0
	}

	var r, shift uint32
	r = boolToUInt32(v > 0xffff) << 4
	v >>= r
	shift = boolToUInt32(v > 0xff) << 3
	v >>= shift
	r |= shift
	shift = boolToUInt32(v > 0xf) << 2
	v >>= shift
	r |= shift
	shift = boolToUInt32(v > 0x3) << 1
	v >>= shift
	r |= shift
	r |= (v >> 1)
	return r
}
