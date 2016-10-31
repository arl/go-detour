package detour

func dtNextPow2(v uint32) uint32 {
	v--
	v |= v >> 1
	v |= v >> 2
	v |= v >> 4
	v |= v >> 8
	v |= v >> 16
	v++
	return v
}

func dtIlog2(v uint32) uint32 {

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

func dtAlign4(x uint32) uint32 {
	//return (x+3) & ~3;
	return ((x + 3) &^ 3)
}
