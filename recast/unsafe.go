package recast

import "unsafe"

func compareSlicesInt32(s1, s2 []int32) bool {
	var add1, add2 uintptr
	add1 = uintptr(unsafe.Pointer(&s1[0]))
	add2 = uintptr(unsafe.Pointer(&s2[0]))
	return add1 == add2
}

func compareSlicesUInt16(s1, s2 []uint16) bool {
	var add1, add2 uintptr
	add1 = uintptr(unsafe.Pointer(&s1[0]))
	add2 = uintptr(unsafe.Pointer(&s2[0]))
	return add1 == add2
}
