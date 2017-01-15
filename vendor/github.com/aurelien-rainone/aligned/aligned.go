package aligned

// AlignN returns the number of byte chunks that are required to align a
// variable of size s (in bytes) on a machine where the word-size is a.
//
// It panics if s is negative or a is lower or equal than 0.
func AlignN(s, a int) int {
	if s < 0 {
		panic("AlignN() s must be positive")
	}
	if a <= 0 {
		panic("AlignN() a must be strictly positive")
	}
	r := s % a
	if r == 0 {
		return s
	}
	return s + (a - r)
}
