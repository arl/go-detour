// +build !debug

package assert

// True ensures cond is true, by panicking if it is false.
//
// The behaviour displayed by True is enabled only if the 'debug' build tags
// has been provided to the `go` tool during compilation, in any other case
// True is a noop (i.e an empty function).
func True(cond bool, format string, args ...interface{}) {}

// False ensures cond is false, by panicking if it is true.
//
// The behaviour displayed by False is enabled only if the 'debug' build tags
// has been provided to the `go` tool during compilation, in any other case
// False is a noop (i.e an empty function).
func False(cond bool, format string, args ...interface{}) {}
