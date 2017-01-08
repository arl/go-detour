package detour

import "fmt"

// Status represents status flags.
type Status uint32

// High level status.
const (
	Failure    Status = 1 << 31 // Operation failed.
	Success           = 1 << 30 // Operation succeed.
	InProgress        = 1 << 29 // Operation still in progress.

	// Detail information for status.
	StatusDetailMask = 0x0ffffff
	WrongMagic       = 1 << 0 // Input data is not recognized.
	WrongVersion     = 1 << 1 // Input data is in wrong version.
	OutOfMemory      = 1 << 2 // Operation ran out of memory.
	InvalidParam     = 1 << 3 // An input parameter was invalid.
	BufferTooSmall   = 1 << 4 // Result buffer for the query was too small to store all results.
	OutOfNodes       = 1 << 5 // Query ran out of nodes during search.
	PartialResult    = 1 << 6 // Query did not reach the end location, returning best guess.
)

// Implementation of the error interface
func (s Status) Error() string {
	if s&Failure != 0 {
		switch s & StatusDetailMask {
		case WrongMagic:
			return "wrong magic number"
		case WrongVersion:
			return "wrong version number"
		case OutOfMemory:
			return "out of memory"
		case InvalidParam:
			return "invalid parameter"
		case OutOfNodes:
			return "out of nodes"
		case PartialResult:
			return "partial result"
		default:
			return fmt.Sprintf("unspecified error 0x%x", s)
		}
	}
	if s == InProgress {
		return "in progress"
	}
	return "success"
}

// StatusSucceed returns true if status is success.
func StatusSucceed(status Status) bool {
	return (status & Success) != 0
}

// StatusFailed returns true if status is failure.
func StatusFailed(status Status) bool {
	return (status & Failure) != 0
}

// StatusInProgress returns true if status is in progress.
func StatusInProgress(status Status) bool {
	return (status & InProgress) != 0
}

// StatusDetail returns true if specific detail is set.
func StatusDetail(status Status, detail uint32) bool {
	return (uint32(status) & detail) != 0
}
