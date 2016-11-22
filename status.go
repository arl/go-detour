package detour

import "fmt"

// DtStatus represents status flags.
type DtStatus uint32

// High level status.
const (
	DtFailure    DtStatus = 1 << 31 // Operation failed.
	DtSuccess             = 1 << 30 // Operation succeed.
	DtInProgress          = 1 << 29 // Operation still in progress.

	// Detail information for status.
	DtStatusDetailMask = 0x0ffffff
	DtWrongMagic       = 1 << 0 // Input data is not recognized.
	DtWrongVersion     = 1 << 1 // Input data is in wrong version.
	DtOutOfMemory      = 1 << 2 // Operation ran out of memory.
	DtInvalidParam     = 1 << 3 // An input parameter was invalid.
	DtBufferTooSmall   = 1 << 4 // Result buffer for the query was too small to store all results.
	DtOutOfNodes       = 1 << 5 // Query ran out of nodes during search.
	DtPartialResult    = 1 << 6 // Query did not reach the end location, returning best guess.
)

// Implementation of the error interface
func (s DtStatus) Error() string {
	if s == DtFailure {
		switch s & DtStatusDetailMask {
		case DtWrongMagic:
			return "wrong magic number"
		case DtWrongVersion:
			return "wrong version number"
		case DtOutOfMemory:
			return "out of memory"
		case DtInvalidParam:
			return "invalid parameter"
		case DtOutOfNodes:
			return "out of nodes"
		case DtPartialResult:
			return "partial result"
		default:
			return fmt.Sprintf("unspecified error 0x%x", s)
		}
	}
	if s == DtInProgress {
		return "in progress"
	}
	return "success"
}

// DtStatusSucceed returns true if status is success.
func DtStatusSucceed(status DtStatus) bool {
	return (status & DtSuccess) != 0
}

// DtStatusFailed returns true if status is failure.
func DtStatusFailed(status DtStatus) bool {
	return (status & DtFailure) != 0
}

// DtStatusInProgress returns true if status is in progress.
func DtStatusInProgress(status DtStatus) bool {
	return (status & DtInProgress) != 0
}

// DtStatusDetail returns true if specific detail is set.
func DtStatusDetail(status DtStatus, detail uint32) bool {
	return (uint32(status) & detail) != 0
}
