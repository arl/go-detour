package detour

type DtStatus uint32

// High level status.
const (
	DT_FAILURE     DtStatus = 1 << 31 // Operation failed.
	DT_SUCCESS              = 1 << 30 // Operation succeed.
	DT_IN_PROGRESS          = 1 << 29 // Operation still in progress.

	// Detail information for status.
	DT_STATUS_DETAIL_MASK = 0x0ffffff
	DT_WRONG_MAGIC        = 1 << 0 // Input data is not recognized.
	DT_WRONG_VERSION      = 1 << 1 // Input data is in wrong version.
	DT_OUT_OF_MEMORY      = 1 << 2 // Operation ran out of memory.
	DT_INVALID_PARAM      = 1 << 3 // An input parameter was invalid.
	DT_BUFFER_TOO_SMALL   = 1 << 4 // Result buffer for the query was too small to store all results.
	DT_OUT_OF_NODES       = 1 << 5 // Query ran out of nodes during search.
	DT_PARTIAL_RESULT     = 1 << 6 // Query did not reach the end location, returning best guess.
)

// Returns true of status is success.
func DtStatusSucceed(status DtStatus) bool {
	return (status & DT_SUCCESS) != 0
}

// Returns true of status is failure.
func DtStatusFailed(status DtStatus) bool {
	return (status & DT_FAILURE) != 0
}

// Returns true of status is in progress.
func DtStatusInProgress(status DtStatus) bool {
	return (status & DT_IN_PROGRESS) != 0
}

// Returns true if specific detail is set.
func DtStatusDetail(status DtStatus, detail uint32) bool {
	return (uint32(status) & detail) != 0
}
