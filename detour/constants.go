package detour

const (
	navMeshSetMagic   int32 = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T' //'MSET';
	navMeshSetVersion int32 = 1
)

const (
	// VertsPerPolygon is the maximum number of vertices per navigation polygon.
	VertsPerPolygon uint32 = 6

	// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
	offMeshConBidir uint32 = 1

	// The maximum number of user defined area ids.
	maxAreas int32 = 64
)
