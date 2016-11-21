package detour

const (
	navMeshSetMagic   int32 = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T' //'MSET';
	navMeshSetVersion int32 = 1
)

const (
	// The maximum number of vertices per navigation polygon.
	dtVertsPerPolygon uint32 = 6

	// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
	dtOffMeshConBidir uint32 = 1

	// The maximum number of user defined area ids.
	dtMaxAreas int32 = 64
)
