package detour

const (
	navMeshSetMagic   = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T' //'MSET';
	navMeshSetVersion = 1

	// VertsPerPolygon is the maximum number of vertices per navigation polygon.
	VertsPerPolygon = 6

	// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
	offMeshConBidir = 1

	// The maximum number of user defined area ids.
	maxAreas = 64
)
