package detour

const (
	NAVMESHSET_MAGIC   int32 = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T' //'MSET';
	NAVMESHSET_VERSION int32 = 1
)

const (
	/// The maximum number of vertices per navigation polygon.
	DT_VERTS_PER_POLYGON uint32 = 6

	/// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
	DT_OFFMESH_CON_BIDIR uint32 = 1

	/// The maximum number of user defined area ids.
	DT_MAX_AREAS int32 = 64
)
