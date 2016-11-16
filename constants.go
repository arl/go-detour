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

// Vertex flags returned by DtNavMeshQuery.findStraightPath.
const (
	DT_STRAIGHTPATH_START              uint8 = 0x01 // The vertex is the start position in the path.
	DT_STRAIGHTPATH_END                uint8 = 0x02 // The vertex is the end position in the path.
	DT_STRAIGHTPATH_OFFMESH_CONNECTION uint8 = 0x04 // The vertex is the start of an off-mesh connection.
)

// Options for DtNavMeshQuery.findStraightPath.
const (
	DT_STRAIGHTPATH_AREA_CROSSINGS uint8 = 0x01 // Add a vertex at every polygon edge crossing where area changes.
	DT_STRAIGHTPATH_ALL_CROSSINGS  uint8 = 0x02 // Add a vertex at every polygon edge crossing.
)
