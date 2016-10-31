package detour

type dtTileRef uint32

type NavMeshTileHeader struct {
	TileRef  dtTileRef
	DataSize int32
}
