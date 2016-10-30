package detour

/// A navigation mesh based on tiles of convex polygons.
type DtNavMesh struct {
	/// @{
	/// @name Initialization and Tile Management

	/// Initializes the navigation mesh for tiled use.
	///  @param[in]	params		Initialization parameters.
	/// @return The status flags for the operation.
	//dtStatus init(const dtNavMeshParams* params);

	/// Initializes the navigation mesh for single tile use.
	///  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
	///  @param[in]	dataSize	The data size of the new tile.
	///  @param[in]	flags		The tile flags. (See: #dtTileFlags)
	/// @return The status flags for the operation.
	///  @see dtCreateNavMeshData
	//dtStatus init(unsigned char* data, const int dataSize, const int flags);

	/// The navigation mesh initialization params.
	//const dtNavMeshParams* getParams() const;

	/// Adds a tile to the navigation mesh.
	///  @param[in]		data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
	///  @param[in]		dataSize	Data size of the new tile mesh.
	///  @param[in]		flags		Tile flags. (See: #dtTileFlags)
	///  @param[in]		lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	///  @param[out]	result		The tile reference. (If the tile was succesfully added.) [opt]
	/// @return The status flags for the operation.
	//dtStatus addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);

	/// Removes the specified tile from the navigation mesh.
	///  @param[in]		ref			The reference of the tile to remove.
	///  @param[out]	data		Data associated with deleted tile.
	///  @param[out]	dataSize	Size of the data associated with deleted tile.
	/// @return The status flags for the operation.
	//dtStatus removeTile(dtTileRef ref, unsigned char** data, int* dataSize);

	/// @}

	/// @{
	/// @name Query Functions

	/// Calculates the tile grid location for the specified world position.
	///  @param[in]	pos  The world position for the query. [(x, y, z)]
	///  @param[out]	tx		The tile's x-location. (x, y)
	///  @param[out]	ty		The tile's y-location. (x, y)
	//void calcTileLoc(const float* pos, int* tx, int* ty) const;

	/// Gets the tile at the specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile, or null if the tile does not exist.
	//const dtMeshTile* getTileAt(const int x, const int y, const int layer) const;

	/// Gets all tiles at the specified grid location. (All layers.)
	///  @param[in]		x			The tile's x-location. (x, y)
	///  @param[in]		y			The tile's y-location. (x, y)
	///  @param[out]	tiles		A pointer to an array of tiles that will hold the result.
	///  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
	/// @return The number of tiles returned in the tiles array.
	//int getTilesAt(const int x, const int y,
	//dtMeshTile const** tiles, const int maxTiles) const;

	/// Gets the tile reference for the tile at specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile reference of the tile, or 0 if there is none.
	//dtTileRef getTileRefAt(int x, int y, int layer) const;

	/// Gets the tile reference for the specified tile.
	///  @param[in]	tile	The tile.
	/// @return The tile reference of the tile.
	//dtTileRef getTileRef(const dtMeshTile* tile) const;

	/// Gets the tile for the specified tile reference.
	///  @param[in]	ref		The tile reference of the tile to retrieve.
	/// @return The tile for the specified reference, or null if the
	///		reference is invalid.
	//const dtMeshTile* getTileByRef(dtTileRef ref) const;

	/// The maximum number of tiles supported by the navigation mesh.
	/// @return The maximum number of tiles supported by the navigation mesh.
	//int getMaxTiles() const;

	/// Gets the tile at the specified index.
	///  @param[in]	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
	/// @return The tile at the specified index.
	//const dtMeshTile* getTile(int i) const;

	/// Gets the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		The reference for the a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// @return The status flags for the operation.
	//dtStatus getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Returns the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		A known valid reference for a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	//void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Checks the validity of a polygon reference.
	///  @param[in]	ref		The polygon reference to check.
	/// @return True if polygon reference is valid for the navigation mesh.
	//bool isValidPolyRef(dtPolyRef ref) const;

	/// Gets the polygon reference for the tile's base polygon.
	///  @param[in]	tile		The tile.
	/// @return The polygon reference for the base polygon in the specified tile.
	//dtPolyRef getPolyRefBase(const dtMeshTile* tile) const;

	/// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	///  @param[in]		prevRef		The reference of the polygon before the connection.
	///  @param[in]		polyRef		The reference of the off-mesh connection polygon.
	///  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
	///  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
	/// @return The status flags for the operation.
	//dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;

	/// Gets the specified off-mesh connection.
	///  @param[in]	ref		The polygon reference of the off-mesh connection.
	/// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	//const dtOffMeshConnection* getOffMeshConnectionByRef(dtPolyRef ref) const;

	/// @}

	/// @{
	/// @name State Management
	/// These functions do not effect #dtTileRef or #dtPolyRef's.

	/// Sets the user defined flags for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	flags	The new flags for the polygon.
	/// @return The status flags for the operation.
	//dtStatus setPolyFlags(dtPolyRef ref, unsigned short flags);

	/// Gets the user defined flags for the specified polygon.
	///  @param[in]		ref				The polygon reference.
	///  @param[out]	resultFlags		The polygon flags.
	/// @return The status flags for the operation.
	//dtStatus getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const;

	/// Sets the user defined area for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	/// @return The status flags for the operation.
	//dtStatus setPolyArea(dtPolyRef ref, unsigned char area);

	/// Gets the user defined area for the specified polygon.
	///  @param[in]		ref			The polygon reference.
	///  @param[out]	resultArea	The area id for the polygon.
	/// @return The status flags for the operation.
	//dtStatus getPolyArea(dtPolyRef ref, unsigned char* resultArea) const;

	/// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	///  @param[in]	tile	The tile.
	/// @return The size of the buffer required to store the state.
	//int getTileStateSize(const dtMeshTile* tile) const;

	/// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	///  @param[in]		tile			The tile.
	///  @param[out]	data			The buffer to store the tile's state in.
	///  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	/// @return The status flags for the operation.
	//dtStatus storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const;

	/// Restores the state of the tile.
	///  @param[in]	tile			The tile.
	///  @param[in]	data			The new state. (Obtained from #storeTileState.)
	///  @param[in]	maxDataSize		The size of the state within the data buffer.
	/// @return The status flags for the operation.
	//dtStatus restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize);

	/// @}

	/// @{
	/// @name Encoding and Decoding
	/// These functions are generally meant for internal use only.

	/// Derives a standard polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	salt	The tile's salt value.
	///  @param[in]	it		The index of the tile.
	///  @param[in]	ip		The index of the polygon within the tile.
	//inline dtPolyRef encodePolyId(unsigned int salt, unsigned int it, unsigned int ip) const
	//{
	//#ifdef DT_POLYREF64
	//return ((dtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
	//#else
	//return ((dtPolyRef)salt << (m_polyBits+m_tileBits)) | ((dtPolyRef)it << m_polyBits) | (dtPolyRef)ip;
	//#endif
	//}

	/// Decodes a standard polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref   The polygon reference to decode.
	///  @param[out]	salt	The tile's salt value.
	///  @param[out]	it		The index of the tile.
	///  @param[out]	ip		The index of the polygon within the tile.
	///  @see #encodePolyId
	//inline void decodePolyId(dtPolyRef ref, unsigned int& salt, unsigned int& it, unsigned int& ip) const
	//{
	//#ifdef DT_POLYREF64
	//const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
	//const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
	//const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
	//salt = (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//it = (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//ip = (unsigned int)(ref & polyMask);
	//#else
	//const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
	//const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
	//const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
	//salt = (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
	//it = (unsigned int)((ref >> m_polyBits) & tileMask);
	//ip = (unsigned int)(ref & polyMask);
	//#endif
	//}

	/// Extracts a tile's salt value from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	//inline unsigned int decodePolyIdSalt(dtPolyRef ref) const
	//{
	//#ifdef DT_POLYREF64
	//const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
	//return (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//#else
	//const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
	//return (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
	//#endif
	//}

	/// Extracts the tile's index from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	//inline unsigned int decodePolyIdTile(dtPolyRef ref) const
	//{
	//#ifdef DT_POLYREF64
	//const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
	//return (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//#else
	//const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
	//return (unsigned int)((ref >> m_polyBits) & tileMask);
	//#endif
	//}

	/// Extracts the polygon's index (within its tile) from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	//inline unsigned int decodePolyIdPoly(dtPolyRef ref) const
	//{
	//#ifdef DT_POLYREF64
	//const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
	//return (unsigned int)(ref & polyMask);
	//#else
	//const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
	//return (unsigned int)(ref & polyMask);
	//#endif
	//}

	/// @}

	//private:
	// Explicitly disabled copy constructor and copy assignment operator.

	/// Returns pointer to tile in the tile array.
	//dtMeshTile* getTile(int i);

	/// Returns neighbour tile based on side.
	//int getTilesAt(const int x, const int y,
	//dtMeshTile** tiles, const int maxTiles) const;

	/// Returns neighbour tile based on side.
	//int getNeighbourTilesAt(const int x, const int y, const int side,
	//dtMeshTile** tiles, const int maxTiles) const;

	/// Returns all polygons in neighbour tile based on portal defined by the segment.
	//int findConnectingPolys(const float* va, const float* vb,
	//const dtMeshTile* tile, int side,
	//dtPolyRef* con, float* conarea, int maxcon) const;

	/// Builds internal polygons links for a tile.
	//void connectIntLinks(dtMeshTile* tile);
	/// Builds internal polygons links for a tile.
	//void baseOffMeshLinks(dtMeshTile* tile);

	/// Builds external polygon links for a tile.
	//void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	/// Builds external polygon links for a tile.
	//void connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side);

	/// Removes external links at specified side.
	//void unconnectLinks(dtMeshTile* tile, dtMeshTile* target);

	// TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.

	/// Queries polygons within a tile.
	//int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
	//dtPolyRef* polys, const int maxPolys) const;
	/// Find nearest polygon within a tile.
	//dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center,
	//const float* extents, float* nearestPt) const;
	/// Returns closest point on polygon.
	//void closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;

	//dtNavMeshParams m_params;			///< Current initialization params. TODO: do not store this info twice.
	m_orig                    [3]float32 ///< Origin of the tile (0,0)
	m_tileWidth, m_tileHeight float32    ///< Dimensions of each tile.
	m_maxTiles                int        ///< Max number of tiles.
	m_tileLutSize             int        ///< Tile hash lookup size (must be pot).
	m_tileLutMask             int        ///< Tile hash lookup mask.

	m_posLookup **dtMeshTile ///< Tile hash lookup.
	m_nextFree  *dtMeshTile  ///< Freelist of tiles.
	m_tiles     *dtMeshTile  ///< List of tiles.

	//#ifndef DT_POLYREF64
	m_saltBits uint ///< Number of salt bits in the tile ID.
	m_tileBits uint ///< Number of tile bits in the tile ID.
	m_polyBits uint ///< Number of poly bits in the tile ID.
	//#endif
}

const (
	DT_SALT_BITS uint = 16
	DT_TILE_BITS uint = 28
	DT_POLY_BITS uint = 20
)

type dtPolyRef uint64

/// Defines a link between polygons.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
type dtLink struct {
	ref  dtPolyRef ///< Neighbour reference. (The neighbor that is linked to.)
	next uint      ///< Index of the next link.
	edge uint8     ///< Index of the polygon edge that owns this link.
	side uint8     ///< If a boundary link, defines on which side the link is.
	bmin uint8     ///< If a boundary link, defines the minimum sub-edge area.
	bmax uint8     ///< If a boundary link, defines the maximum sub-edge area.
}

/// Defines the location of detail sub-mesh data within a dtMeshTile.
type dtPolyDetail struct {
	vertBase  uint  ///< The offset of the vertices in the dtMeshTile::detailVerts array.
	triBase   uint  ///< The offset of the triangles in the dtMeshTile::detailTris array.
	vertCount uint8 ///< The number of vertices in the sub-mesh.
	triCount  uint8 ///< The number of triangles in the sub-mesh.
}

/// Bounding volume node.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
type dtBVNode struct {
	bmin [3]uint16 ///< Minimum bounds of the node's AABB. [(x, y, z)]
	bmax [3]uint16 ///< Maximum bounds of the node's AABB. [(x, y, z)]
	i    int       ///< The node's index. (Negative for escape sequence.)
}

/// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
/// An off-mesh connection is a user defined traversable connection made up to two vertices.
type dtOffMeshConnection struct {
	/// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	pos [6]float32

	/// The radius of the endpoints. [Limit: >= 0]
	rad float32

	/// The polygon reference of the connection within the tile.
	poly uint16

	/// Link flags.
	/// @note These are not the connection's user defined flags. Those are assigned via the
	/// connection's dtPoly definition. These are link flags used for internal purposes.
	flags uint8

	/// End point side.
	side uint8

	/// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	userId uint
}
