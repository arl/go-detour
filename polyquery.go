package detour

import "github.com/aurelien-rainone/math32"

/// Provides custom polygon query behavior.
/// Used by dtNavMeshQuery::queryPolygons.
/// @ingroup detour
type DtPolyQuery interface {

	// Called for each batch of unique polygons touched by the search area in
	// dtNavMeshQuery::queryPolygons. This can be called multiple times for a
	// single query.
	process(tile *DtMeshTile, polys []*DtPoly, refs []DtPolyRef, count int32)
}

type dtFindNearestPolyQuery struct {
	query              *DtNavMeshQuery
	center             []float32
	nearestDistanceSqr float32
	nearestRef         DtPolyRef
	nearestPoint       [3]float32
}

func newDtFindNearestPolyQuery(query *DtNavMeshQuery, center []float32) *dtFindNearestPolyQuery {
	return &dtFindNearestPolyQuery{
		query:              query,
		center:             center,
		nearestDistanceSqr: math32.MaxFloat32,
		nearestRef:         0,
	}
}

func (q *dtFindNearestPolyQuery) NearestRef() DtPolyRef {
	return q.nearestRef
}

func (q *dtFindNearestPolyQuery) NearestPoint() [3]float32 {
	return q.nearestPoint
}

func (q *dtFindNearestPolyQuery) process(tile *DtMeshTile, polys []*DtPoly, refs []DtPolyRef, count int32) {

	for i := int32(0); i < count; i++ {
		ref := refs[i]
		var (
			closestPtPoly [3]float32
			diff          [3]float32
			d             float32
		)
		posOverPoly := false
		q.query.closestPointOnPoly(ref, q.center, closestPtPoly[:], &posOverPoly)

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff[:], q.center, closestPtPoly[:])
		if posOverPoly {
			d = math32.Abs(diff[1]) - tile.Header.WalkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = dtVlenSqr(diff[:])
		}

		if d < q.nearestDistanceSqr {
			dtVcopy(q.nearestPoint[:], closestPtPoly[:])

			q.nearestDistanceSqr = d
			q.nearestRef = ref
		}
	}
}

type dtCollectPolysQuery struct {
	polys        []DtPolyRef
	maxPolys     int32
	numCollected int32
	overflow     bool
}

func newDtCollectPolysQuery(polys []DtPolyRef, maxPolys int32) *dtCollectPolysQuery {

	return &dtCollectPolysQuery{
		polys:        polys,
		maxPolys:     maxPolys,
		numCollected: 0,
		overflow:     false,
	}
}

func (q *dtCollectPolysQuery) process(tile *DtMeshTile, polys []*DtPoly, refs []DtPolyRef, count int32) {

	numLeft := q.maxPolys - q.numCollected
	toCopy := count
	if toCopy > numLeft {
		q.overflow = true
		toCopy = numLeft
	}

	//memcpy(m_polys + m_numCollected, refs, (size_t)toCopy * sizeof(DtPolyRef));
	//TODO: check that...
	copy(q.polys[q.numCollected:], refs[0:toCopy])
	q.numCollected += toCopy
}
