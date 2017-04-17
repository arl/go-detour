package detour

import (
	"log"

	"github.com/aurelien-rainone/gogeo/f32/d3"
	"github.com/aurelien-rainone/math32"
)

// Provides custom polygon query behavior.
// Used by NavMeshQuery.queryPolygons.
type polyQuery interface {

	// Called for each batch of unique polygons touched by the search area in
	// NavMeshQuery::queryPolygons. This can be called multiple times for a
	// single query.
	process(tile *MeshTile, polys []*Poly, refs []PolyRef, count int32)
}

type findNearestPolyQuery struct {
	query              *NavMeshQuery
	center             d3.Vec3
	nearestDistanceSqr float32
	nearestRef         PolyRef
	nearestPoint       d3.Vec3
}

func newFindNearestPolyQuery(query *NavMeshQuery, center d3.Vec3) *findNearestPolyQuery {
	return &findNearestPolyQuery{
		query:              query,
		center:             center,
		nearestDistanceSqr: math32.MaxFloat32,
		nearestRef:         0,
		nearestPoint:       d3.NewVec3(),
	}
}

func (q *findNearestPolyQuery) process(tile *MeshTile, polys []*Poly, refs []PolyRef, count int32) {

	for i := int32(0); i < count; i++ {
		ref := refs[i]
		var (
			closestPtPoly d3.Vec3
			d             float32
		)
		posOverPoly := false
		closestPtPoly = d3.NewVec3()
		q.query.ClosestPointOnPoly(ref, q.center, closestPtPoly, &posOverPoly)

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		diff := q.center.Sub(closestPtPoly)
		if posOverPoly {
			d = math32.Abs(diff[1]) - tile.Header.WalkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = diff.LenSqr()
		}

		if d < q.nearestDistanceSqr {
			q.nearestPoint.Assign(closestPtPoly)

			q.nearestDistanceSqr = d
			q.nearestRef = ref
		}
	}
}

type collectPolysQuery struct {
	polys        []PolyRef
	maxPolys     int32
	numCollected int32
	overflow     bool
}

func newCollectPolysQuery(polys []PolyRef, maxPolys int32) *collectPolysQuery {

	return &collectPolysQuery{
		polys:        polys,
		maxPolys:     maxPolys,
		numCollected: 0,
		overflow:     false,
	}
}

func (q *collectPolysQuery) process(tile *MeshTile, polys []*Poly, refs []PolyRef, count int32) {

	numLeft := q.maxPolys - q.numCollected
	toCopy := count
	if toCopy > numLeft {
		q.overflow = true
		toCopy = numLeft
	}

	log.Fatal("untested")
	copy(q.polys[q.numCollected:], refs[0:toCopy])
	q.numCollected += toCopy
}
