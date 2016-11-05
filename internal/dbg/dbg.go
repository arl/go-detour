package main

import (
	"fmt"
	"log"
	"os"

	"github.com/aurelien-rainone/assertgo"
	detour "github.com/aurelien-rainone/go-detour"
)

func check(err error) {
	if err != nil {
		log.Fatalln(err)
		os.Exit(1)
	}
}

func main() {
	var (
		f    *os.File
		err  error
		mesh *detour.DtNavMesh
		//out  *os.File
	)

	f, err = os.Open("navmesh.bin")
	check(err)
	defer f.Close()

	mesh, err = detour.Decode(f)
	check(err)
	if mesh == nil {
		fmt.Println("error loading mesh")
		return
	}
	fmt.Println("mesh loaded successfully")
	fmt.Printf("mesh params: %#v\n", mesh.Params)

	//firstTile, mesh

	tile := mesh.Tiles[0]
	fmt.Println("tile header", tile.Header)
	fmt.Println("tile min AABB", tile.Header.Bmin, tile.Header.Bmax)
	fmt.Println("tile poly count", tile.Header.PolyCount)
	fmt.Println("tile vert count", tile.Header.VertCount)

	//out, err = os.Create("out")
	//check(err)
	//defer out.Close()
	//out.WriteString(fmt.Sprintf("tile_0\n"))

	for pidx, poly := range tile.Polys[0:tile.Header.PolyCount] {
		fmt.Printf("poly_%d [%d vertices]:\n", pidx, poly.VertCount)

		var j uint8
		for j = 0; j < poly.VertCount; j++ {
			start := poly.Verts[j] * 3
			fmt.Println("vertices", tile.Verts[start:start+3])
		}
		centroid := make([]float32, 3)
		detour.DtCalcPolyCenter(centroid, poly.Verts[:], int32(poly.VertCount), tile.Verts)
		fmt.Println("centroid", centroid)
	}

	fmt.Println("Navigation Query")
	q, st := detour.NewDtNavMeshQuery(mesh, 10)
	if detour.DtStatusFailed(st) {
		log.Fatalf("NewDtNavMeshQuery failed with status 0x%x\n", st)
	}
	fmt.Println("q", q)
	//fmt.Println("tile 17", mesh.Tiles[17])
	//fmt.Println("tile 18", mesh.Tiles[18])

	// Origin: get PolyRef from point
	org := []float32{40, 1, 20}
	extents := []float32{0, 2, 0}
	var (
		orgPolyRef detour.DtPolyRef
		nearestPt  [3]float32
	)
	filter := detour.NewDtQueryFilter()

	// Origin: FindNearestPoly
	status := q.FindNearestPoly(org, extents, filter, &orgPolyRef, nearestPt[:])
	if detour.DtStatusFailed(status) {
		fmt.Printf("FindNearestPoly failed with 0x%x\n", status)
	}
	fmt.Println("ref:", orgPolyRef, "nearestPt:", nearestPt)
	org = nearestPt[:]
	assert.True(mesh.IsValidPolyRef(orgPolyRef), "%d is not a valid poly ref")

	// TileAndPolyByRef
	var (
		ptile *detour.DtMeshTile
		ppoly *detour.DtPoly
	)
	status = mesh.TileAndPolyByRef(orgPolyRef, &ptile, &ppoly)
	if detour.DtStatusFailed(status) {
		fmt.Printf("TileAndPolyByRef failed with 0x%x\n", status)
	}
	fmt.Print("got tile ", ptile.Header.X, ", ", ptile.Header.Y)
	fmt.Print("got poly ", *ppoly)

	// Destination: get PolyRef from point
	dst := []float32{4, 1, 4}

	var dstPolyRef detour.DtPolyRef

	// Destination: FindNearestPoly
	status = q.FindNearestPoly(dst, extents, filter, &dstPolyRef, nearestPt[:])
	if detour.DtStatusFailed(status) {
		fmt.Printf("FindNearestPoly failed with 0x%x\n", status)
	}
	fmt.Println("ref:", dstPolyRef, "nearestPt:", nearestPt)
	dst = nearestPt[:]
	assert.True(mesh.IsValidPolyRef(dstPolyRef), "%d is not a valid poly ref")

	//
	mesh.TileAndPolyByRefUnsafe(dstPolyRef, &ptile, &ppoly)
	fmt.Println("TileAndPolyByRefUnsafe")
	fmt.Println(*ptile)
	fmt.Println(*ppoly)

	var (
		path      []detour.DtPolyRef
		pathCount int32
	)
	path = make([]detour.DtPolyRef, 100)
	status = q.FindPath(orgPolyRef, dstPolyRef, org, dst, filter, &path, &pathCount, 100)
	if detour.DtStatusFailed(status) {
		fmt.Printf("FindPath failed with 0x%x\n", status)
	}

	fmt.Println("FindPath set pathCount to", pathCount)
	fmt.Println("path", path)
}
