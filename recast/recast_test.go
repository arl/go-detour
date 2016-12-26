package recast

import (
	"fmt"
	"testing"

	"github.com/aurelien-rainone/math32"
)

func TestiMin(t *testing.T) {
	ttable := []struct {
		a, b, res int32
	}{
		{1, 2, 1},
		{2, 1, 1},
		{1, 1, 1},
	}

	for _, tt := range ttable {
		got := iMin(tt.a, tt.b)
		if got != tt.res {
			t.Fatalf("iMin(%v, %v) = %v, want %v", tt.a, tt.b, got, tt.res)
		}
	}
}

func TestiMax(t *testing.T) {
	ttable := []struct {
		a, b, res int32
	}{
		{1, 2, 2},
		{2, 1, 2},
		{1, 1, 2},
	}

	for _, tt := range ttable {
		got := iMax(tt.a, tt.b)
		if got != tt.res {
			t.Fatalf("iMax(%v, %v) = %v, want %v", tt.a, tt.b, got, tt.res)
		}
	}
}

func TestiAbs(t *testing.T) {
	ttable := []struct {
		a, res int32
	}{
		{-1, 1},
		{1, 1},
		{0, 0},
	}

	for _, tt := range ttable {
		got := iAbs(tt.a)
		if got != tt.res {
			t.Fatalf("iAbs(%v) = %v, want %v", tt.a, got, tt.res)
		}
	}
}

func TestCalcGridSize(t *testing.T) {
	verts := []float32{
		1, 2, 3,
		0, 2, 6,
	}
	var bmin, bmax [3]float32
	CalcBounds(verts, 2, bmin[:], bmax[:])
	fmt.Println("calcBounds", bmin, bmax)

	cellSize := float32(1.5)

	w, h := CalcGridSize(bmin, bmax, cellSize)
	if w != 1 {
		t.Fatalf("width should be 1, got %v", w)
	}
	if h != 2 {
		t.Fatalf("height should be 2, got %v", h)
	}
}

func TestCreateHeightfield(t *testing.T) {
	verts := []float32{
		1, 2, 3,
		0, 2, 6,
	}
	var bmin, bmax [3]float32
	CalcBounds(verts, 2, bmin[:], bmax[:])

	cellSize := float32(1.5)
	cellHeight := float32(2)

	w, h := CalcGridSize(bmin, bmax, cellSize)

	var hf Heightfield
	result := hf.Create(nil, w, h, bmin[:], bmax[:], cellSize, cellHeight)

	if !result {
		t.Fatalf("result should be true")
	}

	if hf.Width != w {
		t.Fatalf("should have heightfield.width == width")
	}
	if hf.Height != h {
		t.Fatalf("should have heightfield.height == height")
	}

	for i := range bmin {
		if !math32.Approx(hf.BMin[i], bmin[i]) {
			t.Fatalf("hf.BMin[%d] should be approx bmin[%d], got %f and %f", i, i, hf.BMin[i], bmin[i])
		}
	}

	for i := range bmax {
		if !math32.Approx(hf.BMax[i], bmax[i]) {
			t.Fatalf("hf.BMax[%d] should be approx bmax[%d], got %f and %f", i, i, hf.BMax[i], bmax[i])
		}
	}

	if !math32.Approx(hf.Cs, cellSize) {
		t.Fatalf("hf.Cs should be approx cellSize")
	}
	if !math32.Approx(hf.Ch, cellHeight) {
		t.Fatalf("hf.Ch should be approx cellHeight")
	}

	if len(hf.Spans) == 0 {
		t.Fatalf("hf.Spans slice should not be empty")
	}
	if hf.Pools != nil {
		t.Fatalf("hf.Pools should be nil")
	}
	if hf.Freelist != nil {
		t.Fatalf("hf.Freelist should be nil")
	}
}

func TestMarkWalkableTriangles(t *testing.T) {
	var ctx *Context
	walkableSlopeAngle := float32(45)
	verts := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
	}
	nv := int32(3)
	walkable_tri := []int32{0, 1, 2}
	unwalkable_tri := []int32{0, 2, 1}
	nt := int32(1)
	areas := []uint8{RC_NULL_AREA}

	t.Run("One walkable triangle", func(t *testing.T) {
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas)
		if areas[0] != RC_WALKABLE_AREA {
			t.Fatalf("areas[0] should be RC_WALKABLE_AREA, got %v", areas[0])
		}
	})

	t.Run("One non-walkable triangle", func(t *testing.T) {
		areas[0] = RC_NULL_AREA
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas)
		if areas[0] != RC_NULL_AREA {
			t.Fatalf("areas[0] should be RC_NULL_AREA, got %v", areas[0])
		}
	})

	t.Run("Non-walkable triangle area id's are not modified", func(t *testing.T) {
		areas[0] = 42
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas)
		if areas[0] != 42 {
			t.Fatalf("areas[0] should be 42, got %v", areas[0])
		}
	})

	t.Run("Slopes equal to the max slope are considered unwalkable.", func(t *testing.T) {
		areas[0] = RC_NULL_AREA
		walkableSlopeAngle = 0
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas)
		if areas[0] != RC_NULL_AREA {
			t.Fatalf("areas[0] should be RC_NULL_AREA, got %v", areas[0])
		}
	})
}
