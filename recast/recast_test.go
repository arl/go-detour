package recast

import (
	"testing"

	"github.com/arl/math32"
)

func require(t *testing.T, exp bool, err string) {
	if !exp {
		t.Fatalf(err)
	}
}

func TestIMin(t *testing.T) {
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

func TestIMax(t *testing.T) {
	ttable := []struct {
		a, b, res int32
	}{
		{1, 2, 2},
		{2, 1, 2},
		{1, 1, 1},
	}

	for _, tt := range ttable {
		got := iMax(tt.a, tt.b)
		if got != tt.res {
			t.Fatalf("iMax(%v, %v) = %v, want %v", tt.a, tt.b, got, tt.res)
		}
	}
}

func TestIAbs(t *testing.T) {
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

func TestCalcBounds(t *testing.T) {
	ttable := []struct {
		verts            []float32
		wantmin, wantmax []float32
	}{
		// bounds of one vector
		{
			[]float32{1, 2, 3},
			[]float32{1, 2, 3},
			[]float32{1, 2, 3},
		},
		// bounds of more than one vector
		{
			[]float32{0, 2, 3, 1, 2, 5},
			[]float32{0, 2, 3},
			[]float32{1, 2, 5},
		},
	}

	for _, tt := range ttable {
		var bmin, bmax [3]float32
		CalcBounds(tt.verts, int32(len(tt.verts)/3), bmin[:], bmax[:])

		for i := 0; i < 3; i++ {
			if bmin[i] != tt.wantmin[i] {
				t.Errorf("for verts=%v, bmin[%d] == %v, want %v", tt.verts, i, bmin[i], tt.wantmin[i])
			}
			if bmax[i] != tt.wantmax[i] {
				t.Errorf("for verts=%v, bmax[%d] == %v, want %v", tt.verts, i, bmax[i], tt.wantmax[i])
			}
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

	cellSize := float32(1.5)

	w, h := CalcGridSize(bmin[:], bmax[:], cellSize)
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

	w, h := CalcGridSize(bmin[:], bmax[:], cellSize)
	hf := NewHeightfield(w, h, bmin[:], bmax[:], cellSize, cellHeight)

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
	var ctx *BuildContext
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
	areas := []uint8{nullArea}

	t.Run("One walkable triangle", func(t *testing.T) {
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas)
		if areas[0] != WalkableArea {
			t.Fatalf("areas[0] should be RC_WALKABLE_AREA, got %v", areas[0])
		}
	})

	t.Run("One non-walkable triangle", func(t *testing.T) {
		areas[0] = nullArea
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas)
		if areas[0] != nullArea {
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
		areas[0] = nullArea
		walkableSlopeAngle = 0
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas)
		if areas[0] != nullArea {
			t.Fatalf("areas[0] should be RC_NULL_AREA, got %v", areas[0])
		}
	})
}

func TestClearUnwalkableTriangles(t *testing.T) {
	var ctx *BuildContext
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

	t.Run("Sets area ID of unwalkable triangle to RC_NULL_AREA", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas)

		if areas[0] != nullArea {
			t.Fatalf("areas[0] should be RC_NULL_AREA, got %v", areas[0])
		}
	})
	t.Run("Does not modify walkable triangle aread ID's", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas)

		if areas[0] != 42 {
			t.Fatalf("areas[0] should be 42, got %v", areas[0])
		}
	})
	t.Run("Slopes equal to the max slope are considered unwalkable.", func(t *testing.T) {
		areas := []uint8{42}
		walkableSlopeAngle = 0
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas)

		if areas[0] != nullArea {
			t.Fatalf("areas[0] should be RC_NULL_AREA, got %v", areas[0])
		}
	})
}

func TestAddSpan(t *testing.T) {
	verts := []float32{
		1, 2, 3,
		0, 2, 6,
	}
	var bmin, bmax [3]float32
	CalcBounds(verts, 2, bmin[:], bmax[:])

	cellSize := float32(1.5)
	cellHeight := float32(2)

	w, h := CalcGridSize(bmin[:], bmax[:], cellSize)

	var (
		hf           *Heightfield
		x, y         int32
		smin, smax   uint16
		area         uint8
		flagMergeThr int32
	)

	testSetup := func() {
		hf = NewHeightfield(w, h, bmin[:], bmax[:], cellSize, cellHeight)
		x, y = 0, 0
		smin, smax = 0, 1
		area = 42
		flagMergeThr = 1
	}

	t.Run("Add a span to an empty heightfield.", func(t *testing.T) {
		testSetup()
		res := hf.addSpan(x, y, smin, smax, area, flagMergeThr)
		if !res {
			t.Fatalf("result should be true")
		}

		if hf.Spans[0] == nil {
			t.Fatalf("want hf.Spans[0] != nil, got nil")
		}
		if hf.Spans[0].smin != smin {
			t.Fatalf("want hf.Spans[0].smin == smin, got %v", hf.Spans[0].smin)
		}
		if hf.Spans[0].smax != smax {
			t.Fatalf("want hf.Spans[0].smax == smax, got %v", hf.Spans[0].smax)
		}
		if hf.Spans[0].area != area {
			t.Fatalf("want hf.Spans[0].area == area, got %v", hf.Spans[0].area)
		}
	})

	t.Run("Add a span that gets merged with an existing span.", func(t *testing.T) {
		testSetup()
		res := hf.addSpan(x, y, smin, smax, area, flagMergeThr)
		if !res {
			t.Fatalf("result should be true")
		}
		if hf.Spans[0] == nil {
			t.Fatalf("want hf.Spans[0] != nil, got nil")
		}
		if hf.Spans[0].smin != smin {
			t.Fatalf("want hf.Spans[0].smin == smin, got %v", hf.Spans[0].smin)
		}
		if hf.Spans[0].smax != smax {
			t.Fatalf("want hf.Spans[0].smax == smax, got %v", hf.Spans[0].smax)
		}
		if hf.Spans[0].area != area {
			t.Fatalf("want hf.Spans[0].area == area, got %v", hf.Spans[0].area)
		}

		smin = 1
		smax = 2
		res = hf.addSpan(x, y, smin, smax, area, flagMergeThr)

		if !res {
			t.Fatalf("result should be true")
		}
		if hf.Spans[0] == nil {
			t.Fatalf("want hf.Spans[0] != nil, got nil")
		}
		if hf.Spans[0].smin != 0 {
			t.Fatalf("want hf.Spans[0].smin == 0, got %v", hf.Spans[0].smin)
		}
		if hf.Spans[0].smax != 2 {
			t.Fatalf("want hf.Spans[0].smax == 2, got %v", hf.Spans[0].smax)
		}
		if hf.Spans[0].area != area {
			t.Fatalf("want hf.Spans[0].area == area, got %v", hf.Spans[0].area)
		}
	})

	t.Run("Add a span that merges with two spans above and below.", func(t *testing.T) {
		testSetup()
		smin = 0
		smax = 1
		res := hf.addSpan(x, y, smin, smax, area, flagMergeThr)
		if !res {
			t.Fatalf("result should be true")
		}
		if hf.Spans[0] == nil {
			t.Fatalf("want hf.Spans[0] != nil, got nil")
		}
		if hf.Spans[0].smin != smin {
			t.Fatalf("want hf.Spans[0].smin == smin, got %v", hf.Spans[0].smin)
		}
		if hf.Spans[0].smax != smax {
			t.Fatalf("want hf.Spans[0].smax == smax, got %v", hf.Spans[0].smax)
		}
		if hf.Spans[0].area != area {
			t.Fatalf("want hf.Spans[0].area == area, got %v", hf.Spans[0].area)
		}
		if hf.Spans[0].next != nil {
			t.Fatalf("want hf.Spans[0].next == nil, got %v", hf.Spans[0].next)
		}

		smin = 2
		smax = 3
		res = hf.addSpan(x, y, smin, smax, area, flagMergeThr)
		if !res {
			t.Fatalf("result should be true")
		}
		if hf.Spans[0].next == nil {
			t.Fatalf("want hf.Spans[0].next != nil, got nil")
		}
		if hf.Spans[0].next.smin != smin {
			t.Fatalf("want hf.Spans[0].next.smin == smin, got %v", hf.Spans[0].next.smin)
		}
		if hf.Spans[0].next.smax != smax {
			t.Fatalf("want hf.Spans[0].next.smax == smax, got %v", hf.Spans[0].next.smax)
		}
		if hf.Spans[0].next.area != area {
			t.Fatalf("want hf.Spans[0].next.area == area, got %v", hf.Spans[0].next.area)
		}

		smin = 1
		smax = 2
		res = hf.addSpan(x, y, smin, smax, area, flagMergeThr)

		if !res {
			t.Fatalf("result should be true")
		}
		if hf.Spans[0] == nil {
			t.Fatalf("want hf.Spans[0] != nil, got nil")
		}
		if hf.Spans[0].smin != 0 {
			t.Fatalf("want hf.Spans[0].smin == 0, got %v", hf.Spans[0].smin)
		}
		if hf.Spans[0].smax != 3 {
			t.Fatalf("want hf.Spans[0].smax == 3, got %v", hf.Spans[0].smax)
		}
		if hf.Spans[0].area != area {
			t.Fatalf("want hf.Spans[0].area == area, got %v", hf.Spans[0].area)
		}
		if hf.Spans[0].next != nil {
			t.Fatalf("want hf.Spans[0].next == nil, got %v", hf.Spans[0].next)
		}
	})
}

func TestRasterizeTriangle(t *testing.T) {
	var ctx BuildContext
	verts := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
	}
	var bmin, bmax [3]float32
	CalcBounds(verts, 3, bmin[:], bmax[:])

	cellSize := float32(0.5)
	cellHeight := float32(0.5)

	w, h := CalcGridSize(bmin[:], bmax[:], cellSize)

	solid := NewHeightfield(w, h, bmin[:], bmax[:], cellSize, cellHeight)

	area := uint8(42)
	flagMergeThr := int32(1)

	RasterizeTriangle(&ctx, verts[0:3], verts[3:6], verts[6:9], area, solid, flagMergeThr)

	require(t, solid.Spans[0+0*w] != nil, "solid.Spans[0+0*w]")
	require(t, solid.Spans[1+0*w] == nil, "!solid.Spans[1+0*w]")
	require(t, solid.Spans[0+1*w] != nil, "solid.Spans[0+1*w]")
	require(t, solid.Spans[1+1*w] != nil, "solid.Spans[1+1*w]")

	require(t, solid.Spans[0+0*w].smin == 0, "solid.Spans[0+0*w].smin == 0")
	require(t, solid.Spans[0+0*w].smax == 1, "solid.Spans[0+0*w].smax == 1")
	require(t, solid.Spans[0+0*w].area == area, "solid.Spans[0+0*w].area == area")
	require(t, solid.Spans[0+0*w].next == nil, "!solid.Spans[0+0*w].next")

	require(t, solid.Spans[0+1*w].smin == 0, "solid.Spans[0+1*w].smin == 0")
	require(t, solid.Spans[0+1*w].smax == 1, "solid.Spans[0+1*w].smax == 1")
	require(t, solid.Spans[0+1*w].area == area, "solid.Spans[0+1*w].area == area")
	require(t, solid.Spans[0+1*w].next == nil, "!solid.Spans[0+1*w].next")

	require(t, solid.Spans[1+1*w].smin == 0, "solid.Spans[1+1*w].smin == 0")
	require(t, solid.Spans[1+1*w].smax == 1, "solid.Spans[1+1*w].smax == 1")
	require(t, solid.Spans[1+1*w].area == area, "solid.Spans[1+1*w].area == area")
	require(t, solid.Spans[1+1*w].next == nil, "!solid.Spans[1+1*w].next")
}

func TestRasterizeTriangles(t *testing.T) {
	var ctx BuildContext
	verts := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		0, 0, 1,
	}
	tris := []int32{
		0, 1, 2,
		0, 3, 1,
	}
	areas := []uint8{
		1,
		2,
	}
	var bmin, bmax [3]float32
	CalcBounds(verts, 4, bmin[:], bmax[:])

	cellSize := float32(0.5)
	cellHeight := float32(0.5)

	w, h := CalcGridSize(bmin[:], bmax[:], cellSize)

	var (
		solid        *Heightfield
		flagMergeThr int32
	)
	solid = NewHeightfield(w, h, bmin[:], bmax[:], cellSize, cellHeight)

	flagMergeThr = 1

	// SECTION("Rasterize some triangles")
	res := RasterizeTriangles(&ctx, verts, 4, tris, areas, 2, solid, flagMergeThr)
	if !res {
		t.Fatalf("result should be true")
	}

	require(t, solid.Spans[0+0*w] != nil, "solid.spans[0 + 0 * w]")
	require(t, solid.Spans[0+1*w] != nil, "solid.spans[0 + 1 * w]")
	require(t, solid.Spans[0+2*w] != nil, "solid.spans[0 + 2 * w]")
	require(t, solid.Spans[0+3*w] != nil, "solid.spans[0 + 3 * w]")
	require(t, solid.Spans[1+0*w] == nil, "!solid.spans[1 + 0 * w]")
	require(t, solid.Spans[1+1*w] != nil, "solid.spans[1 + 1 * w]")
	require(t, solid.Spans[1+2*w] != nil, "solid.spans[1 + 2 * w]")
	require(t, solid.Spans[1+3*w] == nil, "!solid.spans[1 + 3 * w]")

	require(t, solid.Spans[0+0*w].smin == 0, "solid.spans[0 + 0 * w].smin == 0")
	require(t, solid.Spans[0+0*w].smax == 1, "solid.spans[0 + 0 * w].smax == 1")
	require(t, solid.Spans[0+0*w].area == 1, "solid.spans[0 + 0 * w].area == 1")
	require(t, solid.Spans[0+0*w].next == nil, "!solid.spans[0 + 0 * w].next")

	require(t, solid.Spans[0+1*w].smin == 0, "solid.spans[0 + 1 * w].smin == 0")
	require(t, solid.Spans[0+1*w].smax == 1, "solid.spans[0 + 1 * w].smax == 1")
	require(t, solid.Spans[0+1*w].area == 1, "solid.spans[0 + 1 * w].area == 1")
	require(t, solid.Spans[0+1*w].next == nil, "!solid.spans[0 + 1 * w].next")

	require(t, solid.Spans[0+2*w].smin == 0, "solid.spans[0 + 2 * w].smin == 0")
	require(t, solid.Spans[0+2*w].smax == 1, "solid.spans[0 + 2 * w].smax == 1")
	require(t, solid.Spans[0+2*w].area == 2, "solid.spans[0 + 2 * w].area == 2")
	require(t, solid.Spans[0+2*w].next == nil, "!solid.spans[0 + 2 * w].next")

	require(t, solid.Spans[0+3*w].smin == 0, "solid.spans[0 + 3 * w].smin == 0")
	require(t, solid.Spans[0+3*w].smax == 1, "solid.spans[0 + 3 * w].smax == 1")
	require(t, solid.Spans[0+3*w].area == 2, "solid.spans[0 + 3 * w].area == 2")
	require(t, solid.Spans[0+3*w].next == nil, "!solid.spans[0 + 3 * w].next")

	require(t, solid.Spans[1+1*w].smin == 0, "solid.spans[1 + 1 * w].smin == 0")
	require(t, solid.Spans[1+1*w].smax == 1, "solid.spans[1 + 1 * w].smax == 1")
	require(t, solid.Spans[1+1*w].area == 1, "solid.spans[1 + 1 * w].area == 1")
	require(t, solid.Spans[1+1*w].next == nil, "!solid.spans[1 + 1 * w].next")

	require(t, solid.Spans[1+2*w].smin == 0, "solid.spans[1 + 2 * w].smin == 0")
	require(t, solid.Spans[1+2*w].smax == 1, "solid.spans[1 + 2 * w].smax == 1")
	require(t, solid.Spans[1+2*w].area == 2, "solid.spans[1 + 2 * w].area == 2")
	require(t, solid.Spans[1+2*w].next == nil, "!solid.spans[1 + 2 * w].next")

	//SECTION("Unsigned short overload")
	utris := []int32{
		0, 1, 2,
		0, 3, 1,
	}
	res = RasterizeTriangles(&ctx, verts, 4, utris, areas, 2, solid, flagMergeThr)
	if !res {
		t.Fatalf("result should be true")
	}

	require(t, solid.Spans[0+0*w] != nil, "solid.Spans[0 + 0 * w])")
	require(t, solid.Spans[0+1*w] != nil, "solid.Spans[0 + 1 * w])")
	require(t, solid.Spans[0+2*w] != nil, "solid.Spans[0 + 2 * w])")
	require(t, solid.Spans[0+3*w] != nil, "solid.Spans[0 + 3 * w])")
	require(t, solid.Spans[1+0*w] == nil, "!solid.Spans[1 + 0 * w])")
	require(t, solid.Spans[1+1*w] != nil, "solid.Spans[1 + 1 * w])")
	require(t, solid.Spans[1+2*w] != nil, "solid.Spans[1 + 2 * w])")
	require(t, solid.Spans[1+3*w] == nil, "!solid.Spans[1 + 3 * w])")

	require(t, solid.Spans[0+0*w].smin == 0, "solid.Spans[0 + 0 * w].smin == 0)")
	require(t, solid.Spans[0+0*w].smax == 1, "solid.Spans[0 + 0 * w].smax == 1)")
	require(t, solid.Spans[0+0*w].area == 1, "solid.Spans[0 + 0 * w].area == 1)")
	require(t, solid.Spans[0+0*w].next == nil, "!solid.Spans[0 + 0 * w].next)")

	require(t, solid.Spans[0+1*w].smin == 0, "solid.Spans[0 + 1 * w].smin == 0)")
	require(t, solid.Spans[0+1*w].smax == 1, "solid.Spans[0 + 1 * w].smax == 1)")
	require(t, solid.Spans[0+1*w].area == 1, "solid.Spans[0 + 1 * w].area == 1)")
	require(t, solid.Spans[0+1*w].next == nil, "!solid.Spans[0 + 1 * w].next)")

	require(t, solid.Spans[0+2*w].smin == 0, "solid.Spans[0 + 2 * w].smin == 0)")
	require(t, solid.Spans[0+2*w].smax == 1, "solid.Spans[0 + 2 * w].smax == 1)")
	require(t, solid.Spans[0+2*w].area == 2, "solid.Spans[0 + 2 * w].area == 2)")
	require(t, solid.Spans[0+2*w].next == nil, "!solid.Spans[0 + 2 * w].next)")

	require(t, solid.Spans[0+3*w].smin == 0, "solid.Spans[0 + 3 * w].smin == 0)")
	require(t, solid.Spans[0+3*w].smax == 1, "solid.Spans[0 + 3 * w].smax == 1)")
	require(t, solid.Spans[0+3*w].area == 2, "solid.Spans[0 + 3 * w].area == 2)")
	require(t, solid.Spans[0+3*w].next == nil, "!solid.Spans[0 + 3 * w].next)")

	require(t, solid.Spans[1+1*w].smin == 0, "solid.Spans[1 + 1 * w].smin == 0)")
	require(t, solid.Spans[1+1*w].smax == 1, "solid.Spans[1 + 1 * w].smax == 1)")
	require(t, solid.Spans[1+1*w].area == 1, "solid.Spans[1 + 1 * w].area == 1)")
	require(t, solid.Spans[1+1*w].next == nil, "!solid.Spans[1 + 1 * w].next)")

	require(t, solid.Spans[1+2*w].smin == 0, "solid.Spans[1 + 2 * w].smin == 0)")
	require(t, solid.Spans[1+2*w].smax == 1, "solid.Spans[1 + 2 * w].smax == 1)")
	require(t, solid.Spans[1+2*w].area == 2, "solid.Spans[1 + 2 * w].area == 2)")
	require(t, solid.Spans[1+2*w].next == nil, "!solid.Spans[1 + 2 * w].next)")

	//SECTION("Triangle list overload")
	vertsList := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		0, 0, 0,
		0, 0, 1,
		1, 0, 0,
	}

	res = RasterizeTriangles2(&ctx, vertsList, areas, 2, solid, flagMergeThr)
	if !res {
		t.Fatalf("result should be true")
	}

	require(t, solid.Spans[0+0*w] != nil, "solid.Spans[0 + 0 * w]")
	require(t, solid.Spans[0+1*w] != nil, "solid.Spans[0 + 1 * w]")
	require(t, solid.Spans[0+2*w] != nil, "solid.Spans[0 + 2 * w]")
	require(t, solid.Spans[0+3*w] != nil, "solid.Spans[0 + 3 * w]")
	require(t, solid.Spans[1+0*w] == nil, "!solid.Spans[1 + 0 * w]")
	require(t, solid.Spans[1+1*w] != nil, "solid.Spans[1 + 1 * w]")
	require(t, solid.Spans[1+2*w] != nil, "solid.Spans[1 + 2 * w]")
	require(t, solid.Spans[1+3*w] == nil, "!solid.Spans[1 + 3 * w]")

	require(t, solid.Spans[0+0*w].smin == 0, "solid.Spans[0 + 0 * w].smin == 0")
	require(t, solid.Spans[0+0*w].smax == 1, "solid.Spans[0 + 0 * w].smax == 1")
	require(t, solid.Spans[0+0*w].area == 1, "solid.Spans[0 + 0 * w].area == 1")
	require(t, solid.Spans[0+0*w].next == nil, "!solid.Spans[0 + 0 * w].next")

	require(t, solid.Spans[0+1*w].smin == 0, "solid.Spans[0 + 1 * w].smin == 0")
	require(t, solid.Spans[0+1*w].smax == 1, "solid.Spans[0 + 1 * w].smax == 1")
	require(t, solid.Spans[0+1*w].area == 1, "solid.Spans[0 + 1 * w].area == 1")
	require(t, solid.Spans[0+1*w].next == nil, "!solid.Spans[0 + 1 * w].next")

	require(t, solid.Spans[0+2*w].smin == 0, "solid.Spans[0 + 2 * w].smin == 0")
	require(t, solid.Spans[0+2*w].smax == 1, "solid.Spans[0 + 2 * w].smax == 1")
	require(t, solid.Spans[0+2*w].area == 2, "solid.Spans[0 + 2 * w].area == 2")
	require(t, solid.Spans[0+2*w].next == nil, "!solid.Spans[0 + 2 * w].next")

	require(t, solid.Spans[0+3*w].smin == 0, "solid.Spans[0 + 3 * w].smin == 0")
	require(t, solid.Spans[0+3*w].smax == 1, "solid.Spans[0 + 3 * w].smax == 1")
	require(t, solid.Spans[0+3*w].area == 2, "solid.Spans[0 + 3 * w].area == 2")
	require(t, solid.Spans[0+3*w].next == nil, "!solid.Spans[0 + 3 * w].next")

	require(t, solid.Spans[1+1*w].smin == 0, "solid.Spans[1 + 1 * w].smin == 0")
	require(t, solid.Spans[1+1*w].smax == 1, "solid.Spans[1 + 1 * w].smax == 1")
	require(t, solid.Spans[1+1*w].area == 1, "solid.Spans[1 + 1 * w].area == 1")
	require(t, solid.Spans[1+1*w].next == nil, "!solid.Spans[1 + 1 * w].next")

	require(t, solid.Spans[1+2*w].smin == 0, "solid.Spans[1 + 2 * w].smin == 0")
	require(t, solid.Spans[1+2*w].smax == 1, "solid.Spans[1 + 2 * w].smax == 1")
	require(t, solid.Spans[1+2*w].area == 2, "solid.Spans[1 + 2 * w].area == 2")
	require(t, solid.Spans[1+2*w].next == nil, "!solid.Spans[1 + 2 * w].next")
}
