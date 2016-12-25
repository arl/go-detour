package recast

import (
	"fmt"
	"testing"
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
