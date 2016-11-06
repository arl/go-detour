package detour

import "testing"

func TestVcross(t *testing.T) {

	vecTests := []struct {
		v1, v2 [3]float32
		want   [3]float32
	}{
		{
			[3]float32{3, -3, 1},
			[3]float32{4, 9, 2},
			[3]float32{-15, -2, 39},
		},
		{
			[3]float32{3, -3, 1},
			[3]float32{3, -3, 1},
			[3]float32{0, 0, 0},
		},
	}

	for _, tt := range vecTests {
		var dst [3]float32
		dtVcross(dst[:], tt.v1[:], tt.v2[:])

		// TODO: we should have an Approx for vec3
		for i := range dst {
			c := Approxf32Equal(tt.want[i], dst[i])
			if !c {
				t.Errorf("want dst[%d] (%f) ~= %f, got !=", i, dst[i], tt.want[i])
			}
		}
	}
}

func TestVdot(t *testing.T) {

	vecTests := []struct {
		v1, v2 [3]float32
		want   float32
	}{
		{
			[3]float32{1, 0, 0},
			[3]float32{1, 0, 0},
			1,
		},
		{
			[3]float32{1, 2, 3},
			[3]float32{0, 0, 0},
			0,
		},
	}

	for _, tt := range vecTests {
		got := dtVdot(tt.v1[:], tt.v2[:])
		if !Approxf32Equal(tt.want, got) {
			t.Errorf("%v . %v, want %f, got %f", tt.v1, tt.v2, tt.want, got)
		}
	}
}

func TestApproxf32Equal(t *testing.T) {

	f32eqTests := []struct {
		v1, v2 float32
		want   bool // true means equal
	}{
		{1.0, 1.0, true},
		{1.0, 1.000001, true},
		{1.0, 1.00001, true},
		{1.0, 1.0001, false},
		{1.0, 1.001, false},
		{1.0, 1.01, false},
		{1.0, 0.999999, true},
		{1.0, 0.99999, true},
		{1.0, 0.9999, false},
		{1.0, 0.999, false},
		{1.0, 0.99, false},
		{0.0, 0.000001, true},
		{0.0, 0.00001, true},
		{0.0, 0.0001, false},
		{0.0, 0.001, false},
		{0.0, 0.01, false},
		{0.0, -0.000001, true},
		{0.0, -0.00001, true},
		{0.0, -0.0001, false},
		{0.0, -0.001, false},
		{0.0, -0.01, false},
		{1e12, 1e12 + 0.000001, true},
		{1e12, 1e12 + 0.00001, true},
		{1e12, 1e12 + 0.0001, true},
		{1e12, 1e12 + 0.001, true},
		{1e12, 1e12 + 0.01, true},
		{1e12, 1e12 - 0.000001, true},
		{1e12, 1e12 - 0.00001, true},
		{1e12, 1e12 - 0.0001, true},
		{1e12, 1e12 - 0.001, true},
		{1e12, 1e12 - 0.01, true},
		{NaN, 0, false},
		{NaN, NaN, false},
	}

	for _, tt := range f32eqTests {
		got := Approxf32Equal(tt.v1, tt.v2)
		if got != tt.want {
			t.Errorf("%f approx equals to %f, got %t, want %t", tt.v1, tt.v2, got, tt.want)
		}
	}
}
