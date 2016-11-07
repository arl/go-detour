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

func TestVmad(t *testing.T) {
	vecTests := []struct {
		v1, v2 [3]float32
		s      float32
		want   [3]float32
	}{
		{
			[3]float32{1, 2, 3},
			[3]float32{0, 2, 4},
			2.0,
			[3]float32{1, 6, 11},
		},
		{
			[3]float32{1, 2, 3},
			[3]float32{5, 6, 7},
			0.0,
			[3]float32{1, 2, 3},
		},
	}

	for _, tt := range vecTests {
		var dst [3]float32
		dtVmad(dst[:], tt.v1[:], tt.v2[:], tt.s)

		// TODO: we should have an Approx for vec3
		for i := range dst {
			c := Approxf32Equal(tt.want[i], dst[i])
			if !c {
				t.Errorf("want dst[%d] (%f) ~= %f, got !=", i, dst[i], tt.want[i])
			}
		}
	}
}

func TestVadd(t *testing.T) {
	vecTests := []struct {
		v1, v2 [3]float32
		want   [3]float32
	}{
		{
			[3]float32{1, 2, 3},
			[3]float32{5, 6, 7},
			[3]float32{6, 8, 10},
		},
	}

	for _, tt := range vecTests {
		var dst [3]float32
		dtVadd(dst[:], tt.v1[:], tt.v2[:])

		// TODO: we should have an Approx for vec3
		for i := range dst {
			c := Approxf32Equal(tt.want[i], dst[i])
			if !c {
				t.Errorf("want dst[%d] (%f) ~= %f, got !=", i, dst[i], tt.want[i])
			}
		}
	}
}
