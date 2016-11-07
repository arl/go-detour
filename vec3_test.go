package detour

import "testing"

func TestVcross(t *testing.T) {
	vecTests := []struct {
		v1, v2 Vec3
		want   Vec3
	}{
		{
			Vec3{3, -3, 1},
			Vec3{4, 9, 2},
			Vec3{-15, -2, 39},
		},
		{
			Vec3{3, -3, 1},
			Vec3{3, -3, 1},
			Vec3{0, 0, 0},
		},
	}

	for _, tt := range vecTests {
		dst := NewVec3()
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
		v1, v2 Vec3
		want   float32
	}{
		{
			Vec3{1, 0, 0},
			Vec3{1, 0, 0},
			1,
		},
		{
			Vec3{1, 2, 3},
			Vec3{0, 0, 0},
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
		v1, v2 Vec3
		s      float32
		want   Vec3
	}{
		{
			Vec3{1, 2, 3},
			Vec3{0, 2, 4},
			2.0,
			Vec3{1, 6, 11},
		},
		{
			Vec3{1, 2, 3},
			Vec3{5, 6, 7},
			0.0,
			Vec3{1, 2, 3},
		},
	}

	for _, tt := range vecTests {
		dst := NewVec3()
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
		v1, v2 Vec3
		want   Vec3
	}{
		{
			Vec3{1, 2, 3},
			Vec3{5, 6, 7},
			Vec3{6, 8, 10},
		},
	}

	for _, tt := range vecTests {
		dst := NewVec3()
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
