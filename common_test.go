package detour

import "testing"

func TestNextPow2(t *testing.T) {
	powTests := []struct {
		v    uint32
		want uint32
	}{

		{0, 0},
		{1, 1},
		{2, 2},
		{3, 4},
		{4, 4},
		{5, 8},
		{6, 8},
		{7, 8},
		{8, 8},
		{9, 16},
		{10, 16},
		{11, 16},
		{12, 16},
		{13, 16},
		{14, 16},
		{15, 16},
		{16, 16},
	}
	for _, tt := range powTests {
		got := dtNextPow2(tt.v)
		if got != tt.want {
			t.Errorf("want NextPow2(%d) == %d, got %d", tt.v, tt.want, got)
		}
	}
}

func TestIlog2(t *testing.T) {
	logTests := []struct {
		v    uint32
		want uint32
	}{
		{0, 0},
		{1, 0},
		{2, 1},
		{3, 1},
		{4, 2},
		{5, 2},
		{6, 2},
		{7, 2},
		{8, 3},
		{9, 3},
		{10, 3},
		{11, 3},
		{12, 3},
		{13, 3},
		{14, 3},
		{15, 3},
		{16, 4},
	}
	for _, tt := range logTests {
		got := dtIlog2(tt.v)
		if got != tt.want {
			t.Errorf("want Ilog2(%d) == %d, got %d", tt.v, tt.want, got)
		}
	}
}
