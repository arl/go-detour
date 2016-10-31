package detour

import (
	"fmt"
	"testing"
)

func TestNextPow2(t *testing.T) {
	var i uint32
	for i = 0; i < 17; i++ {
		fmt.Println(i, "->", dtNextPow2(i))
	}
}

func TestIlog2(t *testing.T) {
	var i uint32
	for i = 0; i < 17; i++ {
		fmt.Println(i, "->", dtIlog2(i))
	}
}
