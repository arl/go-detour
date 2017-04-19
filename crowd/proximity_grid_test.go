package crowd

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestProximityGridAddItem(t *testing.T) {
	pg := NewProximityGrid(10, 1)
	assert.Equal(t, 0, pg.ItemCountAt(1, 1), "grid should be empty")

	pg.AddItem(1, 1, 1, 2, 2)
	assert.Equal(t, 1, pg.ItemCountAt(1, 1), "should have 1 item in the grid")

	pg.Clear()
	assert.Equal(t, 0, pg.ItemCountAt(1, 1), "grid should be empty")

	pg.AddItem(1, 1, 1, 2, 2)
	assert.Equal(t, 1, pg.ItemCountAt(1, 1), "should have 1 item in the grid")

	pg.AddItem(2, 1, 1, 2, 2)
	assert.Equal(t, 2, pg.ItemCountAt(1, 1), "should have 2 items in the grid")
}
