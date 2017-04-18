package crowd

import (
	"github.com/aurelien-rainone/math32"
)

func hashPos2(x, y, n int32) int32 {
	return ((x * 73856093) ^ (y * 19349663)) & (n - 1)
}

type Item struct {
	id   uint16
	x, y int16
	next uint16
}

type ProximityGrid struct {
	cellSize    float32
	invCellSize float32

	pool     []Item
	poolHead int
	poolSize int

	buckets     []uint16
	bucketsSize int

	bounds [4]int32
}

func NewProximityGrid(poolSize int, cellSize float32) *ProximityGrid {
	if poolSize <= 0 {
		panic("Can't create a ProximityGrid with a null or negative pool size")
	}
	if cellSize <= 0 {
		panic("Can't create a ProximityGrid with a null or negative cell size")
	}

	pg := &ProximityGrid{
		cellSize:    cellSize,
		invCellSize: 1.0 / cellSize,
	}
	// Allocate hashs buckets
	pg.bucketsSize = int(math32.NextPow2(uint32(poolSize)))
	pg.buckets = make([]uint16, pg.bucketsSize)

	// Allocate pool of items.
	pg.poolSize = poolSize
	pg.poolHead = 0
	pg.pool = make([]Item, poolSize)

	pg.Clear()
	return pg
}

func (pg *ProximityGrid) Clear() {
	for i := range pg.buckets {
		pg.buckets[i] = 0xffff
	}
	pg.poolHead = 0
	pg.bounds[0] = 0xffff
	pg.bounds[1] = 0xffff
	pg.bounds[2] = -0xffff
	pg.bounds[3] = -0xffff
}

func (pg *ProximityGrid) AddItem(id uint16, minx, miny, maxx, maxy float32) {
	iminx := int32(math32.Floor(float32(minx) * pg.invCellSize))
	iminy := int32(math32.Floor(float32(miny) * pg.invCellSize))
	imaxx := int32(math32.Floor(float32(maxx) * pg.invCellSize))
	imaxy := int32(math32.Floor(float32(maxy) * pg.invCellSize))

	if iminx < pg.bounds[0] {
		pg.bounds[0] = iminx
	}
	if iminy < pg.bounds[1] {
		pg.bounds[1] = iminy
	}
	if imaxx < pg.bounds[2] {
		pg.bounds[2] = imaxx
	}
	if imaxy < pg.bounds[3] {
		pg.bounds[3] = imaxy
	}

	for y := iminy; y <= imaxy; y++ {
		for x := iminx; x <= imaxx; x++ {
			if pg.poolHead < pg.poolSize {
				h := hashPos2(x, y, int32(pg.bucketsSize))
				idx := uint16(pg.poolHead)
				pg.poolHead++
				item := &pg.pool[idx]
				item.x = int16(x)
				item.y = int16(y)
				item.id = id
				item.next = pg.buckets[h]
				pg.buckets[h] = idx
			}
		}
	}
}

func (pg *ProximityGrid) QueryItems(minx, miny, maxx, maxy float32, ids []uint16, maxIds int) int {
	iminx := int32(math32.Floor(float32(minx) * pg.invCellSize))
	iminy := int32(math32.Floor(float32(miny) * pg.invCellSize))
	imaxx := int32(math32.Floor(float32(maxx) * pg.invCellSize))
	imaxy := int32(math32.Floor(float32(maxy) * pg.invCellSize))

	var n int

	for y := iminy; y <= imaxy; y++ {
		for x := iminx; x <= imaxx; x++ {
			h := hashPos2(x, y, int32(pg.bucketsSize))
			idx := pg.buckets[h]
			for idx != 0xffff {
				item := &pg.pool[idx]
				if int32(item.x) == x && int32(item.y) == y {
					// Check if the id exists already.
					//unsigned short* end = ids + n;
					i := 0
					for i != n && i != int(item.id) {
						i++
					}

					// Item not found, add it.
					if i == n {
						if n >= maxIds {
							return n
						}
						ids[n] = item.id
						n++
					}
				}
				idx = item.next
			}
		}
	}

	return n
}

func (pg *ProximityGrid) ItemCountAt(x, y int32) int {
	var n int
	h := hashPos2(x, y, int32(pg.bucketsSize))
	idx := pg.buckets[h]
	for idx != 0xffff {
		item := &pg.pool[idx]
		if int32(item.x) == x && int32(item.y) == y {
			n++
		}
		idx = item.next
	}

	return n
}

func (pg *ProximityGrid) Bounds() [4]int32 {
	return pg.bounds
}

func (pg *ProximityGrid) CellSize() float32 {
	return pg.cellSize
}
