package detour

type BufWriter struct {
	buf []byte
	off int
}

func NewBufWriter(buf []byte) *BufWriter {
	return &BufWriter{
		buf: buf,
	}
}

func (bf *BufWriter) Write(p []byte) (n int, err error) {
	if len(p)+bf.off <= len(bf.buf) {
		copy(bf.buf[bf.off:], p)
		bf.GoForward(len(p))
		return len(p), nil
	}
	panic("going past buffer end")
}

func (bf *BufWriter) GoForward(n int) error {
	if n+bf.off <= len(bf.buf) {
		bf.off += n
		return nil
	}
	panic("going past buffer end")
}

func (bf *BufWriter) Current() int {
	return bf.off
}

func (bf *BufWriter) Set(n int) error {
	if n <= len(bf.buf) {
		bf.off = n
		return nil
	}
	panic("going past buffer end")
}

func (bf *BufWriter) Next(n int) []byte {
	if n <= bf.off+len(bf.buf) {
		var buf []byte
		buf = bf.buf[bf.off : bf.off+n]
		bf.off += n
		return buf
	}
	panic("going past buffer end")
}
