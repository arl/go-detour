package aligned

import (
	"encoding/binary"
	"fmt"
	"io"
	"reflect"
)

// A Writer manages its embedded io.Writer so as to guarantee that every write
// operation produces an amount of bytes that is a multiple of the provided
// alignment.
type Writer struct {
	w      io.Writer        // Writer to which calls are forwarded
	align  int              // byte alignment
	padbuf []byte           // input buffer for padding
	order  binary.ByteOrder // byte order to use when packing
}

// NewWriter creates a Writer that performs aligned write operation on w.
func NewWriter(w io.Writer, align int, order binary.ByteOrder) *Writer {
	return &Writer{
		w:      w,
		align:  align,
		padbuf: make([]byte, align),
		order:  order,
	}
}

// Write writes len(b) bytes from b to the underlying data stream.
func (aw *Writer) Write(b []byte) (n int, err error) {
	n, err = aw.w.Write(b)
	switch {
	case err != nil:
		fallthrough
	case n < len(b):
		// couldn't write all the contents of b
		return n, err
	}

	// compute the number of padding bytes we need
	var pad, npad int
	pad = AlignN(n, aw.align) - n
	if pad != 0 {
		// write padding byte(s)
		npad, err = aw.w.Write(aw.padbuf[:pad])
		switch {
		case err != nil:
			fallthrough
		case npad < pad:
			// couldn't write all the required padding
			return n, err
		}
	}
	return n + pad, nil
}

// WriteSlice writes len(s) aligned elements of data into the underlying data
// stream.
//
// s must be a slice.
func (aw *Writer) WriteSlice(s interface{}) error {
	rt, rv := reflect.TypeOf(s), reflect.ValueOf(s)
	if rt.Kind() == reflect.Ptr || rv.Kind() != reflect.Slice {
		return fmt.Errorf("data must be a slice")
	}
	for idx := 0; idx < rv.Len(); idx++ {
		err := binary.Write(aw, aw.order, rv.Index(idx).Interface())
		if err != nil {
			return err
		}
	}
	return nil
}

// WriteVal writes the aligned binary representation of v into the embedded
// io.Writer.
//
// see binary.Write from encoding/binary package for more information.
func (aw *Writer) WriteVal(v interface{}) error {
	return binary.Write(aw, aw.order, v)
}
