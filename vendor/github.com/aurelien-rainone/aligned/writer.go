package aligned

import (
	"encoding/binary"
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
	var slice reflect.Value
	rv := reflect.ValueOf(s)
	if rv.Kind() == reflect.Slice {
		slice = rv
	} else if rv.Kind() == reflect.Ptr {
		slice = rv.Elem()
	}

	// get element size and number of elements
	length := slice.Len()
	if length != 0 {
		// write the whole slice with the embedded reader
		if err := binary.Write(aw.w, aw.order, slice.Interface()); err != nil {
			return err
		}
		// compute the padding
		total := int(slice.Index(0).Type().Size()) * length
		if pad := AlignN(total, aw.align) - total; pad != 0 {
			// move the write forward of 'pad' bytes
			npad, err := aw.w.Write(aw.padbuf[:pad])
			switch {
			case err == io.EOF:
				return io.ErrUnexpectedEOF
			case err != nil:
				return err
			case npad < pad:
				// not enough padding to keep the data stream aligned
				return io.ErrUnexpectedEOF
			}
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
