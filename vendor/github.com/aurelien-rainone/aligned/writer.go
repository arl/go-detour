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
	if err == nil && n < len(b) {
		// couldn't write all the contents of b
		err = io.ErrUnexpectedEOF
	}
	switch err {
	case nil:
		break
	case io.EOF:
		fallthrough
	case io.ErrUnexpectedEOF:
		return 0, io.ErrUnexpectedEOF
	default:
		return n, err
	}

	// compute the number of padding bytes we need
	var pad, npad int
	pad = AlignN(n, aw.align) - n
	if pad != 0 {
		// write padding byte(s)
		npad, err = aw.w.Write(aw.padbuf[:pad])
		if err == nil && npad < pad {
			// not enough padding to keep the data stream aligned
			err = io.ErrUnexpectedEOF
		}
		switch err {
		case nil:
			break
		case io.EOF:
			fallthrough
		case io.ErrUnexpectedEOF:
			return 0, io.ErrUnexpectedEOF
		default:
			return npad, err
		}
	}
	return n + pad, nil
}

// WriteSlice writes len(s) aligned elements of data into the underlying data
// stream.
//
// s must be a slice.
func (aw *Writer) WriteSlice(s interface{}) error {
	var (
		slice reflect.Value
		err   error
	)
	rv := reflect.ValueOf(s)
	if rv.Kind() == reflect.Slice {
		slice = rv
	} else if rv.Kind() == reflect.Ptr {
		slice = rv.Elem()
	}

	// get element size and number of elements
	length := slice.Len()
	if length != 0 {
		for i := 0; i < length; i++ {
			err = binary.Write(aw, aw.order, slice.Index(i).Interface())
			if err != nil {
				return err
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
