package aligned

import (
	"errors"
	"fmt"
	"reflect"
	"testing"
)

func assertPanic(t *testing.T, f func(), format string, a ...interface{}) {
	defer func() {
		if r := recover(); r == nil {
			t.Errorf("should have panicked but didn't: " + fmt.Sprintf(format, a...))
		}
	}()
	f()
}

// create and return an addressable slice of t
func makeSlice(t reflect.Type, l, c int) reflect.Value {
	// create the underlying slice value
	slice := reflect.MakeSlice(reflect.SliceOf(t), l, c)

	// create a value representing a pointer and set it to the slice
	x := reflect.New(slice.Type())
	x.Elem().Set(slice)
	return x
}

type erroringReader struct{ total, trigger int }

var errGenericReadError = errors.New("generic read error")

func (er *erroringReader) Read(b []byte) (n int, err error) {
	er.total += len(b)
	if er.total >= er.trigger {
		return 0, errGenericReadError
	}
	return len(b), nil
}
