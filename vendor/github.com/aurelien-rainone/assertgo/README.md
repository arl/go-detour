# assertgo &nbsp;&nbsp; [![GoDoc](http://img.shields.io/badge/go-documentation-blue.svg?style=flat-square)](http://godoc.org/github.com/aurelien-rainone/assertgo) 
**Conditionally compiled assertions in Go**


## Example program

**`main.go`**:

```go
package main

import (
	"fmt"

	"github.com/aurelien-rainone/assertgo"
)

func main() {
	assert.True(false, "sent false to assert.True")
	fmt.Println("program end")
}
```


### Normal run

```bash
$ go run main.go
program end
```

Nothing happened because `assert.True()` is conditionnaly compiled to a noop,
(i.e an empty function).


### Debug run

```bash
$ go run -tags debug main.go
2016/11/02 22:43:59 --- --- Debug Assertion Failed --- --- ---
panic: sent false to assert.True

goroutine 1 [running]:
panic(0x496be0, 0xc420062200)
	/usr/local/go/src/runtime/panic.go:500 +0x1a1
github.com/aurelien-rainone/assertgo.True(0xc42003ff00, 0x4b5298, 0x19, 0x0, 0x0, 0x0)
	/home/panty/godev/src/github.com/aurelien-rainone/assertgo/assert.go:19 +0x13b
main.main()
	/home/panty/godev/src/github.com/aurelien-rainone/tagtest/main.go:10 +0x5a
exit status 2
```

By providing the `debug` tag to `go build` or any other `go` tool that accepts
[build tags](https://golang.org/pkg/go/build/), `assert.True` is conditionnaly
compiled to a function that calls `panic` if the assertion is `false`.
