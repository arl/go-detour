package gobj

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"strings"
)

// Load reads an .obj formatted file and returns a gobj.File.
//
// This is a convenience function that calls Decode internally.
func Load(path string) (*OBJFile, error) {
	in, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer in.Close()
	return Decode(in)
}

// Decode reads an .obj formatted geometry definition from the provided reader
// and returns a gobj.File, or nil and an error.
func Decode(r io.Reader) (*OBJFile, error) {
	lineno := 1

	// init min/max values
	obj := OBJFile{aabb: NewAABB()}

	scanner := bufio.NewScanner(r)
	for scanner.Scan() {

		line := strings.Split(scanner.Text(), " ")
		kw, vals := line[0], line[1:]

		switch kw {

		case "v":
			err := obj.parseVertex(kw, vals)
			if err != nil {
				return nil, fmt.Errorf("error parsing vertex at line %d,\"%v\": %s", lineno, vals, err)

			}

		case "f":
			err := obj.parseFace(kw, vals)
			if err != nil {
				return nil, fmt.Errorf("error parsing face at line %d,\"%v\": %s", lineno, vals, err)
			}

		default:
			// ignore everything else
		}

		lineno++
	}
	if err := scanner.Err(); err != nil {
		return nil, fmt.Errorf("error parsing file: %v", err)
	}

	return &obj, nil
}
