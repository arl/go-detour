package detour

import (
	"bufio"
	"fmt"
	"os"
	"strings"

	"github.com/aurelien-rainone/math32"
)

type rcMeshLoaderObj struct {
	m_filename  string
	m_scale     float32
	m_verts     []float32
	m_tris      []int32
	m_normals   []float32
	m_vertCount int32
	m_triCount  int32
}

func newMeshLoaderObj() *rcMeshLoaderObj {
	return &rcMeshLoaderObj{
		m_scale: 1.0,
	}
}

func (mlo *rcMeshLoaderObj) addVertex(x, y, z float32, ca *int32) {
	if ca == nil {
		panic("ca should not be nil")
	}
	if mlo.m_vertCount+1 > *ca {
		if *ca == 0 {
			*ca = 8
		} else {
			*ca = *ca * 2
		}

		nv := make([]float32, (*ca)*3)
		if mlo.m_vertCount != 0 {
			//memcpy(nv, m_verts, m_vertCount*3*sizeof(float));
			copy(nv, mlo.m_verts[:mlo.m_vertCount*3*4])
		}
		mlo.m_verts = nv
	}
	dst := mlo.m_verts[mlo.m_vertCount*3 : 3+mlo.m_vertCount*3]
	dst[0] = x * mlo.m_scale
	dst[1] = y * mlo.m_scale
	dst[2] = z * mlo.m_scale
	mlo.m_vertCount++
}

func (mlo *rcMeshLoaderObj) addTriangle(a, b, c int32, ca *int32) {
	if ca == nil {
		panic("ca should not be nil")
	}
	if mlo.m_triCount+1 > *ca {
		if *ca != 0 {
			*ca = 8
		} else {
			*ca = (*ca) * 2
		}
		nv := make([]int32, (*ca)*3)
		if mlo.m_triCount != 0 {
			//memcpy(nv, m_tris, m_triCount*3*sizeof(int));
			copy(nv, mlo.m_tris[:mlo.m_triCount*3*4])
		}
		mlo.m_tris = nv
	}
	dst := mlo.m_tris[mlo.m_triCount*3 : 3+mlo.m_triCount*3]
	dst[0] = a
	dst[1] = b
	dst[2] = c
	mlo.m_triCount++
}

//func parseRow(buf, row string, int len) string {
////start := true;
////done := false;
////int n = 0;
////for (!done && buf < bufEnd) {
////char c = *buf;
////buf++;
////// multirow
////switch (c)
////{
////case '\\':
////break;
////case '\n':
////if (start) break;
////done = true;
////break;
////case '\r':
////break;
////case '\t':
////case ' ':
////if (start) break;
////default:
////start = false;
////row[n++] = c;
////if (n >= len-1)
////done = true;
////break;
////}
////}
////row[n] = '\0';
////return buf;
//}

func parseFace(row string, data []int32, n, vcnt int32) int {
	panic("code is MISSING here")
	return 1
}

func (mlo *rcMeshLoaderObj) load(filename string) error {
	in, err := os.Open(filename)
	if err != nil {
		return err
	}
	defer in.Close()

	scanner := bufio.NewScanner(in)

	var (
	//row     string
	//face    [32]int
	//x, y, z float32
	//nv      int32
	//vcap    int32
	//tcap    int32
	)

	for scanner.Scan() {

		line := strings.Split(scanner.Text(), " ")
		kw, vals := line[0], line[1:]
		fmt.Println("kw", kw, "vals", vals)

		switch kw {
		case "v":
		case "f":
		}

		//// Parse one row
		//src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		//// Skip comments
		//if (row[0] == '#') continue;
		//if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		//{
		//// Vertex pos
		//sscanf(row+1, "%f %f %f", &x, &y, &z);
		//addVertex(x, y, z, vcap);
		//}
		//if (row[0] == 'f')
		//{
		//// Faces
		//nv = parseFace(row+1, face, 32, m_vertCount);
		//for (int i = 2; i < nv; ++i)
		//{
		//const int a = face[0];
		//const int b = face[i-1];
		//const int c = face[i];
		//if (a < 0 || a >= m_vertCount || b < 0 || b >= m_vertCount || c < 0 || c >= m_vertCount)
		//continue;
		//addTriangle(a, b, c, tcap);
		//}
		//}
	}

	// Calculate normals.
	mlo.m_normals = make([]float32, mlo.m_triCount*3)
	for i := int32(0); i < mlo.m_triCount*3; i += 3 {
		v0 := mlo.m_verts[mlo.m_tris[i]*3 : 3+mlo.m_tris[i]*3]
		v1 := mlo.m_verts[mlo.m_tris[i+1]*3 : 3+mlo.m_tris[i+1]*3]
		v2 := mlo.m_verts[mlo.m_tris[i+2]*3 : 3+mlo.m_tris[i+2]*3]
		var e0, e1 [3]float32
		for j := 0; j < 3; j++ {
			e0[j] = v1[j] - v0[j]
			e1[j] = v2[j] - v0[j]
		}
		n := mlo.m_normals[i : 3+i]
		n[0] = e0[1]*e1[2] - e0[2]*e1[1]
		n[1] = e0[2]*e1[0] - e0[0]*e1[2]
		n[2] = e0[0]*e1[1] - e0[1]*e1[0]
		d := math32.Sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2])
		if d > 0 {
			d = 1.0 / d
			n[0] *= d
			n[1] *= d
			n[2] *= d
		}
	}

	mlo.m_filename = filename
	return nil
}
