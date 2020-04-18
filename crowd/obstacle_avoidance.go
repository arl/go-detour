package crowd

import (
	"math"

	"github.com/arl/go-detour/detour"
	"github.com/arl/gogeo/f32"
	"github.com/arl/gogeo/f32/d3"
	"github.com/arl/math32"
)

type ObstacleCircle struct {
	p      d3.Vec3 // Position of the obstacle
	vel    d3.Vec3 // Velocity of the obstacle
	dvel   d3.Vec3 // Velocity of the obstacle
	rad    float32 // Radius of the obstacle
	dp, np d3.Vec3 // Use for side selection during sampling.
}

func NewObstacleCircle() *ObstacleCircle {
	return &ObstacleCircle{
		p:    d3.NewVec3(),
		vel:  d3.NewVec3(),
		dvel: d3.NewVec3(),
		dp:   d3.NewVec3(),
		np:   d3.NewVec3(),
	}
}

type ObstacleSegment struct {
	p, q  d3.Vec3 // End points of the obstacle segment
	touch bool
}

func NewObstacleSegment() *ObstacleSegment {
	return &ObstacleSegment{
		p: d3.NewVec3(),
		q: d3.NewVec3(),
	}
}

type ObstacleAvoidanceDebugData struct {
	nsamples   int
	maxSamples int
	vel        []float32
	ssize      []float32
	pen        []float32
	vpen       []float32
	vcpen      []float32
	spen       []float32
	tpen       []float32
}

func NewObstacleAvoidanceDebugData(maxSamples int) *ObstacleAvoidanceDebugData {
	if maxSamples == 0 {
		panic("oad.maxSamples should not be 0")
	}
	return &ObstacleAvoidanceDebugData{
		maxSamples: maxSamples,
		vel:        make([]float32, 3*maxSamples),
		ssize:      make([]float32, maxSamples),
		pen:        make([]float32, maxSamples),
		vpen:       make([]float32, maxSamples),
		vcpen:      make([]float32, maxSamples),
		spen:       make([]float32, maxSamples),
		tpen:       make([]float32, maxSamples),
	}
}

func (oad *ObstacleAvoidanceDebugData) reset() {
	oad.nsamples = 0
}

func (oad *ObstacleAvoidanceDebugData) addSample(vel d3.Vec3, ssize, pen, vpen, vcpen, spen, tpen float32) {

	if oad.nsamples >= oad.maxSamples {
		return
	}
	copy(oad.vel[oad.nsamples*3:], vel[:3])
	oad.ssize[oad.nsamples] = ssize
	oad.pen[oad.nsamples] = pen
	oad.vpen[oad.nsamples] = vpen
	oad.vcpen[oad.nsamples] = vcpen
	oad.spen[oad.nsamples] = spen
	oad.tpen[oad.nsamples] = tpen
	oad.nsamples++
}

func normalizeArray(arr []float32, n int) {
	// Normalize penaly range.
	var (
		minPen float32 = math.MaxFloat32
		maxPen float32 = -math.MaxFloat32
	)
	for i := 0; i < n; i++ {
		if arr[i] < minPen {
			minPen = arr[i]
		}
		if arr[i] > maxPen {
			maxPen = arr[i]
		}
	}
	penRange := maxPen - minPen
	var s float32 = 1
	if penRange > 0.001 {
		s = 1.0 / penRange
	}

	for i := 0; i < n; i++ {
		arr[i] = f32.Clamp((arr[i]-minPen)*s, 0.0, 1.0)
	}
}

func (oad *ObstacleAvoidanceDebugData) normalizeSamples() {
	normalizeArray(oad.pen, oad.nsamples)
	normalizeArray(oad.vpen, oad.nsamples)
	normalizeArray(oad.vcpen, oad.nsamples)
	normalizeArray(oad.spen, oad.nsamples)
	normalizeArray(oad.tpen, oad.nsamples)
}

func (oad *ObstacleAvoidanceDebugData) SampleCount() int {
	return oad.nsamples
}

func (oad *ObstacleAvoidanceDebugData) SampleVelocity(i int) d3.Vec3 {
	return oad.vel[i*3 : (i+1)*3]
}

func (oad *ObstacleAvoidanceDebugData) SampleSize(i int) float32 {
	return oad.ssize[i]
}

func (oad *ObstacleAvoidanceDebugData) SamplePenalty(i int) float32 {
	return oad.pen[i]
}

func (oad *ObstacleAvoidanceDebugData) SampleDesiredVelocityPenalty(i int) float32 {
	return oad.vpen[i]
}

func (oad *ObstacleAvoidanceDebugData) SampleCurrentVelocityPenalty(i int) float32 {
	return oad.vcpen[i]
}

func (oad *ObstacleAvoidanceDebugData) SamplePreferredVelocityPenalty(i int) float32 {
	return oad.spen[i]
}

func (oad *ObstacleAvoidanceDebugData) SampleCollisionTimePenalty(i int) float32 {
	return oad.tpen[i]
}

const (
	maxPatternDivs  = 32 // Max numver of adaptive divs.
	maxPatternRings = 4  // Max number of adaptive rings.
)

type ObstacleAvoidanceParams struct {
	velBias       float32
	weightDesVel  float32
	weightCurVel  float32
	weightSide    float32
	weightToi     float32
	horizTime     float32
	gridSize      uint8 // grid
	adaptiveDivs  uint8 // adaptive
	adaptiveRings uint8 // adaptive
	adaptiveDepth uint8 // adaptive
}

type ObstacleAvoidanceQuery struct {
	params       ObstacleAvoidanceParams
	invHorizTime float32
	vmax         float32
	invVmax      float32

	maxCircles int
	circles    []ObstacleCircle
	ncircles   int

	maxSegments int
	segments    []ObstacleSegment
	nsegments   int
}

func NewObstacleAvoidanceQuery(maxCircles, maxSegments int) *ObstacleAvoidanceQuery {
	oaq := &ObstacleAvoidanceQuery{
		maxCircles:  maxCircles,
		ncircles:    0,
		circles:     make([]ObstacleCircle, maxCircles),
		maxSegments: maxSegments,
		nsegments:   0,
		segments:    make([]ObstacleSegment, maxSegments),
	}

	for i := range oaq.circles {
		oaq.circles[i] = *NewObstacleCircle()
	}

	for i := range oaq.segments {
		oaq.segments[i] = *NewObstacleSegment()
	}

	return oaq
}

func (oaq *ObstacleAvoidanceQuery) reset() {
	oaq.ncircles = 0
	oaq.nsegments = 0
}

func (oaq *ObstacleAvoidanceQuery) addCircle(pos d3.Vec3, rad float32, vel, dvel d3.Vec3) {
	if oaq.ncircles >= oaq.maxCircles {
		return
	}

	cir := &oaq.circles[oaq.ncircles]
	oaq.ncircles++
	copy(cir.p, pos[:3])
	cir.rad = rad
	copy(cir.vel, vel[:3])
	copy(cir.dvel, dvel)
}

func (oaq *ObstacleAvoidanceQuery) addSegment(p, q d3.Vec3) {
	if oaq.nsegments >= oaq.maxSegments {
		return
	}

	seg := &oaq.segments[oaq.nsegments]
	oaq.nsegments++
	copy(seg.p, p[:3])
	copy(seg.q, q[:3])
}

func (oaq *ObstacleAvoidanceQuery) sampleVelocityGrid(pos d3.Vec3, rad, vmax float32, vel, dvel, nvel d3.Vec3, params *ObstacleAvoidanceParams,
	debug *ObstacleAvoidanceDebugData) int {
	oaq.prepare(pos, dvel)

	oaq.params = *params
	oaq.invHorizTime = 1.0 / oaq.params.horizTime
	oaq.vmax = vmax
	oaq.invVmax = math.MaxFloat32
	if vmax > 0 {
		oaq.invVmax = 1.0 / vmax
	}

	nvel.SetXYZ(0, 0, 0)

	if debug != nil {
		debug.reset()
	}

	cvx := dvel[0] * oaq.params.velBias
	cvz := dvel[2] * oaq.params.velBias
	cs := vmax * 2 * (1 - oaq.params.velBias) / float32(oaq.params.gridSize-1)
	half := float32(oaq.params.gridSize-1) * cs * 0.5

	var minPenalty float32 = math.MaxFloat32
	var ns int = 0

	for y := uint8(0); y < oaq.params.gridSize; y++ {
		for x := uint8(0); x < oaq.params.gridSize; x++ {
			var vcand [3]float32
			vcand[0] = cvx + float32(x)*cs - half
			vcand[1] = 0
			vcand[2] = cvz + float32(y)*cs - half

			if vcand[0]*vcand[0]+vcand[2]*vcand[2] > (vmax+cs/2)*(vmax+cs/2) {
				continue
			}

			penalty := oaq.processSample(vcand[:], cs, pos, rad, vel, dvel, minPenalty, debug)
			ns++
			if penalty < minPenalty {
				minPenalty = penalty
				copy(nvel, vcand[:3])
			}
		}
	}

	return ns
}

// vector normalization that ignores the y-component.
func normalize2D(v d3.Vec3) {
	d := math32.Sqrt(v[0]*v[0] + v[2]*v[2])
	if d == 0 {
		return
	}
	d = 1.0 / d
	v[0] *= d
	v[2] *= d
}

// vector normalization that ignores the y-component.
func rotate2D(dest, v d3.Vec3, ang float32) {
	c := math32.Cos(ang)
	s := math32.Sin(ang)
	dest[0] = v[0]*c - v[2]*s
	dest[2] = v[0]*s + v[2]*c
	dest[1] = v[1]
}

func (oaq *ObstacleAvoidanceQuery) sampleVelocityAdaptive(pos d3.Vec3, rad, vmax float32, vel, dvel, nvel d3.Vec3, params *ObstacleAvoidanceParams, debug *ObstacleAvoidanceDebugData) int {
	oaq.prepare(pos, dvel)

	oaq.params = *params
	oaq.invHorizTime = 1.0 / oaq.params.horizTime
	oaq.vmax = vmax
	oaq.invVmax = math.MaxFloat32
	if vmax > 0 {
		oaq.invVmax = 1.0 / vmax
	}

	nvel.SetXYZ(0, 0, 0)

	if debug != nil {
		debug.reset()
	}

	// Build sampling pattern aligned to desired velocity.
	var (
		pat    [(maxPatternDivs*maxPatternRings + 1) * 2]float32
		npat   int = 0
		ndivs      = int(oaq.params.adaptiveDivs)
		nrings     = int(oaq.params.adaptiveRings)
		depth      = int(oaq.params.adaptiveDepth)

		nd = detour.Int32Clamp(int32(ndivs), 1, maxPatternDivs)
		nr = detour.Int32Clamp(int32(nrings), 1, maxPatternRings)
		da = (1.0 / float32(nd)) * DT_PI * 2
		ca = math32.Cos(da)
		sa = math32.Sin(da)
	)

	// desired direction
	var ddir [6]float32
	d3.Vec3Copy(ddir[:], dvel)
	normalize2D(ddir[:])
	rotate2D(ddir[3:], ddir[:], da*0.5) // rotated by da/2

	// Always add sample at zero
	pat[npat*2+0] = 0
	pat[npat*2+1] = 0
	npat++

	// TODO: THIS PART HAS TO BE CHECKED
	for j := int32(0); j < nr; j++ {
		r := float32(nr-j) / float32(nr)
		pat[npat*2+0] = ddir[(j%2)*3] * r
		pat[npat*2+1] = ddir[(j%2)*3+2] * r
		last1 := pat[npat*2:]
		last2 := last1
		npat++

		for i := int32(1); i < nd-1; i += 2 {
			// get next point on the "right" (rotate CW)
			pat[npat*2+0] = last1[0]*ca + last1[1]*sa
			pat[npat*2+1] = -last1[0]*sa + last1[1]*ca
			// get next point on the "left" (rotate CCW)
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca

			last1 = pat[npat*2:]
			last2 = last1[2:]
			npat += 2
		}

		if (nd & 1) == 0 {
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca
			npat++
		}
	}

	// Start sampling.
	cr := vmax * (1.0 - oaq.params.velBias)
	res := d3.NewVec3XYZ(dvel[0]*oaq.params.velBias, 0, dvel[2]*oaq.params.velBias)
	var ns int = 0

	for k := 0; k < depth; k++ {
		var minPenalty float32 = math.MaxFloat32
		bvel := d3.NewVec3()

		for i := 0; i < npat; i++ {
			vcand := d3.NewVec3XYZ(res[0]+pat[i*2+0]*cr, 0, res[2]+pat[i*2+1]*cr)
			if math32.Sqr(vcand[0])+math32.Sqr(vcand[2]) > math32.Sqr(vmax+0.001) {
				continue
			}

			penalty := oaq.processSample(vcand, cr/10, pos, rad, vel, dvel, minPenalty, debug)
			ns++
			if penalty < minPenalty {
				minPenalty = penalty
				d3.Vec3Copy(bvel, vcand)
			}
		}

		d3.Vec3Copy(res, bvel)

		cr *= 0.5
	}

	d3.Vec3Copy(nvel, res)

	return ns
}

func (oaq *ObstacleAvoidanceQuery) getObstacleCircleCount() int {
	return oaq.ncircles
}

func (oaq *ObstacleAvoidanceQuery) getObstacleCircle(i int) *ObstacleCircle {
	return &oaq.circles[i]
}

func (oaq *ObstacleAvoidanceQuery) getObstacleSegmentCount() int {
	return oaq.nsegments
}

func (oaq *ObstacleAvoidanceQuery) getObstacleSegment(i int) *ObstacleSegment {
	return &oaq.segments[i]
}

func (oaq *ObstacleAvoidanceQuery) prepare(pos, dvel d3.Vec3) {

}

// Calculate the collision penalty for a given velocity vector
//
//  Arguments:
//   vcand      sampled velocity
//   dvel       desired velocity
//   minPenalty threshold penalty for early out
func (oaq *ObstacleAvoidanceQuery) processSample(vcand d3.Vec3, cs float32,
	pos d3.Vec3, rad float32,
	vel, dvel d3.Vec3,
	minPenalty float32,
	debug *ObstacleAvoidanceDebugData) float32 {

	return 0
}

const DT_PI = 3.14159265

func sweepCircleCircle(c0 d3.Vec3, r0 float32, v d3.Vec3,
	c1 d3.Vec3, r1 float32) (tmin, tmax float32, moving bool) {

	const EPS = 0.0001
	s := c1.Sub(c0)
	r := r0 + r1
	c := s.Dot2D(s) - r*r
	a := v.Dot2D(v)
	if a < EPS {
		return tmin, tmax, false // not moving
	}

	// Overlap, calc time to exit.
	b := v.Dot2D(s)
	d := b*b - a*c
	if d < 0.0 {
		return tmin, tmax, false // no intersection.
	}
	a = 1.0 / a
	rd := math32.Sqrt(d)
	tmin = (b - rd) * a
	tmax = (b + rd) * a
	return tmin, tmax, true
}

func isectRaySeg(ap, u, bp, bq d3.Vec3) (isect bool, t float32) {
	v := bq.Sub(bp)
	w := ap.Sub(bp)

	d := u.Perp2D(v)
	if math32.Abs(d) < 1e-6 {
		return false, t
	}
	d = 1.0 / d
	t = v.Perp2D(w) * d
	if t < 0 || t > 1 {
		return false, t
	}
	s := u.Perp2D(w) * d
	if s < 0 || s > 1 {
		return false, t
	}
	return true, t
}
