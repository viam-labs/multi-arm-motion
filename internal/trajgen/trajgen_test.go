package trajgen

import (
	"testing"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

func TestGenerateHappyPath(t *testing.T) {
	from := []referenceframe.Input{0, 0, 0, 0, 0, 0}
	to := []referenceframe.Input{1, 0.5, 0, 0, 0, 0}

	traj, err := Generate(from, to, 0.5, 20*time.Millisecond)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(traj), test.ShouldBeGreaterThan, 10)

	test.That(t, traj[0].Time, test.ShouldEqual, time.Duration(0))
	test.That(t, traj[0].Positions[0], test.ShouldAlmostEqual, from[0], 1e-9)
	test.That(t, traj[0].Positions[1], test.ShouldAlmostEqual, from[1], 1e-9)

	last := traj[len(traj)-1]
	test.That(t, last.Positions[0], test.ShouldAlmostEqual, to[0], 1e-9)
	test.That(t, last.Positions[1], test.ShouldAlmostEqual, to[1], 1e-9)

	for i := 1; i < len(traj); i++ {
		test.That(t, traj[i].Time > traj[i-1].Time, test.ShouldBeTrue)
	}
}

func TestGeneratePeakVelocityMatchesMaxJointDelta(t *testing.T) {
	from := []referenceframe.Input{0, 0}
	to := []referenceframe.Input{2, 0.1}
	maxVel := 1.0

	traj, err := Generate(from, to, maxVel, 20*time.Millisecond)
	test.That(t, err, test.ShouldBeNil)

	var peakJ0 float64
	for i := 1; i < len(traj); i++ {
		dt := (traj[i].Time - traj[i-1].Time).Seconds()
		dv := (traj[i].Positions[0] - traj[i-1].Positions[0]) / dt
		if dv > peakJ0 {
			peakJ0 = dv
		}
	}
	test.That(t, peakJ0, test.ShouldAlmostEqual, maxVel, 0.05)
}

func TestGenerateRejectsLengthMismatch(t *testing.T) {
	_, err := Generate(
		[]referenceframe.Input{0, 0},
		[]referenceframe.Input{0, 0, 0},
		1.0, 20*time.Millisecond,
	)
	test.That(t, err, test.ShouldEqual, errLengthMismatch)
}

func TestGenerateRejectsNonPositiveVelocity(t *testing.T) {
	_, err := Generate(
		[]referenceframe.Input{0}, []referenceframe.Input{1},
		0, 20*time.Millisecond,
	)
	test.That(t, err, test.ShouldEqual, errPositiveVelocityRequired)

	_, err = Generate(
		[]referenceframe.Input{0}, []referenceframe.Input{1},
		-1, 20*time.Millisecond,
	)
	test.That(t, err, test.ShouldEqual, errPositiveVelocityRequired)
}

func TestGenerateRejectsNonPositiveSpacing(t *testing.T) {
	_, err := Generate(
		[]referenceframe.Input{0}, []referenceframe.Input{1},
		1.0, 0,
	)
	test.That(t, err, test.ShouldEqual, errPositiveSpacingRequired)
}

func TestGenerateRejectsFromEqualsTo(t *testing.T) {
	_, err := Generate(
		[]referenceframe.Input{0.5, 1.0}, []referenceframe.Input{0.5, 1.0},
		1.0, 20*time.Millisecond,
	)
	test.That(t, err, test.ShouldEqual, errFromEqualsTo)
}

func TestGenerateDeterministic(t *testing.T) {
	from := []referenceframe.Input{0, 0, 0}
	to := []referenceframe.Input{1, 0.5, 0.25}

	a, err := Generate(from, to, 0.5, 20*time.Millisecond)
	test.That(t, err, test.ShouldBeNil)
	b, err := Generate(from, to, 0.5, 20*time.Millisecond)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(a), test.ShouldEqual, len(b))
	for i := range a {
		test.That(t, a[i].Time, test.ShouldEqual, b[i].Time)
		for j := range a[i].Positions {
			test.That(t, a[i].Positions[j], test.ShouldEqual, b[i].Positions[j])
		}
	}
}
