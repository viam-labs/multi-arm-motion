package trajgen

import (
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

func Generate(
	from, to []referenceframe.Input,
	maxJointVelRadPerSec float64,
	waypointSpacing time.Duration,
) ([]arm.TrajectoryPoint, error) {
	if len(from) != len(to) {
		return nil, errLengthMismatch
	}
	if maxJointVelRadPerSec <= 0 {
		return nil, errPositiveVelocityRequired
	}
	maxDelta := MaxJointDelta(from, to)
	if maxDelta == 0 {
		return nil, errFromEqualsTo
	}
	duration := time.Duration(math.Pi * maxDelta / (2 * maxJointVelRadPerSec) * float64(time.Second))
	return GenerateWithDuration(from, to, duration, waypointSpacing)
}

func GenerateWithDuration(
	from, to []referenceframe.Input,
	duration time.Duration,
	waypointSpacing time.Duration,
) ([]arm.TrajectoryPoint, error) {
	if len(from) != len(to) {
		return nil, errLengthMismatch
	}
	if duration <= 0 {
		return nil, errPositiveDurationRequired
	}
	if waypointSpacing <= 0 {
		return nil, errPositiveSpacingRequired
	}

	numSteps := int(duration / waypointSpacing)
	if numSteps < 1 {
		numSteps = 1
	}

	traj := make([]arm.TrajectoryPoint, 0, numSteps+1)
	traj = append(traj, arm.TrajectoryPoint{Time: 0, Positions: cloneJoints(from)})
	for i := 1; i <= numSteps; i++ {
		frac := float64(i) / float64(numSteps)
		ease := 0.5 * (1 - math.Cos(math.Pi*frac))
		t := time.Duration(float64(duration) * frac)
		traj = append(traj, arm.TrajectoryPoint{Time: t, Positions: interpolate(from, to, ease)})
	}
	return traj, nil
}

func MaxJointDelta(from, to []referenceframe.Input) float64 {
	n := len(from)
	if len(to) < n {
		n = len(to)
	}
	m := 0.0
	for i := 0; i < n; i++ {
		d := math.Abs(to[i] - from[i])
		if d > m {
			m = d
		}
	}
	return m
}

func DurationForMaxDelta(maxDelta float64, maxJointVelRadPerSec float64) time.Duration {
	if maxJointVelRadPerSec <= 0 || maxDelta <= 0 {
		return 0
	}
	return time.Duration(math.Pi * maxDelta / (2 * maxJointVelRadPerSec) * float64(time.Second))
}

func interpolate(from, to []referenceframe.Input, frac float64) []referenceframe.Input {
	out := make([]referenceframe.Input, len(from))
	for i := range from {
		out[i] = from[i] + (to[i]-from[i])*frac
	}
	return out
}

func cloneJoints(j []referenceframe.Input) []referenceframe.Input {
	out := make([]referenceframe.Input, len(j))
	copy(out, j)
	return out
}
