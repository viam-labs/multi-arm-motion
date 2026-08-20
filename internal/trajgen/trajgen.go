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
	if waypointSpacing <= 0 {
		return nil, errPositiveSpacingRequired
	}

	maxDelta := 0.0
	for i := range from {
		d := math.Abs(to[i] - from[i])
		if d > maxDelta {
			maxDelta = d
		}
	}
	if maxDelta == 0 {
		return nil, errFromEqualsTo
	}

	duration := time.Duration(math.Pi * maxDelta / (2 * maxJointVelRadPerSec) * float64(time.Second))
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
