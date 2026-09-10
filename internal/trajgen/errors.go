package trajgen

import "errors"

var (
	errLengthMismatch           = errors.New("from and to must have the same length")
	errPositiveVelocityRequired = errors.New("max joint velocity must be positive")
	errPositiveSpacingRequired  = errors.New("waypoint spacing must be positive")
	errPositiveDurationRequired = errors.New("duration must be positive")
	errFromEqualsTo             = errors.New("from and to are identical; no motion required")
	errAtLeastTwoSteps          = errors.New("need at least two steps to time-space a trajectory")
)
