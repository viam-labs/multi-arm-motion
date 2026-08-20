package group

import "errors"

var (
	errAtLeastTwoArms          = errors.New("must declare at least two arms")
	errNegativeMaxJointVel     = errors.New("max_joint_vel_degs_per_sec must be non-negative")
	errNegativeWaypointSpacing = errors.New("waypoint_spacing_ms must be non-negative")
	errMissingJogDelta         = errors.New("jog requires a delta object with x, y, z fields")
)
