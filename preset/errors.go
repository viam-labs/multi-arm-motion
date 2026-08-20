package preset

import "errors"

var (
	errAtLeastTwoArms          = errors.New("must declare at least two arms")
	errNegativeMaxJointVel     = errors.New("max_joint_vel_degs_per_sec must be non-negative")
	errNegativeWaypointSpacing = errors.New("waypoint_spacing_ms must be non-negative")
	errNoSavedPose             = errors.New("no saved pose; teach a pose first via position 1 (update config)")
)
