package posepreset

import "errors"

var (
	errAtLeastTwoArms               = errors.New("must declare at least two arms")
	errNegativeMaxJointVel          = errors.New("max_joint_vel_degs_per_sec must be non-negative")
	errNegativeWaypointSpacing      = errors.New("waypoint_spacing_ms must be non-negative")
	errNegativeLinearTolerance      = errors.New("linear_tolerance_mm must be non-negative")
	errNegativeOrientationTolerance = errors.New("orientation_tolerance_degs must be non-negative")
	errNoSavedPose                  = errors.New("no saved poses; teach poses first via position 1 (update config)")
)
