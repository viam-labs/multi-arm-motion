package coordinatedarm

import "errors"

var (
	errPrimaryRequired              = errors.New("primary arm name is required")
	errFollowerRequired             = errors.New("follower arm name is required")
	errPrimaryEqualsFollower        = errors.New("primary and follower must be different arms")
	errNegativeMaxJointVel          = errors.New("max_joint_vel_degs_per_sec must be non-negative")
	errNegativeLinearTolerance      = errors.New("linear_tolerance_mm must be non-negative")
	errNegativeOrientationTolerance = errors.New("orientation_tolerance_degs must be non-negative")
	errUnsupported                  = errors.New("not supported by coordinated-arm; use MoveThroughJointPositions instead")
	errNoPositions                  = errors.New("no positions provided")
)
