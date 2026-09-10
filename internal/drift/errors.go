package drift

import "errors"

var (
	errNoArms         = errors.New("need at least one arm to measure drift")
	errArmMissingFromTargets = errors.New("no saved target pose for arm")
)
