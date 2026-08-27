package coord

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"time"
)

// PlanConstrainedTrajectoryToJoints plans a Cartesian-linear path from startInputs to a joint-space
// target for armName, subject to the given tolerances, and returns the resulting trajectory as
// arm.TrajectoryPoint values evenly spaced across totalDuration. If planning fails (which
// LinearConstraint frequently does for large joint deltas under cbirrt), the error propagates.
// PlanConstrainedTrajectoryToJoints plans a Cartesian-linear path for one arm.
// startInputs and targetInputs must contain entries for every input-enabled frame in the frame
// system — armplanning rejects a joint-configuration goal that omits any of them. Callers that
// only want one arm to move should set that arm's target and leave the others at their current
// values.
func PlanConstrainedTrajectoryToJoints(
	ctx context.Context,
	logger logging.Logger,
	fs *referenceframe.FrameSystem,
	armName string,
	startInputs referenceframe.FrameSystemInputs,
	targetInputs referenceframe.FrameSystemInputs,
	lineToleranceMm float64,
	orientationToleranceDegs float64,
	totalDuration time.Duration,
) ([]arm.TrajectoryPoint, error) {
	planOpts, err := armplanning.NewPlannerOptionsFromExtra(map[string]interface{}{"timeout": 30.0})
	if err != nil {
		return nil, fmt.Errorf("planner options: %w", err)
	}
	constraints := motionplan.NewConstraints(
		[]motionplan.LinearConstraint{{
			LineToleranceMm:          lineToleranceMm,
			OrientationToleranceDegs: orientationToleranceDegs,
		}},
		nil, nil, nil,
	)
	plan, _, err := armplanning.PlanMotion(ctx, logger, &armplanning.PlanRequest{
		FrameSystem:    fs,
		Goals:          []*armplanning.PlanState{armplanning.NewPlanState(nil, targetInputs)},
		StartState:     armplanning.NewPlanState(nil, startInputs),
		Constraints:    constraints,
		PlannerOptions: planOpts,
	})
	if err != nil {
		return nil, fmt.Errorf("plan: %w", err)
	}
	steps := plan.Trajectory()
	if len(steps) == 0 {
		return nil, fmt.Errorf("empty plan")
	}

	pts := make([]arm.TrajectoryPoint, 0, len(steps))
	denom := len(steps) - 1
	if denom == 0 {
		denom = 1
	}
	for i, step := range steps {
		joints, ok := step[armName]
		if !ok {
			return nil, fmt.Errorf("step %d missing %q", i, armName)
		}
		t := time.Duration(int64(totalDuration) * int64(i) / int64(denom))
		pts = append(pts, arm.TrajectoryPoint{Time: t, Positions: joints})
	}
	return pts, nil
}
