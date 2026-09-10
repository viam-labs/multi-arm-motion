package coord

import (
	"context"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Returns the planner's full step sequence (not just the endpoint) so callers can
// execute the planned Cartesian-linear path directly instead of interpolating endpoints.
func PlanConstrainedTrajectory(
	ctx context.Context,
	logger logging.Logger,
	fs *referenceframe.FrameSystem,
	armName string,
	startInputs referenceframe.FrameSystemInputs,
	targetWorld spatialmath.Pose,
	lineToleranceMm float64,
	orientationToleranceDegs float64,
) ([][]referenceframe.Input, error) {
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
		FrameSystem: fs,
		Goals: []*armplanning.PlanState{armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				armName: referenceframe.NewPoseInFrame(referenceframe.World, targetWorld),
			},
			nil,
		)},
		StartState:     armplanning.NewPlanState(nil, startInputs),
		Constraints:    constraints,
		PlannerOptions: planOpts,
	})
	if err != nil {
		return nil, fmt.Errorf("plan: %w", err)
	}
	steps := plan.Trajectory()
	if len(steps) < 2 {
		return nil, fmt.Errorf("plan has %d steps; need at least 2 to execute", len(steps))
	}
	out := make([][]referenceframe.Input, 0, len(steps))
	for i, step := range steps {
		joints, ok := step[armName]
		if !ok {
			return nil, fmt.Errorf("plan step %d missing %q", i, armName)
		}
		out = append(out, joints)
	}
	return out, nil
}
