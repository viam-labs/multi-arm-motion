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

func PlanTargetJoints(
	ctx context.Context,
	logger logging.Logger,
	fs *referenceframe.FrameSystem,
	armName string,
	startInputs referenceframe.FrameSystemInputs,
	targetWorld spatialmath.Pose,
) ([]referenceframe.Input, error) {
	planOpts, err := armplanning.NewPlannerOptionsFromExtra(map[string]interface{}{"timeout": 30.0})
	if err != nil {
		return nil, fmt.Errorf("planner options: %w", err)
	}
	constraints := motionplan.NewConstraints(
		[]motionplan.LinearConstraint{{LineToleranceMm: 2.0, OrientationToleranceDegs: 2.0}},
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
	if len(steps) < 1 {
		return nil, fmt.Errorf("empty plan")
	}
	joints, ok := steps[len(steps)-1][armName]
	if !ok {
		return nil, fmt.Errorf("plan missing %q", armName)
	}
	return joints, nil
}
