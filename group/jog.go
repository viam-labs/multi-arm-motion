package group

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/trajgen"
)

type JogDelta struct {
	X, Y, Z float64
}

func (s *service) Jog(ctx context.Context, delta JogDelta) error {
	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return fmt.Errorf("get framesystem: %w", err)
	}

	currentInputs := referenceframe.FrameSystemInputs{}
	currentJoints := make(map[string][]referenceframe.Input, len(s.armOrder))
	for _, name := range s.armOrder {
		j, err := s.arms[name].JointPositions(ctx, nil)
		if err != nil {
			return fmt.Errorf("arm %q: joints: %w", name, err)
		}
		currentJoints[name] = j
		currentInputs[name] = j
	}

	deltaVec := r3.Vector{X: delta.X, Y: delta.Y, Z: delta.Z}
	ops := make([]barrier.Op, 0, len(s.armOrder))
	for _, name := range s.armOrder {
		currentPose, err := s.fsService.TransformPose(ctx,
			referenceframe.NewPoseInFrame(name, spatialmath.NewZeroPose()),
			referenceframe.World, nil)
		if err != nil {
			return fmt.Errorf("arm %q: current world pose: %w", name, err)
		}
		targetPose := spatialmath.NewPose(
			currentPose.Pose().Point().Add(deltaVec),
			currentPose.Pose().Orientation(),
		)

		targetJoints, err := s.planToWorldPose(ctx, fs, name, currentInputs, targetPose)
		if err != nil {
			return fmt.Errorf("arm %q: plan: %w", name, err)
		}

		traj, err := trajgen.Generate(
			currentJoints[name],
			targetJoints,
			s.cfg.maxJointVelRadPerSec(),
			s.cfg.waypointSpacing(),
		)
		if err != nil {
			return fmt.Errorf("arm %q: trajgen: %w", name, err)
		}

		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: traj})
	}

	return barrier.Fire(ctx, ops)
}

func (s *service) planToWorldPose(
	ctx context.Context,
	fs *referenceframe.FrameSystem,
	armName string,
	startInputs referenceframe.FrameSystemInputs,
	target spatialmath.Pose,
) ([]referenceframe.Input, error) {
	planOpts, err := armplanning.NewPlannerOptionsFromExtra(map[string]interface{}{"timeout": 30.0})
	if err != nil {
		return nil, fmt.Errorf("planner options: %w", err)
	}
	constraints := motionplan.NewConstraints(
		[]motionplan.LinearConstraint{{LineToleranceMm: 2.0, OrientationToleranceDegs: 2.0}},
		nil, nil, nil,
	)
	plan, _, err := armplanning.PlanMotion(ctx, s.logger, &armplanning.PlanRequest{
		FrameSystem: fs,
		Goals: []*armplanning.PlanState{armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				armName: referenceframe.NewPoseInFrame(referenceframe.World, target),
			},
			nil,
		)},
		StartState:     armplanning.NewPlanState(nil, startInputs),
		Constraints:    constraints,
		PlannerOptions: planOpts,
	})
	if err != nil {
		return nil, err
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

func parseJog(raw interface{}) (JogDelta, error) {
	m, ok := raw.(map[string]interface{})
	if !ok {
		return JogDelta{}, errMissingJogDelta
	}
	deltaRaw, ok := m["delta"].(map[string]interface{})
	if !ok {
		return JogDelta{}, errMissingJogDelta
	}
	getF := func(k string) (float64, error) {
		v, ok := deltaRaw[k]
		if !ok {
			return 0, nil
		}
		f, ok := v.(float64)
		if !ok {
			return 0, fmt.Errorf("delta.%s must be a number", k)
		}
		return f, nil
	}
	x, err := getF("x")
	if err != nil {
		return JogDelta{}, err
	}
	y, err := getF("y")
	if err != nil {
		return JogDelta{}, err
	}
	z, err := getF("z")
	if err != nil {
		return JogDelta{}, err
	}
	return JogDelta{X: x, Y: y, Z: z}, nil
}
