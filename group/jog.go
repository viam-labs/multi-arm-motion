package group

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/coord"
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

		traj, err := coord.PlanArmToWorldPose(
			ctx, s.logger, fs, name, currentInputs, currentJoints[name],
			targetPose, s.cfg.maxJointVelRadPerSec(), s.cfg.waypointSpacing(),
		)
		if err != nil {
			return fmt.Errorf("arm %q: %w", name, err)
		}
		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: traj})
	}

	return barrier.Fire(ctx, ops)
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
