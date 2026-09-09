package posepreset

import (
	"context"
	"fmt"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/coord"
	"github.com/viam-labs/multi-arm-motion/internal/trajgen"
)

func (s *service) recallBarrier(ctx context.Context) error {
	if len(s.cfg.Poses) == 0 {
		return errNoSavedPose
	}

	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return fmt.Errorf("get framesystem: %w", err)
	}

	currentJoints := make(map[string][]referenceframe.Input, len(s.armOrder))
	currentInputs := referenceframe.FrameSystemInputs{}
	for _, name := range s.armOrder {
		j, err := s.arms[name].JointPositions(ctx, nil)
		if err != nil {
			return fmt.Errorf("arm %q: joints: %w", name, err)
		}
		currentJoints[name] = j
		currentInputs[name] = j
	}

	targetJoints := make(map[string][]referenceframe.Input, len(s.armOrder))
	for _, name := range s.armOrder {
		saved, ok := s.cfg.Poses[name]
		if !ok {
			return fmt.Errorf("arm %q: no saved pose", name)
		}
		tj, err := coord.PlanTargetJoints(ctx, s.logger, fs, name, currentInputs, saved.ToPose())
		if err != nil {
			return fmt.Errorf("arm %q: plan: %w", name, err)
		}
		targetJoints[name] = tj
	}

	var groupMaxDelta float64
	for _, name := range s.armOrder {
		if d := trajgen.MaxJointDelta(currentJoints[name], targetJoints[name]); d > groupMaxDelta {
			groupMaxDelta = d
		}
	}
	if groupMaxDelta == 0 {
		return nil
	}
	duration := trajgen.DurationForMaxDelta(groupMaxDelta, s.cfg.maxJointVelRadPerSec())
	s.logger.Infof("pose-preset recall (barrier): group max delta %.4f rad, shared duration %v", groupMaxDelta, duration)

	ops := make([]barrier.Op, 0, len(s.armOrder))
	for _, name := range s.armOrder {
		traj, err := trajgen.GenerateWithDuration(
			currentJoints[name],
			targetJoints[name],
			duration,
			s.cfg.waypointSpacing(),
		)
		if err != nil {
			return fmt.Errorf("arm %q: trajgen: %w", name, err)
		}
		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: traj})
	}

	return barrier.Fire(ctx, ops)
}
