package preset

import (
	"context"
	"fmt"

	"go.viam.com/rdk/referenceframe"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/trajgen"
)

func (s *service) recall(ctx context.Context) error {
	if len(s.cfg.Joints) == 0 {
		return errNoSavedPose
	}

	currentJoints := make(map[string][]referenceframe.Input, len(s.armOrder))
	targetJoints := make(map[string][]referenceframe.Input, len(s.armOrder))
	for _, name := range s.armOrder {
		j, err := s.arms[name].JointPositions(ctx, nil)
		if err != nil {
			return fmt.Errorf("arm %q: joints: %w", name, err)
		}
		currentJoints[name] = j

		saved, ok := s.cfg.Joints[name]
		if !ok {
			return fmt.Errorf("arm %q: no saved joints", name)
		}
		savedInputs := make([]referenceframe.Input, len(saved))
		for i, v := range saved {
			savedInputs[i] = v
		}
		targetJoints[name] = savedInputs
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
	s.logger.Infof("preset recall: group max delta %.4f rad, shared duration %v", groupMaxDelta, duration)

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
