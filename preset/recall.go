package preset

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/coord"
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
		copy(savedInputs, saved)
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

	if s.cfg.hasConstraints() {
		s.logger.Infof("preset recall (constrained): group max delta %.4f rad, shared duration %v, tol=%.2fmm/%.2fdeg",
			groupMaxDelta, duration, s.cfg.LinearToleranceMm, s.cfg.OrientationToleranceDegs)
		return s.recallConstrained(ctx, currentJoints, targetJoints, duration)
	}

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

func (s *service) recallConstrained(
	ctx context.Context,
	currentJoints, targetJoints map[string][]referenceframe.Input,
	duration time.Duration,
) error {
	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return fmt.Errorf("framesystem: %w", err)
	}
	startInputs := make(referenceframe.FrameSystemInputs, len(s.armOrder))
	for _, name := range s.armOrder {
		startInputs[name] = currentJoints[name]
	}

	ops := make([]barrier.Op, 0, len(s.armOrder))
	for _, name := range s.armOrder {
		// Target: this arm goes to its saved joints, others stay at their current joints.
		// armplanning rejects a joint-configuration goal that omits any input-enabled frame.
		targetInputs := make(referenceframe.FrameSystemInputs, len(s.armOrder))
		for _, other := range s.armOrder {
			if other == name {
				targetInputs[other] = targetJoints[other]
			} else {
				targetInputs[other] = currentJoints[other]
			}
		}
		pts, err := coord.PlanConstrainedTrajectoryToJoints(
			ctx, s.logger, fs, name, startInputs, targetInputs,
			s.cfg.LinearToleranceMm, s.cfg.OrientationToleranceDegs, duration,
		)
		if err != nil {
			return fmt.Errorf("arm %q: constrained plan: %w", name, err)
		}
		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: pts})
	}
	return barrier.Fire(ctx, ops)
}

