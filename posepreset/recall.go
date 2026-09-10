package posepreset

import (
	"context"
	"fmt"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/coord"
	"github.com/viam-labs/multi-arm-motion/internal/drift"
	"github.com/viam-labs/multi-arm-motion/internal/trajgen"
)

func (s *service) recallBarrier(ctx context.Context) error {
	targets, err := s.targetsFromPoses()
	if err != nil {
		return err
	}
	return s.executeConstrainedBarrier(ctx, targets, modeBarrier)
}

func (s *service) recallPrimaryFollower(ctx context.Context) error {
	targets, err := s.targetsFromPrimaryFollower()
	if err != nil {
		return err
	}
	return s.executeConstrainedBarrier(ctx, targets, modePrimaryFollower)
}

func (s *service) targetsFromPoses() (map[string]spatialmath.Pose, error) {
	if len(s.cfg.Poses) == 0 {
		return nil, errNoSavedPose
	}
	targets := make(map[string]spatialmath.Pose, len(s.armOrder))
	for _, name := range s.armOrder {
		saved, ok := s.cfg.Poses[name]
		if !ok {
			return nil, fmt.Errorf("arm %q: no saved pose", name)
		}
		targets[name] = saved.ToPose()
	}
	return targets, nil
}

func (s *service) targetsFromPrimaryFollower() (map[string]spatialmath.Pose, error) {
	primary := s.cfg.Primary
	savedPrimary, ok := s.cfg.Poses[primary]
	if !ok {
		return nil, fmt.Errorf("primary %q: no saved pose", primary)
	}
	primaryPose := savedPrimary.ToPose()
	targets := map[string]spatialmath.Pose{primary: primaryPose}
	for _, name := range s.armOrder {
		if name == primary {
			continue
		}
		offset, ok := s.cfg.FollowerOffsets[name]
		if !ok {
			return nil, fmt.Errorf("follower %q: no offset in config", name)
		}
		targets[name] = spatialmath.Compose(primaryPose, offset.ToPose())
	}
	return targets, nil
}

func (s *service) targetsForMode(mode string) (map[string]spatialmath.Pose, error) {
	switch mode {
	case modeBarrier:
		return s.targetsFromPoses()
	case modePrimaryFollower:
		return s.targetsFromPrimaryFollower()
	default:
		return nil, fmt.Errorf("unsupported mode %q", mode)
	}
}

func (s *service) executeConstrainedBarrier(ctx context.Context, targets map[string]spatialmath.Pose, mode string) error {
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

	plans := make(map[string][][]referenceframe.Input, len(s.armOrder))
	for _, name := range s.armOrder {
		target, ok := targets[name]
		if !ok {
			return fmt.Errorf("arm %q: no target pose", name)
		}
		steps, err := coord.PlanConstrainedTrajectory(
			ctx, s.logger, fs, name, currentInputs, target,
			s.cfg.linearToleranceMm(), s.cfg.orientationToleranceDegs(),
		)
		if err != nil {
			return fmt.Errorf("arm %q: plan: %w", name, err)
		}
		plans[name] = steps
	}

	var groupMaxDelta float64
	for _, name := range s.armOrder {
		end := plans[name][len(plans[name])-1]
		if d := trajgen.MaxJointDelta(currentJoints[name], end); d > groupMaxDelta {
			groupMaxDelta = d
		}
	}
	if groupMaxDelta == 0 {
		return nil
	}
	duration := trajgen.DurationForMaxDelta(groupMaxDelta, s.cfg.maxJointVelRadPerSec())
	s.logger.Infof("pose-preset recall (%s, constrained %.2fmm/%.2fdeg): group max delta %.4f rad, shared duration %v",
		mode, s.cfg.linearToleranceMm(), s.cfg.orientationToleranceDegs(), groupMaxDelta, duration)

	ops := make([]barrier.Op, 0, len(s.armOrder))
	for _, name := range s.armOrder {
		traj, err := trajgen.TimeSpaceSteps(plans[name], duration)
		if err != nil {
			return fmt.Errorf("arm %q: trajgen: %w", name, err)
		}
		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: traj})
	}

	if err := barrier.Fire(ctx, ops); err != nil {
		return err
	}

	if s.cfg.LogDrift {
		if res, derr := drift.Measure(ctx, drift.FramesystemReader(s.fsService), s.armOrder, targets); derr != nil {
			s.logger.Warnf("drift measurement failed: %v", derr)
		} else {
			res.LogAt(s.logger, s.Named.Name().Name, mode)
		}
	}
	return nil
}

func (s *service) measureDrift(ctx context.Context) (*drift.Result, error) {
	targets, err := s.targetsForMode(s.cfg.mode())
	if err != nil {
		return nil, err
	}
	return drift.Measure(ctx, drift.FramesystemReader(s.fsService), s.armOrder, targets)
}
