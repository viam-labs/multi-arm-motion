package group

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/coord"
	"github.com/viam-labs/multi-arm-motion/internal/trajgen"
)

// Cap per-joint motion per jog; a larger planned delta means IK found a
// branch-flipped solution near a singularity and shouldn't be executed.
const maxJointDeltaPerJog = math.Pi / 2

type JogDelta struct {
	X, Y, Z float64
}

func (s *service) Jog(ctx context.Context, delta JogDelta) error {
	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return status.Errorf(codes.Internal, "get framesystem: %v", err)
	}

	currentInputs := referenceframe.FrameSystemInputs{}
	currentJoints := make(map[string][]referenceframe.Input, len(s.armOrder))
	for _, name := range s.armOrder {
		j, err := s.arms[name].JointPositions(ctx, nil)
		if err != nil {
			return status.Errorf(codes.Internal, "arm %q: joints: %v", name, err)
		}
		currentJoints[name] = j
		currentInputs[name] = j
	}

	deltaVec := r3.Vector{X: delta.X, Y: delta.Y, Z: delta.Z}
	plans := make(map[string][][]referenceframe.Input, len(s.armOrder))
	for _, name := range s.armOrder {
		currentPose, err := s.fsService.TransformPose(ctx,
			referenceframe.NewPoseInFrame(name, spatialmath.NewZeroPose()),
			referenceframe.World, nil)
		if err != nil {
			return status.Errorf(codes.Internal, "arm %q: current world pose: %v", name, err)
		}
		targetPose := spatialmath.NewPose(
			currentPose.Pose().Point().Add(deltaVec),
			currentPose.Pose().Orientation(),
		)
		steps, err := coord.PlanConstrainedTrajectory(
			ctx, s.logger, fs, name, currentInputs, targetPose,
			s.cfg.linearToleranceMm(), s.cfg.orientationToleranceDegs(),
		)
		if err != nil {
			return status.Errorf(codes.FailedPrecondition, "arm %q: plan: %v", name, err)
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
	if groupMaxDelta > maxJointDeltaPerJog {
		s.logger.Warnf("jog rejected: planned joint delta %.4f rad exceeds max %.4f rad (near singularity or IK branch flip)",
			groupMaxDelta, maxJointDeltaPerJog)
		return status.Errorf(codes.FailedPrecondition,
			"planned joint delta %.4f rad exceeds max %.4f rad (near singularity or IK branch flip); nudge arm off-singularity or use a smaller jog",
			groupMaxDelta, maxJointDeltaPerJog)
	}
	duration := trajgen.DurationForMaxDelta(groupMaxDelta, s.cfg.maxJointVelRadPerSec())
	s.logger.Infof("jog (constrained %.2fmm/%.2fdeg): group max delta %.4f rad, shared duration %v",
		s.cfg.linearToleranceMm(), s.cfg.orientationToleranceDegs(), groupMaxDelta, duration)

	ops := make([]barrier.Op, 0, len(s.armOrder))
	for _, name := range s.armOrder {
		traj, err := trajgen.TimeSpaceSteps(plans[name], duration)
		if err != nil {
			return status.Errorf(codes.Internal, "arm %q: %v", name, err)
		}
		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: traj})
	}

	if err := barrier.Fire(ctx, ops); err != nil {
		if errors.Is(err, barrier.ErrFireTimeout) {
			return status.Errorf(codes.DeadlineExceeded, "jog execution timed out: %v", err)
		}
		return status.Errorf(codes.Internal, "jog execution: %v", err)
	}
	return nil
}

func parseJog(raw any) (JogDelta, error) {
	m, ok := raw.(map[string]any)
	if !ok {
		return JogDelta{}, errMissingJogDelta
	}
	deltaRaw, ok := m["delta"].(map[string]any)
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
