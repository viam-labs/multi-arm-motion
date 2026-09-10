package coordinatedarm

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
	"github.com/viam-labs/multi-arm-motion/internal/coord"
	"github.com/viam-labs/multi-arm-motion/internal/trajgen"
)

// MoveThroughJointPositions executes the given primary joint trajectory while
// coordinating the follower so their end poses stay in the rigid offset relationship.
// Uses barrier-style parallel execution: matched shared duration, streamed to both arms.
func (s *service) MoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	_ *arm.MoveOptions,
	_ map[string]interface{},
) error {
	if len(positions) == 0 {
		return errNoPositions
	}

	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return fmt.Errorf("get framesystem: %w", err)
	}

	primaryCurrent, err := s.primary.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("primary joints: %w", err)
	}
	followerCurrent, err := s.follower.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("follower joints: %w", err)
	}

	primaryLast := positions[len(positions)-1]

	// FK primary at its trajectory endpoint via the framesystem, then compose
	// with the configured rigid offset to derive the follower's world target.
	inputsAtPrimaryEnd := referenceframe.FrameSystemInputs{
		s.cfg.Primary:  primaryLast,
		s.cfg.Follower: followerCurrent,
	}
	linear := inputsAtPrimaryEnd.ToLinearInputs()
	transformed, err := fs.Transform(linear,
		referenceframe.NewPoseInFrame(s.cfg.Primary, spatialmath.NewZeroPose()),
		referenceframe.World)
	if err != nil {
		return fmt.Errorf("fk primary at endpoint: %w", err)
	}
	primaryEndWorld := transformed.(*referenceframe.PoseInFrame).Pose()
	followerTargetWorld := spatialmath.Compose(primaryEndWorld, s.cfg.FollowerOffset.ToPose())

	// Plan follower to derived target using the same constrained-trajectory helper
	// the pose-preset barrier mode uses. Start state pins primary at its endpoint so
	// the planner doesn't consider primary's current joints (primary will get there
	// on its own via its trajectory).
	startInputs := referenceframe.FrameSystemInputs{
		s.cfg.Primary:  primaryLast,
		s.cfg.Follower: followerCurrent,
	}
	followerSteps, err := coord.PlanConstrainedTrajectory(
		ctx, s.logger, fs, s.cfg.Follower, startInputs, followerTargetWorld,
		s.cfg.linearToleranceMm(), s.cfg.orientationToleranceDegs(),
	)
	if err != nil {
		return fmt.Errorf("plan follower: %w", err)
	}

	// Shared duration from whichever arm has the larger joint delta so neither
	// gets rushed. Primary's delta is start → each waypoint; follower's is start
	// → planned endpoint.
	primaryMaxDelta := maxDeltaThroughTrajectory(primaryCurrent, positions)
	followerMaxDelta := trajgen.MaxJointDelta(followerCurrent, followerSteps[len(followerSteps)-1])
	groupMaxDelta := primaryMaxDelta
	if followerMaxDelta > groupMaxDelta {
		groupMaxDelta = followerMaxDelta
	}
	if groupMaxDelta == 0 {
		return nil
	}
	duration := trajgen.DurationForMaxDelta(groupMaxDelta, s.cfg.maxJointVelRadPerSec())
	s.logger.Infof("coordinated-arm move: primary %d waypoints, follower %d steps, shared duration %v",
		len(positions), len(followerSteps), duration)

	// Time-space both. Primary keeps its own waypoints; follower uses its planned steps.
	primarySteps := make([][]referenceframe.Input, len(positions))
	copy(primarySteps, positions)
	primaryTraj, err := trajgen.TimeSpaceSteps(primarySteps, duration)
	if err != nil {
		return fmt.Errorf("primary trajgen: %w", err)
	}
	followerTraj, err := trajgen.TimeSpaceSteps(followerSteps, duration)
	if err != nil {
		return fmt.Errorf("follower trajgen: %w", err)
	}

	return barrier.Fire(ctx, []barrier.Op{
		{Arm: s.primary, Trajectory: primaryTraj},
		{Arm: s.follower, Trajectory: followerTraj},
	})
}

func maxDeltaThroughTrajectory(start []referenceframe.Input, steps [][]referenceframe.Input) float64 {
	max := 0.0
	prev := start
	for _, step := range steps {
		if d := trajgen.MaxJointDelta(prev, step); d > max {
			max = d
		}
		prev = step
	}
	return max
}

// The remaining arm interface methods are not used by the sanding module's main flow
// (batched executor → plan.Execute → arm.MoveThroughJointPositions). Returning an
// explicit error is safer than a silent single-arm fallback that would break the
// rigid coupling.

func (s *service) MoveToPosition(_ context.Context, _ spatialmath.Pose, _ map[string]interface{}) error {
	return errUnsupported
}

func (s *service) MoveToJointPositions(_ context.Context, _ []referenceframe.Input, _ map[string]interface{}) error {
	return errUnsupported
}

func (s *service) MoveThroughJointPositionsStreamed(
	_ context.Context,
	_ <-chan []arm.TrajectoryPoint,
	_ chan<- arm.Response,
	_ map[string]interface{},
) error {
	return errUnsupported
}

func (s *service) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}
