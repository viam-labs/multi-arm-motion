package coordinatedarm

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

var Model = resource.NewModel("viam", "multi-arm-motion", "coordinated-arm")

const (
	defaultMaxJointVelDegsPerSec    = 30.0
	defaultLinearToleranceMm        = 2.0
	defaultOrientationToleranceDegs = 2.0
)

func init() {
	resource.RegisterComponent(arm.API, Model,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newCoordinatedArm,
		},
	)
}

// Offset is the rigid transform from primary TCP to follower TCP, expressed in primary's TCP frame.
// Uses Viam's canonical flat pose JSON shape ({x, y, z, oX, oY, oZ, theta}).
type Offset struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	OX    float64 `json:"oX"`
	OY    float64 `json:"oY"`
	OZ    float64 `json:"oZ"`
	Theta float64 `json:"theta"`
}

func (o Offset) ToPose() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: o.X, Y: o.Y, Z: o.Z},
		&spatialmath.OrientationVectorDegrees{OX: o.OX, OY: o.OY, OZ: o.OZ, Theta: o.Theta},
	)
}

type Config struct {
	Primary                  string  `json:"primary"`
	Follower                 string  `json:"follower"`
	FollowerOffset           Offset  `json:"follower_offset"`
	MaxJointVelDegsPerSec    float64 `json:"max_joint_vel_degs_per_sec,omitempty"`
	LinearToleranceMm        float64 `json:"linear_tolerance_mm,omitempty"`
	OrientationToleranceDegs float64 `json:"orientation_tolerance_degs,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Primary == "" {
		return nil, nil, resource.NewConfigValidationError(path, errPrimaryRequired)
	}
	if cfg.Follower == "" {
		return nil, nil, resource.NewConfigValidationError(path, errFollowerRequired)
	}
	if cfg.Primary == cfg.Follower {
		return nil, nil, resource.NewConfigValidationError(path, errPrimaryEqualsFollower)
	}
	if cfg.MaxJointVelDegsPerSec < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeMaxJointVel)
	}
	if cfg.LinearToleranceMm < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeLinearTolerance)
	}
	if cfg.OrientationToleranceDegs < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeOrientationTolerance)
	}
	return []string{cfg.Primary, cfg.Follower}, nil, nil
}

func (cfg *Config) maxJointVelRadPerSec() float64 {
	v := cfg.MaxJointVelDegsPerSec
	if v <= 0 {
		v = defaultMaxJointVelDegsPerSec
	}
	return v * math.Pi / 180
}

func (cfg *Config) linearToleranceMm() float64 {
	if cfg.LinearToleranceMm <= 0 {
		return defaultLinearToleranceMm
	}
	return cfg.LinearToleranceMm
}

func (cfg *Config) orientationToleranceDegs() float64 {
	if cfg.OrientationToleranceDegs <= 0 {
		return defaultOrientationToleranceDegs
	}
	return cfg.OrientationToleranceDegs
}

type service struct {
	resource.AlwaysRebuild
	resource.Named
	resource.TriviallyCloseable

	logger    logging.Logger
	cfg       *Config
	primary   arm.Arm
	follower  arm.Arm
	fsService framesystem.Service
}

func newCoordinatedArm(_ context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (arm.Arm, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	primary, err := arm.FromProvider(deps, cfg.Primary)
	if err != nil {
		return nil, fmt.Errorf("resolve primary arm %q: %w", cfg.Primary, err)
	}
	follower, err := arm.FromProvider(deps, cfg.Follower)
	if err != nil {
		return nil, fmt.Errorf("resolve follower arm %q: %w", cfg.Follower, err)
	}
	fs, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("resolve framesystem: %w", err)
	}
	return &service{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		cfg:       cfg,
		primary:   primary,
		follower:  follower,
		fsService: fs,
	}, nil
}

// Primary-forwarded methods. The wrapper presents the primary's frame as its own,
// so state queries and kinematics come from the primary. Follower is coordinated
// under the hood only when motion is commanded.

func (s *service) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	return s.primary.JointPositions(ctx, extra)
}

func (s *service) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.primary.CurrentInputs(ctx)
}

func (s *service) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	return s.primary.EndPosition(ctx, extra)
}

func (s *service) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.primary.Kinematics(ctx)
}

func (s *service) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return s.primary.Geometries(ctx, extra)
}

func (s *service) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return s.primary.Get3DModels(ctx, extra)
}

// Stop and IsMoving act on both arms.

func (s *service) Stop(ctx context.Context, extra map[string]interface{}) error {
	pErr := s.primary.Stop(ctx, extra)
	fErr := s.follower.Stop(ctx, extra)
	if pErr != nil {
		return pErr
	}
	return fErr
}

func (s *service) IsMoving(ctx context.Context) (bool, error) {
	pMoving, err := s.primary.IsMoving(ctx)
	if err != nil {
		return false, err
	}
	if pMoving {
		return true, nil
	}
	return s.follower.IsMoving(ctx)
}

func (s *service) DoCommand(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
	return nil, resource.ErrDoUnimplemented
}
