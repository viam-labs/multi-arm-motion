package posepreset

import (
	"context"
	"fmt"
	"math"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

var Model = resource.NewModel("viam", "multi-arm-motion", "pose-preset")

const (
	positionIdle  uint32 = 0
	positionTeach uint32 = 1
	positionGo    uint32 = 2

	numberOfPositions uint32 = 3

	defaultMaxJointVelDegsPerSec    = 30.0
	defaultWaypointSpacingMs        = 20
	defaultLinearToleranceMm        = 2.0
	defaultOrientationToleranceDegs = 2.0

	modeBarrier         = "barrier"
	modePrimaryFollower = "primary_follower"
)

var positionLabels = []string{"idle", "update config", "go to"}

func init() {
	resource.RegisterComponent(toggleswitch.API, Model,
		resource.Registration[toggleswitch.Switch, *Config]{
			Constructor: newPosePreset,
		},
	)
}

type SavedPose struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	OX    float64 `json:"oX"`
	OY    float64 `json:"oY"`
	OZ    float64 `json:"oZ"`
	Theta float64 `json:"theta"`
}

func (p SavedPose) ToPose() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: p.X, Y: p.Y, Z: p.Z},
		&spatialmath.OrientationVectorDegrees{OX: p.OX, OY: p.OY, OZ: p.OZ, Theta: p.Theta},
	)
}

type Config struct {
	Arms                     []string             `json:"arms"`
	Poses                    map[string]SavedPose `json:"poses,omitempty"`
	Mode                     string               `json:"mode,omitempty"`
	Primary                  string               `json:"primary,omitempty"`
	FollowerOffsets          map[string]SavedPose `json:"follower_offsets,omitempty"`
	MaxJointVelDegsPerSec    float64              `json:"max_joint_vel_degs_per_sec,omitempty"`
	WaypointSpacingMs        int                  `json:"waypoint_spacing_ms,omitempty"`
	LinearToleranceMm        float64              `json:"linear_tolerance_mm,omitempty"`
	OrientationToleranceDegs float64              `json:"orientation_tolerance_degs,omitempty"`
	LogDrift                 bool                 `json:"log_drift,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if len(cfg.Arms) < 2 {
		return nil, nil, resource.NewConfigValidationError(path, errAtLeastTwoArms)
	}
	if cfg.MaxJointVelDegsPerSec < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeMaxJointVel)
	}
	if cfg.WaypointSpacingMs < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeWaypointSpacing)
	}
	if cfg.LinearToleranceMm < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeLinearTolerance)
	}
	if cfg.OrientationToleranceDegs < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeOrientationTolerance)
	}
	deps := make([]string, 0, len(cfg.Arms))
	seen := map[string]struct{}{}
	for i, name := range cfg.Arms {
		if name == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, fmt.Sprintf("arms[%d]", i))
		}
		if _, dup := seen[name]; dup {
			return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf("duplicate arm %q", name))
		}
		seen[name] = struct{}{}
		deps = append(deps, name)
	}
	switch cfg.Mode {
	case "", modeBarrier:
	case modePrimaryFollower:
		if cfg.Primary == "" {
			return nil, nil, resource.NewConfigValidationError(path, errPrimaryRequired)
		}
		if _, ok := seen[cfg.Primary]; !ok {
			return nil, nil, resource.NewConfigValidationError(path,
				fmt.Errorf("primary %q not in arms list", cfg.Primary))
		}
		if _, ok := cfg.FollowerOffsets[cfg.Primary]; ok {
			return nil, nil, resource.NewConfigValidationError(path,
				fmt.Errorf("follower_offsets must not contain the primary %q", cfg.Primary))
		}
		for _, name := range cfg.Arms {
			if name == cfg.Primary {
				continue
			}
			if _, ok := cfg.FollowerOffsets[name]; !ok {
				return nil, nil, resource.NewConfigValidationError(path,
					fmt.Errorf("follower_offsets missing entry for %q", name))
			}
		}
		for name := range cfg.FollowerOffsets {
			if _, ok := seen[name]; !ok {
				return nil, nil, resource.NewConfigValidationError(path,
					fmt.Errorf("follower_offsets has arm %q not declared in arms", name))
			}
		}
	default:
		return nil, nil, resource.NewConfigValidationError(path,
			fmt.Errorf("unknown mode %q; supported: %q, %q", cfg.Mode, modeBarrier, modePrimaryFollower))
	}
	if cfg.Poses != nil {
		for name := range cfg.Poses {
			if _, ok := seen[name]; !ok {
				return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf("poses has arm %q not declared in arms", name))
			}
		}
		switch cfg.Mode {
		case "", modeBarrier:
			for _, name := range cfg.Arms {
				if _, ok := cfg.Poses[name]; !ok {
					return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf("poses missing arm %q", name))
				}
			}
		case modePrimaryFollower:
			if _, ok := cfg.Poses[cfg.Primary]; !ok {
				return nil, nil, resource.NewConfigValidationError(path,
					fmt.Errorf("poses missing primary %q", cfg.Primary))
			}
		}
	}
	return deps, nil, nil
}

func (cfg *Config) maxJointVelRadPerSec() float64 {
	v := cfg.MaxJointVelDegsPerSec
	if v <= 0 {
		v = defaultMaxJointVelDegsPerSec
	}
	return v * math.Pi / 180
}

func (cfg *Config) waypointSpacing() time.Duration {
	ms := cfg.WaypointSpacingMs
	if ms <= 0 {
		ms = defaultWaypointSpacingMs
	}
	return time.Duration(ms) * time.Millisecond
}

func (cfg *Config) mode() string {
	if cfg.Mode == "" {
		return modeBarrier
	}
	return cfg.Mode
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
	logger    logging.Logger
	cfg       *Config
	arms      map[string]arm.Arm
	armOrder  []string
	fsService framesystem.Service
	position  uint32
}

func newPosePreset(_ context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	arms := make(map[string]arm.Arm, len(cfg.Arms))
	for _, name := range cfg.Arms {
		a, err := arm.FromProvider(deps, name)
		if err != nil {
			return nil, fmt.Errorf("resolve arm %q: %w", name, err)
		}
		arms[name] = a
	}

	fs, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("resolve framesystem: %w", err)
	}

	return &service{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		cfg:       cfg,
		arms:      arms,
		armOrder:  append([]string(nil), cfg.Arms...),
		fsService: fs,
	}, nil
}

func (s *service) SetPosition(ctx context.Context, position uint32, _ map[string]interface{}) error {
	switch position {
	case positionIdle:
		s.position = position
		return nil
	case positionTeach:
		s.position = position
		defer func() { s.position = positionIdle }()
		return s.teach(ctx)
	case positionGo:
		s.position = position
		defer func() { s.position = positionIdle }()
		switch s.cfg.mode() {
		case modeBarrier:
			return s.recallBarrier(ctx)
		case modePrimaryFollower:
			return s.recallPrimaryFollower(ctx)
		default:
			return fmt.Errorf("unsupported mode %q", s.cfg.mode())
		}
	default:
		return fmt.Errorf("invalid position %d", position)
	}
}

func (s *service) GetPosition(_ context.Context, _ map[string]interface{}) (uint32, error) {
	return s.position, nil
}

func (s *service) GetNumberOfPositions(_ context.Context, _ map[string]interface{}) (uint32, []string, error) {
	return numberOfPositions, positionLabels, nil
}

func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["measure_drift"]; ok {
		res, err := s.measureDrift(ctx)
		if err != nil {
			return nil, err
		}
		return res.ToMap(), nil
	}
	return nil, resource.ErrDoUnimplemented
}

func (s *service) Close(_ context.Context) error {
	return nil
}
