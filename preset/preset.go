package preset

import (
	"context"
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
)

var Model = resource.NewModel("viam", "multi-arm-motion", "preset")

const (
	positionIdle uint32 = 0
	positionTeach uint32 = 1
	positionGo    uint32 = 2

	numberOfPositions uint32 = 3

	defaultMaxJointVelDegsPerSec = 30.0
	defaultWaypointSpacingMs     = 20
)

var positionLabels = []string{"idle", "update config", "go to"}

func init() {
	resource.RegisterComponent(toggleswitch.API, Model,
		resource.Registration[toggleswitch.Switch, *Config]{
			Constructor: newPreset,
		},
	)
}

type Config struct {
	Arms                  []string             `json:"arms"`
	Joints                map[string][]float64 `json:"joints,omitempty"`
	MaxJointVelDegsPerSec float64              `json:"max_joint_vel_degs_per_sec,omitempty"`
	WaypointSpacingMs     int                  `json:"waypoint_spacing_ms,omitempty"`
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
	if cfg.Joints != nil {
		for _, name := range cfg.Arms {
			if _, ok := cfg.Joints[name]; !ok {
				return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf("joints missing arm %q", name))
			}
		}
		for name := range cfg.Joints {
			if _, ok := seen[name]; !ok {
				return nil, nil, resource.NewConfigValidationError(path, fmt.Errorf("joints has arm %q not declared in arms", name))
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

func newPreset(_ context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
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
		return s.recall(ctx)
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

func (s *service) Close(_ context.Context) error {
	return nil
}
