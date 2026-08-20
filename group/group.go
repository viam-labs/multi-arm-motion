package group

import (
	"context"
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/generic"
)

var Model = resource.NewModel("viam", "multi-arm-motion", "group")

const (
	defaultMaxJointVelDegsPerSec = 30.0
	defaultWaypointSpacingMs     = 20
)

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newGroup,
		},
	)
}

type Config struct {
	Arms                  []string `json:"arms"`
	MaxJointVelDegsPerSec float64  `json:"max_joint_vel_degs_per_sec,omitempty"`
	WaypointSpacingMs     int      `json:"waypoint_spacing_ms,omitempty"`
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
	for i, name := range cfg.Arms {
		if name == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, fmt.Sprintf("arms[%d]", i))
		}
		deps = append(deps, name)
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
}

func newGroup(_ context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (resource.Resource, error) {
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

func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if raw, ok := cmd["jog"]; ok {
		delta, err := parseJog(raw)
		if err != nil {
			return nil, err
		}
		if err := s.Jog(ctx, delta); err != nil {
			return nil, err
		}
		return map[string]interface{}{"success": true}, nil
	}
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (s *service) Close(_ context.Context) error {
	return nil
}
