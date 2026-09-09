package sequencer

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/button"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var Model = resource.NewModel("viam", "multi-arm-motion", "sequencer")

const goToPosition uint32 = 2

func init() {
	resource.RegisterComponent(button.API, Model,
		resource.Registration[button.Button, *Config]{
			Constructor: newSequencer,
		},
	)
}

type Config struct {
	Positions    []string `json:"positions"`
	SleepSeconds float64  `json:"sleep_seconds,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if len(cfg.Positions) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "positions")
	}
	if cfg.SleepSeconds < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errNegativeSleep)
	}
	deps := make([]string, 0, len(cfg.Positions))
	for i, name := range cfg.Positions {
		if name == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, fmt.Sprintf("positions[%d]", i))
		}
		deps = append(deps, name)
	}
	return deps, nil, nil
}

func (cfg *Config) sleep() time.Duration {
	if cfg.SleepSeconds <= 0 {
		return 0
	}
	return time.Duration(cfg.SleepSeconds * float64(time.Second))
}

type service struct {
	resource.AlwaysRebuild
	resource.Named
	logger    logging.Logger
	cfg       *Config
	positions []toggleswitch.Switch
	posOrder  []string
}

func newSequencer(_ context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (button.Button, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	positions := make([]toggleswitch.Switch, 0, len(cfg.Positions))
	for _, name := range cfg.Positions {
		s, err := toggleswitch.FromProvider(deps, name)
		if err != nil {
			return nil, fmt.Errorf("resolve switch %q: %w", name, err)
		}
		positions = append(positions, s)
	}
	return &service{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		cfg:       cfg,
		positions: positions,
		posOrder:  append([]string(nil), cfg.Positions...),
	}, nil
}

func (s *service) Push(ctx context.Context, _ map[string]interface{}) error {
	sleep := s.cfg.sleep()
	for i, sw := range s.positions {
		s.logger.Infof("sequencer step %d/%d: SetPosition(%d) on %q", i+1, len(s.positions), goToPosition, s.posOrder[i])
		if err := sw.SetPosition(ctx, goToPosition, nil); err != nil {
			return fmt.Errorf("step %d, switch %q: %w", i+1, s.posOrder[i], err)
		}
		if sleep > 0 && i < len(s.positions)-1 {
			select {
			case <-ctx.Done():
				return ctx.Err()
			case <-time.After(sleep):
			}
		}
	}
	return nil
}

func (s *service) Close(_ context.Context) error {
	return nil
}
