// Package group implements a coordinated group of arms driven together via
// barrier-fired streamed trajectories.
package group

import (
	"context"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

var Model = resource.NewModel("viam", "multi-arm-motion", "group")

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newGroup,
		},
	)
}

type Config struct {
	Arms []string `json:"arms"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if len(cfg.Arms) < 2 {
		return nil, nil, resource.NewConfigValidationError(path, errAtLeastTwoArms)
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

type service struct {
	resource.AlwaysRebuild
	resource.Named
	logger logging.Logger
}

func newGroup(_ context.Context, _ resource.Dependencies, conf resource.Config, logger logging.Logger) (resource.Resource, error) {
	return &service{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}, nil
}

func (s *service) DoCommand(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{"status": "not implemented"}, nil
}

func (s *service) Close(_ context.Context) error {
	return nil
}
