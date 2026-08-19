// Package group implements a coordinated group of arms driven together via
// barrier-fired streamed trajectories.
package group

import (
	"context"

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

type Config struct{}

func (cfg *Config) Validate(_ string) ([]string, []string, error) {
	return nil, nil, nil
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
