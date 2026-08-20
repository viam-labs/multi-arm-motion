package preset

import (
	"context"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/utils"
)

func (s *service) teach(ctx context.Context) error {
	newJoints := make(map[string][]float64, len(s.armOrder))
	for _, name := range s.armOrder {
		j, err := s.arms[name].JointPositions(ctx, nil)
		if err != nil {
			return fmt.Errorf("arm %q: joints: %w", name, err)
		}
		vals := make([]float64, len(j))
		for i, v := range j {
			vals[i] = float64(v)
		}
		newJoints[name] = vals
	}

	partID := os.Getenv(utils.MachinePartIDEnvVar)
	if partID == "" {
		return fmt.Errorf("%s env var not set", utils.MachinePartIDEnvVar)
	}

	client, err := app.CreateViamClientFromEnvVars(ctx, nil, s.logger)
	if err != nil {
		return fmt.Errorf("create app client: %w", err)
	}
	defer func() { _ = client.Close() }()

	appClient := client.AppClient()
	part, _, err := appClient.GetRobotPart(ctx, partID)
	if err != nil {
		return fmt.Errorf("fetch robot part: %w", err)
	}

	if err := setPresetJoints(part.RobotConfig, s.Named.Name().Name, newJoints); err != nil {
		return err
	}

	if _, err := appClient.UpdateRobotPart(ctx, partID, part.Name, part.RobotConfig); err != nil {
		return fmt.Errorf("update robot part: %w", err)
	}
	s.logger.Infof("preset %q: teach wrote joints for %d arms", s.Named.Name().Name, len(newJoints))
	return nil
}

func setPresetJoints(robotConfig map[string]interface{}, presetName string, joints map[string][]float64) error {
	comp, err := findComponent(robotConfig, presetName)
	if err != nil {
		return fmt.Errorf("preset %q not found in components — teach only works when the preset lives at the top level, not inside a fragment", presetName)
	}
	attrs, _ := comp["attributes"].(map[string]interface{})
	if attrs == nil {
		attrs = map[string]interface{}{}
		comp["attributes"] = attrs
	}
	jointsAny := make(map[string]interface{}, len(joints))
	for k, v := range joints {
		arr := make([]interface{}, len(v))
		for i, f := range v {
			arr[i] = f
		}
		jointsAny[k] = arr
	}
	attrs["joints"] = jointsAny
	return nil
}

func findComponent(robotConfig map[string]interface{}, name string) (map[string]interface{}, error) {
	components, _ := robotConfig["components"].([]interface{})
	for _, c := range components {
		cm, ok := c.(map[string]interface{})
		if !ok {
			continue
		}
		if cm["name"] == name {
			return cm, nil
		}
	}
	return nil, fmt.Errorf("component %q not found in components", name)
}
