package posepreset

import (
	"context"
	"encoding/json"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

func (s *service) teach(ctx context.Context) error {
	newPoses := make(map[string]SavedPose, len(s.armOrder))
	for _, name := range s.armOrder {
		p, err := s.fsService.TransformPose(ctx,
			referenceframe.NewPoseInFrame(name, spatialmath.NewZeroPose()),
			referenceframe.World, nil)
		if err != nil {
			return fmt.Errorf("arm %q: current world pose: %w", name, err)
		}
		pose := p.Pose()
		pt := pose.Point()
		ov := pose.Orientation().OrientationVectorDegrees()
		newPoses[name] = SavedPose{
			X:     pt.X,
			Y:     pt.Y,
			Z:     pt.Z,
			OX:    ov.OX,
			OY:    ov.OY,
			OZ:    ov.OZ,
			Theta: ov.Theta,
		}
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

	if err := setPosePresetPoses(part.RobotConfig, s.Named.Name().Name, newPoses); err != nil {
		return err
	}

	if _, err := appClient.UpdateRobotPart(ctx, partID, part.Name, part.RobotConfig); err != nil {
		return fmt.Errorf("update robot part: %w", err)
	}
	s.logger.Infof("pose-preset %q: teach wrote poses for %d arms", s.Named.Name().Name, len(newPoses))
	return nil
}

func setPosePresetPoses(robotConfig map[string]interface{}, presetName string, poses map[string]SavedPose) error {
	comp, err := findComponent(robotConfig, presetName)
	if err != nil {
		return fmt.Errorf("pose-preset %q not found in components — teach only works when the pose-preset lives at the top level, not inside a fragment", presetName)
	}
	attrs, _ := comp["attributes"].(map[string]interface{})
	if attrs == nil {
		attrs = map[string]interface{}{}
		comp["attributes"] = attrs
	}
	data, err := json.Marshal(poses)
	if err != nil {
		return fmt.Errorf("marshal poses: %w", err)
	}
	var posesAny map[string]interface{}
	if err := json.Unmarshal(data, &posesAny); err != nil {
		return fmt.Errorf("unmarshal poses: %w", err)
	}
	attrs["poses"] = posesAny
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
