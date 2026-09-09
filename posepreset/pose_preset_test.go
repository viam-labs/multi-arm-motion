package posepreset

import (
	"encoding/json"
	"testing"
	"time"

	"go.viam.com/test"
)

func validConfig() *Config {
	return &Config{Arms: []string{"arm-1", "arm-2"}}
}

func samplePose() SavedPose {
	return SavedPose{X: 100, Y: 0, Z: 300, OX: 0, OY: 0, OZ: -1, Theta: 0}
}

func TestValidateHappyPath(t *testing.T) {
	deps, _, err := validConfig().Validate("pose-preset")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"arm-1", "arm-2"})
}

func TestValidateHappyPathWithPoses(t *testing.T) {
	cfg := validConfig()
	cfg.Poses = map[string]SavedPose{
		"arm-1": samplePose(),
		"arm-2": samplePose(),
	}
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldBeNil)
}

func TestValidateHappyPathWithExplicitBarrierMode(t *testing.T) {
	cfg := validConfig()
	cfg.Mode = "barrier"
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldBeNil)
}

func TestValidateRejectsUnknownMode(t *testing.T) {
	cfg := validConfig()
	cfg.Mode = "primary_follower"
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "unknown mode")
}

func TestValidateRejectsFewerThanTwoArms(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1"}}
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "at least two arms")
}

func TestValidateRejectsEmptyArmName(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1", ""}}
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arms[1]")
}

func TestValidateRejectsDuplicateArm(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1", "arm-1"}}
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "duplicate")
}

func TestValidateRejectsPosesMissingArm(t *testing.T) {
	cfg := validConfig()
	cfg.Poses = map[string]SavedPose{"arm-1": samplePose()}
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm-2")
}

func TestValidateRejectsPosesExtraArm(t *testing.T) {
	cfg := validConfig()
	cfg.Poses = map[string]SavedPose{
		"arm-1": samplePose(),
		"arm-2": samplePose(),
		"arm-3": samplePose(),
	}
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm-3")
}

func TestValidateRejectsNegativeMaxJointVel(t *testing.T) {
	cfg := validConfig()
	cfg.MaxJointVelDegsPerSec = -1
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "max_joint_vel_degs_per_sec")
}

func TestValidateRejectsNegativeWaypointSpacing(t *testing.T) {
	cfg := validConfig()
	cfg.WaypointSpacingMs = -1
	_, _, err := cfg.Validate("pose-preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "waypoint_spacing_ms")
}

func TestConfigDefaults(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.maxJointVelRadPerSec(), test.ShouldBeGreaterThan, 0.0)
	test.That(t, cfg.waypointSpacing(), test.ShouldEqual, time.Duration(defaultWaypointSpacingMs)*time.Millisecond)
	test.That(t, cfg.mode(), test.ShouldEqual, modeBarrier)
}

func TestSavedPoseToPose(t *testing.T) {
	sp := samplePose()
	p := sp.ToPose()
	test.That(t, p.Point().X, test.ShouldEqual, 100.0)
	test.That(t, p.Point().Z, test.ShouldEqual, 300.0)
	test.That(t, p.Orientation().OrientationVectorDegrees().OZ, test.ShouldEqual, -1.0)
}

func TestSavedPoseJSONShape(t *testing.T) {
	sp := SavedPose{X: 80, Y: 400, Z: 120, OX: 1, OY: 0, OZ: 0, Theta: -180}
	data, err := json.Marshal(sp)
	test.That(t, err, test.ShouldBeNil)
	got := string(data)
	test.That(t, got, test.ShouldContainSubstring, `"x":80`)
	test.That(t, got, test.ShouldContainSubstring, `"y":400`)
	test.That(t, got, test.ShouldContainSubstring, `"z":120`)
	test.That(t, got, test.ShouldContainSubstring, `"oX":1`)
	test.That(t, got, test.ShouldContainSubstring, `"oY":0`)
	test.That(t, got, test.ShouldContainSubstring, `"oZ":0`)
	test.That(t, got, test.ShouldContainSubstring, `"theta":-180`)

	canonical := `{"x":80.02953962893187,"y":399.59674252099467,"z":120.00704936710844,"oX":0.9999999994829719,"oY":-0.00000756621835589118,"oZ":-0.000031253948799640634,"theta":-179.99880766452415}`
	var round SavedPose
	test.That(t, json.Unmarshal([]byte(canonical), &round), test.ShouldBeNil)
	test.That(t, round.X, test.ShouldAlmostEqual, 80.02953962893187)
	test.That(t, round.OX, test.ShouldAlmostEqual, 0.9999999994829719)
	test.That(t, round.Theta, test.ShouldAlmostEqual, -179.99880766452415)
}
