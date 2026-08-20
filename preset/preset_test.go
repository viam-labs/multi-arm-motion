package preset

import (
	"testing"
	"time"

	"go.viam.com/test"
)

func validConfig() *Config {
	return &Config{Arms: []string{"arm-1", "arm-2"}}
}

func TestValidateHappyPath(t *testing.T) {
	deps, _, err := validConfig().Validate("preset")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"arm-1", "arm-2"})
}

func TestValidateHappyPathWithJoints(t *testing.T) {
	cfg := validConfig()
	cfg.Joints = map[string][]float64{
		"arm-1": {0, 0, 0, 0, 0, 0},
		"arm-2": {0, 0, 0, 0, 0, 0},
	}
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldBeNil)
}

func TestValidateRejectsFewerThanTwoArms(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1"}}
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "at least two arms")
}

func TestValidateRejectsEmptyArmName(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1", ""}}
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arms[1]")
}

func TestValidateRejectsDuplicateArm(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1", "arm-1"}}
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "duplicate")
}

func TestValidateRejectsJointsMissingArm(t *testing.T) {
	cfg := validConfig()
	cfg.Joints = map[string][]float64{"arm-1": {0, 0, 0, 0, 0, 0}}
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm-2")
}

func TestValidateRejectsJointsExtraArm(t *testing.T) {
	cfg := validConfig()
	cfg.Joints = map[string][]float64{
		"arm-1": {0, 0, 0, 0, 0, 0},
		"arm-2": {0, 0, 0, 0, 0, 0},
		"arm-3": {0, 0, 0, 0, 0, 0},
	}
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm-3")
}

func TestValidateRejectsNegativeMaxJointVel(t *testing.T) {
	cfg := validConfig()
	cfg.MaxJointVelDegsPerSec = -1
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "max_joint_vel_degs_per_sec")
}

func TestValidateRejectsNegativeWaypointSpacing(t *testing.T) {
	cfg := validConfig()
	cfg.WaypointSpacingMs = -1
	_, _, err := cfg.Validate("preset")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "waypoint_spacing_ms")
}

func TestConfigDefaults(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.maxJointVelRadPerSec(), test.ShouldBeGreaterThan, 0.0)
	test.That(t, cfg.waypointSpacing(), test.ShouldEqual, time.Duration(defaultWaypointSpacingMs)*time.Millisecond)
}
