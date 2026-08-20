package group

import (
	"testing"
	"time"

	"go.viam.com/test"
)

func validConfig() *Config {
	return &Config{Arms: []string{"arm-1", "arm-2"}}
}

func TestValidateHappyPath(t *testing.T) {
	deps, _, err := validConfig().Validate("group")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"arm-1", "arm-2"})
}

func TestValidateHappyPathThreeArms(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1", "arm-2", "arm-3"}}
	deps, _, err := cfg.Validate("group")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(deps), test.ShouldEqual, 3)
}

func TestValidateRejectsFewerThanTwoArms(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1"}}
	_, _, err := cfg.Validate("group")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "at least two arms")
}

func TestValidateRejectsEmptyArms(t *testing.T) {
	cfg := &Config{Arms: nil}
	_, _, err := cfg.Validate("group")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "at least two arms")
}

func TestValidateRejectsEmptyArmName(t *testing.T) {
	cfg := &Config{Arms: []string{"arm-1", ""}}
	_, _, err := cfg.Validate("group")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arms[1]")
}

func TestValidateRejectsNegativeMaxJointVel(t *testing.T) {
	cfg := validConfig()
	cfg.MaxJointVelDegsPerSec = -1
	_, _, err := cfg.Validate("group")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "max_joint_vel_degs_per_sec")
}

func TestValidateRejectsNegativeWaypointSpacing(t *testing.T) {
	cfg := validConfig()
	cfg.WaypointSpacingMs = -1
	_, _, err := cfg.Validate("group")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "waypoint_spacing_ms")
}

func TestConfigMaxJointVelDefaults(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.maxJointVelRadPerSec(), test.ShouldAlmostEqual, defaultMaxJointVelDegsPerSec*3.14159265358979/180, 1e-6)
}

func TestConfigWaypointSpacingDefaults(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.waypointSpacing(), test.ShouldEqual, time.Duration(defaultWaypointSpacingMs)*time.Millisecond)
}

func TestConfigMaxJointVelHonorsCustom(t *testing.T) {
	cfg := &Config{MaxJointVelDegsPerSec: 90}
	test.That(t, cfg.maxJointVelRadPerSec(), test.ShouldAlmostEqual, 90*3.14159265358979/180, 1e-6)
}

func TestConfigWaypointSpacingHonorsCustom(t *testing.T) {
	cfg := &Config{WaypointSpacingMs: 50}
	test.That(t, cfg.waypointSpacing(), test.ShouldEqual, 50*time.Millisecond)
}
