package coordinatedarm

import (
	"testing"

	"go.viam.com/test"
)

func validConfig() *Config {
	return &Config{
		Primary:        "arm-1",
		Follower:       "arm-2",
		FollowerOffset: Offset{X: 0, Y: -1000, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0},
	}
}

func TestValidateHappyPath(t *testing.T) {
	deps, _, err := validConfig().Validate("coordinated-arm")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"arm-1", "arm-2"})
}

func TestValidateRequiresPrimary(t *testing.T) {
	cfg := validConfig()
	cfg.Primary = ""
	_, _, err := cfg.Validate("coordinated-arm")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "primary")
}

func TestValidateRequiresFollower(t *testing.T) {
	cfg := validConfig()
	cfg.Follower = ""
	_, _, err := cfg.Validate("coordinated-arm")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "follower")
}

func TestValidateRejectsPrimaryEqualsFollower(t *testing.T) {
	cfg := validConfig()
	cfg.Follower = cfg.Primary
	_, _, err := cfg.Validate("coordinated-arm")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "different")
}

func TestValidateRejectsNegativeMaxJointVel(t *testing.T) {
	cfg := validConfig()
	cfg.MaxJointVelDegsPerSec = -1
	_, _, err := cfg.Validate("coordinated-arm")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "max_joint_vel_degs_per_sec")
}

func TestValidateRejectsNegativeLinearTolerance(t *testing.T) {
	cfg := validConfig()
	cfg.LinearToleranceMm = -0.5
	_, _, err := cfg.Validate("coordinated-arm")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "linear_tolerance_mm")
}

func TestConfigDefaults(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.maxJointVelRadPerSec(), test.ShouldBeGreaterThan, 0.0)
	test.That(t, cfg.linearToleranceMm(), test.ShouldEqual, defaultLinearToleranceMm)
	test.That(t, cfg.orientationToleranceDegs(), test.ShouldEqual, defaultOrientationToleranceDegs)
}

func TestOffsetToPose(t *testing.T) {
	o := Offset{X: 1, Y: 2, Z: 3, OX: 0, OY: 0, OZ: 1, Theta: 45}
	p := o.ToPose()
	test.That(t, p.Point().X, test.ShouldAlmostEqual, 1.0)
	test.That(t, p.Point().Y, test.ShouldAlmostEqual, 2.0)
	test.That(t, p.Point().Z, test.ShouldAlmostEqual, 3.0)
	test.That(t, p.Orientation().OrientationVectorDegrees().Theta, test.ShouldAlmostEqual, 45.0)
}
