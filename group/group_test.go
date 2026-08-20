package group

import (
	"testing"

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
