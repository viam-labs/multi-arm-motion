package group

import (
	"testing"

	"go.viam.com/test"
)

func TestParseExecuteHappyPath(t *testing.T) {
	raw := map[string]interface{}{
		"trajectories": map[string]interface{}{
			"arm-1": []interface{}{
				map[string]interface{}{"time_ms": 0.0, "positions": []interface{}{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
				map[string]interface{}{"time_ms": 20.0, "positions": []interface{}{0.1, 0.0, 0.0, 0.0, 0.0, 0.0}},
			},
			"arm-2": []interface{}{
				map[string]interface{}{"time_ms": 0.0, "positions": []interface{}{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
				map[string]interface{}{"time_ms": 20.0, "positions": []interface{}{1.1, 0.0, 0.0, 0.0, 0.0, 0.0}},
			},
		},
	}
	trajs, err := parseExecute(raw)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(trajs), test.ShouldEqual, 2)
	test.That(t, len(trajs["arm-1"]), test.ShouldEqual, 2)
	test.That(t, trajs["arm-1"][0].Positions[0], test.ShouldEqual, 0.0)
	test.That(t, trajs["arm-1"][1].Positions[0], test.ShouldEqual, 0.1)
}

func TestParseExecuteRejectsMissingTrajectories(t *testing.T) {
	_, err := parseExecute(map[string]interface{}{})
	test.That(t, err, test.ShouldEqual, errMissingExecuteTrajectories)
}

func TestParseExecuteRejectsFewerThanTwoWaypoints(t *testing.T) {
	raw := map[string]interface{}{
		"trajectories": map[string]interface{}{
			"arm-1": []interface{}{
				map[string]interface{}{"time_ms": 0.0, "positions": []interface{}{0.0}},
			},
		},
	}
	_, err := parseExecute(raw)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "at least 2 waypoints")
}

func TestParseExecuteRejectsNonMonotonicTimes(t *testing.T) {
	raw := map[string]interface{}{
		"trajectories": map[string]interface{}{
			"arm-1": []interface{}{
				map[string]interface{}{"time_ms": 0.0, "positions": []interface{}{0.0}},
				map[string]interface{}{"time_ms": 100.0, "positions": []interface{}{0.1}},
				map[string]interface{}{"time_ms": 50.0, "positions": []interface{}{0.2}},
			},
		},
	}
	_, err := parseExecute(raw)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "strictly increase")
}

func TestParseExecuteRejectsFirstWaypointNonZero(t *testing.T) {
	raw := map[string]interface{}{
		"trajectories": map[string]interface{}{
			"arm-1": []interface{}{
				map[string]interface{}{"time_ms": 10.0, "positions": []interface{}{0.0}},
				map[string]interface{}{"time_ms": 20.0, "positions": []interface{}{0.1}},
			},
		},
	}
	_, err := parseExecute(raw)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "first waypoint must have time_ms=0")
}

func TestParseExecuteRejectsNonNumericTime(t *testing.T) {
	raw := map[string]interface{}{
		"trajectories": map[string]interface{}{
			"arm-1": []interface{}{
				map[string]interface{}{"time_ms": "0", "positions": []interface{}{0.0}},
				map[string]interface{}{"time_ms": 20.0, "positions": []interface{}{0.1}},
			},
		},
	}
	_, err := parseExecute(raw)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "time_ms must be a number")
}

func TestParseExecuteRejectsNonArrayPositions(t *testing.T) {
	raw := map[string]interface{}{
		"trajectories": map[string]interface{}{
			"arm-1": []interface{}{
				map[string]interface{}{"time_ms": 0.0, "positions": "not an array"},
				map[string]interface{}{"time_ms": 20.0, "positions": []interface{}{0.1}},
			},
		},
	}
	_, err := parseExecute(raw)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "positions must be an array")
}
