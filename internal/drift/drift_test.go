package drift

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

type stubReader struct{ poses map[string]spatialmath.Pose }

func (s stubReader) WorldPose(_ context.Context, name string) (spatialmath.Pose, error) {
	return s.poses[name], nil
}

func pose(x, y, z float64, ox, oy, oz, theta float64) spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: x, Y: y, Z: z},
		&spatialmath.OrientationVectorDegrees{OX: ox, OY: oy, OZ: oz, Theta: theta},
	)
}

func TestMeasureZeroDriftWhenActualMatchesTarget(t *testing.T) {
	targets := map[string]spatialmath.Pose{
		"arm-1": pose(100, 0, 300, 0, 0, -1, 0),
		"arm-2": pose(100, -500, 300, 0, 0, -1, 0),
	}
	reader := stubReader{poses: map[string]spatialmath.Pose{
		"arm-1": targets["arm-1"],
		"arm-2": targets["arm-2"],
	}}

	res, err := Measure(context.Background(), reader, []string{"arm-1", "arm-2"}, targets)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.Arms["arm-1"].PositionErrorMm, test.ShouldAlmostEqual, 0.0, 1e-6)
	test.That(t, res.Arms["arm-2"].PositionErrorMm, test.ShouldAlmostEqual, 0.0, 1e-6)
	test.That(t, res.Arms["arm-1"].OrientationErrorDeg, test.ShouldAlmostEqual, 0.0, 1e-6)
	test.That(t, res.Pairs["arm-1|arm-2"].PositionErrorMm, test.ShouldAlmostEqual, 0.0, 1e-6)
}

func TestMeasureCapturesPositionError(t *testing.T) {
	targets := map[string]spatialmath.Pose{
		"arm-1": pose(100, 0, 300, 0, 0, -1, 0),
		"arm-2": pose(100, -500, 300, 0, 0, -1, 0),
	}
	reader := stubReader{poses: map[string]spatialmath.Pose{
		"arm-1": pose(103, 0, 300, 0, 0, -1, 0), // 3mm off in X
		"arm-2": pose(100, -500, 300, 0, 0, -1, 0),
	}}

	res, err := Measure(context.Background(), reader, []string{"arm-1", "arm-2"}, targets)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.Arms["arm-1"].PositionErrorMm, test.ShouldAlmostEqual, 3.0, 1e-6)
	test.That(t, res.Arms["arm-2"].PositionErrorMm, test.ShouldAlmostEqual, 0.0, 1e-6)
	// Relative pose drift: arm-1 shifted 3mm, arm-2 didn't → relative delta is 3mm.
	test.That(t, res.Pairs["arm-1|arm-2"].PositionErrorMm, test.ShouldAlmostEqual, 3.0, 1e-6)
}

func TestMeasureCapturesOrientationError(t *testing.T) {
	targets := map[string]spatialmath.Pose{
		"arm-1": pose(100, 0, 300, 0, 0, -1, 0),
	}
	// Rotate the tool 5 deg around Z (theta increases).
	reader := stubReader{poses: map[string]spatialmath.Pose{
		"arm-1": pose(100, 0, 300, 0, 0, -1, 5),
	}}

	res, err := Measure(context.Background(), reader, []string{"arm-1"}, targets)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.Arms["arm-1"].OrientationErrorDeg, test.ShouldAlmostEqual, 5.0, 0.01)
}

func TestMeasurePairwiseErrorIsRelative(t *testing.T) {
	// Both arms drifted by the same amount → per-arm error is nonzero, pair error is zero.
	targets := map[string]spatialmath.Pose{
		"arm-1": pose(100, 0, 300, 0, 0, -1, 0),
		"arm-2": pose(100, -500, 300, 0, 0, -1, 0),
	}
	reader := stubReader{poses: map[string]spatialmath.Pose{
		"arm-1": pose(105, 0, 300, 0, 0, -1, 0),
		"arm-2": pose(105, -500, 300, 0, 0, -1, 0),
	}}

	res, err := Measure(context.Background(), reader, []string{"arm-1", "arm-2"}, targets)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.Arms["arm-1"].PositionErrorMm, test.ShouldAlmostEqual, 5.0, 1e-6)
	test.That(t, res.Arms["arm-2"].PositionErrorMm, test.ShouldAlmostEqual, 5.0, 1e-6)
	test.That(t, res.Pairs["arm-1|arm-2"].PositionErrorMm, test.ShouldAlmostEqual, 0.0, 1e-6)
}

func TestMeasureRejectsEmptyArms(t *testing.T) {
	_, err := Measure(context.Background(), stubReader{}, nil, nil)
	test.That(t, err, test.ShouldEqual, errNoArms)
}

func TestMeasureRejectsMissingTarget(t *testing.T) {
	reader := stubReader{poses: map[string]spatialmath.Pose{"arm-1": pose(0, 0, 0, 0, 0, -1, 0)}}
	_, err := Measure(context.Background(), reader, []string{"arm-1"}, map[string]spatialmath.Pose{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm-1")
}

func TestResultToMapShape(t *testing.T) {
	r := &Result{
		Arms:  map[string]ArmError{"arm-1": {PositionErrorMm: 1.5, OrientationErrorDeg: 0.5}},
		Pairs: map[string]ArmError{"arm-1|arm-2": {PositionErrorMm: 2.0, OrientationErrorDeg: 1.0}},
	}
	m := r.ToMap()
	arms := m["arms"].(map[string]interface{})
	arm1 := arms["arm-1"].(map[string]interface{})
	test.That(t, arm1["position_error_mm"], test.ShouldEqual, 1.5)
	test.That(t, arm1["orientation_error_deg"], test.ShouldEqual, 0.5)
	pairs := m["pairs"].(map[string]interface{})
	pair := pairs["arm-1|arm-2"].(map[string]interface{})
	test.That(t, pair["position_error_mm"], test.ShouldEqual, 2.0)
}
