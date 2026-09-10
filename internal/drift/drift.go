package drift

import (
	"context"
	"fmt"
	"math"
	"sort"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

type ArmError struct {
	PositionErrorMm     float64 `json:"position_error_mm"`
	OrientationErrorDeg float64 `json:"orientation_error_deg"`
}

type Result struct {
	Arms  map[string]ArmError `json:"arms"`
	Pairs map[string]ArmError `json:"pairs"`
}

// PoseReader returns a fresh world-frame TCP pose for the named arm.
// Extracted as an interface so tests can supply canned poses without a framesystem.
type PoseReader interface {
	WorldPose(ctx context.Context, armName string) (spatialmath.Pose, error)
}

type fsReader struct{ svc framesystem.Service }

func (r fsReader) WorldPose(ctx context.Context, armName string) (spatialmath.Pose, error) {
	p, err := r.svc.TransformPose(ctx,
		referenceframe.NewPoseInFrame(armName, spatialmath.NewZeroPose()),
		referenceframe.World, nil)
	if err != nil {
		return nil, err
	}
	return p.Pose(), nil
}

func FramesystemReader(svc framesystem.Service) PoseReader { return fsReader{svc: svc} }

// Measure reads each arm's actual world-frame TCP pose and compares to its target.
// Emits per-arm absolute error and pairwise relative-pose error (the metric that
// matters when the arms are rigidly coupled — e.g., a shared sanding board).
func Measure(ctx context.Context, reader PoseReader, armNames []string, targets map[string]spatialmath.Pose) (*Result, error) {
	if len(armNames) < 1 {
		return nil, errNoArms
	}
	names := append([]string(nil), armNames...)
	sort.Strings(names)

	actual := make(map[string]spatialmath.Pose, len(names))
	for _, name := range names {
		p, err := reader.WorldPose(ctx, name)
		if err != nil {
			return nil, fmt.Errorf("arm %q: %w", name, err)
		}
		actual[name] = p
	}

	res := &Result{
		Arms:  make(map[string]ArmError, len(names)),
		Pairs: make(map[string]ArmError),
	}
	for _, name := range names {
		target, ok := targets[name]
		if !ok {
			return nil, fmt.Errorf("%w: %s", errArmMissingFromTargets, name)
		}
		res.Arms[name] = poseError(actual[name], target)
	}
	for i := 0; i < len(names); i++ {
		for j := i + 1; j < len(names); j++ {
			a, b := names[i], names[j]
			targetRel := spatialmath.PoseBetween(targets[a], targets[b])
			actualRel := spatialmath.PoseBetween(actual[a], actual[b])
			res.Pairs[a+"|"+b] = poseError(actualRel, targetRel)
		}
	}
	return res, nil
}

func poseError(actual, target spatialmath.Pose) ArmError {
	posErrMm := actual.Point().Sub(target.Point()).Norm()
	diff := spatialmath.OrientationBetween(actual.Orientation(), target.Orientation())
	oriErrDeg := math.Abs(diff.AxisAngles().Theta) * 180 / math.Pi
	return ArmError{PositionErrorMm: posErrMm, OrientationErrorDeg: oriErrDeg}
}

func (r *Result) LogAt(logger logging.Logger, component, mode string) {
	logger.Infow("drift measurement",
		"component", component,
		"mode", mode,
		"arms", r.Arms,
		"pairs", r.Pairs,
	)
}

func (r *Result) ToMap() map[string]interface{} {
	arms := make(map[string]interface{}, len(r.Arms))
	for k, v := range r.Arms {
		arms[k] = map[string]interface{}{
			"position_error_mm":     v.PositionErrorMm,
			"orientation_error_deg": v.OrientationErrorDeg,
		}
	}
	pairs := make(map[string]interface{}, len(r.Pairs))
	for k, v := range r.Pairs {
		pairs[k] = map[string]interface{}{
			"position_error_mm":     v.PositionErrorMm,
			"orientation_error_deg": v.OrientationErrorDeg,
		}
	}
	return map[string]interface{}{"arms": arms, "pairs": pairs}
}
