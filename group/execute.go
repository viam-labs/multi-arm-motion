package group

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"

	"github.com/viam-labs/multi-arm-motion/internal/barrier"
)

func (s *service) Execute(ctx context.Context, trajectories map[string][]arm.TrajectoryPoint) error {
	for _, name := range s.armOrder {
		if _, ok := trajectories[name]; !ok {
			return fmt.Errorf("execute: missing trajectory for arm %q", name)
		}
	}
	for name := range trajectories {
		if _, ok := s.arms[name]; !ok {
			return fmt.Errorf("execute: trajectory for unknown arm %q", name)
		}
	}

	ops := make([]barrier.Op, 0, len(s.armOrder))
	for _, name := range s.armOrder {
		ops = append(ops, barrier.Op{Arm: s.arms[name], Trajectory: trajectories[name]})
	}
	return barrier.Fire(ctx, ops)
}

func parseExecute(raw interface{}) (map[string][]arm.TrajectoryPoint, error) {
	m, ok := raw.(map[string]interface{})
	if !ok {
		return nil, errMissingExecuteTrajectories
	}
	trajsRaw, ok := m["trajectories"].(map[string]interface{})
	if !ok {
		return nil, errMissingExecuteTrajectories
	}

	out := make(map[string][]arm.TrajectoryPoint, len(trajsRaw))
	for armName, trajRaw := range trajsRaw {
		waypointsRaw, ok := trajRaw.([]interface{})
		if !ok {
			return nil, fmt.Errorf("arm %q: trajectory must be an array", armName)
		}
		if len(waypointsRaw) < 2 {
			return nil, fmt.Errorf("arm %q: trajectory must have at least 2 waypoints", armName)
		}
		waypoints := make([]arm.TrajectoryPoint, 0, len(waypointsRaw))
		var prev time.Duration = -1
		for i, wRaw := range waypointsRaw {
			w, ok := wRaw.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("arm %q waypoint %d: must be an object", armName, i)
			}
			tMs, ok := w["time_ms"].(float64)
			if !ok {
				return nil, fmt.Errorf("arm %q waypoint %d: time_ms must be a number", armName, i)
			}
			t := time.Duration(tMs) * time.Millisecond
			if t < 0 {
				return nil, fmt.Errorf("arm %q waypoint %d: time_ms must be non-negative", armName, i)
			}
			if t <= prev {
				return nil, fmt.Errorf("arm %q waypoint %d: time_ms must strictly increase", armName, i)
			}
			if i == 0 && t != 0 {
				return nil, fmt.Errorf("arm %q: first waypoint must have time_ms=0", armName)
			}
			posRaw, ok := w["positions"].([]interface{})
			if !ok {
				return nil, fmt.Errorf("arm %q waypoint %d: positions must be an array", armName, i)
			}
			positions := make([]referenceframe.Input, len(posRaw))
			for j, pRaw := range posRaw {
				p, ok := pRaw.(float64)
				if !ok {
					return nil, fmt.Errorf("arm %q waypoint %d position %d: must be a number", armName, i, j)
				}
				positions[j] = p
			}
			waypoints = append(waypoints, arm.TrajectoryPoint{Time: t, Positions: positions})
			prev = t
		}
		out[armName] = waypoints
	}
	return out, nil
}
