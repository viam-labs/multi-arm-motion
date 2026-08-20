package streamer

import (
	"context"

	"go.viam.com/rdk/components/arm"
)

type ArmStream interface {
	MoveThroughJointPositionsStreamed(
		ctx context.Context,
		batches <-chan []arm.TrajectoryPoint,
		responses chan<- arm.Response,
		extra map[string]interface{},
	) error
}

func Stream(ctx context.Context, a ArmStream, traj []arm.TrajectoryPoint) error {
	if len(traj) < 2 {
		return errAtLeastTwoWaypoints
	}

	batches := make(chan []arm.TrajectoryPoint, 1)
	responses := make(chan arm.Response, 64)

	done := make(chan struct{})
	var callErr error
	go func() {
		callErr = a.MoveThroughJointPositionsStreamed(ctx, batches, responses, nil)
		close(done)
	}()

	drainDone := make(chan struct{})
	go func() {
		for range responses {
		}
		close(drainDone)
	}()

	batches <- traj
	close(batches)

	<-done
	close(responses)
	<-drainDone

	return callErr
}
