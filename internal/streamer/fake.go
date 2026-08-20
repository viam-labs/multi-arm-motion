package streamer

import (
	"context"

	"go.viam.com/rdk/components/arm"
)

type Fake struct {
	Received []arm.TrajectoryPoint
	Err      error
}

func (f *Fake) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	_ map[string]interface{},
) error {
	for {
		select {
		case batch, ok := <-batches:
			if !ok {
				return f.Err
			}
			f.Received = append(f.Received, batch...)
			responses <- arm.Response{}
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}
