package barrier

import (
	"context"
	"sync"

	"go.viam.com/rdk/components/arm"
	goutils "go.viam.com/utils"

	"github.com/viam-labs/multi-arm-motion/internal/streamer"
)

type Op struct {
	Arm        streamer.ArmStream
	Trajectory []arm.TrajectoryPoint
}

func Fire(ctx context.Context, ops []Op) error {
	if len(ops) == 0 {
		return errNoOps
	}

	ctx, cancel := context.WithCancel(ctx)
	defer cancel()

	signal := make(chan struct{})
	var firstErr error
	var errOnce sync.Once
	setErr := func(e error) {
		errOnce.Do(func() { firstErr = e })
	}

	var wg sync.WaitGroup
	for _, op := range ops {
		wg.Add(1)
		o := op
		goutils.PanicCapturingGo(func() {
			defer wg.Done()
			<-signal
			if err := streamer.Stream(ctx, o.Arm, o.Trajectory); err != nil {
				setErr(err)
				cancel()
			}
		})
	}

	close(signal)
	wg.Wait()

	return firstErr
}
