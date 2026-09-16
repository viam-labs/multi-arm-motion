package barrier

import (
	"context"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	goutils "go.viam.com/utils"

	"github.com/viam-labs/multi-arm-motion/internal/streamer"
)

// Slack past the last trajectory point before Fire gives up on a wedged streamer.
const fireSlack = 10 * time.Second

// After cancel on timeout, wait this long for streamer goroutines to unwind before returning anyway.
const unwindGrace = 2 * time.Second

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

	waitDone := make(chan struct{})
	go func() {
		wg.Wait()
		close(waitDone)
	}()

	select {
	case <-waitDone:
		return firstErr
	case <-time.After(maxTrajectoryEnd(ops) + fireSlack):
		cancel()
		select {
		case <-waitDone:
		case <-time.After(unwindGrace):
		}
		return ErrFireTimeout
	}
}

func maxTrajectoryEnd(ops []Op) time.Duration {
	var max time.Duration
	for _, op := range ops {
		if n := len(op.Trajectory); n > 0 {
			if end := op.Trajectory[n-1].Time; end > max {
				max = end
			}
		}
	}
	return max
}
