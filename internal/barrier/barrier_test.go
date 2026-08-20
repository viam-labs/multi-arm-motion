package barrier

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/test"

	"github.com/viam-labs/multi-arm-motion/internal/streamer"
)

func makeTraj(n int) []arm.TrajectoryPoint {
	pts := make([]arm.TrajectoryPoint, n)
	for i := range pts {
		pts[i] = arm.TrajectoryPoint{Time: time.Duration(i*100) * time.Millisecond}
	}
	return pts
}

func TestFireHappyPath(t *testing.T) {
	a := &streamer.Fake{}
	b := &streamer.Fake{}
	ops := []Op{
		{Arm: a, Trajectory: makeTraj(3)},
		{Arm: b, Trajectory: makeTraj(3)},
	}

	err := Fire(context.Background(), ops)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(a.Received), test.ShouldEqual, 3)
	test.That(t, len(b.Received), test.ShouldEqual, 3)
}

func TestFireRejectsEmptyOps(t *testing.T) {
	err := Fire(context.Background(), nil)
	test.That(t, err, test.ShouldEqual, errNoOps)
}

func TestFirePropagatesArmError(t *testing.T) {
	boom := errors.New("boom")
	a := &streamer.Fake{Err: boom}
	b := &streamer.Fake{}
	ops := []Op{
		{Arm: a, Trajectory: makeTraj(3)},
		{Arm: b, Trajectory: makeTraj(3)},
	}

	err := Fire(context.Background(), ops)
	test.That(t, err, test.ShouldEqual, boom)
}

func TestFireRespectsParentContextCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	ops := []Op{
		{Arm: &streamer.Fake{}, Trajectory: makeTraj(3)},
		{Arm: &streamer.Fake{}, Trajectory: makeTraj(3)},
	}

	err := Fire(ctx, ops)
	test.That(t, err, test.ShouldEqual, context.Canceled)
}
