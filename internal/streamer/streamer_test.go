package streamer

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/test"
)

func makeTraj(n int) []arm.TrajectoryPoint {
	pts := make([]arm.TrajectoryPoint, n)
	for i := range pts {
		pts[i] = arm.TrajectoryPoint{Time: time.Duration(i*100) * time.Millisecond}
	}
	return pts
}

func TestStreamHappyPath(t *testing.T) {
	f := &Fake{}
	traj := makeTraj(5)

	err := Stream(context.Background(), f, traj)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(f.Received), test.ShouldEqual, 5)
}

func TestStreamRejectsFewerThanTwoWaypoints(t *testing.T) {
	err := Stream(context.Background(), &Fake{}, makeTraj(1))
	test.That(t, err, test.ShouldEqual, errAtLeastTwoWaypoints)
}

func TestStreamPropagatesArmError(t *testing.T) {
	boom := errors.New("boom")
	f := &Fake{Err: boom}

	err := Stream(context.Background(), f, makeTraj(3))
	test.That(t, err, test.ShouldEqual, boom)
}

func TestStreamRespectsContextCancellation(t *testing.T) {
	f := &Fake{}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	err := Stream(ctx, f, makeTraj(3))
	test.That(t, err, test.ShouldEqual, context.Canceled)
}
