package sequencer

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/test"
)

func TestValidateHappyPath(t *testing.T) {
	cfg := &Config{Positions: []string{"a", "b"}}
	deps, _, err := cfg.Validate("sequencer")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"a", "b"})
}

func TestValidateRejectsEmptyPositions(t *testing.T) {
	cfg := &Config{}
	_, _, err := cfg.Validate("sequencer")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "positions")
}

func TestValidateRejectsEmptyPositionName(t *testing.T) {
	cfg := &Config{Positions: []string{"a", ""}}
	_, _, err := cfg.Validate("sequencer")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "positions[1]")
}

func TestValidateRejectsNegativeSleep(t *testing.T) {
	cfg := &Config{Positions: []string{"a"}, SleepSeconds: -1}
	_, _, err := cfg.Validate("sequencer")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "sleep_seconds")
}

func TestSleepDefaults(t *testing.T) {
	test.That(t, (&Config{}).sleep(), test.ShouldEqual, time.Duration(0))
	test.That(t, (&Config{SleepSeconds: 0.5}).sleep(), test.ShouldEqual, 500*time.Millisecond)
}

type fakeSwitch struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	mu       sync.Mutex
	name     string
	setCalls []uint32
	err      error
	delay    time.Duration
}

func newFakeSwitch(name string) *fakeSwitch {
	return &fakeSwitch{
		Named: resource.NewName(toggleswitch.API, name).AsNamed(),
		name:  name,
	}
}

func (f *fakeSwitch) SetPosition(ctx context.Context, position uint32, _ map[string]interface{}) error {
	f.mu.Lock()
	f.setCalls = append(f.setCalls, position)
	err := f.err
	delay := f.delay
	f.mu.Unlock()
	if delay > 0 {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(delay):
		}
	}
	return err
}

func (f *fakeSwitch) GetPosition(_ context.Context, _ map[string]interface{}) (uint32, error) {
	return 0, nil
}

func (f *fakeSwitch) GetNumberOfPositions(_ context.Context, _ map[string]interface{}) (uint32, []string, error) {
	return 3, []string{"idle", "update config", "go to"}, nil
}

func (f *fakeSwitch) calls() []uint32 {
	f.mu.Lock()
	defer f.mu.Unlock()
	out := make([]uint32, len(f.setCalls))
	copy(out, f.setCalls)
	return out
}

func newService(cfg *Config, switches []toggleswitch.Switch, names []string) *service {
	return &service{
		Named:     resource.NewName(resource.NewAPI("test", "test", "test"), "seq").AsNamed(),
		logger:    logging.NewTestLogger(&testing.T{}),
		cfg:       cfg,
		positions: switches,
		posOrder:  names,
	}
}

func TestPushSetsEachSwitchToGoInOrder(t *testing.T) {
	a := newFakeSwitch("a")
	b := newFakeSwitch("b")
	c := newFakeSwitch("c")
	s := newService(&Config{Positions: []string{"a", "b", "c"}},
		[]toggleswitch.Switch{a, b, c},
		[]string{"a", "b", "c"})

	err := s.Push(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, a.calls(), test.ShouldResemble, []uint32{goToPosition})
	test.That(t, b.calls(), test.ShouldResemble, []uint32{goToPosition})
	test.That(t, c.calls(), test.ShouldResemble, []uint32{goToPosition})
}

func TestPushStopsOnFirstError(t *testing.T) {
	a := newFakeSwitch("a")
	b := newFakeSwitch("b")
	b.err = errors.New("boom")
	c := newFakeSwitch("c")
	s := newService(&Config{Positions: []string{"a", "b", "c"}},
		[]toggleswitch.Switch{a, b, c},
		[]string{"a", "b", "c"})

	err := s.Push(context.Background(), nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "step 2")
	test.That(t, err.Error(), test.ShouldContainSubstring, `"b"`)
	test.That(t, a.calls(), test.ShouldResemble, []uint32{goToPosition})
	test.That(t, b.calls(), test.ShouldResemble, []uint32{goToPosition})
	test.That(t, c.calls(), test.ShouldBeEmpty)
}

func TestPushSleepsBetweenSteps(t *testing.T) {
	a := newFakeSwitch("a")
	b := newFakeSwitch("b")
	s := newService(&Config{Positions: []string{"a", "b"}, SleepSeconds: 0.05},
		[]toggleswitch.Switch{a, b},
		[]string{"a", "b"})

	start := time.Now()
	err := s.Push(context.Background(), nil)
	elapsed := time.Since(start)

	test.That(t, err, test.ShouldBeNil)
	test.That(t, elapsed, test.ShouldBeGreaterThanOrEqualTo, 50*time.Millisecond)
}

func TestPushDoesNotSleepAfterLastStep(t *testing.T) {
	a := newFakeSwitch("a")
	s := newService(&Config{Positions: []string{"a"}, SleepSeconds: 5.0},
		[]toggleswitch.Switch{a},
		[]string{"a"})

	start := time.Now()
	err := s.Push(context.Background(), nil)
	elapsed := time.Since(start)

	test.That(t, err, test.ShouldBeNil)
	test.That(t, elapsed, test.ShouldBeLessThan, time.Second)
}

func TestPushRespectsContextCancellationDuringSleep(t *testing.T) {
	a := newFakeSwitch("a")
	b := newFakeSwitch("b")
	s := newService(&Config{Positions: []string{"a", "b"}, SleepSeconds: 10.0},
		[]toggleswitch.Switch{a, b},
		[]string{"a", "b"})

	ctx, cancel := context.WithCancel(context.Background())
	go func() {
		time.Sleep(30 * time.Millisecond)
		cancel()
	}()

	start := time.Now()
	err := s.Push(ctx, nil)
	elapsed := time.Since(start)

	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, elapsed, test.ShouldBeLessThan, time.Second)
	test.That(t, a.calls(), test.ShouldResemble, []uint32{goToPosition})
	test.That(t, b.calls(), test.ShouldBeEmpty)
}
