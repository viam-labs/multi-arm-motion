package barrier

import "errors"

var errNoOps = errors.New("barrier requires at least one op")

// ErrFireTimeout is returned when Fire's bounded wait elapses.
var ErrFireTimeout = errors.New("barrier fire timed out")
