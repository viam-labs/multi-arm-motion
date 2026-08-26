package sequencer

import "errors"

var errNegativeSleep = errors.New("sleep_seconds must be >= 0")
