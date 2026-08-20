package barrier

import "errors"

var errNoOps = errors.New("barrier requires at least one op")
