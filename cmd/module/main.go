// Package main runs the multi-arm-motion Viam module.
package main

import (
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"

	"github.com/viam-labs/multi-arm-motion/group"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: group.Model},
	)
}
