// Package main runs the multi-arm-motion Viam module.
package main

import (
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"

	"github.com/viam-labs/multi-arm-motion/group"
	"github.com/viam-labs/multi-arm-motion/preset"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: group.Model},
		resource.APIModel{API: toggleswitch.API, Model: preset.Model},
	)
}
