# multi-arm-motion

Viam module for coordinated motion across two or more arms.

## Why this module?

When you want two or more arms to move together, RDK gives you N independent arm handles. This module wraps them so all arms start and end their motion in lockstep — kicking off trajectories through a shared barrier and sizing every arm's trajectory to a shared duration.

## Models

### `viam:multi-arm-motion:group`

A generic service that treats N arms as one motion target.

- **jog**: incremental synchronized move
- **execute**: play a pre-computed multi-arm trajectory

Config:

```json
{
  "name": "arms",
  "api": "rdk:service:generic",
  "model": "viam:multi-arm-motion:group",
  "attributes": {
    "arms": ["arm-1", "arm-2"],
    "max_joint_vel_degs_per_sec": 30,
    "waypoint_spacing_ms": 20
  }
}
```

DoCommand — `jog`:

```json
{"jog": {"delta": {"x": 10, "y": 0, "z": 0}}}
```

Translation in millimeters, applied to every arm's TCP in world coordinates.

DoCommand — `execute`:

```json
{
  "execute": {
    "trajectories": {
      "arm-1": [
        {"time_ms": 0,  "positions": [0, 0, 0, 0, 0, 0]},
        {"time_ms": 20, "positions": [0.1, 0, 0, 0, 0, 0]}
      ],
      "arm-2": [
        {"time_ms": 0,  "positions": [0, 0, 0, 0, 0, 0]},
        {"time_ms": 20, "positions": [0.1, 0, 0, 0, 0, 0]}
      ]
    }
  }
}
```

Each arm's trajectory must start at `time_ms: 0` and have strictly increasing timestamps. Positions are joint angles in radians.

### `viam:multi-arm-motion:preset`

A switch component that snaps a set of arms to saved joint positions in sync.

- Position 1: update saved joints from current arm state
- Position 2: go to saved joints

Config:

```json
{
  "name": "arms_home",
  "api": "rdk:component:switch",
  "model": "viam:multi-arm-motion:preset",
  "attributes": {
    "arms": ["arm-1", "arm-2"],
    "joints": {
      "arm-1": [0, 0, 0, 0, 0, 0],
      "arm-2": [0, 0, 0, 0, 0, 0]
    }
  }
}
```

## Related

Pair with [multi-arm-calibration](https://github.com/viam-labs/multi-arm-calibration) to register two arms to each other before coordinated motion.
