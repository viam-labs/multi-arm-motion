# CLAUDE.md

## Working in this repo

- Default to no code comments; add terse one-liners only when the WHY (not the WHAT) is non-obvious from the code.
- Do NOT bandaid over upstream bugs or platform gaps in any dependency — raise them immediately with a severity (critical / high / medium / low).

## Platform shortcomings

Known gaps this module cannot fix on its own. Do not paper over them — reference this section when you hit them and, if new ones surface, add them here.

- **Inter-arm collision detection during simultaneous motion (severity: high).** RDK's `armplanning.PlanMotion` is single-arm: when called for one arm, other arms in the framesystem are treated as static obstacles at their **starting** joint configuration. This module plans each arm independently and then executes them concurrently, so mid-trajectory the arms can enter each other's swept volume even when start and end states are individually collision-free. A proper fix needs a coordinated multi-arm planner (planning in the joint concatenation of all arms), which RDK does not currently expose.
