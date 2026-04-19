# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **Status: outdated.** Last validated against KSA v2026.2.38.3713. The current codebase does not build against recent game versions due to several game updates. I will update / fix AFC once i have more time. The Stage Info portion of this mod has moved to its own [StageInfo mod](https://github.com/Maximilian-Nesslauer/KSA-StageInfo) and is actively maintained there.

Improved orbital maneuver planning and execution for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

## Features

### Hyperbolic Targets

Enables the Transfer Planner to target objects on hyperbolic orbits (Oumuamua, 2I/Borisov, 3I/ATLAS, etc.).

Note: At the extreme dV required for interstellar objects, the impulsive burn approximation breaks down
and the patched-conic trajectory diverges ~0.7 AU from the Lambert prediction.
Mid-course corrections will be needed in practice.

### Stage Info Display

Moved to its own mod: [StageInfo](https://github.com/Maximilian-Nesslauer/KSA-StageInfo).

### Finite Burn Correction (planned)

Compensate for finite burn losses by iteratively correcting the planned Delta V. Long burns (ion engines, weak upper stages) lose efficiency because the spacecraft drifts off the optimal trajectory during the burn. The mod simulates the actual burn with numerical integration and adjusts the maneuver node accordingly.

### Oberth Maneuver / Multi Pass Burns (planned)

Automatically split large burns into multiple periapsis passes when burn duration is a significant fraction of the orbital period, preserving the Oberth effect.

## Installation

1. Install the following dependencies: [StarMap](https://github.com/StarMapLoader/StarMap) and [KittenExtensions](https://github.com/tsholmes/KittenExtensions).
2. Download the latest release from the [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases) tab.
3. Extract into `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\`.
4. The game auto-discovers new mods and prompts you to enable them. Alternatively, add to `Documents\My Games\Kitten Space Agency\manifest.toml`:

```toml
[[mods]]
id = "AdvancedFlightComputer"
enabled = true
```

## Dependencies

- [StarMap.API](https://github.com/StarMapLoader/StarMap) (NuGet)
- [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) (NuGet)
- [KittenExtensions](https://github.com/tsholmes/KittenExtensions) (for XML patching of hyperbolic body properties)
