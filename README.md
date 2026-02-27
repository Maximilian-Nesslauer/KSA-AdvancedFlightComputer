# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Improved orbital maneuver planning and execution for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

Validated against KSA build version 2026.2.38.3713.

## Features

### Hyperbolic Targets

Enables the Transfer Planner to target objects on hyperbolic orbits (Oumuamua, 2I/Borisov, 3I/ATLAS, etc.).

Note: At the extreme dV required for interstellar objects, the impulsive burn approximation breaks down
and the patched-conic trajectory diverges ~0.7 AU from the Lambert prediction.
Mid-course corrections will be needed in practice.

### Auto-Staging

Adds an AUTOSTAGE toggle button to the BurnControl gauge panel. When enabled, automatically activates the next stage whenever engines run out of propellant. Works during auto-burns (continues the burn instead of aborting) and also manual burns.

### Stage Info Display

Extends the Staging window with per-stage information:

- Fuel progress bar with percentage on each stage header
- Delta V, TWR, Burn Time, and ISP (visible when a stage is expanded)
- Total Delta V and Burn Time in a footer below the stage list
- Display mode selector: Auto (uses current atmospheric conditions), VAC, ASL, VAC + ASL (dual display), and Planning (choose any celestial body for surface gravity and atmospheric pressure)

### Burn-Stage Analysis

When a burn is planned, the Staging window shows burn-aware information:

- Per-stage burn allocation with color-coded gradient (green = barely used, yellow = half consumed, red = fully consumed)
- "INSUFFICIENT" warning when planned Delta V exceeds available Delta V
- Corrected multi-stage burn duration on the BURN TIME and START BURN IN gauge rollers (the game only computes single-stage burn time)

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
