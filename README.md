# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Improved orbital maneuver planning and execution for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

Validated against KSA build version 2026.2.30.3638.

## Features

### Hyperbolic Targets (done)

Enables the Transfer Planner to target objects on hyperbolic orbits (Oumuamua, 2I/Borisov, 3I/ATLAS, etc.).

Note: At the extreme dV required for interstellar objects, the impulsive burn approximation breaks down
and the patched-conic trajectory diverges ~0.7 AU from the Lambert prediction.
Mid-course corrections will be needed in practice.

### BetterBurnTime (planned)

Enhanced burn information display with time-to-ignition countdown, per-stage delta-V breakdown, multi-stage burn prediction, and fuel warnings.

### Oberth Maneuver / Multi Pass Burns (planned)

Automatically split large burns into multiple periapsis passes when burn duration is a significant fraction of the orbital period, preserving the Oberth effect.

## Installation

1. Install the following dependencies: [StarMap](https://github.com/StarMapLoader/StarMap) and [KittenExtensions](https://github.com/tsholmes/KittenExtensions).
2. Download the latest release from the [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases) tab.
3. Extract into `Kitten Space Agency\Content\AdvancedFlightComputer\`.
4. Add to `Kitten Space Agency\Content\manifest.toml`:

```toml
[[mods]]
id = "AdvancedFlightComputer"
enabled = true
```

## Dependencies

- [StarMap.API](https://github.com/StarMapLoader/StarMap) (NuGet)
- [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) (NuGet)
- [KittenExtensions](https://github.com/tsholmes/KittenExtensions) (for XML patching of hyperbolic body properties)
