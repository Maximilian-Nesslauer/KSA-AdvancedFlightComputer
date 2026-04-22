# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Extra maneuver planning tools for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

Adds set periapsis / set apoapsis / match inclination / set inclination quick-tools to the Transfer Planner, and enables the planner to target interstellar comets on hyperbolic orbits (Oumuamua, 2I/Borisov, 3I/ATLAS).

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

Validated against KSA build version 2026.4.16.4170.

## Features

- **Set Periapsis / Set Apoapsis** - single prograde or retrograde burn at the opposite apse, raising or lowering one apse to a target altitude.
- **Match Inclination** - plane-change burn at AN or DN to align your orbit with a target orbit's plane.
- **Set Inclination** - plane-change burn at AN or DN to set an absolute inclination angle. The reference plane is selectable: **Ecliptic** (matches `Orbit.Inclination`, KSA's system-wide inertial Z) or **Equatorial** (parent body's equator, standard astrodynamics convention). For Earth the two differ by the ~23.4 degree obliquity.
- **Hyperbolic Targets** - the stock Transfer Planner filters out bodies with eccentricity >= 1. This mod lets it target interstellar comets by injecting Mass and SphereOfInfluence for Oumuamua, 2I/Borisov, and 3I/ATLAS, and patching the planner's time-of-flight / alignment math to handle unbound orbits.

## Installation

1. Install [StarMap](https://github.com/StarMapLoader/StarMap) and [KittenExtensions](https://github.com/tsholmes/KittenExtensions) (the latter is only required if you want hyperbolic targets).
2. Download the latest release from the [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases) tab.
3. Extract into `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\`.
4. The game auto-discovers new mods and prompts you to enable them. Alternatively, add to `Documents\My Games\Kitten Space Agency\manifest.toml`:

```toml
[[mods]]
id = "AdvancedFlightComputer"
enabled = true
```

## Dependencies

| Package | Purpose | Tested version |
| --- | --- | --- |
| [StarMap](https://github.com/StarMapLoader/StarMap) | Mod loader, required at runtime (see [Installation](#installation)) | 0.4.5 |
| [KittenExtensions](https://github.com/tsholmes/KittenExtensions) | Optional, required at runtime for the hyperbolic-targets XML patch | v0.4.0 |

## Build dependencies

Required only to build the mod from source. Targets **.NET 10**.

| Package | Source | Tested Version |
| --- | --- | --- |
| [StarMap.API](https://github.com/StarMapLoader/StarMap) | NuGet | 0.3.6 |
| [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) | NuGet | 2.4.2 |

## Mod compatibility

- Known conflicts: none

## Check out my other mods

- [AutoStage](https://github.com/Maximilian-Nesslauer/KSA-AutoStage) - automatic staging during auto-burns and manual flight, with configurable ignition delays ([forum thread](https://forums.ahwoo.com/threads/autostage.891/))
- [StageInfo](https://github.com/Maximilian-Nesslauer/KSA-StageInfo) - per-sequence dV, TWR, burn time, and ISP in the stage info panel ([forum thread](https://forums.ahwoo.com/threads/stageinfo.905/))
