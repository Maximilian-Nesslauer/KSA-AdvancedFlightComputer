# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Maneuver planning, burn execution and closed-loop guidance for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

AFC adds one console for every feature. Its powered guidance flies an ascent to orbit, a launch to a flyby or an orbit of a moon, a booster boostback and a pinpoint powered landing.

**Documentation: [https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/)**

The manual for players is the [User Guide](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/), and the Developer section explains how AFC works inside.

## Features

- The [AFC console](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/console/), one window for every AFC feature, with one row of tabs that follows the order of a mission.
- [Maneuver quick-tools](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/): circularize, set periapsis or apoapsis, match or set inclination.
- [Flyby targeting](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/#flyby-targeting), so a Hohmann transfer arrives as a flyby instead of an impact.
- [Multi-pass burns](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/#multi-pass-burns) for Oberth-efficient departures.
- [RCS-only burn execution](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#rcs-translation-burns) and a [warp to the turn](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#warp-to-the-turn).
- [Automatic staging](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#automatic-staging) with spent-booster drop and configurable staging delays, and [automatic removal of finished burns](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#automatic-burn-removal).
- [Hyperbolic targets](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/#hyperbolic-targets): the planner can target interstellar comets (Oumuamua, 2I/Borisov, 3I/ATLAS).
- [Powered guidance](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/powered-guidance/): ascent, [launch to a moon](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/powered-guidance/#launch-to-a-moon), boostback and powered landing, with [landing sites from other mods](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/landing-sites/).

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

Validated against KSA build version 2026.11.25.5949.

## Installation

1. Install [StarMap](https://github.com/StarMapLoader/StarMap).
2. Download the latest release from the [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases) tab.
3. Extract the zip into `Documents\My Games\Kitten Space Agency\mods\`. It already contains the `AdvancedFlightComputer` folder.
4. The game auto-discovers new mods and prompts you to enable them. Alternatively, add to `Documents\My Games\Kitten Space Agency\manifest.toml`:

```toml
[[mods]]
id = "AdvancedFlightComputer"
enabled = true
```

AFC reads and writes its files (settings, staging delays, multi-pass plans, RCS options and logs) in the profile folder that the game runs in. When a launcher starts the game with its own profile folder (an instance), the mod and its files are in that folder instead of `Documents\My Games\Kitten Space Agency\`.

## Dependencies

| Package | Purpose | Tested version |
| --- | --- | --- |
| [StarMap](https://github.com/StarMapLoader/StarMap) | Mod loader, 0.5.4 or later required at runtime (see [Installation](#installation)) | 0.5.4 |

## Build dependencies

Required only to build the mod from source. Targets **.NET 10**.

| Package | Source | Tested Version |
| --- | --- | --- |
| [StarMap.API](https://github.com/StarMapLoader/StarMap) | NuGet | 0.5.4 |
| [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) | NuGet | 2.4.2 |

The native guidance solvers are built from the vendored sources with Rust and Zig, see [native-build.md](docs/guidance/native-build.md).

## Mod compatibility

- [AutoStage](https://github.com/Maximilian-Nesslauer/KSA-AutoStage), [AutoRemoveFinishedBurns](https://github.com/Maximilian-Nesslauer/KSA-AutoRemoveFinishedBurns) and [PoweredGuidance](https://github.com/cairn5/PoweredGuidance) are part of this mod. Delete their folders under `mods`, because KittenExtensions Continued applies the XML patches of a mod also when the mod is disabled.
- The AFC menu is in the top bar.
- Known conflicts: the standalone AutoStage and AutoRemoveFinishedBurns mods.

## Credits

- **Oliver Cairns ([cairn5](https://github.com/cairn5))** wrote [PoweredGuidance](https://github.com/cairn5/PoweredGuidance), which is the powered guidance in this mod: the UPFG ascent and its landing mode, G-FOLD, the CAT-S successive-convexification 6-DOF guidance, boostback, terminal hover, returnable stages, the impact prediction and the guidance panel. Its full history is part of this repository, and he is co-author of the project.
- **[PEGAS](https://github.com/Noiredd/PEGAS)** by Noiredd is the reference and foundation for the UPFG implementation; its MIT license is kept at [`third_party/pegas/LICENSE`](third_party/pegas/LICENSE).
- **G-FOLD** follows Acikmese and Ploen, *Convex Programming Approach to Powered Descent Guidance for Mars Landing* (2007), and Acikmese, Carson and Blackmore, *Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem* (2013).
- **[Clarabel](https://github.com/oxfordcontrol/Clarabel.rs)** by Goulart and Chen and **[SCS](https://github.com/cvxgrp/scs)** by O'Donoghue are the conic solvers behind G-FOLD and the 6-DOF guidance.

## License

MIT - see [`LICENSE`](LICENSE). Vendored code under `third_party/` keeps its own upstream license.

PoweredGuidance releases up to and including **v0.3.1** linked [ECOS](https://github.com/embotech/ecos), which is GPLv3, and were therefore distributed under GPLv3 as a whole. Those releases stay available under GPLv3, but ECOS was replaced by Clarabel (Apache-2.0) before the import, so **nothing in the current tree is GPL**.

Attribution for the vendored solvers and their own transitive dependencies is in [`THIRD-PARTY-NOTICES.md`](THIRD-PARTY-NOTICES.md); all of them are permissive, and none impose copyleft on this work.

## Community

Thread on the KSA forums: https://forums.ahwoo.com/threads/advanced-flight-computer.783/

## Check out my other mods

- [DeltaVMap](https://github.com/Maximilian-Nesslauer/KSA-DeltaVMap) - interactive delta-v subway map and transfer-window planner, auto-generated from the loaded system ([forum thread](https://forums.ahwoo.com/threads/deltavmap.978/))
- [MeasureTools](https://github.com/Maximilian-Nesslauer/KSA-MeasureTools) - click-to-measure ruler, protractor, and surface measuring in the map view ([forum thread](https://forums.ahwoo.com/threads/measuretools.992/))
