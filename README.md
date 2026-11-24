# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Maneuver planning, burn execution and closed-loop guidance for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

AFC adds one console for every feature. Its powered guidance flies an ascent to orbit, a launch to a flyby or an orbit of a moon, a booster boostback and a pinpoint powered landing.

**Documentation: [https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/)**

Read the [User Guide](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/) to install and fly with AFC, or the [Developer section](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/dev/) for the architecture and guidance maths.

## Features

- The [AFC console](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/console/), one window for every AFC feature, with tabs in the order of a mission.
- [Maneuver quick-tools](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/): circularize, set periapsis or apoapsis, match or set inclination.
- [Transfers and flyby targeting](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/#flyby-targeting): AFC porkchop plots, direct flyby departure burns and moon course corrections.
- [Multi-pass burns](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/#multi-pass-burns) for Oberth-efficient departures.
- [RCS-only burn execution](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#rcs-translation-burns) and [warp to the turn](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#warp-to-the-turn).
- [Automatic staging](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#automatic-staging) with spent-booster drop, payload fairing release and configurable staging delays, plus [automatic burn removal](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/burns-and-staging/#automatic-burn-removal).
- [Hyperbolic targets](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/maneuvers/#hyperbolic-targets): target interstellar comets (Oumuamua, 2I/Borisov, 3I/ATLAS).
- [Powered guidance](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/powered-guidance/): ascent, [launch to a moon](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/powered-guidance/#launch-to-a-moon), boostback, atmospheric entry and powered landing, with [landing sites from other mods](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/landing-sites/).

Written against the [StarMap loader](https://github.com/StarMapLoader/StarMap). Validated against KSA build `2026.11.16.5901`.

## Quick installation

1. Install [StarMap](https://github.com/StarMapLoader/StarMap) (`0.5.4` or later).
2. Download the latest release zip from [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases).
3. Extract it into `Documents\My Games\Kitten Space Agency\mods\` (or your launcher instance's `mods\` folder). The zip already contains the `AdvancedFlightComputer` folder.
4. Enable `AdvancedFlightComputer` when the game prompts you, or in `manifest.toml`.

> **Upgrading from standalone mods:** AutoStage, AutoRemoveFinishedBurns and PoweredGuidance are built into AFC. Delete their old folders under `mods\` before launching the game.

See [Installation](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/installation/) and [Compatibility and credits](https://maximilian-nesslauer.github.io/KSA-AdvancedFlightComputer/user-guide/about/) for profile paths, `manifest.toml` syntax and compatibility notes.

## Building from source

Targets **.NET 10** with `StarMap.API` (`0.5.4`) and `Lib.Harmony` (`2.4.3`) from NuGet. The native guidance solvers (`clarabel_c` and `scs`) are built from vendored sources under `third_party/` with Rust and Zig; see [docs/guidance/native-build.md](docs/guidance/native-build.md).

## Credits and license

- **Oliver Cairns ([cairn5](https://github.com/cairn5))** wrote [PoweredGuidance](https://github.com/cairn5/PoweredGuidance), the powered guidance inside AFC, and is co-author of the project.
- **[PEGAS](https://github.com/Noiredd/PEGAS)** by Noiredd is the reference for the UPFG ascent implementation ([`third_party/pegas/LICENSE`](third_party/pegas/LICENSE)).
- **[Clarabel](https://github.com/oxfordcontrol/Clarabel.rs)** and **[SCS](https://github.com/cvxgrp/scs)** are the conic solvers behind G-FOLD, convex ascent, entry guidance and touchdown guidance.

Licensed under MIT - see [`LICENSE`](LICENSE) and [`THIRD-PARTY-NOTICES.md`](THIRD-PARTY-NOTICES.md). Nothing in the current tree is GPL.

## Community

- KSA forum thread: https://forums.ahwoo.com/threads/advanced-flight-computer.783/
- Other mods: [DeltaVMap](https://github.com/Maximilian-Nesslauer/KSA-DeltaVMap) ([forum](https://forums.ahwoo.com/threads/deltavmap.978/)), [MeasureTools](https://github.com/Maximilian-Nesslauer/KSA-MeasureTools) ([forum](https://forums.ahwoo.com/threads/measuretools.992/))
