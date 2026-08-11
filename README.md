# AutoRemoveFinishedBurns [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Auto-remove finished burns from the burn plan in [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

In stock KSA, when an auto-burn completes the flight computer flips the
burn mode to Manual but leaves the burn entry in the plan. You then have
to click "Delete" manually before the next maneuver can take focus. This
mod cleans up completed auto-burns automatically.

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

Validated against KSA build version 2026.8.19.5261.

## Features

- **Auto-removes finished auto-burns** from the burn plan as soon as the
  flight computer flips out of Auto mode on completion.
- **Out-of-fuel safe**: completion is confirmed via the same delta-V
  vector reversal the stock flight computer uses internally, so a burn
  that flamed out before reaching its target stays in the plan and can
  be resumed after staging.
- **Auto-only**: manual burns are never touched. Players keep full
  control to fine-tune by hand.
- **AdvancedFlightComputer RCS burns**: burns executed by
  [AdvancedFlightComputer](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer)'s
  RCS translation feature complete without the stock Auto-mode
  transition, so this mod listens to AFC's completion event instead
  (soft dependency: without AFC installed nothing changes).
- **In-game toggle** in the Mods settings tab. Setting is persisted to
  a TOML file in the mod's user directory.

## Installation

1. Install [StarMap](https://github.com/StarMapLoader/StarMap).
2. Download the latest release from the [GitHub Releases](https://github.com/Maximilian-Nesslauer/KSA-AutoRemoveFinishedBurns/releases) tab or from [SpaceDock](https://spacedock.info/mod/4255/AutoRemoveFinishedBurns).
3. Extract into `Documents\My Games\Kitten Space Agency\mods\AutoRemoveFinishedBurns\`.
4. The game auto-discovers new mods and prompts you to enable them. Alternatively, add to `Documents\My Games\Kitten Space Agency\manifest.toml`:

```toml
[[mods]]
id = "AutoRemoveFinishedBurns"
enabled = true
```

## Dependencies

| Package | Purpose | Tested version |
| --- | --- | --- |
| [StarMap](https://github.com/StarMapLoader/StarMap) | Mod loader, required at runtime (see [Installation](#installation)) | 0.4.6 |

## Build dependencies

Required only to build the mod from source. Targets **.NET 10**.

| Package | Source | Tested Version |
| --- | --- | --- |
| [StarMap.API](https://github.com/StarMapLoader/StarMap) | NuGet | 0.3.6 |
| [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) | NuGet | 2.4.2 |

## Testing

`AutoRemoveFinishedBurns.HarnessTests/` is a developer-only test suite for [HeadlessHarness](https://github.com/Maximilian-Nesslauer/KSA-HeadlessHarness), which brings the real game up GPU-free and runs plug-in tests against the live simulation:

- `arfb-api-drift` checks every reflection target and the IL anchor of the settings transpiler against the current game build, so an update that breaks the mod is caught without launching the full game.
- `arfb-burn-removal` spawns a vehicle, adds a real burn through the game's input queue, and drives the real flight computer through the Auto -> Manual transition: a completed auto-burn is removed, while out-of-fuel, disabled-setting, manual-mode, zero-delta-V-insert, and uncontrolled-vehicle cases keep the burn.
- `arfb-rcs-interop` binds to AdvancedFlightComputer's RCS completion event and checks the removal policy on it. It skips when AFC is not deployed.

To run it: build this solution and the HeadlessHarness repo, checked out as a sibling of this one (their `CopyToMods` targets deploy everything), then run the harness's `scripts/run-headless.ps1` (optionally with a `-Tests` name filter). Leave the deployed test mod disabled for normal play; it only does anything inside a harness run and is not part of the released mod.

## Mod compatibility

- Known conflicts: none

## Notes

- Detection uses the delta-V vector reversal signal the stock flight
  computer already evaluates each tick to flip Auto -> Manual on burn
  completion. If the vector hasn't reversed (you ran out of fuel before
  reaching the target) the burn entry is preserved.
- Only the vehicle you control is watched. A burn that finishes on a
  background vehicle stays in its plan, and taking control afterwards does
  not clean it up, because the completion already happened. Delete that one
  by hand.
- The toggle is persisted to
  `Documents\My Games\Kitten Space Agency\mods\AutoRemoveFinishedBurns\autoremovefinishedburns.toml`.

## Community

Thread on the KSA forums: https://forums.ahwoo.com/threads/autoremovefinishedburns.928/

## Check out my other mods

- [AdvancedFlightComputer](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer) - Transfer Planner quick-tools (set Pe/Ap, match/set inclination, circularize), multi-pass burn splitting, and hyperbolic-target support (Oumuamua, 2I/Borisov, 3I/ATLAS) ([forum thread](https://forums.ahwoo.com/threads/advanced-flight-computer.783/))
- [AutoStage](https://github.com/Maximilian-Nesslauer/KSA-AutoStage) - automatic staging during auto-burns and manual flight, with configurable ignition delays ([forum thread](https://forums.ahwoo.com/threads/autostage.891/))
- [DeltaVMap](https://github.com/Maximilian-Nesslauer/KSA-DeltaVMap) - interactive delta-v subway map and transfer-window planner, auto-generated from the loaded system ([forum thread](https://forums.ahwoo.com/threads/deltavmap.978/))
- [MeasureTools](https://github.com/Maximilian-Nesslauer/KSA-MeasureTools) - click-to-measure ruler, protractor, and surface measuring in the map view ([forum thread](https://forums.ahwoo.com/threads/measuretools.992/))
