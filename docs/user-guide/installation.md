# Installation

1. Install [StarMap](https://github.com/StarMapLoader/StarMap) (`0.5.4` or later).
2. Download the latest release from the [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases) tab.
3. Extract the zip into `Documents\My Games\Kitten Space Agency\mods\`. The zip already contains the `AdvancedFlightComputer` folder.
4. Start the game and enable `AdvancedFlightComputer` when prompted. Alternatively, add it to `Documents\My Games\Kitten Space Agency\manifest.toml`:

```toml
[[mods]]
id = "AdvancedFlightComputer"
enabled = true
```

AFC reads and writes its files (settings, staging delays, multi-pass plans, RCS options and logs) in the active game profile folder. If a launcher runs the game with its own profile folder (an instance), the mod and its files live in that instance folder instead of `Documents\My Games\Kitten Space Agency\`.

!!! warning "Upgrading from standalone mods"
    AutoStage, AutoRemoveFinishedBurns and PoweredGuidance are built into AFC. Delete their standalone folders under `mods\` before starting the game; disabling them in `manifest.toml` is not enough. See [Mod compatibility](about.md#mod-compatibility).

## Runtime dependencies

| Package | Purpose | Tested version |
| --- | --- | --- |
| [StarMap](https://github.com/StarMapLoader/StarMap) | Mod loader (`0.5.4` or later required at runtime) | `0.5.4` |

Validated against KSA build `2026.11.25.5949`.

## Build dependencies

Required only to build AFC from source. The mod targets **.NET 10**.

| Package | Source | Tested version |
| --- | --- | --- |
| [StarMap.API](https://github.com/StarMapLoader/StarMap) | NuGet | `0.5.4` |
| [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) | NuGet | `2.4.3` |

The native guidance solvers (`clarabel_c` and `scs`) are built from vendored sources with Rust and Zig. See [Native solvers](../guidance/native-build.md).
