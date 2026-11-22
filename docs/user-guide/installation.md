# Installation

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

The native guidance solvers are built from the vendored sources with Rust and Zig, see [native-build.md](../guidance/native-build.md).
