# PoweredGuidance KSA mod

A Kitten Space Agency (KSA) mod that draws an ImGui window in-game and drives the
flight computer. Includes a standalone port of UPFG (Unified Powered Flight Guidance)
for ascent and a convex G-FOLD powered-descent solver for landing.

Requires the [StarMap](https://github.com/StarMapLoader/StarMap) mod loader.

## How it works

1. **Entry point**: `Mod` is a `[StarMapMod]` class. StarMap discovers this assembly
   (declared as `EntryAssembly` in `mod.toml`), constructs `Mod`, and calls its
   lifecycle methods.
   - `[StarMapAllModsLoaded]` applies the Harmony prefix on `Vehicle.PrepareWorker`
     (autopilot writes that must land just before the sim snapshots the FC).
   - `[StarMapAfterGui]` draws the window every frame inside the active ImGui frame
     (`PoweredGuidanceWindow.Draw(Program.MainViewport)`, in `Ui/Window.cs`).
   - `[StarMapUnload]` removes the patches.
2. **Layout.** Everything but `Mod.cs` sits in a folder by role. The whole mod is one
   `partial class PoweredGuidanceWindow` in the global namespace, so the folders are
   organisation rather than namespacing — nothing in the csproj lists files, since
   SDK-style projects glob `**/*.cs`.

   - `Guidance/` — the flight logic. One file per mode (`Ascent`, `Landing`,
     `GfoldDescent`, `SixDof`, `TerminalHover`), plus `Autopilot.cs`, the per-vehicle
     step-and-apply entry point every mode is driven from, and
     `VehicleAutopilotState.cs`, the per-craft state they all act on.
   - `Control/` — the actuator and vehicle boundary: `GimbalControl` and
     `TvcAllocator` (thrust vectoring), `AttitudeRate` (the flight computer's
     feedforward), `EnginePerf` (engine capability and thrust-curve inversion).
   - `Ui/` — the panel shell (`Window`, `Panel`, `GaugeKit`, `StatusBlock`), with
     `Ui/Gauges/` for the in-window readouts and `Ui/Overlays/` for world-space drawing.
   - `Upfg/` — the double-precision UPFG port (`UpfgGuidance`, `CseRoutine`,
     `UpfgTarget`, `UpfgVehicle`, `KsaVehicleAdapter`).
   - `Gfold/` — the bridge (`KsaGfold`) to the standalone G-FOLD solver in `../gfold`.
   - `Scvx/` — the bridge to the 6-DOF successive-convexification solver in `../scvx`,
     plus its flight telemetry logger.

## Dependencies

- **Runtime**: the [StarMap](https://github.com/StarMapLoader/StarMap) loader must be
  installed in the game. StarMap provides Harmony at runtime, so the mod ships only
  itself plus its private deps (`Gfold.Core.dll` + native `clarabel_c.dll` and `scs.dll`).
- **Build**: .NET 10, plus the `StarMap.API` and `Lib.Harmony` NuGet packages
  (restored automatically). Game assemblies are referenced from
  `C:\Program Files\Kitten Space Agency` (override with `-p:KsaDir=...`).

## Build & install

```
dotnet build PoweredGuidance.csproj -c Release
```

The `CopyToMods` build target installs the mod into
`Documents\My Games\Kitten Space Agency\mods\PoweredGuidance\` (the mod DLL, `mod.toml`,
`Gfold.Core.dll`, `clarabel_c.dll` and `scs.dll`). KSA auto-discovers the folder via `mod.toml` and
prompts to enable it (or add it to `manifest.toml`).

The solution also contains `tools/convtest` — a console app that verifies the
steering→Euler attitude conversion round-trips through KSA's own quaternion functions.
