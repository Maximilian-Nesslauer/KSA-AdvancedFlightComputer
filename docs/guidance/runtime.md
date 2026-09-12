# Guidance runtime

How the guidance feature hangs on AFC at run time. The algorithms are described in the other documents in this folder, and the fields guidance writes on a craft are listed in `../control-ownership.md`.

## Entry

`AdvancedFlightComputer/Mod.cs` is the only StarMap entry. Guidance is one feature of it and registers two patch blocks through `FeaturePatchSet`, each on its own Harmony id, so a failed block rolls back on its own and leaves the rest of AFC running.

- The diagnostics block, `GuidanceFeature.ApplyDiagnosticPatches`, installs the managed assembly resolver for the three numerical assemblies and one postfix on `Program.DrawProgramMenusHook` that draws the AFC Guidance menu. The menu holds the Enabled switch and the Show panel switch. It is not drawn while the editor is open.
- The driver block, `GuidanceFeature.ApplyDriverPatches`, installs the prefix on `Vehicle.PrepareWorker` that runs `GuidanceWindow.ApplyAutopilot` for every vehicle on the main thread before stock snapshots the flight computer for its worker, and the postfix on the private `FlightComputer.UpdateAttitudeTarget` that adds the rate feedforward on the vehicle worker. It also subscribes the release of every guided craft to `SaveLoadObserver.SaveLoaded`, because a load replaces every vehicle.

The gimbal writer, `KsaGimbalControl.OnComputeControl`, is not a patch of its own. `VehicleCommandSink` owns the single `FlightComputer.ComputeControl` postfix and dispatches the guidance writer after the RCS writer, with the wake and off-rails bookkeeping applied from the receipt.

The panel is drawn from the loader's after-GUI hook through `GuidanceFeature.DrawGui`, which skips the editor and logs a draw fault once per kind instead of letting it end the frame.

## Ownership

Every guidance mode takes the per-vehicle claim in `Core/VehicleControlOwnership.cs` where it commits to flying a craft, and gives it back only after its cleanup succeeded. A second claimant is refused by name, never overwritten. While guidance holds a craft, stock's burn mode is held in Manual, and an Auto that appears is an engine takeover by the player that stops guidance without an engine cut.

A player change to the attitude target that guidance wrote is a takeover as well, detected by comparing the values guidance last wrote against what the flight computer holds. The release restores only the fields guidance replaced.

## Staging

Guidance has no stager of its own. `AutoSequence` arms AFC's AutoStage feature for the craft through `StagingDetector.Arm` and requests rows through `RequestStaging` for the cues only a plan knows, the cold ignition and the reserve boundary. Burnout and spent-booster detection, delays, crossfeed checks and the cascade are AutoStage's.

## Layout

| Folder under `AdvancedFlightComputer/Features/Guidance` | Contents |
| --- | --- |
| `Modes` | One file per mode, `Ascent`, `Boostback`, `Landing`, `GfoldDescent`, `SixDof`, `TerminalHover`, plus `Autopilot.cs`, the per-vehicle step every mode is driven from, `Lifecycle.cs`, the release paths, and `VehicleAutopilotState.cs`, the per-craft state. |
| `Control` | The actuator boundary: `GimbalControl` and `TvcAllocator` for thrust vectoring, `AttitudeRate` for the rate feedforward, `EnginePerf` for engine capability and thrust inversion, `AttitudeOwnership` for the fields guidance writes. |
| `Adapters` | Game data conversion for the solvers: `KsaGfold` for G-FOLD, `Ksa6DofSetup`, `KsaFrameBridge` and `Ksa6DofGuidance` for the 6-DOF path, the solve worker and its telemetry log. |
| `Upfg` | The double-precision UPFG port and `KsaVehicleAdapter`, which builds the stage model from the stock sequence performance. |
| `Ui` | The panel shell in `Panel.cs`, the per-tab content in `Gauges`, the world-space overlays in `Overlays`, and the legacy diagnostic window in `Window.cs`, off by default. |
| `Numerics`, `Gfold`, `Scvx` | The three game-independent projects, built as their own assemblies so their checks run with no game. |

The whole game-facing part is one `partial class GuidanceWindow` in the `AdvancedFlightComputer.Features.Guidance` namespace, so the folders organise the files without changing their scope.

## Panel

The AFC Guidance panel is a stock console window, so it moves, resizes and closes like the transfer planner, and it can leave the main game window. It starts hidden and is opened from the AFC Guidance menu. One row of EXECUTE, ABORT and RETARGET acts on the selected tab. The tabs are Ascent, Boostback, Deorbit and land, which flies the whole chain from orbit to touchdown, and Land from here, which starts only the terminal powered descent from the current state. RELEASE GUIDANCE sits in the footer and hands the craft back whatever the tab.

## Dependencies

At run time the mod needs the StarMap loader, which also supplies Harmony. The guidance feature ships the three numerical assemblies next to the mod DLL together with the native solvers `clarabel_c.dll` and `scs.dll`. The build refuses a deploy or a Release package when a native is missing, see [README.md](README.md) and [native-build.md](native-build.md).
