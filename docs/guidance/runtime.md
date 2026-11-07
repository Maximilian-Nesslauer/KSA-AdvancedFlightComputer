# Guidance runtime

How the guidance feature hangs on AFC at run time. The algorithms are described in the other documents in this folder, and the fields guidance writes on a craft are listed in `../control-ownership.md`.

## Entry

`AdvancedFlightComputer/Mod.cs` is the only StarMap entry. Guidance is one feature of it and registers two patch blocks through `FeaturePatchSet`, each on its own Harmony id, so a failed block rolls back on its own and leaves the rest of AFC running.

- The diagnostics block, `GuidanceFeature.ApplyDiagnosticPatches`, installs the managed assembly resolver for the three numerical assemblies and one postfix on `Program.DrawProgramMenusHook` that draws the AFC menu (`AfcMenu.Draw`). The menu holds the stop of what runs on the controlled craft as its first item, the GUIDANCE ENABLED switch, and the items that open the AFC console and its pages, which the console registers. In the editor only the pages that work there can be opened. A no-op transpiler on `Program.DrawMenuBar` keeps the hook call when another mod patched the menu bar first. With ModMenu, `Core/ModMenuBridge` draws the menu under Mods instead.
- The driver block, `GuidanceFeature.ApplyDriverPatches`, installs the prefix on `Vehicle.PrepareWorker` that runs `GuidanceWindow.ApplyAutopilot` for every vehicle on the main thread before stock snapshots the flight computer for its worker, and the postfix on the private `FlightComputer.UpdateAttitudeTarget` that adds the rate feedforward on the vehicle worker. It also subscribes the release of every guided craft to `SaveLoadObserver.SaveLoaded`, because a load replaces every vehicle.
The gimbal writer, `KsaGimbalControl.OnComputeControl`, is not a patch of its own. `VehicleCommandSink` owns the single `FlightComputer.ComputeControl` postfix and dispatches the guidance writer after the RCS writer, with the wake and off-rails bookkeeping applied from the receipt.

The guidance pages are drawn by the AFC console from the loader's after-GUI hook (`AfcConsole.Draw`), which logs a draw fault once per kind instead of letting it end the frame. In the editor the console shows only TRANSFER, STAGING and TUNE, and the guidance pages say "In flight only". The console reaches guidance through one bridge, `Ui/Console/GuidanceConsole.cs`.

## Ownership

Guidance claims a craft when it starts control and releases it after cleanup succeeds.
An armed launch and a coast to direct braking wait without a new claim; handovers between guidance modes keep the claim.
The braking burn keeps the engine command through a queued 6-DOF engage.
While AFC controls the engines, a new stock Auto selection is a player takeover that stops guidance without cutting the engine.
During the stock deorbit node, guidance holds a claim while stock Auto controls the burn.

A player change to the attitude target that guidance wrote is a takeover as well, detected by comparing the values guidance last wrote against what the flight computer holds. The release restores only the fields guidance replaced.

## Landing approach

The planner first checks for direct braking.
If the direct approach is refused, it searches a Lambert transfer to a point uprange of the rotating site, 11 km above the site by default.
The search spans the next orbit, checks at most eight candidates, and limits cumulative planner work to 500 ms.
It checks braking, timing, atmosphere and terrain clearance with a 1 km terrain margin.
Automatic transfer arrivals are between 10 degrees down and 0.5 degrees up from the local horizon.
The optional Set deorbit arrival angle requests 0 to 30 degrees down with a one-degree tolerance; it sets the path angle, not the craft attitude.

Planning waits for 1x time warp, an idle engine and a settled source orbit.
For a transfer, AFC claims the craft, creates a stock maneuver node and checks stock's minimum-throttle timing preview before it arms Auto.
Stock flies the node; the optional Auto burn and warp stops before stock alignment starts.
If the player changes the node or takes control, AFC stops the landing and preserves the player's target.
After stock completes the burn, AFC plans braking again from the measured orbit and mass, then flies the braking burn.
The site, approach settings and solver choice stay fixed after the node is committed.

For a G-FOLD landing, VERTICAL GATE sets the elevated braking gate, 500 m by default. DEORBIT + LAND needs at least 100 m; LAND NOW flies any value from 0 m.
G-FOLD removes horizontal speed before the gate and descends through it.
If minimum thrust exceeds weight, the engine cuts for an upright coast and a timed final burn.
The terminal burn uses current speed, height, gravity, thrust and minimum pulse, with at most 15 degrees of commanded tilt.
The descent modes keep the guidance claim until contact, abort or takeover.

## Staging

Guidance has no stager of its own. `AutoSequence` arms AFC's AutoStage feature for the craft through `StagingDetector.Arm` and requests rows through `RequestStaging` for the cues only a plan knows, the cold ignition and the reserve boundary. Burnout and spent-booster detection, delays, crossfeed checks and the cascade are AutoStage's. Guidance arms the switch once per flight and then leaves it to the player, so a disarm holds until the flight ends.

## Layout

| Folder under `AdvancedFlightComputer/Features/Guidance` | Contents |
| --- | --- |
| `Modes` | One file per mode, `Ascent`, `Boostback`, `Landing`, `GfoldDescent`, `SixDof`, `TerminalHover`, plus `Autopilot.cs`, the per-vehicle step every mode is driven from, `Lifecycle.cs`, the release paths, and `VehicleAutopilotState.cs`, the per-craft state. |
| `Control` | The actuator boundary: `GimbalControl` and `TvcAllocator` for thrust vectoring, `AttitudeRate` for the rate feedforward, `EnginePerf` for engine capability and thrust inversion, `AttitudeOwnership` for the fields guidance writes. |
| `Adapters` | Game data conversion for the solvers: `KsaGfold` for G-FOLD; `Scvx/SixDof` for the 6-DOF path, `Ksa6DofSetup` and `Ksa6DofGuidance` with the solve worker and its telemetry log; `Scvx/Ascent` for the convex ascent's `KsaAscentPlan`; and, shared between them, `KsaFrameBridge` and the `KsaAeroSweep` drag table. |
| `Upfg` | The double-precision UPFG port and `KsaVehicleAdapter`, which builds the stage model from the stock sequence performance and scales the burning stage to the craft's ambient pressure. |
| `Ui` | The guidance pages of the AFC console in `Console` (LAUNCH, LAND, TUNE and the bridge the console shell calls), the section content in `Gauges`, the tier kit that splits each section into player, expert, tuning and diagnostics rows in `Tiers`, the world-space overlays in `Overlays`, and the windows drawn after the console in `Window.cs`. |
| `Numerics`, `Conic`, `Gfold`, `Scvx` | The four game-independent projects, built as their own assemblies so their checks run with no game. `Conic` holds the Clarabel and SCS bindings both planners solve with. |

The whole game-facing part is one `partial class GuidanceWindow` in the `AdvancedFlightComputer.Features.Guidance` namespace, so the folders organise the files without changing their scope.

## Console pages

Guidance is on two pages of the AFC console, a stock console window that F4 opens where the transfer planner opened, and that moves, resizes and closes like it. LAUNCH holds the ascent and, in its BOOSTERS region, the returnable stages with their sites and the booster reserve that the solved climb sizes for the stage with a site. LAND holds the landing site and four ways down: DEORBIT flies the whole chain from orbit to touchdown, BOOSTBACK flies a spent booster back to its site after separation, LAND NOW starts only the powered descent from the current state, and HOVER starts the terminal hover. The MODE column of the solver table on LAND NOW offers ENTRY (entry guidance, the 3-DOF glide and landing burn), TOUCHDOWN (touchdown guidance, the 6-DOF solver) or G-FOLD; DEORBIT holds ENTRY, because the braking burn hands over to TOUCHDOWN or G-FOLD. When the mode that flies the landing changes, for example when a boostback glide hands the craft to entry guidance or entry guidance hands its burn to touchdown guidance, the console sets the solver to that mode, moves the LAND way to BOOSTBACK or LAND NOW where the hand-over asks for it (`LandWays.Follow`), and raises a cue with a link to LAND > LAND NOW (`GuidanceWindow.FollowLiveLanding`, `LandSolvers`), so the page, its commit and its stop act on the mode that flies. A plan page draws its inputs first, then its picture, then the folds. The footer of a page commits its mode. The head of the console holds the one stop, which acts on the mode that runs whatever page is on screen, and its drawer holds RELEASE ALL; PICK SITE on the SITE row of LAND, and PICK SITE in the BOOSTERS region of LAUNCH, arm the retarget click. See `../control-ownership.md`.

Each section is drawn once per tier: the player rows on the page or in a fold with its values in the label, the expert rows in its GNC fold, and the tuning and diagnostics rows on the TUNE page. A row draws only in the scope of its tier, so a value that changes tier also moves to a place that draws that scope. The label of a fold comes from the same list of parts that draws its rows (`FoldRules`), and a fold with no part to show does not show. The tier of every input is in `Features/Ui/Logic/GuidanceSettingSpecs.cs`, with its unit, range and default; `afc-ui-setting-defaults` checks the defaults against `VehicleAutopilotState`. A value that differs from its default has a small dot at the end of its label and says its default in the tooltip. Each GNC fold and tuning set counts and resets its changed values; a Player value on the page is in no such count and has no reset. New solver values belong in the Tuning tier unless a player sets them per flight.

A new guidance mode adds a `GuidanceModeId` value and calls `GuidanceWindow.RegisterMode` once (`Features/Guidance/Ui/ModeRegistry.cs`). The call takes a descriptor from `GuidanceModes.Registered` (page, method label, stop order, phases, commit verb, stop label and outcome) and the game-side `GuidanceModeHooks`: the live check and the phase label, which read the craft's `VehicleAutopilotState`, the commit, which runs from the footer verb, and the stop, which runs from the head stop. Both run on the main thread with the craft's state bound. The console then finds the mode with no other change: `CraftActivity.RegisteredModeLive` names it, `StopResolver` returns `StopTarget.Registered` with the descriptor's label, `CommitResolver` gives its verb, refuses it while it flies and asks before it takes the craft from another mode, `ConsolePages.PageOf` opens its page, and the head shows its phase strip. The hooks can also replace the LAUNCH method entry of the same id (`AscentSetup`, an `AscentMethodUi` with its player, expert and tuning content) and add a TUNE > DIAG view (`Diagnostics`). A mode on LAND also adds its case to `ConsolePages.ModeIdFor`, which picks the mode from the WAY row. The mode's own step in the guidance driver is not part of the registry. The gravity turn slot is the built-in unavailable descriptor of `GuidanceModeId.AscentGravityTurn`: its ASCENT segment shows but does not select until the mode registers with `Available` true. The unload sets the registry back to the built-in modes.

Entry guidance is the first of AFC's own modes on the registry (`Ui/Console/EntryMode.cs`, descriptor `EntryGuidance.Descriptor`, id `GuidanceModeId.EntryHere`). `GuidanceWindow.EnsureAfcModes` registers it on the first read of the registry after a load or a reset, so a reset never leaves a flying entry guidance without its stop. Its live check reads `ThreeDofPhase` and `ThreeDofEngagePending` of the craft, because a boostback glide engages it with no commit (`StepBoostback`). Its stop order, 15, puts it ahead of the boostback whose glide it takes over and behind touchdown guidance, which takes its burn. Two optional hooks serve it: `Readout` gives the head the time to ignition or the burn left and the landing numbers, and `StopText` gives the stop outcome of the stage it is in, because `Disengage3Dof` leaves a lit landing burn running and cuts the engine in the glide (`EntryGuidance.StopText`). Its commit sets `AutoStage` and `ThreeDofEngagePending`, and the next step engages it ahead of the control claim. During a deorbit that stock AUTO flies, the commit first ends that landing with `AbortLanding`, which cuts the engine, because `StepStockDeorbit` returns before the request is read and the request would otherwise wait for the end of the deorbit. Its DIAG view is the `Diagnostics` hook.

## Dependencies

At run time the mod needs the StarMap loader, which also supplies Harmony. The guidance feature ships the three numerical assemblies next to the mod DLL together with the native solvers `clarabel_c.dll` and `scs.dll`. The build refuses a deploy or a Release package when a native is missing, see [README.md](README.md) and [native-build.md](native-build.md).
