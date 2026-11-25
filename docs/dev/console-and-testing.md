# AFC console and testing

This page covers the single-window console architecture (`AdvancedFlightComputer/Features/Ui/`), the game-free pure UI logic layer (`Features/Ui/Logic/`), and the three automated test suites under `tests/`.

## Console shell (`Features/Ui/Console/`)

`AfcConsole` docks at the stock Transfer Planner slot (`Dock.cs`), sizes height to content (`ShrinkFrames = 3`), supports normal and `Wide` two-pane layouts, and collapses to a live status strip (`afc-strip`) when closed while AFC flies or after a failure/hand-back.
A fault circuit-breaker returns the planner key to stock after `MaxDrawFaults = 30` consecutive draw exceptions.

- **Key routing (`PlannerRoute`):** patches `TransferPlanner.set_ShowPlanWindow` (plus a per-frame fallback check against JIT inlining) to route `F4` and **View > Transfer Planner** through `RouteDecision.Toggle`. `StockLinkActive` yields the dock when the user explicitly opens the stock planner.
- **Per-frame snapshot (`ConsoleFrame`):** constructed once per draw frame before rendering head, strip, page, footer, or drawer. Holds `Subject`, `Mode` (`ModeReadout` + `CraftActivity`), `Stop` (`StopSpec`), `OtherStop`, `Situation`, `Lines`, and failure-band state.
- **World overlays (`Patch_CelestialSystem_OnPreRender`):** renders orbit and transfer preview lines from `CelestialSystem.OnPreRender(IViewport)`.

| Enum (`ConsolePage`) | Tab label | Saved key | Implementation | Editor / EVA |
| --- | --- | --- | --- | --- |
| `Launch` | `LAUNCH` | `LAUNCH` | `GuidancePages.cs` | No |
| `Orbit` | `MANEUVER` | `ORBIT` | `OrbitPage.cs` | No |
| `Transfer` | `TRANSFER` | `TRANSFER` | `TransferPage.cs` | Yes (`PLANNER` link) |
| `Rcs` | `BURNS` | `RCS` | `RcsPage.cs` | No |
| `Land` | `LAND` | `LAND` | `GuidancePages.cs` | No |
| `Staging` | `STAGING` | `STAGING` | `StagingPage.cs` | Yes (defaults) |
| `Tune` | `TUNE` | `TUNE` | `TunePage.cs` | Yes (`PREFERENCES`) |

`FlightContext.FirstPage` picks the initial tab when a flight starts (Editor -> `Staging`, else running mode's page, else Ground+Engines -> `Launch`, Falling -> `Land`, Orbit+CelestialTarget -> `Transfer`, Orbit/CoastingUp/Escaping -> `Orbit`, else `Launch`).
`ConsoleFooter` guards commit clicks with `VerbGuard` so a verb changing under the cursor cannot trigger an unintended commit.

## Pure UI logic layer (`Features/Ui/Logic/`)

Every file under `Features/Ui/Logic/` has **zero dependency on `KSA.dll`**, allowing the entire rule layer to compile directly into `AdvancedFlightComputer.UiLogic.Tests` and run headless in milliseconds:

- **`GuidanceModes` and `ModeRegistry` (`Features/Guidance/Ui/ModeRegistry.cs`):** pairs a pure `GuidanceModeDescriptor` (id, page, stop priority, phases, verb/stop text) with game-side `GuidanceModeHooks` (`IsLive`, `PhaseLabel`, `Commit`, `Stop`, `Diagnostics`, `Readout`, `StopText`). Lower `StopOrder` wins (closest to ground first): `SixDofHere (10)` -> `EntryHere (15)` -> `Hover (20)` -> `GfoldHere (30)` -> `DeorbitLand (40)` -> `Boostback (50)` -> `AscentUpfg (60)` -> `AscentGravityTurn (70)`.
- **`StopResolver`:** resolves the single console stop button from `CraftActivity` (guidance modes by `StopOrder`, then `Rcs`, `MultiPass`, `StockAuto`) along with short/full engine outcomes and `RELEASE ALL GUIDANCE` visibility.
- **`CommitResolver`:** maps `(GuidanceModeId, CraftActivity, CommitGates)` to `CommitView` (verb, enabled state, refusal reason, takeover confirmation message, fit indicator).
- **`StatusSelector` and `LineAges`:** prioritizes `Severity` (`Diag`, `Info`, `Cue`, `Warning`, `Error`), suppresses `Info` rows while flying, and enforces `NoPlanWaitSeconds = 10 s`, `EndedNoteSeconds = 15 s`, and `EndedWarningSeconds = 30 s`.
- **`GuidanceSettingSpecs` and `SettingSpec`:** single source of truth for every guidance numeric and boolean setting (key, label, unit, `UiTier` = `Player` / `Expert` / `Tuning` / `Diagnostics`, default, range, printf format, tooltip). `SettingSpecDefaultsTest` verifies every spec default matches `VehicleAutopilotState`.
- **`FoldRules`:** builds width-fitted fold headers (`FitSummary`) while preserving `(N CHANGED)` suffixes.
- **`LandSolvers` and `EntryGuidance`:** pure selection rules for `Entry` (`hasAir && !thrust && fallS >= 60 s`), `Touchdown` (primary), and `Gfold` (airless gimballed fallback, never player-selectable).

## Test suites (`tests/`)

AFC validates code at three levels: pure UI rules, standalone numerical solvers (no game DLLs), and full in-game headless flight tests.

| Suite | Project(s) | Game DLLs | How to run | Coverage |
| --- | --- | --- | --- | --- |
| **UI logic tests** | `tests/AdvancedFlightComputer.UiLogic.Tests/UiLogic.Tests.csproj` | None | `dotnet run --project tests/AdvancedFlightComputer.UiLogic.Tests/UiLogic.Tests.csproj` | ~57 check groups: stop/commit/status/fold/route/dock/porkchop logic, `GuidanceSettingSpecs` defaults, `FairingReleaseGate`, `BurnTurnRules`, `LandSolvers`, `ModeRegistry`, `IdleEngineParking`, `LandingSiteFile`, and static `PrintfTextCalls` audit |
| **Guidance solver tests** | `tests/AdvancedFlightComputer.Guidance.Tests/{Gfold,Scvx,Worker.Tests}/` | None | `dotnet run --project tests/AdvancedFlightComputer.Guidance.Tests/Scvx -- [flags]` (or `./build/ci/test-guidance.ps1`) | G-FOLD P3/P4 vs Python (`gfold_ref.py`), terminal braking, accel limits; SCvx forward-mode AD vs JAX (`dyn_ref.py`, `sub_ref.py`, `loop_ref.py`, `threedof_ref.py`), 3-DOF/6-DOF/Ascent/aero/fin/RCS/coast/slope/hand-over checks (`--backend clarabel\|scs`); 7 `Ksa6DofSolveWorker` concurrency/lifecycle checks |
| **Headless harness tests** | `tests/AdvancedFlightComputer.HarnessTests/AdvancedFlightComputer.HarnessTests.csproj` | `KSA.dll` + `HeadlessHarness.dll` (`Private=false`) | `scripts/run-headless.ps1` (filtered by `KSA_HEADLESS_TESTS`) | 116 `AfcTest` (`IHarnessTest`) files across 13 areas against real KSA physics and vehicles: Guidance (52), RcsTranslation (14), Core (13), ManeuverTools (10), AutoStage (9), Flyby (5), AutoRemove (3), Ui (3), MultiPass (2), Transfer (2), Burns (1), HyperbolicTargets (1), LandingSites (1) |

In `HarnessTests`, `AfcTest` pins `GuidanceClock.Source` to simulation-time milliseconds so time-based gates run deterministically regardless of host CPU speed, and intentionally lets `MissingMemberException` / `TypeLoadException` propagate so `HarnessRunner` immediately flags KSA API breaks.

## Code map

| File | Main type or method | Role |
| --- | --- | --- |
| [AfcConsole.Window.cs](../../AdvancedFlightComputer/Features/Ui/Console/AfcConsole.Window.cs) | `AfcConsole.Draw`, `DrawStrip` | Console window, wide mode, strip, fault circuit-breaker |
| [PlannerRoute.cs](../../AdvancedFlightComputer/Features/Ui/Console/PlannerRoute.cs) | `PlannerRoute` | `F4` and Transfer Planner key routing |
| [ConsoleHead.cs](../../AdvancedFlightComputer/Features/Ui/Console/ConsoleHead.cs) | `ConsoleHead`, `ConsoleFooter`, `HeadDrawer` | Owner chips, phase strip, stop button, guarded commits |
| [StopResolver.cs](../../AdvancedFlightComputer/Features/Ui/Logic/StopResolver.cs) | `StopResolver`, `CommitResolver`, `StatusSelector` | Pure-logic stop priority, commit gates, status ageing |
| [GuidanceSettingSpecs.cs](../../AdvancedFlightComputer/Features/Ui/Logic/GuidanceSettingSpecs.cs) | `GuidanceSettingSpecs`, `SettingMath`, `FoldRules` | Single source of truth for setting tiers, defaults and fold headers |
| [ModeRegistry.cs](../../AdvancedFlightComputer/Features/Guidance/Ui/ModeRegistry.cs) | `GuidanceWindow.RegisterMode`, `GuidanceModeHooks` | Mode descriptor + hook registration bridge |
| [Program.cs](../../tests/AdvancedFlightComputer.UiLogic.Tests/Program.cs) | `UiLogic.Tests` | Game-free console test runner for UI logic and printf safety |
| [AfcTest.cs](../../tests/AdvancedFlightComputer.HarnessTests/AfcTest.cs) | `AfcTest` | Base class for deterministic headless KSA flight tests |
