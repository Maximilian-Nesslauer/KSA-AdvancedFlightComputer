# AdvancedFlightComputer.HarnessTests

Tests for this mod, run by [HeadlessHarness](https://github.com/Maximilian-Nesslauer/KSA-HeadlessHarness): it brings the real game up GPU-free and runs plug-in tests against the live simulation. Nothing here ships in a release, and on a normal launch this mod does nothing.

## Layout

```
Framework/           how a test reports
  AfcTest.cs         base class: context in, exit code out
  TestContext.cs     checks, skips, verdict
  Approx.cs          tolerance bounds
Fixtures/            what a test asserts against
  TestWorld.cs       home body, moon lookup
  OrbitFixtures.cs   test orbits, impulsive apply
  VehicleFixtures.cs throwaway vehicle spawning
Features/<Feature>/  one folder per mod folder under test, plus that feature's own helpers
```

`Features/` follows where the code under test lives, not the feature that motivated the test: `StockPinGuardTest` covers `Core/StockPlanner`, so it sits in `Features/Core/`.

## Adding a test

Drop the file in the matching `Features/` folder and derive from `AfcTest`. Discovery is by interface, so there is nothing to register.

```csharp
public sealed class ThingTest : AfcTest
{
    public override string Name => "afc-thing";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        Orbit orbit = OrbitFixtures.EllipticalAt(home, 300_000.0, 2_000_000.0, Universe.GetElapsedTime());
        t.Check("does the thing", Thing.Compute(orbit) > 0.0, "detail the reader needs");
    }
}
```

Report through the context, not through `HarnessLog`: `Check` (plus `CheckRel` / `CheckAbs` / `CheckMixed` for a numeric bound), `Fail` for a precondition that blocked the assertion, `Skip` for a case this run cannot cover, `Info` for anything that is not an assertion. There is no `ok` to thread and nothing to return.

Two things to get right: do not catch exceptions to turn them into failures, because the harness classifies `MissingMemberException` / `TypeLoadException` as game-API drift and reports that as an infrastructure failure; and despawn every spawned vehicle from a `finally`, or it keeps ticking into later tests.

## Running

Build this solution and HeadlessHarness, checked out as a sibling, in the same configuration; the `CopyToMods` targets deploy both. Then run the harness's `scripts/run-headless.ps1`.

`-Tests` filters on exact `Name` values, so renaming a test breaks the invocations that use it. Use one or more of the names below, separated by commas.

The RCS flight tests fly whichever of `RcsTestVehicles.Candidates` the machine has. Set `KSA_HEADLESS_VEHICLES` to override the candidates. Without one of these saves, the flight tests skip.

The staging flight tests use the save named by `KSA_HEADLESS_VEHICLE`, shared with the harness's own flight test, and skip when it is unset. `afc-autostage-spent-drop` instead takes its save from `KSA_HEADLESS_VEHICLES` and defaults to "Test Vehicle 1". That default is the only end-to-end cover of the jettison analysis, so it fails rather than skips when the save is missing: provide a save whose launch stage mixes boosters with a core under that name, or name a substitute in `KSA_HEADLESS_VEHICLES`.

Leave the deployed test mod disabled for normal play. It only does work inside a harness run and is not part of the released mod.

## Test coverage

The oracle is always the game's own orbit propagation, never a re-derivation of the math under test.

### Maneuver quick-tools

- `afc-set-periapsis` and `afc-set-apoapsis` assert that a computed apse burn reaches the requested altitude and leaves the opposite apse unchanged, and that impossible requests yield no maneuver.
- `afc-circularize` asserts circularization at both apses and the "nothing to do" contract for circular and unbound orbits.
- `afc-set-inclination` and `afc-match-inclination` assert node burns against the ecliptic and equatorial references, partial-fraction burns, and the coplanar and hyperbolic edge cases.
- `afc-burn-menu-launcher` checks that the four orbit-menu shortcuts select the requested tool and source, reset input defaults, and leave the planner unchanged when a shortcut type is removed. It also checks closed-window cleanup before the plan-type gate.
- `afc-maneuver-transpilers` checks the shortcut and create-button injections against the current game IL and verifies exception boundaries with Harmony-patched test methods.
- `afc-target-identity` asserts that a Match Inclination target selection keeps naming the same body while vehicles are spawned and despawned around it, because the game's lookup swap-removes on deregister and a held index would drift.

### Flyby targeting

- `afc-flyby-targeting` asserts the flyby impact-parameter closed forms against the game's own hyperbolic orbit elements, the periapsis reference resolution, and the airless-body case where no atmosphere reference is offered.
- `afc-flyby-departure` builds a departure toward a real moon. The center-aimed baseline must impact, the retargeted one must clear the body at the requested periapsis, and Inner and Outer must land on opposite sides of it.
- `afc-flyby-preview` checks that the readout, multi-pass preview, and commit reuse one departure solve, that user input bypasses the thrust freeze, and that a missing trajectory patch or encounter gives an explicit no-prediction state.

### Hyperbolic targets

- `afc-hyperbolic-targets` drives the stock planner entry points against an unbound celestial. It checks the Hohmann estimate against stock's own formula on a stand-in circular orbit, the alignment time, the target list, the transfer window that stock would otherwise throw on, the encounter search against the comet, and the refine step.

### Multi-pass

- `afc-sequence-burnstate` asserts the per-sequence burn-state adapter against the game's own `SequencePerformanceList` on real vehicles, including a disabled tank and a non-default engine flow rule. It checks burnable fuel, mass flow, exhaust velocity, and start mass.
- `afc-multipass-registry` checks TOML persistence, maneuver intent round-trips, save-scope changes, and unique temporary-file cleanup.
- `afc-multipass-fallback` checks the Hohmann fallback transpiler against the current game IL and verifies branch labels and exception boundaries with synthetic test methods.

### RCS translation

- `afc-rcs-allocator` asserts per-axis pulse shaping, per-thruster group pulses, the Hold-strategy performance model, the burn-duration countdown mirror, and the capability helpers.
- `afc-rcs-estimates` asserts the propellant that each attitude strategy needs and the Hold-versus-Align resolution, including the preference margin that keeps Auto from slewing for a marginal saving.
- `afc-rcs-burn-preview` checks that a later planned burn uses the current vehicle for its estimate, follows edits to its time and delta-V, and does not change the loaded burn target or live executor estimate.
- `afc-rcs-registry` asserts the TOML persistence round-trip, including escaped ids and active-execution fields, and the per-burn options key that follows a burn when its time changes.
- `afc-rcs-lp-solver` asserts the LP allocator's simplex on hand-checkable problems, including cost optimality, zero-torque constraint satisfaction, support selection, and clean infeasibility.
- `afc-rcs-translation` flies a full RCS translation burn on the live simulation. The burn must reach its delta-V target within the minimum-impulse bound, consume thruster propellant, and never command a main engine. It also covers align slew, deferred alignment, the warp margin, save and load during coast, changed ignition time, cancellation, and RCS-toggle scenarios.
- `afc-rcs-lp` flies the same burn with both allocators on one vehicle, asserts that both complete with quiet engines, and logs the propellant comparison.
- `afc-rcs-driver-fault` injects driver, attitude, RCS, and fuel telemetry faults on a live vehicle. A faulted tick has to suppress the published command, hand back only the controls it owns, restore attitude and RCS independently, keep a failed restore visible and retryable within its attempt limit, survive save and load without reattaching the burn, and never raise the completion event.

### Automatic staging

The staging tests apply the feature's patches on a test-scoped Harmony owner through `AutoStageTestPatches`, arm the detector through the gauge toggle path, and remove the patches when they end.

- `afc-autostage-flight` flies a staged save at full manual throttle and asserts that every remaining engine sequence is activated automatically and that each one actually lights. A trailing decoupler-only sequence is left standing on purpose, because staging only runs while an engine is still ahead.
- `afc-autostage-delays` measures that configured decoupler and engine ignition delays fire on time, each in isolation.
- `afc-autostage-per-vehicle` spawns two vehicles and checks that the switch, a staging request and the state belong to the vehicle: a request activates a row on the armed vehicle only, waits on a disarmed one, and a disposed vehicle takes its state with it.
- `afc-autostage-spent-drop` flies a save whose launch stage mixes boosters with a core and asserts the boosters are shed as soon as they burn out, never earlier, with the core still firing afterwards, and that the drop never arms on the frame the launch sequence fires.

### Automatic burn removal

- `afc-autoremove-burns` adds a real burn through the game's input queue and drives the real flight computer through the Auto to Manual transition: a completed auto-burn is removed, while out-of-fuel, switched-off, manual-mode, zero-delta-V-insert and uncontrolled-vehicle cases keep the burn.
- `afc-autoremove-rcs` raises the RCS completion event through the feature's own subscription and checks the removal policy on it: raised, switched off, uncontrolled, and a burn already gone.

### Core

- `afc-save-scoped-reset` asserts that the save-scoped reset list runs cold, populated, and twice in a row without throwing, and clears the plan-window inputs that it covers.
- `afc-stock-pin-guard` asserts the guard that decides whether stock's selected-transfer block can index the porkchop array, against fresh, in-flight, populated, zero-sized, and out-of-range `TransferInfo` states.
- `afc-reflection-targets` asserts that every reflection key, transpiler anchor, and typed plan-window accessor resolves against the running game build, so a game-side rename fails in the harness instead of silently disabling a feature. It also checks the AUTOSTAGE gauge enum injection and that stock still activates a sequence row through `Part.ActivateSubtreeInStage`, which the staging execution clones.
- `afc-feature-patch-rollback` checks that a failed feature block removes only its partial patches, keeps other owners intact, and does not prevent a later block or unload.
- `afc-shared-vehicle-hooks` checks the shared AutoStage, MultiPass, RCS and AutoRemove tick order, feature gates, patch bindings, and unconditional registry and cache cleanup when a vehicle is disposed.

### Fixtures

- `afc-vehicle-fixtures` checks rollback after partial construction and failed restore, occupied vehicle ids, full audio and vehicle cleanup, and later reuse of the same id.
