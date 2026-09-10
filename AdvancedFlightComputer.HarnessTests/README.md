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

The flight tests fly whichever of `RcsTestVehicles.Candidates` the machine has. Set `KSA_HEADLESS_VEHICLES` to override the candidates. Without one of these saves, the flight tests skip.

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
- `afc-guidance-handback` drives the guidance release paths, which no vehicle driver reaches while guidance is disabled. A craft guidance never engaged has to come out unchanged, an acquired one gets back only the fields guidance wrote and keeps its plan and throttle, a failed cleanup keeps its ownership and error until a later step, and a terminal path reports once and forgets the craft.
- `afc-guidance-thrust` checks what guidance may command from the engines a craft is running. It inverts the game's own nozzle performance at three pressures, and builds an active, an inactive, a dry, a solid and a partly supplied engine, a combination no shipped vehicle offers, to check which of them count as authority. A solid motor that was lit before its controller was switched off keeps burning, so it refuses throttle control while an unlit one does not.
- `afc-guidance-landing-staging` covers the wait that keeps a landing alive across a staging gap, for both landing modes. Harmony supplies the clock and the sequence activation, so a decouple and a later ignition sit at exact times, and the craft is built rather than flown, because a save cannot be asked to lose its engine at a chosen moment. It checks the cooldown, the settling window after the last sequence, the no-progress and total-wait limits with no further staging on the expiry step, refusal of an unsupported core and of an atmospheric body before any staging request, a restart after an abort, and that every ownership method clears the wait. One case uses a spawned craft and the real ApplyAutopilot to check that a waiting mode keeps the craft and the step that ends the mode gives it back. These are waiting decisions with controlled inputs, not real separation, ignition buffering, or supply propagation.
- `afc-guidance-booster-handover` separates a real vehicle and checks adoption, record consumption, guidance state and expiry. Arming and adoption are called directly, so this does not test autonomous flight. The available saves separate sibling boosters, leaving nested separations untested.

### Core

- `afc-save-scoped-reset` asserts that the save-scoped reset list runs cold, populated, and twice in a row without throwing, and clears the plan-window inputs that it covers.
- `afc-stock-pin-guard` asserts the guard that decides whether stock's selected-transfer block can index the porkchop array, against fresh, in-flight, populated, zero-sized, and out-of-range `TransferInfo` states.
- `afc-reflection-targets` asserts that every reflection key, transpiler anchor, and typed plan-window accessor resolves against the running game build, so a game-side rename fails in the harness instead of silently disabling a feature.
- `afc-feature-patch-rollback` checks that a failed feature block removes only its partial patches, keeps other owners intact, and does not prevent a later block or unload.
- `afc-shared-vehicle-hooks` checks the shared MultiPass and RCS tick order, feature gates, patch bindings, and unconditional registry cleanup when a vehicle is disposed.
- `afc-command-sink` checks the one owner of the `FlightComputer.ComputeControl` postfix. It asserts the receipt semantics, that stock values survive a run, that a writer fault neither escapes nor discards what the writer already reported, and that an executing RCS burn holds its own vehicle in full physics with the flight fixture's off-rails override switched off.

### Fixtures

- `afc-vehicle-fixtures` checks rollback after partial construction and failed restore, occupied vehicle ids, full audio and vehicle cleanup, and later reuse of the same id.
