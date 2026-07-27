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

        Orbit orbit = OrbitFixtures.EllipticalAt(home, 300_000.0, 2_000_000.0, Universe.GetElapsedSimTime());
        t.Check("does the thing", Thing.Compute(orbit) > 0.0, "detail the reader needs");
    }
}
```

Report through the context, not through `HarnessLog`: `Check` (plus `CheckRel` / `CheckAbs` / `CheckMixed` for a numeric bound), `Fail` for a precondition that blocked the assertion, `Skip` for a case this run cannot cover, `Info` for anything that is not an assertion. There is no `ok` to thread and nothing to return.

Two things to get right: do not catch exceptions to turn them into failures, because the harness classifies `MissingMemberException` / `TypeLoadException` as game-API drift and reports that as an infrastructure failure; and despawn every spawned vehicle from a `finally`, or it keeps ticking into later tests.

## Running

Build this solution and HeadlessHarness, checked out as a sibling, in the same configuration; the `CopyToMods` targets deploy both. Then run the harness's `scripts/run-headless.ps1`.

`-Tests` filters on exact `Name` values, so renaming a test breaks the invocations that use it. All 17, each listed once:

| Feature | Filter |
| --- | --- |
| Maneuver quick-tools | `afc-set-periapsis,afc-set-apoapsis,afc-circularize,afc-set-inclination,afc-match-inclination,afc-target-identity` |
| Flyby targeting | `afc-flyby-targeting,afc-flyby-departure` |
| Multi-pass | `afc-sequence-burnstate` |
| RCS translation (pure) | `afc-rcs-allocator,afc-rcs-estimates,afc-rcs-registry,afc-rcs-lp-solver` |
| RCS translation (flight) | `afc-rcs-translation,afc-rcs-lp` |
| Core | `afc-save-scoped-reset,afc-stock-pin-guard` |

The flight tests fly whichever of `RcsTestVehicles.Candidates` the machine has (override with `KSA_HEADLESS_VEHICLES`); without one they skip.
