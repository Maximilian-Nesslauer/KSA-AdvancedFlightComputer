using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Shared flight machinery for the RCS translation tests: vehicle spawn plus
// teardown, burn setup with the impulsive orbit oracle, RCS arming through the
// gauge path, and the step-until-inactive loop. One driver for every scenario
// keeps each test to its own assertions and gives a strategy x allocator x angle
// sweep a single place to build from. The oracle is always the game's own orbit
// math, never a re-derivation.
internal static class RcsFlightSupport
{
    // Spawns a saved vehicle into a fresh circular orbit, runs the body, and
    // always tears the vehicle down afterwards. A save with no loadable
    // vehicle skips with a log line, matching the per-save contract of the
    // flight tests.
    internal static void RunOnSave(
        TestContext t, IParentBody home, string saveId,
        double altitudeM, string spawnName, Action<Vehicle, SimDriver> body)
    {
        CelestialSystem system = t.System;
        SimDriver driver = t.Session.CreateDriver();
        Orbit orbit = OrbitFixtures.CircularAt(home, altitudeM, driver.Elapsed);
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(system);
        // The spawn lives inside the guarded block: Astronomical's constructor
        // registers the vehicle before the Vehicle body's own validation can
        // throw, so a spawn failure other than the expected
        // InvalidOperationException must still reach the despawn cleanup or a
        // half-built vehicle leaks into every later test.
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(saveId, system, home, spawnName, orbit);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saveId}': {e.Message}");
                return;
            }
            t.Info($"vehicle save '{saveId}': mass={vehicle.TotalMass:F0}kg");
            PhysicsBubble._forceOffRails = true;
            body(vehicle, driver);
        }
        finally
        {
            PhysicsBubble._forceOffRails = false;
            RcsExecRegistry.Reset();
            RcsCommandChannel.Reset();
            TestSupport.DespawnNewVehicles(system, preexisting);
        }
    }

    // The CCI direction the given signed body axis points in right now.
    internal static double3 AxisDirCci(Vehicle vehicle, int axis)
        => double3.Unpack(RcsCapabilitySnapshot.AxisDirection(axis)).Transform(vehicle.GetBody2Cci());

    internal sealed class BurnSetup
    {
        public required Burn Burn { get; init; }
        public required BurnTarget BurnTarget { get; init; }

        // The orbit that results from applying the exact target delta-V
        // impulsively through the game's own orbit math: the flight assertion
        // oracle. Meaningful only when the burn direction is the one whose
        // effect the test wants to check.
        public required Orbit Predicted { get; init; }
        public required double InitialSma { get; init; }
        public required double InitialEcc { get; init; }
    }

    // Plans a burn of magnitude dvMs along the given CCI direction at leadSec
    // in the future, loads it, and returns it with the loaded BurnTarget and
    // the impulsive orbit oracle. Null when no flight-plan patch covers the
    // burn time or the BurnTarget did not load.
    internal static BurnSetup? AddBurn(
        Vehicle vehicle, SimDriver driver, double3 dvDirCci, double dvMs, double leadSec)
    {
        FlightComputer fc = vehicle.FlightComputer;
        UniverseTime burnTime = driver.Elapsed + leadSec;
        double burnTimeSec = burnTime.Seconds();
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(burnTime);
        if (patch == null)
            return null;
        StateVectors burnSv = patch.Orbit.GetStateVectorsAt(burnTime);
        doubleQuat vlf2Cci = burnSv.GetVlf2ParentCci().OrIdentity();
        double3 dvVlf = (dvDirCci * dvMs).Transform(vlf2Cci.Inverse());
        Orbit predicted = OrbitFixtures.ApplyImpulse(patch.Orbit, dvDirCci * dvMs, burnTime);
        OrbitPointCce point = patch.Orbit.GetPointAt(burnTime);
        Burn burn = Burn.Create(point, burnTimeSec, dvVlf, patch, vehicle);
        fc.AddBurn(burn);
        BurnTarget? bt = fc.Burn;
        if (bt == null)
            return null;
        return new BurnSetup
        {
            Burn = burn,
            BurnTarget = bt,
            Predicted = predicted,
            InitialSma = patch.Orbit.SemiMajorAxis,
            InitialEcc = patch.Orbit.Eccentricity,
        };
    }

    // Arms the loaded burn for RCS with a fixed strategy and triggers it
    // through the same Vehicle.SetEnum path the gauge button uses. Returns the
    // engaged execution, or null when it did not engage.
    internal static RcsExecution? ArmAndEngage(
        Vehicle vehicle, Burn burn, RcsExecutionMode mode,
        RcsAttitudeStrategy attitude, RcsAllocator allocator)
    {
        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions options = exec.GetOrCreateOptions(burn.Time.Seconds(), burn.DeltaVVlf.Length());
        options.Mode = mode;
        options.Attitude = attitude;
        options.Allocator = allocator;
        vehicle.SetEnum(FlightComputerBurnMode.Auto);
        return RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? engaged) && engaged.IsActive
            ? engaged
            : null;
    }

    internal readonly struct RunResult
    {
        public bool Completed { get; init; }
        public bool EnginesQuiet { get; init; }
    }

    // Steps the sim until the execution deactivates (completed or cancelled)
    // or a main engine receives a throttle command, whichever comes first.
    // onActiveStep runs each tick the execution is still active, for
    // per-scenario sampling (firing nozzles, LP state, countdown mirror); its
    // int argument is the step index, so a scenario can throttle its tracing.
    // The executor ends a burn in the ApplyVehicleSolvers postfix of the last step, so a stock flag it
    // raised is still pending on return. The next step's FlightComputer.RaisePendingAlerts acts on it.
    internal static RunResult RunUntilInactive(
        Vehicle vehicle, SimDriver driver, double stepSec, int maxSteps,
        Action<int, RcsExecution>? onActiveStep = null)
    {
        bool completed = false;
        bool enginesQuiet = true;
        for (int i = 0; i < maxSteps; i++)
        {
            driver.Step(stepSec);
            if (AnyEngineCommanded(vehicle))
            {
                enginesQuiet = false;
                break;
            }
            if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) || !exec.IsActive)
            {
                completed = true;
                break;
            }
            onActiveStep?.Invoke(i, exec);
        }
        return new RunResult { Completed = completed, EnginesQuiet = enginesQuiet };
    }

    // Largest per-axis minimum-impulse floor as a delta-V tolerance for the
    // vehicle mass, plus a flat margin. The completion check stops at or just
    // past the target, so a magnitude tolerance on the remainder suffices.
    internal static double ResidualToleranceMs(
        in RcsCapabilitySnapshot cap, double totalMass, double marginMs)
    {
        double impulseFloor = 0.0;
        for (int i = 0; i < 6; i++)
        {
            RcsAxisGroup g = cap.Get(i);
            if (g.IsUsable)
                impulseFloor = Math.Max(impulseFloor, g.MinImpulseNs);
        }
        return 1.5 * impulseFloor / totalMass + marginMs;
    }

    internal static bool AnyEngineCommanded(Vehicle vehicle)
    {
        if (!ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var stateList))
            return false;
        var enumerator = new ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
            .StateList.ModuleAndStateEnumerator(stateList);
        while (enumerator.MoveNext())
        {
            if (enumerator.Current.State.CommandThrottle > 0f)
                return true;
        }
        return false;
    }

    internal static void CleanupBurns(FlightComputer fc)
    {
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
    }

    // Puts one vehicle into the state of an RCS burn that flies the given burn, with no control
    // taken, so a test can end it through RcsExecutor.Complete or RcsExecutor.StopOutOfPropellant on
    // any vehicle, also one without thrusters, and the release writes nothing to the flight computer.
    internal static RcsExecution SeedFlyingExecution(Vehicle vehicle, Burn burn)
    {
        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        double timeSec = burn.Time.Seconds();
        double dvMs = burn.DeltaVVlf.Length();
        exec.ActiveBurn = burn;
        exec.ActiveBurnTimeSec = timeSec;
        exec.ActiveBurnDvMs = dvMs;
        exec.ResolvedStrategy = RcsAttitudeStrategy.Hold;
        exec.ResolvedAxis = -1;
        exec.ControlTaken = true;
        exec.ReconciledAfterLoad = true;
        exec.GetOrCreateOptions(timeSec, dvMs).Mode = RcsExecutionMode.Rcs;
        return exec;
    }

    // Reads how the executor ended a burn. It ends a burn through the stock flags
    // FlightComputer.AutoBurnCompleted and AutoBurnStoppedOutOfPropellant, which stay pending until
    // the next driver step, where FlightComputer.RaisePendingAlerts clears them and ends the burn.
    // AnyEnd, CompletedOn and StoppedOutOfPropellantOn read the pending flags. A prefix on
    // RaisePendingAlerts counts the ends stock read, so a check over several steps also sees an end
    // that stock has already cleared. A cancel raises neither flag.
    internal sealed class CompletionWatcher : IDisposable
    {
        private static CompletionWatcher? _active;
        private readonly Harmony _harmony = new("com.maxi.afc.harnesstests.rcs.completions");
        private readonly Vehicle _vehicle;

        public CompletionWatcher(Vehicle vehicle)
        {
            _vehicle = vehicle;
            Reset();
            _harmony.Patch(
                AccessTools.Method(typeof(FlightComputer), nameof(FlightComputer.RaisePendingAlerts),
                    new[] { typeof(Vehicle) }),
                prefix: new HarmonyMethod(typeof(CompletionWatcher), nameof(PendingAlertsPrefix)));
            _active = this;
        }

        // The ends that stock FlightComputer.RaisePendingAlerts read for this vehicle since Reset.
        public int StockReadEnds { get; private set; }

        public bool AnyEnd => _vehicle.FlightComputer.AutoBurnCompleted
                              || _vehicle.FlightComputer.AutoBurnStoppedOutOfPropellant;

        public bool NoEndSinceReset => !AnyEnd && StockReadEnds == 0;

        public bool CompletedOn(Burn burn)
            => _vehicle.FlightComputer.AutoBurnCompleted && IsLoadedFirst(burn);

        public bool StoppedOutOfPropellantOn(Burn burn)
            => _vehicle.FlightComputer.AutoBurnStoppedOutOfPropellant && IsLoadedFirst(burn);

        // The burn FlightComputer.EndAutoBurn would end, which is the first executable one.
        private bool IsLoadedFirst(Burn burn)
        {
            FlightComputer fc = _vehicle.FlightComputer;
            return ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), burn)
                   && StockBurnIdentity.IsLoaded(fc.Burn, burn);
        }

        // Also clears a pending flag, because a case that ends a burn without a step never lets stock
        // read it, and the next case would start with that end still pending.
        public void Reset()
        {
            _vehicle.FlightComputer.AutoBurnCompleted = false;
            _vehicle.FlightComputer.AutoBurnStoppedOutOfPropellant = false;
            StockReadEnds = 0;
        }

        public void Dispose()
        {
            _harmony.UnpatchAll(_harmony.Id);
            if (ReferenceEquals(_active, this))
                _active = null;
            Reset();
        }

        private static void PendingAlertsPrefix(FlightComputer __instance, Vehicle vehicle)
        {
            CompletionWatcher? watcher = _active;
            if (watcher != null && ReferenceEquals(vehicle, watcher._vehicle)
                && (__instance.AutoBurnCompleted || __instance.AutoBurnStoppedOutOfPropellant))
                watcher.StockReadEnds++;
        }
    }
}
