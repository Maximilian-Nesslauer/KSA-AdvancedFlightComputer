using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// End-to-end RCS translation burn against the live simulation: spawn a
// player-built save with RCS thrusters, plan a small prograde burn, arm it
// for RCS, trigger it through the same Vehicle.SetEnum path the gauge
// button uses, and let the executor fly it closed-loop. The oracle is the
// game's own delta-V accounting (BurnTarget.DeltaVAccumCci, fed by the
// physics integrator), never a re-derivation.
//
// The vehicle comes from KSA_HEADLESS_VEHICLE (default "Test Vehicle 1");
// a save without translation-capable thrusters skips with a log line, the
// same policy the harness flight test uses for a missing save.
public sealed class RcsTranslationTest : IHarnessTest
{
    private const double SpawnAltitudeM = 500_000.0;
    private const double BurnDvMs = 0.5;
    private const double BurnLeadSec = 20.0;
    private const double StepSec = 0.05;
    private const double ResidualMarginMs = 0.05;

    public string Name => "afc-rcs-translation";

    private Vehicle? _completedVehicle;
    private Burn? _completedBurn;

    public int Run(HeadlessSession session)
    {
        CelestialSystem system = session.System;
        if (system.HomeBody is not IParentBody home || home is not Astronomical)
        {
            HarnessLog.Line($"[{Name}] FAIL: the loaded system has no home body.");
            return 1;
        }
        IReadOnlyList<string> saves = RcsTestVehicles.Resolve();
        if (saves.Count == 0)
        {
            HarnessLog.Line($"[{Name}] SKIP: no RCS test vehicle save present.");
            return 0;
        }

        RcsTestPatches.Ensure();

        bool allOk = true;
        foreach (string saveId in saves)
            allOk &= RunForSave(session, home, saveId);
        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(allOk)}");
        return allOk ? 0 : 1;
    }

    private bool RunForSave(HeadlessSession session, IParentBody home, string saveId)
    {
        CelestialSystem system = session.System;
        SimDriver driver = session.CreateDriver();
        Orbit orbit = VehicleSpawner.CircularCci(
            home, ((Astronomical)home).MeanRadius + SpawnAltitudeM, driver.Elapsed);

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(system);
        Vehicle vehicle;
        try
        {
            vehicle = VehicleSpawner.SpawnFromSave(saveId, system, home, "HarnessRcsTest", orbit);
        }
        catch (InvalidOperationException e)
        {
            HarnessLog.Line($"[{Name}] SKIP '{saveId}': {e.Message}");
            return true;
        }
        HarnessLog.Line($"[{Name}] vehicle save '{saveId}': mass={vehicle.TotalMass:F0}kg");
        _completedVehicle = null;
        _completedBurn = null;

        bool ok;
        Action<Vehicle, Burn> onCompleted = (v, b) =>
        {
            _completedVehicle = v;
            _completedBurn = b;
        };
        try
        {
            VehicleUpdateTask._forceOffRails = true;
            RcsBurnCompletions.Completed += onCompleted;
            ok = Fly(vehicle, driver);
            ok &= FlyAlignScenario(vehicle, driver);
        }
        finally
        {
            RcsBurnCompletions.Completed -= onCompleted;
            VehicleUpdateTask._forceOffRails = false;
            RcsExecRegistry.Reset();
            RcsCommandChannel.Reset();
            TestSupport.DespawnNewVehicles(system, preexisting);
        }
        return ok;
    }

    // Regression for the align-slew stall: an Align burn pointing well away
    // from the current attitude spends its whole ignition lead (and more)
    // slewing without delivering any delta-V. The no-progress watchdog must
    // not cancel that (it once did, blaming thruster coverage); the burn
    // must align, fire, and complete.
    private bool FlyAlignScenario(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        CleanupBurns(fc);
        // The Hold burn before this scenario already drained RCS tanks and
        // the 90-degree slew is propellant-hungry; without a refill the
        // scenario tests tank size, not the executor.
        vehicle.RefillConsumables();
        driver.Step(StepSec, 10);
        _completedVehicle = null;
        _completedBurn = null;
        RcsCancelLogPatch.LastReason = null;

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int bestAxis = cap.BestAxis();
        double burnTimeSec = driver.Elapsed.Seconds() + BurnLeadSec;
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(new SimTime(burnTimeSec));
        if (patch == null)
        {
            HarnessLog.Line($"[{Name}] FAIL (align): no flight-plan patch at the burn time.");
            return false;
        }

        // Perpendicular to where the best axis currently points, so the
        // executor has to slew ~90 degrees before the attitude gate opens.
        double3 axisCci = double3.Unpack(RcsCapabilitySnapshot.AxisDirection(bestAxis))
            .Transform(vehicle.GetBody2Cci());
        double3 dvDirCci = double3.Cross(axisCci, double3.UnitZ);
        if (dvDirCci.IsNearlyZero())
            dvDirCci = double3.Cross(axisCci, double3.UnitY);
        dvDirCci = dvDirCci.Normalized();

        StateVectors burnSv = patch.Orbit.GetStateVectorsAt(new SimTime(burnTimeSec));
        doubleQuat vlf2Cci = burnSv.GetVlf2ParentCci().OrIdentity();
        double3 dvVlf = (dvDirCci * BurnDvMs).Transform(vlf2Cci.Inverse());
        OrbitPointCce point = patch.Orbit.GetPointAt(new SimTime(burnTimeSec));
        Burn burn = Burn.Create(point, burnTimeSec, dvVlf, patch, vehicle);
        fc.AddBurn(burn);
        BurnTarget? bt = fc.Burn;
        if (bt == null)
        {
            HarnessLog.Line($"[{Name}] FAIL (align): BurnTarget did not load.");
            return false;
        }

        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions options = exec.GetOrCreateOptions(burn.Time.Seconds(), burn.DeltaVVlf.Length());
        options.Mode = RcsExecutionMode.Rcs;
        options.Attitude = RcsAttitudeStrategy.Align;
        vehicle.SetEnum(FlightComputerBurnMode.Auto);
        if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
        {
            HarnessLog.Line($"[{Name}] FAIL (align): SetEnum(Auto) did not engage the executor.");
            CleanupBurns(fc);
            return false;
        }
        double propellantAtStartKg = RcsPropellant.AvailableKg(vehicle);
        HarnessLog.Line($"[{Name}] align scenario: propellant={propellantAtStartKg:F0}kg " +
                        $"est slew={exec.Estimates.AlignSlewPropellantKg:F0}kg/" +
                        $"{exec.Estimates.AlignSlewDurationSec:F0}s");

        // Generous budget: ignition lead plus a slow slew plus the burn.
        int steps = (int)((BurnLeadSec + 200.0) / StepSec);
        int tracePeriod = (int)(10.0 / StepSec);
        bool completed = false;
        bool enginesQuiet = true;
        for (int i = 0; i < steps; i++)
        {
            driver.Step(StepSec);
            if (AnyEngineCommanded(vehicle))
            {
                enginesQuiet = false;
                break;
            }
            if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
            {
                completed = true;
                break;
            }
            if (i % tracePeriod == 0)
                HarnessLog.Line($"[{Name}] align t+{i * StepSec:F0}s: " +
                                $"errY={fc.ErrorAngles.Y:F4} errZ={fc.ErrorAngles.Z:F4} " +
                                $"deadband={fc.AngleDeadband:F4} " +
                                $"turnY={fc.AngleTurnaround.Y:F4} turnZ={fc.AngleTurnaround.Z:F4} " +
                                $"togo={bt.DeltaVToGoCci.Length():F3} " +
                                $"accum={bt.DeltaVAccumCci.Length():F3} " +
                                $"mode={fc.AttitudeMode}/{fc.AttitudeTrackTarget}");
        }

        bool ok = true;
        if (!enginesQuiet)
        {
            HarnessLog.Line($"[{Name}] FAIL (align): a main engine received a throttle command.");
            ok = false;
        }
        if (!completed)
        {
            HarnessLog.Line($"[{Name}] FAIL (align): burn did not complete " +
                            $"(to go {bt.DeltaVToGoCci.Length():F3}m/s).");
            ok = false;
        }
        else
        {
            float accum = bt.DeltaVAccumCci.Length();
            float residual = bt.DeltaVToGoCci.Length();
            // A cancel also deactivates the executor but never raises the
            // completion event, so the event separates a genuine completion
            // from any cancel path.
            bool viaEvent = ReferenceEquals(_completedBurn, burn);
            if (!viaEvent
                && RcsCancelLogPatch.LastReason == "no translation authority"
                && RcsPropellant.AvailableKg(vehicle) < 0.02 * propellantAtStartKg)
            {
                // Dev-propellant saves (Test Vehicle 1) can drain the whole
                // RCS tank slewing 90 degrees; the executor's propellant
                // stall cancel is the correct outcome there. The recorded
                // cancel reason plus the empty tank prove it was that
                // cancel; any other cancel (watchdog, align timeout) still
                // fails even when the tank happens to be empty.
                HarnessLog.Line($"[{Name}] SKIP (align): RCS propellant exhausted mid-slew " +
                                "(align is infeasible on this save); executor cancelled cleanly.");
            }
            else
            {
                bool deliveredOk = viaEvent
                    && accum > (float)(BurnDvMs * 0.9) && residual <= 0.1f;
                HarnessLog.Line($"[{Name}] TEST align burn: accum={accum:F3}m/s of {BurnDvMs:F2}m/s " +
                                $"residual={residual:F4}m/s completedEvent={viaEvent} => " +
                                $"{TestSupport.Verdict(deliveredOk)}");
                ok &= deliveredOk;
            }
        }
        CleanupBurns(fc);
        return ok;
    }

    private static void CleanupBurns(FlightComputer fc)
    {
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
    }

    private bool Fly(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);

        // Let the update task pick the vehicle up and the thruster states
        // (propellant availability, intended forces) materialize.
        driver.Step(StepSec, 40);

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        if (!cap.HasAnyTranslation)
        {
            HarnessLog.Line($"[{Name}] SKIP: save has no translation-capable RCS thrusters " +
                            $"({vehicle.Parts.Modules.Get<ThrusterController>().Length} thruster module(s)). " +
                            "Save a vehicle with RCS in the Vehicles window to run this test.");
            return true;
        }
        int bestAxis = cap.BestAxis();
        HarnessLog.Line($"[{Name}] capability: best axis {RcsExecutor.AxisName(bestAxis)} " +
                        $"F={cap.Get(bestAxis).ForceN:F1}N mdot={cap.Get(bestAxis).MassFlowKgS * 1000.0:F2}g/s, " +
                        $"mass={vehicle.TotalMass:F1}kg");

        // Rate-hold the spawn attitude, then plan the burn along wherever
        // the strongest translation axis already points (Teleport would be
        // the alternative, but it walks the camera/viewport chain and
        // cannot run headless). The Hold strategy then has a fully
        // feasible single-axis direction regardless of thruster layout.
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        fc.SetNullRot(VehicleReferenceFrame.EclBody);

        double burnTimeSec = driver.Elapsed.Seconds() + BurnLeadSec;
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(new SimTime(burnTimeSec));
        if (patch == null)
        {
            HarnessLog.Line($"[{Name}] FAIL: no flight-plan patch at the burn time.");
            return false;
        }
        double3 axisBody = double3.Unpack(RcsCapabilitySnapshot.AxisDirection(bestAxis));
        double3 dvDirCci = axisBody.Transform(vehicle.GetBody2Cci());
        StateVectors burnSv = patch.Orbit.GetStateVectorsAt(new SimTime(burnTimeSec));
        doubleQuat vlf2Cci = burnSv.GetVlf2ParentCci().OrIdentity();
        double3 dvVlf = (dvDirCci * BurnDvMs).Transform(vlf2Cci.Inverse());

        // Orbit oracle: the impulsive application of the target delta-V
        // through the game's own orbit math. The achieved orbit must land
        // near this prediction, not merely have received the right delta-V
        // magnitude somewhere.
        double initialSma = patch.Orbit.SemiMajorAxis;
        double initialEcc = patch.Orbit.Eccentricity;
        Orbit predicted = Orbit.CreateFromStateCci(
            patch.Orbit.Parent, new SimTime(burnTimeSec), burnSv.PositionCci,
            burnSv.VelocityCci + dvDirCci * BurnDvMs, VehicleSpawner.OrbitLineColor);

        OrbitPointCce point = patch.Orbit.GetPointAt(new SimTime(burnTimeSec));
        Burn burn = Burn.Create(point, burnTimeSec, dvVlf, patch, vehicle);
        fc.AddBurn(burn);
        BurnTarget? bt = fc.Burn;
        if (bt == null)
        {
            HarnessLog.Line($"[{Name}] FAIL: BurnTarget did not load from the added burn.");
            return false;
        }

        // Arm the burn for RCS with a fixed strategy (no estimate coin-flip
        // in the assertion path) and trigger through the stock enum sink.
        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions options = exec.GetOrCreateOptions(burn.Time.Seconds(), burn.DeltaVVlf.Length());
        options.Mode = RcsExecutionMode.Rcs;
        options.Attitude = RcsAttitudeStrategy.Hold;
        vehicle.SetEnum(FlightComputerBurnMode.Auto);

        if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
        {
            HarnessLog.Line($"[{Name}] FAIL: SetEnum(Auto) did not engage the RCS executor " +
                            $"(controllable={vehicle.IsControllable}).");
            return false;
        }
        if (fc.BurnMode != FlightComputerBurnMode.Manual)
        {
            HarnessLog.Line($"[{Name}] FAIL: BurnMode is {fc.BurnMode}, expected Manual during RCS execution.");
            return false;
        }

        double expectedDuration = vehicle.TotalMass * BurnDvMs / cap.Get(bestAxis).ForceN;
        double timeoutSec = BurnLeadSec + expectedDuration * 4.0 + 60.0;
        double m0 = vehicle.TotalMass;

        bool completed = false;
        bool enginesQuiet = true;
        int steps = (int)(timeoutSec / StepSec);
        for (int i = 0; i < steps; i++)
        {
            driver.Step(StepSec);
            if (AnyEngineCommanded(vehicle))
            {
                enginesQuiet = false;
                break;
            }
            if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
            {
                completed = true;
                break;
            }
        }

        bool ok = true;
        if (!enginesQuiet)
        {
            HarnessLog.Line($"[{Name}] FAIL: a main engine received a throttle command during the RCS burn.");
            ok = false;
        }
        if (!completed)
        {
            HarnessLog.Line($"[{Name}] FAIL: RCS burn did not complete within {timeoutSec:F0}s sim time " +
                            $"(to go {bt.DeltaVToGoCci.Length():F3}m/s of {BurnDvMs:F2}m/s).");
            ok = false;
        }
        else
        {
            float residual = bt.DeltaVToGoCci.Length();
            double impulseFloor = 0.0;
            for (int i = 0; i < 6; i++)
            {
                RcsAxisGroup g = cap.Get(i);
                if (g.IsUsable)
                    impulseFloor = Math.Max(impulseFloor, g.MinImpulseNs);
            }
            double tol = 1.5 * impulseFloor / vehicle.TotalMass + ResidualMarginMs;
            bool residualOk = residual <= tol;
            // The executor already stopped at or just past the target (its
            // completion check flips on the to-go/target dot product), so a
            // plain magnitude tolerance on the remainder suffices here.
            HarnessLog.Line($"[{Name}] TEST residual: {residual:F4}m/s (tol {tol:F4}) " +
                            $"accum={bt.DeltaVAccumCci.Length():F4}m/s target={BurnDvMs:F2}m/s => " +
                            $"{TestSupport.Verdict(residualOk)}");
            ok &= residualOk;

            double burned = m0 - vehicle.TotalMass;
            bool massOk = burned > 0.0;
            HarnessLog.Line($"[{Name}] TEST propellant: {burned * 1000.0:F1}g consumed => {TestSupport.Verdict(massOk)}");
            ok &= massOk;

            bool eventOk = ReferenceEquals(_completedVehicle, vehicle)
                && ReferenceEquals(_completedBurn, burn);
            HarnessLog.Line($"[{Name}] TEST completion event: vehicle and burn delivered => " +
                            $"{TestSupport.Verdict(eventOk)}");
            ok &= eventOk;

            ok &= RcsOrbitCheck.Assert(Name, "orbit", vehicle.Orbit, predicted,
                initialSma, initialEcc);
        }
        return ok;
    }

    private static bool AnyEngineCommanded(Vehicle vehicle)
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

}

// The harness manifest runs only Core + HeadlessHarness, so the mod's
// OnFullyLoaded never applies its patches; the RCS tests apply exactly the
// executor-relevant subset once per process (gauge/UI patches stay off:
// nothing renders headless).
internal static class RcsTestPatches
{
    private static Harmony? _harmony;

    public static void Ensure()
    {
        if (_harmony != null)
            return;
        RcsExecRegistry.Init();
        _harmony = new Harmony("com.maxi.afc.harnesstests.rcs");
        _harmony.CreateClassProcessor(typeof(RcsComputeControlPatch)).Patch();
        _harmony.CreateClassProcessor(typeof(RcsDriverPatch)).Patch();
        _harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
        _harmony.CreateClassProcessor(typeof(RcsCancelLogPatch)).Patch();
    }
}

// Cancels log their reason to the game log, which headless runs never
// write; mirror the reason into the harness log so a failed flight test
// explains itself instead of just going inactive. The recorded reason also
// gates the align scenario's propellant-exhaustion SKIP.
[HarmonyPatch(typeof(RcsExecutor), nameof(RcsExecutor.Cancel))]
internal static class RcsCancelLogPatch
{
    internal static string? LastReason;

    static void Postfix(Vehicle vehicle, string reason)
    {
        LastReason = reason;
        HarnessLog.Line($"[afc-rcs] executor cancelled '{vehicle.Id}': {reason}");
    }
}
