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
    private const string DefaultSave = "Test Vehicle 1";
    private const double SpawnAltitudeM = 500_000.0;
    private const double BurnDvMs = 0.5;
    private const double BurnLeadSec = 20.0;
    private const double StepSec = 0.05;
    private const double ResidualMarginMs = 0.05;

    public string Name => "afc-rcs-translation";

    public int Run(HeadlessSession session)
    {
        string saveId = Environment.GetEnvironmentVariable(TestSupport.VehicleEnvVar) ?? DefaultSave;

        CelestialSystem system = session.System;
        if (system.HomeBody is not IParentBody home || home is not Astronomical)
        {
            HarnessLog.Line($"[{Name}] FAIL: the loaded system has no home body.");
            return 1;
        }

        RcsTestPatches.Ensure();

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
            HarnessLog.Line($"[{Name}] SKIP: {e.Message}");
            return 0;
        }

        bool ok;
        try
        {
            VehicleUpdateTask._forceOffRails = true;
            ok = Fly(vehicle, driver);
        }
        finally
        {
            VehicleUpdateTask._forceOffRails = false;
            RcsExecRegistry.Reset();
            RcsCommandChannel.Reset();
            TestSupport.DespawnNewVehicles(system, preexisting);
        }

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
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
            // Overshoot flips the to-go vector against the target; the dot
            // check distinguishes "stopped just past the target" (fine,
            // within tolerance) from a runaway.
            HarnessLog.Line($"[{Name}] TEST residual: {residual:F4}m/s (tol {tol:F4}) " +
                            $"accum={bt.DeltaVAccumCci.Length():F4}m/s target={BurnDvMs:F2}m/s => " +
                            $"{TestSupport.Verdict(residualOk)}");
            ok &= residualOk;

            double burned = m0 - vehicle.TotalMass;
            bool massOk = burned > 0.0;
            HarnessLog.Line($"[{Name}] TEST propellant: {burned * 1000.0:F1}g consumed => {TestSupport.Verdict(massOk)}");
            ok &= massOk;
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
    }
}
