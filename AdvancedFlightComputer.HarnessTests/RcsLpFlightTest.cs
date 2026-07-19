using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A/B flight of the two RCS allocators on the same vehicle: one burn with
// the axis-group allocator, a refill, then one burn with the LP allocator.
// Both must complete within the minimum-impulse residual bound without a
// single engine command; the propellant comparison is logged (the LP win
// depends on how torque-balanced the save's layout is, so it is reported,
// not asserted - only a hard regression bound is enforced). On a layout
// where the zero-torque LP is infeasible the executor's fallback to groups
// must still complete the burn; the test then reports the fallback instead
// of a comparison.
public sealed class RcsLpFlightTest : IHarnessTest
{
    private const double SpawnAltitudeM = 600_000.0;
    private const double BurnDvMs = 0.5;
    private const double BurnLeadSec = 20.0;
    private const double StepSec = 0.05;
    private const double ResidualMarginMs = 0.05;

    /// <summary>Runaway guard only. The LP can legitimately spend more
    /// than groups on torque-coupled layouts: it pays for exact zero net
    /// torque with opposed thrust the group path leaves to the attitude
    /// hold (measured +49% on the default test vehicle). Its win case is
    /// layouts where the groups fight the attitude instead. Anything past
    /// this factor is a double-delivery style defect, not physics.</summary>
    private const double LpRegressionFactor = 3.0;

    public string Name => "afc-rcs-lp";

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
            vehicle = VehicleSpawner.SpawnFromSave(saveId, system, home, "HarnessRcsLpTest", orbit);
        }
        catch (InvalidOperationException e)
        {
            HarnessLog.Line($"[{Name}] SKIP '{saveId}': {e.Message}");
            return true;
        }
        HarnessLog.Line($"[{Name}] vehicle save '{saveId}': mass={vehicle.TotalMass:F0}kg");

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
        return ok;
    }

    private bool Fly(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
        driver.Step(StepSec, 40);

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        if (!cap.HasAnyTranslation)
        {
            HarnessLog.Line($"[{Name}] SKIP: save has no translation-capable RCS thrusters.");
            return true;
        }
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        fc.SetNullRot(VehicleReferenceFrame.EclBody);

        bool ok = RunBurn(vehicle, driver, RcsAllocator.Groups,
            out double groupsPropellantKg, out bool _);
        vehicle.RefillConsumables();
        driver.Step(StepSec, 20);
        ok &= RunBurn(vehicle, driver, RcsAllocator.Lp,
            out double lpPropellantKg, out bool lpSolutionUsed);

        if (!ok)
            return false;

        if (lpSolutionUsed)
        {
            HarnessLog.Line($"[{Name}] TEST A/B propellant: groups={groupsPropellantKg * 1000.0:F1}g " +
                            $"lp={lpPropellantKg * 1000.0:F1}g " +
                            $"({(lpPropellantKg - groupsPropellantKg) / Math.Max(groupsPropellantKg, 1e-9) * 100.0:+0.0;-0.0}%)");
            bool bounded = lpPropellantKg <= groupsPropellantKg * LpRegressionFactor;
            if (!bounded)
                HarnessLog.Line($"[{Name}] TEST A/B propellant: LP exceeded the regression bound " +
                                $"({LpRegressionFactor:F2}x groups) => FAIL");
            ok &= bounded;
        }
        else
        {
            HarnessLog.Line($"[{Name}] NOTE: the LP was infeasible on this layout and fell back " +
                            "to groups; completion asserted, no A/B comparison.");
        }
        return ok;
    }

    private bool RunBurn(
        Vehicle vehicle, SimDriver driver, RcsAllocator allocator,
        out double propellantKg, out bool lpSolutionUsed)
    {
        propellantKg = 0.0;
        lpSolutionUsed = false;
        FlightComputer fc = vehicle.FlightComputer;
        string label = allocator.ToString().ToLowerInvariant();

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int bestAxis = cap.BestAxis();
        double burnTimeSec = driver.Elapsed.Seconds() + BurnLeadSec;
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(new SimTime(burnTimeSec));
        if (patch == null)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): no flight-plan patch at the burn time.");
            return false;
        }
        double3 axisBody = double3.Unpack(RcsCapabilitySnapshot.AxisDirection(bestAxis));
        double3 dvDirCci = axisBody.Transform(vehicle.GetBody2Cci());
        StateVectors burnSv = patch.Orbit.GetStateVectorsAt(new SimTime(burnTimeSec));
        doubleQuat vlf2Cci = burnSv.GetVlf2ParentCci().OrIdentity();
        double3 dvVlf = (dvDirCci * BurnDvMs).Transform(vlf2Cci.Inverse());
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
            HarnessLog.Line($"[{Name}] FAIL ({label}): BurnTarget did not load.");
            return false;
        }

        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions options = exec.GetOrCreateOptions(burn.Time.Seconds(), burn.DeltaVVlf.Length());
        options.Mode = RcsExecutionMode.Rcs;
        options.Attitude = RcsAttitudeStrategy.Hold;
        options.Allocator = allocator;
        vehicle.SetEnum(FlightComputerBurnMode.Auto);

        if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): SetEnum(Auto) did not engage the executor.");
            CleanupBurns(fc);
            return false;
        }

        double expectedDuration = vehicle.TotalMass * BurnDvMs / cap.Get(bestAxis).ForceN;
        int steps = (int)((BurnLeadSec + expectedDuration * 4.0 + 60.0) / StepSec);
        double m0 = vehicle.TotalMass;
        bool completed = false;
        bool enginesQuiet = true;
        float durationAtFiringStart = -1f;
        float lastDuration = -1f;
        for (int i = 0; i < steps; i++)
        {
            driver.Step(StepSec);
            if (AnyEngineCommanded(vehicle))
            {
                enginesQuiet = false;
                break;
            }
            if (RcsExecRegistry.TryGet(vehicle.Id, out exec!) && exec.IsActive)
            {
                lpSolutionUsed |= exec.LpSecondsPerImpulse != null;
                if (driver.Elapsed.Seconds() >= bt.IgnitionTime.Seconds())
                {
                    if (durationAtFiringStart < 0f)
                        durationAtFiringStart = bt.BurnDuration;
                    lastDuration = bt.BurnDuration;
                }
            }
            else
            {
                completed = true;
                break;
            }
        }
        propellantKg = m0 - vehicle.TotalMass;

        bool ok = true;
        if (!enginesQuiet)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): a main engine received a throttle command.");
            ok = false;
        }
        if (!completed)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): burn did not complete " +
                            $"(to go {bt.DeltaVToGoCci.Length():F3}m/s).");
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
            HarnessLog.Line($"[{Name}] TEST {label}: residual={residual:F4}m/s (tol {tol:F4}) " +
                            $"propellant={propellantKg * 1000.0:F1}g => {TestSupport.Verdict(residualOk)}");
            ok &= residualOk;

            ok &= RcsOrbitCheck.Assert(Name, $"{label} orbit", vehicle.Orbit, predicted,
                initialSma, initialEcc);

            // The BurnTarget duration mirror must count down with the
            // remaining delta-V, not freeze at the total (the in-game
            // "Burn Time" display reads it).
            bool countdownOk = durationAtFiringStart > 0f
                && lastDuration >= 0f
                && lastDuration < durationAtFiringStart * 0.5f;
            HarnessLog.Line($"[{Name}] TEST {label} countdown: " +
                            $"{durationAtFiringStart:F1}s -> {lastDuration:F1}s => " +
                            $"{TestSupport.Verdict(countdownOk)}");
            ok &= countdownOk;
        }
        CleanupBurns(fc);
        return ok;
    }

    private static void CleanupBurns(FlightComputer fc)
    {
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
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
