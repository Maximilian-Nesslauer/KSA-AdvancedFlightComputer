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
// of a comparison. Spawn, burn setup, arming, and the step loop live in
// RcsFlightSupport.
public sealed class RcsLpFlightTest : IHarnessTest
{
    private const double SpawnAltitudeM = 600_000.0;
    private const double BurnDvMs = 0.5;
    private const double BurnLeadSec = 20.0;
    private const double StepSec = 0.05;
    private const double ResidualMarginMs = 0.05;

    /// <summary>Runaway guard only. On the dev save "Test Vehicle 1" the
    /// LP reads ~+40% over groups, but that gap is a tank artifact, not
    /// allocator waste: the groups pattern drains a partially present
    /// reactant mix (ResourceManager.MassChange under-withdraws while
    /// thrust stays full), the LP support draws from fully stocked mixes
    /// and pays the real model price. Anything past this factor is a
    /// double-delivery style defect, not physics.</summary>
    private const double LpRegressionFactor = 3.0;

    public string Name => "afc-rcs-lp";

    private double _lastSlackCostPerImpulse;
    private float3 _lastResidualTorque;

    public int Run(HeadlessSession session)
    {
        CelestialSystem system = session.System;
        if (system.HomeBody is not IParentBody home || home is not Astronomical)
        {
            HarnessLog.Line($"[{Name}] FAIL: the loaded system has no home body.");
            return 1;
        }
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            HarnessLog.Line($"[{Name}] SKIP: no RCS test vehicle save present.");
            return 0;
        }

        RcsTestPatches.Ensure();

        bool allOk = true;
        foreach (string saveId in saves)
            allOk &= RcsFlightSupport.RunOnSave(
                session, home, saveId, SpawnAltitudeM, "HarnessRcsLpTest", Name,
                (vehicle, driver) => Fly(vehicle, driver));
        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(allOk)}");
        return allOk ? 0 : 1;
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
            HarnessLog.Line($"[{Name}] lp slack: {_lastSlackCostPerImpulse * 1e6:F2}mg/Ns, " +
                            $"residual tau=({_lastResidualTorque.X:F2},{_lastResidualTorque.Y:F2}," +
                            $"{_lastResidualTorque.Z:F2})Nms/Ns");
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
        _lastSlackCostPerImpulse = 0.0;
        _lastResidualTorque = default;
        FlightComputer fc = vehicle.FlightComputer;
        string label = allocator.ToString().ToLowerInvariant();

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int bestAxis = cap.BestAxis();
        double3 dvDirCci = RcsFlightSupport.AxisDirCci(vehicle, bestAxis);
        RcsFlightSupport.BurnSetup? setup = RcsFlightSupport.AddBurn(vehicle, driver, dvDirCci, BurnDvMs, BurnLeadSec);
        if (setup == null)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): no flight-plan patch or BurnTarget at the burn time.");
            return false;
        }
        Burn burn = setup.Burn;
        BurnTarget bt = setup.BurnTarget;

        RcsExecution? exec = RcsFlightSupport.ArmAndEngage(
            vehicle, burn, RcsExecutionMode.Rcs, RcsAttitudeStrategy.Hold, allocator);
        if (exec == null)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): SetEnum(Auto) did not engage the executor.");
            RcsFlightSupport.CleanupBurns(fc);
            return false;
        }

        double expectedDuration = vehicle.TotalMass * BurnDvMs / cap.Get(bestAxis).ForceN;
        // 4x duration slack for min-pulse round-up and trim, plus a flat
        // 60 s pad for the pre-ignition coast wobble.
        int steps = (int)((BurnLeadSec + expectedDuration * 4.0 + 60.0) / StepSec);
        double m0 = vehicle.TotalMass;
        bool lpUsed = false;
        float durationAtFiringStart = -1f;
        float lastDuration = -1f;
        RcsFlightSupport.RunResult result = RcsFlightSupport.RunUntilInactive(
            vehicle, driver, StepSec, steps,
            (_, active) =>
            {
                if (active.LpSecondsPerImpulse != null)
                {
                    lpUsed = true;
                    _lastSlackCostPerImpulse = active.LpSlackCostPerImpulse;
                    _lastResidualTorque = active.LpResidualTorquePerNs;
                }
                if (driver.Elapsed.Seconds() >= bt.IgnitionTime.Seconds())
                {
                    if (durationAtFiringStart < 0f)
                        durationAtFiringStart = bt.BurnDuration;
                    lastDuration = bt.BurnDuration;
                }
            });
        propellantKg = m0 - vehicle.TotalMass;
        lpSolutionUsed = lpUsed;

        bool ok = true;
        if (!result.EnginesQuiet)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): a main engine received a throttle command.");
            ok = false;
        }
        if (!result.Completed)
        {
            HarnessLog.Line($"[{Name}] FAIL ({label}): burn did not complete " +
                            $"(to go {bt.DeltaVToGoCci.Length():F3}m/s).");
            ok = false;
        }
        else
        {
            float residual = bt.DeltaVToGoCci.Length();
            double tol = RcsFlightSupport.ResidualToleranceMs(in cap, vehicle.TotalMass, ResidualMarginMs);
            bool residualOk = residual <= tol;
            HarnessLog.Line($"[{Name}] TEST {label}: residual={residual:F4}m/s (tol {tol:F4}) " +
                            $"propellant={propellantKg * 1000.0:F1}g => {TestSupport.Verdict(residualOk)}");
            ok &= residualOk;

            ok &= RcsOrbitCheck.Assert(Name, $"{label} orbit", vehicle.Orbit, setup.Predicted,
                setup.InitialSma, setup.InitialEcc);

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
        RcsFlightSupport.CleanupBurns(fc);
        return ok;
    }
}
