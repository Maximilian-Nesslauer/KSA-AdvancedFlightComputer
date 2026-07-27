using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
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
public sealed class RcsLpFlightTest : AfcTest
{
    private const double SpawnAltitudeM = 600_000.0;
    private const double BurnDvMs = 0.5;
    private const double BurnLeadSec = 20.0;
    private const double StepSec = 0.05;
    private const double ResidualMarginMs = 0.05;

    public override string Name => "afc-rcs-lp";

    private double _lastSlackCostPerImpulse;
    private float3 _lastResidualTorque;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no RCS test vehicle save present.");
            return;
        }

        RcsTestPatches.Ensure();

        foreach (string saveId in saves)
        {
            RcsFlightSupport.RunOnSave(t, home, saveId, SpawnAltitudeM, "HarnessRcsLpTest",
                (vehicle, driver) => Fly(t, vehicle, driver));
        }
    }

    private void Fly(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
        driver.Step(StepSec, 40);

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        if (!cap.HasAnyTranslation)
        {
            t.Skip("save has no translation-capable RCS thrusters.");
            return;
        }
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        fc.SetNullRot(VehicleReferenceFrame.EclBody);

        bool ok = RunBurn(t, vehicle, driver, RcsAllocator.Groups,
            out double groupsPropellantKg, out bool _);
        vehicle.RefillConsumables();
        driver.Step(StepSec, 20);
        ok &= RunBurn(t, vehicle, driver, RcsAllocator.Lp,
            out double lpPropellantKg, out bool lpSolutionUsed);

        // The comparison only means anything if both burns actually flew.
        if (!ok)
            return;

        if (!lpSolutionUsed)
        {
            t.Info("the LP was infeasible on this layout and fell back to groups; " +
                   "completion asserted, no A/B comparison.");
            return;
        }

        t.Info($"A/B propellant: groups={groupsPropellantKg * 1000.0:F1}g " +
               $"lp={lpPropellantKg * 1000.0:F1}g " +
               $"({(lpPropellantKg - groupsPropellantKg) / Math.Max(groupsPropellantKg, 1e-9) * 100.0:+0.0;-0.0}%)");
        t.Info($"lp slack: {_lastSlackCostPerImpulse * 1e6:F2}mg/Ns, " +
               $"residual tau=({_lastResidualTorque.X:F2},{_lastResidualTorque.Y:F2}," +
               $"{_lastResidualTorque.Z:F2})Nms/Ns");
        // The A/B ratio is report-only: LP is legitimately far costlier
        // than groups on strongly off-CoM layouts (measured ~9.7x on the
        // asymmetric save's strong-axis burn, where exact zero torque
        // forces opposed counter-thrust the group path leaves to the
        // attitude hold), so a multiplicative bound would flag physics as
        // a defect. Per-burn residual/orbit assertions cover correctness;
        // here only a finiteness/positivity sanity guards a runaway solve.
        t.Check("A/B propellant sane",
            double.IsFinite(lpPropellantKg) && lpPropellantKg > 0.0,
            $"lp={lpPropellantKg * 1000.0:F1}g");
    }

    private bool RunBurn(
        TestContext t, Vehicle vehicle, SimDriver driver, RcsAllocator allocator,
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
        RcsFlightSupport.BurnSetup? setup =
            RcsFlightSupport.AddBurn(vehicle, driver, dvDirCci, BurnDvMs, BurnLeadSec);
        if (setup == null)
            return t.Fail(label, "no flight-plan patch or BurnTarget at the burn time");
        Burn burn = setup.Burn;
        BurnTarget bt = setup.BurnTarget;

        RcsExecution? exec = RcsFlightSupport.ArmAndEngage(
            vehicle, burn, RcsExecutionMode.Rcs, RcsAttitudeStrategy.Hold, allocator);
        if (exec == null)
        {
            RcsFlightSupport.CleanupBurns(fc);
            return t.Fail(label, "SetEnum(Auto) did not engage the executor");
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
            ok &= t.Fail($"{label} engines quiet", "a main engine received a throttle command");
        if (!result.Completed)
        {
            ok &= t.Fail(label, $"burn did not complete (to go {bt.DeltaVToGoCci.Length():F3}m/s)");
        }
        else
        {
            float residual = bt.DeltaVToGoCci.Length();
            double tol = RcsFlightSupport.ResidualToleranceMs(in cap, vehicle.TotalMass, ResidualMarginMs);
            ok &= t.Check(label, residual <= tol,
                $"residual={residual:F4}m/s (tol {tol:F4}) propellant={propellantKg * 1000.0:F1}g");

            ok &= RcsOrbitCheck.Assert(t, $"{label} orbit", vehicle.Orbit, setup.Predicted,
                setup.InitialSma, setup.InitialEcc);

            // The BurnTarget duration mirror must count down with the
            // remaining delta-V, not freeze at the total (the in-game "Burn
            // Time" display reads it). Only observable when the burn spans
            // several sample steps; a sub-step burn (high thrust over a light
            // vehicle, e.g. the asymmetric save's fast group path finishes in
            // ~12 ms, under one 50 ms step) is done before the loop samples a
            // nonzero remaining duration, so there is nothing to count down.
            const float ObservableDurationSec = 0.2f;
            if (durationAtFiringStart >= ObservableDurationSec)
            {
                ok &= t.Check($"{label} countdown",
                    lastDuration >= 0f && lastDuration < durationAtFiringStart * 0.5f,
                    $"{durationAtFiringStart:F1}s -> {lastDuration:F1}s");
            }
            else
            {
                t.Skip($"{label} countdown: burn too fast to sample " +
                       $"({durationAtFiringStart:F2}s at ignition).");
            }
        }
        RcsFlightSupport.CleanupBurns(fc);
        return ok;
    }
}
