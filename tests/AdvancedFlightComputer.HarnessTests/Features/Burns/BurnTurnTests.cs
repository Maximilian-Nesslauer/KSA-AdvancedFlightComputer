using AdvancedFlightComputer.Features.Burns;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

internal static class BurnTurnSupport
{
    internal const double StepDt = 0.125;

    internal static Vehicle Spawn(TestContext t, IParentBody home, double rcsScale, string id)
    {
        Orbit orbit = OrbitFixtures.CircularAt(home, 400_000, Universe.GetElapsedTime());
        return VehicleFixtures.SpawnFromSaveData(t.System, home, LightProbeFixture.Design(rcsScale), id, orbit);
    }

    internal static Burn AddNode(Vehicle vehicle, double leadSeconds, double deltaV, float? plannedThrottle = null)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + leadSeconds).Seconds(), new double3(deltaV, 0.0, 0.0), patch, vehicle);
        burn.PlannedThrottle = plannedThrottle;
        vehicle.FlightComputer.AddBurn(burn);
        return burn;
    }

    internal static void ClearNodes(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        while (fc.BurnPlan.FindFirstExecutableBurn() is Burn burn)
            fc.RemoveBurn(burn);
    }

    internal static string Numbers(FlightComputer fc)
        => $"rate bit {fc.RateBit} rad/s, rcs torque {fc.RcsTorqueAuthority} rad/s^2, deadband {fc.AngleDeadband} rad, "
            + $"stall edge {BurnPointing.RestingStallEdge(fc):F4} rad, shortest aimed burn {BurnPointing.MinimumBurnSeconds(fc):F3} s, "
            + $"control {fc.ActiveControlSystem.Y}/{fc.ActiveControlSystem.Z}";
}

// The warp to the turn of a node that stock AUTO flies: where it ends, that it leaves AUTO armed, that it is AFC's only
// while Universe.AutoWarpTime still names its end, and that a cancel stops only that warp.
public sealed class BurnTurnWarpTest : AfcTest
{
    public override string Name => "afc-burn-turn-warp";

    private const double NodeLead = 400.0;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        Vehicle? prior = Program.ControlledVehicle;
        Vehicle probe = BurnTurnSupport.Spawn(t, home, LightProbeFixture.FineRcs, "HarnessTurnWarpProbe");
        SimDriver driver = t.Session.CreateDriver();
        try
        {
            Universe.AutoWarpStop(resetSimulationSpeed: true);
            TurnWarp.Reset();
            Program.ControlledVehicle = probe;
            FlightComputer fc = probe.FlightComputer;
            fc.BurnMode = FlightComputerBurnMode.Manual;
            driver.Step(BurnTurnSupport.StepDt, 4);
            BurnTurnSupport.AddNode(probe, NodeLead, 20.0);
            driver.Step(BurnTurnSupport.StepDt, 2);
            if (!t.Check("the node is loaded", fc.Burn != null))
                return;
            t.Check("no warp to the turn is offered before AUTO", TurnWarp.OfferedEnd(probe) == null);

            fc.BurnMode = FlightComputerBurnMode.Auto;
            driver.Step(BurnTurnSupport.StepDt, 2);
            BurnTarget target = fc.Burn!;
            UniverseTime now = Universe.GetElapsedTime();
            t.Info($"alignment start in {(target.AlignmentStartTime - now).Seconds():F1} s, ignition in {(target.IgnitionTime - now).Seconds():F1} s");
            UniverseTime expected = target.AlignmentStartTime - new UniverseTime(BurnTurnRules.WarpLeadSeconds);
            if (!t.Check("with AUTO on, the warp is offered to 30 s before the alignment start", TurnWarp.OfferedEnd(probe) == expected,
                $"offered {TurnWarp.OfferedEnd(probe)?.Seconds():F1}, expected {expected.Seconds():F1}"))
                return;
            t.Check("the turn starts before the ignition, so the stock warp to the burn runs past it", target.AlignmentStartTime < target.IgnitionTime);

            t.Check("the warp starts and is AFC's", TurnWarp.Start(probe) && TurnWarp.Running && Universe.AutoWarpTime == expected);
            t.Check("the warp leaves AUTO armed", fc.BurnMode == FlightComputerBurnMode.Auto);
            t.Check("while the warp runs, a second one is not offered", TurnWarp.OfferedEnd(probe) == null);
            TurnWarp.Cancel();
            t.Check("a cancel stops AFC's warp and leaves AUTO armed", !Universe.IsAutoWarpActive && !TurnWarp.Running
                && fc.BurnMode == FlightComputerBurnMode.Auto);

            TurnWarp.Start(probe);
            UniverseTime other = target.IgnitionTime;
            Universe.AutoWarpTo(other, 3.0, isUncapped: true);
            t.Check("a warp to the ignition replaces the warp to the turn", !TurnWarp.Running && Universe.AutoWarpTime == other);
            TurnWarp.Cancel();
            t.Check("a cancel then leaves the other warp running", Universe.IsAutoWarpActive && Universe.AutoWarpTime == other);
            t.Check("over the other warp the warp to the turn is offered again", TurnWarp.OfferedEnd(probe) == expected);
            Universe.AutoWarpStop(resetSimulationSpeed: true);

            TurnWarp.Start(probe);
            driver.Step(BurnTurnSupport.StepDt);
            t.Check("a step with the node in place keeps the warp", TurnWarp.Running && TurnWarp.RunningFor(probe));
            fc.RemoveBurn(fc.BurnPlan.FindFirstExecutableBurn()!);
            driver.Step(BurnTurnSupport.StepDt);
            t.Check("removing the node stops the warp at the next step", !Universe.IsAutoWarpActive && !TurnWarp.Running);

            BurnTurnSupport.AddNode(probe, NodeLead, 20.0);
            driver.Step(BurnTurnSupport.StepDt, 2);
            fc.BurnMode = FlightComputerBurnMode.Auto;
            driver.Step(BurnTurnSupport.StepDt, 2);
            target = fc.Burn!;
            expected = target.AlignmentStartTime - new UniverseTime(BurnTurnRules.WarpLeadSeconds);
            t.Check("the warp starts again", TurnWarp.Start(probe));
            // Universe.ProcessAutoWarp ends the warp at its end; the driver does not run it, so the test calls it.
            double toEnd = (expected - Universe.GetElapsedTime()).Seconds();
            int steps = (int)Math.Ceiling((toEnd + 0.5) / 1.0);
            for (int i = 0; i < steps; i++)
            {
                driver.Step(1.0);
                Universe.ProcessAutoWarp(1.0 / 60.0);
                if (!Universe.IsAutoWarpActive)
                    break;
            }
            now = Universe.GetElapsedTime();
            double beforeTurn = (fc.Burn!.AlignmentStartTime - now).Seconds();
            t.Check("the warp ends before the turn, with AUTO armed and the burn not started",
                !Universe.IsAutoWarpActive && !TurnWarp.Running && beforeTurn > 0.8 * BurnTurnRules.WarpLeadSeconds
                && beforeTurn < BurnTurnRules.WarpLeadSeconds + 2.0
                && fc.BurnMode == FlightComputerBurnMode.Auto && !fc.Burn.StartedAt.HasValue,
                $"{beforeTurn:F1} s before the turn, mode {fc.BurnMode}");
            t.Check("near the turn no warp is offered", TurnWarp.OfferedEnd(probe) == null);

            fc.BurnMode = FlightComputerBurnMode.Manual;
            Program.ControlledVehicle = prior;
            t.Check("no warp is offered for a craft that is not controlled", TurnWarp.OfferedEnd(probe) == null);
        }
        finally
        {
            TurnWarp.Cancel();
            if (Universe.IsAutoWarpActive)
                Universe.AutoWarpStop(resetSimulationSpeed: true);
            Program.ControlledVehicle = prior;
            if (!probe.IsDisposed)
            {
                BurnTurnSupport.ClearNodes(probe);
                VehicleSpawner.Despawn(probe);
            }
        }
    }
}

// The pointing check before stock AUTO flies a node, on a light probe: full-size RCS blocks turn it so far per pulse
// that stock never starts the turn, scaled blocks let stock aim it, RCS off is named, and a planned burn shorter than
// the engine needs to settle its aim is warned about.
public sealed class BurnPointingCheckTest : AfcTest
{
    public override string Name => "afc-burn-pointing-check";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        Vehicle? prior = Program.ControlledVehicle;
        SimDriver driver = t.Session.CreateDriver();

        Vehicle coarse = BurnTurnSupport.Spawn(t, home, LightProbeFixture.FullRcs, "HarnessCoarseProbe");
        try
        {
            Program.ControlledVehicle = coarse;
            coarse.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            driver.Step(BurnTurnSupport.StepDt, 4);
            BurnTurnSupport.AddNode(coarse, 600.0, 30.0);
            driver.Step(BurnTurnSupport.StepDt, 2);
            FlightComputer fc = coarse.FlightComputer;
            t.Info("full-size RCS: " + BurnTurnSupport.Numbers(fc));
            PointingCheck check = BurnPointing.CheckNextNode(coarse);
            t.Check("full-size RCS on a light probe is warned about", check.Warning is PointingWarning.CannotTurn or PointingWarning.StallBand,
                $"{check.Warning}: {check.Caution}");
        }
        finally
        {
            Program.ControlledVehicle = prior;
            if (!coarse.IsDisposed)
            {
                BurnTurnSupport.ClearNodes(coarse);
                VehicleSpawner.Despawn(coarse);
            }
        }

        Vehicle fine = BurnTurnSupport.Spawn(t, home, LightProbeFixture.FineRcs, "HarnessFineProbe");
        try
        {
            Program.ControlledVehicle = fine;
            FlightComputer fc = fine.FlightComputer;
            fc.BurnMode = FlightComputerBurnMode.Manual;
            driver.Step(BurnTurnSupport.StepDt, 4);
            BurnTurnSupport.AddNode(fine, 600.0, 30.0);
            driver.Step(BurnTurnSupport.StepDt, 2);
            t.Info("scaled RCS: " + BurnTurnSupport.Numbers(fc));
            PointingCheck check = BurnPointing.CheckNextNode(fine);
            t.Check("scaled RCS and a 30 m/s burn give no warning", check.Warning == PointingWarning.None, $"{check.Warning}: {check.Caution}");

            fc.RCSMode = FlightComputerRCSMode.Disabled;
            driver.Step(BurnTurnSupport.StepDt, 2);
            check = BurnPointing.CheckNextNode(fine);
            t.Check("RCS off is named", check.Warning == PointingWarning.RcsOff, $"{check.Warning}, control {fc.ActiveControlSystem.Y}/{fc.ActiveControlSystem.Z}");
            fc.RCSMode = FlightComputerRCSMode.Enabled;
            driver.Step(BurnTurnSupport.StepDt, 2);

            BurnTurnSupport.ClearNodes(fine);
            BurnTurnSupport.AddNode(fine, 600.0, 0.05, plannedThrottle: 1f);
            driver.Step(BurnTurnSupport.StepDt, 2);
            check = BurnPointing.CheckNextNode(fine);
            t.Check("a planned 0.05 m/s burn at full throttle is too short to aim", check.Warning == PointingWarning.ShortBurn
                && check.BurnSeconds < check.MinimumSeconds, $"{check.Warning}: {check.Caution}");

            BurnTurnSupport.ClearNodes(fine);
            BurnTurnSupport.AddNode(fine, 600.0, 0.05);
            driver.Step(BurnTurnSupport.StepDt, 2);
            check = BurnPointing.CheckNextNode(fine);
            // FlightComputer.SolveBurnThrottle stretches an unplanned burn down to the least throttle of the engine.
            t.Info($"unplanned 0.05 m/s burn: {check.Warning} {check.Caution}, least throttle {fc.ActiveEnginePerformanceMax.MinThrottle:F2}");
        }
        finally
        {
            Program.ControlledVehicle = prior;
            if (!fine.IsDisposed)
            {
                BurnTurnSupport.ClearNodes(fine);
                VehicleSpawner.Despawn(fine);
            }
        }
    }
}
