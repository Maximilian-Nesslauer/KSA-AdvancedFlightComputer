using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Check how a running burn responds when another writer takes the attitude.
// The cases call Vehicle.SetEnum directly, then step the executor to detect the change.
public sealed class RcsAttitudeYieldTest : AfcTest
{
    private const double LeadSec = 12.0;
    private const double BurnDvMs = 1.0;

    public override string Name => "afc-rcs-attitude-yield";

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

        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        TheAlignKeepsTheAttitudeItWrote(t, home, saves[0]);
        APlayerTrackClickMakesTheAlignYield(t, home, saves[0]);
        TheReleaseLeavesTheNewOwnerAlone(t, home, saves[0]);
        ARateChangeDuringHoldYields(t, home, saves[0]);
        AFrameChangeDuringHoldYields(t, home, saves[0]);
        AFrameChangeDuringCustomAlignYields(t, home, saves[0]);
        AManualSelectionDuringAlignSurvives(t, home, saves[0]);
        AManualSelectionAfterAYieldSurvives(t, home, saves[0]);
        AManualSelectionDuringHoldSurvives(t, home, saves[0]);
        SwitchingRcsOffCancelsTheBurn(t, home, saves[0]);
        ATargetChangeAndRcsOffInOneGapKeepTheTarget(t, home, saves[0]);
        ACancelRightAfterATargetChangeKeepsTheTarget(t, home, saves[0]);
        ALoadedYieldDoesNotTakeTheAttitudeBack(t, home, saves[0]);
        ALoadedHoldStillSeesATargetChange(t, home, saves[0]);
    }

    private static void TheAlignKeepsTheAttitudeItWrote(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldKeep", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;
            RcsAttitudeCommand written = exec.CommandedAttitude!.Value;

            driver.Step(0.05, 20);
            t.Check("an undisturbed align keeps its own command",
                exec.AlignCommanded
                && exec.ResolvedStrategy == RcsAttitudeStrategy.Align
                && exec.CommandedAttitude != null
                && written.Matches(fc));
        });
    }

    private static void APlayerTrackClickMakesTheAlignYield(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldClick", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;

            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            t.Check("the click reaches the flight computer",
                fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde);

            driver.Step(0.05, 20);
            t.Check("the align stands down instead of writing over the click",
                !exec.AlignCommanded
                && exec.CommandedAttitude == null
                && exec.ResolvedStrategy == RcsAttitudeStrategy.Hold);
            t.Check("the player target survives the next steps",
                fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde);
            RcsWorkerCommand? cmd = null;
            for (int i = 0; i < 40 && cmd == null; i++)
            {
                driver.Step(0.05, 4);
                RcsCommandChannel.TryGet(fc.BurnPlan, out cmd);
            }
            if (cmd == null)
                t.Skip("no command was published after the yield, so the firing gate is not observable here.");
            else
                t.Check("the burn is not steered by an error angle it does not own", !cmd.RequireAttitude);
        });
    }

    private static void TheReleaseLeavesTheNewOwnerAlone(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldRelease", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;

            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            double3 playerCoordinates = fc.CustomAttitudeTarget;
            driver.Step(0.05, 20);
            if (!t.Check("the align yielded before the release", !exec.AlignCommanded))
                return;

            RcsExecutor.Cancel(vehicle, exec, "test release");
            t.Check("the release leaves the tracker the new owner selected",
                fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde
                && fc.AttitudeMode == FlightComputerAttitudeMode.Auto);
            t.Check("the release keeps the coordinates it does not own",
                fc.CustomAttitudeTarget.Equals(playerCoordinates));
        });
    }

    // Under None the tracker reads the coordinates as a rotation rate and adds the frame rates, so
    // both fields belong to the command a hold burn wrote.
    private static void ARateChangeDuringHoldYields(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldHoldRate", (vehicle, driver) =>
        {
            if (!Engage(t, vehicle, driver, RcsAttitudeStrategy.Hold, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;
            if (!t.Check("the hold burn commands no rotation to start with",
                    fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.None
                    && fc.CustomAttitudeTarget.Equals(default(double3))))
                return;

            double3 rate = new(0.0, 0.0, 0.05);
            fc.CustomAttitudeTarget = rate;
            driver.Step(0.05, 8);
            t.Check("a rate the player set makes the hold burn yield", exec.AttitudeYielded);
            t.Check("the rate survives the steps after the yield", fc.CustomAttitudeTarget.Equals(rate));

            if (exec.IsActive)
                RcsExecutor.Cancel(vehicle, exec, "test release");
            t.Check("the release keeps the rate it does not own", fc.CustomAttitudeTarget.Equals(rate));
        });
    }

    private static void AFrameChangeDuringHoldYields(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldHoldFrame", (vehicle, driver) =>
        {
            if (!Engage(t, vehicle, driver, RcsAttitudeStrategy.Hold, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;
            VehicleReferenceFrame before = fc.AttitudeFrame;
            VehicleReferenceFrame other = before == VehicleReferenceFrame.EclBody
                ? VehicleReferenceFrame.VlfBody
                : VehicleReferenceFrame.EclBody;

            // This is the player selecting another null-rotation frame on the gauge.
            vehicle.SetEnum(other);
            if (!t.Check("the frame click reaches the flight computer", fc.AttitudeFrame == other))
                return;

            driver.Step(0.05, 8);
            t.Check("a frame the player selected makes the hold burn yield", exec.AttitudeYielded);

            if (exec.IsActive)
                RcsExecutor.Cancel(vehicle, exec, "test release");
            t.Check("the release keeps the frame it does not own", fc.AttitudeFrame == other);
        });
    }

    // A custom align keeps its frame and its euler angles, so a change to either is a takeover.
    private static void AFrameChangeDuringCustomAlignYields(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldCustomAlign", (vehicle, driver) =>
        {
            if (!Engage(t, vehicle, driver, RcsAttitudeStrategy.Align, out RcsExecution? exec,
                    forceAxis: 2))
                return;
            FlightComputer fc = vehicle.FlightComputer;
            if (fc.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.Custom)
            {
                t.Skip("the chosen control axis fell back to a built-in target on this save.");
                return;
            }

            double3 angles = fc.CustomAttitudeTarget;
            double3 changed = angles + new double3(0.0, 0.0, 0.1);
            fc.CustomAttitudeTarget = changed;
            driver.Step(0.05, 8);
            t.Check("changed euler angles make the custom align yield", exec.AttitudeYielded);

            if (exec.IsActive)
                RcsExecutor.Cancel(vehicle, exec, "test release");
            t.Check("the release keeps the angles it does not own",
                fc.CustomAttitudeTarget.Equals(changed));
        });
    }

    private static void AManualSelectionDuringAlignSurvives(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldManualAlign", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            ManualSurvives(t, vehicle, driver, exec, "align");
        });
    }

    private static void AManualSelectionAfterAYieldSurvives(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldManualAfter", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            driver.Step(0.05, 20);
            if (!t.Check("the align yielded before the manual selection", exec.AttitudeYielded))
                return;
            ManualSurvives(t, vehicle, driver, exec, "a yielded align");
        });
    }

    private static void AManualSelectionDuringHoldSurvives(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldManualHold", (vehicle, driver) =>
        {
            if (!Engage(t, vehicle, driver, RcsAttitudeStrategy.Hold, out RcsExecution? exec))
                return;
            ManualSurvives(t, vehicle, driver, exec, "hold");
        });
    }

    // Manual is the selection AFC used to replace on every step, so it has to survive the steps
    // that follow and the release.
    private static void ManualSurvives(
        TestContext t, Vehicle vehicle, SimDriver driver, RcsExecution exec, string during)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.Burn == null || !exec.IsActive)
        {
            t.Skip($"the burn ended before the manual selection during {during}.");
            return;
        }

        vehicle.SetEnum(FlightComputerAttitudeMode.Manual);
        if (!t.Check($"the manual selection reaches the flight computer during {during}",
                fc.AttitudeMode == FlightComputerAttitudeMode.Manual))
            return;

        // The comparison runs on the next executor step. Staying short keeps the rest of the
        // control lead window, so the burn target is still there to compare against.
        driver.Step(0.05, 8);
        t.Check($"manual attitude survives the steps after a selection during {during}",
            fc.AttitudeMode == FlightComputerAttitudeMode.Manual);
        t.Check($"AFC records the takeover after a selection during {during}", exec.AttitudeYielded);

        driver.Step(0.05, 32);
        t.Check($"manual attitude survives the later steps after a selection during {during}",
            fc.AttitudeMode == FlightComputerAttitudeMode.Manual);

        if (exec.IsActive)
            RcsExecutor.Cancel(vehicle, exec, "test release");
        t.Check($"manual attitude survives the release after a selection during {during}",
            fc.AttitudeMode == FlightComputerAttitudeMode.Manual);
    }

    private static void SwitchingRcsOffCancelsTheBurn(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldRcsOff", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;

            fc.RCSMode = FlightComputerRCSMode.Disabled;
            driver.Step(0.05, 20);
            t.Check("the burn stands down when the player switches RCS off", !exec.IsActive);
            t.Check("RCS stays off instead of being switched back on",
                fc.RCSMode == FlightComputerRCSMode.Disabled);
        });
    }

    // Both actions land between two steps, so the cancel runs before the per-step comparison ever
    // sees the new target. Cleanup has to read the attitude itself.
    private static void ATargetChangeAndRcsOffInOneGapKeepTheTarget(
        TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldGap", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;

            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            double3 coordinates = fc.CustomAttitudeTarget;
            fc.RCSMode = FlightComputerRCSMode.Disabled;

            driver.Step(0.05, 20);
            t.Check("the burn stands down on the switched off actuator", !exec.IsActive);
            t.Check("RCS stays off", fc.RCSMode == FlightComputerRCSMode.Disabled);
            t.Check("the cancel leaves the target the player selected",
                fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde
                && fc.AttitudeMode == FlightComputerAttitudeMode.Auto);
            // Prograde derives its frame, so this case checks the target and coordinates.
            t.Check("the cancel leaves the coordinates alone",
                fc.CustomAttitudeTarget.Equals(coordinates));
        });
    }

    // A cancel straight after the change reaches cleanup with no step at all in between.
    private static void ACancelRightAfterATargetChangeKeepsTheTarget(
        TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldCancelGap", (vehicle, driver) =>
        {
            if (!Align(t, vehicle, driver, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;

            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            double3 coordinates = fc.CustomAttitudeTarget;
            RcsExecutor.Cancel(vehicle, exec, "test release");

            t.Check("the cancel leaves the target the player selected",
                fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde
                && fc.AttitudeMode == FlightComputerAttitudeMode.Auto);
            t.Check("the cancel leaves the coordinates alone",
                fc.CustomAttitudeTarget.Equals(coordinates));
            t.Check("the execution ends", !exec.IsActive);
        });
    }

    // A load clears the transient state and the next step reconciles. RCS enabled before
    // acquisition is the branch where no forced RCS marker survives the save.
    private static void ALoadedYieldDoesNotTakeTheAttitudeBack(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldLoadedYield", (vehicle, driver) =>
        {
            if (!Engage(t, vehicle, driver, RcsAttitudeStrategy.Align, out RcsExecution? exec))
                return;
            FlightComputer fc = vehicle.FlightComputer;
            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            driver.Step(0.05, 20);
            if (!t.Check("the align yielded before the load", exec.AttitudeYielded && !exec.ForcedRcsOn))
                return;

            vehicle.SetEnum(FlightComputerAttitudeMode.Manual);
            SimulateLoad(exec);
            driver.Step(0.05, 40);
            t.Check("a loaded yield keeps the attitude with the player",
                fc.AttitudeMode == FlightComputerAttitudeMode.Manual && exec.AttitudeYielded);
            t.Check("a loaded yield does not record a command again", exec.CommandedAttitude == null);
        });
    }

    private static void ALoadedHoldStillSeesATargetChange(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessYieldLoadedHold", (vehicle, driver) =>
        {
            // RCS disabled before acquisition is the other reconstruction branch, where the
            // forced RCS marker survives the save.
            if (!Engage(t, vehicle, driver, RcsAttitudeStrategy.Hold, out RcsExecution? exec,
                    FlightComputerRCSMode.Disabled))
                return;
            FlightComputer fc = vehicle.FlightComputer;
            if (!t.Check("the hold burn took the attitude and the RCS mode",
                    exec.ForcedAttitudeAuto && exec.ForcedRcsOn))
                return;

            SimulateLoad(exec);
            driver.Step(0.05, 20);
            if (!t.Check("the loaded hold burn has a command to compare against",
                    exec.CommandedAttitude != null && exec.ForcedAttitudeAuto))
                return;

            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            driver.Step(0.05, 20);
            t.Check("a loaded hold burn still sees the target change", exec.AttitudeYielded);

            if (exec.IsActive)
                RcsExecutor.Cancel(vehicle, exec, "test release");
            t.Check("the release after a loaded yield leaves the player in Auto",
                fc.AttitudeMode == FlightComputerAttitudeMode.Auto
                && fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde);
        });
    }

    // Clears exactly the fields a save does not carry, so the next driver step reconciles the
    // execution the way it does after loading.
    private static void SimulateLoad(RcsExecution exec)
    {
        exec.ActiveBurn = null;
        exec.CommandedAttitude = null;
        exec.ControlTaken = false;
        exec.ReconciledAfterLoad = false;
    }

    private static bool Align(TestContext t, Vehicle vehicle, SimDriver driver, out RcsExecution exec)
        => Engage(t, vehicle, driver, RcsAttitudeStrategy.Align, out exec);

    private static bool Engage(
        TestContext t, Vehicle vehicle, SimDriver driver, RcsAttitudeStrategy attitude, out RcsExecution exec,
        FlightComputerRCSMode rcsBefore = FlightComputerRCSMode.Enabled, int forceAxis = -1)
    {
        exec = null!;
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        fc.BurnMode = FlightComputerBurnMode.Manual;

        // AFC records a command only where it writes the attitude itself, so the case decides the
        // mode it takes over from. The RCS mode decides which reconstruction branch a load takes.
        fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
        fc.RCSMode = rcsBefore;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
        driver.Step(0.05, 40);
        if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
        {
            t.Skip("save has no active RCS translation capability.");
            return false;
        }
        if (RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, BurnDvMs, LeadSec) == null)
        {
            t.Fail("attitude yield setup", "no patch or loaded burn target");
            return false;
        }

        double burnTimeSec = (driver.Elapsed + LeadSec).Seconds();
        RcsExecution prepared = RcsExecRegistry.GetOrCreate(vehicle.Id);
        prepared.GetOrCreateOptions(burnTimeSec, BurnDvMs).Attitude = attitude;

        RcsExecutor.Activate(vehicle);
        if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? started) || !started.IsActive)
        {
            t.Fail("attitude yield setup", "the RCS burn did not engage");
            return false;
        }
        if (started.ResolvedStrategy != attitude)
        {
            t.Skip($"save does not resolve this burn to {attitude}.");
            return false;
        }

        // An align on a control axis other than the burn direction writes a custom euler target
        // instead of a built-in one, which is the command shape this save does not select on its own.
        if (forceAxis >= 0)
        {
            started.ResolvedAxis = forceAxis;
            started.AlignCommanded = false;
            started.CommandedAttitude = null;
        }

        // AFC writes the attitude only inside the control lead window, so wait for that write.
        for (int i = 0; i < 60 && started.CommandedAttitude == null; i++)
            driver.Step(0.05, 4);
        if (started.CommandedAttitude == null)
        {
            t.Fail("attitude yield setup", $"{attitude} recorded no command inside its control lead window");
            return false;
        }
        exec = started;
        return true;
    }

    private static void Fly(
        TestContext t, IParentBody home, string save, string spawnName, Action<Vehicle, SimDriver> body)
        => RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, spawnName, (vehicle, driver) =>
        {
            try
            {
                body(vehicle, driver);
            }
            finally
            {
                VehicleControlOwnership.ReleaseAll(vehicle);
                vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            }
        });
}
