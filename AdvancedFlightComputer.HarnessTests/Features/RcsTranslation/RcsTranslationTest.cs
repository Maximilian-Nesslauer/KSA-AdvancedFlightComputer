using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
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
// The vehicles come from TestSupport.ResolveVehicleSaves: the present members
// of RcsTestVehicles.Candidates, or KSA_HEADLESS_VEHICLES when set. A save
// without translation-capable thrusters skips with a log line. The spawn,
// burn setup, arming, and step loop live in RcsFlightSupport.
public sealed class RcsTranslationTest : AfcTest
{
    private const double SpawnAltitudeM = 500_000.0;
    private const double BurnDvMs = 0.5;
    private const double BurnLeadSec = 20.0;
    private const double StepSec = 0.05;
    private const double ResidualMarginMs = 0.05;

    public override string Name => "afc-rcs-translation";

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
            RcsFlightSupport.RunOnSave(t, home, saveId, SpawnAltitudeM, "HarnessRcsTest",
                (vehicle, driver) =>
                {
                    using var watcher = new RcsFlightSupport.CompletionWatcher();
                    Fly(t, vehicle, driver, watcher);
                    FlyAlignScenario(t, vehicle, driver, watcher);
                    FlyDeferredAlignCheck(t, vehicle, driver);
                    FlyRcsToggleScenario(t, vehicle, driver, watcher);
                });
        }
    }

    // Regression for the align-slew stall: an Align burn pointing well away
    // from the current attitude spends its whole ignition lead (and more)
    // slewing without delivering any delta-V. The no-progress watchdog must
    // not cancel that (it once did, blaming thruster coverage); the burn
    // must align, fire, and complete.
    private static void FlyAlignScenario(
        TestContext t, Vehicle vehicle, SimDriver driver, RcsFlightSupport.CompletionWatcher watcher)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        // The Hold burn before this scenario already drained RCS tanks and
        // the 90-degree slew is propellant-hungry; without a refill the
        // scenario tests tank size, not the executor.
        vehicle.RefillConsumables();
        driver.Step(StepSec, 10);
        watcher.Reset();
        RcsCancelLogPatch.LastReason = null;

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int bestAxis = cap.BestAxis();

        // Perpendicular to where the best axis currently points, so the
        // executor has to slew ~90 degrees before the attitude gate opens.
        double3 axisCci = RcsFlightSupport.AxisDirCci(vehicle, bestAxis);
        double3 dvDirCci = double3.Cross(axisCci, double3.UnitZ);
        if (dvDirCci.IsNearlyZero())
            dvDirCci = double3.Cross(axisCci, double3.UnitY);
        dvDirCci = dvDirCci.Normalized();

        RcsFlightSupport.BurnSetup? setup =
            RcsFlightSupport.AddBurn(vehicle, driver, dvDirCci, BurnDvMs, BurnLeadSec);
        if (setup == null)
        {
            t.Fail("align", "no flight-plan patch or BurnTarget at the burn time");
            return;
        }
        Burn burn = setup.Burn;
        BurnTarget bt = setup.BurnTarget;

        RcsExecution? exec = RcsFlightSupport.ArmAndEngage(
            vehicle, burn, RcsExecutionMode.Rcs, RcsAttitudeStrategy.Align, RcsAllocator.Groups);
        if (exec == null)
        {
            t.Fail("align", "SetEnum(Auto) did not engage the executor");
            RcsFlightSupport.CleanupBurns(fc);
            return;
        }
        double propellantAtStartKg = RcsPropellant.AvailableKg(vehicle);
        t.Info($"align scenario: propellant={propellantAtStartKg:F0}kg " +
               $"est slew={exec.Estimates.AlignSlewPropellantKg:F0}kg/" +
               $"{exec.Estimates.AlignSlewDurationSec:F0}s");

        // Generous budget: ignition lead plus a slow slew plus the burn.
        int steps = (int)((BurnLeadSec + 200.0) / StepSec);
        int tracePeriod = (int)(10.0 / StepSec);
        RcsFlightSupport.RunResult result = RcsFlightSupport.RunUntilInactive(
            vehicle, driver, StepSec, steps,
            (i, _) =>
            {
                if (i % tracePeriod == 0)
                    t.Info($"align t+{i * StepSec:F0}s: " +
                           $"errY={fc.ErrorAngles.Y:F4} errZ={fc.ErrorAngles.Z:F4} " +
                           $"deadband={fc.AngleDeadband:F4} " +
                           $"turnY={fc.AngleTurnaround.Y:F4} turnZ={fc.AngleTurnaround.Z:F4} " +
                           $"togo={bt.DeltaVToGoCci.Length():F3} " +
                           $"accum={bt.DeltaVAccumCci.Length():F3} " +
                           $"mode={fc.AttitudeMode}/{fc.AttitudeTrackTarget}");
            });

        if (!result.EnginesQuiet)
            t.Fail("align engines quiet", "a main engine received a throttle command");
        if (!result.Completed)
        {
            t.Fail("align burn", $"burn did not complete (to go {bt.DeltaVToGoCci.Length():F3}m/s)");
        }
        else
        {
            float accum = bt.DeltaVAccumCci.Length();
            float residual = bt.DeltaVToGoCci.Length();
            // A cancel also deactivates the executor but never raises the
            // completion event, so the event separates a genuine completion
            // from any cancel path.
            bool viaEvent = ReferenceEquals(watcher.LastBurn, burn);
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
                t.Skip("align: RCS propellant exhausted mid-slew " +
                       "(align is infeasible on this save); executor cancelled cleanly.");
            }
            else
            {
                bool deliveredOk = viaEvent && accum > (float)(BurnDvMs * 0.9) && residual <= 0.1f;
                t.Check("align burn", deliveredOk,
                    $"accum={accum:F3}m/s of {BurnDvMs:F2}m/s residual={residual:F4}m/s " +
                    $"completedEvent={viaEvent}");

                if (viaEvent)
                {
                    // The 90-degree offset guarantees a real slew, so the
                    // telemetry's slew bucket must have caught propellant.
                    RcsFuelSummary fuel = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? done)
                        ? done.LastFuel : default;
                    t.Check("align fuel",
                        fuel.Valid && fuel.SlewKg > 0.0 && fuel.TotalKg >= fuel.SlewKg,
                        $"total={fuel.TotalKg * 1000.0:F1}g slew {fuel.SlewKg * 1000.0:F1}g");
                }
            }
        }
        RcsFlightSupport.CleanupBurns(fc);
    }

    // The align tracker must stay idle through the coast and only be
    // commanded inside the ignition lead window (deferring it is what keeps
    // a long warped wait from burning the tank on attitude hold). Observes
    // the tracker directly, then cancels; full align completion is covered
    // by FlyAlignScenario.
    private static void FlyDeferredAlignCheck(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        const double DeferredLeadSec = 150.0;
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        vehicle.RefillConsumables();
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        fc.SetNullRot(VehicleReferenceFrame.EclBody);
        driver.Step(StepSec, 10);

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int bestAxis = cap.BestAxis();
        double3 dvDirCci = RcsFlightSupport.AxisDirCci(vehicle, bestAxis);
        RcsFlightSupport.BurnSetup? setup =
            RcsFlightSupport.AddBurn(vehicle, driver, dvDirCci, BurnDvMs, DeferredLeadSec);
        if (setup == null)
        {
            t.Fail("deferred align", "no flight-plan patch or BurnTarget at the burn time");
            return;
        }
        Burn burn = setup.Burn;
        double burnTimeSec = burn.Time.Seconds();

        RcsExecution? exec = RcsFlightSupport.ArmAndEngage(
            vehicle, burn, RcsExecutionMode.Rcs, RcsAttitudeStrategy.Align, RcsAllocator.Groups);
        if (exec == null)
        {
            t.Fail("deferred align", "SetEnum(Auto) did not engage the executor");
            RcsFlightSupport.CleanupBurns(fc);
            return;
        }
        if (exec.ResolvedStrategy != RcsAttitudeStrategy.Align)
        {
            t.Skip($"deferred align: resolved to {exec.ResolvedStrategy} on this save.");
            RcsExecutor.Cancel(vehicle, exec, "test cleanup");
            RcsFlightSupport.CleanupBurns(fc);
            return;
        }
        // A slew estimate so long that the lead window is already open at
        // engage makes deferral unobservable; that is a save property, not
        // a bug.
        double leadSec = RcsExecutor.AlignLeadFactor * exec.Estimates.AlignSlewDurationSec
            + RcsExecutor.AlignLeadMarginSec;
        if (leadSec >= DeferredLeadSec - 10.0)
        {
            t.Skip($"deferred align: slew estimate " +
                   $"{exec.Estimates.AlignSlewDurationSec:F0}s leaves no coast to observe.");
            RcsExecutor.Cancel(vehicle, exec, "test cleanup");
            RcsFlightSupport.CleanupBurns(fc);
            return;
        }

        double engageSec = driver.Elapsed.Seconds();
        driver.Step(StepSec, 40);
        bool idleDuringCoast = fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.None;

        bool commanded = false;
        double commandedElapsed = 0.0;
        int steps = (int)(DeferredLeadSec / StepSec);
        for (int i = 0; i < steps; i++)
        {
            driver.Step(StepSec);
            if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
                break;
            if (fc.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.None)
            {
                commanded = true;
                commandedElapsed = driver.Elapsed.Seconds() - engageSec;
                break;
            }
        }

        // Commanded well after engage (the deferral) but still before
        // ignition (the lead window).
        t.Check("deferred align",
            idleDuringCoast && commanded && commandedElapsed > 20.0
                && engageSec + commandedElapsed < burnTimeSec,
            $"idle during coast={idleDuringCoast} commanded at T+{commandedElapsed:F0}s " +
            $"of {DeferredLeadSec:F0}s lead (window opens ~T+{DeferredLeadSec - leadSec:F0}s)");

        if (RcsExecRegistry.TryGet(vehicle.Id, out exec!) && exec.IsActive)
            RcsExecutor.Cancel(vehicle, exec, "test cleanup");
        RcsFuelSummary fuel = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? done)
            ? done.LastFuel : default;
        t.Info($"deferred align fuel: total={fuel.TotalKg * 1000.0:F1}g " +
               $"coast {fuel.CoastKg * 1000.0:F1}g slew {fuel.SlewKg * 1000.0:F1}g");
        RcsFlightSupport.CleanupBurns(fc);
    }

    // The stock RCS toggle (FlightComputer.RCSMode, default key R) gates the
    // attitude-authority scan: with RCS disabled the executor's rate hold and
    // align slew get no authority and a Hold burn would tumble on its
    // residual torque while translation keeps firing. The executor owns the
    // FC for the burn, so it forces RCS on and restores the pilot's setting
    // afterwards. Part A covers disabled-at-activation through a real
    // completion (delta-V delivered, restored to off); Part B covers a
    // mid-burn toggle being re-enabled by the driver and restored on cancel.
    private static void FlyRcsToggleScenario(
        TestContext t, Vehicle vehicle, SimDriver driver, RcsFlightSupport.CompletionWatcher watcher)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        vehicle.RefillConsumables();
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        fc.SetNullRot(VehicleReferenceFrame.EclBody);
        driver.Step(StepSec, 10);
        watcher.Reset();

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int bestAxis = cap.BestAxis();
        if (bestAxis < 0)
        {
            t.Skip("rcs toggle: no usable translation axis on this save.");
            return;
        }

        // Part A: pilot has RCS off before clicking Auto.
        RcsFlightSupport.BurnSetup? setupA = BuildStrongAxisBurn(vehicle, driver, bestAxis);
        if (setupA == null)
        {
            t.Fail("rcs toggle A", "could not set up the burn");
            return;
        }
        Burn burn = setupA.Burn;
        BurnTarget bt = setupA.BurnTarget;

        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions options = exec.GetOrCreateOptions(burn.Time.Seconds(), burn.DeltaVVlf.Length());
        options.Mode = RcsExecutionMode.Rcs;
        options.Attitude = RcsAttitudeStrategy.Hold;

        fc.RCSMode = FlightComputerRCSMode.Disabled;
        vehicle.SetEnum(FlightComputerBurnMode.Auto);
        if (!RcsExecRegistry.TryGet(vehicle.Id, out exec!) || !exec.IsActive)
        {
            t.Fail("rcs toggle A", "SetEnum(Auto) did not engage the executor");
            RcsFlightSupport.CleanupBurns(fc);
            return;
        }

        t.Check("rcs toggle A engage",
            fc.RCSMode == FlightComputerRCSMode.Enabled && exec.ForcedRcsOn,
            $"RCSMode={fc.RCSMode} forcedFlag={exec.ForcedRcsOn}");

        int steps = (int)((BurnLeadSec + 200.0) / StepSec);
        RcsFlightSupport.RunResult result = RcsFlightSupport.RunUntilInactive(vehicle, driver, StepSec, steps);
        if (!result.EnginesQuiet)
            t.Fail("rcs toggle A engines quiet", "a main engine received a throttle command");
        if (!result.Completed)
        {
            t.Fail("rcs toggle A finish",
                $"burn did not complete (to go {bt.DeltaVToGoCci.Length():F3}m/s)");
        }
        else
        {
            // The completion event separates a genuine finish from any cancel
            // path (a tumbling Hold burn without RCS would have stalled).
            bool viaEvent = ReferenceEquals(watcher.LastBurn, burn);
            bool restored = fc.RCSMode == FlightComputerRCSMode.Disabled;
            float accum = bt.DeltaVAccumCci.Length();
            float residual = bt.DeltaVToGoCci.Length();
            bool deliveredOk = viaEvent && accum > (float)(BurnDvMs * 0.9) && residual <= 0.1f;
            t.Check("rcs toggle A finish", deliveredOk && restored,
                $"viaEvent={viaEvent} accum={accum:F3}m/s residual={residual:F4}m/s " +
                $"restoredToOff={restored}");
        }
        RcsFlightSupport.CleanupBurns(fc);

        // Part B: RCS on at activation, toggled off mid-burn (during the
        // pre-ignition coast), must be re-enabled by the driver and restored
        // to off on cancel.
        vehicle.RefillConsumables();
        driver.Step(StepSec, 10);
        watcher.Reset();
        RcsFlightSupport.BurnSetup? setupB = BuildStrongAxisBurn(vehicle, driver, bestAxis);
        if (setupB == null)
        {
            t.Fail("rcs toggle B", "could not set up the burn");
            return;
        }
        Burn burnB = setupB.Burn;

        RcsExecution execB = RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions optionsB = execB.GetOrCreateOptions(burnB.Time.Seconds(), burnB.DeltaVVlf.Length());
        optionsB.Mode = RcsExecutionMode.Rcs;
        optionsB.Attitude = RcsAttitudeStrategy.Hold;

        fc.RCSMode = FlightComputerRCSMode.Enabled;
        vehicle.SetEnum(FlightComputerBurnMode.Auto);
        if (!RcsExecRegistry.TryGet(vehicle.Id, out execB!) || !execB.IsActive)
        {
            t.Fail("rcs toggle B", "SetEnum(Auto) did not engage the executor");
            RcsFlightSupport.CleanupBurns(fc);
            return;
        }
        bool notForcedAtActivation = !execB.ForcedRcsOn && fc.RCSMode == FlightComputerRCSMode.Enabled;

        // Simulate a mid-burn R press and let one driver tick react.
        fc.RCSMode = FlightComputerRCSMode.Disabled;
        driver.Step(StepSec);
        bool reEnabled = false;
        bool forcedFlag = false;
        if (RcsExecRegistry.TryGet(vehicle.Id, out execB!) && execB.IsActive)
        {
            reEnabled = fc.RCSMode == FlightComputerRCSMode.Enabled;
            forcedFlag = execB.ForcedRcsOn;
        }
        t.Check("rcs toggle B guard", notForcedAtActivation && reEnabled && forcedFlag,
            $"notForcedAtActivation={notForcedAtActivation} reEnabledMidBurn={reEnabled} " +
            $"forcedFlag={forcedFlag}");

        if (RcsExecRegistry.TryGet(vehicle.Id, out execB!) && execB.IsActive)
            RcsExecutor.Cancel(vehicle, execB, "test cleanup");
        t.Check("rcs toggle B restore", fc.RCSMode == FlightComputerRCSMode.Disabled,
            $"RCSMode after cancel={fc.RCSMode}");

        RcsFlightSupport.CleanupBurns(fc);
    }

    // Adds a small burn along the strongest translation axis at BurnLeadSec
    // in the future (Hold is fully feasible on a single strong axis for any
    // layout) and returns the setup with the loaded BurnTarget.
    private static RcsFlightSupport.BurnSetup? BuildStrongAxisBurn(
        Vehicle vehicle, SimDriver driver, int bestAxis)
        => RcsFlightSupport.AddBurn(
            vehicle, driver, RcsFlightSupport.AxisDirCci(vehicle, bestAxis), BurnDvMs, BurnLeadSec);

    private static void Fly(
        TestContext t, Vehicle vehicle, SimDriver driver, RcsFlightSupport.CompletionWatcher watcher)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);

        // Let the update task pick the vehicle up and the thruster states
        // (propellant availability, intended forces) materialize.
        driver.Step(StepSec, 40);
        watcher.Reset();

        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        if (!cap.HasAnyTranslation)
        {
            t.Skip("save has no translation-capable RCS thrusters " +
                   $"({vehicle.Parts.Modules.Get<ThrusterController>().Length} thruster module(s)). " +
                   "Save a vehicle with RCS in the Vehicles window to run this test.");
            return;
        }
        int bestAxis = cap.BestAxis();
        t.Info($"capability: best axis {RcsExecutor.AxisName(bestAxis)} " +
               $"F={cap.Get(bestAxis).ForceN:F1}N mdot={cap.Get(bestAxis).MassFlowKgS * 1000.0:F2}g/s, " +
               $"mass={vehicle.TotalMass:F1}kg");
        // Rotation authority per axis plus the implied LP torque-slack
        // price (flow over torque, kg per N m s); the ground truth when
        // slack behavior needs explaining.
        t.Info($"rotation: tau=({cap.RotationTorqueNm.X / 1000f:F0}," +
               $"{cap.RotationTorqueNm.Y / 1000f:F0},{cap.RotationTorqueNm.Z / 1000f:F0})kNm " +
               $"mdot=({cap.RotationMassFlowKgS.X:F1},{cap.RotationMassFlowKgS.Y:F1}," +
               $"{cap.RotationMassFlowKgS.Z:F1})kg/s " +
               $"price~({PriceMg(cap, 0):F3},{PriceMg(cap, 1):F3},{PriceMg(cap, 2):F3})mg/Nms");

        // Rate-hold the spawn attitude, then plan the burn along wherever
        // the strongest translation axis already points (Teleport would be
        // the alternative, but it walks the camera/viewport chain and
        // cannot run headless). The Hold strategy then has a fully
        // feasible single-axis direction regardless of thruster layout.
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        fc.SetNullRot(VehicleReferenceFrame.EclBody);

        double3 dvDirCci = RcsFlightSupport.AxisDirCci(vehicle, bestAxis);
        RcsFlightSupport.BurnSetup? setup =
            RcsFlightSupport.AddBurn(vehicle, driver, dvDirCci, BurnDvMs, BurnLeadSec);
        if (setup == null)
        {
            t.Fail("hold burn", "no flight-plan patch or BurnTarget at the burn time");
            return;
        }
        Burn burn = setup.Burn;
        BurnTarget bt = setup.BurnTarget;

        // Arm the burn for RCS with a fixed strategy (no estimate coin-flip
        // in the assertion path) and trigger through the stock enum sink.
        RcsExecution? exec = RcsFlightSupport.ArmAndEngage(
            vehicle, burn, RcsExecutionMode.Rcs, RcsAttitudeStrategy.Hold, RcsAllocator.Groups);
        if (exec == null)
        {
            t.Fail("hold burn",
                $"SetEnum(Auto) did not engage the RCS executor (controllable={vehicle.IsControllable})");
            return;
        }
        if (fc.BurnMode != FlightComputerBurnMode.Manual)
        {
            t.Fail("hold burn", $"BurnMode is {fc.BurnMode}, expected Manual during RCS execution");
            return;
        }

        // Estimate vs the measured fuel line below: the calibration record for
        // the attitude-fight term (this is a strong-axis Hold, so the fight is
        // the best group's own residual torque, not a multi-axis mix).
        RcsEstimates est0 = exec.Estimates;
        t.Info($"estimate (pre-fire): hold {est0.HoldPropellantKg * 1000.0:F1}g " +
               $"(feasible {est0.HoldFeasible}), align {est0.AlignTotalPropellantKg * 1000.0:F1}g " +
               $"(feasible {est0.AlignFeasible}, slew {est0.AlignSlewPropellantKg * 1000.0:F1}g)");

        double expectedDuration = vehicle.TotalMass * BurnDvMs / cap.Get(bestAxis).ForceN;
        double timeoutSec = BurnLeadSec + expectedDuration * 4.0 + 60.0;
        double m0 = vehicle.TotalMass;

        bool firingSampled = false;
        int steps = (int)(timeoutSec / StepSec);
        RcsFlightSupport.RunResult result = RcsFlightSupport.RunUntilInactive(
            vehicle, driver, StepSec, steps,
            (_, __) =>
            {
                // One live-performance sample while thrusters actually fire:
                // the capability probe evaluates nozzle performance at probe
                // conditions, and this line is the ground truth to hold the
                // model against when the fuel numbers disagree. Comparable at
                // vacuum only: the sampled Performance.TotalThrust is the
                // flow-separation-clamped effective thrust, the model's
                // GetTotalThrust is unclamped; they match at ambient ~0.
                if (!firingSampled && driver.Elapsed.Seconds() > bt.IgnitionTime.Seconds() + 0.2)
                {
                    firingSampled = true;
                    SampleFiringNozzles(t, vehicle);
                }
            });

        if (!result.EnginesQuiet)
            t.Fail("hold engines quiet", "a main engine received a throttle command during the RCS burn");
        if (!result.Completed)
        {
            t.Fail("hold burn",
                $"RCS burn did not complete within {timeoutSec:F0}s sim time " +
                $"(to go {bt.DeltaVToGoCci.Length():F3}m/s of {BurnDvMs:F2}m/s)");
            return;
        }

        float residual = bt.DeltaVToGoCci.Length();
        double tol = RcsFlightSupport.ResidualToleranceMs(in cap, vehicle.TotalMass, ResidualMarginMs);
        // The executor already stopped at or just past the target (its
        // completion check flips on the to-go/target dot product), so a
        // plain magnitude tolerance on the remainder suffices here.
        t.Check("residual", residual <= tol,
            $"{residual:F4}m/s (tol {tol:F4}) accum={bt.DeltaVAccumCci.Length():F4}m/s " +
            $"target={BurnDvMs:F2}m/s");

        double burned = m0 - vehicle.TotalMass;
        t.Check("propellant", burned > 0.0, $"{burned * 1000.0:F1}g consumed");

        t.Check("completion event",
            ReferenceEquals(watcher.LastVehicle, vehicle) && ReferenceEquals(watcher.LastBurn, burn),
            "vehicle and burn delivered");

        // Fuel telemetry: recorded, total consistent with the test's own
        // mass delta, delivered delta-V pointing at the target, and the
        // translation attribution equal to the group cost model applied
        // to the burn (single-axis burn, so the model reduces to the
        // best group's flow per force). Deliberately NOT translation
        // versus total: dev saves can carry partially present reactant
        // mixes, where ResourceManager.MassChange withdraws only the
        // available reactants' share while the nozzle keeps firing at
        // full thrust, so the tank delta is no upper bound for the
        // modeled cost (observed 74.25 kg drained for a 101 kg burn on
        // Test Vehicle 1; the fuel line's negative attitude bucket is
        // how that surfaces).
        RcsFuelSummary fuel = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? done)
            ? done.LastFuel : default;
        RcsAxisGroup bestGroup = cap.Get(bestAxis);
        double modelKg = m0 * BurnDvMs * (bestGroup.MassFlowKgS / bestGroup.ForceN);
        bool fuelOk = fuel.Valid
            && fuel.TotalKg > 0.0
            && fuel.TranslationKg >= modelKg * 0.9                      // attribution at least the clean model
            && fuel.TranslationKg <= modelKg * 1.5                      // asymmetric drift inflates the mix cost
            && Math.Abs(fuel.TotalKg - burned) <= 0.1 * burned + 0.005  // 10% + 5 g mass agreement
            && fuel.DvAngleDeg < 5.0;                                   // pointing gate
        t.Check("fuel telemetry", fuelOk,
            $"total={fuel.TotalKg * 1000.0:F1}g (translation {fuel.TranslationKg * 1000.0:F1}g, " +
            $"slew {fuel.SlewKg * 1000.0:F1}g, coast {fuel.CoastKg * 1000.0:F1}g, " +
            $"attitude {fuel.AttitudeKg * 1000.0:F1}g) model={modelKg * 1000.0:F1}g " +
            $"ve_eff={fuel.EffectiveVeMs:F0}m/s dv angle={fuel.DvAngleDeg:F2}deg");

        RcsOrbitCheck.Assert(t, "orbit", vehicle.Orbit, setup.Predicted,
            setup.InitialSma, setup.InitialEcc);
    }

    private static double PriceMg(in RcsCapabilitySnapshot cap, int axis)
    {
        float flow = axis == 0 ? cap.RotationMassFlowKgS.X
            : axis == 1 ? cap.RotationMassFlowKgS.Y : cap.RotationMassFlowKgS.Z;
        float torque = axis == 0 ? cap.RotationTorqueNm.X
            : axis == 1 ? cap.RotationTorqueNm.Y : cap.RotationTorqueNm.Z;
        return torque > 1f ? flow / torque * 1e6 : 0.0;
    }

    private static void SampleFiringNozzles(TestContext t, Vehicle vehicle)
    {
        if (!ModuleStateful<RocketNozzle, RocketNozzleState, EmptyStruct, RocketNozzleFxState>
                .TryGetFrom(vehicle.Parts.States, out var stateList))
            return;
        float thrustN = 0f;
        float thrustXN = 0f;
        float flowKgS = 0f;
        int firing = 0;
        var enumerator = new ModuleStateful<RocketNozzle, RocketNozzleState, EmptyStruct, RocketNozzleFxState>
            .StateList.ModuleAndStateEnumerator(stateList);
        while (enumerator.MoveNext())
        {
            ref readonly RocketNozzleState state = ref enumerator.Current.State;
            // ThrustFraction is the per-frame fraction of max thrust produced;
            // 0 means the nozzle is not firing this frame.
            if (state.ThrustFraction <= 0f || state.Performance.MassFlowRate <= 0f)
                continue;
            firing++;
            float f = state.Performance.TotalThrust;
            thrustN += f;
            thrustXN += f * state.ThrustDirectionVehicleAsmb.X;
            flowKgS += state.Performance.MassFlowRate;
        }
        t.Info($"firing sample: {firing} nozzle(s), thrust={thrustN:F0}N " +
               $"(X {thrustXN:F0}N), flow={flowKgS * 1000.0:F0}g/s, " +
               $"ve={(flowKgS > 0f ? thrustN / flowKgS : 0f):F0}m/s");
    }
}
