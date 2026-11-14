using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Installs the production guidance hooks on a test-scoped Harmony id and steps a craft with its engine
// lit. Its gimbals are narrowed until stock FlightComputer.UpdateActiveControlSystems gives pitch and
// yaw to the RCS, as it does on a small lander whose RCS sits far from the centre of mass. While
// guidance flies a landing burn on it with STEER BURN WITH GIMBAL on, RCS must be switched off and
// the gimbal must take pitch and yaw; the release switches RCS on again and gives the stock choice
// back. A player who switches RCS on during the burn keeps it, the setting off leaves RCS alone, and
// so does a craft whose gimbals have no range. A save record made while RCS is held off carries RCS
// on. Gimbals that leave pitch or yaw without authority keep RCS, and a burn that loses an axis gets
// RCS back on that step. The player's switch is also seen while touchdown guidance holds the craft.
// The landing step and the touchdown step are replaced, so the burn's attitude and engine commands
// come from ApplyAutopilot and the stock flight computer alone.
public sealed class GuidanceLandingGimbalTest : AfcTest
{
    public override string Name => "afc-guidance-landing-gimbal";

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    // Where the narrowed gimbal leaves the TVC authority against the RCS authority, under the tenth at
    // which stock gives an axis to the TVC. Not much under it, because the stock trim of a craft whose
    // thrust misses its centre of mass takes part of the range, and a range narrowed too far leaves an
    // axis no authority in one direction, which stock reports as none at all.
    private const float TargetAuthorityRatio = 0.06f;

    private static Vehicle? _held;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no test vehicle save present.");
            return;
        }
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        Vehicle? previousFocus = Program.ControlledVehicle;
        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        bool previousOffRails = PhysicsBubble._forceOffRails;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.landinggimbal");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            GuidanceFeature.ApplySaveRcsPatch(harmony);
            harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), "StepLanding"),
                prefix: new HarmonyMethod(typeof(GuidanceLandingGimbalTest), nameof(ReplaceTheLandingStep)));
            harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), "AutoSequence"),
                prefix: new HarmonyMethod(typeof(GuidanceLandingGimbalTest), nameof(ReplaceTheLandingStep)));
            harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), nameof(GuidanceWindow.Step6Dof)),
                prefix: new HarmonyMethod(typeof(GuidanceLandingGimbalTest), nameof(ReplaceTheLandingStep)));
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGuidanceLandingGimbal",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            _held = vehicle;
            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 20);

            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            driver.Step(0.05, 5);
            TestSupport.SetManualControlInputs(vehicle, 1f, engineOn: true);
            vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            vehicle.FlightComputer.RCSMode = FlightComputerRCSMode.Enabled;
            driver.Step(0.05, 5);

            if (!NarrowTheGimbals(t, vehicle, driver))
                return;
            try
            {
                TheGimbalSteersTheLandingBurn(t, vehicle, driver);
                ThePlayerKeepsRcsSwitchedOnAgain(t, vehicle, driver);
                ThePlayerSwitchIsSeenUnderTouchdownGuidance(t, vehicle, driver);
                TheSettingOffLeavesRcs(t, vehicle, driver);
                AGimbalWithoutAnAxisKeepsRcs(t, vehicle, driver);
                ACraftWithNoGimbalRangeKeepsRcs(t, vehicle, driver);
            }
            finally
            {
                VehicleControlOwnership.ReleaseAll(vehicle);
                VehicleAutopilotState.Remove(vehicle);
            }
        }
        finally
        {
            _held = null;
            PhysicsBubble._forceOffRails = previousOffRails;
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            Program.ControlledVehicle = previousFocus;
            AmbientState() = previousAmbient;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // Narrows every gimbal by one factor until the stock rule gives pitch and yaw to the RCS.
    private static bool NarrowTheGimbals(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        float tvc = Math.Min(fc.TvcTorqueAuthority.Y, fc.TvcTorqueAuthority.Z);
        float rcs = Math.Min(fc.RcsTorqueAuthority.Y, fc.RcsTorqueAuthority.Z);
        if (!(tvc > 0f) || !(rcs > 0f))
        {
            t.Skip($"the craft needs a lit gimballed engine and RCS on pitch and yaw (TVC {fc.TvcTorqueAuthority}, RCS {fc.RcsTorqueAuthority}).");
            return false;
        }

        ScaleTheGimbals(vehicle, Math.Min(TargetAuthorityRatio * rcs / tvc, 1f));
        driver.Step(0.05, 3);

        bool rcsSteers = RcsSteersPitchAndYaw(fc) && fc.TvcTorqueAuthority.Y > 0f;
        t.Check("with the gimbal narrowed and no guidance, stock gives pitch and yaw to the RCS", rcsSteers, Axes(fc));
        return rcsSteers;
    }

    private static void ScaleTheGimbals(Vehicle vehicle, float scale)
    {
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
        {
            Gimbal g = gc.Gimbal;
            g.AxisY = new GimbalAxis { MaxAngle = g.AxisY.MaxAngle * scale };
            g.AxisZ = new GimbalAxis { MaxAngle = g.AxisZ.MaxAngle * scale };
        }
    }

    private static void TheGimbalSteersTheLandingBurn(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);

        t.Check("the landing burn flies the attitude through the flight computer",
            state.ControlAcquired && fc.AttitudeMode == FlightComputerAttitudeMode.Auto
            && fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom,
            $"acquired {state.ControlAcquired}, mode {fc.AttitudeMode}, target {fc.AttitudeTrackTarget}");
        t.Check("RCS is switched off while the gimbal steers the landing burn",
            fc.RCSMode == FlightComputerRCSMode.Disabled, $"RCS {fc.RCSMode}, lit gimbal thrust {KsaGimbalControl.LitGimbalThrust(vehicle):F0} N");
        // An axis the gimbal has no authority on has no control system at all with RCS off, so only the axes with authority are asked for.
        bool pitch = fc.TvcTorqueAuthority.Y > 0f, yaw = fc.TvcTorqueAuthority.Z > 0f;
        t.Check("the gimbal takes pitch and yaw while the landing burn flies",
            (pitch || yaw)
            && (!pitch || fc.ActiveControlSystem.Y == AttitudeControlSystem.Tvc)
            && (!yaw || fc.ActiveControlSystem.Z == AttitudeControlSystem.Tvc),
            Axes(fc));
        t.Check("no axis is left with the RCS", fc.ActiveControlSystem.X != AttitudeControlSystem.Rcs
            && fc.ActiveControlSystem.Y != AttitudeControlSystem.Rcs && fc.ActiveControlSystem.Z != AttitudeControlSystem.Rcs, Axes(fc));

        // FlightComputer.SerializeSave writes the mode into the save record of the craft, and a load drops the hold.
        FlightComputerRCSMode saved = vehicle.SerializeSave().FlightComputerData.RCSMode;
        t.Check("a save made while guidance holds RCS off carries RCS on, and the craft keeps RCS off",
            saved == FlightComputerRCSMode.Enabled && fc.RCSMode == FlightComputerRCSMode.Disabled,
            $"saved {saved}, craft {fc.RCSMode}");

        EndTheLanding(state, driver);
        t.Check("the release switches RCS on again and gives pitch and yaw back to the stock rule",
            !state.ControlAcquired && fc.RCSMode == FlightComputerRCSMode.Enabled && RcsSteersPitchAndYaw(fc),
            $"acquired {state.ControlAcquired}, RCS {fc.RCSMode}, {Axes(fc)}");
    }

    private static void ThePlayerKeepsRcsSwitchedOnAgain(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        bool heldOff = fc.RCSMode == FlightComputerRCSMode.Disabled;

        fc.RCSMode = FlightComputerRCSMode.Enabled;
        driver.Step(0.05, 6);
        t.Check("RCS switched on during the burn stays on", heldOff && fc.RCSMode == FlightComputerRCSMode.Enabled
            && state.ControlAcquired && RcsSteersPitchAndYaw(fc),
            $"held off before {heldOff}, RCS {fc.RCSMode}, acquired {state.ControlAcquired}, {Axes(fc)}");

        EndTheLanding(state, driver);
        t.Check("the release leaves the RCS the player switched on", fc.RCSMode == FlightComputerRCSMode.Enabled, $"RCS {fc.RCSMode}");

        // RCS switched off by the player before the burn is not switched on by the release.
        fc.RCSMode = FlightComputerRCSMode.Disabled;
        state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        FlightComputerRCSMode saved = vehicle.SerializeSave().FlightComputerData.RCSMode;
        EndTheLanding(state, driver);
        t.Check("RCS that was off before the burn stays off after it, and a save made in the burn carries it off",
            fc.RCSMode == FlightComputerRCSMode.Disabled && saved == FlightComputerRCSMode.Disabled, $"RCS {fc.RCSMode}, saved {saved}");
        fc.RCSMode = FlightComputerRCSMode.Enabled;
        driver.Step(0.05, 3);
    }

    // Touchdown guidance keeps the RCS hold of the burn it took over. A player who switches RCS on and off again while it flies has RCS off at the release.
    private static void ThePlayerSwitchIsSeenUnderTouchdownGuidance(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        bool heldOff = fc.RCSMode == FlightComputerRCSMode.Disabled;

        state.Active = true;
        driver.Step(0.05, 3);
        bool keptOff = fc.RCSMode == FlightComputerRCSMode.Disabled;
        fc.RCSMode = FlightComputerRCSMode.Enabled;
        driver.Step(0.05, 3);
        fc.RCSMode = FlightComputerRCSMode.Disabled;
        driver.Step(0.05, 3);

        state.Active = false;
        EndTheLanding(state, driver);
        t.Check("RCS that the player switched on and off again under touchdown guidance stays off at the release",
            heldOff && keptOff && !state.ControlAcquired && fc.RCSMode == FlightComputerRCSMode.Disabled,
            $"held off in the burn {heldOff}, kept off by touchdown guidance {keptOff}, acquired {state.ControlAcquired}, RCS {fc.RCSMode}");
        fc.RCSMode = FlightComputerRCSMode.Enabled;
        driver.Step(0.05, 3);
    }

    // Gimbals that turn about one axis only leave pitch or yaw with no gimbal authority (FlightComputer.UpdateTvcParams), and with RCS off that axis would have no control system at all.
    private static void AGimbalWithoutAnAxisKeepsRcs(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        float[] rangeZ = TakeTheZAxis(vehicle);
        driver.Step(0.05, 3);
        bool oneAxis = !GuidanceWindow.GimbalHoldsPitchAndYaw(fc) && Math.Max(fc.TvcTorqueAuthority.Y, fc.TvcTorqueAuthority.Z) > 0f;
        if (!oneAxis)
        {
            t.Info($"the one-axis cases are skipped, because the gimbals of this craft keep both axes without their Z range ({Axes(fc)}).");
            GiveTheZAxis(vehicle, rangeZ);
            driver.Step(0.05, 3);
            return;
        }

        VehicleAutopilotState state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        t.Check("gimbals without authority on one of pitch and yaw keep RCS",
            state.ControlAcquired && fc.RCSMode == FlightComputerRCSMode.Enabled
            && fc.ActiveControlSystem.Y != AttitudeControlSystem.None && fc.ActiveControlSystem.Z != AttitudeControlSystem.None,
            $"acquired {state.ControlAcquired}, RCS {fc.RCSMode}, {Axes(fc)}");
        EndTheLanding(state, driver);

        GiveTheZAxis(vehicle, rangeZ);
        driver.Step(0.05, 3);
        state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        bool heldOff = fc.RCSMode == FlightComputerRCSMode.Disabled;
        TakeTheZAxis(vehicle);
        // One step for the flight computer to see the gimbals, one for guidance to see the authority.
        driver.Step(0.05, 2);
        t.Check("a burn that loses an axis of the gimbal gets RCS back at once",
            heldOff && fc.RCSMode == FlightComputerRCSMode.Enabled && state.ControlAcquired,
            $"held off before {heldOff}, RCS {fc.RCSMode}, acquired {state.ControlAcquired}, {Axes(fc)}");
        driver.Step(0.05, 3);
        t.Check("and no axis is without a control system", fc.ActiveControlSystem.Y != AttitudeControlSystem.None
            && fc.ActiveControlSystem.Z != AttitudeControlSystem.None, Axes(fc));

        GiveTheZAxis(vehicle, rangeZ);
        driver.Step(0.05, 6);
        t.Check("RCS then stays on for the rest of the burn", fc.RCSMode == FlightComputerRCSMode.Enabled
            && GuidanceWindow.GimbalHoldsPitchAndYaw(fc), $"RCS {fc.RCSMode}, {Axes(fc)}");
        EndTheLanding(state, driver);

        state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        t.Check("the next landing burn takes RCS off again", fc.RCSMode == FlightComputerRCSMode.Disabled, $"RCS {fc.RCSMode}, {Axes(fc)}");
        EndTheLanding(state, driver);
    }

    private static float[] TakeTheZAxis(Vehicle vehicle)
    {
        var ranges = new List<float>();
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
        {
            ranges.Add(gc.Gimbal.AxisZ.MaxAngle);
            gc.Gimbal.AxisZ = new GimbalAxis { MaxAngle = 0f };
        }
        return ranges.ToArray();
    }

    private static void GiveTheZAxis(Vehicle vehicle, float[] ranges)
    {
        int i = 0;
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
            gc.Gimbal.AxisZ = new GimbalAxis { MaxAngle = ranges[i++] };
    }

    private static void TheSettingOffLeavesRcs(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState state = StartALandingBurn(vehicle, gimbalSteers: false);
        driver.Step(0.05, 5);
        t.Check("with the setting off, RCS stays on and steers as stock chooses",
            state.ControlAcquired && fc.RCSMode == FlightComputerRCSMode.Enabled && RcsSteersPitchAndYaw(fc),
            $"acquired {state.ControlAcquired}, RCS {fc.RCSMode}, {Axes(fc)}");
        EndTheLanding(state, driver);
    }

    private static void ACraftWithNoGimbalRangeKeepsRcs(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        ScaleTheGimbals(vehicle, 0f);
        driver.Step(0.05, 3);
        VehicleAutopilotState state = StartALandingBurn(vehicle, gimbalSteers: true);
        driver.Step(0.05, 5);
        t.Check("a craft whose gimbals have no range keeps RCS",
            state.ControlAcquired && fc.RCSMode == FlightComputerRCSMode.Enabled
            && KsaGimbalControl.LitGimbalThrust(vehicle) == 0.0,
            $"acquired {state.ControlAcquired}, RCS {fc.RCSMode}, lit gimbal thrust {KsaGimbalControl.LitGimbalThrust(vehicle):F0} N");
        EndTheLanding(state, driver);
    }

    private static VehicleAutopilotState StartALandingBurn(Vehicle vehicle, bool gimbalSteers)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.LandingGimbalSteers = gimbalSteers;
        state.Engage = true;
        state.AutoStage = true;
        state.LandingPhase = GuidanceWindow.LandingPhase.Burn;
        state.CommandDir = KsaFrameBridge.ThrustAxisCci(vehicle);
        state.HasCommand = true;
        AccessTools.PropertySetter(typeof(UpfgGuidance), nameof(UpfgGuidance.Throttle))
            .Invoke(state.Upfg, new object[] { 1.0 });
        return state;
    }

    // The engine stays lit for the next case, as the player would light it again.
    private static void EndTheLanding(VehicleAutopilotState state, SimDriver driver)
    {
        state.Engage = false;
        state.LandingPhase = GuidanceWindow.LandingPhase.Idle;
        driver.Step(0.05, 3);
        if (_held != null)
            TestSupport.SetManualControlInputs(_held, 1f, engineOn: true);
        driver.Step(0.05, 3);
    }

    private static bool RcsSteersPitchAndYaw(FlightComputer fc)
        => fc.ActiveControlSystem.Y == AttitudeControlSystem.Rcs && fc.ActiveControlSystem.Z == AttitudeControlSystem.Rcs;

    private static string Axes(FlightComputer fc)
        => $"ACS {fc.ActiveControlSystem.X}/{fc.ActiveControlSystem.Y}/{fc.ActiveControlSystem.Z}, TVC {fc.TvcTorqueAuthority}, RCS {fc.RcsTorqueAuthority}";

    // A Harmony prefix that returns false skips the original for the fixture, so the burn state the
    // case sets stays as it is and every other craft's step runs as usual.
    private static bool ReplaceTheLandingStep(Vehicle vehicle) => !ReferenceEquals(vehicle, _held);
}
