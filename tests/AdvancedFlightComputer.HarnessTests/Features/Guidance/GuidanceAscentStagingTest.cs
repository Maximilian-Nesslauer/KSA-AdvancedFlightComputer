using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Flies the real UPFG ascent, with the production hooks installed, through a spent-booster drop
// that the AutoStage feature performs while the core keeps burning. The craft starts on a circular
// orbit rather than on a pad, so the phase machine is in the closed loop from the first steps and
// the burn under test is the stretch across the drop. The case needs a save whose launch stage
// mixes boosters with a core, the same one the spent-drop test uses.
//
// What it proves: guidance lights the launch stage through its cold-ignition request, keeps the
// claim, the attitude command and the running ascent through the separation, and the commanded
// direction does not swing up when the stage list changes shape under UPFG.
public sealed class GuidanceAscentStagingTest : AfcTest
{
    public override string Name => "afc-guidance-ascent-staging";

    private static readonly string[] DefaultSaves = { "Test Vehicle 2" };
    private const double StepDt = 0.25;

    private const double MaxBurnSeconds = 600.0;
    private const double AfterDropSeconds = 20.0;
    private const double ReconvergeSeconds = 10.0;
    private const double MaxPitchRiseDeg = 15.0;
    private const double MaxPacedBurnError = 0.15;
    private const double SampleIntervalS = 2.0;
    private const double ProbeIntervalS = 10.0;
    private const int CurvePoints = 65;
    private const double TargetApoapsisKm = 1_000_000.0;
    private const double TargetPeriapsisRiseKm = 100.0;
    private const double GLimitG = 2.0;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo StartGuidance =
        AccessTools.Method(typeof(GuidanceWindow), "StartGuidance");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"'{DefaultSaves[0]}' is not in the game's Vehicles folder.");
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
        bool previousDropSpent = StagingConfig.DropSpentStages;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        using AutoStageTestPatches.Scope staging = AutoStageTestPatches.Apply();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.ascent-staging");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            Vehicle vehicle;
            try
            {
                vehicle = AutoStageFlightSupport.SpawnFromSave(t, saves[0], "HarnessGuidanceAscentStaging", out _);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }

            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            StagingConfig.DropSpentStages = true;
            driver.Step(0.05, 40);

            Fly(t, vehicle, home, driver);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            StagingConfig.DropSpentStages = previousDropSpent;
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private static void Fly(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver)
    {
        // The target keeps the plane and raises the apoapsis to near escape, and the g-limit holds the
        // acceleration down so the burn outlasts the boosters. The plane comes from the state vectors, so no unit convention of
        // the orbit elements is assumed.
        Orbit orbit = vehicle.Orbit;
        double3 r = orbit.StateVectors.PositionCci;
        double3 v = orbit.StateVectors.VelocityCci;
        double3 h = double3.Cross(r, v);
        double3 hHat = double3.Normalize(h);
        double incDeg = Math.Acos(Math.Clamp(hHat.Z, -1.0, 1.0)) * 180.0 / Math.PI;
        double lanDeg = Math.Atan2(hHat.X, -hHat.Y) * 180.0 / Math.PI;
        if (lanDeg < 0.0)
            lanDeg += 360.0;
        double altKm = (r.Length() - home.MeanRadius) / 1000.0;

        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.PeKm = altKm + TargetPeriapsisRiseKm;
        state.ApKm = TargetApoapsisKm;
        state.IncDeg = incDeg;
        state.LanDeg = lanDeg;
        state.LanSeeded = true;
        state.Engage = true;
        state.AutoStage = true;
        state.GLimitEnabled = true;
        state.GLimitG = GLimitG;
        state.ReserveArmed = false;
        state.TargetId = "";
        AmbientState() = state;
        DescribeSequences(t, vehicle, "at spawn");

        // The launch stage is lit the way the spent-drop test lights it, through the gauge arm and
        // the stock staging key, because the save spawns with an engine already flagged active and
        // guidance's own cold-ignition cue would see thrust. The activation lands on the next step.
        if (!AutoStageFlightSupport.Arm(t, vehicle))
            return;
        vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
        driver.Step(1.0);
        double time = 1.0;
        DescribeSequences(t, vehicle, "after ignition");

        IReadOnlySet<Part>? jettison = JettisonAnalysis.GetPendingJettison(vehicle, StagingDetector.StateOf(vehicle));
        StagingHelpers.EngineSurvey survey = StagingHelpers.SurveyActiveEngines(vehicle, jettison);
        if (jettison == null || survey.FueledInside == 0 || survey.FueledOutside == 0)
        {
            t.Skip($"'{DefaultSaves[0]}' is not a mixed launch stage (jettison={jettison != null}, inside={survey.FueledInside}, outside={survey.FueledOutside}).");
            return;
        }
        t.Info($"launch stage: {survey.FueledInside} engine(s) on the jettisoned side, {survey.FueledOutside} staying");

        StartGuidance.Invoke(null, new object[] { vehicle, orbit, home });
        t.Check("the ascent starts running", state.Running && state.Phase == GuidanceWindow.AscentPhase.Vertical);
        t.Info($"target {state.PeKm:F0} x {state.ApKm:F0} km, inc {state.IncDeg:F2} deg (from {incDeg:F2}), LAN {state.LanDeg:F2} deg");
        int vehiclesBefore = TestSupport.CountVehicles(t.System);
        driver.Step(StepDt, 12);
        time += StepDt * 12;
        DescribeState(t, vehicle, state, time);
        t.Check("guidance takes the lit craft and commands it",
            state.ControlAcquired && state.HasCommand
            && VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.Guidance,
            $"acquired={state.ControlAcquired} command={state.HasCommand} holder={VehicleControlOwnership.HolderOf(vehicle)} error='{state.GuidanceError}'");
        t.Info($"stage model after ignition: {GuidanceLog.DescribeStages(state.StageModel)}");
        double stackBurnAtIgnition = StackBurnSeconds(state.StageModel);
        double stackBurnPeak = stackBurnAtIgnition;

        // Across the booster burn, sampling the solution, until the drop or the end of the ascent.
        double3 up = double3.Normalize(vehicle.Orbit.StateVectors.PositionCci);
        double pitchBefore = PitchDeg(up, state.CommandDir);
        double nextSample = 0.0;
        double ignitionTime = time;
        double pacedBurnLeft = DescribeSolids(t, vehicle, jettison, time);

        SolidMotor? probeSolid = FindShedSolid(vehicle, jettison);
        DescribeCurve(t, probeSolid);
        double probeGrain = 0.0, probeTime = time;
        TryUsableGrainMass(probeSolid, out probeGrain);
        double nextProbe = time + ProbeIntervalS;

        while (time < MaxBurnSeconds && TestSupport.CountVehicles(t.System) <= vehiclesBefore)
        {
            if (!state.Running)
            {
                t.Skip($"the ascent ended at t={time:F1} s before the boosters were spent ({state.Status}), so the drop was not exercised.");
                return;
            }
            if (time >= nextSample)
            {
                nextSample += SampleIntervalS;
                Sample(t, vehicle, state, time);
            }
            if (time >= nextProbe)
            {
                nextProbe += ProbeIntervalS;
                ProbeSolid(t, vehicle, probeSolid, time, ref probeGrain, ref probeTime);
            }
            up = double3.Normalize(vehicle.Orbit.StateVectors.PositionCci);
            pitchBefore = PitchDeg(up, state.CommandDir);
            stackBurnPeak = Math.Max(stackBurnPeak, StackBurnSeconds(state.StageModel));
            driver.Step(StepDt);
            time += StepDt;
        }
        double dropTime = time;
        if (!t.Check("AutoStage drops the spent boosters while guidance flies",
                TestSupport.CountVehicles(t.System) > vehiclesBefore,
                $"no separation within {MaxBurnSeconds:F0} s"))
            return;
        double boosterBurn = dropTime - ignitionTime;
        t.Info($"boosters dropped at t={boosterBurn:F1} s after ignition, command pitch before the drop {pitchBefore:F1} deg, " +
               $"tgo {state.Upfg.Tgo:F1} s, vgo {state.Upfg.VgoMag:F0} m/s, converged {state.Upfg.Converged}");
        // The drain model paces a burning solid by the burn it has left. A model that gave the boosters their full burn again on every refresh would move the core's leftover propellant into a booster-only trickle phase and more than double the modelled burn of the whole stack before the drop. A progressive grain still makes the estimate drift, so the bound is loose.
        t.Check("the modelled burn of the stack does not balloon while the boosters burn",
            stackBurnAtIgnition > 0.0 && stackBurnPeak <= 1.5 * stackBurnAtIgnition,
            $"{stackBurnAtIgnition:F0} s at ignition, peak {stackBurnPeak:F0} s");
        // The flown burn also holds the burnout detection and the decoupler step, hence the tolerance.
        t.Check($"the paced burn left of the boosters holds to the flown burn within {MaxPacedBurnError:P0}",
            pacedBurnLeft > 0.0 && boosterBurn > 0.0
            && Math.Abs(pacedBurnLeft - boosterBurn) <= MaxPacedBurnError * boosterBurn,
            $"{pacedBurnLeft:F1} s paced at t={ignitionTime:F1} s, {boosterBurn:F1} s flown");

        // After the drop: the craft stays guidance's, and the command stays continuous.
        double maxPitch = double.NegativeInfinity, minPitch = double.PositiveInfinity;
        double reconvergedAt = double.NaN;
        bool claimHeld = true, commandHeld = true, tracking = true;
        string takeover = "";
        double end = time + AfterDropSeconds;
        nextSample = time;
        while (time < end)
        {
            driver.Step(StepDt);
            time += StepDt;
            up = double3.Normalize(vehicle.Orbit.StateVectors.PositionCci);
            double pitch = PitchDeg(up, state.CommandDir);
            if (!double.IsNaN(pitch))
            {
                maxPitch = Math.Max(maxPitch, pitch);
                minPitch = Math.Min(minPitch, pitch);
            }
            if (time >= nextSample)
            {
                nextSample += SampleIntervalS;
                Sample(t, vehicle, state, time);
            }
            claimHeld &= VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.Guidance && state.ControlAcquired;
            commandHeld &= state.Running && state.HasCommand;
            tracking &= vehicle.FlightComputer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom;
            if (takeover.Length == 0 && (state.TakeoverStop || state.Status.StartsWith("Guidance stopped", StringComparison.Ordinal)))
                takeover = state.Status.Length > 0 ? state.Status : "takeover flagged";
            if (double.IsNaN(reconvergedAt) && state.Upfg.Converged && time - dropTime >= 1.0)
                reconvergedAt = time - dropTime;
        }

        t.Info($"after the drop: command pitch {minPitch:F1} to {maxPitch:F1} deg over {AfterDropSeconds:F0} s, " +
               $"reconverged {(double.IsNaN(reconvergedAt) ? "never" : reconvergedAt.ToString("F1") + " s after the drop")}, " +
               $"stage model: {GuidanceLog.DescribeStages(state.StageModel)}");
        t.Check("guidance keeps the claim through the drop", claimHeld);
        t.Check("no takeover is detected at the drop", takeover.Length == 0, takeover);
        t.Check("the ascent keeps running through the drop", commandHeld, $"running={state.Running} status='{state.Status}'");
        t.Check("the flight computer keeps tracking guidance's custom target", tracking);
        t.Check($"the commanded pitch rises by at most {MaxPitchRiseDeg:F0} deg after the drop",
            maxPitch - pitchBefore <= MaxPitchRiseDeg, $"{pitchBefore:F1} -> {maxPitch:F1} deg");
        t.Check($"UPFG converges again within {ReconvergeSeconds:F0} s of the drop",
            !double.IsNaN(reconvergedAt) && reconvergedAt <= ReconvergeSeconds,
            double.IsNaN(reconvergedAt) ? "not converged" : $"{reconvergedAt:F1} s");
    }

    private static string _modelSignature = "";
    private static string _planSignature = "";

    private static void Sample(TestContext t, Vehicle vehicle, VehicleAutopilotState state, double time)
    {
        double3 up = double3.Normalize(vehicle.Orbit.StateVectors.PositionCci);
        int stages = state.UpfgVehicle?.Stages.Count ?? 0;
        t.Info($"t={time,6:F1}s phase={state.Phase} tgo={state.Upfg.Tgo,7:F1}s vgo={state.Upfg.VgoMag,6:F0}m/s converged={state.Upfg.Converged} " +
               $"steer={PitchDeg(up, state.Upfg.Steering),6:F1}deg command={PitchDeg(up, state.CommandDir),6:F1}deg " +
               $"throttle={state.Upfg.Throttle:F2} mass={vehicle.TotalMass / 1000.0,7:F1}t stages={stages} status='{state.Status}'");

        // The two stage lists whenever either changes shape: the live model and the copy the solve planned against.
        string model = GuidanceLog.StageSignature(state.StageModel);
        if (model != _modelSignature)
        {
            _modelSignature = model;
            t.Info($"   stage model: {GuidanceLog.DescribeStages(state.StageModel)}");
        }
        string plan = GuidanceLog.StageSignature(state.UpfgVehicle);
        if (plan != _planSignature)
        {
            _planSignature = plan;
            t.Info($"   solve model: {GuidanceLog.DescribeStages(state.UpfgVehicle)}");
        }
    }

    // Logs each burning solid of the shed side, because the harness does not capture the game log.
    // Returns the longest paced burn left.
    private static double DescribeSolids(TestContext t, Vehicle vehicle, IReadOnlySet<Part> jettison, double time)
    {
        double longest = 0.0;
        PartTree tree = vehicle.Parts;
        if (tree?.Moles == null || tree.RocketNozzles == null)
            return longest;

        foreach (EngineController engine in tree.Modules.Get<EngineController>())
        {
            if (!engine.IsActive || !jettison.Contains(engine.Parent.FullPart))
                continue;
            foreach (RocketCore core in engine.Cores)
            {
                if (core is not SolidMotor solid || solid.Rocket == null || !solid.Stack.IsValid)
                    continue;

                double liveFlow = 0.0;
                foreach (var nozzle in tree.RocketNozzles.GetModulesAndStates(solid.Rocket.Nozzles.AsSpan()))
                    liveFlow += nozzle.State.Performance.MassFlowRate;
                if (!(liveFlow > 0.0))
                    continue;

                TryUsableGrainMass(solid, out double usable);
                bool paced = TryBurnSecondsLeft(solid, usable, out double left);
                t.Info($"solid '{solid.TemplateId}' at t={time:F1}s: grain {usable / 1000.0:F1} t of " +
                       $"{solid.InitialBurnableGrainMass / 1000.0:F1} t burnable, live flow {liveFlow:F0} kg/s, " +
                       $"profile reports {solid.VacuumThrustProfile.TotalBurnSeconds:F1} s, grain over live flow {usable / liveFlow:F1} s, " +
                       $"paced burn left {(paced ? left.ToString("F1") + " s" : "not read")}");
                if (paced)
                    longest = Math.Max(longest, left);
            }
        }
        return longest;
    }

    // The grain that can still burn, without the residue that never does.
    private static bool TryUsableGrainMass(SolidMotor? solid, out double usable)
    {
        usable = 0.0;
        PartTree? tree = solid?.Rocket?.Parent?.FullPart?.Tree;
        if (solid == null || tree?.Moles == null || !solid.Stack.IsValid)
            return false;
        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        foreach (SolidGrainSegment segment in solid.Stack.Segments)
        {
            if (segment.Grain != null)
                usable += Math.Max(0.0, moles[segment.Grain.StatesIdx].Mass - segment.UnburnableGrainMass);
        }
        return usable > 0.0;
    }

    // The burn left that the game's drain model paces a solid by. The model picks the profile from Sequence.Environment, and the ascent save leaves its sequences at the default Vacuum.
    private static bool TryBurnSecondsLeft(SolidMotor solid, double usable, out double seconds)
    {
        seconds = 0.0;
        if (!solid.VacuumThrustProfile.TryComputeRemainingBurn((float)usable, out float left, out _))
            return false;
        seconds = left;
        return true;
    }

    private static SolidMotor? FindShedSolid(Vehicle vehicle, IReadOnlySet<Part>? jettison)
    {
        PartTree? tree = vehicle?.Parts;
        if (tree == null || jettison == null)
            return null;
        foreach (EngineController engine in tree.Modules.Get<EngineController>())
        {
            if (!engine.IsActive || !jettison.Contains(engine.Parent.FullPart))
                continue;
            foreach (RocketCore core in engine.Cores)
                if (core is SolidMotor solid && solid.Rocket?.Nozzles != null && solid.Stack.IsValid)
                    return solid;
        }
        return null;
    }

    // Logs the curve against the grain and each segment's GrainVolume against its own geometry. A
    // craft spawned outside the editor keeps GrainVolume sized for the default geometry.
    private static void DescribeCurve(TestContext t, SolidMotor? solid)
    {
        if (solid == null)
            return;
        Span<float> thrust = stackalloc float[CurvePoints];
        Span<float> isp = stackalloc float[CurvePoints];
        Span<float> pressure = stackalloc float[CurvePoints];
        var samples = new SolidMotor.ThrustCurveSamples
        {
            ThrustNewtons = thrust,
            IspSeconds = isp,
            ChamberPressurePascals = pressure,
        };
        if (!solid.TrySampleThrustCurve(samples, out SolidMotor.ThrustCurvePreview preview))
        {
            t.Info($"curve '{solid.TemplateId}': not sampled");
            return;
        }

        double running = 0.0;
        double previous = Flow(thrust[0], isp[0]);
        for (int i = 1; i < CurvePoints; i++)
        {
            double next = Flow(thrust[i], isp[i]);
            running += 0.5 * (previous + next);
            previous = next;
        }
        double curveMass = running * preview.BurnSeconds / (CurvePoints - 1);

        double residue = 0.0;
        foreach (SolidGrainSegment segment in solid.Stack.Segments)
            residue += segment.UnburnableGrainMass;

        t.Info($"curve '{solid.TemplateId}': reports {preview.BurnSeconds:F1} s, first flow {Flow(thrust[0], isp[0]):F1} kg/s, " +
               $"last flow {Flow(thrust[CurvePoints - 1], isp[CurvePoints - 1]):F1} kg/s, " +
               $"first pressure {pressure[0] / 1e6:F3} MPa, last pressure {pressure[CurvePoints - 1] / 1e6:F3} MPa; " +
               $"its flows over its burn add up to {curveMass / 1000.0:F1} t against {solid.InitialBurnableGrainMass / 1000.0:F1} t burnable " +
               $"(ratio {(solid.InitialBurnableGrainMass > 0f ? curveMass / solid.InitialBurnableGrainMass : double.NaN):F3}); " +
               $"its residue {preview.UnburnableGrainKg / 1000.0:F2} t against {residue / 1000.0:F2} t on the segments");

        // A volume ratio under 1 means the grain runs out before the web ends.
        PartTree? tree = solid.Rocket?.Parent?.FullPart?.Tree;
        if (tree?.Moles == null)
            return;
        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        foreach (SolidGrainSegment segment in solid.Stack.Segments)
        {
            GrainGeometryTable lut = segment.Geometry.Lut;
            double geometryVolume = lut.InitialGrainArea * segment.CasingInnerRadius * segment.CasingInnerRadius * segment.Length;
            double moleMass = segment.Grain != null ? moles[segment.Grain.StatesIdx].Mass : double.NaN;
            t.Info($"segment '{segment.Geometry.Id}' (library default '{GrainGeometryLibrary.Default.Id}'): R {segment.CasingInnerRadius:F4} m, " +
                   $"L {segment.Length:F3} m, GrainVolume {segment.GrainVolume:F3} m3 against {geometryVolume:F3} m3 for its geometry " +
                   $"(ratio {segment.GrainVolume / geometryVolume:F4}), mole container {segment.Grain?.ContainerVolume ?? float.NaN:F3} m3, " +
                   $"mole {moleMass / 1000.0:F3} t of {segment.InitialGrainMass / 1000.0:F3} t, " +
                   $"empty web {segment.ComputeGrainDepth(0f):F4} of {lut.MaxDepth:F4}");
        }
    }

    private static double Flow(double thrustNewtons, double ispSeconds) =>
        ispSeconds > 0.0 ? thrustNewtons / (ispSeconds * 9.80665) : 0.0;

    // Logs the grain one motor really consumes against the flow, pressure and regression rate that
    // its published state and a repeated curve solve give for the same grain.
    private static void ProbeSolid(TestContext t, Vehicle vehicle, SolidMotor? solid, double time,
                                   ref double previousGrain, ref double previousTime)
    {
        PartTree? tree = vehicle?.Parts;
        if (solid == null || tree?.Moles == null || tree.RocketNozzles == null || tree.RocketCores == null)
            return;
        if (!TryUsableGrainMass(solid, out double usable))
            return;

        double elapsed = time - previousTime;
        double consumption = elapsed > 0.0 ? (previousGrain - usable) / elapsed : double.NaN;
        previousGrain = usable;
        previousTime = time;

        RocketCoreState coreState = tree.RocketCores.States[solid.StatesIdx];
        double statePressure = coreState.Conditions.Core.Pressure;

        double nozzleFlow = 0.0;
        foreach (var nozzle in tree.RocketNozzles.GetModulesAndStates(solid.Rocket.Nozzles.AsSpan()))
            nozzleFlow += nozzle.State.Performance.MassFlowRate;

        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        float area = solid.ComputeBurningArea(moles);
        RocketCoreConditions solved = solid.SolveConditionsForArea(area, (float)statePressure);
        double solvedPressure = solved.Core.Pressure;
        double solvedFlow = 0.0;
        foreach (var nozzle in tree.RocketNozzles.GetModulesAndStates(solid.Rocket.Nozzles.AsSpan()))
            solvedFlow += nozzle.Module.ComputePerformance(in solved, 0f).MassFlowRate;

        double density = solid.Propellant?.StorageDensity ?? 0.0;
        double lawRate = solid.BurnRate.Evaluate((float)solvedPressure);
        double flownRate = density > 0.0 && area > 0f ? consumption / (density * area) : double.NaN;

        SolidGrainSegment? first = solid.Stack.Segments.Length > 0 ? solid.Stack.Segments[0] : null;
        double depth = first?.Grain != null ? first.ComputeGrainDepth(moles[first.Grain.StatesIdx].Mass) : double.NaN;
        double maxDepth = first != null ? first.Geometry.Lut.MaxDepth : double.NaN;

        t.Info($"probe t={time,6:F1}s grain={usable / 1000.0,6:F1}t flown={consumption,7:F1}kg/s " +
               $"state={coreState.MassFlowRate,7:F1}kg/s nozzles={nozzleFlow,7:F1}kg/s solved={solvedFlow,7:F1}kg/s " +
               $"p state={statePressure / 1e6,6:F3}MPa solved={solvedPressure / 1e6,6:F3}MPa " +
               $"area={area,7:F3}m2 rate flown={flownRate * 1000.0,6:F3}mm/s law={lawRate * 1000.0,6:F3}mm/s " +
               $"web={depth,6:F4} of {maxDepth:F4}");
    }

    private static void DescribeSequences(TestContext t, Vehicle vehicle, string when)
    {
        SequenceList list = vehicle.Parts.SequenceList;
        string rows = "";
        foreach (Sequence sequence in list.Sequences)
            rows += $" [{sequence.Number}: activated={sequence.Activated} parts={sequence.Parts.Length}]";
        int engines = 0, active = 0;
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            engines++;
            if (engine.IsActive)
                active++;
        }
        t.Info($"sequences {when}: next={list.GetNextSequenceNumber()}{rows}; engines {active}/{engines} active, " +
               $"thrust with propellant={StagingHelpers.HasActiveEngineWithPropellant(vehicle)}");
    }

    private static void DescribeState(TestContext t, Vehicle vehicle, VehicleAutopilotState state, double time)
    {
        t.Info($"t={time:F2}s running={state.Running} phase={state.Phase} acquired={state.ControlAcquired} " +
               $"holder={VehicleControlOwnership.HolderOf(vehicle)} failStreak={state.FailStreak} " +
               $"armed={StagingDetector.IsArmed(vehicle)} activations={StagingDetector.ActivationsOf(vehicle)} " +
               $"stagingActive={state.StagingActive} status='{state.Status}' error='{state.GuidanceError}'");
    }

    // The burn time of the whole stack at full thrust, the figure the staging bar totals.
    private static double StackBurnSeconds(UpfgVehicle model)
    {
        if (model == null)
            return 0.0;
        double total = 0.0;
        foreach (UpfgStage stage in model.Stages)
        {
            double flow = stage.Thrust / (stage.Isp * 9.80665);
            if (flow > 0.0)
                total += (stage.MassTotal - stage.MassDry) / flow;
        }
        return total;
    }

    // The angle above the local horizon, so a command that swings toward the vertical reads as a rise.
    private static double PitchDeg(double3 up, double3 dir)
    {
        if (dir.Length() < 0.5)
            return double.NaN;
        double cos = Math.Clamp(double3.Dot(up, double3.Normalize(dir)), -1.0, 1.0);
        return 90.0 - Math.Acos(cos) * 180.0 / Math.PI;
    }
}
