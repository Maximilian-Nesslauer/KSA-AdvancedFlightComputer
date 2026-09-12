using AutoStage.Core;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AutoStage.HarnessTests;

// The partial-depletion scenario: a launch stage that mixes boosters with a core must shed the
// boosters the moment they burn out, while the core keeps firing. The all-engines-dry trigger never
// sees an edge here, so this is the only thing that proves the jettison analysis works end to end.
//
// Needs a save whose launch sequence has engines on both sides of the next sequence's decouplers,
// i.e. boosters that burn out while a core keeps firing; any other save skips. The candidate is
// resolved through TestSupport.ResolveVehicleSaves, so KSA_HEADLESS_VEHICLES overrides it.
public sealed class SpentStageDropTest : IHarnessTest
{
    private const string DefaultSave = "Test Vehicle 1";
    private const double BurnDt = 1.0;
    private const double ReactionDt = 0.25;
    private const double MaxBurnSeconds = 900.0;
    // The detector needs its confirmation frames and the activation lands via the input queue a
    // step later, so the drop trails booster burnout by a few steps.
    private const double MaxReactionSeconds = 20.0;

    public string Name => "autostage-spent-drop";

    public int Run(HeadlessSession session)
    {
        AutoStageHost.EnsureInitialized();
        if (!AutoStageHost.CoreOk)
        {
            HarnessLog.Line("[autostage-spent-drop] FAIL: AutoStage core patches are not active (see autostage-api-drift).");
            return 1;
        }

        // The built-in default is known to have a mixed launch stage, so it being rejected means
        // the analysis lost its grip on the game build, not that the save is unsuitable. Only an
        // operator-supplied save list is allowed to skip.
        bool defaultOnly = string.IsNullOrWhiteSpace(
            Environment.GetEnvironmentVariable(TestSupport.VehiclesEnvVar));

        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSave);
        if (saves.Count == 0)
        {
            // Passing here would report the only end-to-end cover of the jettison analysis as green
            // while it ran nothing at all. A missing default is lost coverage, not a valid run.
            if (defaultOnly)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: the default save '{DefaultSave}' is not in " +
                                "the game's Vehicles folder, so the spent-stage drop was not exercised. " +
                                $"Recreate it, or name a substitute in {TestSupport.VehiclesEnvVar}.");
                return 1;
            }
            HarnessLog.Line($"[autostage-spent-drop] SKIP: none of the saves named in " +
                            $"{TestSupport.VehiclesEnvVar} are available.");
            return 0;
        }

        foreach (string saveId in saves)
        {
            int result = RunSave(session, saveId, out bool ran);
            if (ran)
                return result;
        }

        if (defaultOnly)
        {
            HarnessLog.Line($"[autostage-spent-drop] FAIL: '{DefaultSave}' was rejected as a mixed " +
                            "launch stage; the jettison analysis no longer recognises it.");
            return 1;
        }

        HarnessLog.Line("[autostage-spent-drop] SKIP: no candidate save has a launch stage that " +
                        "keeps firing after its boosters burn out.");
        return 0;
    }

    private static int RunSave(HeadlessSession session, string saveId, out bool ran)
    {
        ran = false;
        CelestialSystem system = session.System;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(system);
        SimDriver driver = session.CreateDriver();
        bool ok = true;
        try
        {
            // Inside the try: Astronomical's constructor registers with the system before the
            // spawner finishes, so a throw part-way still leaves a vehicle for cleanup to remove.
            Vehicle vehicle;
            try
            {
                vehicle = AutoStageHost.SpawnFromSave(session, saveId, "AutoStageSpentDrop", out _);
            }
            catch (InvalidOperationException e)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: {e.Message}");
                ran = true;
                return 1;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;

            // Zero delays so the measured reaction is the detector's, not a configured countdown.
            // CleanupAfterFlight reloads the user's config, including the flag.
            Config.EngineDelays.Clear();
            Config.DecouplerDelays.Clear();
            Config.DropSpentStages = true;

            vehicle.ToggleEnum(AutoStageToggle.Enabled);
            if (!Mod.AutoStageEnabled)
            {
                HarnessLog.Line("[autostage-spent-drop] FAIL: ToggleEnum(AutoStageToggle) did not enable the mod.");
                ran = true;
                return 1;
            }

            AutoStageHost.HoldPrograde(vehicle);

            // Fly with a stale BurnTarget aboard, which is what a save carries when a burn was
            // removed from the plan after the flight computer had already accumulated delta-V
            // against it. Its DeltaVTargetCci is the zero vector, and the overshoot test that
            // decides "burn complete" degenerates on a zero target, which used to disable staging
            // for the whole flight. Installed here so the scenario runs against that state.
            vehicle.FlightComputer.Burn = new BurnTarget
            {
                DeltaVTargetCci = float3.Zero,
                DeltaVAccumCci = new float3(7.23f, -29.44f, 16.46f),
            };

            // Light the launch stage and, before the activation is drained, assert the drop is not
            // armed. Staging only queues EngineController.SetIsActive, and in the running game the
            // detector's postfix runs before InputEvents applies it, so there is a frame where the
            // sequence counts as activated while its boosters are still IsActive == false and full.
            // Treating those as spent would jettison a stage the instant it is staged. The driver
            // drains at the top of a step, so the window only exists between calls.
            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            StagingHelpers.EngineSurvey fresh = Survey(vehicle, out bool freshJettison);
            if (freshJettison && fresh.SpentInside > 0 && fresh.FueledInside == 0
                && fresh.BrokenInside == 0 && fresh.InactiveInside == 0)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: the drop armed on the frame the launch " +
                                $"sequence fired, before its engines ran (inside: {fresh.SpentInside} " +
                                $"spent / {fresh.InactiveInside} not run).");
                ran = true;
                return 1;
            }
            driver.Step(1.0);

            StagingHelpers.EngineSurvey survey = Survey(vehicle, out bool hasJettison);
            if (!hasJettison || survey.FueledInside == 0 || survey.FueledOutside == 0)
            {
                HarnessLog.Line($"[autostage-spent-drop] '{saveId}' is not a mixed launch stage " +
                                $"(jettison={hasJettison}, inside={survey.FueledInside}, " +
                                $"outside={survey.FueledOutside}); trying the next save.");
                LogVehicleShape(vehicle);
                return 0;
            }

            ran = true;
            HarnessLog.Line($"[autostage-spent-drop] '{saveId}': {survey.FueledInside} engine(s) on the " +
                            $"jettisoned side, {survey.FueledOutside} staying with the vehicle.");

            // Hold the Sequence itself, not its number: SequenceList.Remove decrements every number
            // at or above a removed sequence, and RemoveSpentSequences runs inside both
            // ActivateNextSequence and Vehicle.Split, so a number captured here can end up naming a
            // different sequence by the time the drop lands.
            Sequence? pending = FindPendingSequence(vehicle);
            if (pending == null)
            {
                HarnessLog.Line("[autostage-spent-drop] FAIL: no pending sequence after ignition.");
                return 1;
            }
            int vehiclesBefore = TestSupport.CountVehicles(system);

            double t = 0.0;
            while (t < MaxBurnSeconds && survey.FueledInside > 0)
            {
                if (pending.Activated)
                {
                    HarnessLog.Line($"[autostage-spent-drop] FAIL: staged at t={t:F1}s while " +
                                    $"{survey.FueledInside} jettisoned engine(s) still had propellant.");
                    return 1;
                }
                if (!survey.AnyFueled)
                {
                    HarnessLog.Line($"[autostage-spent-drop] FAIL: the whole stack ran dry at t={t:F1}s " +
                                    "without an early drop.");
                    return 1;
                }
                driver.Step(BurnDt);
                t += BurnDt;
                survey = Survey(vehicle, out hasJettison);

                // Without this the loop cannot tell "the boosters burnt out" from "the analysis
                // stopped recognising the jettison": both leave every inside counter at zero, and
                // the assertions below would score the second as success.
                if (!hasJettison)
                {
                    HarnessLog.Line($"[autostage-spent-drop] FAIL: the jettison analysis stopped " +
                                    $"recognising the pending sequence at t={t:F1}s.");
                    return 1;
                }
            }

            if (survey.FueledInside > 0)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: the jettisoned engines never burnt out " +
                                $"within {MaxBurnSeconds:F0}s.");
                return 1;
            }
            double tBurnout = t;
            // Snapshot here, not before the burn: comparing against the pre-burn mass would pass on
            // propellant spent alone and prove nothing about separation.
            double massAtBurnout = vehicle.TotalMass;

            double deadline = t + MaxReactionSeconds;
            while (!pending.Activated && t < deadline)
            {
                driver.Step(ReactionDt);
                t += ReactionDt;
            }
            double tStaged = t;

            // The activation only enqueues the decouple; Decoupler.Decouple runs when the next step
            // drains InputEvents, and the shed vehicle is registered after that.
            while (TestSupport.CountVehicles(system) <= vehiclesBefore && t < deadline)
            {
                driver.Step(ReactionDt);
                t += ReactionDt;
            }

            if (!pending.Activated)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: the pending sequence never activated, " +
                                $"{MaxReactionSeconds:F0}s after the jettisoned engines went dry.");
                ok = false;
            }
            if (!StagingHelpers.HasActiveEngineWithPropellant(vehicle))
            {
                HarnessLog.Line("[autostage-spent-drop] FAIL: the vehicle lost all thrust across the drop; " +
                                "the point is to shed the boosters while the core keeps firing.");
                ok = false;
            }
            int vehiclesAfter = TestSupport.CountVehicles(system);
            if (vehiclesAfter <= vehiclesBefore)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: nothing separated ({vehiclesBefore} -> " +
                                $"{vehiclesAfter} vehicles).");
                ok = false;
            }
            if (vehicle.TotalMass >= massAtBurnout)
            {
                HarnessLog.Line($"[autostage-spent-drop] FAIL: no mass was shed across the drop " +
                                $"({massAtBurnout:F1} -> {vehicle.TotalMass:F1}kg).");
                ok = false;
            }

            HarnessLog.Line($"[autostage-spent-drop] summary: boosters spent at t={tBurnout:F1}s, " +
                            $"staged {tStaged - tBurnout:F2}s later, {vehiclesAfter - vehiclesBefore} " +
                            $"vehicle(s) shed, mass {massAtBurnout:F1} -> {vehicle.TotalMass:F1}kg, " +
                            $"core still firing, stale burn target survived={vehicle.FlightComputer.Burn != null}.");
        }
        finally
        {
            AutoStageHost.CleanupAfterFlight(system, preexisting);
        }

        HarnessLog.Line($"[autostage-spent-drop] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private static Sequence? FindPendingSequence(Vehicle vehicle)
    {
        SequenceList seqList = vehicle.Parts.SequenceList;
        int number = seqList.GetNextSequenceNumber();
        foreach (Sequence sequence in seqList.Sequences)
        {
            if (sequence.Number == number) return sequence;
        }
        return null;
    }

    // Why a save was rejected is the interesting part of a skip: it is either a save that genuinely
    // has no mixed launch stage, or a jettison analysis that lost its grip on the game build.
    private static void LogVehicleShape(Vehicle vehicle)
    {
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            string cores = "";
            foreach (RocketCore core in engine.Cores)
            {
                bool burning = coreStates[core.StatesIdx].Throttle > 0f;
                cores += $" [{core.GetType().Name} burning={burning} " +
                         $"fed={core.ComputePropellantAvailable(moleStates, burning)}]";
            }
            // The engine's own row, not the part's.
            HarnessLog.Line($"[autostage-spent-drop]   engine '{engine.Parent.FullPart.Id}' " +
                            $"seq={engine.Sequence} active={engine.IsActive}{cores}");
        }

        int next = vehicle.Parts.SequenceList.GetNextSequenceNumber();
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
        {
            if (sequence.Number != next) continue;
            foreach (Part part in sequence.Parts)
            {
                // Only what this row fires.
                foreach (ISequenced module in part.InSequence(next))
                {
                    if (module is not Decoupler decoupler) continue;
                    HarnessLog.Line($"[autostage-spent-drop]   next seq {next} decoupler on " +
                                    $"'{part.Id}' {SequencedModules.Describe(module)} " +
                                    $"connector='{decoupler.Connector.Id}' " +
                                    $"connected={decoupler.Connector.Connection != null}");
                }
            }
        }
    }

    private static StagingHelpers.EngineSurvey Survey(Vehicle vehicle, out bool hasJettison)
    {
        IReadOnlySet<Part>? jettison = JettisonAnalysis.GetPendingJettison(vehicle);
        hasJettison = jettison != null;
        return StagingHelpers.SurveyActiveEngines(vehicle, jettison);
    }
}
