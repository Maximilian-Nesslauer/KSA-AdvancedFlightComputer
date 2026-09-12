using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A launch stage that mixes boosters with a core must shed the boosters the moment they burn out
// while the core keeps firing. The all-engines-dry trigger never sees an edge here, so this is the
// only end-to-end proof of the jettison analysis.
//
// Needs a save whose launch sequence has engines on both sides of the next sequence's decouplers.
// The candidate is resolved through TestSupport.ResolveVehicleSaves, so KSA_HEADLESS_VEHICLES
// overrides it.
public sealed class SpentStageDropTest : AfcTest
{
    private const string DefaultSave = "Test Vehicle 1";
    private const double BurnDt = 1.0;
    private const double ReactionDt = 0.25;
    private const double MaxBurnSeconds = 900.0;
    // The detector needs its dwell and the activation lands through the input queue a step later.
    private const double MaxReactionSeconds = 20.0;

    public override string Name => "afc-autostage-spent-drop";

    protected override void Execute(TestContext t)
    {
        // The default is known to have a mixed launch stage, so its rejection means the analysis
        // lost its grip on the game build. Only an operator-supplied save list may skip.
        bool defaultOnly = string.IsNullOrWhiteSpace(Environment.GetEnvironmentVariable(TestSupport.VehiclesEnvVar));

        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSave);
        if (saves.Count == 0)
        {
            if (defaultOnly)
                t.Fail("default save", $"'{DefaultSave}' is not in the game's Vehicles folder, so the spent-stage drop " +
                                       $"was not exercised. Recreate it, or name a substitute in {TestSupport.VehiclesEnvVar}");
            else
                t.Skip($"none of the saves named in {TestSupport.VehiclesEnvVar} are available");
            return;
        }

        using AutoStageTestPatches.Scope patches = AutoStageTestPatches.Apply();
        foreach (string saveId in saves)
        {
            if (RunSave(t, saveId))
                return;
        }

        if (defaultOnly)
            t.Fail("default save", $"'{DefaultSave}' was rejected as a mixed launch stage; the jettison analysis no longer recognises it");
        else
            t.Skip("no candidate save has a launch stage that keeps firing after its boosters burn out");
    }

    // True when the save was a usable scenario, whatever the verdict.
    private static bool RunSave(TestContext t, string saveId)
    {
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = AutoStageFlightSupport.SpawnFromSave(t, saveId, "AfcAutoStageSpentDrop", out _);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return true;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;

            // Zero delays, so the measured reaction is the detector's and not a configured countdown.
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            StagingConfig.DropSpentStages = true;

            if (!AutoStageFlightSupport.Arm(t, vehicle))
                return true;
            AutoStageFlightSupport.HoldPrograde(vehicle);

            // A stale BurnTarget, which a save carries when a burn was removed after the flight
            // computer accumulated delta-V against it. Its zero DeltaVTargetCci degenerates the
            // overshoot test, which used to disable staging for the whole flight.
            vehicle.FlightComputer.Burn = new BurnTarget
            {
                DeltaVTargetCci = float3.Zero,
                DeltaVAccumCci = new float3(7.23f, -29.44f, 16.46f),
            };

            // Staging only queues EngineController.SetIsActive, and the detector's postfix runs
            // before InputEvents applies it, so there is a frame where the sequence counts as
            // activated while its boosters are still inactive and full. That frame must not arm.
            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            StagingHelpers.EngineSurvey fresh = Survey(vehicle, out bool freshJettison);
            bool armedOnLaunchFrame = freshJettison && fresh.SpentInside > 0 && fresh.FueledInside == 0
                                      && fresh.BrokenInside == 0 && fresh.InactiveInside == 0;
            if (!t.Check("the drop does not arm on the frame the launch sequence fires", !armedOnLaunchFrame,
                    $"inside: {fresh.SpentInside} spent / {fresh.InactiveInside} not run"))
                return true;
            driver.Step(1.0);

            StagingHelpers.EngineSurvey survey = Survey(vehicle, out bool hasJettison);
            if (!hasJettison || survey.FueledInside == 0 || survey.FueledOutside == 0)
            {
                t.Info($"'{saveId}' is not a mixed launch stage (jettison={hasJettison}, inside={survey.FueledInside}, " +
                       $"outside={survey.FueledOutside}); trying the next save");
                LogVehicleShape(t, vehicle);
                return false;
            }

            t.Info($"'{saveId}': {survey.FueledInside} engine(s) on the jettisoned side, {survey.FueledOutside} staying with the vehicle");

            // The Sequence itself, not its number: SequenceList.Remove renumbers, and RemoveSpentSequences
            // runs inside ActivateNextSequence and Vehicle.Split.
            Sequence? pending = FindPendingSequence(vehicle);
            if (pending == null)
            {
                t.Fail("pending sequence", "no pending sequence after ignition");
                return true;
            }
            int vehiclesBefore = TestSupport.CountVehicles(t.System);

            double time = 0.0;
            while (time < MaxBurnSeconds && survey.FueledInside > 0)
            {
                if (pending.Activated)
                {
                    t.Fail("no early drop", $"staged at t={time:F1}s while {survey.FueledInside} jettisoned engine(s) still had propellant");
                    return true;
                }
                if (!survey.AnyFueled)
                {
                    t.Fail("the core keeps firing", $"the whole stack ran dry at t={time:F1}s without an early drop");
                    return true;
                }
                driver.Step(BurnDt);
                time += BurnDt;
                survey = Survey(vehicle, out hasJettison);

                // Both burnout and a lost jettison leave every inside counter at zero.
                if (!hasJettison)
                {
                    t.Fail("the jettison analysis keeps recognising the pending sequence", $"lost at t={time:F1}s");
                    return true;
                }
            }

            if (survey.FueledInside > 0)
            {
                t.Fail("booster burnout", $"the jettisoned engines never burnt out within {MaxBurnSeconds:F0}s");
                return true;
            }
            double tBurnout = time;
            // Snapshot here, not before the burn, or propellant spent alone would pass the mass check.
            double massAtBurnout = vehicle.TotalMass;

            double deadline = time + MaxReactionSeconds;
            while (!pending.Activated && time < deadline)
            {
                driver.Step(ReactionDt);
                time += ReactionDt;
            }
            double tStaged = time;

            // The activation only enqueues the decouple; the shed vehicle registers after the next drain.
            while (TestSupport.CountVehicles(t.System) <= vehiclesBefore && time < deadline)
            {
                driver.Step(ReactionDt);
                time += ReactionDt;
            }

            t.Check("the pending sequence activates after burnout", pending.Activated,
                $"{MaxReactionSeconds:F0}s after the jettisoned engines went dry");
            t.Check("the core is still firing after the drop", StagingHelpers.HasActiveEngineWithPropellant(vehicle));
            int vehiclesAfter = TestSupport.CountVehicles(t.System);
            t.Check("something separated", vehiclesAfter > vehiclesBefore, $"{vehiclesBefore} -> {vehiclesAfter} vehicles");
            t.Check("mass was shed across the drop", vehicle.TotalMass < massAtBurnout,
                $"{massAtBurnout:F1} -> {vehicle.TotalMass:F1}kg");
            t.Info($"summary: boosters spent at t={tBurnout:F1}s, staged {tStaged - tBurnout:F2}s later, " +
                   $"{vehiclesAfter - vehiclesBefore} vehicle(s) shed, stale burn target survived={vehicle.FlightComputer.Burn != null}");
            return true;
        }
        finally
        {
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
        }
    }

    private static Sequence? FindPendingSequence(Vehicle vehicle)
    {
        SequenceList seqList = vehicle.Parts.SequenceList;
        int number = seqList.GetNextSequenceNumber();
        foreach (Sequence sequence in seqList.Sequences)
        {
            if (sequence.Number == number)
                return sequence;
        }
        return null;
    }

    // Why a save was rejected: either it has no mixed launch stage, or the analysis lost its grip.
    private static void LogVehicleShape(TestContext t, Vehicle vehicle)
    {
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            string cores = "";
            foreach (RocketCore core in engine.Cores)
            {
                bool burning = coreStates[core.StatesIdx].Throttle > 0f;
                cores += $" [{core.GetType().Name} burning={burning} fed={core.ComputePropellantAvailable(moleStates, burning)}]";
            }
            t.Info($"  engine '{engine.Parent.FullPart.Id}' seq={engine.Sequence} active={engine.IsActive}{cores}");
        }

        int next = vehicle.Parts.SequenceList.GetNextSequenceNumber();
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
        {
            if (sequence.Number != next)
                continue;
            foreach (Part part in sequence.Parts)
            {
                foreach (ISequenced module in part.InSequence(next))
                {
                    if (module is not Decoupler decoupler)
                        continue;
                    t.Info($"  next seq {next} decoupler on '{part.Id}' {SequencedModules.Describe(module)} " +
                           $"connector='{decoupler.Connector.Id}' connected={decoupler.Connector.Connection != null}");
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
