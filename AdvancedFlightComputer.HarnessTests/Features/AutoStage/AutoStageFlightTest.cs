using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Spawns a staged save clear of the home body, lights the first stage like a player, then holds
// full manual throttle and lets the detector perform every further staging, including a cascade
// through decoupler-only rows. Passes when no unactivated engine sequence is left, every activated
// engine sequence was seen producing thrust, and the vehicle ends dry on an orbit that clears the
// home body. A trailing decoupler-only sequence is left standing on purpose: staging only runs
// while an engine is still ahead.
//
// The vehicle comes from KSA_HEADLESS_VEHICLE, shared with the harness flight test; unset skips.
public sealed class AutoStageFlightTest : AfcTest
{
    private const int MaxFlightSeconds = 3600;
    private const double MinPeriapsisClearanceM = 100_000.0;
    // The last activation's ignition lands through the input queue a step later, so one dry sample
    // right after an activation is not burnout yet.
    private const int DrySteps = 3;

    public override string Name => "afc-autostage-flight";

    protected override void Execute(TestContext t)
    {
        string? saveId = Environment.GetEnvironmentVariable(TestSupport.VehicleEnvVar);
        if (string.IsNullOrEmpty(saveId))
        {
            t.Skip($"{TestSupport.VehicleEnvVar} not set");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        using AutoStageTestPatches.Scope patches = AutoStageTestPatches.Apply();
        try
        {
            Vehicle vehicle;
            Astronomical homeBody;
            try
            {
                vehicle = AutoStageFlightSupport.SpawnFromSave(t, saveId, "AfcAutoStageFlight", out homeBody);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;

            // Zero delays, because this test asserts plain immediate staging; the delays have their
            // own test. The spent-stage drop stays at its shipped default.
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            StagingConfig.DropSpentStages = StagingConfig.DropSpentStagesDefault;

            List<Sequence> pending = CollectUnactivatedSequences(vehicle);
            if (pending.Count < 2)
            {
                t.Skip($"'{saveId}' has {pending.Count} unactivated sequence(s); auto-staging needs a stage beyond the launch stage");
                return;
            }

            if (!AutoStageFlightSupport.Arm(t, vehicle))
                return;

            AutoStageFlightSupport.HoldPrograde(vehicle);
            AutoStageFlightSupport.IgniteFirstStage(vehicle, driver);
            if (!t.Check("the first stage ignites", StagingHelpers.HasActiveEngineWithPropellant(vehicle)))
                return;

            double startMass = vehicle.TotalMass;
            double mu = vehicle.Orbit.Parent.Mu;
            double startEnergy = Orbit.GetOrbitalEnergy(in vehicle.Orbit.StateVectors, mu);

            // Sequence objects, not numbers: SequenceList.Remove renumbers, and RemoveSpentSequences
            // runs inside Vehicle.Split, so the next number moves without anything being staged.
            int autoActivations = 0;
            int elapsed = 0;
            int dryStreak = 0;
            int engineSeqsLeft = RemainingEngineSequences(vehicle);
            // Set when an engine sequence activates, cleared once thrust is seen again. Without it
            // "no engine sequence left and not fueled" would accept a flight where every stage was
            // marked activated and none ever lit.
            bool awaitingThrust = false;
            while (elapsed < MaxFlightSeconds)
            {
                driver.Step(1.0);
                elapsed++;
                int activated = CountActivated(pending);
                while (autoActivations < activated)
                {
                    autoActivations++;
                    t.Info($"auto staging {autoActivations} at t={elapsed}s (next sequence " +
                           $"{vehicle.Parts.SequenceList.GetNextSequenceNumber()}, mass={vehicle.TotalMass:F1}kg)");
                }

                int engineSeqsNow = RemainingEngineSequences(vehicle);
                if (engineSeqsNow < engineSeqsLeft)
                {
                    awaitingThrust = true;
                    engineSeqsLeft = engineSeqsNow;
                }
                bool fueled = StagingHelpers.HasActiveEngineWithPropellant(vehicle);
                if (fueled)
                    awaitingThrust = false;

                dryStreak = engineSeqsNow == 0 && !fueled && !awaitingThrust ? dryStreak + 1 : 0;
                if (dryStreak >= DrySteps)
                    break;
            }

            bool dry = !StagingHelpers.HasActiveEngineWithPropellant(vehicle);
            int remaining = vehicle.Parts.SequenceList.GetNextSequenceNumber();
            t.Check("the flight ends before the runaway guard", elapsed < MaxFlightSeconds,
                $"next sequence {remaining}, {(dry ? "dry" : "still burning")}");
            t.Check("no engine sequence is left unactivated", RemainingEngineSequences(vehicle) == 0,
                $"next sequence {remaining}");
            t.Check("automatic staging was observed", autoActivations >= 1);
            t.Check("the last auto-staged engine sequence produced thrust", !awaitingThrust);
            t.Check("mass decreased", vehicle.TotalMass < startMass, $"{startMass:F1} -> {vehicle.TotalMass:F1}kg");

            // A prograde burn adds orbital energy on an elliptical and a hyperbolic end state alike.
            double endEnergy = Orbit.GetOrbitalEnergy(in vehicle.Orbit.StateVectors, mu);
            t.Check("orbital energy increased", endEnergy > startEnergy, $"{startEnergy:E3} -> {endEnergy:E3} J/kg");
            double clearance = vehicle.Orbit.Periapsis - homeBody.MeanRadius;
            t.Check("periapsis clears the home body", clearance >= MinPeriapsisClearanceM,
                $"{clearance / 1000.0:F0}km above '{homeBody.Id}', min {MinPeriapsisClearanceM / 1000.0:F0}km");
            t.Info($"summary: {autoActivations} auto staging(s), {elapsed}s sim time, mass {startMass:F1} -> " +
                   $"{vehicle.TotalMass:F1}kg, periapsis clearance {clearance / 1000.0:F0}km");
        }
        finally
        {
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
        }
    }

    private static List<Sequence> CollectUnactivatedSequences(Vehicle vehicle)
    {
        List<Sequence> result = new();
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (!seq.Activated && !seq.Parts.IsEmpty)
                result.Add(seq);
        }
        return result;
    }

    private static int CountActivated(List<Sequence> sequences)
    {
        int count = 0;
        foreach (Sequence seq in sequences)
        {
            if (seq.Activated)
                count++;
        }
        return count;
    }

    // Per module, the same way the detector asks it.
    private static int RemainingEngineSequences(Vehicle vehicle)
    {
        int count = 0;
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (!seq.Activated && !seq.Parts.IsEmpty && SequencedModules.LightsEngine(seq))
                count++;
        }
        return count;
    }
}
