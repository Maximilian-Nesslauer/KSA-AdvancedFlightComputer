using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AutoStage.HarnessTests;

// The end-to-end scenario: spawn a staged save into a circular orbit clear of the home body, light
// the first stage like a player, then hold full manual throttle and let AutoStage perform every
// further staging on its own (including cascading through decoupler-only sequences). Passes when no
// unactivated sequence carrying an engine is left, every engine sequence that activated was seen
// producing thrust, and the vehicle ends dry without a single manual staging call, still on an orbit
// that does not intersect the home body. A trailing decoupler-only sequence is left standing on
// purpose: AutoStage only stages while an engine is still ahead, so that is a valid end state.
//
// The vehicle comes from KSA_HEADLESS_VEHICLE (shared with the harness flight test); unset skips.
public sealed class AutoStagingFlightTest : IHarnessTest
{
    private const int MaxFlightSeconds = 3600; // same runaway guard as the harness's own flight test
    private const double MinPeriapsisClearanceM = 100_000.0; // periapsis height above the home body's mean radius
    // The last activation's engine ignition lands via the input queue a step later, so a single dry
    // sample right after an activation is not burnout yet. Require it to persist.
    private const int DrySteps = 3;

    public string Name => "autostage-flight";

    public int Run(HeadlessSession session)
    {
        string? saveId = Environment.GetEnvironmentVariable(TestSupport.VehicleEnvVar);
        if (string.IsNullOrEmpty(saveId))
        {
            HarnessLog.Line($"[autostage-flight] SKIP: {TestSupport.VehicleEnvVar} not set.");
            return 0;
        }

        AutoStageHost.EnsureInitialized();
        if (!AutoStageHost.CoreOk)
        {
            HarnessLog.Line("[autostage-flight] FAIL: AutoStage core patches are not active (see autostage-api-drift).");
            return 1;
        }

        CelestialSystem system = session.System;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(system);
        SimDriver driver = session.CreateDriver();
        bool ok = true;
        try
        {
            // Inside the try: Astronomical's constructor registers with the system before the
            // spawner finishes, so a throw part-way still leaves a vehicle for cleanup to remove.
            Vehicle vehicle;
            Astronomical homeBody;
            try
            {
                vehicle = AutoStageHost.SpawnFromSave(session, saveId, "AutoStageFlight", out homeBody);
            }
            catch (InvalidOperationException e)
            {
                HarnessLog.Line($"[autostage-flight] FAIL: {e.Message}");
                return 1;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;

            // Zero delays: this test asserts plain immediate auto-staging. The delay behaviour has
            // its own test; CleanupAfterFlight reloads the user's config. The spent-stage drop is
            // pinned to its shipped default so the end-to-end run covers the same behaviour a
            // player gets, whatever the local config says.
            Core.Config.EngineDelays.Clear();
            Core.Config.DecouplerDelays.Clear();
            Core.Config.DropSpentStages = Core.Config.DropSpentStagesDefault;

            int stagesLeft = CountUnactivatedSequences(vehicle);
            if (stagesLeft < 2)
            {
                HarnessLog.Line($"[autostage-flight] SKIP: '{saveId}' has {stagesLeft} unactivated sequence(s); " +
                                "auto-staging needs a stage beyond the launch stage.");
                return 0;
            }

            vehicle.ToggleEnum(AutoStageToggle.Enabled);
            if (!Mod.AutoStageEnabled)
            {
                HarnessLog.Line("[autostage-flight] FAIL: ToggleEnum(AutoStageToggle) did not enable the mod.");
                return 1;
            }

            AutoStageHost.HoldPrograde(vehicle);
            AutoStageHost.IgniteFirstStage(vehicle, driver);
            if (!StagingHelpers.HasActiveEngineWithPropellant(vehicle))
            {
                HarnessLog.Line("[autostage-flight] FAIL: the first stage did not ignite.");
                return 1;
            }

            double startMass = vehicle.TotalMass;
            double mu = vehicle.Orbit.Parent.Mu;
            double startEnergy = Orbit.GetOrbitalEnergy(in vehicle.Orbit.StateVectors, mu);
            // Sequence objects, not GetNextSequenceNumber: SequenceList.Remove decrements every
            // number at or above a removed sequence and RemoveSpentSequences runs inside
            // Vehicle.Split, so the reported next number moves without anything being staged, and
            // can even move backwards. Activated is not a latch either (SetActiveSequence rewrites
            // it across the whole list), so the running count only ever moves up.
            List<Sequence> pending = CollectUnactivatedSequences(vehicle);
            int autoActivations = 0;
            int elapsed = 0;
            int dryStreak = 0;
            int engineSeqsLeft = RemainingEngineSequences(vehicle);
            // Set when a sequence carrying an engine activates, cleared once the vehicle is seen
            // producing thrust again. Without it, "no engine sequence left and not fueled" would
            // accept a flight where every remaining stage was marked activated and none ever lit,
            // and HasActiveEngineWithPropellant dips to false on its own while a solid motor is
            // quenched, so a transient alone could end the flight.
            bool awaitingThrust = false;
            while (elapsed < MaxFlightSeconds)
            {
                driver.Step(1.0);
                elapsed++;
                int activated = CountActivated(pending);
                while (autoActivations < activated)
                {
                    autoActivations++;
                    HarnessLog.Line($"[autostage-flight] auto staging {autoActivations} at t={elapsed}s " +
                                    $"(next sequence {vehicle.Parts.SequenceList.GetNextSequenceNumber()}, " +
                                    $"mass={vehicle.TotalMass:F1}kg).");
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

                dryStreak = (engineSeqsNow == 0 && !fueled && !awaitingThrust) ? dryStreak + 1 : 0;
                if (dryStreak >= DrySteps)
                    break;
            }

            bool dry = !StagingHelpers.HasActiveEngineWithPropellant(vehicle);
            int remaining = vehicle.Parts.SequenceList.GetNextSequenceNumber();
            int remainingEngineSeqs = RemainingEngineSequences(vehicle);
            if (elapsed >= MaxFlightSeconds)
            {
                HarnessLog.Line($"[autostage-flight] FAIL: guard hit after {MaxFlightSeconds}s " +
                                $"(next sequence {remaining}, {(dry ? "dry" : "still burning")}).");
                ok = false;
            }
            // Not "every sequence activated": AutoStage only stages while a sequence carrying an
            // engine is still ahead, so a trailing decoupler-only sequence is left standing by
            // design and is a valid end state. What must be gone is anything that could still light.
            if (remainingEngineSeqs != 0)
            {
                HarnessLog.Line($"[autostage-flight] FAIL: flight ended with {remainingEngineSeqs} engine " +
                                $"sequence(s) never auto-activated (next sequence {remaining}).");
                ok = false;
            }
            if (autoActivations < 1)
            {
                HarnessLog.Line("[autostage-flight] FAIL: no automatic staging was observed.");
                ok = false;
            }
            if (awaitingThrust)
            {
                HarnessLog.Line("[autostage-flight] FAIL: the last auto-staged engine sequence never " +
                                "produced thrust; its parts were marked activated but nothing lit.");
                ok = false;
            }
            if (vehicle.TotalMass >= startMass)
            {
                HarnessLog.Line($"[autostage-flight] FAIL: mass did not decrease ({startMass:F1} -> {vehicle.TotalMass:F1}kg).");
                ok = false;
            }
            // A prograde burn must add orbital energy; valid for the elliptical start and a
            // hyperbolic end state alike, unlike an apoapsis comparison.
            double endEnergy = Orbit.GetOrbitalEnergy(in vehicle.Orbit.StateVectors, mu);
            if (endEnergy <= startEnergy)
            {
                HarnessLog.Line($"[autostage-flight] FAIL: orbital energy did not increase " +
                                $"({startEnergy:E3} -> {endEnergy:E3} J/kg); the burn did no prograde work.");
                ok = false;
            }
            double clearance = vehicle.Orbit.Periapsis - homeBody.MeanRadius;
            if (clearance < MinPeriapsisClearanceM)
            {
                HarnessLog.Line($"[autostage-flight] FAIL: periapsis only {clearance / 1000.0:F0}km above " +
                                $"'{homeBody.Id}' (min {MinPeriapsisClearanceM / 1000.0:F0}km), collision trajectory.");
                ok = false;
            }
            HarnessLog.Line($"[autostage-flight] summary: {autoActivations} auto staging(s), {elapsed}s sim time, " +
                            $"mass {startMass:F1} -> {vehicle.TotalMass:F1}kg, energy {startEnergy:E3} -> {endEnergy:E3} J/kg, " +
                            $"periapsis clearance {clearance / 1000.0:F0}km.");
        }
        finally
        {
            AutoStageHost.CleanupAfterFlight(system, preexisting);
        }

        HarnessLog.Line($"[autostage-flight] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private static int CountUnactivatedSequences(Vehicle vehicle)
        => CollectUnactivatedSequences(vehicle).Count;

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

    // Rows AutoStage could still stage for. Per module, the same way the mod asks it.
    private static int RemainingEngineSequences(Vehicle vehicle)
    {
        int count = 0;
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq.Activated || seq.Parts.IsEmpty)
                continue;
            if (SequencedModules.LightsEngine(seq))
                count++;
        }
        return count;
    }
}
