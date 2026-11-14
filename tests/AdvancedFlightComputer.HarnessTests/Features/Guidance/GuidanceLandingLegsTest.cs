using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The legs of a landing, on a craft with stowed landing legs that falls toward the ground of an airless moon with its engine off.
// The landing is a held claim and a landing phase, so no mode flies and the fall is the game's own. Each step runs the leg deploy
// the guidance prefix runs, and the test reads the goal times of the legs:
//  - no leg is commanded while the legs are above LEGS DOWN AT, every leg on the first step at or below it, and no other animation;
//  - the legs come down in the length of their animation, and the vehicle height the plans use stays where it was;
//  - a retract by hand after the deploy stands for the rest of that landing;
//  - with the setting off, or with no claim on the craft, nothing is commanded, and a new landing that starts below the height
//    deploys at once.
public sealed class GuidanceLandingLegsTest : AfcTest
{
    private const double StepS = 0.05;
    private const double DeployAtM = 1000.0;
    private const double StartLegsM = 1800.0;
    private const double SinkMs = 30.0;

    public override string Name => "afc-guidance-landing-legs";

    protected override void Execute(TestContext t)
    {
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves("Luna 2", "Luna 1");
        if (saves.Count == 0 || t.System.All.Get("Luna") is not Celestial lunaBody || lunaBody is not IParentBody luna)
        {
            t.Skip("a landing-leg save (Luna 2 or Luna 1) and the Luna body are required.");
            return;
        }

        SimDriver driver = t.Session.CreateDriver();
        driver.StepOrbits = true;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        Vehicle? previousFocus = Program.ControlledVehicle;
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? craft = null;
        try
        {
            driver.Step(StepS, 2);
            // The vehicle height of the spawn is a guess, because the legs height the test reads uses the measured one.
            craft = Spawn(t, saves[0], luna, lunaBody, driver, StartLegsM, SinkMs, 3.0);
            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = craft;
            driver.Step(StepS, 2);

            var others = new List<(KeyframeAnimationModule Anim, float Goal)>();
            int legCount = 0;
            foreach (KeyframeAnimationModule anim in craft.Parts.Modules.Get<KeyframeAnimationModule>())
            {
                if (KsaLandingLegs.IsLeg(anim))
                {
                    legCount++;
                    anim.TimeGoal = 0f;
                }
                else
                    others.Add((anim, anim.TimeGoal));
            }
            t.Info($"{saves[0]}: {legCount} landing legs, {others.Count} other animations, deploy {KsaLandingLegs.DeploySeconds(craft):F3} s");
            if (!t.Check("the craft has landing legs", legCount > 0) || !t.Check("the legs stow", Settle(driver, craft)))
                return;
            t.Check("every leg reads stowed", KsaLandingLegs.Count(craft, out int stowedNow) == legCount && stowedNow == legCount,
                $"{stowedNow} of {legCount} stowed");
            if (!t.Check("measures the craft", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result before)))
                return;

            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            state.VehicleHeightAuto = false;
            state.VehicleHeightM = before.DeployedM;
            state.LegsAutoDeploy = true;
            state.LegsDeployHeightM = DeployAtM;
            Fall(t, craft, state, driver, before, others);
        }
        finally
        {
            if (craft != null)
            {
                VehicleControlOwnership.Release(craft, ControlClaimant.Guidance);
                VehicleAutopilotState.Remove(craft);
            }
            Program.ControlledVehicle = previousFocus;
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void Fall(TestContext t, Vehicle craft, VehicleAutopilotState state, SimDriver driver,
                             KsaVehicleHeight.Result before, List<(KeyframeAnimationModule Anim, float Goal)> others)
    {
        if (!t.Check("guidance takes the claim", VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _)))
            return;
        state.LandingPhase = GuidanceWindow.LandingPhase.TerminalCoast;

        double commandedAt = double.NaN, lowestBefore = double.NaN, highestStep = 0.0, last = double.NaN;
        double sim = 0.0;
        while (sim < 120.0)
        {
            double h = GuidanceWindow.LegsHeightM(craft, state);
            if (double.IsFinite(last))
                highestStep = Math.Max(highestStep, last - h);
            last = h;
            GuidanceWindow.StepLandingLegs(craft, state);
            int legs = KsaLandingLegs.Count(craft, out int stowed);
            if (stowed < legs)
            {
                commandedAt = h;
                t.Check("every leg is commanded at once", stowed == 0, $"{stowed} of {legs} still stowed");
                break;
            }
            lowestBefore = h;
            driver.Step(StepS);
            sim += StepS;
        }
        t.Info($"fall: legs commanded at {commandedAt:F1} m, last step above at {lowestBefore:F1} m, {highestStep:F2} m per step, after {sim:F1} s");
        if (!t.Check("the legs are commanded in the fall", double.IsFinite(commandedAt)))
            return;
        t.Check("no leg is commanded above the height", lowestBefore > DeployAtM, $"last step without a command at {lowestBefore:F1} m");
        t.Check("the legs are commanded on the first step at or below the height", commandedAt <= DeployAtM && commandedAt > DeployAtM - highestStep - 1e-6,
            $"commanded at {commandedAt:F1} m, {highestStep:F2} m per step");
        t.CheckAbs("the landing records the height", state.LegsFiredAtM, commandedAt, 1e-9);
        t.Check("no other animation is commanded", others.TrueForAll(o => o.Anim.TimeGoal == o.Goal));

        double deployStart = Universe.GetElapsedSeconds();
        bool down = Settle(driver, craft, () => GuidanceWindow.StepLandingLegs(craft, state));
        double deployS = Universe.GetElapsedSeconds() - deployStart;
        double h1 = GuidanceWindow.LegsHeightM(craft, state);
        t.Info($"deploy: down after {deployS:F2} s, legs at {h1:F1} m, {commandedAt - h1:F1} m below the command");
        if (t.Check("the legs come down", down))
            t.CheckAbs("the legs come down in the length of their animation", deployS, KsaLandingLegs.DeploySeconds(craft), 3.0 * StepS);
        if (t.Check("measures the deployed craft", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result after)))
        {
            t.Info($"vehicle height: stowed {before.DeployedM:F3} m, deployed {after.DeployedM:F3} m, as is {after.CurrentM:F3} m");
            // Stowed, the legs end before.DeployedM - before.CurrentM higher, which a plan measured from the craft as it is would jump by.
            t.Info($"a plan measured from the stowed craft would move by {before.DeployedM - before.CurrentM:F3} m");
            t.CheckAbs("the vehicle height the plans use stays where it was", after.DeployedM, before.DeployedM, 0.1);
            t.CheckAbs("the deployed legs are where the plan put them", after.CurrentM, after.DeployedM, 0.01);
        }

        foreach (KeyframeAnimationModule anim in craft.Parts.Modules.Get<KeyframeAnimationModule>())
        {
            if (KsaLandingLegs.IsLeg(anim))
                anim.TimeGoal = 0f;
        }
        for (int i = 0; i < 20; i++)
        {
            GuidanceWindow.StepLandingLegs(craft, state);
            driver.Step(StepS);
        }
        KsaLandingLegs.Count(craft, out int retracted);
        t.Check("a retract by hand stands for the rest of the landing", retracted == KsaLandingLegs.Count(craft), $"{retracted} retracted");

        // The landing ends: the claim and the phase go, and the next landing may deploy again.
        VehicleControlOwnership.Release(craft, ControlClaimant.Guidance);
        state.LandingPhase = GuidanceWindow.LandingPhase.Idle;
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("the end of the landing rearms the deploy", !state.LegsFired && double.IsNaN(state.LegsFiredAtM));
        Settle(driver, craft);

        double low = GuidanceWindow.LegsHeightM(craft, state);
        t.Info($"second landing at {low:F1} m");
        state.LegsAutoDeploy = false;
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        state.LandingPhase = GuidanceWindow.LandingPhase.TerminalCoast;
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("with the setting off no leg is commanded", KsaLandingLegs.Count(craft, out int offStowed) == offStowed && !state.LegsFired);

        state.LegsAutoDeploy = true;
        VehicleControlOwnership.Release(craft, ControlClaimant.Guidance);
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("without the claim no leg is commanded", KsaLandingLegs.Count(craft, out int freeStowed) == freeStowed && !state.LegsFired);

        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("a landing that starts below the height deploys at once", KsaLandingLegs.Count(craft, out int nowStowed) > 0 && nowStowed == 0 && state.LegsFired,
            $"{nowStowed} stowed, legs at {low:F1} m");
        t.Check("still no other animation is commanded", others.TrueForAll(o => o.Anim.TimeGoal == o.Goal));
        state.LandingPhase = GuidanceWindow.LandingPhase.Idle;
    }

    // A craft at legsM over the terrain under it, sinking at sinkMs, with its legs heightM under the centre of mass.
    private static Vehicle Spawn(TestContext t, string save, IParentBody luna, Celestial body, SimDriver driver, double legsM, double sinkMs, double heightM)
    {
        double3 dirCci = double3.Normalize(new double3(0.3, 0.9, 0.2));
        double3 dirCcf = dirCci.Transform(luna.GetCci2Ccf());
        double terrain = body.GetTerrainHeightFromDirCcf(dirCcf);
        double radius = luna.MeanRadius + terrain + heightM + legsM;
        // A small speed across, because a fall straight down is a degenerate conic.
        double3 across = double3.Normalize(double3.Cross(dirCci, new double3(0.0, 0.0, 1.0))) * 20.0;
        Orbit orbit = Orbit.CreateFromStateCci(luna, Universe.GetElapsedTime(), dirCci * radius, dirCci * -sinkMs + across, VehicleSpawner.OrbitLineColor);
        Vehicle craft = VehicleSpawner.SpawnFromSave(save, t.System, luna, "AfcLandingLegs_" + Guid.NewGuid().ToString("N"), orbit);
        driver.Step(StepS, 2);
        return craft;
    }

    private static bool Settle(SimDriver driver, Vehicle craft, Action? eachStep = null)
    {
        for (int i = 0; i < 400; i++)
        {
            if (!KeyframeAnimationModule.AnyAnimating(craft.Parts.States))
            {
                driver.Step(StepS, 2);
                return true;
            }
            eachStep?.Invoke();
            driver.Step(StepS);
        }
        return false;
    }
}
