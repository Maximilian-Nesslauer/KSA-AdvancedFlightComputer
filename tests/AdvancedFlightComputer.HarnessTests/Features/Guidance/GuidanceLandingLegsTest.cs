using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Gfold;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The legs of a landing, on a craft with stowed landing legs that falls toward the ground of an airless moon with its engine off.
// The landing is a held claim and a landing phase, so no mode flies and the fall is the game's own. Each step runs the leg deploy
// the guidance prefix runs, and the test reads the goal times of the legs. The deploy time is the deploy time of the legs plus the
// buffer:
//  - with an entry plan that ends 10 s after the start, no leg is commanded while the plan has more than the deploy time to go,
//    every leg on the first step at or under it, far above where the fall would have them, and no other animation;
//  - the legs come down in the length of their animation, and the vehicle height the plans use stays where it was;
//  - a retract by hand after the deploy stands for the rest of that landing;
//  - with the setting off, or with no claim on the craft, nothing is commanded, and a new landing whose plan already has less than
//    the deploy time to go deploys at once;
//  - with no plan, the clock is the fall of the legs to the ground at the descent rate under gravity, and the legs are commanded on
//    the first step whose fall time is at or under the deploy time.
// Before the craft, the test reads which clock the modes of a bare state select (GuidanceWindow.LegsClocksOf): the touchdown plan
// while touchdown guidance flies or stands by, the entry plan in its glide or burn, the G-FOLD arrival unless a search runs, the
// impact of a boostback glide capped by the fall time, a plan whose end has passed never, and the order between them.
public sealed class GuidanceLandingLegsTest : AfcTest
{
    private const double StepS = 0.05;
    private const double BufferS = 1.0;
    private const double PlanS = 10.0;
    private const double StartLegsM = 1800.0;
    private const double SinkMs = 30.0;

    public override string Name => "afc-guidance-landing-legs";

    protected override void Execute(TestContext t)
    {
        ClockSelection(t);
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
            state.LegsDeployBufferS = BufferS;
            double deployAt = KsaLandingLegs.DeploySeconds(craft) + BufferS;
            t.CheckAbs("the deploy time is the deploy time of the legs plus the buffer", LegsRules.DeployAtS(KsaLandingLegs.DeploySeconds(craft), BufferS), deployAt, 1e-12);
            if (PlanLanding(t, craft, state, driver, before, others, deployAt))
                FallLanding(t, craft, state, driver, deployAt);
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

    private static bool PlanLanding(TestContext t, Vehicle craft, VehicleAutopilotState state, SimDriver driver,
                                    KsaVehicleHeight.Result before, List<(KeyframeAnimationModule Anim, float Goal)> others, double deployAt)
    {
        if (!t.Check("guidance takes the claim", VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _)))
            return false;
        double planEnd = Universe.GetElapsedSeconds() + PlanS;
        FlyEntryPlan(state, planEnd);

        double commandedToGo = double.NaN, commandedAtM = double.NaN, lastToGoBefore = double.NaN, fallAtCommand = double.NaN;
        LegsClock clockAtCommand = LegsClock.None;
        double sim = 0.0;
        while (sim < 2.0 * PlanS)
        {
            double h = GuidanceWindow.LegsHeightM(craft, state);
            double toGo = GuidanceWindow.LegsToGo(craft, state, h, Universe.GetElapsedSeconds(), out LegsClock clock);
            GuidanceWindow.StepLandingLegs(craft, state);
            int legs = KsaLandingLegs.Count(craft, out int stowed);
            if (stowed < legs)
            {
                (commandedToGo, commandedAtM, clockAtCommand) = (toGo, h, clock);
                fallAtCommand = FallSeconds(craft, h);
                t.Check("every leg is commanded at once", stowed == 0, $"{stowed} of {legs} still stowed");
                break;
            }
            lastToGoBefore = toGo;
            driver.Step(StepS);
            sim += StepS;
        }
        t.Info($"plan: legs commanded at {commandedToGo:F2} s to go on the {LegsRules.ClockName(clockAtCommand)}, {commandedAtM:F1} m, " +
               $"where the fall has {fallAtCommand:F1} s to go; last step without a command at {lastToGoBefore:F2} s, after {sim:F2} s");
        if (!t.Check("the legs are commanded in the landing", double.IsFinite(commandedToGo)))
            return false;
        t.Check("the clock is the plan that flies", clockAtCommand == LegsClock.EntryPlan, clockAtCommand.ToString());
        t.Check("no leg is commanded with more than the deploy time to go", lastToGoBefore > deployAt, $"{lastToGoBefore:F3} s");
        t.Check("the legs are commanded on the first step at or under the deploy time", commandedToGo <= deployAt && commandedToGo > deployAt - StepS - 1e-6,
            $"{commandedToGo:F3} s, deploy time {deployAt:F3} s");
        t.Check("the plan, not the height, sets the deploy", fallAtCommand > 2.0 * deployAt, $"fall {fallAtCommand:F1} s");
        t.CheckAbs("the landing records the time to go", state.LegsFiredToGoS, commandedToGo, 1e-9);
        t.CheckAbs("the landing records the height", state.LegsFiredAtM, commandedAtM, 1e-9);
        t.Check("no other animation is commanded", others.TrueForAll(o => o.Anim.TimeGoal == o.Goal));

        double deployStart = Universe.GetElapsedSeconds();
        bool down = Settle(driver, craft, () => GuidanceWindow.StepLandingLegs(craft, state));
        double deployS = Universe.GetElapsedSeconds() - deployStart;
        t.Info($"deploy: down after {deployS:F2} s, {planEnd - Universe.GetElapsedSeconds():F2} s before the plan end");
        if (t.Check("the legs come down", down))
        {
            t.CheckAbs("the legs come down in the length of their animation", deployS, KsaLandingLegs.DeploySeconds(craft), 3.0 * StepS);
            t.Check("the legs are down before the plan ends", Universe.GetElapsedSeconds() < planEnd, $"{planEnd - Universe.GetElapsedSeconds():F2} s left");
        }
        if (t.Check("measures the deployed craft", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result after)))
        {
            t.Info($"vehicle height: stowed {before.DeployedM:F3} m, deployed {after.DeployedM:F3} m, as is {after.CurrentM:F3} m");
            // Stowed, the legs end before.DeployedM - before.CurrentM higher, which a plan measured from the craft as it is would jump by.
            t.Info($"a plan measured from the stowed craft would move by {before.DeployedM - before.CurrentM:F3} m");
            t.CheckAbs("the vehicle height the plans use stays where it was", after.DeployedM, before.DeployedM, 0.1);
            t.CheckAbs("the deployed legs are where the plan put them", after.CurrentM, after.DeployedM, 0.01);
        }

        Retract(craft);
        for (int i = 0; i < 20; i++)
        {
            GuidanceWindow.StepLandingLegs(craft, state);
            driver.Step(StepS);
        }
        KsaLandingLegs.Count(craft, out int retracted);
        t.Check("a retract by hand stands for the rest of the landing", retracted == KsaLandingLegs.Count(craft), $"{retracted} retracted");

        EndLanding(craft, state);
        t.Check("the end of the landing rearms the deploy", !state.LegsFired && double.IsNaN(state.LegsFiredAtM) && double.IsNaN(state.LegsFiredToGoS));
        Settle(driver, craft);

        // A second landing whose plan ends within the deploy time.
        state.LegsAutoDeploy = false;
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        FlyEntryPlan(state, Universe.GetElapsedSeconds() + 0.5 * deployAt);
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("with the setting off no leg is commanded", KsaLandingLegs.Count(craft, out int offStowed) == offStowed && !state.LegsFired);

        state.LegsAutoDeploy = true;
        VehicleControlOwnership.Release(craft, ControlClaimant.Guidance);
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("without the claim no leg is commanded", KsaLandingLegs.Count(craft, out int freeStowed) == freeStowed && !state.LegsFired);

        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        GuidanceWindow.StepLandingLegs(craft, state);
        t.Check("a landing that starts within the deploy time deploys at once", KsaLandingLegs.Count(craft, out int nowStowed) > 0 && nowStowed == 0 && state.LegsFired,
            $"{nowStowed} stowed, {state.LegsFiredToGoS:F2} s to go");
        t.Check("still no other animation is commanded", others.TrueForAll(o => o.Anim.TimeGoal == o.Goal));

        EndLanding(craft, state);
        Retract(craft);
        return t.Check("the legs stow for the fall", Settle(driver, craft));
    }

    // The fall ends on the ground about a second after the legs are down, so the test steps no further than the command.
    private static void FallLanding(TestContext t, Vehicle craft, VehicleAutopilotState state, SimDriver driver, double deployAt)
    {
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        state.LandingPhase = GuidanceWindow.LandingPhase.TerminalCoast;
        double commandedToGo = double.NaN, commandedAtM = double.NaN, lastToGoBefore = double.NaN, oracle = double.NaN;
        LegsClock clockAtCommand = LegsClock.None;
        double sim = 0.0, worstOracle = 0.0;
        while (sim < 60.0)
        {
            double h = GuidanceWindow.LegsHeightM(craft, state);
            double toGo = GuidanceWindow.LegsToGo(craft, state, h, Universe.GetElapsedSeconds(), out LegsClock clock);
            oracle = FallSeconds(craft, h);
            worstOracle = Math.Max(worstOracle, Math.Abs(toGo - oracle));
            GuidanceWindow.StepLandingLegs(craft, state);
            int legs = KsaLandingLegs.Count(craft, out int stowed);
            if (stowed < legs)
            {
                (commandedToGo, commandedAtM, clockAtCommand) = (toGo, h, clock);
                t.Check("every leg is commanded at once in the fall", stowed == 0, $"{stowed} of {legs} still stowed");
                break;
            }
            lastToGoBefore = toGo;
            driver.Step(StepS);
            sim += StepS;
        }
        t.Info($"fall: legs commanded at {commandedToGo:F2} s to go on the {LegsRules.ClockName(clockAtCommand)}, {commandedAtM:F1} m; " +
               $"last step without a command at {lastToGoBefore:F2} s, after {sim:F2} s");
        if (!t.Check("the legs are commanded in the fall", double.IsFinite(commandedToGo)))
            return;
        t.Check("with no plan the clock is the fall", clockAtCommand == LegsClock.Fall, clockAtCommand.ToString());
        t.Check("the fall time is the one of the height, the descent rate and gravity", worstOracle < 1e-9, $"{worstOracle:E2} s");
        t.Check("no leg is commanded with more than the deploy time to fall", lastToGoBefore > deployAt, $"{lastToGoBefore:F3} s");
        t.Check("the legs are commanded on the first step at or under the deploy time", commandedToGo <= deployAt && commandedToGo > deployAt - 2.0 * StepS,
            $"{commandedToGo:F3} s, deploy time {deployAt:F3} s");
        t.CheckAbs("the fall landing records the height", state.LegsFiredAtM, commandedAtM, 1e-9);
        EndLanding(craft, state);
    }

    private static void ClockSelection(TestContext t)
    {
        const double Now = 1000.0, Fall = 7.0;
        var touchdown = new Ksa6DofPlan(Array.Empty<double>(), Array.Empty<double>(), 10.0, Now - 5.0, 0);
        var ended = new Ksa6DofPlan(Array.Empty<double>(), Array.Empty<double>(), 4.0, Now - 5.0, 0);
        var entry = new Plan3Dof(Array.Empty<double>(), Array.Empty<double>(), 0, 0, 5.0, 15.0, Now, Now, Phase3Dof.Burn, 0.0);
        var gfold = new GfoldTrajectory
        {
            Status = ConicStatus.Optimal, Dt = 1.0, Position = Array.Empty<double[]>(), Velocity = Array.Empty<double[]>(),
            AccelCmd = Array.Empty<double[]>(), Sigma = Array.Empty<double>(), Mass = Array.Empty<double>(),
            LandingPoint = new double[3], LandingErrorNorm = 0.0, Iterations = 0,
        };

        void Expect(string name, VehicleAutopilotState s, Ksa6DofPlan? plan, double fall, LegsClock clock, double toGo)
        {
            LegsClocks clocks = GuidanceWindow.LegsClocksOf(s, plan!, fall, Now);
            double got = LegsRules.ToGo(in clocks, out LegsClock gotClock);
            t.Check("clock: " + name, gotClock == clock && (Math.Abs(got - toGo) < 1e-9 || (double.IsNaN(got) && double.IsNaN(toGo))), $"{gotClock} at {got:F3} s, expected {clock} at {toGo:F3} s");
        }

        var s = new VehicleAutopilotState();
        Expect("no mode gives the fall", s, touchdown, Fall, LegsClock.Fall, Fall);
        s.Active = true;
        Expect("touchdown guidance flies its plan", s, touchdown, Fall, LegsClock.TouchdownPlan, 5.0);
        Expect("a touchdown plan whose end has passed gives no time", s, ended, Fall, LegsClock.Fall, Fall);
        Expect("touchdown guidance with no plan gives no time", s, null, Fall, LegsClock.Fall, Fall);
        s.ThreeDofPlan = entry;
        s.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Burn;
        Expect("the touchdown plan comes before the entry plan", s, touchdown, Fall, LegsClock.TouchdownPlan, 5.0);
        s.Active = false;
        Expect("entry guidance in its burn flies its plan", s, touchdown, Fall, LegsClock.EntryPlan, 20.0);
        s.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Glide;
        Expect("entry guidance in its glide flies its plan", s, touchdown, Fall, LegsClock.EntryPlan, 20.0);
        s.LandingPhase = GuidanceWindow.LandingPhase.GfoldDescent;
        s.GfoldPlan = gfold;
        s.GfoldArrivalTime = Now + 30.0;
        Expect("the entry plan comes before the G-FOLD arrival", s, touchdown, Fall, LegsClock.EntryPlan, 20.0);
        s.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Idle;
        Expect("entry guidance out of its glide and burn gives no time", s, null, double.NaN, LegsClock.GfoldPlan, 30.0);
        s.SixDofStandby = true;
        Expect("the G-FOLD arrival comes before the standby plan", s, touchdown, Fall, LegsClock.GfoldPlan, 30.0);
        s.GfoldForceSearch = true;
        Expect("a G-FOLD search hides its arrival", s, touchdown, Fall, LegsClock.TouchdownStandby, 5.0);
        s.GfoldForceSearch = false;
        s.GfoldPlan = null;
        Expect("G-FOLD with no plan gives no time", s, touchdown, Fall, LegsClock.TouchdownStandby, 5.0);
        s.LandingPhase = GuidanceWindow.LandingPhase.Idle;
        s.BoostbackPhase = GuidanceWindow.BoostbackPhase.Glide;
        s.GlideLastUpdate = Now - 2.0;
        s.GlideTgo = 40.0;
        Expect("the standby plan comes before the glide impact", s, touchdown, Fall, LegsClock.TouchdownStandby, 5.0);
        s.SixDofStandby = false;
        Expect("a glide gives its impact", s, touchdown, 50.0, LegsClock.Impact, 38.0);
        Expect("the fall time caps the glide impact", s, touchdown, Fall, LegsClock.Impact, Fall);
        s.BoostbackPhase = GuidanceWindow.BoostbackPhase.Boostback;
        Expect("a boostback out of its glide gives no impact", s, touchdown, 50.0, LegsClock.Fall, 50.0);
        Expect("no clock at all gives none", s, null, double.NaN, LegsClock.None, double.NaN);
    }

    private static void FlyEntryPlan(VehicleAutopilotState state, double endS)
    {
        state.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Burn;
        state.ThreeDofPlan = new Plan3Dof(Array.Empty<double>(), Array.Empty<double>(), 0, 0, 0.0, endS - Universe.GetElapsedSeconds(),
            Universe.GetElapsedSeconds(), Universe.GetElapsedSeconds(), Phase3Dof.Burn, 0.0);
    }

    // The landing ends: the claim, the plan and the phases go, and the next landing may deploy again.
    private static void EndLanding(Vehicle craft, VehicleAutopilotState state)
    {
        VehicleControlOwnership.Release(craft, ControlClaimant.Guidance);
        state.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Idle;
        state.ThreeDofPlan = null;
        state.LandingPhase = GuidanceWindow.LandingPhase.Idle;
        GuidanceWindow.StepLandingLegs(craft, state);
    }

    private static void Retract(Vehicle craft)
    {
        foreach (KeyframeAnimationModule anim in craft.Parts.Modules.Get<KeyframeAnimationModule>())
        {
            if (KsaLandingLegs.IsLeg(anim))
                anim.TimeGoal = 0f;
        }
    }

    // The fall time of the legs from the craft's own state vectors, as an oracle for the clock the landing reads.
    private static double FallSeconds(Vehicle craft, double legsM)
    {
        double3 r = craft.Orbit.StateVectors.PositionCci;
        double3 v = craft.Orbit.StateVectors.VelocityCci;
        double radius = r.Length();
        double sink = -double3.Dot(v, r) / radius;
        double g = craft.Orbit.Parent.Mu / (radius * radius);
        return LegsRules.FallSeconds(legsM, sink, g);
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
