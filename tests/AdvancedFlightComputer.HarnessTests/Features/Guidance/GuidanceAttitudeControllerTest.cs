using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTorque;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// AFC's own attitude controller (AttitudeController, entry guidance's glide) against the stock flight computer, on the same
// craft in the same air: the player's 2stage_KSAero dropped engine first at 25 km, 200 m/s down, ordered to hold 3 deg off the
// vertical with its roll held. Each controller flies a fresh copy for TrialS; the stock one through CommandAttitude as entry
// guidance points it, AFC's through FlyOwnAttitude. Both are scored on the same counts: the attitude error, the RCS propellant
// fired (KsaRcsTorqueControl.FiredFlow, whoever commands it), and how often the fired pitch and yaw torque changes sign - the
// "RCS firing in all directions" of 2026-11-20. With KSAero loaded the craft's fins fly a third trial.
public sealed class GuidanceAttitudeControllerTest : AfcTest
{
    public override string Name => "afc-guidance-attitude-controller";

    private const string Save = "2stage_KSAero";
    private const double StepS = 0.05;
    private const double TrialS = 15.0;
    private const double SettleS = 6.0;
    private const double StartAltM = 25_000.0, StartSinkMs = 200.0;
    private const double TiltDeg = 3.0;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private readonly record struct Score(string Name, double MeanErrDeg, double MaxErrDeg, double RcsKg, double HoldKg, int Reversals, int HoldReversals, double DeadbandShare, double SurfaceShare);

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }
        using KsaeroScope ksaero = KsaeroScope.Apply();
        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? previousFocus = Program.ControlledVehicle;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.attitudecontroller");
        var scores = new List<Score>();
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            // The guidance step hook, installed as every guidance test does, but its step skipped (SkipGuidanceStep): the test state is no engaged mode, so a step would release the craft. Installed all the same because a test that steps the sim long with PrepareWorker unpatched lets the JIT inline it into its caller, and a later test's patch then never runs (the booster landing test after this one, 2026-11-20).
            GuidanceFeature.ApplyDriverPatches(harmony);
            harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), nameof(GuidanceWindow.ApplyAutopilot)),
                prefix: new HarmonyMethod(typeof(GuidanceAttitudeControllerTest), nameof(SkipGuidanceStep)));
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            if (Trial(t, home, "stock flight computer", own: false, fins: false, preexisting) is not { } stock)
                return;
            scores.Add(stock);
            if (Trial(t, home, "AFC, RCS only", own: true, fins: false, preexisting) is not { } ownRcs)
                return;
            scores.Add(ownRcs);
            if (ksaero.Active && Trial(t, home, "AFC, fins and RCS", own: true, fins: true, preexisting) is { } ownFins)
                scores.Add(ownFins);

            foreach (Score s in scores)
                t.Info($"{s.Name}: error after {SettleS:F0} s mean {s.MeanErrDeg:F2} max {s.MaxErrDeg:F2} deg, RCS {s.RcsKg:F2} kg, " +
                       $"of which {s.HoldKg:F2} kg holding after {SettleS:F0} s, {s.Reversals} pitch/yaw reversals ({s.HoldReversals} holding), deadband {100 * s.DeadbandShare:F0} %, surfaces gave {100 * s.SurfaceShare:F0} % of the torque");
            t.Check($"AFC holds the tilt: mean {ownRcs.MeanErrDeg:F2} deg, worst {ownRcs.MaxErrDeg:F2} deg", ownRcs.MeanErrDeg < 1.0 && ownRcs.MaxErrDeg < 2.5);
            t.Check($"AFC fires less RCS than stock: {ownRcs.RcsKg:F2} against {stock.RcsKg:F2} kg", ownRcs.RcsKg < stock.RcsKg);
            t.Check($"AFC fires less RCS than stock holding the tilt: {ownRcs.HoldKg:F2} against {stock.HoldKg:F2} kg", ownRcs.HoldKg < stock.HoldKg);
            if (scores.Count == 3)
                t.Check($"the fins take RCS work: {scores[2].RcsKg:F2} against {ownRcs.RcsKg:F2} kg", scores[2].RcsKg < ownRcs.RcsKg);
            else
                t.Info($"no fin trial: KSAero is not loaded ({ksaero.Status}).");
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            AmbientState() = previousAmbient;
            GuidanceWindow.SetModActive(previousModActive);
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            PhysicsBubble._forceOffRails = railsBefore;
            Program.ControlledVehicle = previousFocus;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static Score? Trial(TestContext t, IParentBody home, string name, bool own, bool fins, HashSet<string> preexisting)
    {
        string only = Environment.GetEnvironmentVariable("AFC_ATT_TRIALS") ?? "";
        if (only.Length > 0 && !only.Contains(own ? fins ? "fins" : "own" : "stock", StringComparison.Ordinal))
            return new Score(name, 0, 0, 0, 0, 0, 0, 0, 0);
        SimDriver driver = t.Session.CreateDriver();
        // Engine first: body +X (the thrust axis) up, falling. The spawn puts the body axes on the planet-fixed axes (identity body to CCE), so the craft is placed over planet-fixed +X.
        double3 up = double3.Normalize(double3.UnitX.Transform(doubleQuat.Inverse(home.GetCci2Cce())));
        double3 side = double3.Cross(double3.UnitZ, up);
        double3 east = double3.Normalize(side.Length() > 1e-6 ? side : double3.Cross(double3.UnitY, up));
        double3 r0 = up * (home.MeanRadius + StartAltM);
        double3 v0 = -StartSinkMs * up + double3.Cross(home.GetAngularVelocityCci(), r0);
        Orbit orbit = Orbit.CreateFromStateCci(home, driver.Elapsed, r0, v0, VehicleSpawner.OrbitLineColor);
        Vehicle craft;
        try
        {
            craft = VehicleSpawner.SpawnFromSave(Save, t.System, home, "HarnessAttitude" + (own ? fins ? "Fins" : "Own" : "Stock"), orbit);
        }
        catch (InvalidOperationException e)
        {
            t.Skip($"{Save}: {e.Message}");
            return null;
        }
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        try
        {
            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = craft;
            AmbientState() = state;
            driver.Step(StepS, 2);

            if (!KsaAeroSweep.TryBuild(craft, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result aero, out string error))
            {
                t.Check($"{name}: the craft is swept", false, error);
                return null;
            }
            if (!fins)
                aero.Surfaces = [];
            state.Aero = aero;
            // AFC_ATT_DEADBAND sets the controller's deadband, degrees (VehicleAutopilotState.AttitudeDeadbandDeg).
            if (double.TryParse(Environment.GetEnvironmentVariable("AFC_ATT_DEADBAND"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double deadbandDeg))
                state.AttitudeDeadbandDeg = deadbandDeg;

            // The order: the nose 3 deg off the vertical toward east, wings level to it, held still.
            double tilt = TiltDeg * Math.PI / 180.0;
            double3 Dir() => double3.Normalize(Math.Cos(tilt) * double3.Normalize(craft.Orbit.StateVectors.PositionCci) + Math.Sin(tilt) * east);
            // Wings (body +Y) held where the spawn put them: only the tilt is ordered.
            double3 rollRef = double3.UnitY.Transform(KsaFrameBridge.BodyToCci(craft));
            craft.FlightComputer.RCSMode = FlightComputerRCSMode.Enabled;
            double3 noseAtStart = KsaFrameBridge.BodyNoseCci(craft);
            t.Info($"{name}: nose {Math.Acos(Math.Clamp(double3.Dot(noseAtStart, up), -1.0, 1.0)) * 180.0 / Math.PI:F1} deg off the vertical at the start");
            MethodInfo fly = Method("FlyOwnAttitude");
            MethodInfo command = Method("CommandAttitude");

            double rcsKg = 0.0, holdKg = 0.0, errSum = 0.0, errMax = 0.0, deadband = 0.0, surfaceNm = 0.0, demandNm = 0.0;
            int reversals = 0, holdReversals = 0, samples = 0, ownSteps = 0, finReversals = 0;
            double[]? lastFin = null;
            int[] finDir = new int[8];
            var lastSign = new int[2];
            int steps = (int)Math.Round(TrialS / StepS);
            for (int i = 0; i < steps; i++)
            {
                double now = Universe.GetElapsedSeconds();
                state.CommandDir = Dir();
                state.CommandRate = double3.Zero;
                GuidanceWindow.UpdateMomentLimiter(craft, now);
                if (own)
                    fly.Invoke(null, [craft, home, rollRef]);
                else
                    command.Invoke(null, [craft, home, state.CommandDir, i == 0, rollRef, FlightComputerRollMode.Up]);
                driver.Step(StepS, 1);

                double dt = craft.KinematicMeasurements.DeltaTime;
                double sub = dt > 0.0 && dt < 1.0 ? dt : StepS;
                bool holding = (i + 1) * StepS >= SettleS;
                if (KsaRcsTorqueControl.FiredFlow(craft, sub, out double flow, out _))
                {
                    rcsKg += flow * StepS;
                    if (holding)
                        holdKg += flow * StepS;
                }
                KsaFrameBridge.BodyAxes(craft, out double3 mx, out double3 my, out _);
                double3[] axes = [mx, my];
                for (int a = 0; a < 2; a++)
                    if (KsaRcsTorqueControl.FiredTorqueAlong(craft, axes[a], sub, out double fired, out _, out _) && Math.Abs(fired) > 1.0)
                    {
                        int sign = Math.Sign(fired);
                        if (lastSign[a] != 0 && sign != lastSign[a])
                        {
                            reversals++;
                            if (holding)
                                holdReversals++;
                        }
                        lastSign[a] = sign;
                    }
                if (own)
                {
                    AttitudeController.Report rep = AttitudeController.Last(craft);
                    ownSteps++;
                    if (rep.InDeadband)
                        deadband++;
                    surfaceNm += rep.SurfacesNm.Length();
                    // Fin chatter: how often a fin's commanded deflection turns back, moves under 0.05 deg ignored.
                    if (rep.FinCommandDeg is { } fin)
                    {
                        if (lastFin != null && lastFin.Length == fin.Length)
                            for (int f = 0; f < Math.Min(fin.Length, finDir.Length); f++)
                            {
                                double move = fin[f] - lastFin[f];
                                if (Math.Abs(move) < 0.05)
                                    continue;
                                int dir = Math.Sign(move);
                                if (finDir[f] != 0 && dir != finDir[f])
                                    finReversals++;
                                finDir[f] = dir;
                            }
                        lastFin = (double[])fin.Clone();
                    }
                    demandNm += rep.DemandNm.Length();
                }
                if (i == 10 || i == 100)
                {
                    FlightComputer fc = craft.FlightComputer;
                    AttitudeController.Report rep = AttitudeController.Last(craft);
                    KsaRcsTorqueControl.Report rr = KsaRcsTorqueControl.Last(craft);
                    t.Info($"{name} @ {(i + 1) * StepS:F1} s: acquired {state.ControlAcquired}, own {state.OwnAttitudeFlying}, mode {fc.AttitudeMode}, RCS {fc.RCSMode}, stock RCS authority {fc.RcsTorqueAuthority}, " +
                           $"err {rep.ErrorDeg:F1} deg, demand {rep.DemandNm}, air {rep.AirNm} ({rep.AirKnown}), rcs asked {rep.RcsAskedNm} given {rep.RcsNm} flow {rep.RcsFlowKgS:F3}; rcs report active {rr.Active} firing {rr.Firing}/{rr.Thrusters} short {rr.ShortfallNm:F0}");
                }
                double t1 = (i + 1) * StepS;
                if (t1 >= SettleS)
                {
                    double3 nose = KsaFrameBridge.BodyNoseCci(craft);
                    double err = Math.Acos(Math.Clamp(double3.Dot(double3.Normalize(nose), Dir()), -1.0, 1.0)) * 180.0 / Math.PI;
                    errSum += err;
                    errMax = Math.Max(errMax, err);
                    samples++;
                }
            }
            double alt = craft.Orbit.StateVectors.PositionCci.Length() - home.MeanRadius;
            if (fins)
                t.Info($"{name}: fin commands turned back {finReversals} times in {TrialS:F0} s");
            t.Info($"{name}: ended at {alt:F0} m, {craft.Orbit.StateVectors.VelocityCci.Length():F0} m/s (inertial); moment limiter {state.MomentLimiterPerPa * 1e3:F3} per kPa");
            return new Score(name, errSum / Math.Max(samples, 1), errMax, rcsKg, holdKg, reversals, holdReversals,
                ownSteps > 0 ? deadband / ownSteps : 0.0, demandNm > 0.0 ? surfaceNm / demandNm : 0.0);
        }
        finally
        {
            if (own)
                GuidanceWindow.StopOwnAttitude(craft);
            VehicleAutopilotState.Remove(craft);
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static bool SkipGuidanceStep() => false;

    private static MethodInfo Method(string name)
        => typeof(GuidanceWindow).GetMethod(name, BindingFlags.NonPublic | BindingFlags.Static)
           ?? throw new MissingMethodException(nameof(GuidanceWindow), name);
}
