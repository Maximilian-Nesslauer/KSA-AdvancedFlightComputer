#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Linq;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

// The convex ascent: launch3dof.py's minimum-propellant ascent, solved offline for this vehicle by SCvx and flown open loop in place of the vertical rise and gravity turn, with UPFG taking over for the terminal guidance once the vehicle is high enough.
//
// The commit asks for a plan when there is none that fits the launch, and launches - or arms for the window - once it converges; one that does not converge launches nothing, and the console offers the vertical rise and deg/s gravity turn as a backup. Calculate ascent asks for one without launching, to look at it first. The request is picked up by the vehicle's sim step, which is the one place the part tree can be read without racing the game's own recompute, and which builds the problem there as plain data: the staging model with each stage's thrust across back pressure, the drag table sampled off the vehicle, the body's atmosphere, and the target orbit. The solve then runs on its own thread (AscentPlanJob) and publishes an immutable AscentPlan, which the LAUNCH page draws and the commit flies.
//
// What the commit flies is pitch and azimuth against AIR-RELATIVE SPEED (see ConvexAscentProfile), not the plan's clock, until the vehicle is above the hand-over altitude - or the plan's own speed runs out - and UPFG has converged; from there it is the ordinary UPFG ascent.
public static partial class GuidanceWindow
{
    /// <summary>Back pressures each stage's thrust is sampled at, evenly from vacuum to the body's sea level. Fine enough that the spline follows KSA's flow-separation knee.</summary>
    private const int ThrustPressureSamples = 17;

    /// <summary>A vehicle that has moved this far over the ground since the plan was solved is no longer where the plan starts, m.</summary>
    private const double PlanStartToleranceM = 500.0;

    /// <summary>Nor one whose mass has changed by more than this fraction.</summary>
    private const double PlanMassTolerance = 0.01;

    /// <summary>Stages the first attempt plans: the fewest whose ideal dV reaches this multiple of the insertion speed.</summary>
    private const double InitialStagesDvMargin = 1.1;

    /// <summary>The script's throttle floor, and the one a stage that cannot throttle is held at. Never 100 %: see AscentSettings.ThrottleMin.</summary>
    private const double FullThrottleFloor = 0.99;

    /// <summary>How long the command takes to turn from the profile's attitude to UPFG's after the hand-over, s.</summary>
    private const double ConvexBlendSeconds = 15.0;

    /// <summary>A plan flown this far from the lift-off instant it was solved for starts from a pad the body has turned under the target plane, s. A minute is a quarter of a degree of spin, which UPFG steers out after the hand-over; a calculation at 1x takes seconds.</summary>
    private const double PlanLaunchToleranceS = 60.0;

    /// <summary>
    /// How far either side of the launch window a plan with a free launch time may move its lift-off, s (VehicleAutopilotState.ConvexOptimiseTiming); for a free LAN, the same angle of the body's spin. Ten minutes brackets every best launch time measured so far: two to seven minutes ahead of the pad passing under the plane, against LanLeadSeconds' fixed three.
    /// </summary>
    private const double ConvexTimingFreedomS = 600.0;

    /// <summary>The earliest a free launch time may lift off, s from when the plan is posed: time to solve it and arm before it arrives.</summary>
    private const double ConvexTimingEarliestS = 60.0;

    /// <summary>The game destroys a vehicle at this dynamic pressure, Pa (PhysicsBubble.DetectStructuralFailure). It is compiled into the mod.</summary>
    private const double BreakupQ = VehicleStructuralLimits.MaxDynamicPressure;

    /// <summary>No flown plan or relaxed retry may reach this share of <see cref="BreakupQ"/>, because plan q is sampled at nodes and the flight does not follow the plan exactly.</summary>
    private const double BreakupQMargin = 0.98;

    private static double QPlanHighest => AscentMaxQ.Highest(BreakupQMargin * BreakupQ);

    // --- request and collection, from the sim step ---------------------------

    /// <summary>Picks up a Calculate request and collects a finished solve. Runs from the vehicle's sim step for the focused craft; a fault costs the plan, never the step.</summary>
    private static void StepAscentPlanner(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        try
        {
            StepAscentPlannerCore(vehicle, orbit, parent);
        }
        catch (Exception e)
        {
            _s.AscentPlanRequested = false;
            _s.AscentPlanStatus = "PLANNER FAULT: " + e.Message;
            LogHelper.WarnOnce($"guidance-ascent-planner-{vehicle.Id}:{e.GetType().Name}",
                $"[AFC] Convex ascent planner failed on '{vehicle.Id}': {e}");
        }
    }

    private static void StepAscentPlannerCore(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        AscentPlanJob job = _s.AscentPlanJob;
        if (job != null && job.IsDone)
        {
            _s.AscentPlanJob = null;
            AscentPlan plan = job.Result;
            string failure;
            if (plan != null)
            {
                _s.AscentPlan = plan;
                // The booster reserve is the plan's: re-read it now rather than at the next 4 Hz tick, so a launch straight from the commit stages on it from the start.
                _s.StageModelDirty = true;
                AscentSolution sol = plan.Solution;
                // A chosen LAN is the target now: the LAUNCH page shows it, UPFG flies it after the hand-over, and the plan's own target check compares against it.
                if (plan.Usable && plan.LanChosen)
                    _s.LanDeg = plan.LanDeg;
                string booster = plan.Usable && double.IsFinite(sol.RecoveryReserveKg)
                    ? FormattableString.Invariant($" BOOSTER RESERVE {sol.RecoveryReserveKg / 1000.0:F2} t: BOOSTBACK {sol.BoostbackDv:F0} m/s, LANDING {sol.LandingDv:F0} m/s.")
                    : "";
                string timing = !plan.Usable ? ""
                    : plan.LaunchDelayS != 0.0 ? FormattableString.Invariant($" LIFTOFF {Math.Abs(plan.LaunchDelayS):F0} s {(plan.LaunchDelayS > 0.0 ? "AFTER" : "BEFORE")} WINDOW.")
                    : plan.LanChosen ? FormattableString.Invariant($" LAN {plan.LanDeg:F2} deg.") : "";
                // The commonest reason there is no plan: a stack with the thrust to spare cannot climb through the thick air under the q limit at full throttle, and the problem - which keeps the throttle within 1 % of full, as the script does - has no feasible trajectory. The seed search measured how low it can get.
                string hint = plan.Usable ? "" : ConvexQHint(plan);
                string relaxed = plan.QRelaxed
                    ? FormattableString.Invariant($" FIRST SOLVE DID NOT CONVERGE: MAX Q {plan.QMaxRequestedKpa:F0} kPa OUT OF REACH. RETRIED WITH LOOSER LIMITS, MAX Q {plan.QMaxKpa:F0} kPa.")
                    : "";
                string refused = sol.Usable && plan.OverQCeiling
                    ? FormattableString.Invariant($"PEAK Q {plan.PeakQKpa:F1} kPa, LIMIT {plan.QCeilingKpa:F0} kPa (BREAKUP {BreakupQ / 1000.0:F0} kPa). LOWER MAX Q.")
                    : "";
                _s.AscentPlanStatus = plan.Usable
                    ? FormattableString.Invariant($"PLAN READY: {sol.FinalMass / 1000.0:F2} t TO ORBIT, {plan.StagesPlanned} OF {plan.StagesAvailable} STAGES, {plan.WallSeconds:F1} s.{timing}{booster}{relaxed}")
                    : refused.Length > 0 ? "PLAN NOT FLOWN: " + refused
                    : $"NO SOLUTION: {sol.Message}.{relaxed}{hint}";
                // More than one solve means a looser retry also failed. Notes are not counted, because one can say that no retry was possible.
                bool retried = job.Attempts > 1;
                failure = plan.Usable ? ""
                    : refused.Length > 0 ? "PLAN NOT FLOWN: " + refused
                    : $"NO SOLUTION{(retried ? ", even after retrying with looser constraints" : "")}: {sol.Message}.{relaxed}{hint}";
                GuidanceLog.Info(vehicle, plan.Usable
                    ? FormattableString.Invariant($"convex ascent planned: {sol.FinalMass / 1000.0:F2} t to orbit, {plan.StagesPlanned} of {plan.StagesAvailable} stages, {plan.WallSeconds:F1} s.")
                    : refused.Length > 0 ? FormattableString.Invariant($"convex ascent refused: peak q {plan.PeakQKpa:F1} kPa.")
                    : $"convex ascent did not converge: {GuidanceLog.Cut(sol.Message, 37)}.");
                if (GuidanceLog.Enabled)
                {
                    GuidanceLog.DebugWrapped(vehicle, "convex solver: ", sol.Message);
                    GuidanceLog.DebugWrapped(vehicle, "convex detail: ", FormattableString.Invariant(
                        $"{sol.Iterations} iterations, {sol.Accepted} accepted, {plan.WallSeconds:F1} s, kick {sol.KickDeg:F3} deg"));
                    if (sol.Nodes > 0)
                    {
                        GuidanceLog.DebugWrapped(vehicle, "convex detail: ", FormattableString.Invariant(
                            $"{sol.FinalMass / 1000.0:F2} t, burns {string.Join("/", Array.ConvertAll(sol.BurnTime, b => FormattableString.Invariant($"{b:F0}")))} s"));
                        GuidanceLog.DebugWrapped(vehicle, "convex detail: ", FormattableString.Invariant(
                            $"max q {Max(sol.DynamicPressure) / 1000.0:F1} kPa, max q-alpha {Max(sol.QAlpha):F0} Pa rad"));
                        GuidanceLog.DebugWrapped(vehicle, "convex detail: ", FormattableString.Invariant(
                            $"insertion miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s"));
                    }
                    if (timing.Length > 0)
                        GuidanceLog.DebugWrapped(vehicle, "convex timing: ", timing.Trim());
                    if (booster.Length > 0)
                        GuidanceLog.DebugWrapped(vehicle, "convex booster: ", booster.Trim());
                    foreach (string note in plan.Notes)
                        GuidanceLog.DebugWrapped(vehicle, "convex note: ", note);
                }
            }
            else
            {
                string[] notes = job.Notes;
                _s.AscentPlanStatus = notes.Length > 0 ? "NO PLAN: " + notes[^1] + "." : "NO PLAN.";
                failure = "NO SOLUTION: " + (notes.Length > 0 ? notes[^1] + "." : "SOLVE ENDED WITHOUT PLAN.");
                GuidanceLog.Info(vehicle, "convex ascent calculation ended without a plan.");
                if (GuidanceLog.Enabled)
                {
                    foreach (string note in notes)
                        GuidanceLog.DebugWrapped(vehicle, "convex note: ", note);
                }
            }
            if (_s.ConvexLaunchPending)
                ContinueConvexLaunch(vehicle, orbit, parent, plan, failure);
        }

        if (!_s.AscentPlanRequested)
            return;
        _s.AscentPlanRequested = false;
        if (_s.AscentPlanJob != null)
            return;

        if (TryStartAscentPlan(vehicle, orbit, parent, out AscentPlanJob started, out string error))
        {
            _s.AscentPlanJob = started;
            _s.AscentPlanStatus = "SOLVING...";
        }
        else
        {
            _s.AscentPlanStatus = "NO SOLUTION: " + error + ".";
            GuidanceLog.Info(vehicle, $"convex ascent not calculated: {GuidanceLog.Cut(error, 39)}.");
            if (GuidanceLog.Enabled && error.Length > 39)
                GuidanceLog.DebugWrapped(vehicle, "convex error: ", error);
            if (_s.ConvexLaunchPending)
                FailConvexLaunch(vehicle, "NO SOLUTION (CLIMB): " + error + ".");
        }
    }

    // The commit: calculate, then launch.

    /// <summary>The launch window's instant when there is a target to wait for, now otherwise: the lift-off a plan is posed for.</summary>
    private static double WindowInstant()
        => _s.TargetId.Length > 0 && !double.IsNaN(_s.LaunchTargetTime) ? Math.Max(_s.LaunchTargetTime, SimNow()) : SimNow();

    /// <summary>
    /// The lift-off the commit would launch at: the window, moved by the launch time the convex plan chose for it, if it has one for this window (AscentPlan.LaunchDelayS). The window itself stays as it is tracked, so the backup gravity turn, and a plan for another window, keep it.
    /// </summary>
    private static double ExecuteLaunchInstant()
    {
        double window = WindowInstant();
        AscentPlan p = _s.AscentPlan;
        if (_s.FlyConvexAscent && !_s.BackupAscent && p != null && p.Usable && p.LaunchDelayS != 0.0
            && Math.Abs(p.WindowAt - window) <= PlanLaunchToleranceS)
            return Math.Max(window + p.LaunchDelayS, SimNow());
        return window;
    }

    /// <summary>Ask the sim step for a plan whose lift-off is at <paramref name="launchAt"/>, sim time, or now for NaN. A solve already running is left to finish: its plan is judged when it lands.</summary>
    private static void RequestAscentPlan(double launchAt)
    {
        _s.AscentPlanLaunchAt = launchAt;
        if (_s.AscentPlanJob != null)
            return;
        _s.AscentPlanRequested = true;
        _s.AscentPlanStatus = "STARTING...";
    }

    /// <summary>
    /// The commit without a plan that fits the launch. The convex profile is the ascent and the gravity turn only its backup, so the commit solves a plan first and commits once it converges: starting the ascent, or arming it for the window, which the plan is then solved for. The vehicle is claimed now, as arming claims it, since the launch is committed.
    /// </summary>
    private static void RequestConvexLaunch(Vehicle vehicle, bool atWindow, string why)
    {
        ClaimVehicle(GuidanceMode.Ascent, vehicle);
        _s.ConvexLaunchPending = true;
        _s.ConvexLaunchAtWindow = atWindow;
        _s.ConvexLaunchRetried = false;
        _s.ConvexLaunchFailure = "";
        RequestAscentPlan(atWindow ? WindowInstant() : double.NaN);
        GuidanceLog.Info(vehicle, $"launch: calculating the convex ascent, then {(atWindow ? "arming for the window" : "launching")}.");
        if (GuidanceLog.Enabled)
            GuidanceLog.DebugWrapped(vehicle, "launch calculates because: ", why);
    }

    /// <summary>
    /// The plan the commit waited on has landed. Launch on it if it converged and still fits; a converged plan that no longer fits - solved under time warp, or across a change of target - is asked for once more; anything else fails the launch, and the console offers the backup.
    /// </summary>
    private static void ContinueConvexLaunch(Vehicle vehicle, Orbit orbit, IParentBody parent, AscentPlan plan, string failure)
    {
        _s.ConvexLaunchPending = false;
        if (plan == null || !plan.Usable)
        {
            FailConvexLaunch(vehicle, failure);
            return;
        }
        double launchAt = _s.ConvexLaunchAtWindow ? ExecuteLaunchInstant() : SimNow();
        if (!ConvexPlanFlyable(vehicle, orbit, parent, launchAt, out string why))
        {
            if (!_s.ConvexLaunchRetried)
            {
                _s.ConvexLaunchRetried = true;
                _s.ConvexLaunchPending = true;
                RequestAscentPlan(_s.ConvexLaunchAtWindow ? WindowInstant() : double.NaN);
                GuidanceLog.Info(vehicle, "launch: the convex plan does not fit the launch, calculating again.");
                if (GuidanceLog.Enabled)
                    GuidanceLog.DebugWrapped(vehicle, "launch plan misfit: ", why);
                return;
            }
            FailConvexLaunch(vehicle, $"SOLVED PLAN DOES NOT FIT LAUNCH: {why}.");
            return;
        }
        GuidanceLog.Info(vehicle, $"launch: convex plan ready, {(_s.ConvexLaunchAtWindow ? "arming for the launch window" : "launching")}.");
        LaunchAscent(vehicle, orbit, parent, _s.ConvexLaunchAtWindow);
    }

    /// <summary>Nothing launches: the console says why and offers the backup gravity turn, which is the player's call and never taken for them.</summary>
    private static void FailConvexLaunch(Vehicle vehicle, string failure)
    {
        _s.ConvexLaunchPending = false;
        _s.ConvexLaunchFailure = failure;
        _s.ConvexBackupChosen = false;
        GuidanceLog.Info(vehicle, "launch: not launching, no convex plan fits this launch.");
        if (GuidanceLog.Enabled)
            GuidanceLog.DebugWrapped(vehicle, "launch failure: ", failure);
    }

    /// <summary>The backup the failure offers: the vertical rise and deg/s gravity turn, flown to the same target, now or at the window as the commit asked.</summary>
    private static void LaunchBackupAscent(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        _s.ConvexLaunchFailure = "";
        _s.ConvexBackupChosen = false;
        _s.BackupAscent = true;
        GuidanceLog.Info(vehicle, "launching in the backup deg/s gravity turn, by the player's choice.");
        LaunchAscent(vehicle, orbit, parent, _s.ConvexLaunchAtWindow);
    }

    private static double Max(double[] v)
    {
        double m = double.NegativeInfinity;
        foreach (double x in v) m = Math.Max(m, x);
        return m;
    }

    /// <summary>
    /// Why a plan that is not fit to fly could not hold its max q, when every seed climbed through it; empty otherwise. When the vehicle needs more than the highest max q AFC plans to, the text names the break-up limit and warns about the backup gravity turn, which also climbs at full thrust or at the acceleration limit (Autopilot flies UPFG's throttle, which holds only the acceleration limit).
    /// </summary>
    private static string ConvexQHint(AscentPlan plan)
    {
        AscentSolution sol = plan.Solution;
        if (sol.Nodes == 0 || !AscentMaxQ.OutOfReach(sol, plan.QMaxKpa * 1000.0))
            return "";
        double seedKpa = sol.SeedLeastMaxQ / 1000.0;
        double highestKpa = AscentMaxQ.Highest(plan.QCeilingKpa * 1000.0) / 1000.0;
        if (seedKpa < highestKpa && plan.QMaxKpa < highestKpa - 1e-6)
            return FormattableString.Invariant($" STEEPEST CLIMB REACHES {seedKpa:F0} kPa. TRY MAX Q {seedKpa:F0} TO {highestKpa:F0} kPa.");
        return FormattableString.Invariant($" STEEPEST CLIMB REACHES {seedKpa:F0} kPa. NO PLAN ABOVE {highestKpa:F0} kPa (BREAKUP {BreakupQ / 1000.0:F0} kPa). GRAVITY TURN HOLDS NO MAX Q: SET A G-LIMIT.");
    }

    /// <summary>
    /// Build the problem from the live vehicle and start the solve. Everything the worker reads is copied here: the stage list, the drag table (immutable), the atmosphere (immutable), the lift-off state and the target.
    /// </summary>
    private static bool TryStartAscentPlan(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                           out AscentPlanJob job, out string error)
    {
        job = null;
        double mu = parent.Mu, bodyRadius = parent.MeanRadius, omega = parent.GetAngularVelocity();
        if (!(mu > 0.0) || !(bodyRadius > 0.0) || !double.IsFinite(omega))
        {
            error = "BODY INVALID";
            return false;
        }

        // Drag and air. The sweep is the game's own force function, whatever drag model is active, measured onto a table, and the atmosphere it mirrors; an airless body comes back with no atmosphere, which is not a failure.
        if (!KsaAeroSweep.TryBuild(vehicle, parent, SimNow(), out KsaAeroSweep.Result aero, out string aeroError))
        {
            error = "NO DRAG MODEL (" + aeroError + ")";
            return false;
        }
        AtmosphereModel atmosphere = aero.Atmosphere;
        AscentAtmosphere air = atmosphere != null
            ? new KsaAscentAtmosphere(atmosphere, aero.Table)
            : VacuumAscentAtmosphere.Instance;

        double[] grid = null;
        if (atmosphere != null)
        {
            grid = new double[ThrustPressureSamples];
            for (int g = 0; g < grid.Length; g++)
                grid[g] = atmosphere.SeaLevelPressure * g / (grid.Length - 1);
        }

        // The staging model, as RefreshStageModel builds it, with thrust sampled across back pressure, and each stage's engines so its solids can be told from its liquids.
        SequencePerformanceList performance = vehicle.Parts?.PerformanceSequences;
        if (performance == null || vehicle.Parts.SequenceList == null)
        {
            error = "NO STAGING SEQUENCE";
            return false;
        }
        float ambient = vehicle.PhysicsEnvironment.AtmosphericPressure;
        performance.RecomputeForFlight(ambient);
        var stageParts = new Dictionary<UpfgStage, HashSet<Part>>();
        UpfgVehicle model = KsaVehicleAdapter.Build(vehicle, ambient, grid, stageParts);
        if (model.Stages.Count == 0)
        {
            error = "NO STAGE WITH THRUST AND PROPELLANT";
            return false;
        }

        // The stages. Each of the drain model's phases is one, with its solid motors' burns sampled from the game's own model and added on outside the throttle (#73), and a core that would run dry before its boosters throttled down to outlast them where it can (#32). Stage 0 starts at the LIVE mass, as UPFG reconciles it; whatever the model drops between one burnout and the next ignition is the jettison. The booster reserve comes off the first liquid load, as the ascent itself will fly it.
        // The floor on the LAUNCH page, but never under what the engines can throttle to, and never at 100 %, which would freeze the thrust direction. A stage of solids alone, or one whose liquids are held at full thrust, keeps the script's 99 %.
        double floor = Math.Clamp(Math.Max(_s.ConvexThrottleMinPct / 100.0, vehicle.GetMinThrottle()), 0.01, FullThrottleFloor);
        double m0 = vehicle.TotalMass;
        double dragArea = aero.ReferenceArea;
        // The acceleration limit of the stack on the pad goes to every stage that can hold it. The plan is solved once and flies its own throttle, so an upper stage on a smaller stack is flown under the pad limit even where stock would let it push harder (see EffectiveAccelLimitG).
        double userGLimit = UserGLimitG;
        double structuralGLimit = KsaEnginePerf.StructuralAccelerationLimit(vehicle) / StandardGravity;
        double accelLimitG = Math.Min(userGLimit, structuralGLimit);
        KsaAscentStages.Gathered gathered = KsaAscentStages.Gather(vehicle, model, stageParts, grid);
        AscentPhasePlan.Result stagePlan;
        try
        {
            // No reserve going in: a booster with a target has its reserve sized below, from nothing, and there is no other kind (RefreshReserve).
            stagePlan = AscentPhasePlan.Build(gathered.Phases, gathered.Motors, grid, m0, dragArea, floor, FullThrottleFloor,
                                              0.0, accelLimitG * StandardGravity);
        }
        catch (ArgumentException e)
        {
            error = "STAGE MODEL INVALID (" + e.Message + ")";
            return false;
        }
        AscentStage[] allStages = stagePlan.Stages;
        int count = allStages.Length;
        var dvCum = new double[count];
        double dvSum = 0.0;
        for (int i = 0; i < count; i++)
        {
            AscentStage st = allStages[i];
            double start = stagePlan.StartMass[i];
            if (!(st.PropellantMass > 1.0) || !(start - st.PropellantMass > 0.0))
            {
                error = $"STAGE {i + 1}: NOTHING LEFT TO BURN";
                return false;
            }
            dvSum += st.ExhaustVelocity * Math.Log(start / (start - st.PropellantMass));
            dvCum[i] = dvSum;
        }

        // The target: the orbit on the LAUNCH page, inserted into at periapsis - or where the ellipse crosses the top of the atmosphere, if periapsis is inside it, the floor UPFG itself keeps.
        double rp = bodyRadius + Math.Min(_s.PeKm, _s.ApKm) * 1000.0;
        double ra = bodyRadius + Math.Max(_s.PeKm, _s.ApKm) * 1000.0;
        if (!(rp > bodyRadius))
        {
            error = "TARGET PE BELOW SURFACE";
            return false;
        }
        double rIns = Math.Min(Math.Max(rp, parent.GetAtmosphereRadius()), ra);
        double sma = 0.5 * (rp + ra);
        double ecc = (ra - rp) / (ra + rp);
        double vIns = Math.Sqrt(mu * (2.0 / rIns - 1.0 / sma));
        double hMom = Math.Sqrt(mu * sma * (1.0 - ecc * ecc));
        double cosG = Math.Clamp(hMom / (rIns * vIns), -1.0, 1.0);
        double radialRate = rIns * vIns * Math.Sqrt(Math.Max(0.0, 1.0 - cosG * cosG));
        double3 normal = UpfgTarget.OrbitNormal(UpfgTarget.DegToRad(_s.IncDeg), UpfgTarget.DegToRad(_s.LanDeg));

        // Lift-off. On the pad the air-relative velocity is zero, and the script starts its vehicle rising at 10 m/s, clear of the singularity that makes the angle of attack meaningless at rest.
        double3 rNow = orbit.StateVectors.PositionCci;
        double3 r = rNow;
        double3 v = orbit.StateVectors.VelocityCci;
        double3 spin = new double3(0, 0, omega);
        double3 up = double3.Normalize(r);
        bool atRest = (v - double3.Cross(spin, r)).Length() < 10.0;
        if (atRest)
            v = double3.Cross(spin, r) + 10.0 * up;

        // A launch armed for a window lifts off later, from the same pad carried round by the body's spin, and the plan starts there: its steering is fixed to the ground and the target plane is not. Only a vehicle at rest on the ground waits for a window.
        double now = SimNow();
        double launchAt = now;
        if (atRest && double.IsFinite(_s.AscentPlanLaunchAt) && _s.AscentPlanLaunchAt > now)
        {
            launchAt = _s.AscentPlanLaunchAt;
            double turn = omega * (launchAt - now);
            double c = Math.Cos(turn), sn = Math.Sin(turn);
            r = new double3(c * r.X - sn * r.Y, sn * r.X + c * r.Y, r.Z);
            v = new double3(c * v.X - sn * v.Y, sn * v.X + c * v.Y, v.Z);
        }

        // THE LAUNCH TIMING THE PLAN MAY CHOOSE (ConvexOptimiseTiming), as a turn of the target plane about the spin axis (AscentProblem.PlaneTurnMin): a lift-off t later is the plane turned by -omega t.
        //  - Waiting for a target's window: when, within ConvexTimingFreedomS of the window, and not before the plan can be armed. The plane is the target's.
        //  - No target: the LAN, within the same angle of spin, starting from the plane over the pad. The plane is only an inclination, so nothing waits.
        //  - Launching now to a target: nothing; its plane is fixed and the lift-off is now.
        bool freeTime = false, freeLan = false;
        double turnMin = 0.0, turnMax = 0.0, lanDeg = _s.LanDeg;
        if (_s.ConvexOptimiseTiming && omega > 0.0)
        {
            if (_s.TargetId.Length > 0 && launchAt > now)
            {
                double earliest = Math.Max(-ConvexTimingFreedomS, now + ConvexTimingEarliestS - launchAt);
                double latest = ConvexTimingFreedomS;
                if (earliest < latest)
                {
                    freeTime = true;
                    turnMin = -omega * latest;
                    turnMax = -omega * earliest;
                }
            }
            else if (_s.TargetId.Length == 0)
            {
                freeLan = true;
                lanDeg = LanOverhead(r, _s.IncDeg, parent);
                normal = UpfgTarget.OrbitNormal(UpfgTarget.DegToRad(_s.IncDeg), UpfgTarget.DegToRad(lanDeg));
                turnMin = -omega * ConvexTimingFreedomS;
                turnMax = omega * ConvexTimingFreedomS;
            }
        }

        double groundRadius = Math.Min(bodyRadius, r.Length() - 1.0);
        double qMax = ConvexQMaxKpaUsed() * 1000.0;
        double qCeiling = BreakupQMargin * BreakupQ;
        double qAlphaMax = ConvexQAlphaMaxUsed();
        double throttleRequestedPct = _s.ConvexThrottleMinPct;
        double[] r0 = [r.X, r.Y, r.Z], v0 = [v.X, v.Y, v.Z], n0 = [normal.X, normal.Y, normal.Z];

        // THE BOOSTER THE PLAN BRINGS HOME: a stage that can fly home has a landing target (BoostbackTarget). The plan sizes its reserve from nothing, for the boostback to that target and the landing, in the first stage that drops its liquid load whole (AscentPhasePlan.ReserveStage) - a core under strap-ons included, which is why the reserve need not be armed yet: it arms when that booster is next to separate. The site has turned with the body to the launch, like the pad.
        AscentRecovery recovery = null;
        string recoveryNote = "";
        ReturnableStage booster = BoostbackTarget();
        string recoverySite = RecoverySiteKey(booster);
        if (booster != null && stagePlan.ReserveStage < 0)
            recoveryNote = "NO BOOSTER RESERVE: SEPARATING STAGE HAS NO LIQUID LOAD OF ITS OWN";
        else if (booster != null)
        {
            if (aero.Table == null || aero.Atmosphere == null)
                recoveryNote = "NO BOOSTER RESERVE: NO AIR FOR RETURN";
            else
            {
                double latDeg = booster.TargetLatDeg;
                double lonDeg = booster.TargetLonDeg;
                double3 site = (SiteDirCcfAt(latDeg, lonDeg) * bodyRadius).Transform(parent.GetCcf2Cci());
                double turn = omega * (launchAt - now);
                double cs = Math.Cos(turn), sn = Math.Sin(turn);
                site = new double3(cs * site.X - sn * site.Y, sn * site.X + cs * site.Y, site.Z);
                AscentStage b = allStages[stagePlan.ReserveStage];
                recovery = new AscentRecovery
                {
                    Stage = stagePlan.ReserveStage,
                    InitialReserveKg = stagePlan.ReserveKeptKg,
                    BoosterDryKg = stagePlan.BoosterDryKg,
                    ExhaustVelocity = b.Thrust / b.MassFlow,
                    SiteCci = [site.X, site.Y, site.Z],
                    ReferenceArea = aero.ReferenceArea,
                    Drag = aero.Table,
                    Lift = aero.Lift,
                    Alpha = GlideNominalAlpha(aero),
                    Air = aero.Atmosphere,
                };
                recoveryNote = FormattableString.Invariant($"BOOSTER RESERVE FROM PLAN: '{booster.Name}' ({stagePlan.BoosterDryKg / 1000.0:F2} t dry) HOME TO lat {latDeg:F3} lon {lonDeg:F3}");
            }
        }

        AscentProblem Build(int stages, double qLimit, bool[] held)
        {
            AscentStage[] list = AscentPhasePlan.Truncate(allStages, stages);
            if (held != null)
                for (int i = 0; i < list.Length && i < held.Length; i++)
                    if (held[i])
                        list[i] = list[i].WithThrottleMin(FullThrottleFloor);
            return new AscentProblem
            {
                Mu = mu,
                BodyRadius = bodyRadius,
                Omega = omega,
                R0 = r0,
                V0 = v0,
                M0 = m0,
                Stages = list,
                Atmosphere = air,
                GroundRadius = groundRadius,
                TargetRadius = rIns,
                TargetSpeed = vIns,
                TargetRadialRate = radialRate,
                PlaneNormal = n0,
                QMax = qLimit,
                QAlphaMax = qAlphaMax,
                PlaneTurnMin = turnMin,
                PlaneTurnMax = turnMax,
                // Only while the booster is a lower stage: a stack planned on fewer stages carries it to the end.
                Recovery = recovery != null && recovery.Stage < stages - 1 ? recovery : null,
            };
        }

        int initial = count;
        for (int i = 0; i < count; i++)
            if (dvCum[i] >= InitialStagesDvMargin * vIns)
            {
                initial = i + 1;
                break;
            }

        // Everything the plan records about what it was for, captured now: the publisher runs on the worker and must not touch the game.
        string vehicleId = vehicle.Id;
        string bodyName = parent.Id;
        double solvedAt = launchAt;
        double3 startCcf = rNow.Transform(parent.GetCci2Ccf());
        double peKm = _s.PeKm, apKm = _s.ApKm, incDeg = _s.IncDeg;
        bool optimiseTiming = _s.ConvexOptimiseTiming;
        double qMaxKpa = qMax / 1000.0;
        double insAltKm = (rIns - bodyRadius) / 1000.0;

        AscentPlan Publish(AscentSolution solved, int stages, double qUsed, string[] notes, double wall)
        {
            // A chosen launch time lifts off delay later, from where the spin has carried the pad: the same plan turned by omega delay, which is -PlaneTurn, and so back in the target's own plane. A chosen LAN flies now, in the plane turned by PlaneTurn.
            double delay = freeTime && solved.Nodes > 0 ? -solved.PlaneTurn / omega : 0.0;
            AscentSolution sol = freeTime ? solved.RotatedAboutSpin(-solved.PlaneTurn) : solved;
            double lanPlanned = freeLan && solved.Nodes > 0 ? lanDeg + UpfgTarget.RadToDeg(solved.PlaneTurn) : lanDeg;
            lanPlanned = (lanPlanned % 360.0 + 360.0) % 360.0;
            return new AscentPlan
            {
            Solution = sol,
            Profile = sol.Nodes > 0 ? ConvexAscentProfile.FromSolution(sol, omega, bodyRadius) : null,
            Series = AscentPlanSeries.Build(sol, omega, bodyRadius, qUsed, qAlphaMax),
            VehicleId = vehicleId,
            BodyName = bodyName,
            Body = parent,
            Omega = omega,
            BodyRadius = bodyRadius,
            SolvedAt = solvedAt + delay,
            WindowAt = solvedAt,
            LaunchDelayS = delay,
            LanChosen = freeLan,
            OptimisedTiming = optimiseTiming,
            RecoverySite = recoverySite,
            Mass0 = m0,
            StartCcf = startCcf,
            PeKm = peKm,
            ApKm = apKm,
            IncDeg = incDeg,
            LanDeg = lanPlanned,
            ThrottleMinPct = floor * 100.0,
            ThrottleMinRequestedPct = throttleRequestedPct,
            GLimitG = userGLimit,
            QCeilingKpa = qCeiling / 1000.0,
            SolidStages = allStages.Take(stages).Count(st => st.Solid != null),
            CoreThrottledDown = stagePlan.CoreThrottledDown,
            StageLines = stagePlan.Describe.Take(stages).Select((d, i) => d + (allStages[i].IsPinned ? $", PINNED TO {allStages[i].FixedBurnTime:F1} s" : "")).ToArray(),
            StageNotes = stagePlan.Notes.ToArray(),
            QMaxKpa = qUsed / 1000.0,
            QMaxRequestedKpa = qMaxKpa,
            QAlphaMax = qAlphaMax,
            InsertionAltKm = insAltKm,
            FinalMassFloor = Build(stages, qUsed, null).FinalMassFloor,
            StagesPlanned = stages,
            StagesAvailable = count,
            Notes = notes,
            WallSeconds = wall,
            };
        }

        // The whole problem, every stage, as JSON beside the temp files: the test console's --ascent-replay solves it again offline, which is how a plan that fails in the game gets diagnosed.
        string dump = "";
        try
        {
            string dir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "afc-convex-ascent");
            System.IO.Directory.CreateDirectory(dir);
            dump = System.IO.Path.Combine(dir, $"{string.Concat(vehicleId.Split(System.IO.Path.GetInvalidFileNameChars()))}.json");
            System.IO.File.WriteAllText(dump, AscentProblemFile.ToJson(Build(count, qMax, null)));
        }
        catch (Exception e)
        {
            dump = "not written: " + e.Message;
        }

        GuidanceLog.Info(vehicle, FormattableString.Invariant($"convex ascent requested: {count} stages, trying {initial} first."));
        if (GuidanceLog.Enabled)
        {
            const string head = "convex request: ";
            if (launchAt > now)
                GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant($"lift-off at the window, {launchAt - now:F0} s from now"));
            if (freeTime)
                GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant(
                    $"launch time free from {-turnMax / omega:+0;-0} to {-turnMin / omega:+0;-0} s of the window"));
            if (freeLan)
                GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant(
                    $"LAN free {UpfgTarget.RadToDeg(turnMin):+0.00;-0.00} to {UpfgTarget.RadToDeg(turnMax):+0.00;-0.00} deg of the plane over the pad"));
            if (recoveryNote.Length > 0)
                GuidanceLog.DebugWrapped(vehicle, head, recoveryNote);
            GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant(
                $"{m0 / 1000.0:F1} t, target {(rIns - bodyRadius) / 1000.0:F0} km at {vIns:F0} m/s"));
            GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant($"inc {_s.IncDeg:F2} deg, LAN {lanDeg:F2} deg"));
            GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant($"q {qMaxKpa:F0} kPa, q-alpha {qAlphaMax:F0} Pa rad"));
            GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant(
                $"throttle floor {floor * 100.0:F0} %, by stage {string.Join("/", allStages.Select(st => FormattableString.Invariant($"{st.ThrottleMin * 100.0:F0}")))} %"));
            GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant(
                $"accel limit {accelLimitG:F2} g on {allStages.Count(st => double.IsFinite(st.AccelerationLimit))} stages, structure {structuralGLimit:F2} g"));
            GuidanceLog.DebugWrapped(vehicle, head, double.IsFinite(userGLimit)
                ? FormattableString.Invariant($"g-limit {userGLimit:F2} g, drag area {dragArea:F1} m^2")
                : FormattableString.Invariant($"g-limit off, drag area {dragArea:F1} m^2"));
            GuidanceLog.DebugWrapped(vehicle, head, atmosphere != null
                ? FormattableString.Invariant($"air rho0 {atmosphere.SeaLevelDensity:G4} kg/m^3, P0 {atmosphere.SeaLevelPressure:G6} Pa")
                : "no air");
            if (atmosphere != null)
                GuidanceLog.DebugWrapped(vehicle, head, FormattableString.Invariant(
                    $"air H0 {atmosphere.SeaLevelScaleHeight / 1000.0:F2} km, top {atmosphere.TopAltitude / 1000.0:F1} km, a {atmosphere.SpeedOfSound:F1} m/s"));
            GuidanceLog.DebugWrapped(vehicle, "convex problem: ", dump);
            for (int i = 0; i < count; i++)
                GuidanceLog.DebugWrapped(vehicle, FormattableString.Invariant($"convex stage {i + 1}: "), stagePlan.Describe[i]
                    + (allStages[i].IsPinned ? FormattableString.Invariant($", pinned to {allStages[i].FixedBurnTime:F1} s") : "")
                    + (allStages[i].JettisonMass > 0.0 ? FormattableString.Invariant($", drops {allStages[i].JettisonMass / 1000.0:F2} t") : ""));
            for (int m = 0; m < gathered.Motors.Count; m++)
                GuidanceLog.DebugWrapped(vehicle, "convex solid: ", FormattableString.Invariant(
                    $"'{gathered.Motors[m].Name}' burns {gathered.Motors[m].BurnTime:F1} s in the game's model, its thrust profile {gathered.ProfileSeconds[m]:F1} s, flow {gathered.Motors[m].MassFlow[0]:F0} -> {gathered.Motors[m].MassFlow[^1]:F0} kg/s"));
            foreach (string note in stagePlan.Notes)
                GuidanceLog.DebugWrapped(vehicle, "convex note: ", note);
        }

        double handoverRadius = bodyRadius + _s.ConvexHandoverAltKm * 1000.0;
        job = new AscentPlanJob(Build, count, initial, qMax, Publish,
                                (problem, sol) => StagesUpfgFliesThrottled(problem, sol, handoverRadius), qCeiling);
        error = "";
        return true;
    }

    /// <summary>A planned throttle under this fraction of what the stage can give, full thrust or the acceleration limit, is one UPFG does not fly.</summary>
    private const double UpfgThrottleSlack = 0.9;

    /// <summary>
    /// The stages of a converged plan that light at or above the hand-over radius and that the plan throttles below what UPFG gives them, or null for none. Runs on the worker, on plain data.
    ///
    /// WHY THEY ARE HELD. Every stage that lights above the hand-over altitude burns wholly under UPFG, which flies full thrust and holds only the acceleration limit. Left free, the plan can throttle such a stage far down, a low-thrust arc on which a lofted climb coasts up to the insertion radius while the engines add the horizontal speed slowly. That arc is the plan's own optimum, but UPFG cannot fly it. From the lofted state it is handed, UPFG burns at full thrust, so it reaches orbital speed long before the plan would and has to pitch up and then far down to stop the climb at the insertion radius. Its steering is then tens of degrees off the plan at the hand-over, and the burn costs hundreds of m/s more than planned. Held at full thrust, the plan shapes its climb for the burn that will be flown.
    ///
    /// A stage that lights below the hand-over altitude keeps its throttle, because the plan flies it through the air. Only liquid stages that are free to throttle are candidates: solids follow their grain, and a pinned stage is already held.
    /// </summary>
    private static bool[] StagesUpfgFliesThrottled(AscentProblem problem, AscentSolution sol, double handoverRadius)
    {
        bool[] held = null;
        for (int i = 0; i < problem.Stages.Length; i++)
        {
            AscentStage st = problem.Stages[i];
            double floor = double.IsNaN(st.ThrottleMin) ? problem.Settings.ThrottleMin : st.ThrottleMin;
            if (st.Solid != null || !st.HasLiquid || st.IsPinned || floor >= FullThrottleFloor)
                continue;
            int first = Array.IndexOf(sol.NodeStage, i);
            if (first < 0 || NodeRadius(sol, first) < handoverRadius)
                continue;
            for (int k = first; k < sol.Nodes && sol.NodeStage[k] == i; k++)
            {
                double u = new double3(sol.Throttle[k * 3], sol.Throttle[k * 3 + 1], sol.Throttle[k * 3 + 2]).Length();
                double full = k < sol.FullThrust.Length ? sol.FullThrust[k] : 0.0;
                double most = double.IsFinite(st.AccelerationLimit) && full > 0.0
                    ? Math.Min(1.0, st.AccelerationLimit * sol.Mass[k] / full)
                    : 1.0;
                if (u < UpfgThrottleSlack * most)
                {
                    (held ??= new bool[problem.Stages.Length])[i] = true;
                    break;
                }
            }
        }
        return held;
    }

    private static double NodeRadius(AscentSolution sol, int k)
        => new double3(sol.Position[k * 3], sol.Position[k * 3 + 1], sol.Position[k * 3 + 2]).Length();

    // --- flying the plan ------------------------------------------------------

    /// <summary>
    /// Whether the stored plan may be flown on this vehicle as it is now. A plan belongs to the vehicle, place, mass, lift-off instant, target and limits it was solved for; the commit calculates a new one for a plan that has lost any of them, and the reason says why.
    /// </summary>
    /// <param name="launchAt">The lift-off it would be flown from, sim time: now for a launch, the window's instant for one armed to wait for it.</param>
    private static bool ConvexPlanFlyable(Vehicle vehicle, Orbit orbit, IParentBody parent, double launchAt, out string why)
    {
        AscentPlan p = _s.AscentPlan;
        if (p == null)
        {
            why = "NO PLAN";
            return false;
        }
        if (!p.Usable)
        {
            why = p.Solution?.Usable == true && p.OverQCeiling
                ? FormattableString.Invariant($"PEAK Q {p.PeakQKpa:F0} kPa, LIMIT {p.QCeilingKpa:F0} kPa (BREAKUP {BreakupQ / 1000.0:F0} kPa)")
                : "NO SOLUTION";
            return false;
        }
        if (p.VehicleId != vehicle.Id || !ReferenceEquals(p.Body, parent))
        {
            why = "PLANNED FOR ANOTHER CRAFT OR BODY";
            return false;
        }
        double3 ccf = orbit.StateVectors.PositionCci.Transform(parent.GetCci2Ccf());
        if ((ccf - p.StartCcf).Length() > PlanStartToleranceM)
        {
            why = "CRAFT MOVED";
            return false;
        }
        if (Math.Abs(vehicle.TotalMass - p.Mass0) > PlanMassTolerance * p.Mass0)
        {
            why = "MASS CHANGED";
            return false;
        }
        if (p.TargetDiffers(_s.PeKm, _s.ApKm, _s.IncDeg, _s.LanDeg))
        {
            why = "ORBIT CHANGED";
            return false;
        }
        if (p.SettingsDiffer(ConvexQMaxKpaUsed(), ConvexQAlphaMaxUsed(), _s.ConvexThrottleMinPct, UserGLimitG)
            || p.OptimisedTiming != _s.ConvexOptimiseTiming)
        {
            why = "LIMITS CHANGED";
            return false;
        }
        if (p.RecoverySite != RecoverySiteKey(BoostbackTarget()))
        {
            why = "BOOSTER LANDING TARGET CHANGED SINCE PLAN";
            return false;
        }
        double late = launchAt - p.SolvedAt;
        if (Math.Abs(late) > PlanLaunchToleranceS)
        {
            why = late > 0.0
                ? $"PLANNED FOR LIFTOFF {late:F0} s EARLIER"
                : $"PLANNED FOR LIFTOFF {-late:F0} s LATER";
            return false;
        }
        why = "";
        return true;
    }

    /// <summary>The max-q of the LAUNCH page as a plan is solved to it, floored away from zero and held at <see cref="QPlanHighest"/> at most, because a plan that needs more is not flown.</summary>
    private static double ConvexQMaxKpaUsed() => Math.Clamp(_s.ConvexQMaxKpa, 0.1, QPlanHighest / 1000.0);
    private static double ConvexQAlphaMaxUsed() => Math.Max(_s.ConvexQAlphaMax, 1.0);

    /// <summary>Air-relative speed: what the profile is indexed by.</summary>
    private static double AirSpeed(double3 r, double3 v, IParentBody parent)
        => (v - double3.Cross(new double3(0, 0, parent.GetAngularVelocity()), r)).Length();

    /// <summary>
    /// The open-loop phase's command: the plan's attitude where the vehicle is on the plan, and the rate that command is turning at. See <see cref="ConvexProfileDirection"/> for where it is on the plan; the rate is the command's own, differenced step to step, so it includes the local horizon turning under the vehicle as well as the programme itself.
    /// </summary>
    private static double3 ConvexProfileCommand(double3 r, double3 v, double mass, IParentBody parent, double stepDt,
                                                out double3 rate)
    {
        double3 want = ConvexProfileDirection(r, v, mass, parent);
        rate = DifferencedRate(ref _s.ConvexLastWant, want, stepDt);
        return want;
    }

    /// <summary>
    /// The plan's attitude where the vehicle is on the plan, moving the plan on first. Where it is on the plan is a plan time, carried from step to step and moved on by the vehicle's air speed within the stage its mass says it is burning (see ConvexAscentProfile.Advance): pitch against speed while the vehicle is gaining speed it has never had in that stage, the plan's own clock while it is not, and back in step with the plan at each separation. The clock runs on SIM time since the last step, not the slew's clamped step, so a warped step moves the plan as far as it moved the vehicle.
    /// </summary>
    private static double3 ConvexProfileDirection(double3 r, double3 v, double mass, IParentBody parent)
    {
        ConvexAscentProfile profile = _s.FlyingPlan?.Profile;
        if (profile == null)
            return double3.Normalize(r);
        double now = SimNow();
        double dt = double.IsFinite(_s.ConvexLastTime) ? Math.Max(0.0, now - _s.ConvexLastTime) : 0.0;
        _s.ConvexLastTime = now;
        _s.ConvexPlanTime = profile.Advance(_s.ConvexPlanTime, AirSpeed(r, v, parent), mass, dt,
            ref _s.ConvexStage, ref _s.ConvexStageSpeed, ref _s.ConvexFastest);
        return profile.Direction(r, _s.ConvexPlanTime);
    }

    /// <summary>The rotation rate that takes <paramref name="last"/> to <paramref name="want"/> in one step, rad/s about their common normal; and <paramref name="last"/> moved on to <paramref name="want"/>.</summary>
    private static double3 DifferencedRate(ref double3 last, double3 want, double stepDt)
    {
        double3 rate = default;
        if (last.Length() > 0.5 && stepDt > 1e-6)
        {
            double3 axis = double3.Cross(last, want);
            if (axis.Length() > 1e-12)
                rate = double3.Normalize(axis) * (AngleBetween(last, want) / stepDt);
        }
        last = want;
        return rate;
    }

    /// <summary>
    /// How far the command has turned from the profile's attitude to UPFG's since the hand-over: 0 at the hand-over, 1 once <see cref="ConvexBlendSeconds"/> have passed, and 1 when there is no blend. A smoothstep, so the turn starts and ends at zero rate rather than kinking the command twice.
    ///
    /// WHY BLEND. At the hand-over UPFG's steering and the plan's attitude differ by up to ten degrees - the plan was solved to insertion with the whole climb in view, UPFG from the state it is handed - and the command slew limit turned that difference into a five-degree-a-second snap, which reads as a jerk. Blended over fifteen seconds the same difference is under one degree a second, and UPFG, which re-solves every cycle regardless, loses only the fraction of a second of optimality the blend holds it off its own steering.
    /// </summary>
    private static double ConvexBlendWeight()
    {
        if (!double.IsFinite(_s.ConvexBlendStart) || _s.FlyingPlan?.Profile == null)
            return 1.0;
        double x = (SimNow() - _s.ConvexBlendStart) / ConvexBlendSeconds;
        if (x >= 1.0)
        {
            _s.ConvexBlendStart = double.NaN;
            return 1.0;
        }
        x = Math.Max(x, 0.0);
        return x * x * (3.0 - 2.0 * x);
    }

    /// <summary>The ClosedLoop command while the hand-over blend runs: the profile's attitude turned toward UPFG's by the blend weight, about their common normal.</summary>
    private static double3 ConvexBlendCommand(double3 r, double3 v, double mass, IParentBody parent, double3 upfgDir,
                                              double weight, double stepDt, out double3 rate)
    {
        double3 from = double3.Normalize(ConvexProfileDirection(r, v, mass, parent));
        double3 to = double3.Normalize(upfgDir);
        double3 axis = double3.Cross(from, to);
        double3 want = axis.Length() > 1e-12
            ? RotateAbout(from, double3.Normalize(axis), weight * AngleBetween(from, to))
            : to;
        rate = DifferencedRate(ref _s.ConvexLastWant, want, stepDt);
        return want;
    }

    /// <summary>
    /// The profile's throttle, as the engines take it. The plan's throttle is a fraction of the liquid engines' full thrust at the altitude flown, and thrust is not proportional to the throttle setting in air (see KsaEnginePerf.ThrustAtThrottle), so the demand is turned into a setting through the engines' own curve. Full throttle whenever the plan is within half a percent of it - which, at the script's 99 % floor, is all the time - and whenever the vehicle cannot throttle.
    ///
    /// The fraction is of the UNCAPPED full thrust, because the plan holds the acceleration limit itself. Taking it of the thrust KsaEnginePerf.StructuralThrottleCap leaves would apply the structural limit a second time.
    ///
    /// WHILE A SOLID BURNS the throttle is the liquid engines' alone: the plan adds the solids' thrust outside it (#73), and a core lit with its boosters may be planned well below full to outlast them (#32). The fraction is then of the liquids' full thrust, set through their curve alone.
    /// </summary>
    private static float ConvexThrottle(Vehicle vehicle, IParentBody parent)
    {
        ConvexAscentProfile profile = _s.FlyingPlan?.Profile;
        if (profile == null)
            return 1f;
        double fraction = profile.Throttle(_s.ConvexPlanTime);
        if (!(fraction < 0.995))
            return 1f;
        double altitude = vehicle.Orbit.StateVectors.PositionCci.Length() - parent.MeanRadius;
        double pressure = KsaEnginePerf.AmbientPressureAt(parent, altitude);
        double full = KsaEnginePerf.UncappedAtPressure(vehicle, pressure).thrust;
        if (!(full > 0.0))
            return 1f;
        KsaEnginePerf.ThrustCommand command = KsaEnginePerf.CommandForThrust(vehicle, fraction * full, pressure);
        if (command.Status == KsaEnginePerf.ThrustStatus.UnsupportedEngine)
            command = KsaEnginePerf.CommandLiquidFraction(vehicle, fraction, pressure);
        return command.Status is KsaEnginePerf.ThrustStatus.Available or KsaEnginePerf.ThrustStatus.BelowMinimum
                                 or KsaEnginePerf.ThrustStatus.AboveMaximum
            ? (float)command.Throttle
            : 1f;
    }

    /// <summary>
    /// Whether the open-loop profile is done: above the hand-over altitude with UPFG converged, or out of plan altogether. The convergence test is a guard, not a criterion: UPFG solves from lift-off, so by the hand-over altitude it has had minutes to converge, but flying an unconverged solution would steer on nothing.
    /// </summary>
    private static bool ConvexProfileDone(double3 r, double3 v, IParentBody parent, double bodyRadius, out string why)
    {
        ConvexAscentProfile profile = _s.FlyingPlan?.Profile;
        double alt = r.Length() - bodyRadius;
        if (profile == null)
        {
            why = "NO PROFILE TO FLY";
            return true;
        }
        if (_s.ConvexPlanTime >= profile.EndTime)
        {
            why = "PLAN RAN OUT";
            return true;
        }
        if (alt >= _s.ConvexHandoverAltKm * 1000.0 && _s.Upfg.Converged)
        {
            why = $"ABOVE {_s.ConvexHandoverAltKm:F0} km";
            return true;
        }
        why = "";
        return false;
    }
}
