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
// EXECUTE asks for a plan when there is none that fits the launch, and launches - or arms for the window - once it converges; one that does not converge launches nothing, and the panel offers the vertical rise and deg/s gravity turn as a backup. Calculate ascent asks for one without launching, to look at it first. The request is picked up by the vehicle's sim step, which is the one place the part tree can be read without racing the game's own recompute, and which builds the problem there as plain data: the staging model with each stage's thrust across back pressure, the drag table sampled off the vehicle, the body's atmosphere, and the target orbit. The solve then runs on its own thread (AscentPlanJob) and publishes an immutable AscentPlan, which the panel draws and EXECUTE flies.
//
// What EXECUTE flies is pitch and azimuth against AIR-RELATIVE SPEED (see ConvexAscentProfile), not the plan's clock, until the vehicle is above the hand-over altitude - or the plan's own speed runs out - and UPFG has converged; from there it is the ordinary UPFG ascent.
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
            _s.AscentPlanStatus = "Planner fault: " + e.Message;
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
                AscentSolution sol = plan.Solution;
                // The commonest reason there is no plan: a stack with the thrust to spare cannot climb through the thick air under the q limit at full throttle, and the problem - which keeps the throttle within 1 % of full, as the script does - has no feasible trajectory. The seed search measured how low it can get.
                string hint = !plan.Usable && sol.Nodes > 0 && double.IsFinite(sol.SeedLeastMaxQ)
                              && sol.SeedLeastMaxQ > 0.98 * plan.QMaxKpa * 1000.0
                    ? $" At full throttle even its steepest climb reaches {sol.SeedLeastMaxQ / 1000.0:F0} kPa: raise Max q above that."
                    : "";
                string relaxed = plan.QRelaxed
                    ? $" Max q {plan.QMaxRequestedKpa:F0} kPa is out of this vehicle's reach at full throttle, so it was planned to {plan.QMaxKpa:F0} kPa."
                    : "";
                _s.AscentPlanStatus = plan.Usable
                    ? $"Plan ready: {sol.FinalMass / 1000.0:F2} t to orbit, {plan.StagesPlanned} of {plan.StagesAvailable} stages, {plan.WallSeconds:F1} s.{relaxed}"
                    : $"No converged plan: {sol.Message}.{relaxed}{hint}";
                failure = plan.Usable ? "" : $"Convergence was not possible: {sol.Message}.{relaxed}{hint}";
                GuidanceLog.Info(vehicle, $"convex ascent {(plan.Usable ? "planned" : "did not converge")}: {sol.Message}, "
                    + $"{sol.Iterations} iterations ({sol.Accepted} accepted) in {plan.WallSeconds:F1} s, kick {sol.KickDeg:F3} deg, "
                    + (sol.Nodes > 0
                        ? $"{sol.FinalMass / 1000.0:F2} t to orbit, burns {string.Join("/", Array.ConvertAll(sol.BurnTime, b => b.ToString("F0")))} s, "
                          + $"max q {Max(sol.DynamicPressure) / 1000.0:F1} kPa, max q-alpha {Max(sol.QAlpha):F0} Pa rad, "
                          + $"insertion miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s; "
                        : "")
                    + string.Join("; ", plan.Notes) + ".");
            }
            else
            {
                string[] notes = job.Notes;
                _s.AscentPlanStatus = notes.Length > 0 ? "No plan: " + notes[^1] + "." : "No plan.";
                failure = "Convergence was not possible: " + (notes.Length > 0 ? notes[^1] + "." : "the calculation ended without a plan.");
                GuidanceLog.Info(vehicle, $"convex ascent calculation ended without a plan: {string.Join("; ", notes)}.");
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
            _s.AscentPlanStatus = "Calculating...";
        }
        else
        {
            _s.AscentPlanStatus = "Cannot calculate: " + error + ".";
            GuidanceLog.Info(vehicle, $"convex ascent not calculated: {error}.");
            if (_s.ConvexLaunchPending)
                FailConvexLaunch(vehicle, "The convex ascent cannot be calculated: " + error + ".");
        }
    }

    // --- EXECUTE: calculate, then launch -----------------------------------------

    /// <summary>The lift-off EXECUTE would commit to: the launch window's instant when there is a target to wait for, now otherwise.</summary>
    private static double ExecuteLaunchInstant()
        => _s.TargetId.Length > 0 && !double.IsNaN(_s.LaunchTargetTime) ? Math.Max(_s.LaunchTargetTime, SimNow()) : SimNow();

    /// <summary>Ask the sim step for a plan whose lift-off is at <paramref name="launchAt"/>, sim time, or now for NaN. A solve already running is left to finish: its plan is judged when it lands.</summary>
    private static void RequestAscentPlan(double launchAt)
    {
        _s.AscentPlanLaunchAt = launchAt;
        if (_s.AscentPlanJob != null)
            return;
        _s.AscentPlanRequested = true;
        _s.AscentPlanStatus = "Starting...";
    }

    /// <summary>
    /// EXECUTE without a plan that fits the launch. The convex profile is the ascent and the gravity turn only its backup, so EXECUTE solves a plan first and commits once it converges: starting the ascent, or arming it for the window, which the plan is then solved for. The vehicle is claimed now, as arming claims it, since the launch is committed.
    /// </summary>
    private static void RequestConvexLaunch(Vehicle vehicle, bool atWindow, string why)
    {
        ClaimVehicle(GuidanceMode.Ascent, vehicle);
        _s.ConvexLaunchPending = true;
        _s.ConvexLaunchAtWindow = atWindow;
        _s.ConvexLaunchRetried = false;
        _s.ConvexLaunchFailure = "";
        RequestAscentPlan(atWindow ? ExecuteLaunchInstant() : double.NaN);
        GuidanceLog.Info(vehicle, $"EXECUTE: calculating the convex ascent before {(atWindow ? "arming for the launch window" : "launching")} ({why}).");
    }

    /// <summary>
    /// The plan EXECUTE waited on has landed. Launch on it if it converged and still fits; a converged plan that no longer fits - solved under time warp, or across a change of target - is asked for once more; anything else fails the launch, and the panel offers the backup.
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
                RequestAscentPlan(_s.ConvexLaunchAtWindow ? launchAt : double.NaN);
                GuidanceLog.Info(vehicle, $"EXECUTE: the convex plan came back but does not fit the launch ({why}), calculating it again.");
                return;
            }
            FailConvexLaunch(vehicle, $"The convex plan does not fit the launch: {why}.");
            return;
        }
        GuidanceLog.Info(vehicle, $"EXECUTE: convex plan ready, {(_s.ConvexLaunchAtWindow ? "arming for the launch window" : "launching")}.");
        LaunchAscent(vehicle, orbit, parent, _s.ConvexLaunchAtWindow);
    }

    /// <summary>Nothing launches: the panel says why and offers the backup gravity turn, which is the player's call and never taken for them.</summary>
    private static void FailConvexLaunch(Vehicle vehicle, string failure)
    {
        _s.ConvexLaunchPending = false;
        _s.ConvexLaunchFailure = failure;
        GuidanceLog.Info(vehicle, $"EXECUTE: not launching. {failure}");
    }

    /// <summary>The backup the failure offers: the vertical rise and deg/s gravity turn, flown to the same target, now or at the window as EXECUTE asked.</summary>
    private static void LaunchBackupAscent(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        _s.ConvexLaunchFailure = "";
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
    /// Build the problem from the live vehicle and start the solve. Everything the worker reads is copied here: the stage list, the drag table (immutable), the atmosphere (immutable), the lift-off state and the target.
    /// </summary>
    private static bool TryStartAscentPlan(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                           out AscentPlanJob job, out string error)
    {
        job = null;
        double mu = parent.Mu, bodyRadius = parent.MeanRadius, omega = parent.GetAngularVelocity();
        if (!(mu > 0.0) || !(bodyRadius > 0.0) || !double.IsFinite(omega))
        {
            error = "the body is not valid";
            return false;
        }

        // Drag and air. The sweep is the game's own box model sampled onto a table, and the atmosphere it mirrors; an airless body comes back with no atmosphere, which is not a failure.
        if (!KsaAeroSweep.TryBuild(vehicle, parent, SimNow(), out KsaAeroSweep.Result aero, out string aeroError))
        {
            error = "no drag model (" + aeroError + ")";
            return false;
        }
        ExponentialAtmosphere atmosphere = aero.Atmosphere;
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
            error = "the vehicle has no staging sequence";
            return false;
        }
        float ambient = vehicle.PhysicsEnvironment.AtmosphericPressure;
        performance.RecomputeForFlight(ambient);
        var stageParts = new Dictionary<UpfgStage, HashSet<Part>>();
        UpfgVehicle model = KsaVehicleAdapter.Build(vehicle, ambient, grid, stageParts);
        if (model.Stages.Count == 0)
        {
            error = "no stage has thrust and propellant";
            return false;
        }

        // The stages. Each of the drain model's phases is one, with its solid motors' burns sampled from the game's own model and added on outside the throttle (#73), and a core that would run dry before its boosters throttled down to outlast them where it can (#32). Stage 0 starts at the LIVE mass, as UPFG reconciles it; whatever the model drops between one burnout and the next ignition is the jettison. The booster reserve comes off the first liquid load, as the ascent itself will fly it.
        // The floor on the panel, but never under what the engines can throttle to, and never at 100 %, which would freeze the thrust direction. A stage of solids alone, or one whose liquids are held at full thrust, keeps the script's 99 %.
        double floor = Math.Clamp(Math.Max(_s.ConvexThrottleMinPct / 100.0, vehicle.GetMinThrottle()), 0.01, FullThrottleFloor);
        double m0 = vehicle.TotalMass;
        double dragArea = aero.ReferenceArea;
        KsaAscentStages.Gathered gathered = KsaAscentStages.Gather(vehicle, model, stageParts, grid);
        AscentPhasePlan.Result stagePlan;
        try
        {
            stagePlan = AscentPhasePlan.Build(gathered.Phases, gathered.Motors, grid, m0, dragArea, floor, FullThrottleFloor,
                                              _s.ReserveArmed ? _s.ReserveKg : 0.0);
        }
        catch (ArgumentException e)
        {
            error = "the stage model is not valid (" + e.Message + ")";
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
                error = $"stage {i + 1} has nothing left to burn";
                return false;
            }
            dvSum += st.ExhaustVelocity * Math.Log(start / (start - st.PropellantMass));
            dvCum[i] = dvSum;
        }

        // The target: the orbit on the panel, inserted into at periapsis - or where the ellipse crosses the top of the atmosphere, if periapsis is inside it, the floor UPFG itself keeps.
        double rp = bodyRadius + Math.Min(_s.PeKm, _s.ApKm) * 1000.0;
        double ra = bodyRadius + Math.Max(_s.PeKm, _s.ApKm) * 1000.0;
        if (!(rp > bodyRadius))
        {
            error = "the target periapsis is below the surface";
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

        double groundRadius = Math.Min(bodyRadius, r.Length() - 1.0);
        double qMax = ConvexQMaxKpaUsed() * 1000.0;
        double qAlphaMax = ConvexQAlphaMaxUsed();
        double throttleRequestedPct = _s.ConvexThrottleMinPct;
        double[] r0 = [r.X, r.Y, r.Z], v0 = [v.X, v.Y, v.Z], n0 = [normal.X, normal.Y, normal.Z];

        AscentProblem Build(int stages, double qLimit)
        {
            AscentStage[] list = AscentPhasePlan.Truncate(allStages, stages);
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
        double peKm = _s.PeKm, apKm = _s.ApKm, incDeg = _s.IncDeg, lanDeg = _s.LanDeg;
        double qMaxKpa = qMax / 1000.0;
        double insAltKm = (rIns - bodyRadius) / 1000.0;

        AscentPlan Publish(AscentSolution sol, int stages, double qUsed, string[] notes, double wall) => new()
        {
            Solution = sol,
            Profile = sol.Nodes > 0 ? ConvexAscentProfile.FromSolution(sol, omega, bodyRadius) : null,
            Series = AscentPlanSeries.Build(sol, omega, bodyRadius, qUsed, qAlphaMax),
            VehicleId = vehicleId,
            BodyName = bodyName,
            Body = parent,
            Omega = omega,
            BodyRadius = bodyRadius,
            SolvedAt = solvedAt,
            Mass0 = m0,
            StartCcf = startCcf,
            PeKm = peKm,
            ApKm = apKm,
            IncDeg = incDeg,
            LanDeg = lanDeg,
            ThrottleMinPct = floor * 100.0,
            ThrottleMinRequestedPct = throttleRequestedPct,
            SolidStages = allStages.Take(stages).Count(st => st.Solid != null),
            CoreThrottledDown = stagePlan.CoreThrottledDown,
            StageLines = stagePlan.Describe.Take(stages).Select((d, i) => d + (allStages[i].IsPinned ? $", pinned to {allStages[i].FixedBurnTime:F1} s" : "")).ToArray(),
            StageNotes = stagePlan.Notes.ToArray(),
            QMaxKpa = qUsed / 1000.0,
            QMaxRequestedKpa = qMaxKpa,
            QAlphaMax = qAlphaMax,
            InsertionAltKm = insAltKm,
            FinalMassFloor = Build(stages, qUsed).FinalMassFloor,
            StagesPlanned = stages,
            StagesAvailable = count,
            Notes = notes,
            WallSeconds = wall,
        };

        // The whole problem, every stage, as JSON beside the temp files: the test console's --ascent-replay solves it again offline, which is how a plan that fails in the game gets diagnosed.
        string dump = "";
        try
        {
            string dir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "afc-convex-ascent");
            System.IO.Directory.CreateDirectory(dir);
            dump = System.IO.Path.Combine(dir, $"{string.Concat(vehicleId.Split(System.IO.Path.GetInvalidFileNameChars()))}.json");
            System.IO.File.WriteAllText(dump, AscentProblemFile.ToJson(Build(count, qMax)));
        }
        catch (Exception e)
        {
            dump = "not written: " + e.Message;
        }

        GuidanceLog.Info(vehicle, $"convex ascent requested (problem {dump}): {count} stage(s) available, trying {initial} first; "
            + (launchAt > now ? $"lift-off at the window, {launchAt - now:F0} s from now; " : "")
            + $"{m0 / 1000.0:F1} t, target {(rIns - bodyRadius) / 1000.0:F0} km at {vIns:F0} m/s, inc {_s.IncDeg:F2} deg, LAN {_s.LanDeg:F2} deg, "
            + $"q {qMaxKpa:F0} kPa, q-alpha {qAlphaMax:F0} Pa rad, throttle floor {floor * 100.0:F0} % ({string.Join("/", allStages.Select(st => (st.ThrottleMin * 100.0).ToString("F0")))} by stage), drag area {dragArea:F1} m^2, "
            + (atmosphere != null ? $"air {atmosphere}" : "no air") + ".");
        for (int i = 0; i < count; i++)
            GuidanceLog.Info(vehicle, $"convex ascent stage {i + 1}: {stagePlan.Describe[i]}"
                + (allStages[i].IsPinned ? $", pinned to {allStages[i].FixedBurnTime:F1} s" : "")
                + (allStages[i].JettisonMass > 0.0 ? $", drops {allStages[i].JettisonMass / 1000.0:F2} t" : "") + ".");
        for (int m = 0; m < gathered.Motors.Count; m++)
            GuidanceLog.Info(vehicle, $"convex ascent solid '{gathered.Motors[m].Name}': burns {gathered.Motors[m].BurnTime:F1} s stepped through the game's model "
                + $"(its thrust profile reports {gathered.ProfileSeconds[m]:F1} s for the full grain), flow {gathered.Motors[m].MassFlow[0]:F0} -> {gathered.Motors[m].MassFlow[^1]:F0} kg/s.");
        foreach (string note in stagePlan.Notes)
            GuidanceLog.Info(vehicle, "convex ascent: " + note + ".");

        job = new AscentPlanJob(Build, count, initial, qMax, Publish);
        error = "";
        return true;
    }

    // --- flying the plan ------------------------------------------------------

    /// <summary>
    /// Whether the stored plan may be flown on this vehicle as it is now. A plan belongs to the vehicle, place, mass, lift-off instant, target and limits it was solved for; EXECUTE calculates a new one for a plan that has lost any of them, and the reason says why.
    /// </summary>
    /// <param name="launchAt">The lift-off it would be flown from, sim time: now for a launch, the window's instant for one armed to wait for it.</param>
    private static bool ConvexPlanFlyable(Vehicle vehicle, Orbit orbit, IParentBody parent, double launchAt, out string why)
    {
        AscentPlan p = _s.AscentPlan;
        if (p == null)
        {
            why = "no plan calculated";
            return false;
        }
        if (!p.Usable)
        {
            why = "the plan did not converge";
            return false;
        }
        if (p.VehicleId != vehicle.Id || !ReferenceEquals(p.Body, parent))
        {
            why = "planned for another vehicle or body";
            return false;
        }
        double3 ccf = orbit.StateVectors.PositionCci.Transform(parent.GetCci2Ccf());
        if ((ccf - p.StartCcf).Length() > PlanStartToleranceM)
        {
            why = "the vehicle has moved since it was planned";
            return false;
        }
        if (Math.Abs(vehicle.TotalMass - p.Mass0) > PlanMassTolerance * p.Mass0)
        {
            why = "the vehicle's mass has changed since it was planned";
            return false;
        }
        if (p.TargetDiffers(_s.PeKm, _s.ApKm, _s.IncDeg, _s.LanDeg))
        {
            why = "the target orbit has changed since it was planned";
            return false;
        }
        if (p.SettingsDiffer(ConvexQMaxKpaUsed(), ConvexQAlphaMaxUsed(), _s.ConvexThrottleMinPct))
        {
            why = "the convex settings have changed since it was planned";
            return false;
        }
        double late = launchAt - p.SolvedAt;
        if (Math.Abs(late) > PlanLaunchToleranceS)
        {
            why = late > 0.0
                ? $"it was planned for a lift-off {late:F0} s earlier"
                : $"it was planned for a lift-off {-late:F0} s later";
            return false;
        }
        why = "";
        return true;
    }

    /// <summary>The panel's max-q and q-alpha as a plan is solved to them, floored away from zero.</summary>
    private static double ConvexQMaxKpaUsed() => Math.Max(_s.ConvexQMaxKpa, 0.1);
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
        double full = KsaEnginePerf.ActiveThrustCapability(vehicle, pressure);
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
            why = "no profile to fly";
            return true;
        }
        if (_s.ConvexPlanTime >= profile.EndTime)
        {
            why = "the plan ran out";
            return true;
        }
        if (alt >= _s.ConvexHandoverAltKm * 1000.0 && _s.Upfg.Converged)
        {
            why = $"above {_s.ConvexHandoverAltKm:F0} km";
            return true;
        }
        why = "";
        return false;
    }
}
