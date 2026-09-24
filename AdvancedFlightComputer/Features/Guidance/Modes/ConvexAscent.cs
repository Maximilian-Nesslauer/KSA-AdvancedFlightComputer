#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

// The convex ascent: launch3dof.py's minimum-propellant ascent, solved offline for this vehicle by SCvx and flown open loop in place of the vertical rise and gravity turn, with UPFG taking over for the terminal guidance once the vehicle is high enough.
//
// Calculate ascent asks for a plan. The request is picked up by the vehicle's sim step, which is the one place the part tree can be read without racing the game's own recompute, and which builds the problem there as plain data: the staging model with each stage's thrust across back pressure, the drag table sampled off the vehicle, the body's atmosphere, and the target orbit. The solve then runs on its own thread (AscentPlanJob) and publishes an immutable AscentPlan, which the panel draws and EXECUTE flies.
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
                GuidanceLog.Info(vehicle, $"convex ascent calculation ended without a plan: {string.Join("; ", notes)}.");
            }
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
        }
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

        // The staging model, as RefreshStageModel builds it, with thrust sampled across back pressure.
        SequencePerformanceList performance = vehicle.Parts?.PerformanceSequences;
        if (performance == null || vehicle.Parts.SequenceList == null)
        {
            error = "the vehicle has no staging sequence";
            return false;
        }
        float ambient = vehicle.PhysicsEnvironment.AtmosphericPressure;
        performance.RecomputeForFlight(ambient);
        UpfgVehicle model = KsaVehicleAdapter.Build(vehicle, ambient, grid);
        if (model.Stages.Count == 0)
        {
            error = "no stage has thrust and propellant";
            return false;
        }
        // The booster reserve cuts the first burn short, as the ascent itself will: planned on the same copy UPFG flies.
        if (_s.ReserveArmed)
            ApplyAscentReserve(model, _s.ReserveKg);

        // The mass chain. Stage 0 starts at the LIVE mass, as UPFG reconciles it; each later stage starts where the model says, and whatever the model drops between one burnout and the next ignition is the jettison.
        const double g0 = 9.80665;
        double m0 = vehicle.TotalMass;
        int count = model.Stages.Count;
        var thrust = new double[count];
        var massFlow = new double[count];
        var prop = new double[count];
        var jettison = new double[count];
        var grids = new double[count][];
        var tables = new double[count][];
        var dvCum = new double[count];
        double dvSum = 0.0;
        for (int i = 0; i < count; i++)
        {
            UpfgStage st = model.Stages[i];
            double start = i == 0 ? m0 : st.MassTotal;
            thrust[i] = st.Thrust;
            massFlow[i] = st.Thrust / (st.Isp * g0);
            prop[i] = start - st.MassDry;
            jettison[i] = i + 1 < count ? Math.Max(0.0, st.MassDry - model.Stages[i + 1].MassTotal) : 0.0;
            grids[i] = st.PressureGrid;
            tables[i] = st.ThrustAtPressure;
            if (!(prop[i] > 1.0) || !(massFlow[i] > 0.0))
            {
                error = $"stage {i + 1} has nothing left to burn";
                return false;
            }
            dvSum += st.Isp * g0 * Math.Log(start / st.MassDry);
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
        double3 r = orbit.StateVectors.PositionCci;
        double3 v = orbit.StateVectors.VelocityCci;
        double3 spin = new double3(0, 0, omega);
        double3 up = double3.Normalize(r);
        if ((v - double3.Cross(spin, r)).Length() < 10.0)
            v = double3.Cross(spin, r) + 10.0 * up;

        double groundRadius = Math.Min(bodyRadius, r.Length() - 1.0);
        double qMax = Math.Max(_s.ConvexQMaxKpa, 0.1) * 1000.0;
        double qAlphaMax = Math.Max(_s.ConvexQAlphaMax, 1.0);
        double dragArea = aero.ReferenceArea;
        double[] r0 = [r.X, r.Y, r.Z], v0 = [v.X, v.Y, v.Z], n0 = [normal.X, normal.Y, normal.Z];

        AscentProblem Build(int stages, double qLimit)
        {
            var list = new AscentStage[stages];
            for (int i = 0; i < stages; i++)
                list[i] = new AscentStage(thrust[i], massFlow[i], prop[i],
                                          i < stages - 1 ? jettison[i] : 0.0, dragArea, grids[i], tables[i]);
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
        double solvedAt = SimNow();
        double3 startCcf = r.Transform(parent.GetCci2Ccf());
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
            + $"{m0 / 1000.0:F1} t, target {(rIns - bodyRadius) / 1000.0:F0} km at {vIns:F0} m/s, inc {_s.IncDeg:F2} deg, LAN {_s.LanDeg:F2} deg, "
            + $"q {qMaxKpa:F0} kPa, q-alpha {qAlphaMax:F0} Pa rad, drag area {dragArea:F1} m^2, "
            + (atmosphere != null ? $"air {atmosphere}" : "no air") + ".");

        job = new AscentPlanJob(Build, count, initial, qMax, Publish);
        error = "";
        return true;
    }

    // --- flying the plan ------------------------------------------------------

    /// <summary>
    /// Whether EXECUTE may fly the stored plan on this vehicle as it is now. A plan belongs to the vehicle, place and mass it was solved from; one that has lost any of them flies the legacy gravity turn instead, and the reason says why.
    /// </summary>
    private static bool ConvexPlanFlyable(Vehicle vehicle, Orbit orbit, IParentBody parent, out string why)
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
        why = "";
        return true;
    }

    /// <summary>Air-relative speed: what the profile is indexed by.</summary>
    private static double AirSpeed(double3 r, double3 v, IParentBody parent)
        => (v - double3.Cross(new double3(0, 0, parent.GetAngularVelocity()), r)).Length();

    /// <summary>
    /// The open-loop phase's command: the plan's attitude where the vehicle is on the plan, and the rate that command is turning at.
    ///
    /// Where it is on the plan is a plan time, carried from step to step and moved on by the vehicle's air speed (see ConvexAscentProfile.Advance): pitch against speed while the vehicle is gaining speed it has never had, the plan's own clock while it is not. The clock runs on SIM time since the last step, not the slew's clamped step, so a warped step moves the plan as far as it moved the vehicle.
    ///
    /// The rate is the command's own, differenced step to step - it includes the local horizon turning under the vehicle as well as the programme itself.
    /// </summary>
    private static double3 ConvexProfileCommand(double3 r, double3 v, IParentBody parent, double stepDt,
                                                out double3 rate)
    {
        rate = default;
        ConvexAscentProfile profile = _s.FlyingPlan?.Profile;
        if (profile == null)
            return double3.Normalize(r);

        double now = SimNow();
        double dt = double.IsFinite(_s.ConvexLastTime) ? Math.Max(0.0, now - _s.ConvexLastTime) : 0.0;
        _s.ConvexLastTime = now;
        _s.ConvexPlanTime = profile.Advance(_s.ConvexPlanTime, AirSpeed(r, v, parent), dt, ref _s.ConvexFastest);

        double3 want = profile.Direction(r, _s.ConvexPlanTime);
        double3 last = _s.ConvexLastWant;
        if (last.Length() > 0.5 && stepDt > 1e-6)
        {
            double3 axis = double3.Cross(last, want);
            double angle = AngleBetween(last, want);
            if (axis.Length() > 1e-12)
                rate = double3.Normalize(axis) * (angle / stepDt);
        }
        _s.ConvexLastWant = want;
        return want;
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
