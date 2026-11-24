#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;
using AdvancedFlightComputer.Guidance.Scvx.Bellyflop;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using Brutal.Numerics;
using KSA;

/// <summary>
/// BELLYFLOP: the belly-first terminal descent from the orbital entry's hand-over to the flip (docs/guidance/bellyflop-design.md).
///
///  AIM. Before each entry re-plan, the range the entry leaves at the hand-over is set to bellyflop's predicted fall from the plan's own hand-over state (BellyflopPlanner.FallRange), held at 20 deg of pitch, so the entry aims at the ground through it; and its reach band, the falls held nose down and nose up, outside which alone the entry prices the range (EntryConfig.TerminalRangeShortM).
///  PLAN. From OrbitalEntryBellyflopArmMach the planner (Scvx/Bellyflop) solves in the background on its own worker, warm from its last plan: pitch and bank (the airflow's angles on the belly) and yaw over a free flight time, falling onto the site.
///  TAKE-OVER. At its first converged plan, from OrbitalEntryBellyflopArmMach (the entry's hand-over at OrbitalEntryHandoverMach is only the latest), bellyflop flies the craft: the yaw reference is held from then, and the plan's angles become the nose and belly the attitude controller is given, as the entry's are.
/// </summary>
public static partial class GuidanceWindow
{
    private static BellyflopConfig BellyflopConfigNow() => new()
    {
        AngleMaxDeg = _s.OrbitalEntryBellyflopAngleMaxDeg,
        PitchUpMaxDeg = _s.OrbitalEntryBellyflopPitchUpDeg,
        BankMaxDeg = _s.OrbitalEntryBellyflopBankMaxDeg,
        AngleRateDegS = Math.Max(_s.OrbitalEntryRollRateDegS, 1.0),
        YawRateDegS = Math.Max(_s.OrbitalEntryRollRateDegS, 1.0),
    };

    /// <summary>The landing-site frame at the site's ground, and the site's distance from the body's centre.</summary>
    private static KsaFrameBridge.SiteFrame BellyflopSiteFrame(IParentBody parent, out double siteRadius)
    {
        siteRadius = parent.MeanRadius + SiteTerrainHeight(parent);
        return KsaFrameBridge.BuildSiteFrame(SiteDirCciAt(parent, 0) * siteRadius);
    }

    /// <summary>
    /// The range the entry should leave at its hand-over: bellyflop's predicted belly-first fall from the plan's own hand-over state, easing from the entry's angle of attack there. Kept as it was where there is nothing to predict from yet.
    /// </summary>
    private static void UpdateBellyflopRangeTarget(Vehicle vehicle, IParentBody parent, OrbitalEntryState st)
    {
        var bf = st.Bellyflop;
        if (!_s.OrbitalEntryBellyflop || !st.TablesReady || st.Aero?.BellyBodyAero == null || st.Plan == null)
            return;
        var frame = BellyflopSiteFrame(parent, out double siteRadius);
        BellyflopModel model = KsaBellyflopSetup.Build(vehicle, parent, frame, siteRadius, st.Aero, new double3(1, 0, 0), BellyflopForceScale(st), out _);
        if (model == null)
            return;
        var p = st.Plan;
        int last = p.E.Length - 1;
        double handMach = _s.OrbitalEntryHandoverMach;
        // The entry's angle of attack there is from nose-first; the belly's is 90 deg less.
        double alphaBelly = (90.0 - st.Trim.AlphaAt(handMach)) * Math.PI / 180.0;
        double altitude = p.AltitudeM[last] - (siteRadius - parent.MeanRadius);
        // Holding BellyflopAimPitchDeg rather than falling flat: bellyflop can glide far beyond its flat fall and go only a little short of it, so the site is aimed inside its reach, not at the short edge.
        BellyflopConfig bcfg = BellyflopConfigNow();
        double range = BellyflopPlanner.FallRange(model, bcfg, altitude, p.SpeedMs[last], p.Gamma[last], alphaBelly, BellyflopAimPitchDeg * Math.PI / 180.0);
        // The fall ends BELLYFLOP AIM SHORT before the site, so the entry leaves that much more.
        double aimShort = BellyflopAimShortM();
        if (double.IsFinite(range) && range > 0)
            bf.TerminalRangeM = range + aimShort;
        // The reach band: the fall held at the reserve's share of the pitch limits each way, nose down (short) and nose up (long). Turns and sideslip reach a little shorter still, so the short edge is a safe one.
        double share = Math.Clamp(bcfg.LimitMargin, 0.1, 1.0);
        double shortM = BellyflopPlanner.FallRange(model, bcfg, altitude, p.SpeedMs[last], p.Gamma[last], alphaBelly, -share * bcfg.AngleMaxDeg * Math.PI / 180.0);
        double longM = BellyflopPlanner.FallRange(model, bcfg, altitude, p.SpeedMs[last], p.Gamma[last], alphaBelly, share * bcfg.PitchUpMaxDeg * Math.PI / 180.0);
        if (double.IsFinite(shortM) && double.IsFinite(longM) && longM > shortM && shortM > 0)
            (bf.TerminalShortM, bf.TerminalLongM) = (Math.Min(shortM + aimShort, bf.TerminalRangeM), Math.Max(longM + aimShort, bf.TerminalRangeM));
    }

    /// <summary>BELLYFLOP AIM SHORT, m, never negative.</summary>
    private static double BellyflopAimShortM() => Math.Max(_s.OrbitalEntryBellyflopAimUprangeM, 0.0);

    /// <summary>
    /// The state as the planner sees it: from the aim point, BELLYFLOP AIM SHORT short of the site along the bearing, rather than from the site. The plan's miss, cone and crossrange are all about the aim point; the status still reads the distance to the site.
    /// </summary>
    private static double[] BellyflopAimState(double[] x, double3 bearing)
    {
        double d = BellyflopAimShortM();
        return [x[0] + d * bearing.X, x[1] + d * bearing.Y, x[2], x[3], x[4], x[5]];
    }

    /// <summary>The pitch the entry's aim assumes bellyflop holds after the hand-over, deg: a fall a little longer than flat, so the site sits inside bellyflop's reach with room to go short.</summary>
    private const double BellyflopAimPitchDeg = 20.0;

    /// <summary>Bellyflop's force scale: its own measurement once it has one, else the entry's density scale, measured against the entry's model before the craft turns belly first.</summary>
    private static double BellyflopForceScale(OrbitalEntryState st) => st.Bellyflop.ForceScaleKnown ? st.Bellyflop.ForceScale : st.DensityScale;

    /// <summary>Refusals in a row after which the next solve starts cold.</summary>
    private const int BellyflopColdAfterRefusals = 3;

    /// <summary>Time constant of bellyflop's force-scale low-pass, s, as the entry's.</summary>
    private const double BellyflopScaleSmoothS = 5.0;

    /// <summary>
    /// Bellyflop's force scale from this step's measured aero force against its own model's at the angles flown, with scale 1: the air and the tables together, as the craft flies now. The entry's density scale cannot stand in once the craft is belly first, because it compares the drag with the entry's model at its own angle of attack (it read 1.9 in the first flight, doubling the force bellyflop planned with).
    /// </summary>
    private static void MeasureBellyflopForceScale(OrbitalEntryState st, Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame, double siteRadius,
                                                   ReadOnlySpan<double> x, double alpha, double beta, double psi, double dt)
    {
        var bf = st.Bellyflop;
        if (!st.AeroAccelKnown)
            return;
        BellyflopModel unit = KsaBellyflopSetup.Build(vehicle, parent, frame, siteRadius, st.Aero, bf.BearingLocal, 1.0, out _);
        if (unit == null || !unit.AirForce(x, alpha, beta, psi, out double fx, out double fy, out double fz))
            return;
        double modelAccel = Math.Sqrt(fx * fx + fy * fy + fz * fz) / unit.Mass;
        double measured = st.AeroAccelCci.Length();
        if (!(modelAccel > 0.3) || !(measured > 0.3))
            return;
        double ratio = Math.Clamp(measured / modelAccel, 0.5, 2.0);
        if (!bf.ForceScaleKnown)
        {
            bf.ForceScale = ratio;
            bf.ForceScaleKnown = true;
            return;
        }
        bf.ForceScale += Math.Clamp(dt / BellyflopScaleSmoothS, 0.0, 1.0) * (ratio - bf.ForceScale);
    }

    /// <summary>
    /// One step of bellyflop under an orbital entry: take what the worker published, solve again in the background from BELLYFLOP FROM MACH, and take over at the hand-over once a plan has converged.
    /// </summary>
    private static void StepBellyflop(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, double mach, double now, double dt)
    {
        var bf = st.Bellyflop;
        if (!_s.OrbitalEntryBellyflop || !st.TablesReady || st.Aero?.BellyBodyAero == null || st.BellyIndex < 0)
            return;
        bool flying = st.Phase == OrbitalEntryPhase.Bellyflop;
        if (!flying && mach > _s.OrbitalEntryBellyflopArmMach)
            return;

        var frame = BellyflopSiteFrame(parent, out double siteRadius);
        double[] x14 = KsaFrameBridge.ToModelState(vehicle, frame);
        double[] x = [x14[0], x14[1], x14[2], x14[3], x14[4], x14[5]];
        // The yaw reference: the bearing to the site, followed until the take-over and held from then.
        if (!flying)
        {
            var toSite = new double3(-x[0], -x[1], 0).NormalizeOrZero();
            if (toSite.Length() > 0.5)
                bf.BearingLocal = toSite;
        }
        BellyflopModel model = KsaBellyflopSetup.Build(vehicle, parent, frame, siteRadius, st.Aero, bf.BearingLocal, 1.0, out string why);
        if (model == null)
            return;
        double3 bellyBody = KsaBellyflopSetup.BellyAxisBody(st.Aero.BellyBodyAero);
        var (aNow, bNow, pNow) = KsaBellyflopSetup.AnglesNow(vehicle, frame, model, bellyBody, x);
        bf.PitchDeg = aNow * 180.0 / Math.PI; bf.BankDeg = bNow * 180.0 / Math.PI; bf.YawDeg = pNow * 180.0 / Math.PI;
        MeasureBellyflopForceScale(st, vehicle, parent, frame, siteRadius, x, aNow, bNow, pNow, dt);
        model = model with { ForceScale = BellyflopForceScale(st) };

        // ---- plans: take what the worker published, ask for the next ----
        if (Interlocked.Exchange(ref bf.Pending, null) is { } res)
        {
            bf.Solves++;
            var p = res.Plan;
            string text = p == null ? "no plan: " + res.Why : FormattableString.Invariant(
                $"{(p.Converged ? "converged" : "REFUSED " + res.Why)}, {p.Iterations} it, {p.SolveMs:F0} ms, lands {p.MissM:F0} m from the site after {p.Sigma:F0} s, defect {p.MaxDefect:F3}{(p.Cold ? ", cold" : "")}");
            // A plan whose dynamics closed seeds the next solve, also one refused only for its miss.
            if (p != null && p.MaxDefect <= new BellyflopConfig().GateDefect)
                bf.Warm = p;
            if (p != null && p.Converged)
            {
                bf.RefusedInRow = 0;
                bf.Plan = p;
                bf.Model = bf.PendingModel ?? bf.Model;
                GuidanceLog.Debug(vehicle, $"Bellyflop solve {bf.Solves}: {text}.");
            }
            else
            {
                bf.Refusals++;
                bf.RefusedInRow++;
                bf.LastRefusal = res.Why.Length > 0 ? res.Why : "no plan";
                // Before the take-over the entry is still flying to the hand-over, so a site out of reach from here is expected: said once in the game log, then in the debug log.
                // Under bellyflop a refused plan is still shot from the craft's state, so its landing is what flying it gives: flown when nearer than the plan flown now would land from the same state. Flying the last converged plan on through a run of refusals (2026-11-24, from 15 km down) held its schedule from a state long gone: a 70 deg turn at 13 km, past the site, 1.4 km off.
                bool better = flying && p != null && bf.PendingModel != null && p.MissM < bf.PendingFlownMissM;
                if (better)
                {
                    bf.Plan = p;
                    bf.Model = bf.PendingModel;
                }
                string flown = better ? FormattableString.Invariant($"; flown, the plan before would land {bf.PendingFlownMissM:F0} m off") : "";
                if (flying || bf.Refusals == 1)
                    GuidanceLog.Info(vehicle, $"Bellyflop solve {bf.Solves}: {text}{flown}{(flying ? "" : " (the entry still flies to the hand-over)")}.");
                else
                    GuidanceLog.Debug(vehicle, $"Bellyflop solve {bf.Solves}: {text}.");
            }
        }
        if (now >= bf.NextSolveAt && (bf.Worker == null || !bf.Worker.Busy))
        {
            bf.Worker ??= new Ksa3DofWorker();
            BellyflopConfig cfg = BellyflopConfigNow();
            // A run of refusals means the warm start no longer leads anywhere from the real state (2026-11-23: 40 in a row, each seeded from the same stale plan): start cold.
            bool cold = bf.RefusedInRow >= BellyflopColdAfterRefusals;
            BellyflopPlan warm = cold ? null : bf.Plan ?? bf.Warm;
            if (cold)
            {
                bf.RefusedInRow = 0;
                bf.Warm = null;
            }
            BellyflopModel solveModel = model;
            double[] xAim = BellyflopAimState(x, bf.BearingLocal);
            BellyflopPlan flownPlan = flying ? bf.Plan : null;
            double a0 = aNow, b0 = bNow, p0 = pNow;
            bf.NextSolveAt = now + Math.Max(_s.OrbitalEntryBellyflopResolveS, 0.25);
            bf.Worker.TryRun(bf, ct =>
            {
                var r = bf.Planner.Solve(solveModel, cfg, xAim, now, a0, b0, p0, warm);
                bf.PendingFlownMissM = flownPlan != null ? BellyflopPlanner.ShotMiss(solveModel, cfg, xAim, now, a0, b0, p0, flownPlan) : double.PositiveInfinity;
                bf.PendingModel = solveModel;
                bf.Pending = r;
            });
        }

        // ---- take-over: at the first plan that reaches the site, solved in the background from BELLYFLOP FROM MACH; HAND-OVER MACH is only the latest, where the entry holds its attitude until one comes. Waiting for Mach 5 with a plan in hand (2026-11-24) left the entry to fly on against a point it could no longer reach. ----
        if (!flying && (st.Phase is OrbitalEntryPhase.Entry or OrbitalEntryPhase.Holding)
            && bf.Plan != null && bf.Plan.Converged && bf.Model != null)
        {
            st.Phase = OrbitalEntryPhase.Bellyflop;
            st.Tracker.Reset();
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"BELLYFLOP TAKES OVER at Mach {mach:F2}, {st.AltitudeM / 1000:F1} km: {Math.Sqrt(x[0] * x[0] + x[1] * x[1]) / 1000:F1} km from the site, the plan landing {bf.Plan.MissM:F0} m from it after {bf.Plan.EndTime - now:F0} s; pitch {bf.PitchDeg:F0}, bank {bf.BankDeg:F0}, yaw {bf.YawDeg:F0} deg."));
        }
        if (st.Phase == OrbitalEntryPhase.Bellyflop)
        {
            var plan = bf.Plan;
            double toGo = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
            string warn = bf.Refusals > 0 && bf.LastRefusal.Length > 0 ? " (LAST SOLVE REFUSED: " + bf.LastRefusal.ToUpperInvariant() + ")" : "";
            string touchdown = bf.TouchdownStandby ? " TOUCHDOWN " + SixDofStandbyText(now).ToUpperInvariant() + "." : "";
            st.SetStatus(FormattableString.Invariant(
                $"BELLYFLOP: {toGo / 1000:F1} KM TO GO AT {x14[2] / 1000:F1} KM, PLAN LANDS {plan?.MissM ?? double.NaN:F0} M FROM ITS AIM IN {(plan != null ? plan.EndTime - now : double.NaN):F0} s. PITCH {bf.PitchDeg:F0}, BANK {bf.BankDeg:F0}, YAW {bf.YawDeg:F0} DEG.") + touchdown + warn,
                warn: plan == null || warn.Length > 0);
            StepBellyflopTouchdown(vehicle, parent, st, x14, now);
        }
    }

    /// <summary>Time constant of the low-pass on bellyflop's commanded angles, s (BellyflopAttitude): a re-solve's worth, so a new plan is blended in over about a second.</summary>
    private const double BellyflopCommandSmoothS = 0.5;

    /// <summary>Share of the engines' full thrust the burn-time seed assumes on average, less gravity.</summary>
    private const double BellyflopBurnSeedThrottle = 0.7;

    /// <summary>
    /// THE HAND-OVER TO TOUCHDOWN, as entry guidance's glide hands over before its ignition (StepEntryTouchdownOwn): from TOUCHDOWN FROM seconds before bellyflop's plan reaches the ground, touchdown guidance solves on standby from the measured state, with a coast to the ignition it chooses, the flip on the craft's whole-sphere aero (KsaAeroSweep.Result.SphereBodyAero) and the landing burn; at its first plan the hand-over is asked for, and taken on the next step ahead of the claim (CheckPendingBellyflopTouchdownHandoff). The engine is off until touchdown's plan lights it.
    /// </summary>
    private static void StepBellyflopTouchdown(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, double[] x14, double now)
    {
        var bf = st.Bellyflop;
        BellyflopPlan plan = bf.Plan;
        if (!_s.OrbitalEntryBellyflopTouchdown || plan == null || !(plan.EndTime - now <= _s.OrbitalEntryBellyflopTouchdownS) || now < bf.TouchdownHandoffRetryAt)
            return;
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        // The seeds: a burn that stops the craft at BellyflopBurnSeedThrottle of its thrust less gravity, and a coast to it at the speed it falls now.
        double speed = Math.Sqrt(x14[3] * x14[3] + x14[4] * x14[4] + x14[5] * x14[5]);
        double radius = siteCci.Length() + x14[2];
        double ambientPa = KsaEnginePerf.AmbientPressureAt(parent, radius - parent.MeanRadius);
        double thrust = KsaEnginePerf.ActiveThrustCapability(vehicle, ambientPa);
        double gravity = parent.Mu / (radius * radius), accel = thrust / Math.Max(vehicle.TotalMass, 1.0);
        // Refused only where full thrust cannot hold the craft up; whether it can stop in time is touchdown's to plan. At 70 % of it, the flopper's 753 kN were refused all the way down (harness, 2026-11-24).
        if (!(accel - gravity > 0.5))
        {
            LogSixDofStandbyRefusal(vehicle, FormattableString.Invariant($"Touchdown guidance cannot take the flip: {thrust / 1000:F0} kN of active engines cannot stop the craft."));
            return;
        }
        double decel = Math.Max(BellyflopBurnSeedThrottle * accel - gravity, 0.3 * (accel - gravity));
        double burn = Math.Max(speed / decel, SixDofStandbyMinSeedS);
        double sink = Math.Max(-x14[5], 1.0);
        double coast = Math.Max((x14[2] - 0.5 * speed * burn) / sink, 0.0);
        if (!bf.TouchdownStandby)
        {
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Touchdown guidance preparing for the flip {plan.EndTime - now:F0} s before bellyflop's ground: planning the coast and the burn from {x14[2]:F0} m at {speed:F0} m/s, a {coast:F0} s coast and a {burn:F1} s burn to seed."));
            StopSixDofStandby();
            _s.SixDofStandbyRetryAt = now;
            _s.SixDofStandbyPredicted = false;
            _s.SixDofCoastPending = true;
            bf.TouchdownStandby = true;
        }
        _s.SixDofCoastSeedS = coast;
        StepSixDofStandby(vehicle, parent, siteCci, x14, now, burn);
        if (bf.TouchdownHandoff == null && SixDofStandbyReady(now))
            bf.TouchdownHandoff = FormattableString.Invariant($"HANDOVER FROM BELLYFLOP: TOUCHDOWN AT {x14[2]:F0} m, {plan.EndTime - now:F0} s FROM BELLYFLOP'S GROUND.");
    }

    /// <summary>
    /// The hand-over to touchdown asked for by the last bellyflop step, taken here ahead of the claim, as the glide's (CheckPendingEntryTouchdownHandoff): touchdown guidance engages from its standby, which then claims the craft from orbital entry. A craft it cannot take stays with bellyflop.
    /// </summary>
    private static void CheckPendingBellyflopTouchdownHandoff(Vehicle vehicle)
    {
        var st = _s.OrbitalEntry;
        string handoff = st.Bellyflop.TouchdownHandoff;
        st.Bellyflop.TouchdownHandoff = null;
        if (handoff == null || st.Phase != OrbitalEntryPhase.Bellyflop || !_s.SixDofCoastPending || HasTouchedDown(vehicle))
            return;
        if (!Can6DofTake(vehicle, out string why) || !StandbyEnginesStillMatch(vehicle))
        {
            GuidanceLog.Info(vehicle, "Touchdown guidance cannot take the flip: " + (why ?? "ENGINE CONFIGURATION CHANGED.") + " Bellyflop keeps it.");
            StopSixDofStandby();
            _s.SixDofCoastPending = false;
            st.Bellyflop.TouchdownStandby = false;
            st.Bellyflop.TouchdownHandoffRetryAt = SimNow() + 5.0;
            return;
        }
        if (!SixDofStandbyReady(SimNow()))
            return;
        GuidanceLog.Info(vehicle, handoff);
        Engage6Dof(vehicle, fromBrakingBurn: true, tookBurn: false);
    }

    /// <summary>
    /// Bellyflop's attitude now, CCI: the nose and belly its plan flies at the craft's state. False where there is no plan or no airflow to build it on.
    /// </summary>
    private static bool BellyflopAttitude(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, double now, out double3 noseCci, out double3 bellyCci)
    {
        noseCci = bellyCci = default;
        var bf = st.Bellyflop;
        BellyflopPlan plan = bf.Plan;
        BellyflopModel model = bf.Model;
        if (plan == null || model == null)
            return false;
        var frame = BellyflopSiteFrame(parent, out _);
        double[] x14 = KsaFrameBridge.ToModelState(vehicle, frame);
        // THE COMMAND IS SMOOTH BY CONSTRUCTION, as entry guidance's glide learned (ThreeDof.cs): a first-order low-pass over the plans, from the angles flown at the take-over, so a new plan each second does not step the target the fins chase. Read BellyflopCommandSmoothS ahead, so the plan's own ramps come through without the filter's lag. Flown straight off the newest plan (2026-11-24), the fins stood at 13-32 deg and saturated 27-38 % of the time, against 3-5 deg and 3-6 % in the entry.
        var (ta, tb, tp) = plan.ControlAt(now + BellyflopCommandSmoothS);
        if (!double.IsFinite(bf.CmdAt))
        {
            const double toRad = Math.PI / 180.0;
            (bf.CmdAlpha, bf.CmdBeta, bf.CmdPsi) = double.IsFinite(bf.PitchDeg) && double.IsFinite(bf.BankDeg) && double.IsFinite(bf.YawDeg)
                ? (bf.PitchDeg * toRad, bf.BankDeg * toRad, bf.YawDeg * toRad) : (ta, tb, tp);
            bf.CmdAt = now;
        }
        double k = 1.0 - Math.Exp(-Math.Clamp(now - bf.CmdAt, 0.0, 1.0) / BellyflopCommandSmoothS);
        bf.CmdAt = now;
        bf.CmdAlpha += k * (ta - bf.CmdAlpha);
        bf.CmdBeta += k * (tb - bf.CmdBeta);
        bf.CmdPsi += k * (tp - bf.CmdPsi);
        double a = bf.CmdAlpha, b = bf.CmdBeta, p = bf.CmdPsi;
        if (!model.Axes(x14.AsSpan(0, 6), a, b, p, out V3 nose, out _, out V3 belly, out _))
            return false;
        noseCci = frame.VecToCci(KsaBellyflopSetup.ToDouble3(nose)).NormalizeOrZero();
        bellyCci = frame.VecToCci(KsaBellyflopSetup.ToDouble3(belly)).NormalizeOrZero();
        return noseCci.Length() > 0.5 && bellyCci.Length() > 0.5;
    }
}
