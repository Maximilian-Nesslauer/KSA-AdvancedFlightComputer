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
///  AIM. Before each entry re-plan, the range the entry leaves at the hand-over is set to bellyflop's predicted belly-first fall from the plan's own hand-over state (BellyflopPlanner.FallRange), so the entry aims at the ground through it.
///  PLAN. From OrbitalEntryBellyflopArmMach the planner (Scvx/Bellyflop) solves in the background on its own worker, warm from its last plan: pitch and bank (the airflow's angles on the belly) and yaw over a free flight time, falling onto the site.
///  TAKE-OVER. At the hand-over (OrbitalEntryHandoverMach) with a converged plan, bellyflop flies the craft: the yaw reference is held from then, and the plan's angles become the nose and belly the attitude controller is given, as the entry's are.
/// </summary>
public static partial class GuidanceWindow
{
    private static BellyflopConfig BellyflopConfigNow() => new()
    {
        AngleMaxDeg = _s.OrbitalEntryBellyflopAngleMaxDeg,
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
        BellyflopModel model = KsaBellyflopSetup.Build(vehicle, parent, frame, siteRadius, st.Aero, new double3(1, 0, 0), st.DensityScale, out _);
        if (model == null)
            return;
        var p = st.Plan;
        int last = p.E.Length - 1;
        double handMach = _s.OrbitalEntryHandoverMach;
        // The entry's angle of attack there is from nose-first; the belly's is 90 deg less.
        double alphaBelly = (90.0 - st.Trim.AlphaAt(handMach)) * Math.PI / 180.0;
        double altitude = p.AltitudeM[last] - (siteRadius - parent.MeanRadius);
        double range = BellyflopPlanner.FallRange(model, BellyflopConfigNow(), altitude, p.SpeedMs[last], p.Gamma[last], alphaBelly);
        if (double.IsFinite(range) && range > 0)
            bf.TerminalRangeM = range;
    }

    /// <summary>
    /// One step of bellyflop under an orbital entry: take what the worker published, solve again in the background from BELLYFLOP FROM MACH, and take over at the hand-over once a plan has converged.
    /// </summary>
    private static void StepBellyflop(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, double mach, double now)
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
        BellyflopModel model = KsaBellyflopSetup.Build(vehicle, parent, frame, siteRadius, st.Aero, bf.BearingLocal, st.DensityScale, out string why);
        if (model == null)
            return;
        double3 bellyBody = KsaBellyflopSetup.BellyAxisBody(st.Aero.BellyBodyAero);
        var (aNow, bNow, pNow) = KsaBellyflopSetup.AnglesNow(vehicle, frame, model, bellyBody, x);
        bf.PitchDeg = aNow * 180.0 / Math.PI; bf.BankDeg = bNow * 180.0 / Math.PI; bf.YawDeg = pNow * 180.0 / Math.PI;

        // ---- plans: take what the worker published, ask for the next ----
        if (Interlocked.Exchange(ref bf.Pending, null) is { } res)
        {
            bf.Solves++;
            var p = res.Plan;
            string text = p == null ? "no plan: " + res.Why : FormattableString.Invariant(
                $"{(p.Converged ? "converged" : "REFUSED " + res.Why)}, {p.Iterations} it, {p.SolveMs:F0} ms, lands {p.MissM:F0} m from the site after {p.Sigma:F0} s, defect {p.MaxDefect:F3}{(p.Cold ? ", cold" : "")}");
            if (p != null && p.Converged)
            {
                bf.Plan = p;
                GuidanceLog.Debug(vehicle, $"Bellyflop solve {bf.Solves}: {text}.");
            }
            else
            {
                bf.Refusals++;
                bf.LastRefusal = res.Why.Length > 0 ? res.Why : "no plan";
                GuidanceLog.Info(vehicle, $"Bellyflop solve {bf.Solves}: {text}.");
            }
        }
        if (now >= bf.NextSolveAt && (bf.Worker == null || !bf.Worker.Busy))
        {
            bf.Worker ??= new Ksa3DofWorker();
            BellyflopConfig cfg = BellyflopConfigNow();
            BellyflopPlan warm = bf.Plan;
            BellyflopModel solveModel = model;
            double a0 = aNow, b0 = bNow, p0 = pNow;
            bf.NextSolveAt = now + Math.Max(_s.OrbitalEntryBellyflopResolveS, 0.25);
            bf.Worker.TryRun(bf, ct =>
            {
                var r = bf.Planner.Solve(solveModel, cfg, x, now, a0, b0, p0, warm);
                if (r.Plan != null && r.Plan.Converged)
                    bf.Model = solveModel;
                bf.Pending = r;
            });
        }

        // ---- take-over ----
        if (!flying && (st.Phase is OrbitalEntryPhase.Entry or OrbitalEntryPhase.Holding)
            && mach <= _s.OrbitalEntryHandoverMach && bf.Plan != null && bf.Plan.Converged && bf.Model != null)
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
            st.SetStatus(FormattableString.Invariant(
                $"BELLYFLOP: {toGo / 1000:F1} KM TO GO AT {x14[2] / 1000:F1} KM, PLAN LANDS {plan?.MissM ?? double.NaN:F0} M FROM THE SITE IN {(plan != null ? plan.EndTime - now : double.NaN):F0} s. PITCH {bf.PitchDeg:F0}, BANK {bf.BankDeg:F0}, YAW {bf.YawDeg:F0} DEG.") + warn,
                warn: plan == null || warn.Length > 0);
        }
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
        var (a, b, p) = plan.ControlAt(now);
        if (!model.Axes(x14.AsSpan(0, 6), a, b, p, out V3 nose, out _, out V3 belly, out _))
            return false;
        noseCci = frame.VecToCci(KsaBellyflopSetup.ToDouble3(nose)).NormalizeOrZero();
        bellyCci = frame.VecToCci(KsaBellyflopSetup.ToDouble3(belly)).NormalizeOrZero();
        return noseCci.Length() > 0.5 && bellyCci.Length() > 0.5;
    }
}
