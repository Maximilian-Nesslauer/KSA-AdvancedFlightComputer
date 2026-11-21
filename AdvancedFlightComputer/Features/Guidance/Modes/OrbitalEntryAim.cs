#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

/// <summary>
/// AIMING THE DEORBIT AT THE SITE, for an orbital entry (DEORBIT ONLY with ENTRY AFTER DEORBIT and AIM AT SITE on).
///
/// A DEORBIT ONLY burn on its own goes at the next chance, so the entry starts wherever the orbit happens to be. On the first flight (2026-11-21) that put the site inside the shortest range the craft could still fly: it overbanked, flew past and turned back. Here the burn time is chosen so the site sits at a fraction of the corridor (AimFraction, half by default) when the craft reaches the air.
///  1. MEASURE. The belly-first sweep and its trim schedule, as the entry itself takes them, give the planner's model of the craft.
///  2. CORRIDOR. From one predicted entry state, the coast down to the start drag (EntryPlanner.PredictStart) and the corridor there (EntryPlanner.Corridor): the shortest and longest range still reachable. Every burn time gives the same entry energy, so this is done once.
///  3. SEARCH. Departure times are scanned over the coming hours (AimHorizonH) for a pass whose ground track comes within AimMaxCrossKm of the site, and on it the burn time that puts the site at the aimed range ahead of the start of the air, refined by bisection. Burns are horizontal, down to the entry periapsis: they cannot turn the orbit plane, so the crossrange is the pass's.
///  4. CHECK. One cold plan from the predicted start must reach the site: converged, with a small miss. A miss moves the aimed range by the miss and searches again, up to three times; a plan that cannot be had refuses the burn.
/// </summary>
public static partial class GuidanceWindow
{
    private const int AimMaxTries = 3;

    /// <summary>
    /// DEORBIT ONLY's burn for an orbital entry, aimed at the site. True when the burn is planned (DeorbitPlan set, as PlanEntryBurn sets it); false while it measures, searches or checks, with the request dropped so the next step comes back here, or when it refuses.
    /// </summary>
    private static bool PlanAimedEntryBurn(Vehicle vehicle, IParentBody parent, DeorbitRequest request)
    {
        var st = _s.OrbitalEntry;
        double now = SimNow();
        EntryConfig cfg = OrbitalEntryConfig();

        // ---- 1. measure ----
        if (st.AimAero == null || st.AimAero.MeshHash != KsaAeroSweep.LiveMeshHash(vehicle))
        {
            if (st.AimJob == null)
            {
                st.AimAero = null;
                if (!KsaAeroSweep.TryStart(vehicle, parent, now, out KsaAeroSweep.Job job, out string error, bellyFirst: true))
                {
                    RefuseDeorbit(vehicle, "AIMED ENTRY: AERO SWEEP FAILED: " + error.ToUpperInvariant() + ".");
                    return false;
                }
                st.AimJob = job;
            }
            if (!st.AimJob.TryTake(out KsaAeroSweep.Result aero, out string sweepError))
                return AimWaits("AIMED ENTRY: MEASURING THE CRAFT BELLY FIRST.");
            st.AimJob = null;
            if (aero?.Belly == null || aero.Atmosphere == null || !TryBuildEntryTrim(aero, out EntryTrim trim, out int index, out string why))
            {
                RefuseDeorbit(vehicle, "AIMED ENTRY: NO BELLY-FIRST MODEL (" + (sweepError.Length > 0 ? sweepError.ToUpperInvariant() : "NO TRIM") + ").");
                return false;
            }
            st.AimAero = aero;
            st.AimTrim = trim;
            st.AimStage = OrbitalEntryState.AimSteps.None;
        }
        if (st.AimStage is OrbitalEntryState.AimSteps.None or OrbitalEntryState.AimSteps.Ready)
        {
            st.AimStage = OrbitalEntryState.AimSteps.Search;
            st.AimShiftM = 0.0;
            st.AimTries = 0;
        }

        // ---- 2 and 3. corridor and search ----
        if (st.AimStage == OrbitalEntryState.AimSteps.Search)
        {
            if (!SearchAimedDeparture(vehicle, parent, request, st, cfg, now, out string refusal))
            {
                RefuseDeorbit(vehicle, refusal);
                ClearAim(st);
                return false;
            }
            st.AimStage = OrbitalEntryState.AimSteps.Check;
            st.AimCheck = null;
            st.AimWorker ??= new Ksa3DofWorker();
            EntryModel model = st.AimModel;
            double e0 = st.AimStartE;
            double[] x0 = st.AimStartX;
            st.AimWorker.TryRun(st, ct => { st.AimCheck = _entryPlanner.Solve(model, cfg, e0, x0, null); });
        }

        // ---- 4. check ----
        if (st.AimStage == OrbitalEntryState.AimSteps.Check)
        {
            if (Interlocked.Exchange(ref st.AimCheck, null) is not { } check)
                return AimWaits(FormattableString.Invariant($"AIMED ENTRY: CHECKING THE PLAN FROM {st.AimStartRangeM / 1000:F0} KM OUT."));
            EntryPlan p = check.Plan;
            st.AimTries++;
            string result = p == null ? "no plan: " + check.Why : FormattableString.Invariant(
                $"{(p.Converged ? "converged" : "refused " + check.Why)}, miss {p.TerminalRangeErrorM / 1000:F1} km, {p.Iterations} it, {p.SolveMs:F0} ms");
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Aimed entry check {st.AimTries}: departure in {st.AimDeparture - now:F0} s, site {st.AimStartRangeM / 1000:F0} km ahead at the start of the air (corridor {st.AimShortM / 1000:F0}-{st.AimLongM / 1000:F0} km, aimed {st.AimTargetM / 1000:F0}), crossrange {st.AimCrossM / 1000:F1} km: {result}."));
            if (p == null)
            {
                RefuseDeorbit(vehicle, "AIMED ENTRY: NO PLAN FROM THE AIMED ENTRY (" + check.Why.ToUpperInvariant() + ").");
                ClearAim(st);
                return false;
            }
            if (!p.Converged || Math.Abs(p.TerminalRangeErrorM) > cfg.GateRangeM)
            {
                if (st.AimTries >= AimMaxTries)
                {
                    RefuseDeorbit(vehicle, FormattableString.Invariant($"AIMED ENTRY: NO PLAN REACHES THE SITE ({p.TerminalRangeErrorM / 1000:F0} KM MISS AFTER {st.AimTries} TRIES)."));
                    ClearAim(st);
                    return false;
                }
                // Range left at the hand-over: start that much closer. Overshot: that much further.
                st.AimShiftM -= p.TerminalRangeErrorM;
                st.AimStage = OrbitalEntryState.AimSteps.Search;
                return AimWaits("AIMED ENTRY: MOVING THE BURN BY THE PLAN'S MISS.");
            }

            // ---- commit ----
            DeorbitPlan? plan = EntryDeorbitPlanner.PlanAt(request, st.AimDeparture, out string why, _s.DeorbitEntryMeasuredPreparation);
            if (plan is not { } entry)
            {
                RefuseDeorbit(vehicle, "AIMED ENTRY: " + why);
                ClearAim(st);
                return false;
            }
            _s.DeorbitPlan = entry;
            _s.BurnStartTime = entry.ArrivalTime;
            _s.LandingPhase = LandingPhase.DeorbitCoast;
            _s.LandingStatus = FormattableString.Invariant(
                $"ENTRY BURN AIMED: {entry.DeltaV.Length():F1} M/S IN {Fmt.TimeFine(entry.DepartureTime - now)}, SITE {st.AimStartRangeM / 1000:F0} KM AHEAD AT THE AIR (CORRIDOR {st.AimShortM / 1000:F0}-{st.AimLongM / 1000:F0}), {Math.Abs(st.AimCrossM) / 1000:F0} KM {(st.AimCrossM >= 0 ? "LEFT" : "RIGHT")}");
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Aimed entry burn planned: departure in {entry.DepartureTime - now:F1} s, delta-v {entry.DeltaV.Length():F2} m/s, entry interface in {entry.ArrivalTime - now:F1} s, plan miss {p.TerminalRangeErrorM / 1000:F2} km."));
            st.AimStage = OrbitalEntryState.AimSteps.Ready;
            return true;
        }
        return AimWaits("AIMED ENTRY: PLANNING.");
    }

    /// <summary>A step of the aim that is not done yet: the request is dropped, so the next step captures a fresh one and comes back.</summary>
    private static bool AimWaits(string status)
    {
        _s.DeorbitRequest = null;
        _s.LandingStatus = status;
        return false;
    }

    private static void ClearAim(OrbitalEntryState st)
    {
        st.AimJob = null;
        st.AimStage = OrbitalEntryState.AimSteps.None;
        st.AimCheck = null;
        st.AimShiftM = 0;
        st.AimTries = 0;
    }

    /// <summary>The site's range along the ground track and its crossrange (+ left), rad, from a state (CCI, inertial) at sim time <paramref name="atSeconds"/>.</summary>
    private static void SiteGeometry(IParentBody parent, double3 r, double3 vInertial, double atSeconds, out double downRad, out double crossRad, out double3 vRel)
    {
        vRel = vInertial - double3.Cross(parent.GetAngularVelocityCci(), r);
        double3 rHat = double3.Normalize(r);
        double3 vHat = vRel.NormalizeOrZero();
        double3 site = double3.Normalize(SiteDirCciAt(parent, atSeconds - SimNow()));
        double3 head = (vHat - double3.Dot(vHat, rHat) * rHat).NormalizeOrZero();
        downRad = Math.Atan2(double3.Dot(site, head), double3.Dot(site, rHat));
        crossRad = Math.Asin(Math.Clamp(double3.Dot(double3.Cross(r, vRel).NormalizeOrZero(), site), -1.0, 1.0));
    }

    /// <summary>
    /// Steps 2 and 3: the corridor from one predicted entry, then the first burn time over the horizon whose pass comes within the crossrange limit and puts the site at the aimed range ahead of the start of the air.
    /// </summary>
    private static bool SearchAimedDeparture(Vehicle vehicle, IParentBody parent, DeorbitRequest request, OrbitalEntryState st, EntryConfig cfg, double now, out string refusal)
    {
        refusal = "";
        double R0 = parent.MeanRadius;
        double lead = 120.0;
        double t0 = now + lead;
        double period = request.Source.IsBound() ? request.Source.Period : 5400.0;
        if (!double.IsFinite(period) || period <= 0) period = 5400.0;

        // The mass after the burn, from the first burn that can be had.
        double mass = request.Mass;
        for (double t = t0; t < t0 + period; t += 30.0)
        {
            if (EntryDeorbitPlanner.PlanAt(request, t, out _, _s.DeorbitEntryMeasuredPreparation) is { } first)
            {
                mass = first.CutoffMass;
                t0 = t;
                break;
            }
        }
        EntryModel model = OrbitalEntryModel(parent, st.AimAero, st.AimTrim, mass, 1.0, 1.0);
        st.AimModel = model;

        // The coast from the entry interface to the start drag, and the corridor there: the same for every burn time to within the orbit's own variation.
        if (!EntryDeorbitPlanner.TryArrival(request, t0, out double refArrival, out StateVectors refState))
        {
            refusal = "AIMED ENTRY: THE ENTRY BURN DOES NOT MEET THE AIR.";
            return false;
        }
        SiteGeometry(parent, refState.PositionCci, refState.VelocityCci, refArrival, out _, out _, out double3 refRel);
        double refGamma = Math.Asin(Math.Clamp(double3.Dot(refRel.NormalizeOrZero(), double3.Normalize(refState.PositionCci)), -1.0, 1.0));
        var coast = EntryPlanner.PredictStart(model, cfg, refState.PositionCci.Length(), refRel.Length(), refGamma, 0.0);
        if (coast == null)
        {
            refusal = "AIMED ENTRY: THE ENTRY DOES NOT REACH THE START DRAG.";
            return false;
        }
        double coastRangeM = -coast.Value.X[EntryDynamics.IS] * R0;
        double coastS = coast.Value.SecondsAhead;
        (double shortM, double longM) = EntryPlanner.Corridor(model, cfg, coast.Value.E);
        double fraction = Math.Clamp(_s.OrbitalEntryAimFraction, 0.0, 1.0);
        double target = shortM + fraction * (longM - shortM) + st.AimShiftM;
        st.AimShortM = shortM;
        st.AimLongM = longM;
        st.AimTargetM = target;
        double maxCross = Math.Max(_s.OrbitalEntryAimMaxCrossKm, 1.0) * 1000.0;

        // f(t) = range ahead at the start of the air less the aim: falls as the burn moves later along a pass.
        bool Eval(double t, out double f, out double crossM)
        {
            f = crossM = double.NaN;
            if (!EntryDeorbitPlanner.TryArrival(request, t, out double arrival, out StateVectors s))
                return false;
            SiteGeometry(parent, s.PositionCci, s.VelocityCci, arrival + coastS, out double down, out double cross, out _);
            crossM = cross * R0;
            f = down * R0 - coastRangeM - target;
            return down > 0.0 && down < Math.PI / 2;
        }

        double horizon = Math.Max(_s.OrbitalEntryAimHorizonH, 0.1) * 3600.0;
        double step = Math.Clamp(period / 180.0, 10.0, 60.0);
        double bestCross = double.PositiveInfinity;
        bool havePrev = false;
        double tPrev = 0, fPrev = 0;
        for (double t = t0; t <= t0 + horizon; t += step)
        {
            bool ok = Eval(t, out double f, out double crossM);
            if (ok && Math.Abs(crossM) < Math.Abs(bestCross)) bestCross = crossM;
            if (!ok || Math.Abs(crossM) > maxCross)
            {
                havePrev = false;
                continue;
            }
            if (havePrev && fPrev > 0 && f <= 0)
            {
                double lo = tPrev, hi = t;
                for (int i = 0; i < 40 && hi - lo > 0.05; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    if (Eval(mid, out double fm, out _) && fm > 0) lo = mid; else hi = mid;
                }
                double td = 0.5 * (lo + hi);
                if (!EntryDeorbitPlanner.TryArrival(request, td, out double arrival, out StateVectors s))
                    break;
                SiteGeometry(parent, s.PositionCci, s.VelocityCci, arrival, out double down, out double cross, out double3 rel);
                double gamma = Math.Asin(Math.Clamp(double3.Dot(rel.NormalizeOrZero(), double3.Normalize(s.PositionCci)), -1.0, 1.0));
                var start = EntryPlanner.PredictStart(model, cfg, s.PositionCci.Length(), rel.Length(), gamma, down);
                if (start == null)
                    break;
                st.AimDeparture = td;
                st.AimStartE = start.Value.E;
                st.AimStartX = start.Value.X;
                st.AimStartRangeM = start.Value.X[EntryDynamics.IS] * R0;
                st.AimCrossM = cross * R0;
                GuidanceLog.Info(vehicle, FormattableString.Invariant(
                    $"Aimed entry search: corridor {shortM / 1000:F0}-{longM / 1000:F0} km at the start of the air (coast {coastS:F0} s, {coastRangeM / 1000:F0} km from the interface), aimed {target / 1000:F0} km (fraction {fraction:F2}, shift {st.AimShiftM / 1000:F0} km); burn in {td - now:F0} s, crossrange {st.AimCrossM / 1000:F1} km."));
                return true;
            }
            havePrev = true;
            tPrev = t;
            fPrev = f;
        }
        refusal = double.IsFinite(bestCross)
            ? FormattableString.Invariant($"AIMED ENTRY: NO PASS IN {_s.OrbitalEntryAimHorizonH:F0} H COMES WITHIN {maxCross / 1000:F0} KM OF THE SITE (CLOSEST {Math.Abs(bestCross) / 1000:F0} KM).")
            : "AIMED ENTRY: NO BURN IN THE HORIZON MEETS THE AIR.";
        return false;
    }
}
