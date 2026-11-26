#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

/// <summary>
/// AIMING THE DEORBIT AT THE SITE, for an orbital entry (DEORBIT ONLY with AIM AT SITE on, whether ENTRY AFTER DEORBIT hands the craft to the entry or the player engages it).
///
/// A DEORBIT ONLY burn on its own goes at the next chance, so the entry starts wherever the orbit happens to be. On the first flight (2026-11-21) that put the site inside the shortest range the craft could still fly: it overbanked, flew past and turned back. Here the burn time is chosen so the site sits at a fraction of the corridor (AimFraction, half by default) when the craft reaches the air.
///  1. MEASURE. The belly-first sweep and its trim schedule, as the entry itself takes them, give the planner's model of the craft.
///  2. CORRIDOR. From one predicted entry state, the coast down to the start drag (EntryPlanner.PredictStart) and the corridor there (EntryPlanner.Corridor): the shortest and longest range still reachable. Every burn time gives the same entry energy, so this is done once.
///  3. SEARCH. Departure times are scanned over the coming hours (AimHorizonH) for a pass whose ground track comes within AimMaxCrossKm of the site, and on it the burn time that puts the site at the aimed range ahead of the start of the air, refined by bisection. Burns are horizontal, down to the entry periapsis: they cannot turn the orbit plane, so the crossrange is the pass's.
///  3b. AIM AT THE POINT. A horizontal burn cannot take out the pass's crossrange. With the departure and flight time the search found, Maxi's Lambert transfer of DEORBIT + LAND (DeorbitPlanner.TryLambert) aims at the entry point itself: at the top of the air, on the great circle to the site, the coast and the aimed corridor point short of it. It is taken when it costs little more than the horizontal burn and meets the air at the same flight path angle, for which the corridor was worked out (AimLambert); else the horizontal burn stands.
///  4. CHECK. One cold plan from the predicted start must reach the site: converged, with a small miss. A miss moves the aimed range by the miss and searches again, up to three times; a plan that cannot be had refuses the burn.
/// </summary>
public static partial class GuidanceWindow
{
    private const int AimMaxTries = 3;

    private static string AimTrimKeyNow()
        => FormattableString.Invariant($"{_s.OrbitalEntryTrimAoa}|{_s.OrbitalEntryAoaDeg:R}|{_s.OrbitalEntryAoaMinDeg:R}|{_s.OrbitalEntryAoaMaxDeg:R}|{_s.OrbitalEntryBellyAxis}|{_s.BellySideWanted()}");

    /// <summary>
    /// DEORBIT ONLY's burn for an orbital entry, aimed at the site. True when the burn is planned (DeorbitPlan set, as PlanEntryBurn sets it); false while it measures, searches or checks, with the request dropped so the next step comes back here, or when it refuses.
    /// </summary>
    private static bool PlanAimedEntryBurn(Vehicle vehicle, IParentBody parent, DeorbitRequest request)
    {
        var st = _s.OrbitalEntry;
        double now = SimNow();
        EntryConfig cfg = OrbitalEntryConfig();
        if (EntryRetro)
            cfg.TerminalRangeM = Math.Max(_s.OrbitalEntryRetroAimShortKm, 0.0) * 1000.0;
        else if (EntryFlips)
            cfg.TerminalRangeM = st.Bellyflop.TerminalRangeM > 0.0 ? st.Bellyflop.TerminalRangeM : BellyflopAimShortM();

        // ---- 1. measure ----
        int wantedSide = EntryBellySideWanted();
        string trimKey = AimTrimKeyNow();
        bool aeroStale = st.AimAero == null
                         || !ReferenceEquals(st.AimAero.Parent, parent)
                         || st.AimAero.MeshHash != KsaAeroSweep.LiveMeshHash(vehicle)
                         || (wantedSide >= 0 && st.AimAero.BellySide != wantedSide);
        if (aeroStale)
        {
            if (st.AimJob == null)
            {
                st.AimAero = null;
                st.AimTrim = null;
                st.AimTrimKey = "";
                if (!KsaAeroSweep.TryStart(vehicle, parent, now, out KsaAeroSweep.Job job, out string error, bellyFirst: true, bellySide: wantedSide))
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
            st.AimTrimKey = trimKey;
            st.AimStage = OrbitalEntryState.AimSteps.None;
        }
        else if (st.AimTrim == null || st.AimTrimKey != trimKey)
        {
            if (!TryBuildEntryTrim(st.AimAero, out EntryTrim trim, out _, out string why))
            {
                RefuseDeorbit(vehicle, "AIMED ENTRY: NO BELLY-FIRST MODEL (" + (why.Length > 0 ? why.ToUpperInvariant() : "NO TRIM") + ").");
                return false;
            }
            st.AimTrim = trim;
            st.AimTrimKey = trimKey;
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
            if (st.AimWorker is { Busy: true })
                return AimWaits("AIMED ENTRY: WAITING FOR PREVIOUS CHECK TO FINISH.");
            if (!SearchAimedDeparture(vehicle, parent, request, st, cfg, now, out string refusal))
            {
                RefuseDeorbit(vehicle, refusal);
                ClearAim(st);
                return false;
            }
            AimLambert(vehicle, parent, request, st, cfg);
            st.AimWorker ??= new Ksa3DofWorker();
            var slot = new OrbitalEntryState.AimCheckSlot();
            EntryModel model = st.AimModel;
            double e0 = st.AimStartE;
            double[] x0 = st.AimStartX;
            // The check has the iterations of two cold solves: it is solved once, not every two seconds.
            EntryConfig checkCfg = cfg.Clone();
            checkCfg.ColdIterations = Math.Max(cfg.ColdIterations * 2, 50);
            if (!st.AimWorker.TryRun(st, ct =>
            {
                var solved = _entryPlanner.Solve(model, checkCfg, e0, x0, null);
                if (!ct.IsCancellationRequested)
                    Volatile.Write(ref slot.Result, solved);
            }))
            {
                return AimWaits("AIMED ENTRY: WAITING FOR PLANNER WORKER.");
            }
            st.AimCheck = slot;
            st.AimStage = OrbitalEntryState.AimSteps.Check;
        }

        // ---- 4. check ----
        if (st.AimStage == OrbitalEntryState.AimSteps.Check)
        {
            // Snapshot Busy before reading error/result because the worker publishes them before clearing _busy.
            bool workerBusy = st.AimWorker?.Busy ?? false;
            string workerError = st.AimWorker?.TakeError(st) ?? "";
            if (workerError.Length > 0)
            {
                GuidanceLog.Info(vehicle, "Aimed entry check failed on worker: " + workerError);
                RefuseDeorbit(vehicle, "AIMED ENTRY: PLANNER FAILED.");
                ClearAim(st);
                return false;
            }
            EntryPlanner.Result check = st.AimCheck != null ? Interlocked.Exchange(ref st.AimCheck.Result, null) : null;
            if (check == null)
            {
                if (!workerBusy)
                {
                    RefuseDeorbit(vehicle, "AIMED ENTRY: PLANNER ENDED WITHOUT A RESULT.");
                    ClearAim(st);
                    return false;
                }
                return AimWaits(FormattableString.Invariant($"AIMED ENTRY: CHECKING THE PLAN FROM {st.AimStartRangeM / 1000:F0} KM OUT."));
            }
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
            // A plan that reaches the site is enough to commit the burn: the flight's own re-solves refine it. A defect a few times the flight gate (2026-11-21: 0.084 against 0.05, miss 0 km, three times over) is not a reason to refuse.
            bool reaches = Math.Abs(p.TerminalRangeErrorM) <= cfg.GateRangeM && p.MaxDefect <= 5.0 * cfg.GateDefect;
            if (!reaches)
            {
                if (st.AimTries >= AimMaxTries)
                {
                    RefuseDeorbit(vehicle, FormattableString.Invariant($"AIMED ENTRY: NO PLAN REACHES THE SITE ({p.TerminalRangeErrorM / 1000:F0} KM MISS AFTER {st.AimTries} TRIES)."));
                    ClearAim(st);
                    return false;
                }
                // Where the check starts against where the search aimed: a burn that does not put the site where the search meant to, by more than the gate, is moved by that error. The plan's miss says nothing then - a cold plan from outside the corridor meets the site by breaking its dynamics, miss 0 (2026-11-25: three identical tries, 220 km short each time). Otherwise, range left at the hand-over: start that much closer; overshot: that much further.
                double startErrorM = (st.AimTargetM - st.AimShiftM) - st.AimStartRangeM;
                bool startOff = Math.Abs(startErrorM) > cfg.GateRangeM;
                st.AimShiftM += startOff ? startErrorM : -p.TerminalRangeErrorM;
                st.AimStage = OrbitalEntryState.AimSteps.Search;
                return AimWaits(startOff
                    ? FormattableString.Invariant($"AIMED ENTRY: THE BURN PUT THE SITE {Math.Abs(startErrorM) / 1000:F0} KM {(startErrorM > 0 ? "CLOSER" : "FURTHER")} THAN AIMED; MOVING IT.")
                    : "AIMED ENTRY: MOVING THE BURN BY THE PLAN'S MISS.");
            }

            // ---- commit ----
            DeorbitPlan? plan = st.AimVelocity is { } aimed
                ? EntryDeorbitPlanner.PlanAtVelocity(request, st.AimDeparture, aimed, out string why, _s.DeorbitEntryMeasuredPreparation)
                : EntryDeorbitPlanner.PlanAt(request, st.AimDeparture, out why, _s.DeorbitEntryMeasuredPreparation);
            if (plan is not { } entry)
            {
                GuidanceLog.Info(vehicle, FormattableString.Invariant(
                    $"Aimed entry burn refused at commit: {why} Engines in the request {request.Engines.Length}, throttle control {KsaEnginePerf.GetThrottleControlStatus(vehicle)}, exhaust velocity {request.ExhaustVelocity:F0} m/s, bound {request.Source.IsBound()}, departure in {st.AimDeparture - now:F0} s."));
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
                $"Aimed entry burn planned: departure in {entry.DepartureTime - now:F1} s, delta-v {entry.DeltaV.Length():F2} m/s ({(st.AimVelocity.HasValue ? "Lambert to the entry point" : "horizontal")}), entry interface in {entry.ArrivalTime - now:F1} s, plan miss {p.TerminalRangeErrorM / 1000:F2} km."));
            st.AimWorker?.Dispose();
            st.AimWorker = null;
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
        st.AimWorker?.Dispose();
        st.AimWorker = null;
        st.AimVelocity = null;
        st.AimJob = null;
        st.AimStage = OrbitalEntryState.AimSteps.None;
        st.AimCheck = null;
        st.AimShiftM = 0;
        st.AimTries = 0;
    }

    /// <summary>The most the Lambert burn may cost over the horizontal one: this share of it, and at least <see cref="AimLambertExtraLeastMs"/>. Own choice: the entry's bank reversals take out a crossrange for free, so the burn should only pay for what is cheap.</summary>
    private const double AimLambertExtraShare = 0.25;
    private const double AimLambertExtraLeastMs = 10.0;

    /// <summary>The flight path angle at the top of the air may differ from the horizontal burn's by this much, deg: the corridor was worked out for that one.</summary>
    private const double AimLambertGammaToleranceDeg = 0.5;

    /// <summary>
    /// Step 3b: with the departure (AimDeparture) and flight time the search found, the Lambert transfer to the entry point on the great circle to the site, the coast and the aimed corridor point short of it. Taken, with the start of the air predicted again from where it arrives, when its delta-v is within <see cref="AimLambertExtraShare"/> of the horizontal burn's and its flight path angle at the top of the air within <see cref="AimLambertGammaToleranceDeg"/>; logged either way.
    /// </summary>
    private static void AimLambert(Vehicle vehicle, IParentBody parent, DeorbitRequest request, OrbitalEntryState st, EntryConfig cfg)
    {
        st.AimVelocity = null;
        double td = st.AimDeparture;
        if (!EntryDeorbitPlanner.TryArrival(request, td, out double arrival, out StateVectors arrived)
            || !EntryDeorbitPlanner.TryHorizontalBurn(request, td, out double3 dvHorizontal))
            return;
        double R0 = parent.MeanRadius;
        double rInterface = arrived.PositionCci.Length();
        // The aimed range ahead of the start of the air, plus the coast from the top of the air to it: the distance of the entry point short of the site, as the search measured it - over the ground, in the planet's turning frame. So it is laid back from where the site is when the craft arrives at the top of the air, where the two frames agree, as SiteGeometry and the check measure it. (Laid back from where the site is when the air starts, as it was, the entry point sat the planet's turn during the coast too far east: 2026-11-25, 855 s of coast put the site 220-240 km closer than aimed and every check refused.)
        double shortM = st.AimTargetM + st.AimCoastRangeM;
        if (!double.IsFinite(shortM) || shortM <= 0.0)
            return;
        double3 site = double3.Normalize(SiteDirCciAt(parent, arrival - SimNow()));
        StateVectors source = request.Source.GetStateVectorsAt(new UniverseTime(td));
        double3 orbitNormal = double3.Normalize(double3.Cross(source.PositionCci, source.VelocityCci));
        double3 normal = double3.Cross(arrived.PositionCci, site);
        if (normal.Length() < 1.0)
            return;
        normal = double3.Normalize(normal);
        if (double3.Dot(normal, orbitNormal) < 0.0)
            normal = -normal;
        double3 endpoint = RotateAbout(site, normal, -shortM / R0) * rInterface;
        if (!DeorbitPlanner.TryLambert(parent.Mu, source.PositionCci, endpoint, arrival - td, orbitNormal, out double3 v1, out double3 v2))
        {
            GuidanceLog.Info(vehicle, "Aimed entry: no Lambert transfer to the entry point; the horizontal burn stands.");
            return;
        }
        double3 dvLambert = v1 - source.VelocityCci;
        double extra = dvLambert.Length() - dvHorizontal.Length();
        double allowed = Math.Max(AimLambertExtraLeastMs, AimLambertExtraShare * dvHorizontal.Length());
        double Gamma(double3 r, double3 v) => Math.Asin(Math.Clamp(double3.Dot(double3.Normalize(r), v.NormalizeOrZero()), -1.0, 1.0)) * 180.0 / Math.PI;
        double gammaHorizontal = Gamma(arrived.PositionCci, arrived.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), arrived.PositionCci));
        double gammaLambert = Gamma(endpoint, v2 - double3.Cross(parent.GetAngularVelocityCci(), endpoint));
        bool take = extra <= allowed && Math.Abs(gammaLambert - gammaHorizontal) <= AimLambertGammaToleranceDeg;
        string verdict = FormattableString.Invariant(
            $"Aimed entry Lambert to the entry point {shortM / 1000:F0} km short of the site: delta-v {dvLambert.Length():F1} m/s against {dvHorizontal.Length():F1} horizontal (allowed +{allowed:F1}), flight path angle {gammaLambert:F2} against {gammaHorizontal:F2} deg");
        if (!take)
        {
            GuidanceLog.Info(vehicle, verdict + "; the horizontal burn stands.");
            return;
        }
        // The start of the air again, from where the Lambert burn arrives.
        SiteGeometry(parent, endpoint, v2, arrival, out double down, out double cross, out double3 rel);
        EntryModel model = st.AimModel.WithRotationAt(parent, endpoint, rel);
        double gamma = Math.Asin(Math.Clamp(double3.Dot(rel.NormalizeOrZero(), double3.Normalize(endpoint)), -1.0, 1.0));
        var start = EntryPlanner.PredictStart(model, cfg, endpoint.Length(), rel.Length(), gamma, down);
        if (start == null)
        {
            GuidanceLog.Info(vehicle, verdict + "; its entry does not reach the start drag, the horizontal burn stands.");
            return;
        }
        GuidanceLog.Info(vehicle, verdict + FormattableString.Invariant($"; taken, crossrange {st.AimCrossM / 1000:F1} -> {cross * R0 / 1000:F1} km."));
        st.AimVelocity = v1;
        st.AimModel = model;
        st.AimStartE = start.Value.E;
        st.AimStartX = start.Value.X;
        st.AimStartRangeM = start.Value.X[EntryDynamics.IS] * R0;
        st.AimCrossM = cross * R0;
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
        // The coast from the interface turns with the body: its latitude and heading there.
        model = model.WithRotationAt(parent, refState.PositionCci, refRel);
        st.AimModel = model;
        double refGamma = Math.Asin(Math.Clamp(double3.Dot(refRel.NormalizeOrZero(), double3.Normalize(refState.PositionCci)), -1.0, 1.0));
        var coast = EntryPlanner.PredictStart(model, cfg, refState.PositionCci.Length(), refRel.Length(), refGamma, 0.0);
        if (coast == null)
        {
            refusal = "AIMED ENTRY: THE ENTRY DOES NOT REACH THE START DRAG.";
            return false;
        }
        double coastRangeM = -coast.Value.X[EntryDynamics.IS] * R0;
        double coastS = coast.Value.SecondsAhead;
        // Flown from the predicted start of the air itself, as the plans' own corridor is (EntryPlanner.ReachBand).
        (double shortM, double longM) = EntryPlanner.Corridor(model, cfg, coast.Value.E, coast.Value.X);
        shortM += cfg.TerminalRangeM;
        longM += cfg.TerminalRangeM;
        double fraction = Math.Clamp(_s.OrbitalEntryAimFraction, 0.0, 1.0);
        double target = shortM + fraction * (longM - shortM) + st.AimShiftM;
        st.AimShortM = shortM;
        st.AimLongM = longM;
        st.AimCoastRangeM = coastRangeM;
        st.AimCoastS = coastS;
        st.AimTargetM = target;
        double maxCross = Math.Max(_s.OrbitalEntryAimMaxCrossKm, 1.0) * 1000.0;

        // f(t) = range ahead at the start of the air less the aim: falls as the burn moves later along a pass.
        bool Eval(double t, out double f, out double crossM)
        {
            f = crossM = double.NaN;
            if (!EntryDeorbitPlanner.TryArrival(request, t, out double arrival, out StateVectors s))
                return false;
            // The site where it is at the arrival, as the coast range is measured: over the ground, in the turning frame (see AimLambert).
            SiteGeometry(parent, s.PositionCci, s.VelocityCci, arrival, out double down, out double cross, out _);
            crossM = cross * R0;
            f = down * R0 - coastRangeM - target;
            return down > 0.0;
        }

        double horizon = Math.Max(_s.OrbitalEntryAimHorizonH, 0.1) * 3600.0;
        double step = Math.Clamp(period / 180.0, 10.0, 60.0);
        double bestCross = double.PositiveInfinity;
        // Over the passes close enough to the track: the most and least range ahead the site ever sat at the start of the air, to say why none lined up.
        double aheadMax = double.NegativeInfinity, aheadMin = double.PositiveInfinity;
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
            aheadMax = Math.Max(aheadMax, f + target);
            aheadMin = Math.Min(aheadMin, f + target);
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
                model = model.WithRotationAt(parent, s.PositionCci, rel);
                st.AimModel = model;
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
        if (!double.IsFinite(bestCross))
            refusal = "AIMED ENTRY: NO BURN IN THE HORIZON MEETS THE AIR.";
        else if (Math.Abs(bestCross) > maxCross)
            refusal = FormattableString.Invariant($"AIMED ENTRY: NO PASS IN {_s.OrbitalEntryAimHorizonH:F0} H COMES WITHIN {maxCross / 1000:F0} KM OF THE SITE (CLOSEST {Math.Abs(bestCross) / 1000:F0} KM).");
        else
            refusal = FormattableString.Invariant(
                $"AIMED ENTRY: PASSES COME WITHIN {maxCross / 1000:F0} KM, BUT ON THEM THE SITE IS {aheadMin / 1000:F0}-{aheadMax / 1000:F0} KM AHEAD AT THE AIR, NEVER THE AIMED {target / 1000:F0} KM (CORRIDOR {shortM / 1000:F0}-{longM / 1000:F0}, COAST {coastRangeM / 1000:F0} KM).");
        GuidanceLog.Info(vehicle, FormattableString.Invariant(
            $"Aimed entry search failed: corridor {shortM / 1000:F0}-{longM / 1000:F0} km, aimed {target / 1000:F0} km, coast {coastS:F0} s and {coastRangeM / 1000:F0} km, closest crossrange {Math.Abs(bestCross) / 1000:F1} km, site ahead {aheadMin / 1000:F0}-{aheadMax / 1000:F0} km on passes within {maxCross / 1000:F0} km."));
        return false;
    }
}
