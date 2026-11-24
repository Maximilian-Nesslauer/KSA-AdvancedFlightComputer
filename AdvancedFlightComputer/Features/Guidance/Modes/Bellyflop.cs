#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;
using AdvancedFlightComputer.Guidance.Scvx.Bellyflop;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
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
            LimitBellyflopBankToRollTrim(vehicle, st, cfg, mach, aNow);
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

    /// <summary>Natural frequency of the critically damped filter on bellyflop's commanded angles, rad/s (BellyflopAttitude): a step settles in about 2 s, and a turn the plan starts at 7 deg/s asks at most about 5 deg/s^2, inside what the flaps spare at Mach 5.</summary>
    private const double BellyflopCommandOmega = 2.0;

    /// <summary>
    /// THE BANK THE FLAPS CAN HOLD. Bellyflop's bank is the airflow's sideslip on the belly, and the air rolls the craft harder the more of it there is; past a bank the flaps cannot hold that roll, and the attitude is held on the RCS or not at all. A Starship banked to 48 deg 10 km up, its flaps at their stops against 480 kN m of roll and the RCS firing at 98 kg/s; the RCS ran dry, the bank passed the belly-first tables' 45 deg, and it tumbled (2026-11-24). So each plan's bank limit is held to the largest bank, either way, at which the flaps can hold the body's rolling moment with RollTrimShare of their reach, read off the belly-first tables at the Mach and pitch flown now: the same tables and rule the entry trims its pitch on (RetrimWithFlaps), on the roll axis. Never under BankHoldFloorDeg; unchanged where the tables have no flaps.
    /// </summary>
    private static void LimitBellyflopBankToRollTrim(Vehicle vehicle, OrbitalEntryState st, BellyflopConfig cfg, double mach, double pitchRad)
    {
        var bf = st.Bellyflop;
        double hold = BellyflopRollTrimBankDeg(st.Aero, mach, pitchRad);
        if (!double.IsFinite(hold))
            return;
        double limit = Math.Clamp(hold, BankHoldFloorDeg, cfg.BankMaxDeg);
        if (!double.IsFinite(bf.BankHoldDeg) || Math.Abs(limit - bf.BankHoldDeg) >= 5.0)
        {
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Bellyflop: the flaps hold the roll to {hold:F1} deg of bank at Mach {mach:F2}, pitch {pitchRad * 180.0 / Math.PI:F0} deg; plans bank to {limit:F1} (BELLYFLOP BANK {cfg.BankMaxDeg:F0})."));
            bf.BankHoldDeg = limit;
        }
        cfg.BankMaxDeg = limit;
    }

    /// <summary>The largest bank, deg, at which the flaps hold the body's rolling moment both ways with RollTrimShare of their reach (belly-first tables, BankHoldStepDeg apart); NaN without flap tables.</summary>
    internal static double BellyflopRollTrimBankDeg(KsaAeroSweep.Result aero, double mach, double pitchRad)
    {
        BodyAeroTable body = aero?.BellyBodyMoment;
        var surfaces = aero?.Surfaces;
        if (body == null || surfaces == null || surfaces.Length == 0)
            return double.NaN;
        foreach (var t in surfaces)
            if (t.BellyMoment.Length != t.DeflectionDeg.Length || t.BellyMoment.Length == 0)
                return double.NaN;
        const double deg = Math.PI / 180.0;
        double m = Math.Clamp(mach, body.MachMin, body.MachMax);
        double a = Math.Clamp(pitchRad, KsaAeroSweep.BellyAlphaTableMinDeg * deg, KsaAeroSweep.BellyAlphaTableMaxDeg * deg);
        double held = 0.0;
        for (double b = BankHoldStepDeg; b <= KsaAeroSweep.BellyBetaTableMaxDeg + 1e-9; b += BankHoldStepDeg)
        {
            foreach (double sign in new[] { 1.0, -1.0 })
            {
                double beta = sign * b * deg;
                body.Coefficients(m, a, beta, out double roll, out _, out _);
                double down = 0.0, up = 0.0;
                foreach (var t in surfaces)
                {
                    double lo = 0.0, hi = 0.0;
                    foreach (BodyAeroTable table in t.BellyMoment)
                    {
                        table.Coefficients(m, a, beta, out double r, out _, out _);
                        lo = Math.Min(lo, r);
                        hi = Math.Max(hi, r);
                    }
                    down += lo;
                    up += hi;
                }
                // Held: the flaps' share of their reach brings the body's roll to zero.
                if (!(roll + RollTrimShare * down <= 0.0 && 0.0 <= roll + RollTrimShare * up))
                    return held;
            }
            held = b;
        }
        return held;
    }

    /// <summary>The share of the flaps' rolling reach a plan's bank may use (LimitBellyflopBankToRollTrim): the rest is for the pitch they trim at the same time and for the attitude feedback.</summary>
    private const double RollTrimShare = 0.6;
    private const double BankHoldStepDeg = 2.5;
    private const double BankHoldFloorDeg = 5.0;

    /// <summary>
    /// The oldest standby plan the hand-over from bellyflop takes, s: under the 3 s after which the standby starts again (SixDofStandbyStaleRestartS), over the 0.5 s the glide's takes. While bellyflop flies the craft, touchdown's standby re-solves are refused now and then on small rate and attitude errors at their first node (1.1 to 8 times the gate), so its plan is seldom under 0.5 s old: in the 2026-11-24 15:48 flight it converged three times and was never handed the craft. Once it flies, touchdown re-solves from the craft's state at once.
    /// </summary>
    private const double BellyflopHandoverPlanAgeS = 2.5;

    /// <summary>The flip's share of touchdown's burn-time seed, s: 90 deg at about 15 deg/s.</summary>
    private const double BellyflopFlipSeedS = 6.0;

    /// <summary>
    /// Bellyflop's plan as a path for touchdown guidance's cold seed (ColdSeed6Dof.BuildAlongPath), as the glide's is (EntrySeedPath): the coast from now to the ignition touchdown seeds (SixDofCoastSeedS) along bellyflop's plan, its position and velocity in the site frame and its nose as the body z axis wanted, so touchdown's coast holds the attitude bellyflop flies; the burn a straight braking burn from the coast's end, thrust against the velocity, slowing evenly to rest over the burn's seed, which the seed's own end correction lays onto the target. Without it touchdown planned a coast of its own, belly first but wandering 60 to 106 deg off vertical; while bellyflop flew the craft every standby re-solve was refused on the attitude at its first node, and the standby never had a plan when the hand-over came due (harness, Starship, 2026-11-24). Null while bellyflop has no plan. The plan and model are immutable, so the worker can sample them.
    /// </summary>
    internal static AdvancedFlightComputer.Guidance.Scvx.SixDof.SeedPath BellyflopSeedPath(double now)
    {
        var st = _s.OrbitalEntry;
        var bf = st.Bellyflop;
        BellyflopPlan plan = bf.Plan;
        BellyflopModel model = bf.Model;
        if (st.Phase != OrbitalEntryPhase.Bellyflop || plan == null || model == null)
            return null;
        double coast = Math.Max(_s.SixDofCoastSeedS, 0.0);
        double burn = Math.Max(_s.SixDofEntrySigmaSeed, SixDofStandbyMinSeedS);
        double d = BellyflopAimShortM();
        double3 bearing = bf.BearingLocal;
        double ignitionTime = Math.Min(now + coast, plan.EndTime);
        // The plan's states are about its aim point, BELLYFLOP AIM SHORT short of the site along the bearing: back to the site.
        void Sample(double t, Span<double> x)
        {
            plan.StateAt(Math.Clamp(t, plan.StartTime, plan.EndTime), x);
            x[0] -= d * bearing.X;
            x[1] -= d * bearing.Y;
        }
        var xi = new double[6];
        Sample(ignitionTime, xi);
        double vi = Math.Sqrt(xi[3] * xi[3] + xi[4] * xi[4] + xi[5] * xi[5]);
        return (int phase, double f, Span<double> rv, Span<double> axis, out double thrustN) =>
        {
            thrustN = 0.0;
            if (phase == 0)
            {
                Span<double> x = stackalloc double[6];
                double t = now + f * coast;
                Sample(t, x);
                x.CopyTo(rv);
                var (a, b, p) = plan.ControlAt(Math.Min(t, plan.EndTime));
                Span<double> xa = stackalloc double[6];
                plan.StateAt(Math.Clamp(t, plan.StartTime, plan.EndTime), xa);
                if (!model.Axes(xa, a, b, p, out V3 nose, out _, out _, out _))
                    return false;
                axis[0] = nose.X; axis[1] = nose.Y; axis[2] = nose.Z;
                return true;
            }
            // The braking burn: v(f) = v_i (1 - f), r(f) = r_i + v_i burn (f - f^2 / 2), the thrust against the velocity.
            double g = 1.0 - f, s = burn * (f - 0.5 * f * f);
            for (int i = 0; i < 3; i++)
            {
                rv[i] = xi[i] + xi[3 + i] * s;
                rv[3 + i] = xi[3 + i] * g;
            }
            if (vi > 1e-3)
                for (int i = 0; i < 3; i++) axis[i] = -xi[3 + i] / vi;
            else
            {
                axis[0] = 0; axis[1] = 0; axis[2] = 1;
            }
            // Per unit of mass: the builder clamps it into the engines' range and corrects it for the end; the craft's weight and the braking.
            thrustN = Math.Max(model.Mass, 1.0) * (vi / burn + 9.81);
            return true;
        };
    }

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
        // THE LANDING GROUP chosen on LAND, switched on before touchdown's standby builds its model, as GLIDE + BURN does at its engage (PrepareSelectedLandingEngines): ORBITAL ENTRY never applied it, so bellyflop handed touchdown every engine that was active, a Starship's three sea-level Raptors whose 39 % floor is more than twice its weight (2026-11-24). Bellyflop flies on while the engines are switched; a group that cannot be confirmed is said once, and touchdown plans on the engines active then.
        if (!bf.TouchdownStandby && SelectedLandingEngines != null && !bf.LandingGroupSettled)
        {
            LandingEnginePreparation engines = PrepareSelectedLandingEngines(vehicle, now, false, out string groupWhy);
            if (engines == LandingEnginePreparation.Waiting)
                return;
            if (engines == LandingEnginePreparation.Refused)
                GuidanceLog.Info(vehicle, $"Touchdown guidance: the landing group was not switched on ({groupWhy}); it plans on the engines active now.");
            bf.LandingGroupSettled = true;
        }
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
        // Not before the flaps' tables are in (TouchdownWaitsForSurfaces): bellyflop flies on meanwhile.
        if (!bf.TouchdownStandby)
        {
            EnsureBoostbackAero(vehicle, parent);
            if (TouchdownWaitsForSurfaces(vehicle, out string waitWhy))
            {
                if (!bf.SurfaceWaitLogged)
                {
                    GuidanceLog.Info(vehicle, "Touchdown guidance: " + waitWhy.ToLowerInvariant() + " before it plans the flip; bellyflop flies on.");
                    bf.SurfaceWaitLogged = true;
                }
                return;
            }
        }
        double decel = Math.Max(BellyflopBurnSeedThrottle * accel - gravity, 0.3 * (accel - gravity));
        // The burn has the flip in it: the coast's seed holds the craft belly first, so the burn must turn it 90 deg as well as stop it. Seeded with the stop alone (2 s for the Starship at 51 m/s), every cold solve had to turn the craft in the burn's last intervals at 2 rad/s, lengthened the burn 0.2 s an iteration, and was restarted as stalled before it got there (2026-11-24).
        double burn = Math.Max(speed / decel, SixDofStandbyMinSeedS) + BellyflopFlipSeedS;
        double sink = Math.Max(-x14[5], 1.0);
        double coast = Math.Max((x14[2] - sink * BellyflopFlipSeedS - 0.5 * speed * (burn - BellyflopFlipSeedS)) / sink, 0.0);
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
        // DUE BEFORE TOUCHDOWN'S PLAN TURNS AWAY from the attitude bellyflop flies (SixDofTurnAwayDue: 1 s before it is 5 deg off), or at the latest THREE-DOF TAKEOVER BEFORE IGNITION before touchdown's ignition or THREE-DOF TAKEOVER BEFORE IMPACT before bellyflop's ground. With the craft's whole-sphere aero, touchdown's plans turn the craft for the flip early, on the air and the flaps; held to the ignition rule alone, its re-solves from the craft bellyflop held belly first drifted off and were refused from 750 m, and it never took the craft (harness, Starship, 2026-11-24). Before that, handed the craft at its first plan with no air moment in its model, it lost the belly-first attitude in the coast (harness, 2026-11-24).
        if (bf.TouchdownHandoff == null && SixDofStandbyReady(now, BellyflopHandoverPlanAgeS) && _s.Guidance?.Published is { } td)
        {
            double ignition = td.HasCoast ? td.IgnitionTime : now;
            double turnAway = SixDofTurnAwayDue(x14, now);
            double due = Math.Min(Math.Min(ignition - Math.Max(_s.ThreeDofTakeoverBeforeIgnitionS, 0.0), plan.EndTime - Math.Max(_s.ThreeDofTakeoverBeforeImpactS, 0.0)), turnAway);
            if (now >= due)
                bf.TouchdownHandoff = FormattableString.Invariant($"HANDOVER FROM BELLYFLOP: TOUCHDOWN AT {x14[2]:F0} m, IGNITION IN {ignition - now:F1} s, {plan.EndTime - now:F0} s FROM BELLYFLOP'S GROUND, ") + (now >= turnAway ? "ITS PLAN TURNING AWAY." : "AT THE LATEST.");
        }
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
        if (!SixDofStandbyReady(SimNow(), BellyflopHandoverPlanAgeS))
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
        // THE COMMAND IS SMOOTH BY CONSTRUCTION, as entry guidance's glide learned (ThreeDof.cs): a critically damped second-order filter over the plans (BellyflopCommandOmega), from the angles flown at the take-over and at rest, so the command, its rate and its acceleration are all continuous: a new plan each second does not step the target, and a turn the plan starts does not step its rate. Read 2/omega ahead, the filter's lag on a ramp, so the plan's own turns come through on time. Flown straight off the newest plan, the fins stood at 13-32 deg and saturated 27-38 % of the time, against 3-5 deg and 3-6 % in the entry; through a first-order filter the plan's first turn, a 90 deg yaw at the take-over, still stepped the rate to 7 deg/s, and the craft rocked 33 deg in bank for 15 s with the flaps at their stops and the RCS at 80-116 kg/s (2026-11-24).
        double lead = 2.0 / BellyflopCommandOmega;
        var (ta, tb, tp) = plan.ControlAt(now + lead);
        if (!double.IsFinite(bf.CmdAt))
        {
            const double toRad = Math.PI / 180.0;
            (bf.CmdAlpha, bf.CmdBeta, bf.CmdPsi) = double.IsFinite(bf.PitchDeg) && double.IsFinite(bf.BankDeg) && double.IsFinite(bf.YawDeg)
                ? (bf.PitchDeg * toRad, bf.BankDeg * toRad, bf.YawDeg * toRad) : (ta, tb, tp);
            bf.CmdAlphaRate = bf.CmdBetaRate = bf.CmdPsiRate = 0.0;
            bf.CmdAt = now;
        }
        double span = Math.Clamp(now - bf.CmdAt, 0.0, 1.0);
        bf.CmdAt = now;
        const double maxStep = 0.02;
        double w = BellyflopCommandOmega;
        for (double done = 0.0; done < span - 1e-9;)
        {
            double h = Math.Min(maxStep, span - done);
            done += h;
            // Semi-implicit: the rate first, then the angle with the new rate.
            bf.CmdAlphaRate += h * (w * w * (ta - bf.CmdAlpha) - 2.0 * w * bf.CmdAlphaRate);
            bf.CmdBetaRate += h * (w * w * (tb - bf.CmdBeta) - 2.0 * w * bf.CmdBetaRate);
            bf.CmdPsiRate += h * (w * w * (tp - bf.CmdPsi) - 2.0 * w * bf.CmdPsiRate);
            bf.CmdAlpha += h * bf.CmdAlphaRate;
            bf.CmdBeta += h * bf.CmdBetaRate;
            bf.CmdPsi += h * bf.CmdPsiRate;
        }
        double a = bf.CmdAlpha, b = bf.CmdBeta, p = bf.CmdPsi;
        if (!model.Axes(x14.AsSpan(0, 6), a, b, p, out V3 nose, out _, out V3 belly, out _))
            return false;
        noseCci = frame.VecToCci(KsaBellyflopSetup.ToDouble3(nose)).NormalizeOrZero();
        bellyCci = frame.VecToCci(KsaBellyflopSetup.ToDouble3(belly)).NormalizeOrZero();
        return noseCci.Length() > 0.5 && bellyCci.Length() > 0.5;
    }
}
