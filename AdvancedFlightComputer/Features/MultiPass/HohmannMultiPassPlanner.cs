using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Real-K multi-pass planner for stock Hohmann / interplanetary transfer
/// departures. Schedules N burns at the SAME CCI direction P (the Lambert
/// burn point) so each chained orbit's periapsis stays at P and the
/// vehicle returns to P at every scheduled firing time.
///
/// Phasing constraint (the only structural requirement):
///   sum(K_i) is an integer
/// where K_k = post-burn-k chained-orbit-period / T_park (real, &gt; 1
/// for prograde burns). Individual K_k need NOT be integer; only the sum
/// must be, so that vehicle in parking orbit at times[0] is at the same
/// CCI direction P as vehicle in parking orbit at T_final (parking-orbit
/// periodicity).
///
/// Algorithm:
///   1. <see cref="Splitter.Allocate"/> splits total dV across N passes
///      per the user's <see cref="SplitMode"/> (EqualBurnTime default
///      keeps finite-burn arc length uniform; EqualDv is the simpler
///      uniform-magnitude alternative).
///   2. Cumulative per-pass dV -&gt; v_p target sequence -&gt; real K
///      schedule via vis-viva.
///   3. Initial plan (startPassIndex=0): round sum(K) to the nearest
///      integer M, adjust the LAST prior K to absorb the rounding
///      residual, schedule times[0] = T_final - M * T_park.
///   4. Recompute (startPassIndex&gt;0): times[0] = vehicle's next
///      chained-periapsis time (physical, computed from vehicle.Orbit);
///      target sum = (T_final - times[0]) / T_park (real); adjust last
///      prior K to match.
///   5. Each pass adds two LOCAL-VLF components at periapsis: an X kick
///      sets |v_post_k| to the K-scheduled speed; a Y kick rotates the
///      plane by theta_k around the periapsis radial. dV.Z = 0 for
///      priors, so periapsis CCI position is preserved.
///   6. theta_k is Lagrange-optimal: proportional to delta_R_k /
///      (v_pre_k * v_post_k), minimising the total plane-change dV cost.
///   7. Final pass at T_final adds the radial D.Z residual of the
///      Lambert dV.
///
/// Same-parent vs cross-parent: identical scheduling logic. The only
/// difference is in <see cref="ComputeVpTarget"/>: cross-parent uses
/// v_inf (hyperbolic excess), same-parent uses the target apoapsis radius.
/// </summary>
internal static class HohmannMultiPassPlanner
{
    // Stock OrbitalTransfers.BuildFlightPlan uses 5 patches; we bump to 8
    // so a chained pre-final orbit with a near-SOI apoapsis still has
    // enough patch headroom for the post-burn hyperbolic escape + any
    // sibling-body close-approach detection.
    private const int FlightPlanPatchLimit = 8;
    private const int FlightPlanPolynomialOrder = 8;

    // Leaves a 5% gap to parent SOI so a chained intermediate orbit that
    // brushes against SOI in a noisy patched-conic propagation does not
    // silently escape. Above ~95% the FlightPlan's Escape detection
    // gets unreliable.
    private const double SoiEnvelopeFraction = 0.95;

    // Don't schedule pass 0 closer than this to <c>now</c>; gives the
    // user a few seconds to actually warp / engage Auto.
    private const double EarliestPassMarginSec = 30.0;

    // Lower bound on K_0 (the first prior's post-burn period multiple).
    // K = 1 exactly is a zero-dV burn (post = parking). The 0.01 margin
    // keeps the chained orbit a meaningfully separate orbit from parking,
    // so the planner doesn't propose burns that contribute essentially
    // nothing to the chain (which would also break the GetTimeSincePeriapsis
    // logic on the near-circular result).
    private const double KFloor = 1.01;

    // 64 steps over [minTransit, maxTransit] gives sub-1% transit resolution
    // for typical Earth-Luna and Earth-Mars windows. Lambert dV minima are
    // smooth enough that finer scans rarely shift the picked candidate;
    // coarser (32) would risk aliasing in long-window targets.
    private const int TransitScanSteps = 64;

    /// <summary>Locked inputs from the stock-Lambert porkchop entry that
    /// the planner needs to schedule a multi-pass chain.
    ///
    /// Two flavours:
    ///   * Cross-parent (e.g. Earth -> Mars): the post-burn orbit is
    ///     hyperbolic relative to the parking-orbit parent. Final pass
    ///     targets v_p_target = sqrt(VInfMs^2 + 2 mu / r_p).
    ///   * Same-parent (e.g. Earth -> Luna): the post-burn orbit is
    ///     bound; we target a specific apoapsis. Final pass targets
    ///     v_p_target = sqrt(mu * (2/r_p - 1/((r_p + ApoTargetRadiusMeters)/2))).
    /// </summary>
    internal readonly record struct HohmannPlanInput(
        IOrbiter Target,
        SimTime TFinal,
        double3 DFinalVlf,
        bool IsCrossParent,
        double VInfMs,
        double ApoTargetRadiusMeters);

    /// <summary>Plans the N-pass chain. Slices into the original K-sequence
    /// via <paramref name="startPassIndex"/>: 0 for the initial UI plan,
    /// >0 when invoked from <see cref="HohmannTransferIntent.RecomputePass"/>
    /// for the remaining passes of an active execution.
    ///
    /// <paramref name="parkingPeriodSec"/> MUST be the original parking orbit's
    /// period (locked at intent creation). Reading vehicle.Orbit.Period at
    /// recompute time would return the chained orbit's period, which would
    /// double-count K and put subsequent passes at the chained orbit's
    /// apoapsis instead of periapsis.</summary>
    public static PassPreviewResult Plan(
        Vehicle source,
        HohmannPlanInput input,
        int totalPassCount,
        int startPassIndex,
        double parkingPeriodSec,
        SequenceBurnState vehicleState,
        SimTime now,
        SplitMode mode)
    {
        if (source?.Orbit?.Parent == null)
            return Fail($"vehicle '{source?.Id ?? "?"}' has no orbit parent", PassPlanFailure.Other);
        if (totalPassCount < 1)
            return Fail($"vehicle '{source.Id}': totalPassCount {totalPassCount} < 1", PassPlanFailure.Other);
        if (startPassIndex < 0 || startPassIndex >= totalPassCount)
            return Fail($"vehicle '{source.Id}': startPassIndex {startPassIndex} out of range [0,{totalPassCount})", PassPlanFailure.Other);
        if (!(parkingPeriodSec > 0.0))
            return Fail($"vehicle '{source.Id}': parkingPeriodSec {parkingPeriodSec} must be positive", PassPlanFailure.Other);

        Orbit currentOrbit = source.Orbit;
        if (!currentOrbit.IsBound())
            return Fail($"vehicle '{source.Id}': current orbit is unbound", PassPlanFailure.Other);

        double mu = currentOrbit.Mu;
        double rp = currentOrbit.Periapsis;
        if (!(rp > 0.0))
            return Fail($"vehicle '{source.Id}': current orbit has non-positive periapsis", PassPlanFailure.Other);

        double tPark = parkingPeriodSec;

        double soiLimit = currentOrbit.Parent.SphereOfInfluence * SoiEnvelopeFraction;
        if (!(soiLimit > 0.0))
            return Fail($"vehicle '{source.Id}': parent '{currentOrbit.Parent.Id}' has no SOI", PassPlanFailure.Other);

        int remainingCount = totalPassCount - startPassIndex;

        if (input.DFinalVlf.LengthSquared() < 1.0)
            return Fail($"vehicle '{source.Id}': stock Hohmann dV is zero", PassPlanFailure.Other);

        if (!input.IsCrossParent && !(input.ApoTargetRadiusMeters > 0.0))
            return Fail($"vehicle '{source.Id}': stock Hohmann produced an unbound transfer orbit, multi-pass not applicable",
                PassPlanFailure.Other);

        double vpTarget = ComputeVpTarget(input, mu, rp);
        if (!(vpTarget > 0.0))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': vpTarget non-positive ({1:F2}m/s)",
                source.Id, vpTarget), PassPlanFailure.Other);

        // vTargetXy is the in-plane component of the Lambert target speed;
        // D.Z (radial-out) is preserved separately on the final pass. For
        // tangential Hohmann porkchop optima D.Z is ~0 and vTargetXy ==
        // vpTarget. Guard against |D.Z| >= vpTarget (numerical noise) by
        // clamping the radicand to >= 0.
        double dz = input.DFinalVlf.Z;
        double vTargetXy = Math.Sqrt(Math.Max(0.0, vpTarget * vpTarget - dz * dz));

        if (!(vTargetXy > 0.0))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': Lambert tangential component degenerate " +
                "(|D.Z|={1:F1} >= vpTarget={2:F1}); porkchop entry has near-purely-radial dV",
                source.Id, Math.Abs(dz), vpTarget), PassPlanFailure.Other);

        // Parking-orbit reference SMA, locked via tPark. Used both as the
        // K-reference (a_chained = aPark * K^(2/3)) and to define
        // vpParking for the absolute thetaTotal computation.
        double aPark = Math.Pow(
            Math.Pow(tPark / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);
        double vpParking = Math.Sqrt(mu * (2.0 / rp - 1.0 / aPark));

        // Total plane rotation in the X-Y tangent plane. atan2 handles
        // sign; for tangential Hohmann (D.Y == 0) thetaTotal collapses to
        // 0 and all theta_k go to 0, reproducing pure-prograde priors.
        double thetaTotal = Math.Atan2(input.DFinalVlf.Y, vpParking + input.DFinalVlf.X);

        // Live periapsis speed: pre-burn-0 speed for the K-schedule. At
        // initial plan it equals vpParking; at recompute it's the chained
        // orbit's periapsis speed (= post-previous-pass v_p).
        double vpLive = Math.Sqrt(mu * (2.0 / rp - 1.0 / currentOrbit.SemiMajorAxis));

        // State-independent per-pass theta schedule for the full N. Uses
        // an equal-vp-step reference dvSeq so the allocation is purely
        // orbital-mechanics-driven and stable across initial plan vs
        // recompute. The actual K-schedule (SplitMode-driven) may differ,
        // but theta_k for absolute pass k stays the same value whether we
        // hit it on the initial plan or after k-1 priors. Without this,
        // each recompute would re-distribute the FULL thetaTotal across
        // only the remaining passes, double-counting the rotation that
        // priors already applied (sub-degree for Mars, visible for higher-
        // inclination targets).
        double[] thetaKAbsolute = AllocateAbsoluteThetaSchedule(
            thetaTotal, vpParking, vTargetXy, totalPassCount);

        // remainingCount == 1: just the final pass with residual dV from
        // the live orbit state. Two cases:
        //   * Initial N=1 (startPassIndex == 0): vehicle in parking orbit
        //     at T_final, fire at T_final exactly. dV equals stock single
        //     burn (matches Lambert closure).
        //   * Mid-exec last pass (startPassIndex > 0): vehicle in the
        //     post-pass-(N-2) chained orbit. Finite-burn drift in priors
        //     has shifted vehicle's actual next-periapsis time relative
        //     to the locked input.TFinal (observed: ~9000s drift for N=6
        //     LEO -> Mars). Firing at input.TFinal would put vehicle off
        //     periapsis (vpLive != v_p of the chained orbit), inflating
        //     the dV by ~14x (530 m/s planned -> 7600 m/s actual) and
        //     pointing the burn in the wrong CCI direction (because the
        //     LIVE-VLF axes are rotated from the chained-periapsis axes).
        //     Use vehicle's ACTUAL next-periapsis time instead.
        if (remainingCount == 1)
        {
            SimTime burnTime = (startPassIndex == 0)
                ? input.TFinal
                : currentOrbit.GetNextPeriapsisTime(now);
            return PlanSinglePass(source, input, now,
                burnTime, vTargetXy, thetaKAbsolute[startPassIndex]);
        }

        int priors = remainingCount - 1;

        // K-schedule (real per pass, integer sum at initial plan / time-
        // budget-locked at recompute). Uses Splitter.Allocate to pick
        // per-pass dV per the user's SplitMode, then maps cumulative dV
        // to v_p targets to real K via vis-viva. Auto-caps each prior at
        // 0.999 * v_p_at_SOI_envelope so the chain stays bound even for
        // high-energy departures (Uranus, Saturn) where EqualBurnTime /
        // EqualDv allocation would otherwise push priors past escape;
        // final pass absorbs the lost dV.
        var schedule = BuildRealKSchedule(
            mu, rp, aPark, soiLimit, vpLive, vTargetXy,
            remainingCount, mode, vehicleState);
        if (schedule.K == null)
        {
            PassPlanFailure kind = schedule.Failure ?? PassPlanFailure.ParabolicVp;
            string detail = kind == PassPlanFailure.FuelShort
                ? "vehicle has insufficient fuel for this N"
                : "priors degenerate under cap (transfer too high-energy for this N)";
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': K-schedule build failed; {1}",
                source.Id, detail), kind);
        }

        double[] kSeq = schedule.K!;
        double[] vPre = schedule.VPre!;
        double[] vPost = schedule.VPost!;

        // Compute target sum + first-pass time. Initial plan rounds the
        // K-sum to integer so vehicle in parking orbit at times[0] is at
        // the same CCI direction as at T_final (parking-orbit periodicity).
        // Recompute reads the vehicle's next chained-periapsis time from
        // live state; target sum is whatever (T_final - times[0]) / T_park
        // works out to, real value.
        double targetSumPeriods;
        double timesZeroSec;
        if (startPassIndex == 0)
        {
            double sumRaw = 0.0;
            for (int i = 0; i < priors; i++) sumRaw += kSeq[i];
            targetSumPeriods = Math.Round(sumRaw);
            timesZeroSec = input.TFinal.Seconds() - targetSumPeriods * tPark;
        }
        else
        {
            SimTime nextPe = currentOrbit.GetNextPeriapsisTime(now);
            timesZeroSec = nextPe.Seconds();
            targetSumPeriods = (input.TFinal.Seconds() - timesZeroSec) / tPark;
        }

        // Pre-adjustment K-sequence from BuildRealKSchedule is guaranteed
        // monotonic (cumulative v_p increases monotonically with each
        // prior, and K(v_p) is monotonic). The only way the sequence
        // breaks monotonicity is the integer-sum rounding adjustment
        // applied to kSeq[priors-1] below, which we explicitly check
        // post-adjustment.
        if (!(kSeq[0] > KFloor))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': K_0 = {1:F3} below floor {2:F2}; " +
                "first pass would be essentially a zero-dV burn (reduce N)",
                source.Id, kSeq[0], KFloor), PassPlanFailure.KFloor);

        // Apply target-sum adjustment to the LAST prior K. Recompute
        // vPost / dvSeq / vPre[priors] / dvSeq[priors] downstream of the
        // adjustment so the per-pass dV reflects the integer (or budget-
        // locked) chain length, not the raw splitter-derived sum.
        double sumK = 0.0;
        for (int i = 0; i < priors; i++) sumK += kSeq[i];
        double adjustment = targetSumPeriods - sumK;
        kSeq[priors - 1] += adjustment;
        if (!(kSeq[priors - 1] > KFloor))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': last prior K = {1:F3} after integer-sum rounding " +
                "(adjustment {2:F3}), below floor {3:F2}; reduce N",
                source.Id, kSeq[priors - 1], adjustment, KFloor),
                PassPlanFailure.KFloor);
        if (priors >= 2 && !(kSeq[priors - 1] > kSeq[priors - 2]))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': integer-sum rounding made K[{1}] = {2:F3} <= K[{3}] = {4:F3} " +
                "(adjustment {5:F3}); reduce N",
                source.Id, priors - 1, kSeq[priors - 1], priors - 2, kSeq[priors - 2], adjustment),
                PassPlanFailure.NonMonotonicK);
        double aLast = aPark * Math.Pow(kSeq[priors - 1], 2.0 / 3.0);
        double vpLast = Math.Sqrt(mu * (2.0 / rp - 1.0 / aLast));
        vPost[priors - 1] = vpLast;
        vPre[priors] = vpLast;

        // SOI / final-pass-dV validation. The schedule cap (0.999 of
        // v_p_at_SOI_envelope) is conservative; this check is defensive
        // against numerical edges (e.g. integer-sum rounding nudging the
        // last K above what the cap allowed in the loop).
        double apoLast = 2.0 * aLast - rp;
        if (apoLast > soiLimit)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': last prior apoapsis {1:F0}m exceeds SOI limit " +
                "{2:F0}m (K={3:F1}); reduce N",
                source.Id, apoLast, soiLimit, kSeq[priors - 1]),
                PassPlanFailure.SoiCeiling);
        if (!(vTargetXy > vpLast))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': vTargetXy {1:F1}m/s <= pre-final v_p {2:F1}m/s; " +
                "priors over-pumped the chain (likely target's vTargetXy below cap)",
                source.Id, vTargetXy, vpLast),
                PassPlanFailure.ParabolicVp);

        if (timesZeroSec < now.Seconds() + EarliestPassMarginSec)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': first pass would fire {1:F0}s before now+margin " +
                "(needs ~{2:F1} parking periods = {3:F0}s of warning time); " +
                "pick later porkchop entry",
                source.Id,
                now.Seconds() + EarliestPassMarginSec - timesZeroSec,
                targetSumPeriods, targetSumPeriods * tPark),
                PassPlanFailure.TimeBudget);

        // Schedule per-pass firing times. Each subsequent pass fires K_k
        // parking periods after the previous: vehicle returns to chained
        // periapsis after one full chained-orbit period (= K_k * T_park).
        var times = new SimTime[remainingCount];
        times[0] = new SimTime(timesZeroSec);
        for (int k = 1; k < remainingCount; k++)
            times[k] = new SimTime(times[k - 1].Seconds() + kSeq[k - 1] * tPark);

        // Slice the absolute theta schedule for the current call. theta_k
        // for absolute pass k stays stable across recomputes (see
        // AllocateAbsoluteThetaSchedule above), so the sum across the
        // sliced remaining passes equals (thetaTotal - sum of priors that
        // already executed their planned theta share). This avoids the
        // "re-allocate full thetaTotal across only the remaining N-k
        // passes" pitfall, which would over-rotate the orbital plane.
        double[] thetaK = new double[remainingCount];
        for (int i = 0; i < remainingCount; i++)
            thetaK[i] = thetaKAbsolute[startPassIndex + i];

        // Per-pass dV in LOCAL VLF:
        //   priors: (vPost*cos(theta_k) - vPre, vPost*sin(theta_k), 0)
        //   final:  same X/Y plus D.Z to preserve any radial Lambert
        //           residual (numerical noise in tangential Hohmann, but
        //           still applied at the final pass to keep the planar
        //           K-chain invariant clean for the priors).
        // For the final pass, vPost is vTargetXy (the in-plane component
        // of the Lambert post-burn speed); D.Z carries the remaining
        // radial-out velocity, giving total post-burn |v| = vpTarget.
        var dvVlfSeq = new double3[remainingCount];
        var dvMagSeq = new double[remainingCount];
        for (int k = 0; k < remainingCount; k++)
        {
            double vP = vPre[k];
            double vPo = (k < priors) ? vPost[k] : vTargetXy;
            double th = thetaK[k];
            double dvX = vPo * Math.Cos(th) - vP;
            double dvY = vPo * Math.Sin(th);
            double dvZ = (k == priors) ? dz : 0.0;
            dvVlfSeq[k] = new double3(dvX, dvY, dvZ);
            dvMagSeq[k] = dvVlfSeq[k].Length();
        }

        // Per-pass burn time estimate via multi-stage Tsiolkovsky drain.
        // Uses full per-pass |dV| (X+Y+Z), so the plane-change extra cost
        // shows up in the time budget the same way the magnitude does.
        double[] burnTimes = EstimateBurnTimes(dvMagSeq, vehicleState);

        // Forward-chain flight plans for the preview. The K-scheduled
        // times mean each chained orbit's periapsis lands at times[k+1]
        // by construction; the Y-component rotation does not affect this
        // because periapsis position stays fixed when dV.Z = 0.
        var previews = new List<PassPreview>(remainingCount);
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(times[0]);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': no current-orbit patch at t={1:F0}s",
                source.Id, times[0].Seconds()), PassPlanFailure.Other);

        for (int k = 0; k < remainingCount; k++)
        {
            double3 dvVlf = dvVlfSeq[k];

            var (fp, burnPatch) = BuildPassFlightPlan(
                source, prePatch, times[k], dvVlf, input.Target);
            previews.Add(new PassPreview(
                BurnTime: times[k],
                DvVlf: dvVlf,
                EstimatedBurnTimeSec: burnTimes[k],
                FlightPlan: fp));

            if (!burnPatch.Orbit.IsBound() && k < remainingCount - 1)
                return new PassPreviewResult(previews.ToArray(), Failed: true,
                    FailureReason: $"vehicle '{source.Id}': pass {startPassIndex + k} produced an unbound orbit",
                    FailureKind: PassPlanFailure.ParabolicVp);

            // Inter-pass sanity: chained orbit must reach the next pass
            // time without escaping / encountering / impacting.
            if (k < remainingCount - 1)
            {
                string? interFail = CheckInterPass(fp, times[k + 1], k + 1);
                if (interFail != null)
                    return new PassPreviewResult(previews.ToArray(),
                        Failed: true, FailureReason: $"vehicle '{source.Id}': {interFail}",
                        FailureKind: PassPlanFailure.Other);
            }

            prePatch = burnPatch;
        }

        // Final-pass impact / unintended-SOI advisory. Soft warning - the
        // plan is still usable, but stock's porkchop filter would have
        // rejected this geometry, so the user should see it. CheckInterPass
        // above already hard-failed if a PRIOR pass hits these conditions;
        // here we cover the gap for the FINAL pass's chained flight plan.
        string? finalAdvisory = CheckFinalPassAdvisory(
            previews[remainingCount - 1].FlightPlan, source, input.Target);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.Plan: total={0} startIdx={1} remaining={2} " +
                "mode={3} sumK={4:F2} T_park={5:F1}s T_final={6:F0}s T_0={7:F0}s span={8:F0}s " +
                "vpTarget={9:F1}m/s vTargetXy={10:F1}m/s thetaTotal={11:F4}rad " +
                "K[{12}] thetaK[{13}]rad dV[{14}]m/s advisory='{15}'",
                totalPassCount, startPassIndex, remainingCount, mode, targetSumPeriods, tPark,
                input.TFinal.Seconds(), times[0].Seconds(),
                input.TFinal.Seconds() - times[0].Seconds(), vpTarget, vTargetXy,
                thetaTotal, FormatKSeq(kSeq), FormatThetaSeq(thetaK), FormatDvSeq(dvMagSeq),
                finalAdvisory ?? "-"));

        return new PassPreviewResult(previews.ToArray(), Failed: false,
            FailureReason: null, Advisory: finalAdvisory);
    }

    /// <summary>Largest N that fits the SOI envelope and time budget for
    /// the initial plan (startPassIndex=0). Probes from
    /// <paramref name="requestedN"/> downward; returns 1 if no multi-pass
    /// N is feasible. The probe's first failure reason + classifier (at
    /// requestedN) are returned so the UI's auto-clamp banner can give
    /// context-appropriate advice (different for SOI ceiling vs time
    /// budget vs cumulative-dV-past-escape).
    ///
    /// Cost: O(requestedN) probes, each calling Plan which builds up to
    /// N flight plans, so worst case is O(N^2) FP builds. In practice
    /// most failures (SoiCeiling, ParabolicVp, KFloor, NonMonotonicK,
    /// TimeBudget) exit Plan before reaching the FP loop, making those
    /// probes near-free. Reaching the expensive FP loop requires passing
    /// all validation gates, which is rare at high N. Binary search is not
    /// applicable because feasibility is non-monotonic: a specific N can
    /// fail (e.g. integer-sum rounding breaks K monotonicity) while N+1
    /// succeeds, so there is no threshold to bisect on.</summary>
    public static int LargestFeasibleN(
        Vehicle source, HohmannPlanInput input, SequenceBurnState state,
        double parkingPeriodSec, SimTime now, int requestedN, SplitMode mode,
        out string? firstFailureReason, out PassPlanFailure firstFailureKind)
    {
        firstFailureReason = null;
        firstFailureKind = PassPlanFailure.None;
        int n = Math.Clamp(requestedN, 1, Splitter.MaxPasses);
        while (n > 1)
        {
            var probe = Plan(source, input, n, 0, parkingPeriodSec, state, now, mode);

            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                    "[AFC] HohmannMultiPassPlanner.LargestFeasibleN: probe N={0} mode={1} -> " +
                    "failed={2} kind={3} reason='{4}' passes={5}",
                    n, mode, probe.Failed, probe.FailureKind, probe.FailureReason ?? "-",
                    probe.Passes.Length));

            if (firstFailureReason == null && probe.Failed)
            {
                firstFailureReason = probe.FailureReason;
                firstFailureKind = probe.FailureKind;
            }
            if (!probe.Failed) return n;
            n--;
        }
        return 1;
    }

    /// <summary>Outcome of <see cref="PrepareShiftedInput"/>:
    /// (possibly-shifted) <see cref="HohmannPlanInput"/> plus the integer
    /// number of parking periods T_final was pushed by.
    ///
    /// <see cref="ScanAdvisory"/> is non-null when the shifted-Lambert mini-
    /// scan found no candidate that passes stock's Impacts / BadEncounter
    /// filter; we then fall back to the cheapest dirty candidate so the
    /// preview is still drawn, and the advisory surfaces what the dirty
    /// trajectory does. Null when (a) no shift was needed, or (b) the best
    /// candidate is clean.</summary>
    internal readonly record struct ShiftResult(
        HohmannPlanInput Input, int KShift, string? ScanAdvisory = null);

    /// <summary>
    /// Same-parent moon transfers (LEO -> Luna, low-Mars -> Phobos, ...) have
    /// a stock porkchop start axis that spans only ~1 parking period from
    /// `now`, because the synodic period of parking orbit vs moon is
    /// dominated by the parking orbit's period. Multi-pass with N >= 2 needs
    /// `T_final - now >= K_total * tPark + margin`, which the porkchop cannot
    /// offer.
    ///
    /// Fix: shift `T_final` forward by integer multiples of `tPark` until the
    /// K-schedule fits, then re-solve Lambert at the shifted time. The shift
    /// is integer parking periods so vehicle CCI position at the shifted
    /// T_final equals its CCI at the original porkchop pick (the K-integer
    /// property the planner relies on). Only target position changes (Luna
    /// drifts ~1.6 deg per shifted parking period for LEO); the re-Lambert
    /// absorbs that. A mini-scan over transit finds the lowest-dV candidate
    /// at the shifted geometry.
    ///
    /// Cross-parent transfers (LEO -> Mars) typically don't need the shift
    /// because their synodic period spans many parking periods anyway; the
    /// early-return on `raw.TFinal &gt;= earliestAllowed` keeps Mars
    /// untouched.
    ///
    /// Results are cached by (vehicle id, target id, raw T_final bucket,
    /// passCount, parking-period bucket, computed kShift). The 64-Lambert
    /// scan only runs on cache miss; while kShift stays in the same bucket
    /// (~one parking period of sim time at 1x), the cached result is reused.
    /// </summary>
    internal static ShiftResult PrepareShiftedInput(
        HohmannPlanInput raw, Vehicle source, OrbitalTransfers.TransferInfo info,
        int passCount, double parkingPeriodSec, SimTime now,
        SplitMode mode, SequenceBurnState state)
    {
        if (passCount <= 1 || !(parkingPeriodSec > 0.0))
            return new ShiftResult(raw, 0);

        // Cross-parent FinalizeLambert (hyperbolic-escape branch, taken
        // when transferInfo.Source != Vehicle) nudges Start by
        // timeFromPeTo2 - timeFromPeTo to align the optimized burn TA with
        // vehicle position. Both are in [0, tPark), so the nudge magnitude
        // is bounded by tPark and breaks the K-integer invariant the shift
        // trick relies on. Same-parent (LEO -> Luna) is unaffected because
        // stock's TransferTask.Run sets Source = Vehicle for SameSoi
        // transfers before workers run, taking FinalizeLambert's no-nudge
        // branch. The porkchop's T_final is far enough out that kShift
        // would be 0 in normal cross-parent flows anyway; this gate makes
        // that explicit so a future shorter-T_final pick doesn't silently
        // fall through the Lambert scan with no valid candidate.
        if (raw.IsCrossParent)
            return new ShiftResult(raw, 0);

        int kTotal = EstimateRequiredKTotal(
            raw, source, passCount, mode, state, parkingPeriodSec);
        if (kTotal <= 0)
            // Infeasible split (priors push past parabolic) or N=1: planner
            // will surface a more descriptive error; no shift needed.
            return new ShiftResult(raw, 0);

        double rawTFinalSec = raw.TFinal.Seconds();
        double earliestAllowedSec = now.Seconds()
            + kTotal * parkingPeriodSec
            + EarliestPassMarginSec;
        if (rawTFinalSec >= earliestAllowedSec)
            return new ShiftResult(raw, 0);

        int kShift = (int)Math.Ceiling(
            (earliestAllowedSec - rawTFinalSec) / parkingPeriodSec);
        if (kShift <= 0)
            return new ShiftResult(raw, 0);

        var key = new ShiftCacheKey(
            VehicleId: source.Id,
            TargetId: (info.Target as Astronomical)?.Id ?? string.Empty,
            RawTFinalBucketSec: (long)rawTFinalSec,
            PassCount: passCount,
            ParkingPeriodBucketSec: (long)Math.Round(parkingPeriodSec),
            KShift: kShift);
        if (_hasShiftCache && key == _shiftCacheKey)
            return new ShiftResult(_shiftCacheInput, kShift, _shiftCacheAdvisory);

        SimTime shiftedStart = new SimTime(rawTFinalSec + kShift * parkingPeriodSec);

        // Floor on min transit: defensive against an uninitialised
        // TransferInfo where MinTransferTimeOfFlight is 0 (would feed
        // SolveLambert a meaningless geometry). 60s is well below any
        // realistic moon / planet transit.
        double minTransitSec = Math.Max(60.0, info.MinTransferTimeOfFlight.Seconds());
        double maxTransitSec = Math.Max(
            minTransitSec + 1.0, info.MaxTransferTimeOfFlight.Seconds());

        // Scan candidates and apply stock's filter (Impacts / BadEncounter).
        // We track two bests: the cheapest CLEAN candidate (preferred) and
        // the cheapest dirty fallback. Pruning skips the expensive
        // BuildFlightPlan classification when the Lambert dV alone can't
        // improve either best.
        double bestCleanDv = double.MaxValue;
        double bestDirtyDv = double.MaxValue;
        OrbitalTransfers.TransferData? bestCleanTd = null;
        OrbitalTransfers.TransferData? bestDirtyTd = null;
        string? bestDirtyAdvisory = null;

        for (int i = 0; i < TransitScanSteps; i++)
        {
            double frac = (double)i / (TransitScanSteps - 1);
            double transitSec = minTransitSec + frac * (maxTransitSec - minTransitSec);
            var candidate = new OrbitalTransfers.TransferData
            {
                Start = shiftedStart,
                Transit = new SimTime(transitSec),
                ClosestApproachDistance = double.MaxValue,
            };
            if (!OrbitalTransfers.SolveLambert(info, ref candidate)) continue;

            double dvLen = candidate.TransferDvVlf.Length();
            if (!double.IsFinite(dvLen) || dvLen <= 0.0) continue;
            // Lambert dV can't improve either best: skip BuildFlightPlan.
            if (dvLen >= bestCleanDv && dvLen >= bestDirtyDv) continue;

            var probeFp = FlightPlan.CreateUninitialized(source.Hash);
            if (!OrbitalTransfers.BuildFlightPlan(
                    ref probeFp, info, candidate.Start, candidate.TransferDvVlf,
                    out _, out _))
                continue;

            // Replicates stock TransferTask.WorkerTask.CalculateAutomaticTransfer:
            // patches that end in Impact on a body other than the target before
            // the planned arrival mark the candidate as impacting; patches with
            // a PrimaryBody outside the {Source, Source.Parent, Target} tuple
            // mark a bad encounter.
            SimTime arrival = candidate.Start + candidate.Transit;
            string? dirtyReason = ClassifyScanCandidate(probeFp, info, arrival);
            bool clean = dirtyReason == null;

            if (clean)
            {
                if (dvLen < bestCleanDv)
                {
                    bestCleanDv = dvLen;
                    bestCleanTd = candidate;
                }
            }
            else if (dvLen < bestDirtyDv)
            {
                bestDirtyDv = dvLen;
                bestDirtyTd = candidate;
                bestDirtyAdvisory = dirtyReason;
            }
        }

        OrbitalTransfers.TransferData bestTd;
        double bestDvLen;
        string? scanAdvisory;
        if (bestCleanTd != null)
        {
            bestTd = bestCleanTd;
            bestDvLen = bestCleanDv;
            scanAdvisory = null;
        }
        else if (bestDirtyTd != null)
        {
            bestTd = bestDirtyTd;
            bestDvLen = bestDirtyDv;
            scanAdvisory = bestDirtyAdvisory;
        }
        else
        {
            return new ShiftResult(raw, 0);
        }

        // Same-parent: stock's entry.FlightPlan.Patches[0].Orbit.Apoapsis is
        // the un-shifted target. BuildFlightPlan at the shifted Start + new
        // dV gives the actual post-burn apoapsis to track.
        double apoTargetRadiusM = 0.0;
        if (!raw.IsCrossParent)
        {
            var fp = FlightPlan.CreateUninitialized(source.Hash);
            OrbitalTransfers.BuildFlightPlan(
                ref fp, info, bestTd.Start, bestTd.TransferDvVlf, out _, out _);
            if (fp.Patches.Count > 0)
            {
                double apo = fp.Patches[0].Orbit.Apoapsis;
                if (double.IsFinite(apo) && apo > source.Orbit.Periapsis)
                    apoTargetRadiusM = apo;
            }
        }

        double vInfMs = raw.IsCrossParent
            ? bestTd.EjectionVelocityCci.Length()
            : 0.0;

        var shifted = raw with
        {
            TFinal = bestTd.Start,
            DFinalVlf = bestTd.TransferDvVlf,
            VInfMs = vInfMs,
            ApoTargetRadiusMeters = apoTargetRadiusM,
        };

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.PrepareShiftedInput: vehicle='{0}' " +
                "target='{1}' passCount={2} K_total={3} K_shift={4} " +
                "rawTFinal={5:F0}s shiftedTFinal={6:F0}s rawDv={7:F1}m/s " +
                "shiftedDv={8:F1}m/s shiftedTransit={9:F0}s apoTarget={10:F0}m " +
                "vInf={11:F1}m/s isCrossParent={12} scanAdvisory='{13}'",
                source.Id, key.TargetId, passCount, kTotal, kShift,
                rawTFinalSec, bestTd.Start.Seconds(),
                raw.DFinalVlf.Length(), bestDvLen,
                bestTd.Transit.Seconds(), apoTargetRadiusM, vInfMs,
                raw.IsCrossParent, scanAdvisory ?? "-"));

        _shiftCacheKey = key;
        _shiftCacheInput = shifted;
        _shiftCacheAdvisory = scanAdvisory;
        _hasShiftCache = true;
        return new ShiftResult(shifted, kShift, scanAdvisory);
    }

    /// <summary>Drops the shift-input cache. Call from the UI's Reset and
    /// from Mod.Unload so a fresh session does not see stale entries
    /// (different save, recycled vehicle ids, etc.).</summary>
    public static void ResetShiftCache()
    {
        _hasShiftCache = false;
        _shiftCacheInput = default;
        _shiftCacheKey = default;
        _shiftCacheAdvisory = null;
    }

    // SplitMode is not a field on the key on purpose: it affects K_total
    // (via EstimateRequiredKTotal), and K_total feeds K_shift directly.
    // Same K_shift across two modes => same shifted geometry => same
    // scan result and advisory. So mode is captured transitively.
    private readonly record struct ShiftCacheKey(
        string VehicleId,
        string TargetId,
        long RawTFinalBucketSec,
        int PassCount,
        long ParkingPeriodBucketSec,
        int KShift);

    private static ShiftCacheKey _shiftCacheKey;
    private static HohmannPlanInput _shiftCacheInput;
    private static string? _shiftCacheAdvisory;
    private static bool _hasShiftCache;

    #region Internal helpers

    /// <summary>Real-K schedule for the remaining passes. Uses
    /// <see cref="Splitter.Allocate"/> with <paramref name="mode"/> to
    /// split totalDv = vTargetXy - vpLive across <paramref name="remainingCount"/>
    /// passes, then converts cumulative dV to v_p targets via vis-viva,
    /// then to K via Kepler. K array covers the priors only
    /// (length = remainingCount - 1); vPre/vPost/dvSeq cover ALL remaining
    /// passes (length = remainingCount). The last entry of vPre/vPost/
    /// dvSeq is the final pass (vPost = vTargetXy).
    ///
    /// The returned K-sum is the RAW splitter-derived sum (real). The
    /// caller is expected to adjust K[priors-1] to enforce either
    /// integer-sum (initial plan) or time-budget-locked sum (recompute);
    /// vPost / dvSeq are then recomputed downstream of that adjustment.
    ///
    /// Returns K = null when a per-pass dV target pushes v_p past
    /// parabolic, which means the split is infeasible for this passCount
    /// (caller fails and the UI auto-clamps via LargestFeasibleN).</summary>
    private readonly struct RealKScheduleResult
    {
        public double[]? K { get; init; }
        public double[]? VPre { get; init; }
        public double[]? VPost { get; init; }
        public PassPlanFailure? Failure { get; init; }
    }

    private static RealKScheduleResult BuildRealKSchedule(
        double mu, double rp, double aPark, double soiLimit,
        double vpLive, double vTargetXy,
        int remainingCount, SplitMode mode, SequenceBurnState state)
    {
        int priors = remainingCount - 1;
        if (priors < 1) return default;

        double totalDv = vTargetXy - vpLive;
        if (!(totalDv > 0.0)) return default;

        PassAllocation[] alloc = Splitter.Allocate(totalDv, remainingCount, mode, state);

        var K = new double[priors];
        var vPre = new double[remainingCount];
        var vPost = new double[remainingCount];

        // Per-prior v_p cap from the parent-SOI envelope: chained orbit
        // apoapsis must stay below 0.95 * SOI, which directly bounds v_p
        // at periapsis. Tiny 0.001 margin absorbs numerical noise so the
        // separate apo-vs-soiLimit check downstream doesn't fire on the
        // capped-just-at-limit case.
        //
        // vp_at_SOI_envelope is always strictly below vp_escape (the
        // escape limit corresponds to a -> infinity, but soiLimit is
        // finite), so this cap is equivalent to "stay bound" too.
        //
        // Whichever SplitMode is active still drives the per-pass
        // allocation; the cap only clips priors that would otherwise
        // exceed vp_max, and the final pass absorbs the residual. For
        // Mars-like targets where vTargetXy stays below the cap, the cap
        // never binds and the SplitMode's per-pass allocation is
        // delivered as-is.
        double aMaxBySoi = (rp + soiLimit) / 2.0;
        double vpMaxPrior = (aMaxBySoi > rp)
            ? Math.Sqrt(Math.Max(0.0, mu * (2.0 / rp - 1.0 / aMaxBySoi))) * 0.999
            : 0.0;

        double vpCum = vpLive;
        bool reachedCap = false;
        for (int k = 0; k < priors; k++)
        {
            vPre[k] = vpCum;
            double targetVp = vpCum + alloc[k].DvCapacityMs;
            if (targetVp > vpMaxPrior)
            {
                if (reachedCap)
                    // Previous prior already saturated; this pass would
                    // contribute zero/negative dV and break K-monotonicity.
                    // Fail so LargestFeasibleN clamps N down to the
                    // largest where each prior carries meaningful dV.
                    return default;
                if (vpMaxPrior <= vpCum)
                    // First pass already starts above cap (live v_p too
                    // high). Mid-execution edge: prior passes saturated
                    // the chain before recompute. Infeasible.
                    return default;
                targetVp = vpMaxPrior;
                reachedCap = true;
            }
            if (!(targetVp > vPre[k]))
                return new RealKScheduleResult { Failure = PassPlanFailure.FuelShort };
            vpCum = targetVp;
            double term = 2.0 / rp - vpCum * vpCum / mu;
            if (!(term > 0.0))
                // Bound-orbit invariant violated despite cap (numerical
                // edge). Defensive.
                return default;
            double aPost = 1.0 / term;
            K[k] = Math.Pow(aPost / aPark, 1.5);
            vPost[k] = vpCum;
        }
        vPre[priors] = vpCum;
        vPost[priors] = vTargetXy;

        return new RealKScheduleResult { K = K, VPre = vPre, VPost = vPost };
    }

    /// <summary>Integer round of the K-schedule sum for the given
    /// passCount / mode / state, matching the value the planner uses
    /// at initial plan to set times[0]. Used by PrepareShiftedInput
    /// to size the same-parent moon shift.
    ///
    /// Returns 0 for passCount &lt;= 1, and -1 when the split is
    /// infeasible (priors push past parabolic before reaching the
    /// target).</summary>
    internal static int EstimateRequiredKTotal(
        HohmannPlanInput raw, Vehicle source, int passCount,
        SplitMode mode, SequenceBurnState state, double parkingPeriodSec)
    {
        if (passCount <= 1) return 0;
        Orbit o = source.Orbit;
        if (o?.Parent == null) return 0;
        double mu = o.Mu;
        double rp = o.Periapsis;
        if (!(rp > 0.0) || !(parkingPeriodSec > 0.0)) return 0;

        double soiLimit = o.Parent.SphereOfInfluence * SoiEnvelopeFraction;
        if (!(soiLimit > 0.0)) return -1;

        double vpTarget = ComputeVpTarget(raw, mu, rp);
        if (!(vpTarget > 0.0)) return -1;

        double dz = raw.DFinalVlf.Z;
        double vTargetXy = Math.Sqrt(Math.Max(0.0, vpTarget * vpTarget - dz * dz));
        double vpLive = Math.Sqrt(mu * (2.0 / rp - 1.0 / o.SemiMajorAxis));
        double aPark = Math.Pow(
            Math.Pow(parkingPeriodSec / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);

        var schedule = BuildRealKSchedule(
            mu, rp, aPark, soiLimit, vpLive, vTargetXy, passCount, mode, state);
        if (schedule.K == null) return -1;

        double sumK = 0.0;
        for (int i = 0; i < schedule.K.Length; i++) sumK += schedule.K[i];
        return (int)Math.Round(sumK);
    }

    /// <summary>Per-pass theta schedule for the FULL N-pass plan, against
    /// a stable state-independent equal-vp-step reference dvSeq. The
    /// resulting thetaK[k] is the planned plane-rotation share for
    /// ABSOLUTE pass k regardless of when the planner is called (initial
    /// vs recompute). Caller slices [startPassIndex..startPassIndex+remaining]
    /// for the current call so the remaining passes pick up exactly the
    /// residual rotation thetaTotal - sum(priors already done their share).
    ///
    /// Why equal-vp-step and not the SplitMode-driven K-schedule's dvSeq:
    /// the SplitMode allocation depends on vehicle mass (for
    /// EqualBurnTime), which drifts across recomputes as fuel is burned.
    /// Allocating theta_k against that moving target would shift theta_k
    /// for absolute pass k between recomputes - breaking the "sliced
    /// schedule sums to remaining theta budget" invariant the slice
    /// relies on. Equal-vp-step is purely orbital mechanics, identical
    /// across calls.
    ///
    /// The Lagrange optimality is slightly off (the reference dvSeq
    /// doesn't match the actual scheduled dvSeq exactly), but for the
    /// stock Sol thetaTotal range (Mars 0.002 rad, Jupiter 0.17 rad) the
    /// extra cost is sub-m/s; the alternative (persisting per-pass
    /// thetaK in the intent) would require new TOML fields and reload
    /// migration logic.</summary>
    private static double[] AllocateAbsoluteThetaSchedule(
        double thetaTotal, double vpParking, double vTargetXy, int totalPassCount)
    {
        var dvSeq = new double[totalPassCount];
        var vPre = new double[totalPassCount];
        var vPost = new double[totalPassCount];
        double stepDv = (vTargetXy - vpParking) / totalPassCount;
        double vp = vpParking;
        for (int i = 0; i < totalPassCount; i++)
        {
            vPre[i] = vp;
            vp += stepDv;
            vPost[i] = vp;
            dvSeq[i] = stepDv;
        }
        return AllocatePlaneChangeLagrange(thetaTotal, dvSeq, vPre, vPost);
    }

    /// <summary>Lagrange-optimal plane-change allocation: distributes
    /// <paramref name="thetaTotal"/> across N passes proportional to
    /// `delta_R / (v_pre * v_post)`. This minimises the total extra dV
    /// cost from rotating the orbital plane, given the cost-per-pass
    /// formula `v_pre * v_post * theta_k^2 / (2 * delta_R_k)` (Lagrange
    /// multiplier on the sum_k theta_k = thetaTotal constraint).
    ///
    /// Versus the simpler "proportional to delta_R" weighting, this
    /// shifts rotation toward passes with smaller v_pre*v_post (typically
    /// pass 0, which kicks from parking-speed) and away from the final
    /// pass (highest v_pre*v_post). For stock-Sol targets with small
    /// plane changes (Mars 0.1deg) the difference is sub-m/s; for high-
    /// theta cases (Jupiter 9.6deg, custom systems) it saves ~10-20 m/s
    /// AND makes the first prior pass's plane rotation more visible in
    /// the 3D preview. Last pass absorbs floating-point residual so the
    /// sum exactly matches <paramref name="thetaTotal"/>.</summary>
    private static double[] AllocatePlaneChangeLagrange(
        double thetaTotal, double[] dvSeq, double[] vPre, double[] vPost)
    {
        var thetaK = new double[dvSeq.Length];
        var weights = new double[dvSeq.Length];
        double sumWeight = 0.0;
        for (int i = 0; i < dvSeq.Length; i++)
        {
            double denom = vPre[i] * vPost[i];
            weights[i] = (denom > 0.0) ? Math.Abs(dvSeq[i]) / denom : 0.0;
            sumWeight += weights[i];
        }

        if (!(sumWeight > 0.0))
        {
            // Degenerate: equal split. Hit only if all per-pass weights are
            // zero, which would also fail downstream vis-viva checks; we
            // still return a finite allocation so the caller's loop runs.
            double per = thetaTotal / dvSeq.Length;
            for (int i = 0; i < dvSeq.Length; i++) thetaK[i] = per;
            return thetaK;
        }

        double acc = 0.0;
        for (int i = 0; i < dvSeq.Length - 1; i++)
        {
            thetaK[i] = thetaTotal * weights[i] / sumWeight;
            acc += thetaK[i];
        }
        thetaK[dvSeq.Length - 1] = thetaTotal - acc;
        return thetaK;
    }

    /// <summary>Target periapsis speed after the final pass.
    /// Cross-parent: derived from v_inf at parking r_p (gives a hyperbolic
    /// orbit with the right asymptote magnitude).
    /// Same-parent: derived from the target apoapsis radius (gives the
    /// bound transfer ellipse).</summary>
    private static double ComputeVpTarget(HohmannPlanInput input, double mu, double rp)
    {
        if (input.IsCrossParent)
            return Math.Sqrt(input.VInfMs * input.VInfMs + 2.0 * mu / rp);

        double aTarget = (rp + input.ApoTargetRadiusMeters) * 0.5;
        if (!(aTarget > 0.0)) return 0.0;
        double term = 2.0 / rp - 1.0 / aTarget;
        if (!(term > 0.0)) return 0.0;
        return Math.Sqrt(mu * term);
    }

    private static (FlightPlan fp, PatchedConic burnPatch) BuildPassFlightPlan(
        Vehicle source, PatchedConic prePatch,
        SimTime burnTime, double3 dvVlf, IOrbiter target)
    {
        SimTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime);
        FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
        PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, dvVlf, burnTime);
        fp.Patches.Add(burnPatch);
        // target-aware: encounter detection populates ClosestApproaches
        // with the destination body so the preview shows how close the
        // departure actually gets to (e.g.) Mars.
        fp.ComputeCompleteTrajectory(out _, FlightPlanPatchLimit, FlightPlanPolynomialOrder, target);
        if (source.Target != null)
            fp.CalculateTargetNodes(source.Target);
        return (fp, burnPatch);
    }

    private static string? CheckInterPass(FlightPlan priorFp, SimTime nextTime, int passIndex)
    {
        foreach (PatchedConic p in priorFp.Patches)
        {
            if (p.EndTime >= nextTime) continue;
            if (p.EndTransition == PatchTransition.Escape)
                return $"pass {passIndex - 1} escapes SOI before pass {passIndex}";
            if (p.EndTransition == PatchTransition.Encounter)
                return $"pass {passIndex - 1} encounters another body before pass {passIndex}";
            if (p.EndTransition == PatchTransition.Impact)
                return $"pass {passIndex - 1} impacts the parent body before pass {passIndex}";
        }
        return null;
    }

    /// <summary>Stock's per-candidate filter from
    /// <c>TransferTask.WorkerTask.CalculateAutomaticTransfer</c>. Returns
    /// a human-readable advisory string when the flight plan would have
    /// been rejected by stock's Impacts / BadEncounter filter, null when
    /// it's clean. Used by <see cref="PrepareShiftedInput"/> to prefer
    /// clean candidates over impacting ones.
    ///
    /// <paramref name="arrival"/> is the candidate's Lambert arrival time
    /// (Start + Transit); stock only treats Impact as disqualifying when
    /// the impact happens BEFORE arrival, so a "captured by Luna at the end"
    /// patch labelled Impact (because we don't model atmospheric capture)
    /// doesn't trip the filter.</summary>
    private static string? ClassifyScanCandidate(
        FlightPlan fp, OrbitalTransfers.TransferInfo info, SimTime arrival)
    {
        string targetId = info.Target?.Id ?? string.Empty;
        string targetParentId = info.Target?.Parent?.Id ?? string.Empty;
        string sourceId = info.Source?.Id ?? string.Empty;
        string sourceParentId = info.Source?.Parent?.Id ?? string.Empty;

        foreach (PatchedConic patch in fp.Patches)
        {
            if (patch.EndTransition == PatchTransition.Impact
                && patch.Orbit.Parent?.Id != targetId
                && patch.EndTime < arrival)
            {
                string body = patch.Orbit.Parent?.Id ?? "parent body";
                return $"Departure trajectory impacts {body} before arrival";
            }
            string pbId = patch.PrimaryBody?.Id ?? string.Empty;
            if (pbId == targetParentId) continue;
            if (pbId != sourceId && pbId != sourceParentId && pbId != targetId)
                return $"Departure trajectory crosses unintended SOI of '{pbId}'";
        }
        return null;
    }

    /// <summary>Post-plan advisory: walks the FINAL pass's flight plan for
    /// the same Impact / unintended-SOI conditions stock's porkchop filter
    /// uses. Unlike <see cref="CheckInterPass"/> (which hard-fails the plan
    /// for problems BETWEEN priors), this runs on the final-burn FP and
    /// only generates an advisory string; the caller flags it on
    /// <see cref="PassPreviewResult.Advisory"/> without setting Failed.
    /// Returns null when the final pass is clean.
    ///
    /// Accepts the parking parent, the target, and the target's parent as
    /// legitimate PrimaryBody values. For same-parent transfers
    /// (target.Parent == parking parent) the target-parent entry is
    /// redundant with the parking-parent entry but harmless; for cross-
    /// parent (e.g. Earth -> Mars via Sun) the target-parent entry is what
    /// admits the cruise patch.</summary>
    private static string? CheckFinalPassAdvisory(
        FlightPlan finalFp, Vehicle source, IOrbiter target)
    {
        string parkingParentId = source.Orbit.Parent.Id;
        string targetId = target?.Id ?? string.Empty;
        string targetParentId = target?.Parent?.Id ?? string.Empty;

        foreach (PatchedConic patch in finalFp.Patches)
        {
            if (patch.EndTransition == PatchTransition.Impact
                && patch.Orbit.Parent?.Id != targetId)
            {
                string body = patch.Orbit.Parent?.Id ?? "parent body";
                return $"Final-pass trajectory impacts {body}";
            }
            string pbId = patch.PrimaryBody?.Id ?? string.Empty;
            if (pbId == parkingParentId) continue;
            if (pbId == targetId) continue;
            if (pbId == targetParentId) continue;
            return $"Final-pass trajectory crosses unintended SOI of '{pbId}'";
        }
        return null;
    }

    /// <summary>Per-pass burn time via multi-stage Tsiolkovsky drain.
    /// Walks the dV sequence in firing order; later passes see the
    /// vehicle drained by earlier passes. Returns 0s entries when the
    /// vehicle has no usable engines (UI just hides the time column).</summary>
    private static double[] EstimateBurnTimes(double[] dvSeq, SequenceBurnState state)
    {
        var burnTimes = new double[dvSeq.Length];
        if (!state.HasUsableEngines) return burnTimes;

        int seqCount = state.Sequences.Count;
        var startMass = new double[seqCount];
        var fuelRemaining = new double[seqCount];
        var mDot = new double[seqCount];
        var vExhaust = new double[seqCount];
        for (int i = 0; i < seqCount; i++)
        {
            var s = state.Sequences[i];
            startMass[i] = s.StartMassKg;
            fuelRemaining[i] = s.FuelMassKg;
            mDot[i] = s.MassFlowKgPerSec;
            vExhaust[i] = s.ExhaustVelocityMs;
        }

        for (int k = 0; k < dvSeq.Length; k++)
        {
            double dvLeft = dvSeq[k];
            double timeUsed = 0.0;
            for (int s = 0; s < seqCount && dvLeft > 0.0; s++)
            {
                if (mDot[s] <= 0.0 || fuelRemaining[s] <= 0.0) continue;
                double m0 = startMass[s];
                double mMin = Math.Max(0.0, m0 - fuelRemaining[s]);
                if (mMin <= 0.0 || m0 <= 0.0) continue;
                double dvStageMax = vExhaust[s] * Math.Log(m0 / mMin);
                double dvFromStage = Math.Min(dvLeft, dvStageMax);
                double mEnd = m0 * Math.Exp(-dvFromStage / vExhaust[s]);
                double fuelBurned = m0 - mEnd;
                double tBurn = fuelBurned / mDot[s];
                startMass[s] = mEnd;
                fuelRemaining[s] -= fuelBurned;
                timeUsed += tBurn;
                dvLeft -= dvFromStage;
            }
            burnTimes[k] = timeUsed;
        }
        return burnTimes;
    }

    /// <summary>Single-pass case: the only remaining pass closes from the
    /// live orbit to the Lambert target. dV in LIVE-VLF:
    ///   dV = (vTargetXy*cos(theta) - vLive, vTargetXy*sin(theta), D.Z)
    /// For N=1 initial plan (startPassIndex=0, theta = thetaTotal), the
    /// algebra collapses to (D.X, D.Y, D.Z) when vLive == vpParking,
    /// matching the stock single-burn dV up to Lambert's internal
    /// numerical roundoff.
    ///
    /// <paramref name="burnTime"/> must land at a periapsis of the live
    /// orbit so vpLive == v_p (the apse-geometry assumption baked into
    /// vTargetXy). For initial N=1 the caller passes <c>input.TFinal</c>
    /// (vehicle in parking orbit at the locked Lambert burn point). For
    /// mid-exec last pass the caller passes <c>currentOrbit.GetNextPeriapsisTime(now)</c>
    /// of the chained orbit, which may drift seconds-to-hours away from
    /// <c>input.TFinal</c> due to finite-burn losses on prior passes.</summary>
    private static PassPreviewResult PlanSinglePass(
        Vehicle source, HohmannPlanInput input, SimTime now,
        SimTime burnTime, double vTargetXy, double theta)
    {
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(burnTime);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': no patch at burn time t={1:F0}s",
                source.Id, burnTime.Seconds()), PassPlanFailure.Other);

        Orbit o = prePatch.Orbit;
        StateVectors svAt = o.GetStateVectorsAt(burnTime);
        double vpLive = svAt.VelocityCci.Length();
        double rpLive = svAt.PositionCci.Length();
        if (!(rpLive > 0.0))
            return Fail($"vehicle '{source.Id}': degenerate position at burn time",
                PassPlanFailure.Other);

        // dV in LIVE-VLF. LIVE-VLF X is along live velocity (which has
        // magnitude vpLive); the target post-burn velocity has in-plane
        // magnitude vTargetXy at angle theta from LIVE-X. D.Z is preserved
        // in LIVE-VLF Z because the periapsis radial axis is the same in
        // PARKING-VLF and LIVE-VLF (rotations between them are around Z,
        // which leaves Z components invariant).
        double dvX = vTargetXy * Math.Cos(theta) - vpLive;
        double dvY = vTargetXy * Math.Sin(theta);
        double dvZ = input.DFinalVlf.Z;
        double3 dvVlf = new(dvX, dvY, dvZ);
        double dvFinalMag = dvVlf.Length();
        if (!(dvFinalMag > 0.0))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': residual dV at burn time is non-positive ({1:F2}m/s); " +
                "priors already over-shot Lambert target",
                source.Id, dvFinalMag), PassPlanFailure.ParabolicVp);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.PlanSinglePass: vehicle='{0}' " +
                "burnTime={1:F0}s (input.TFinal={2:F0}s, drift={3:F0}s) " +
                "vpLive={4:F1}m/s vTargetXy={5:F1}m/s theta={6:F5}rad " +
                "dV=({7:F1},{8:F1},{9:F1})m/s |dV|={10:F1}m/s",
                source.Id, burnTime.Seconds(), input.TFinal.Seconds(),
                burnTime.Seconds() - input.TFinal.Seconds(),
                vpLive, vTargetXy, theta, dvX, dvY, dvZ, dvFinalMag));

        var (fp, _) = BuildPassFlightPlan(
            source, prePatch, burnTime, dvVlf, input.Target);
        var single = new PassPreview(
            BurnTime: burnTime,
            DvVlf: dvVlf,
            EstimatedBurnTimeSec: 0.0,
            FlightPlan: fp);
        string? finalAdvisory = CheckFinalPassAdvisory(fp, source, input.Target);
        return new PassPreviewResult(new[] { single }, Failed: false,
            FailureReason: null, Advisory: finalAdvisory);
    }

    private static PassPreviewResult Fail(string reason, PassPlanFailure kind) =>
        new(Array.Empty<PassPreview>(), Failed: true, FailureReason: reason,
            FailureKind: kind);

    private static string FormatDvSeq(double[] dvSeq)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < dvSeq.Length; i++)
        {
            if (i > 0) sb.Append(", ");
            sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F1}", dvSeq[i]);
        }
        return sb.ToString();
    }

    private static string FormatThetaSeq(double[] thetaK)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < thetaK.Length; i++)
        {
            if (i > 0) sb.Append(", ");
            sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F5}", thetaK[i]);
        }
        return sb.ToString();
    }

    private static string FormatKSeq(double[] kSeq)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < kSeq.Length; i++)
        {
            if (i > 0) sb.Append(", ");
            sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F2}", kSeq[i]);
        }
        return sb.ToString();
    }

    #endregion
}
