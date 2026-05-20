using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// K-integer-multiple multi-pass planner for stock Hohmann / interplanetary
/// transfer departures. Implements gravhoek's "schedule passes in previous
/// orbits, handle phasing of intermediate orbits" hint properly: each
/// intermediate orbit's period is forced to be an integer multiple of the
/// parking orbit period so that every chained burn lands at the SAME CCI
/// position as the stock Lambert burn point at T_final - preserving the
/// departure phasing exactly.
///
/// Algorithm:
///   1. Pick K sequence: minimum monotonic increasing integers starting at
///      2, i.e. K = (2, 3, ..., N) for N passes. K_total = N(N+1)/2 - 1.
///   2. Schedule: times[0] = T_final - K_total * T_park, then
///      times[k+1] = times[k] + K_k * T_park. By parking-orbit periodicity,
///      vehicle on parking orbit at times[0] is at the same CCI direction
///      as at T_final.
///   3. Each pass adds two components in its LOCAL VLF at periapsis:
///      a K-prograde X kick that sets |v_post_k| to the K-scheduled speed,
///      and a Y kick that rotates the orbital plane by theta_k around the
///      periapsis radial. Because dV.Z = 0 for priors, periapsis CCI
///      position is preserved across the chain and the K-integer property
///      holds regardless of theta_k.
///   4. Final pass at T_final, vehicle still at the locked CCI direction.
///      The radial D.Z component of the Lambert dV (typically ~0 for
///      tangential Hohmann porkchop optima) is added on the final pass
///      only, so the priors' K-chain stays planar.
///   5. theta_k is allocated proportional to delta_R_k / (v_pre_k *
///      v_post_k), the Lagrange-optimal weighting that minimises total
///      extra dV from rotating the orbital plane. Deterministic from
///      locked inputs so recompute mid-execution stays consistent.
///   6. Per-pass dV magnitudes are COMPUTED from the K-sequence + theta_k
///      via vis-viva, not free-allocated by a splitter. SplitMode is
///      therefore irrelevant for Hohmann; the only user choice is N.
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

    /// <summary>K_total = sum(2 + 3 + ... + N) = N*(N+1)/2 - 1, the integer
    /// number of parking periods spanned by the N-pass K-schedule. Shared by
    /// Plan, GetSpanSeconds, and PrepareShiftedInput so all three sides
    /// agree on the time-budget formula.</summary>
    private static int GetKTotal(int passCount) =>
        passCount <= 1 ? 0 : passCount * (passCount + 1) / 2 - 1;

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
        SimTime now)
    {
        if (source?.Orbit?.Parent == null)
            return Fail($"vehicle '{source?.Id ?? "?"}' has no orbit parent");
        if (totalPassCount < 1)
            return Fail($"vehicle '{source.Id}': totalPassCount {totalPassCount} < 1");
        if (startPassIndex < 0 || startPassIndex >= totalPassCount)
            return Fail($"vehicle '{source.Id}': startPassIndex {startPassIndex} out of range [0,{totalPassCount})");
        if (!(parkingPeriodSec > 0.0))
            return Fail($"vehicle '{source.Id}': parkingPeriodSec {parkingPeriodSec} must be positive");

        Orbit currentOrbit = source.Orbit;
        if (!currentOrbit.IsBound())
            return Fail($"vehicle '{source.Id}': current orbit is unbound");

        double mu = currentOrbit.Mu;
        double rp = currentOrbit.Periapsis;
        if (!(rp > 0.0))
            return Fail($"vehicle '{source.Id}': current orbit has non-positive periapsis");

        double tPark = parkingPeriodSec;

        double soiLimit = currentOrbit.Parent.SphereOfInfluence * SoiEnvelopeFraction;
        if (!(soiLimit > 0.0))
            return Fail($"vehicle '{source.Id}': parent '{currentOrbit.Parent.Id}' has no SOI");

        int remainingCount = totalPassCount - startPassIndex;

        if (input.DFinalVlf.LengthSquared() < 1.0)
            return Fail($"vehicle '{source.Id}': stock Hohmann dV is zero");

        // Plane-change distribution: each pass carries both K-prograde dV
        // (vPost*cos(theta_k) - vPre, along local-X) and an in-plane Y
        // component (vPost*sin(theta_k)) that rotates the orbital plane
        // by theta_k around the periapsis radial. The Y component
        // preserves the K-integer property (periapsis CCI position
        // unchanged because dV.Z = 0; post-burn |v| still equals K-target
        // because |vPost*(cos,sin,0)| = vPost) while spreading the
        // Lambert plane change across passes. Extra cost per pass is
        // v_pre*v_post*theta_k^2 / (2*delta_R_k); the sum is minimised
        // by allocating theta_k proportional to delta_R_k/(v_pre*v_post)
        // (Lagrange-optimal, see AllocatePlaneChangeLagrange).
        double vpTarget = ComputeVpTarget(input, mu, rp);
        if (!(vpTarget > 0.0))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': vpTarget non-positive ({1:F2}m/s)",
                source.Id, vpTarget));

        // vTargetXy is the in-plane component of the Lambert target speed;
        // D.Z (radial-out) is preserved separately on the final pass. For
        // tangential Hohmann porkchop optima D.Z is ~0 and vTargetXy ==
        // vpTarget. Guard against |D.Z| >= vpTarget (numerical noise) by
        // clamping the radicand to >= 0.
        double dz = input.DFinalVlf.Z;
        double vTargetXy = Math.Sqrt(Math.Max(0.0, vpTarget * vpTarget - dz * dz));

        // Parking-orbit periapsis speed: the K-prograde reference for the
        // original (startPassIndex=0) chain. Locked via tPark, independent
        // of live drift, so thetaTotal and the per-pass allocation are
        // identical across initial plan and every recompute.
        double aParking = Math.Pow(
            Math.Pow(tPark / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);
        double vpParking = Math.Sqrt(mu * (2.0 / rp - 1.0 / aParking));

        // Total plane rotation in the X-Y tangent plane. atan2 handles
        // sign; for tangential Hohmann (D.Y == 0) thetaTotal collapses to
        // 0 and all theta_k go to 0, reproducing pure-prograde priors.
        double thetaTotal = Math.Atan2(input.DFinalVlf.Y, vpParking + input.DFinalVlf.X);

        // Allocate theta_k over the FULL original N-pass plan; slice into
        // [startPassIndex..] for the current call. Using the locked initial
        // K-chain (not the live chain) keeps allocation deterministic
        // across recomputes: the same theta_k is planned for absolute pass k
        // whether we hit it on the initial plan or after k-1 priors.
        var (dvSeqInitial, vPreInitial, vPostInitial) = ComputeInitialKChain(
            mu, rp, tPark, totalPassCount, vTargetXy);
        double[] thetaKInitial = AllocatePlaneChangeLagrange(
            thetaTotal, dvSeqInitial, vPreInitial, vPostInitial);

        // remaining=1: just the final pass at T_final with residual dV
        // from the live orbit state. For initial N=1 it equals stock
        // single-burn (theta_k = thetaTotal, identical algebra). For
        // "last remaining pass of an N>1 plan" the planned final-pass
        // theta_k = thetaKInitial[N-1] rotates the remaining share.
        if (remainingCount == 1)
            return PlanSinglePass(
                source, input, now,
                vTargetXy, thetaKInitial[startPassIndex]);

        // K sub-sequence: slice of the original K_k = k+2 starting at
        // startPassIndex. For initial plan (start=0, total=N): K=(2,3,...,N).
        // For recompute at start=1 in N=3 plan: K=(3,) for the only
        // remaining prior pass (final pass at T_final is the last entry).
        int[] kSeq = BuildKSubSequence(totalPassCount, startPassIndex);
        int kTotal = 0;
        for (int i = 0; i < kSeq.Length; i++) kTotal += kSeq[i];

        // Schedule anchored to the FINAL pass landing at T_final. Pass
        // startPassIndex fires K_total parking periods before T_final;
        // intermediate passes step forward by K_k parking periods. Because
        // tPark is the locked original parking period (and the chained
        // orbit's periapsis stays at the locked CCI position even after a
        // Y-component burn, because dV.Z = 0), the vehicle ends up at
        // chained-periapsis at every scheduled time.
        var times = new SimTime[remainingCount];
        times[0] = new SimTime(input.TFinal.Seconds() - kTotal * tPark);
        for (int k = 1; k < remainingCount; k++)
            times[k] = new SimTime(times[k - 1].Seconds() + kSeq[k - 1] * tPark);

        if (times[0].Seconds() < now.Seconds() + EarliestPassMarginSec)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': first pass would fire {1:F0}s before now+margin " +
                "(needs K_total={2} parking periods = {3:F0}s of warning time); " +
                "reduce passes or pick later porkchop entry",
                source.Id,
                now.Seconds() + EarliestPassMarginSec - times[0].Seconds(),
                kTotal, kTotal * tPark));

        // Compute per-pass SMA / v_p analytically. The chain starts from
        // the live current orbit (parking for initial, chained for
        // recompute). K-integer schedule enforces |v_post_k| = vp(K_k);
        // direction (theta_k rotation in the tangent plane) is the free
        // parameter, planned by the Lagrange allocator below.
        double[] aBefore = new double[remainingCount];   // SMA before each remaining pass
        double[] vpBefore = new double[remainingCount];  // periapsis speed before each remaining pass
        aBefore[0] = currentOrbit.SemiMajorAxis;
        vpBefore[0] = Math.Sqrt(mu * (2.0 / rp - 1.0 / aBefore[0]));

        for (int k = 0; k < remainingCount - 1; k++)
        {
            double tPostK = kSeq[k] * tPark;
            double aPost = Math.Pow(Math.Pow(tPostK / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);
            double apoPost = 2.0 * aPost - rp;
            if (apoPost > soiLimit)
                return Fail(string.Format(CultureInfo.InvariantCulture,
                    "vehicle '{0}': pass {1} apoapsis {2:F0}m would exceed SOI limit " +
                    "{3:F0}m (K={4}); reduce passes",
                    source.Id, startPassIndex + k, apoPost, soiLimit, kSeq[k]));
            aBefore[k + 1] = aPost;
            vpBefore[k + 1] = Math.Sqrt(mu * (2.0 / rp - 1.0 / aPost));
        }

        if (!(vTargetXy > vpBefore[remainingCount - 1]))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': vTargetXy {1:F1}m/s <= pre-final v_p {2:F1}m/s; " +
                "K sequence over-pumped for the chosen passCount",
                source.Id, vTargetXy, vpBefore[remainingCount - 1]));

        // Per-pass dV in LOCAL VLF:
        //   priors: (vPost*cos(theta_k) - vPre, vPost*sin(theta_k), 0)
        //   final:  same X/Y plus D.Z to preserve any radial Lambert
        //           residual (numerical noise in tangential Hohmann, but
        //           still applied at the final pass to keep the planar
        //           K-chain invariant clean for the priors).
        // For the final pass, vPost is vTargetXy (the in-plane component
        // of the Lambert post-burn speed); D.Z carries the remaining
        // radial-out velocity, giving total post-burn |v| = vpTarget.
        // For stock Sol (theta_k ~ 0.01rad) dvX stays positive. Custom
        // systems with very large theta could push dvX negative on early
        // passes, which is mildly retrograde along the old local-X; the
        // chain still works because |v_post| = vPost is theta-independent.
        var dvVlfSeq = new double3[remainingCount];
        var dvMagSeq = new double[remainingCount];
        for (int k = 0; k < remainingCount; k++)
        {
            double vPre = vpBefore[k];
            double vPost = (k < remainingCount - 1)
                ? vpBefore[k + 1]
                : vTargetXy;
            double thetaK = thetaKInitial[startPassIndex + k];
            double dvX = vPost * Math.Cos(thetaK) - vPre;
            double dvY = vPost * Math.Sin(thetaK);
            double dvZ = (k == remainingCount - 1) ? dz : 0.0;
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
                source.Id, times[0].Seconds()));

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
                    FailureReason: $"vehicle '{source.Id}': pass {startPassIndex + k} produced an unbound orbit");

            // Inter-pass sanity: chained orbit must reach the next pass
            // time without escaping / encountering / impacting.
            if (k < remainingCount - 1)
            {
                string? interFail = CheckInterPass(fp, times[k + 1], k + 1);
                if (interFail != null)
                    return new PassPreviewResult(previews.ToArray(),
                        Failed: true, FailureReason: $"vehicle '{source.Id}': {interFail}");
            }

            prePatch = burnPatch;
        }

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.Plan: total={0} startIdx={1} remaining={2} " +
                "K_total={3} T_park={4:F1}s T_final={5:F0}s T_0={6:F0}s span={7:F0}s " +
                "vpTarget={8:F1}m/s vTargetXy={9:F1}m/s thetaTotal={10:F4}rad " +
                "thetaK[{11}]rad dV[{12}]m/s",
                totalPassCount, startPassIndex, remainingCount, kTotal, tPark,
                input.TFinal.Seconds(), times[0].Seconds(),
                input.TFinal.Seconds() - times[0].Seconds(), vpTarget, vTargetXy,
                thetaTotal, FormatThetaSlice(thetaKInitial, startPassIndex, remainingCount),
                FormatDvSeq(dvMagSeq)));

        return new PassPreviewResult(previews.ToArray(), Failed: false, FailureReason: null);
    }

    /// <summary>Largest N that fits the SOI envelope and time budget for
    /// the initial plan (startPassIndex=0). Probes from
    /// <paramref name="requestedN"/> downward; returns 1 if no multi-pass
    /// N is feasible. Used by the UI to auto-clamp.</summary>
    public static int LargestFeasibleN(
        Vehicle source, HohmannPlanInput input, SequenceBurnState state,
        double parkingPeriodSec, SimTime now, int requestedN)
    {
        int n = Math.Clamp(requestedN, 1, Splitter.MaxPasses);
        while (n > 1)
        {
            var probe = Plan(source, input, n, 0, parkingPeriodSec, state, now);

            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                    "[AFC] HohmannMultiPassPlanner.LargestFeasibleN: probe N={0} -> " +
                    "failed={1} reason='{2}' passes={3}",
                    n, probe.Failed, probe.FailureReason ?? "-", probe.Passes.Length));

            if (!probe.Failed) return n;
            n--;
        }
        return 1;
    }

    /// <summary>Total elapsed sim time the multi-pass schedule occupies
    /// (T_final - times[0]). Used by the UI to show "this multi-pass
    /// takes X parking periods of warning time".</summary>
    public static double GetSpanSeconds(double parkingPeriodSec, int passCount)
    {
        return GetKTotal(passCount) * parkingPeriodSec;
    }

    /// <summary>Outcome of <see cref="PrepareShiftedInput"/>:
    /// (possibly-shifted) <see cref="HohmannPlanInput"/> plus the integer
    /// number of parking periods T_final was pushed by.</summary>
    internal readonly record struct ShiftResult(HohmannPlanInput Input, int KShift);

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
        int passCount, double parkingPeriodSec, SimTime now)
    {
        if (passCount <= 1 || !(parkingPeriodSec > 0.0))
            return new ShiftResult(raw, 0);

        int kTotal = GetKTotal(passCount);
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
            return new ShiftResult(_shiftCacheInput, kShift);

        SimTime shiftedStart = new SimTime(rawTFinalSec + kShift * parkingPeriodSec);

        // Floor on min transit: defensive against an uninitialised
        // TransferInfo where MinTransferTimeOfFlight is 0 (would feed
        // SolveLambert a meaningless geometry). 60s is well below any
        // realistic moon / planet transit.
        double minTransitSec = Math.Max(60.0, info.MinTransferTimeOfFlight.Seconds());
        double maxTransitSec = Math.Max(
            minTransitSec + 1.0, info.MaxTransferTimeOfFlight.Seconds());

        const int TransitScanSteps = 64;
        double bestDvLen = double.MaxValue;
        OrbitalTransfers.TransferData? bestTd = null;

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
            if (dvLen >= bestDvLen) continue;

            bestDvLen = dvLen;
            bestTd = candidate;
        }

        if (bestTd == null)
            return new ShiftResult(raw, 0);

        // Cross-parent FinalizeLambert nudges Start by sub-parking-period
        // amounts to align the hyperbolic burn TA. The 0.5s threshold is
        // floating-point noise tolerance, NOT a fraction of tPark; if the
        // post-Lambert Start is more than ~noise below the requirement we
        // abandon the shift and let the planner's standard "needs K_total
        // parking periods" failure surface.
        const double LambertStartDriftToleranceSec = 0.5;
        if (bestTd.Start.Seconds() + LambertStartDriftToleranceSec < earliestAllowedSec)
            return new ShiftResult(raw, 0);

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
                "vInf={11:F1}m/s isCrossParent={12}",
                source.Id, key.TargetId, passCount, kTotal, kShift,
                rawTFinalSec, bestTd.Start.Seconds(),
                raw.DFinalVlf.Length(), bestDvLen,
                bestTd.Transit.Seconds(), apoTargetRadiusM, vInfMs,
                raw.IsCrossParent));

        _shiftCacheKey = key;
        _shiftCacheInput = shifted;
        _hasShiftCache = true;
        return new ShiftResult(shifted, kShift);
    }

    /// <summary>Drops the shift-input cache. Call from the UI's Reset and
    /// from Mod.Unload so a fresh session does not see stale entries
    /// (different save, recycled vehicle ids, etc.).</summary>
    public static void ResetShiftCache()
    {
        _hasShiftCache = false;
        _shiftCacheInput = default;
        _shiftCacheKey = default;
    }

    private readonly record struct ShiftCacheKey(
        string VehicleId,
        string TargetId,
        long RawTFinalBucketSec,
        int PassCount,
        long ParkingPeriodBucketSec,
        int KShift);

    private static ShiftCacheKey _shiftCacheKey;
    private static HohmannPlanInput _shiftCacheInput;
    private static bool _hasShiftCache;

    #region Internal helpers

    /// <summary>K sub-sequence for the remaining passes of an N-pass plan
    /// starting at <paramref name="startPassIndex"/>: K_k = startPassIndex
    /// + k + 2 for k = 0..N-startPassIndex-2. This preserves the original
    /// schedule when called from RecomputePass mid-execution.</summary>
    private static int[] BuildKSubSequence(int totalPassCount, int startPassIndex)
    {
        int remaining = totalPassCount - startPassIndex;
        int n = remaining - 1;
        if (n <= 0) return Array.Empty<int>();
        var k = new int[n];
        for (int i = 0; i < n; i++) k[i] = startPassIndex + i + 2;
        return k;
    }

    /// <summary>Per-pass K-prograde data for the original N-pass plan
    /// starting from a parking orbit of period <paramref name="tPark"/> at
    /// periapsis radius <paramref name="rp"/>. Returns dvSeq (delta_R per
    /// pass), vPre (pre-burn periapsis speed), and vPost (post-burn
    /// periapsis speed). Last entry is the final pass closing from K=N to
    /// <paramref name="vTargetXy"/> (the in-plane magnitude of the Lambert
    /// target). Independent of live drift, so the allocation is identical
    /// at initial plan and every recompute. Consumed by
    /// <see cref="AllocatePlaneChangeLagrange"/>.</summary>
    private static (double[] dvSeq, double[] vPre, double[] vPost) ComputeInitialKChain(
        double mu, double rp, double tPark, int totalPassCount, double vTargetXy)
    {
        var dvSeq = new double[totalPassCount];
        var vPre = new double[totalPassCount];
        var vPost = new double[totalPassCount];
        double aParking = Math.Pow(
            Math.Pow(tPark / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);
        double vpPrev = Math.Sqrt(mu * (2.0 / rp - 1.0 / aParking));
        for (int k = 0; k < totalPassCount - 1; k++)
        {
            int kIdx = k + 2;
            double tPostK = kIdx * tPark;
            double aPost = Math.Pow(
                Math.Pow(tPostK / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);
            double vpPostK = Math.Sqrt(mu * (2.0 / rp - 1.0 / aPost));
            vPre[k] = vpPrev;
            vPost[k] = vpPostK;
            dvSeq[k] = vpPostK - vpPrev;
            vpPrev = vpPostK;
        }
        vPre[totalPassCount - 1] = vpPrev;
        vPost[totalPassCount - 1] = vTargetXy;
        dvSeq[totalPassCount - 1] = vTargetXy - vpPrev;
        return (dvSeq, vPre, vPost);
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
        fp.ComputeCompleteTrajectory(FlightPlanPatchLimit, FlightPlanPolynomialOrder, target);
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
    /// numerical roundoff. For the last remaining pass of an N>1 plan,
    /// theta is the final pass's share of the Lagrange allocation -
    /// any prior plane rotation done by earlier passes is implicit in
    /// the live orbit's plane.
    ///
    /// Assumes T_final lands at a periapsis of the live orbit (rpLive
    /// = parking r_p). The current call paths satisfy this: K-integer
    /// scheduling places times[N-1] = T_final at the chained orbit's
    /// periapsis, and N=1 hits T_final at parking periapsis. A direct
    /// non-periapsis caller would under-/over-burn by (vpParking - vpLive)
    /// because the in-plane vTargetXy formula assumes apse geometry.</summary>
    private static PassPreviewResult PlanSinglePass(
        Vehicle source, HohmannPlanInput input, SimTime now,
        double vTargetXy, double theta)
    {
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(input.TFinal);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': no parking patch at t={1:F0}s",
                source.Id, input.TFinal.Seconds()));

        Orbit o = prePatch.Orbit;
        StateVectors svAt = o.GetStateVectorsAt(input.TFinal);
        double vpLive = svAt.VelocityCci.Length();
        double rpLive = svAt.PositionCci.Length();
        if (!(rpLive > 0.0))
            return Fail($"vehicle '{source.Id}': degenerate position at T_final");

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
                "vehicle '{0}': residual dV at T_final is non-positive ({1:F2}m/s); " +
                "priors already over-shot Lambert target",
                source.Id, dvFinalMag));

        var (fp, _) = BuildPassFlightPlan(
            source, prePatch, input.TFinal, dvVlf, input.Target);
        var single = new PassPreview(
            BurnTime: input.TFinal,
            DvVlf: dvVlf,
            EstimatedBurnTimeSec: 0.0,
            FlightPlan: fp);
        return new PassPreviewResult(new[] { single }, Failed: false, FailureReason: null);
    }

    private static PassPreviewResult Fail(string reason) =>
        new(Array.Empty<PassPreview>(), Failed: true, FailureReason: reason);

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

    private static string FormatThetaSlice(double[] thetaK, int start, int count)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < count; i++)
        {
            if (i > 0) sb.Append(", ");
            sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F5}", thetaK[start + i]);
        }
        return sb.ToString();
    }

    #endregion
}
