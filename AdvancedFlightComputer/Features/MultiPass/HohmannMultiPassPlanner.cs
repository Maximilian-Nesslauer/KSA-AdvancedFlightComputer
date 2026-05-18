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
/// asymptote direction exactly.
///
/// Algorithm:
///   1. Pick K sequence: minimum monotonic increasing integers starting at
///      2, i.e. K = (2, 3, ..., N) for N passes. K_total = N(N+1)/2 - 1.
///   2. Schedule: times[0] = T_final - K_total * T_park, then
///      times[k+1] = times[k] + K_k * T_park. By parking-orbit periodicity,
///      vehicle on parking orbit at times[0] is at the same CCI direction
///      as at T_final.
///   3. Each prior pass fires prograde at periapsis of its chained orbit.
///      Because T_post_k = K_k * T_park exactly, the chained orbit's
///      periapsis returns to the same CCI direction every K_k parking
///      periods - so pass k+1 lands at the same CCI direction as pass k.
///   4. Final pass at T_final, vehicle still at the locked CCI direction.
///      Stock D_finalVlf direction is preserved because the VLF basis at
///      this periapsis is identical to the parking-orbit VLF basis at
///      T_final (both have tangential velocity at the same position).
///   5. Per-pass dV is COMPUTED from the K-sequence via vis-viva, not
///      free-allocated by a splitter. SplitMode is therefore irrelevant
///      for Hohmann; the only user choice is N.
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

        // remaining=1: just the final pass at T_final with residual dV
        // from the live orbit state. For initial N=1 it equals stock
        // single-burn; for "last remaining pass of an N>1 plan" it
        // converges to whatever brings live v_p to v_p_target.
        if (remainingCount == 1)
            return PlanSinglePass(source, input, now);

        double3 dFinalDir = input.DFinalVlf.NormalizeOrZero();
        if (dFinalDir.LengthSquared() < 0.5)
            return Fail($"vehicle '{source.Id}': stock Hohmann dV direction is zero");

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
        // orbit at this point has periapsis at the locked CCI direction
        // because prior passes were prograde-at-periapsis), vehicle ends
        // up at chained-periapsis at every scheduled time.
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

        // Compute per-pass SMA / v_p / dV analytically. The chain starts
        // from the live current orbit (parking for initial, chained for
        // recompute). K-integer schedule enforces the impulsive-prograde-
        // at-periapsis limit so the chain is exact.
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

        // Per-pass dV: priors raise v_p step by step; final pass closes
        // the gap to v_p_target. For recompute starting mid-chain, the
        // dV adapts to the live state (vpBefore[0] = chained orbit's v_p).
        double vpTarget = ComputeVpTarget(input, mu, rp);
        if (!(vpTarget > vpBefore[remainingCount - 1]))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': v_p target {1:F1}m/s <= pre-final v_p {2:F1}m/s; " +
                "K sequence over-pumped for the chosen passCount",
                source.Id, vpTarget, vpBefore[remainingCount - 1]));

        var dvSeq = new double[remainingCount];
        for (int k = 0; k < remainingCount - 1; k++)
            dvSeq[k] = vpBefore[k + 1] - vpBefore[k];
        dvSeq[remainingCount - 1] = vpTarget - vpBefore[remainingCount - 1];

        // Per-pass burn time estimate via multi-stage Tsiolkovsky drain.
        double[] burnTimes = EstimateBurnTimes(dvSeq, vehicleState);

        // Forward-chain flight plans for the preview. The K-scheduled
        // times mean each chained orbit's periapsis lands at times[k+1]
        // by construction.
        var previews = new List<PassPreview>(remainingCount);
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(times[0]);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': no current-orbit patch at t={1:F0}s",
                source.Id, times[0].Seconds()));

        for (int k = 0; k < remainingCount; k++)
        {
            // Prior passes: pure prograde at periapsis. Final pass:
            // stock direction (preserves asymptote) scaled to residual dV.
            double3 dvVlf = k < remainingCount - 1
                ? new double3(dvSeq[k], 0.0, 0.0)
                : dFinalDir * dvSeq[k];

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

        DefaultCategory.Log.Info(string.Format(CultureInfo.InvariantCulture,
            "[AFC] HohmannMultiPassPlanner.Plan: total={0} startIdx={1} remaining={2} " +
            "K_total={3} T_park={4:F1}s T_final={5:F0}s T_0={6:F0}s span={7:F0}s " +
            "vpTarget={8:F1}m/s dV[{9}]m/s",
            totalPassCount, startPassIndex, remainingCount, kTotal, tPark,
            input.TFinal.Seconds(), times[0].Seconds(),
            input.TFinal.Seconds() - times[0].Seconds(), vpTarget,
            FormatDvSeq(dvSeq)));

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
        if (passCount <= 1) return 0.0;
        int[] kSeq = BuildKSequence(passCount);
        int kTotal = 0;
        for (int i = 0; i < kSeq.Length; i++) kTotal += kSeq[i];
        return kTotal * parkingPeriodSec;
    }

    #region Internal helpers

    /// <summary>K sequence for a full N-pass plan: (2, 3, ..., N). K_k is
    /// the integer number of parking periods between pass k and pass k+1
    /// (equals T_post_k / T_park).</summary>
    private static int[] BuildKSequence(int passCount)
        => BuildKSubSequence(passCount, 0);

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

    /// <summary>Single pass = the stock final-pass case with the residual
    /// dV computed from the live orbit state. For N=1 from the UI, the
    /// vehicle is still in the parking orbit so residual = full stock
    /// |D_final| and the result is identical to the stock single burn.</summary>
    private static PassPreviewResult PlanSinglePass(
        Vehicle source, HohmannPlanInput input, SimTime now)
    {
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(input.TFinal);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': no parking patch at t={1:F0}s",
                source.Id, input.TFinal.Seconds()));

        Orbit o = prePatch.Orbit;
        double mu = o.Mu;
        StateVectors svAt = o.GetStateVectorsAt(input.TFinal);
        double vpLive = svAt.VelocityCci.Length();
        double rpLive = svAt.PositionCci.Length();
        if (!(rpLive > 0.0))
            return Fail($"vehicle '{source.Id}': degenerate position at T_final");

        double vpTarget = ComputeVpTarget(input, mu, rpLive);
        double dvFinalMag = vpTarget - vpLive;
        if (!(dvFinalMag > 0.0))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': residual dV at T_final is non-positive ({1:F2}m/s); " +
                "priors already over-shot v_p_target",
                source.Id, dvFinalMag));

        double3 dvFinalDir = input.DFinalVlf.NormalizeOrZero();
        if (dvFinalDir.LengthSquared() < 0.5)
            return Fail($"vehicle '{source.Id}': stock Hohmann dV direction is zero");

        double3 dvVlf = dvFinalDir * dvFinalMag;
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

    #endregion
}
