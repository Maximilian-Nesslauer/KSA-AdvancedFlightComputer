using System;
using System.Collections.Generic;
using System.Globalization;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;
// Each prior burn preserves the periapsis radial direction in CCI.
// The sum of prior period ratios is an integer for the initial parking orbit.
internal static class HohmannMultiPassPlanner
{
    private const double SoiEnvelopeFraction = 0.95;
    private const double EarliestPassMarginSec = 30.0;
    // A prior at K = 1 adds no speed and leaves the periapsis direction undefined on a circular orbit.
    private const double KFloor = 1.01;
    private const int TransitScanSteps = 64;
    internal readonly record struct HohmannPlanInput(
        IOrbiter Target,
        UniverseTime TFinal,
        double3 DFinalVlf,
        bool IsCrossParent,
        double VInfMs,
        double ApoTargetRadiusMeters);
    // Keep parkingPeriodSec fixed when recalculating because the period after a burn is no longer the parking period.
    public static PassPreviewResult Plan(
        Vehicle source,
        HohmannPlanInput input,
        int totalPassCount,
        int startPassIndex,
        double parkingPeriodSec,
        SequenceBurnState vehicleState,
        UniverseTime now,
        SplitMode mode)
    {
        var failure = GetPlanGeometry(source, input, totalPassCount, startPassIndex,
            parkingPeriodSec, out var geometry);
        if (failure != null) return failure.Value;
        var (_, _, tPark, _, remainingCount, vpTarget,
            vTargetXy, _, vpParking, thetaTotal, _) = geometry;
        // Absolute shares prevent already applied plane rotation from being allocated again.
        double[] thetaKAbsolute = AllocateAbsoluteThetaSchedule(
            thetaTotal, vpParking, vTargetXy, totalPassCount);
        if (remainingCount == 1)
        {
            UniverseTime? lastBurnTime = (startPassIndex == 0)
                ? input.TFinal
                : source.Orbit.GetNextPeriapsisTime(now);
            if (lastBurnTime is not UniverseTime burnTime)
                return Fail($"vehicle '{source.Id}': chained orbit has no next periapsis",
                    PassPlanFailure.Other);
            return PlanSinglePass(source, input, now,
                burnTime, vTargetXy, thetaKAbsolute[startPassIndex]);
        }

        failure = BuildSchedule(source, input, startPassIndex, geometry, vehicleState, now, mode,
            out var kSeq, out var times, out double targetSumPeriods);
        if (failure != null) return failure.Value;
        double[] thetaK = new double[remainingCount];
        for (int i = 0; i < remainingCount; i++)
            thetaK[i] = thetaKAbsolute[startPassIndex + i];
        double3[] dvVlfSeq = BuildBurnVectors(geometry, kSeq, thetaK, input.DFinalVlf.Z);
        var dvMagSeq = new double[remainingCount];
        for (int k = 0; k < remainingCount; k++) dvMagSeq[k] = dvVlfSeq[k].Length();
        double[] burnTimes = EstimateBurnTimes(dvMagSeq, vehicleState);
        var result = BuildPreviewChain(source, input, startPassIndex, times, dvVlfSeq, burnTimes);
        if (result.Failed) return result;
        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.Plan: total={0} startIdx={1} remaining={2} " +
                "mode={3} sumK={4:F2} T_park={5:F1}s T_final={6:F0}s T_0={7:F0}s span={8:F0}s " +
                "vpTarget={9:F1}m/s vTargetXy={10:F1}m/s thetaTotal={11:F4}rad " +
                "K[{12}] thetaK[{13}]rad dV[{14}]m/s advisory='{15}'",
                totalPassCount, startPassIndex, remainingCount, mode, targetSumPeriods, tPark,
                input.TFinal.Seconds(), times[0].Seconds(),
                (input.TFinal - times[0]).Seconds(), vpTarget, vTargetXy,
                thetaTotal, FormatSequence(kSeq, "F2"), FormatSequence(thetaK, "F5"), FormatSequence(dvMagSeq, "F1"),
                result.Advisory ?? "-"));

        return result;
    }
    private readonly record struct PlanGeometry(
        double Mu, double Rp, double TPark, double SoiLimit, int RemainingCount,
        double VpTarget, double VTargetXy, double APark, double VpParking,
        double ThetaTotal, double VpLive);

    private static PassPreviewResult? GetPlanGeometry(
        Vehicle source, HohmannPlanInput input, int totalPassCount,
        int startPassIndex, double parkingPeriodSec, out PlanGeometry geometry)
    {
        geometry = default;
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
        double dz = input.DFinalVlf.Z;
        double vTargetXy = Math.Sqrt(Math.Max(0.0, vpTarget * vpTarget - dz * dz));

        if (!(vTargetXy > 0.0))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': Lambert tangential component degenerate " +
                "(|D.Z|={1:F1} >= vpTarget={2:F1}); porkchop entry has near-purely-radial dV",
                source.Id, Math.Abs(dz), vpTarget), PassPlanFailure.Other);
        double aPark = Math.Pow(
            Math.Pow(tPark / (2.0 * Math.PI), 2.0) * mu, 1.0 / 3.0);
        double vpParking = Math.Sqrt(mu * (2.0 / rp - 1.0 / aPark));
        double thetaTotal = Math.Atan2(input.DFinalVlf.Y, vpParking + input.DFinalVlf.X);
        double vpLive = Math.Sqrt(mu * (2.0 / rp - 1.0 / currentOrbit.SemiMajorAxis));

        geometry = new PlanGeometry(mu, rp, tPark, soiLimit, remainingCount,
            vpTarget, vTargetXy, aPark, vpParking, thetaTotal, vpLive);
        return null;
    }

    private static PassPreviewResult? BuildSchedule(
        Vehicle source, HohmannPlanInput input, int startPassIndex,
        PlanGeometry geometry, SequenceBurnState vehicleState, UniverseTime now, SplitMode mode,
        out double[] kSeq, out UniverseTime[] times, out double targetSumPeriods)
    {
        kSeq = Array.Empty<double>();
        times = Array.Empty<UniverseTime>();
        targetSumPeriods = 0.0;
        var (mu, rp, tPark, soiLimit, remainingCount, _,
            vTargetXy, aPark, _, _, vpLive) = geometry;
        Orbit currentOrbit = source.Orbit;
        int priors = remainingCount - 1;
        var schedule = BuildRealKSchedule(
            mu, rp, aPark, soiLimit, vpLive, vTargetXy,
            remainingCount, mode, vehicleState);
        if (schedule.K == null)
        {
            PassPlanFailure kind = schedule.Failure ?? PassPlanFailure.Other;
            string detail = kind switch
            {
                PassPlanFailure.FuelShort => "vehicle has insufficient fuel for this N",
                PassPlanFailure.SoiCeiling =>
                    "a prior orbit would reach past the parent SOI envelope at this N",
                PassPlanFailure.ParabolicVp => "a prior orbit would reach escape speed at this N",
                _ => "the requested split has no usable prior schedule",
            };
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': K-schedule build failed; {1}",
                source.Id, detail), kind);
        }

        kSeq = schedule.K!;
        UniverseTime timesZero;
        if (startPassIndex == 0)
        {
            double sumRaw = 0.0;
            for (int i = 0; i < priors; i++) sumRaw += kSeq[i];
            targetSumPeriods = Math.Round(sumRaw);
            timesZero = input.TFinal - targetSumPeriods * tPark;
        }
        else
        {
            if (currentOrbit.GetNextPeriapsisTime(now) is not UniverseTime nextPe)
                return Fail($"vehicle '{source.Id}': chained orbit has no next periapsis",
                    PassPlanFailure.Other);
            timesZero = nextPe;
            targetSumPeriods = (input.TFinal - timesZero).Seconds() / tPark;
        }
        var failure = AdjustLastPeriod(source, geometry, kSeq, targetSumPeriods);
        if (failure != null) return failure;
        UniverseTime earliestAllowed = now + EarliestPassMarginSec;
        if (timesZero < earliestAllowed)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': first pass would fire {1:F0}s before now+margin " +
                "(needs ~{2:F1} parking periods = {3:F0}s of warning time); " +
                "pick later porkchop entry",
                source.Id,
                (earliestAllowed - timesZero).Seconds(),
                targetSumPeriods, targetSumPeriods * tPark),
                PassPlanFailure.TimeBudget);
        times = new UniverseTime[remainingCount];
        times[0] = timesZero;
        for (int k = 1; k < remainingCount; k++)
            times[k] = times[k - 1] + kSeq[k - 1] * tPark;
        return null;
    }

    private static PassPreviewResult? AdjustLastPeriod(
        Vehicle source, PlanGeometry geometry, double[] kSeq, double targetSumPeriods)
    {
        var (mu, rp, _, soiLimit, _, _,
            vTargetXy, aPark, _, _, _) = geometry;
        int priors = kSeq.Length;
        if (!(kSeq[0] > KFloor))
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': K_0 = {1:F3} below floor {2:F2}; " +
                "first pass would be essentially a zero-dV burn (reduce N)",
                source.Id, kSeq[0], KFloor), PassPlanFailure.KFloor);
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

        return null;
    }

    private static double3[] BuildBurnVectors(
        PlanGeometry geometry, double[] kSeq, double[] thetaK, double radialDv)
    {
        int priors = kSeq.Length;
        var dvVlfSeq = new double3[priors + 1];
        double vPre = geometry.VpLive;
        for (int k = 0; k <= priors; k++)
        {
            double vPost = k < priors
                ? Math.Sqrt(geometry.Mu * (2.0 / geometry.Rp
                    - 1.0 / (geometry.APark * Math.Pow(kSeq[k], 2.0 / 3.0))))
                : geometry.VTargetXy;
            // Prior burns have no radial component, so they preserve the shared burn position.
            dvVlfSeq[k] = new double3(vPost * Math.Cos(thetaK[k]) - vPre,
                vPost * Math.Sin(thetaK[k]), k == priors ? radialDv : 0.0);
            vPre = vPost;
        }
        return dvVlfSeq;
    }

    private static PassPreviewResult BuildPreviewChain(
        Vehicle source, HohmannPlanInput input, int startPassIndex,
        UniverseTime[] times, double3[] dvVlfSeq, double[] burnTimes)
    {
        int remainingCount = times.Length;
        var previews = new List<PassPreview>(remainingCount);
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(times[0]);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return Fail(string.Format(CultureInfo.InvariantCulture,
                "vehicle '{0}': no current-orbit patch at t={1:F0}s",
                source.Id, times[0].Seconds()), PassPlanFailure.Other);

        for (int k = 0; k < remainingCount; k++)
        {
            double3 dvVlf = dvVlfSeq[k];

            var (fp, burnPatch) = MultiPassForwardChainPlanner.BuildPassFlightPlan(
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
        string? finalAdvisory = CheckFinalPassAdvisory(
            previews[remainingCount - 1].FlightPlan, source, input.Target);

        return new PassPreviewResult(previews.ToArray(), Failed: false,
            FailureReason: null, Advisory: finalAdvisory);
    }

    // Rounding the last period can make one pass count fail even when a larger count succeeds.
    public static int LargestFeasibleN(
        Vehicle source, HohmannPlanInput input, SequenceBurnState state,
        double parkingPeriodSec, UniverseTime now, int requestedN, SplitMode mode,
        out string? firstFailureReason, out PassPlanFailure firstFailureKind)
    {
        firstFailureReason = null;
        firstFailureKind = PassPlanFailure.None;
        int n = Math.Clamp(requestedN, 1, Splitter.MaxPasses);
        while (n > 1)
        {
            var probe = Plan(source, input, n, 0, parkingPeriodSec, state, now, mode);

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
    // ShiftedTransit is default when no shift was needed. ScanAdvisory describes the fallback trajectory when no clean candidate was found.
    internal readonly record struct ShiftResult(
        HohmannPlanInput Input, int KShift, string? ScanAdvisory = null,
        UniverseTime ShiftedTransit = default);
    internal static ShiftResult PrepareShiftedInput(
        HohmannPlanInput raw, Vehicle source, OrbitalTransfers.TransferInfo info,
        int passCount, double parkingPeriodSec, UniverseTime now,
        SplitMode mode, SequenceBurnState state)
    {
        if (passCount <= 1 || !(parkingPeriodSec > 0.0))
            return new ShiftResult(raw, 0);
        // OrbitalTransfers.FinalizeLambert adjusts departure times for transfers between different parents, which can break the integer period constraint.
        if (raw.IsCrossParent)
            return new ShiftResult(raw, 0);

        int kTotal = EstimateRequiredKTotal(
            raw, source, passCount, mode, state, parkingPeriodSec);
        if (kTotal <= 0)
            return new ShiftResult(raw, 0);

        UniverseTime earliestAllowed = now
            + (kTotal * parkingPeriodSec + EarliestPassMarginSec);
        if (raw.TFinal >= earliestAllowed)
            return new ShiftResult(raw, 0);

        int kShift = (int)Math.Ceiling(
            (earliestAllowed - raw.TFinal).Seconds() / parkingPeriodSec);
        if (kShift <= 0)
            return new ShiftResult(raw, 0);

        var key = BuildShiftKey(raw, source, info, passCount, parkingPeriodSec, kShift);
        if (_hasShiftCache && key == _shiftCacheKey)
            return new ShiftResult(_shiftCacheInput, kShift, _shiftCacheAdvisory, _shiftCacheTransit);

        UniverseTime shiftedStart = raw.TFinal + kShift * parkingPeriodSec;
        if (!ScanShiftedTransfers(source, info, shiftedStart,
                out var bestTd, out double bestDvLen, out string? scanAdvisory))
            return new ShiftResult(raw, 0);
        double apoTargetRadiusM = 0.0;
        var fp = FlightPlan.CreateUninitialized(source.Hash);
        OrbitalTransfers.BuildFlightPlan(
            ref fp, info, bestTd.Start, bestTd.TransferDvVlf, out _, out _);
        if (fp.Patches.Count > 0)
        {
            double apo = fp.Patches[0].Orbit.Apoapsis;
            if (double.IsFinite(apo) && apo > source.Orbit.Periapsis)
                apoTargetRadiusM = apo;
        }

        var shifted = raw with
        {
            TFinal = bestTd.Start,
            DFinalVlf = bestTd.TransferDvVlf,
            ApoTargetRadiusMeters = apoTargetRadiusM,
        };

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.PrepareShiftedInput: vehicle='{0}' " +
                "target='{1}' passCount={2} K_total={3} K_shift={4} " +
                "rawTFinal={5:F0}s shiftedTFinal={6:F0}s rawDv={7:F1}m/s " +
                "shiftedDv={8:F1}m/s shiftedTransit={9:F0}s apoTarget={10:F0}m " +
                "vInf={11:F1}m/s isCrossParent={12} scanAdvisory='{13}'",
                source.Id, key.TargetId, passCount, kTotal, kShift,
                raw.TFinal.Seconds(), bestTd.Start.Seconds(),
                raw.DFinalVlf.Length(), bestDvLen,
                bestTd.Transit.Seconds(), apoTargetRadiusM, shifted.VInfMs,
                raw.IsCrossParent, scanAdvisory ?? "-"));

        _shiftCacheKey = key;
        _shiftCacheInput = shifted;
        _shiftCacheAdvisory = scanAdvisory;
        _shiftCacheTransit = bestTd.Transit;
        _hasShiftCache = true;
        return new ShiftResult(shifted, kShift, scanAdvisory, bestTd.Transit);
    }
    private static ShiftCacheKey BuildShiftKey(
        HohmannPlanInput raw, Vehicle source, OrbitalTransfers.TransferInfo info,
        int passCount, double parkingPeriodSec, int kShift)
    {
        return new ShiftCacheKey(
            VehicleId: source.Id,
            TargetId: (info.Target as Astronomical)?.Id ?? string.Empty,
            RawTFinalBucketSec: (long)raw.TFinal.Seconds(),
            PassCount: passCount,
            ParkingPeriodBucketSec: (long)Math.Round(parkingPeriodSec),
            KShift: kShift,
            Source: source,
            Info: info,
            MinTransit: info.MinTransferTimeOfFlight,
            MaxTransit: info.MaxTransferTimeOfFlight,
            ImpactMargin: source.BoundingSphereRadiusBody,
            RawInput: raw,
            ParkingPeriod: parkingPeriodSec,
            Geometry: ShiftGeometry.Capture(info, raw.TFinal + kShift * parkingPeriodSec));
    }

    private static bool ScanShiftedTransfers(
        Vehicle source, OrbitalTransfers.TransferInfo info, UniverseTime shiftedStart,
        out OrbitalTransfers.TransferData bestTd, out double bestDvLen, out string? scanAdvisory)
    {
        bestTd = null!;
        bestDvLen = 0.0;
        scanAdvisory = null;
        double minTransitSec = Math.Max(60.0, info.MinTransferTimeOfFlight.Seconds());
        double maxTransitSec = Math.Max(
            minTransitSec + 1.0, info.MaxTransferTimeOfFlight.Seconds());
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
                Transit = new UniverseTime(transitSec),
                ClosestApproachDistance = double.MaxValue,
            };
            if (!OrbitalTransfers.SolveLambert(info, ref candidate)) continue;

            double dvLen = candidate.TransferDvVlf.Length();
            if (!double.IsFinite(dvLen) || dvLen <= 0.0) continue;
            if (dvLen >= bestCleanDv && dvLen >= bestDirtyDv) continue;

            if (!TryClassifyCandidate(source, info, candidate, out string? dirtyReason)) continue;
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
            return false;
        }
        return true;
    }

    private static bool TryClassifyCandidate(
        Vehicle source, OrbitalTransfers.TransferInfo info, OrbitalTransfers.TransferData candidate,
        out string? dirtyReason)
    {
        dirtyReason = null;
        var probeFp = FlightPlan.CreateUninitialized(source.Hash);
        probeFp.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
        if (!OrbitalTransfers.BuildFlightPlan(
                ref probeFp, info, candidate.Start, candidate.TransferDvVlf,
                out _, out _))
            return false;
        // Detached plans have no worker to advance the incremental terrain search.
        if (probeFp.ImpactSearchUnresolved)
            probeFp.ComputeCompleteTrajectory(out _, 5, 8, info.Target,
                resolveImpactsCompletely: true);
        UniverseTime arrival = candidate.Start + candidate.Transit;
        dirtyReason = ClassifyScanCandidate(probeFp, info, arrival);
        return true;
    }

    public static void ResetShiftCache()
    {
        _hasShiftCache = false;
        _shiftCacheInput = default;
        _shiftCacheKey = default;
        _shiftCacheAdvisory = null;
        _shiftCacheTransit = default;
    }

    // The cache key can keep transfer data for a disposed vehicle reachable.
    public static void OnVehicleDisposed(string vehicleId)
    {
        if (!_hasShiftCache || _shiftCacheKey.VehicleId != vehicleId) return;
        ResetShiftCache();
    }
    // SplitMode changes the scan only through KShift, which is calculated before the cache lookup.
    private readonly record struct ShiftCacheKey(
        string VehicleId,
        string TargetId,
        long RawTFinalBucketSec,
        int PassCount,
        long ParkingPeriodBucketSec,
        int KShift,
        Vehicle Source,
        OrbitalTransfers.TransferInfo Info,
        UniverseTime MinTransit,
        UniverseTime MaxTransit,
        double ImpactMargin,
        HohmannPlanInput RawInput,
        double ParkingPeriod,
        ShiftGeometry Geometry);

    private readonly record struct OrbitSnapshot(
        IParentBody Parent, double Mu, double SemiMajorAxis, double Eccentricity,
        double Inclination, double LongitudeOfAscendingNode, double ArgumentOfPeriapsis,
        UniverseTime TimeAtPeriapsis)
    {
        public static OrbitSnapshot Capture(Orbit orbit) => new(
            orbit.Parent, orbit.Mu, orbit.SemiMajorAxis, orbit.Eccentricity,
            orbit.Inclination, orbit.LongitudeOfAscendingNode, orbit.ArgumentOfPeriapsis,
            orbit.TimeAtPeriapsis);
    }

    private readonly record struct ShiftGeometry(
        Vehicle Vehicle, IOrbiter Source, IOrbiter Target,
        OrbitSnapshot SourceOrbit, OrbitSnapshot TargetOrbit, OrbitSnapshot StartingOrbit,
        OrbitSnapshot VehicleOrbit, OrbitSnapshot? DepartureOrbit,
        UniverseTime PatchStart, UniverseTime PatchEnd)
    {
        public static ShiftGeometry Capture(OrbitalTransfers.TransferInfo info, UniverseTime start)
        {
            // OrbitalTransfers.BuildFlightPlan uses the live plan first, then the burn plan.
            PatchedConic? patch = info.Vehicle.FlightPlan.TryFindPatch(start);
            if (patch == null) info.Vehicle.FlightComputer.BurnPlan.FindPatchAt(start, out patch);
            return new ShiftGeometry(info.Vehicle, info.Source, info.Target,
                OrbitSnapshot.Capture(info.Source.Orbit), OrbitSnapshot.Capture(info.Target.Orbit),
                OrbitSnapshot.Capture(info.Starting), OrbitSnapshot.Capture(info.Vehicle.Orbit),
                patch == null ? null : OrbitSnapshot.Capture(patch.Orbit),
                patch?.StartTime ?? default, patch?.EndTime ?? default);
        }
    }

    private static ShiftCacheKey _shiftCacheKey;
    private static HohmannPlanInput _shiftCacheInput;
    private static string? _shiftCacheAdvisory;
    private static UniverseTime _shiftCacheTransit;
    private static bool _hasShiftCache;

    #region Internal helpers
    private readonly struct RealKScheduleResult
    {
        public double[]? K { get; init; }
        public PassPlanFailure? Failure { get; init; }
    }

    // vpMaxPrior is the speed whose apoapsis reaches the SOI envelope.
    private static RealKScheduleResult BuildRealKSchedule(
        double mu, double rp, double aPark, double soiLimit,
        double vpLive, double vTargetXy,
        int remainingCount, SplitMode mode, SequenceBurnState state)
    {
        int priors = remainingCount - 1;
        if (priors < 1) return Refuse(PassPlanFailure.Other);

        double totalDv = vTargetXy - vpLive;
        if (!(totalDv > 0.0)) return Refuse(PassPlanFailure.Other);

        PassAllocation[] alloc = Splitter.Allocate(totalDv, remainingCount, mode, state);

        var K = new double[priors];
        double aMaxBySoi = (rp + soiLimit) / 2.0;
        double vpMaxPrior = (aMaxBySoi > rp)
            ? Math.Sqrt(Math.Max(0.0, mu * (2.0 / rp - 1.0 / aMaxBySoi))) * 0.999
            : 0.0;

        double vpCum = vpLive;
        bool reachedCap = false;
        for (int k = 0; k < priors; k++)
        {
            double targetVp = vpCum + alloc[k].DvCapacityMs;
            if (targetVp > vpMaxPrior)
            {
                // The chain cannot gain more speed after it reaches the SOI cap.
                if (reachedCap)
                    return Refuse(PassPlanFailure.SoiCeiling);
                if (vpMaxPrior <= vpCum)
                    return Refuse(PassPlanFailure.SoiCeiling);
                targetVp = vpMaxPrior;
                reachedCap = true;
            }
            if (!(targetVp > vpCum))
                return Refuse(PassPlanFailure.FuelShort);
            vpCum = targetVp;
            double term = 2.0 / rp - vpCum * vpCum / mu;
            if (!(term > 0.0))
                return Refuse(PassPlanFailure.ParabolicVp);
            double aPost = 1.0 / term;
            K[k] = Math.Pow(aPost / aPark, 1.5);
        }

        return new RealKScheduleResult { K = K };
    }

    private static RealKScheduleResult Refuse(PassPlanFailure kind)
        => new() { Failure = kind };
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
    // Equal speed increments keep absolute angle shares independent of fuel drain and split mode.
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
    // Minimize sum(vPre * vPost * theta^2 / (2 * deltaV)) subject to sum(theta) = thetaTotal.
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

    private static string? CheckInterPass(FlightPlan priorFp, UniverseTime nextTime, int passIndex)
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
    // TransferTask.WorkerTask.CalculateAutomaticTransfer rejects impact only before arrival.
    private static string? ClassifyScanCandidate(
        FlightPlan fp, OrbitalTransfers.TransferInfo info, UniverseTime arrival)
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
    private static double[] EstimateBurnTimes(double[] dvSeq, SequenceBurnState state)
    {
        var burnTimes = new double[dvSeq.Length];
        if (!state.HasUsableEngines) return burnTimes;

        var drain = new Splitter.SequenceDrain(state);
        for (int k = 0; k < dvSeq.Length; k++)
            burnTimes[k] = drain.ByDv(dvSeq[k]).timeUsed;
        return burnTimes;
    }
    // A recomputed final pass must use the live periapsis time, which finite burns can shift.
    private static PassPreviewResult PlanSinglePass(
        Vehicle source, HohmannPlanInput input, UniverseTime now,
        UniverseTime burnTime, double vTargetXy, double theta)
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

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] HohmannMultiPassPlanner.PlanSinglePass: vehicle='{0}' " +
                "burnTime={1:F0}s (input.TFinal={2:F0}s, drift={3:F0}s) " +
                "vpLive={4:F1}m/s vTargetXy={5:F1}m/s theta={6:F5}rad " +
                "dV=({7:F1},{8:F1},{9:F1})m/s |dV|={10:F1}m/s",
                source.Id, burnTime.Seconds(), input.TFinal.Seconds(),
                (burnTime - input.TFinal).Seconds(),
                vpLive, vTargetXy, theta, dvX, dvY, dvZ, dvFinalMag));

        var (fp, _) = MultiPassForwardChainPlanner.BuildPassFlightPlan(
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

    private static string FormatSequence(double[] values, string format)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < values.Length; i++)
        {
            if (i > 0) sb.Append(", ");
            sb.Append(values[i].ToString(format, CultureInfo.InvariantCulture));
        }
        return sb.ToString();
    }

    #endregion
}
