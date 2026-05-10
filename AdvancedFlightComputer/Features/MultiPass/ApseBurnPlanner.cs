using System.Collections.Generic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Forward-chains N apse-burn flight plans (burnTa = Zero for Set
/// Apoapsis, Pi for Set Periapsis). All passes share one dV unit
/// vector: a prograde-at-apsis kick preserves the line of apsides.
/// Failed=true partial result on missing parking patch, SOI escape /
/// encounter before the next pass, or unbound post-burn orbit.
/// </summary>
internal static class ApseBurnPlanner
{
    // Slack so chained TimeOfTrueAnomaly starts strictly after pass i.
    private const double NextOccurrenceEpsilonSec = 1.0;

    // Stock live BurnPlan uses ~100 patches; our previews settle in
    // 1-2 even with an SOI transition, 8 is generous headroom.
    private const int FlightPlanPatchLimit = 8;

    // Per-segment encounter precision; lower than stock live (16) is
    // fine for a preview that only needs an indicative trajectory.
    private const int FlightPlanPolynomialOrder = 8;

    public static PassPreviewResult Plan(
        Vehicle source,
        double3 totalDvVlf,
        TrueAnomaly burnTa,
        PassAllocation[] allocations,
        SimTime now)
    {
        double3 dvDir = totalDvVlf.NormalizeOrZero();
        if (dvDir.LengthSquared() < 0.5)
            return new PassPreviewResult(System.Array.Empty<PassPreview>(), Failed: true,
                FailureReason: "zero dV direction");

        // Caller must guard Eccentricity < 1 so this returns >= now.
        SimTime burnTime = source.Orbit.TimeOfTrueAnomaly(burnTa, now);

        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(burnTime);
        if (prePatch == null || prePatch.PrimaryBody == null)
            return new PassPreviewResult(System.Array.Empty<PassPreview>(), Failed: true,
                FailureReason: $"no parking patch at t={burnTime.Seconds():F0}s");

        var results = new List<PassPreview>(allocations.Length);

        for (int i = 0; i < allocations.Length; i++)
        {
            double3 dvVlf = dvDir * allocations[i].DvCapacityMs;

            var (fp, burnPatch) = BuildPassFlightPlan(source, prePatch, burnTime, dvVlf);
            results.Add(new PassPreview(
                BurnTime: burnTime,
                DvVlf: dvVlf,
                EstimatedBurnTimeSec: allocations[i].EstimatedBurnTimeSec,
                FlightPlan: fp));

            if (!burnPatch.Orbit.IsBound())
                return new PassPreviewResult(results.ToArray(), Failed: true,
                    $"pass {i} produced an unbound orbit");

            if (i == allocations.Length - 1)
                break;

            SimTime nextBurnTime = NextOccurrenceAfter(burnPatch.Orbit, burnTa, burnTime);

            // Either transition flips the reference frame for the
            // chained TimeOfTrueAnomaly; abort if it happens before
            // the next planned pass.
            foreach (PatchedConic p in fp.Patches)
            {
                if (p.EndTime >= nextBurnTime) continue;
                if (p.EndTransition == PatchTransition.Escape)
                    return new PassPreviewResult(results.ToArray(), Failed: true,
                        $"pass {i} escapes SOI before next pass");
                if (p.EndTransition == PatchTransition.Encounter)
                    return new PassPreviewResult(results.ToArray(), Failed: true,
                        $"pass {i} encounters another body before next pass");
            }

            prePatch = burnPatch;
            burnTime = nextBurnTime;
        }

        return new PassPreviewResult(results.ToArray(), Failed: false, FailureReason: null);
    }

    /// <summary>One pass's flight plan: a burn patch plus SOI
    /// propagation. Caller chains pass i+1 off the returned
    /// <paramref name="burnPatch"/>.</summary>
    private static (FlightPlan fp, PatchedConic burnPatch) BuildPassFlightPlan(
        Vehicle source, PatchedConic prePatch, SimTime burnTime, double3 dvVlf)
    {
        SimTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime);
        FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
        PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, dvVlf, burnTime);
        fp.Patches.Add(burnPatch);
        fp.ComputeCompleteTrajectory(FlightPlanPatchLimit, FlightPlanPolynomialOrder);
        if (source.Target != null)
            fp.CalculateTargetNodes(source.Target);
        return (fp, burnPatch);
    }

    private static SimTime NextOccurrenceAfter(Orbit orbit, TrueAnomaly ta, SimTime after)
        => orbit.TimeOfTrueAnomaly(ta, new SimTime(after.Seconds() + NextOccurrenceEpsilonSec));
}
