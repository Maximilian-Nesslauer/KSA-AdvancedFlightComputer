using System;
using System.Threading;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// For hyperbolic targets, replaces TryFindIntercept entirely: builds
/// the flight plan from the Lambert-optimal dV and computes the actual
/// patched-conic closest approach distance for honest reporting.
///
/// The game's RefineBurnTask sweeps dV +/-5% and probes each trajectory
/// for SOI intercepts via OrbitalTransfers.InterceptsBody, which calls
/// PatchedConic.TryFindClosestEncounter. Encounters are populated by
/// FindClosestApproaches (PatchedConic.cs:397), and that function gates
/// on `Math.Min(..., secondBody.Orbit.Period)`. Period is NaN for
/// hyperbolic targets, the gate is `!num2.IsFinite() return;`, so no
/// encounter is ever found. Even with a finite gate, the patched-conic
/// trajectory diverges ~0.3 AU from the Lambert solution at arrival
/// because the impulsive approximation breaks down at extreme dV
/// (~26 km/s).
///
/// Instead we build the flight plan with the Lambert-optimal dV directly
/// and sweep the trajectory in two passes (coarse then refined) for an
/// honest closest-approach distance, so users see how much mid-course
/// correction will be needed.
///
/// For bound targets, runs the original dV sweep + encounter detection.
/// </summary>
[HarmonyPatch(typeof(RefineBurnTask), nameof(RefineBurnTask.TryFindIntercept))]
internal static class Patch_TryFindIntercept
{
    static bool Prefix(
        OrbitalTransfers.TransferInfo transferInfo,
        ref OrbitalTransfers.PorkChopEntry selectedEntry,
        ref bool __result)
    {
        if (transferInfo.Target.Orbit.Eccentricity < 1.0)
            return true;

        try
        {
            double3 dv = selectedEntry.TransferData.TransferDvVlf;
            FlightPlan flightPlan = FlightPlan.CreateUninitialized(transferInfo.Vehicle.Hash);

            if (OrbitalTransfers.BuildFlightPlan(ref flightPlan, transferInfo,
                    selectedEntry.TransferData.Start, dv,
                    out var closestPoint, out var _))
            {
                selectedEntry.FlightPlan = flightPlan;
                selectedEntry.TransferData.Point = closestPoint;

                double patchedConicDist = FindPatchedConicClosestApproach(
                    flightPlan, transferInfo.Target,
                    selectedEntry.TransferData.Transit.Seconds());
                selectedEntry.TransferData.ClosestApproachDistance = patchedConicDist;
                __result = true;

                // Volatile read: TryFindIntercept runs on the ThreadPool
                // (RefineBurnTask.Run -> ThreadPool.QueueUserWorkItem).
                if (Volatile.Read(ref DebugConfig.HyperbolicTargets))
                {
                    DefaultCategory.Log.Debug(
                        $"[AFC] RefineBurnTask: hyperbolic target, " +
                        $"Lambert dV={dv.Length():F1} m/s, " +
                        $"patched conic miss={patchedConicDist / 1000:F0} km");
                }
            }
            else
            {
                __result = false;
            }

            return false;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] TryFindIntercept prefix: {ex.Message}");
            return true;
        }
    }

    /// <summary>
    /// Sweeps the heliocentric patch in a coarse pass (1-day step) to find
    /// a rough minimum, then refines around it (1-minute step) so fast
    /// flybys aren't aliased between coarse samples.
    /// </summary>
    private static double FindPatchedConicClosestApproach(
        FlightPlan flightPlan, IOrbiter target, double transitSeconds)
    {
        double minDist = double.MaxValue;
        double minTime = double.NaN;
        Orbit targetOrbit = target.Orbit;

        const double CoarseStep = 86400.0;   // 1 day
        const double RefineStep = 60.0;      // 1 minute
        const double RefineHalfWindow = CoarseStep;

        foreach (var patch in flightPlan.Patches)
        {
            if (patch.PrimaryBody?.Hash != targetOrbit.Parent.Hash)
                continue;

            double start = patch.StartTime.Seconds();
            double end = Math.Min(start + transitSeconds * 2.0, patch.EndTime.Seconds());

            for (double t = start; t <= end; t += CoarseStep)
            {
                double dist = DistanceAt(patch, targetOrbit, t);
                if (dist < minDist) { minDist = dist; minTime = t; }
            }

            if (double.IsNaN(minTime)) continue;

            double refineStart = Math.Max(start, minTime - RefineHalfWindow);
            double refineEnd = Math.Min(end, minTime + RefineHalfWindow);
            for (double t = refineStart; t <= refineEnd; t += RefineStep)
            {
                double dist = DistanceAt(patch, targetOrbit, t);
                if (dist < minDist) minDist = dist;
            }
        }

        return minDist;
    }

    private static double DistanceAt(PatchedConic patch, Orbit targetOrbit, double t)
    {
        var posShip = patch.Orbit.GetStateVectorsAt(new SimTime(t)).PositionCci;
        var posTarget = targetOrbit.GetStateVectorsAt(new SimTime(t)).PositionCci;
        return (posTarget - posShip).Length();
    }
}
