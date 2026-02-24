using System;
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
/// The game's RefineBurnTask sweeps dV +/-5% and checks each trajectory
/// for SOI intercepts via patched conic propagation. This fails for
/// hyperbolic targets because FindClosestApproaches uses the target's
/// Period (NaN for unbound orbits) as the time limit, causing it to
/// bail out immediately. Even with a finite time limit, the patched
/// conic trajectory diverges ~0.3 AU from the Lambert solution at
/// arrival because the impulsive approximation breaks down at extreme
/// dV (~26 km/s).
///
/// Instead, we build the flight plan with the Lambert-optimal dV
/// directly and sweep the trajectory at 1-day intervals to compute
/// an honest closest-approach distance so users know mid-course
/// corrections will be needed.
///
/// For bound targets, runs the original dV sweep + encounter detection.
/// </summary>
[HarmonyPatch(typeof(RefineBurnTask), nameof(RefineBurnTask.TryFindIntercept))]
static class Patch_TryFindIntercept
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

                if (Mod.DebugMode)
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
    /// Sweeps the heliocentric patch at 1-day intervals to find the
    /// minimum distance between the spacecraft and the target.
    /// </summary>
    private static double FindPatchedConicClosestApproach(
        FlightPlan flightPlan, IOrbiter target, double transitSeconds)
    {
        double minDist = double.MaxValue;
        Orbit targetOrbit = target.Orbit;

        foreach (var patch in flightPlan.Patches)
        {
            if (patch.PrimaryBody?.Hash != targetOrbit.Parent.Hash)
                continue;

            double start = patch.StartTime.Seconds();
            double end = Math.Min(start + transitSeconds * 2.0, patch.EndTime.Seconds());
            double step = 86400.0; // 1 day

            for (double t = start; t <= end; t += step)
            {
                var posShip = patch.Orbit.GetStateVectorsAt(new SimTime(t)).PositionCci;
                var posTarget = targetOrbit.GetStateVectorsAt(new SimTime(t)).PositionCci;
                double dist = (posTarget - posShip).Length();
                minDist = Math.Min(minDist, dist);
            }
        }

        return minDist;
    }
}
