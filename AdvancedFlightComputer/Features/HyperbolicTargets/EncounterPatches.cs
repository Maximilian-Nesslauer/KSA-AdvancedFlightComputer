using System;
using System.Reflection;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Replaces <c>RefineBurnTask.TryFindIntercept</c> for unbound targets. Stock
/// sweeps the departure dV by five percent either way and probes each trajectory
/// for an SOI encounter, but its encounter search cannot resolve one against an
/// unbound body, see <see cref="Patch_FindClosestApproaches"/>, and at the dV a
/// comet intercept needs the impulsive patched conic trajectory diverges from the
/// Lambert solution by a large fraction of an AU anyway. So the flight plan is
/// built from the Lambert dV as is, and the closest approach is measured by
/// sampling the heliocentric patch, which tells the user how much correction to
/// expect on the way. Bound targets run the stock sweep.
///
/// Runs on the ThreadPool thread <c>RefineBurnTask</c> queues itself on.
/// </summary>
[HarmonyPatch(typeof(RefineBurnTask), nameof(RefineBurnTask.TryFindIntercept))]
internal static class Patch_TryFindIntercept
{
    static bool Prefix(
        OrbitalTransfers.TransferInfo transferInfo,
        ref OrbitalTransfers.PorkChopEntry selectedEntry,
        ref bool __result)
    {
        try
        {
            if (transferInfo.Target?.Orbit == null
                || transferInfo.Target.Orbit.IsBound())
                return true;

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

                if (DebugConfig.HyperbolicTargets)
                    DefaultCategory.Log.Debug(
                        $"[AFC] RefineBurnTask: hyperbolic target, " +
                        $"Lambert dV={dv.Length():F1} m/s, " +
                        $"patched conic miss={patchedConicDist / 1000:F0} km");
            }
            else
            {
                __result = false;
            }

            return false;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] TryFindIntercept prefix: {ex}");
            return true;
        }
    }

    /// <summary>Coarse pass at one day, then one minute around the coarse minimum
    /// so a fast flyby is not aliased between coarse samples.</summary>
    private static double FindPatchedConicClosestApproach(
        FlightPlan flightPlan, IOrbiter target, double transitSeconds)
    {
        double minDist = double.MaxValue;
        double minTime = double.NaN;
        Orbit targetOrbit = target.Orbit;

        const double CoarseStep = 86400.0;
        const double RefineStep = 60.0;
        const double RefineHalfWindow = CoarseStep;

        foreach (var patch in flightPlan.Patches)
        {
            if (patch.PrimaryBody?.Hash != targetOrbit.Parent.Hash)
                continue;

            // Offsets from the patch start rather than absolute sim seconds, because
            // a UniverseTime holds 128 bit nanoseconds and an absolute double would
            // spend its mantissa on the epoch instead of on the window.
            UniverseTime patchStart = patch.StartTime;
            double end = Math.Min(transitSeconds * 2.0, (patch.EndTime - patchStart).Seconds());

            for (double t = 0.0; t <= end; t += CoarseStep)
            {
                double dist = DistanceAt(patch, targetOrbit, patchStart + t);
                if (dist < minDist) { minDist = dist; minTime = t; }
            }

            if (double.IsNaN(minTime)) continue;

            double refineStart = Math.Max(0.0, minTime - RefineHalfWindow);
            double refineEnd = Math.Min(end, minTime + RefineHalfWindow);
            for (double t = refineStart; t <= refineEnd; t += RefineStep)
            {
                double dist = DistanceAt(patch, targetOrbit, patchStart + t);
                if (dist < minDist) minDist = dist;
            }
        }

        return minDist;
    }

    private static double DistanceAt(PatchedConic patch, Orbit targetOrbit, UniverseTime t)
    {
        var posShip = patch.Orbit.GetStateVectorsAt(t).PositionCci;
        var posTarget = targetOrbit.GetStateVectorsAt(t).PositionCci;
        return (posTarget - posShip).Length();
    }
}

/// <summary>
/// Skips stock's closest approach search in the one case where it throws, which
/// is a patch with a finite end time against a second body on an unbound orbit.
/// That branch sizes its time window as the smaller of the patch length and the
/// second body's Period, which is NaN there, and it builds a UniverseTime from
/// the window before its own finiteness check, which is where the NaN throws.
/// The end of time branch sizes the window from the asymptotic speeds instead
/// and never reads Period, so it runs as stock wrote it. The search is reached
/// from every flight plan's encounter scan once a comet has an SOI, which
/// HyperbolicBodies.xml gives it, and from the target node computation when a
/// comet is the vehicle's target. Finding nothing is what the finiteness check
/// would settle on, and the transfer's own closest approach comes from
/// <see cref="Patch_TryFindIntercept"/>.
/// </summary>
[HarmonyPatch]
internal static class Patch_FindClosestApproaches
{
    private static readonly Type[] Signature =
    {
        typeof(Span<Encounter>),
        typeof(int).MakeByRefType(),
        typeof(IOrbiter),
        typeof(UniverseTime).MakeByRefType(),
    };

    private static MethodInfo? Anchor =>
        AccessTools.Method(typeof(PatchedConic), "FindClosestApproaches", Signature);

    /// <summary>Whether the private stock method still exists in this build, so
    /// the feature can report the gap instead of failing to patch.</summary>
    public static bool IsAnchorPresent => Anchor != null;

    static MethodBase TargetMethod() =>
        Anchor ?? throw new InvalidOperationException(
            "[AFC] PatchedConic.FindClosestApproaches not found; "
            + "patching this class requires an IsAnchorPresent check first.");

    static bool Prefix(PatchedConic __instance, IOrbiter secondBody)
    {
        try
        {
            // Mirrors the branch selection in the original. The window comes from
            // the second body's Period only when neither eccentricity is below 1
            // and the patch has a finite end time.
            Orbit? target = secondBody?.Orbit;
            if (target == null || target.IsBound()) return true;
            if (__instance.EndTime.IsEndOfTime()) return true;
            if (__instance.Orbit.Eccentricity < 1.0 && target.Eccentricity < 1.0) return true;
            return false;
        }
        catch (Exception ex)
        {
            // Fails open, because the original reads the same orbit right away and
            // reports its own failure. Deduped because the scan runs per patch per
            // frame.
            LogHelper.WarnOnce("find-closest-approaches:" + ex.GetType().Name,
                $"[AFC] FindClosestApproaches prefix: {ex}; running stock search.");
            return true;
        }
    }
}
