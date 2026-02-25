using System;
using System.Diagnostics;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Workaround for https://forums.ahwoo.com/threads/781/
/// Remove once fixed upstream.
///
/// GenerateSpacedPoints creates orbit line points between a start and end
/// true anomaly. For bound orbits, -Pi..+Pi covers the full ellipse. For
/// hyperbolic orbits, Pi exceeds the asymptote (acos(-1/e)), producing
/// non-finite positions that the renderer draws as lines through the origin.
/// We clamp to the asymptote instead.
/// </summary>
[HarmonyPatch(typeof(UpdateTaskUtils), "GenerateSpacedPoints",
    [typeof(PatchedConic), typeof(Orbit)])]
static class Patch_ClipPointGeneration
{
    static bool Prefix(PatchedConic? patch, Orbit o,
        ref MemoryOwner<OrbitPointCce> __result)
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        if (o.IsBound()) return true;

        // Only intervene when the original would fall back to -Pi..+Pi.
        // Escape burns and encounter patches have valid bounds, leave them.
        bool hasValidBounds = patch != null
            && patch.EndTrueAnomaly.IsNotNaN()
            && patch.EndTransition != PatchTransition.Final;
        if (hasValidBounds) return true;

        try
        {
            var orb2ParentCce = o.GetOrb2ParentCce();
            var points = MemoryOwner<OrbitPointCce>.Allocate(HyperbolicTargets.OrbitPointCount);

            double asymptote = Math.Acos(-1.0 / o.Eccentricity) * HyperbolicTargets.AsymptoteMargin;
            double startTa = -asymptote;
            double endTa = asymptote;

            if (patch != null
                && patch.StartTrueAnomaly.IsNotNegativePi()
                && patch.StartTrueAnomaly.IsNotZero())
            {
                startTa = patch.StartTrueAnomaly.Value();
            }

            double step = (endTa - startTa) / (HyperbolicTargets.OrbitPointCount - 1);
            for (int i = 0; i < HyperbolicTargets.OrbitPointCount; i++)
            {
                var ta = new TrueAnomaly(startTa + i * step);
                EccentricAnomaly ea = o.GetEccentricAnomaly(ta);
                if (double.IsFinite(ea.Value()))
                {
                    SimTime timeFromPe = o.GetTimeFromPeTo(ea);
                    SimTime remaining = o.GetRemainingTimeTo(ea);
                    double3 posCce = o.GetPositionOrb(ea).Transform(orb2ParentCce);
                    points.Span[i] = new OrbitPointCce(posCce, timeFromPe, remaining, ta);
                }
                else
                {
                    // Repeat last valid point to avoid default-zero gaps
                    if (i > 0) points.Span[i] = points.Span[i - 1];
                }
            }

            __result = points;
#if DEBUG
            if (DebugConfig.Performance)
                PerfTracker.Record("ClipPointGeneration.Prefix", Stopwatch.GetTimestamp() - perfStart);
#endif
            return false;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] ClipPointGeneration prefix: {ex.Message}");
            return true;
        }
    }
}
