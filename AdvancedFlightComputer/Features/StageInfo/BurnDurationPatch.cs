using System.Diagnostics;
using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.StageInfo;

/// <summary>
/// Thread-safe state transfer for the corrected multi-stage burn duration.
/// Written on the main thread by Patch_CorrectedBurnDuration, read on the
/// worker thread by Patch_WorkerIgnitionTiming.
///
/// Uses volatile fields for cross-thread visibility. Float and reference
/// reads/writes are atomic on x64. One-frame staleness is acceptable
/// because the correction value changes slowly (dV consumed per frame is
/// tiny relative to total burn dV).
/// </summary>
static class CorrectedBurnState
{
    /// <summary>
    /// The BurnTarget reference from the controlled vehicle, used for
    /// identification on the worker thread via reference equality.
    /// The FC copy constructor does Burn = existing.Burn (shared reference),
    /// so the worker thread's FC has the same BurnTarget object.
    /// </summary>
    public static volatile BurnTarget? TrackedBurn;

    /// <summary>
    /// The multi-stage burn duration in seconds, computed by
    /// StageAnalyzer.AnalyzeBurn. Only valid when TrackedBurn is non-null.
    /// </summary>
    public static volatile float CorrectedDuration;

    public static void Clear()
    {
        TrackedBurn = null;
        CorrectedDuration = 0f;
    }
}

/// <summary>
/// Drives per-frame stage analysis and corrects BurnDuration/IgnitionTime
/// with multi-stage values on the main thread.
///
/// Runs as a Postfix on Vehicle.UpdateFromTaskResults, which fires every
/// frame for every vehicle. Calls StageAnalysisCache.Update() to run the
/// analysis (conditionally - skipped when no burn and panel not visible),
/// then applies the corrected burn duration to fc.Burn and publishes
/// CorrectedBurnState for the worker thread.
/// </summary>
[HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
static class Patch_CorrectedBurnDuration
{
    static void Postfix(Vehicle __instance)
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        if (__instance != Program.ControlledVehicle) return;

        StageAnalysisCache.Update(__instance);

        FlightComputer fc = __instance.FlightComputer;
        if (fc.Burn == null)
        {
            CorrectedBurnState.Clear();
            return;
        }

        float? corrected = StageAnalysisCache.GetCorrectedBurnDuration();
        if (corrected == null || corrected.Value <= 0f)
        {
            CorrectedBurnState.Clear();
            return;
        }

        fc.Burn.BurnDuration = corrected.Value;
        fc.Burn.IgnitionTime = fc.Burn.ImpulsiveInstant - 0.5 * (double)fc.Burn.BurnDuration;

        CorrectedBurnState.CorrectedDuration = corrected.Value;
        CorrectedBurnState.TrackedBurn = fc.Burn;

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("CorrectedBurnDuration.Postfix", Stopwatch.GetTimestamp() - perfStart);
#endif
    }
}

/// <summary>
/// Postfix on FlightComputer.UpdateBurnTarget (worker thread) that overrides
/// single-stage BurnDuration and IgnitionTime with multi-stage values.
///
/// UpdateBurnTarget computes BurnDuration from single-stage Tsiolkovsky, then
/// derives IgnitionTime = ImpulsiveInstant - 0.5 * BurnDuration. This postfix
/// runs immediately after, replacing both with the multi-stage values from
/// CorrectedBurnState before ComputeControl uses IgnitionTime for the
/// ignition decision.
///
/// Only applies in Auto burn mode because Manual mode does not use IgnitionTime
/// for ignition decisions, and the game's throttle-adjusted BurnDuration should
/// be preserved for display.
///
/// Identifies the controlled vehicle's FC via BurnTarget reference equality:
/// the FC copy constructor does Burn = existing.Burn (shared reference), so
/// comparing against CorrectedBurnState.TrackedBurn correctly identifies
/// only the controlled vehicle's FlightComputer.
/// </summary>
static class Patch_WorkerIgnitionTiming
{
    public static void Postfix(FlightComputer __instance)
    {
        if (__instance.BurnMode != FlightComputerBurnMode.Auto) return;

        BurnTarget? burn = __instance.Burn;
        if (burn == null) return;

        BurnTarget? tracked = CorrectedBurnState.TrackedBurn;
        if (tracked == null || !ReferenceEquals(burn, tracked)) return;

        float duration = CorrectedBurnState.CorrectedDuration;
        if (duration <= 0f) return;

        burn.BurnDuration = duration;
        burn.IgnitionTime = burn.ImpulsiveInstant - 0.5 * (double)duration;
    }
}
