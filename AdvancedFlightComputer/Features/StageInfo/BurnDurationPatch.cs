using System.Diagnostics;
using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.StageInfo;

/// <summary>
/// Overrides BurnDuration and IgnitionTime with multi-stage values after the
/// background job's FlightComputer copy is applied on the main thread.
///
/// The game's UpdateBurnTarget (worker thread) computes a single-stage
/// BurnDuration using only the currently active engines. This postfix
/// replaces it with the multi-stage burn time from our StageAnalyzer,
/// which accounts for all future stages.
///
/// This corrects the BURN TIME and START BURN IN gauge rollers, which
/// read from FlightComputer.Burn on the main thread during the render pass.
///
/// The worker thread's ignition decision uses its own freshly-computed
/// single-stage value (slightly off for multi-stage burns). This is
/// acceptable because auto-staging handles burn continuation.
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

        FlightComputer fc = __instance.FlightComputer;
        if (fc.Burn == null) return;

        float? corrected = StageInfoPanel.GetCorrectedBurnDuration();
        if (corrected == null || corrected.Value <= 0f) return;

        fc.Burn.BurnDuration = corrected.Value;
        fc.Burn.IgnitionTime = fc.Burn.ImpulsiveInstant - 0.5 * (double)fc.Burn.BurnDuration;

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("CorrectedBurnDuration.Postfix", Stopwatch.GetTimestamp() - perfStart);
#endif
    }
}
