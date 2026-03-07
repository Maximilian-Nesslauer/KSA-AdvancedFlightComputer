using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using static AdvancedFlightComputer.Features.ManeuverTools.ManeuverTools;
using AdvancedFlightComputer.Features.StageInfo;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

/// <summary>
/// Bridges the ManeuverTools plan types with the OberthMultiPass preview and
/// creation pipeline. Holds GoalFunc construction and multi-pass preview/create
/// logic that is specific to AFC plan types but belongs conceptually in
/// OberthMultiPass rather than the window-level DrawPlanWindowPatch.
/// </summary>
static class OberthManeuverIntegration
{
    private static int _previewInputHash;

    internal static void Reset() => _previewInputHash = 0;

    // Called after RemoveAllBurns so the next frame recomputes the preview with
    // fresh state. Without this, the same input hash causes the preview to be
    // skipped, leaving a stale preview after BackToSingle.
    internal static void InvalidatePreviewHash() => _previewInputHash = 0;

    #region Window UI

    /// <summary>
    /// Draws the Oberth advisory, pass count selector, and pass list.
    /// Called each frame by DrawPlanWindowPatch while a maneuver result exists.
    /// </summary>
    internal static void DrawOberthSection(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        ImGui.Separator();
        OberthUI.DrawAdvisory(source, maneuver.DvCci.Length());
        OberthUI.DrawPassCountSelector();

        // Validate active split state once per frame at the section level, not inside the UI
        // component, so any Reset() happens before DrawPassList reads the state.
        if (MultiPassState.HasActiveSplit)
            MultiPassState.ValidateState();

        if (OberthUI.CurrentPassCount > 1 && !MultiPassState.HasActiveSplit)
        {
            TryRecomputePreview(source, maneuver, typeKey);

            if (MultiPassState.PreviewFailed && !MultiPassState.HasPreview)
            {
                ImGui.Spacing();
                ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 150, 50, 255));
                ImGui.TextWrapped("Multi-pass preview not possible: orbit escapes SOI or becomes unbound before all passes can complete. Try fewer passes."u8);
                ImGui.PopStyleColor();
            }
        }

        OberthUI.DrawPassList();
    }

    /// <summary>
    /// Recomputes the multi-pass preview when inputs change.
    /// Called each frame while passCount > 1 and no active split exists.
    /// </summary>
    private static void TryRecomputePreview(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        int targetOrbitHash = ManeuverToolsWindow.GetSelectedTargetOrbit()?.GetHashCode() ?? 0;
        int hash = HashCode.Combine(
            typeKey,
            OberthUI.CurrentPassCount,
            (int)OberthUI.CurrentSplitMode,
            ManeuverToolsWindow.TargetAltitude.GetHashCode(),
            ManeuverToolsWindow.TargetInclinationRad.GetHashCode(),
            ManeuverToolsWindow.UseDescendingNode,
            targetOrbitHash,
            source.Id);

        if (hash == _previewInputHash)
            return;

        _previewInputHash = hash;

        GoalFuncSet gfs = BuildGoalFuncs(typeKey, source, maneuver);

        VehicleBurnAnalysis analysis = StageAnalysisCache.Analysis
            ?? StageAnalysisCache.Empty;
        List<PassAllocation> allocations = BurnTimeSplitter.ComputeAllocations(
            maneuver.DvCci.Length(), OberthUI.CurrentPassCount, analysis,
            OberthUI.CurrentSplitMode);

        if (gfs.DvDirection != default(double3))
            MultiPassPlanner.ComputeApseBurnPreview(source, allocations, gfs.BurnTa, gfs.DvDirection);
        else if (gfs.PreviewGoal != null)
            MultiPassPlanner.ComputeGoalPreview(
                source, allocations, gfs.PreviewGoal, Universe.GetElapsedSimTime(), gfs.TotalAngleFunc);
    }

    #endregion

    #region Burn creation

    /// <summary>
    /// Computes a preview and creates multi-pass burns
    /// from the preview. Called when the user clicks Create with passCount > 1.
    /// </summary>
    internal static void CreateMultiPassBurns(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        GoalFuncSet gfs = BuildGoalFuncs(typeKey, source, maneuver);

        VehicleBurnAnalysis analysis = StageAnalysisCache.Analysis
            ?? StageAnalysisCache.Empty;
        List<PassAllocation> allocations = BurnTimeSplitter.ComputeAllocations(
            maneuver.DvCci.Length(), OberthUI.CurrentPassCount, analysis,
            OberthUI.CurrentSplitMode);

        if (!MultiPassState.HasPreview)
        {
            bool ok = gfs.DvDirection != default(double3)
                ? MultiPassPlanner.ComputeApseBurnPreview(source, allocations, gfs.BurnTa, gfs.DvDirection)
                : (gfs.PreviewGoal != null
                    ? MultiPassPlanner.ComputeGoalPreview(
                        source, allocations, gfs.PreviewGoal,
                        Universe.GetElapsedSimTime(), gfs.TotalAngleFunc)
                    : false);

            if (!ok)
            {
                if (DebugConfig.ManeuverTools)
                    DefaultCategory.Log.Debug(
                        "[AFC] OberthManeuverIntegration: multi-pass preview failed.");
                return;
            }
        }

        MultiPassPlanner.CreateBurns(
            source, maneuver.DvVlf, maneuver.BurnTime, gfs.CorrectionGoal,
            MultiPassState.ExtractPreviewDvCapacities(), gfs.BurnTa);
    }

    #endregion

    #region GoalFunc construction

    /// <summary>
    /// Constructs the GoalFunc delegates, burn geometry, and remaining-angle
    /// function for a given plan type. See GoalFuncSet for field descriptions.
    /// </summary>
    private static GoalFuncSet BuildGoalFuncs(
        string key, Vehicle source, OrbitManeuvers.ManeuverResult maneuver)
    {
        if (key == KeySetApoapsis)
        {
            double alt = ManeuverToolsWindow.TargetAltitude;
            double pr = source.Parent?.MeanRadius ?? 0.0;
            return new GoalFuncSet(
                PreviewGoal:    null,
                CorrectionGoal: (o, t, _) => OrbitManeuvers.ComputeSetApoapsis(o, alt, pr, t),
                DvDirection:    maneuver.DvVlf.NormalizeOrZero(),
                BurnTa:         TrueAnomaly.Zero,
                TotalAngleFunc: null);
        }

        if (key == KeySetPeriapsis)
        {
            double alt = ManeuverToolsWindow.TargetAltitude;
            double pr = source.Parent?.MeanRadius ?? 0.0;
            return new GoalFuncSet(
                PreviewGoal:    null,
                CorrectionGoal: (o, t, _) => OrbitManeuvers.ComputeSetPeriapsis(o, alt, pr, t),
                DvDirection:    maneuver.DvVlf.NormalizeOrZero(),
                BurnTa:         new TrueAnomaly(Math.PI),
                TotalAngleFunc: null);
        }

        if (key == KeyMatchInclination)
        {
            Orbit? targetOrbit = ManeuverToolsWindow.GetSelectedTargetOrbit();
            if (targetOrbit == null)
                return new GoalFuncSet(null, null, default, TrueAnomaly.NaN, null);
            bool useDesc = ManeuverToolsWindow.UseDescendingNode;
            GoalFunc g = (o, t, f) =>
                OrbitManeuvers.ComputeMatchInclination(o, targetOrbit, useDesc, t, f);
            Func<Orbit, double> angleFunc = o => o.GetRelativeInclination(targetOrbit).Value();
            return new GoalFuncSet(g, g, default, TrueAnomaly.NaN, angleFunc);
        }

        if (key == KeySetInclination)
        {
            double targetIncRad = ManeuverToolsWindow.TargetInclinationRad;
            bool useDesc = ManeuverToolsWindow.UseDescendingNode;
            GoalFunc g = (o, t, f) =>
            {
                double intermediate = o.Inclination + (targetIncRad - o.Inclination) * f;
                return OrbitManeuvers.ComputeSetInclination(o, intermediate, useDesc, t);
            };
            Func<Orbit, double> angleFunc = o => Math.Abs(targetIncRad - o.Inclination);
            return new GoalFuncSet(g, g, default, TrueAnomaly.NaN, angleFunc);
        }

        return new GoalFuncSet(null, null, default, TrueAnomaly.NaN, null);
    }

    #endregion
}
