using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.StageInfo;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

/// <summary>
/// ImGui components for the multi-pass Oberth UI section. Drawn inline
/// within the Transfer Planning window by DrawPlanWindowPatch.
/// </summary>
static class OberthUI
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static int _userPassCount = 1;
    private static bool _passCountInitialized;
    private static OberthAssessmentResult _lastAssessment;
    private static SplitMode _splitMode = SplitMode.BurnTime;

    public static int CurrentPassCount => _userPassCount;
    public static SplitMode CurrentSplitMode => _splitMode;

    public static void DrawAdvisory(Vehicle source, double dvMagnitude)
    {
        if (source.Orbit == null)
            return;

        VehicleBurnAnalysis analysis = StageAnalysisCache.Analysis
            ?? StageAnalysisCache.Empty;

        bool debugThreshold = DebugConfig.OberthMultiPass;
        _lastAssessment = OberthAssessment.Assess(source.Orbit, dvMagnitude, analysis, debugThreshold);

        if (!_lastAssessment.ExceedsThreshold)
            return;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 200, 60, 255));

        double burnRatioPct = _lastAssessment.BurnRatio * 100.0;
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Burn duration ({0:F0}s) is {1:F0}%% of orbital period. " +
            "Splitting across {2} passes saves ~{3:F0} m/s.",
            _lastAssessment.BurnDuration,
            burnRatioPct,
            _lastAssessment.SuggestedPasses,
            _lastAssessment.EstimatedSavings));

        ImGui.PopStyleColor();
    }

    /// <summary>
    /// Draws the [less-than] N [greater-than] pass count selector and split mode
    /// toggle. Always visible when no active split exists. Auto-initializes to
    /// the suggested pass count. Clears the preview when anything changes.
    /// </summary>
    public static void DrawPassCountSelector()
    {
        if (MultiPassState.HasActiveSplit)
            return;

        if (!_passCountInitialized)
        {
            _userPassCount = Math.Max(1, _lastAssessment.SuggestedPasses);
            _passCountInitialized = true;
        }

        ImGui.Spacing();
        ImGui.Text("Passes:");
        ImGui.SameLine();

        if (ImGuiHelper.DrawButton("<"u8, KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (_userPassCount > 1)
            {
                _userPassCount--;
                MultiPassState.ClearPreview();
            }
        }

        ImGui.SameLine();
        ImGui.Text(_userPassCount.ToString());
        ImGui.SameLine();

        if (ImGuiHelper.DrawButton(">"u8, KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (_userPassCount < OberthAssessment.MaxPasses)
            {
                _userPassCount++;
                MultiPassState.ClearPreview();
            }
        }

        if (_userPassCount < 2)
            return;

        ImGui.Spacing();
        bool isBurnTime = _splitMode == SplitMode.BurnTime;
        if (ImGui.RadioButton("Equal Burn Time"u8, isBurnTime) && !isBurnTime)
        {
            _splitMode = SplitMode.BurnTime;
            MultiPassState.ClearPreview();
        }
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(
                "Each pass fires engines for equal duration, accounting for\nstaging. Front-loads dV toward periapsis (Oberth-optimal)."u8);

        ImGui.SameLine();
        bool isEqualDv = _splitMode == SplitMode.EqualDv;
        if (ImGui.RadioButton("Equal Delta-V"u8, isEqualDv) && !isEqualDv)
        {
            _splitMode = SplitMode.EqualDv;
            MultiPassState.ClearPreview();
        }
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip("Each pass receives the same delta-v.\nSimpler but less Oberth-optimal."u8);
    }

    public static void DrawPassList()
    {
        // Preview phase (before Create is clicked)
        if (MultiPassState.HasPreview && !MultiPassState.HasActiveSplit
                && MultiPassState.PreviewPasses is { Count: > 1 })
        {
            ImGui.Spacing();
            SimTime previewNow = Universe.GetElapsedSimTime();
            for (int i = 0; i < MultiPassState.PreviewPasses.Count; i++)
            {
                PassResult pass = MultiPassState.PreviewPasses[i];
                double dvMag = pass.DvVlf.Length();
                double timeToNode = pass.BurnTime.Seconds() - previewNow.Seconds();
                DrawPassRow(i, dvMag, timeToNode, pass.EstimatedBurnTime, interactive: false);
            }
            return;
        }

        // Active split phase (burns exist in BurnPlan)
        if (!MultiPassState.HasActiveSplit || MultiPassState.PassBurns == null)
            return;

        ImGui.Spacing();
        SimTime now = Universe.GetElapsedSimTime();
        VehicleBurnAnalysis analysis = StageAnalysisCache.Analysis
            ?? StageAnalysisCache.Empty;

        for (int i = 0; i < MultiPassState.PassBurns.Count; i++)
        {
            Burn burn = MultiPassState.PassBurns[i];
            double dvMag = burn.DeltaVVlf.Length();
            double timeToNode = burn.Time.Seconds() - now.Seconds();
            double burnDuration = (MultiPassState.PlannedBurnTimes != null && i < MultiPassState.PlannedBurnTimes.Length)
                ? MultiPassState.PlannedBurnTimes[i]
                : StageAnalyzer.AnalyzeBurn(analysis, (float)dvMag).TotalBurnTime;
            DrawPassRow(i, dvMag, timeToNode, burnDuration, interactive: true);
        }

        ImGui.Spacing();
        ImGui.Separator();

        bool canGoBack = !MultiPassState.HasExecutedPasses;
        if (!canGoBack)
            ImGui.BeginDisabled();

        if (ImGuiHelper.DrawButton("Back to Single Burn"u8,
                KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (MultiPassState.Vehicle != null)
                MultiPassPlanner.RemoveAllBurns(MultiPassState.Vehicle);
            // Reset pass count so the next frame shows the single-burn UI
            // (preview orbit/flight plan checkboxes) rather than immediately
            // recomputing a multi-pass preview with the old count.
            _userPassCount = 1;
        }

        if (!canGoBack)
        {
            ImGui.EndDisabled();
            if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
                ImGui.SetTooltip("Not available after a pass has been executed."u8);
        }
    }

    /// <summary>
    /// Resets initialization so the
    /// pass count is re-suggested on the next draw.
    /// </summary>
    public static void OnTypeChanged()
    {
        _passCountInitialized = false;
        MultiPassState.ClearPreview();
    }

    public static void Reset()
    {
        _userPassCount = 1;
        _passCountInitialized = false;
        _lastAssessment = default;
        _splitMode = SplitMode.BurnTime;
    }

    #region Pass Row

    private static void DrawPassRow(int i, double dvMag, double timeToNode,
        double burnDuration, bool interactive)
    {
        ImGui.PushID(i);

        bool selected = (i == MultiPassState.SelectedPassIndex);
        if (!interactive)
            ImGui.BeginDisabled();
        if (ImGui.RadioButton("##p"u8, selected))
        {
            MultiPassState.SelectedPassIndex = i;
            // Activate gizmo and set as active burn so the stock burn window opens.
            if (MultiPassState.PassBurns != null && i < MultiPassState.PassBurns.Count)
            {
                for (int j = 0; j < MultiPassState.PassBurns.Count; j++)
                    MultiPassState.PassBurns[j].IsGizmoActive = (j == i);
                Program.ActiveBurn = MultiPassState.PassBurns[i];
            }
        }
        if (!interactive)
            ImGui.EndDisabled();
        ImGui.SameLine();

        // "Pass N" label: white when selected, grey otherwise.
        if (!selected)
            ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        ImGui.Text(string.Format(Inv, "Pass {0}", i + 1));
        if (!selected)
            ImGui.PopStyleColor();

        // Secondary info in TextDisabled (grey)
        ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        string timeStr = timeToNode > 0
            ? string.Format(Inv, "in {0}", ManeuverToolsWindow.FormatTimeSpan(timeToNode))
            : "past";
        string detail = burnDuration > 0.5
            ? string.Format(Inv, "  Burn: {0:F0}s   {1}   Delta V: {2:F1} m/s",
                burnDuration, timeStr, dvMag)
            : string.Format(Inv, "  {0}   Delta V: {1:F1} m/s",
                timeStr, dvMag);
        // Safe: format values (numbers, time strings) never contain '%'.
        ImGui.TextWrapped(detail);
        ImGui.PopStyleColor();

        ImGui.PopID();
    }

    #endregion
}
