using System;
using System.Globalization;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using static AdvancedFlightComputer.Features.ManeuverTools.ManeuverTools;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Inline section in the Transfer Planning window when an apse-burn
/// type is selected: pass-count stepper, split-mode radio, finite-
/// burn-loss advisory. Cache lives in <see cref="MultiPassPreviewCache"/>.
/// </summary>
internal static class MultiPassUI
{
    private const int MinPasses = 1;

    /// <summary>UI renders only when true; lets a future safety gate
    /// (e.g. missing reflection target) hide it cleanly.</summary>
    public static bool Enabled { get; set; }

    // Burn-time / period above which we advise splitting.
    private const double SuggestThreshold = 0.15;

    private const double SuggestPerPassLossCeiling = 0.005;   // 0.5% per-pass fractional finite-burn loss
    private const double SuggestMarginalSavingCeiling = 0.001; // 0.1% of total dV gained per added pass

    private const int SuggestMinN = 2;
    private const int SuggestMaxN = 8;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static int _passCount = 1;
    private static SplitMode _splitMode = SplitMode.EqualBurnTime;

    private static string? _lastTypeKey;
    private static string? _lastSourceId;

    public static int PassCount => _passCount;
    public static SplitMode CurrentSplitMode => _splitMode;

    public static bool HasMultiPassPreview =>
        Enabled && _passCount > 1 && MultiPassPreviewCache.HasPreview;

    /// <summary>Types that the multi-pass pipeline can plan and execute:
    /// the four AFC-injected quick-tools plus the two stock circularize
    /// entries AFC claims via Patch_DrawPlanWindow.</summary>
    public static bool IsMultiPassSupportedType(string typeKey) =>
        typeKey == KeySetApoapsis
        || typeKey == KeySetPeriapsis
        || typeKey == KeyMatchInclination
        || typeKey == KeySetInclination
        || typeKey == KeyStockCircularizeApoapsis
        || typeKey == KeyStockCircularizePeriapsis;

    /// <summary>Multi-pass selected and preview is usable.</summary>
    public static bool IsArmed(string typeKey) =>
        Enabled
        && _passCount > 1
        && IsMultiPassSupportedType(typeKey)
        && MultiPassPreviewCache.HasPreview;

    /// <summary>Multi-pass selected but planner could not produce a
    /// preview; Create should be disabled rather than fall back.</summary>
    public static bool WantsMultiPassButCannot(string typeKey) =>
        Enabled
        && _passCount > 1
        && IsMultiPassSupportedType(typeKey)
        && !MultiPassPreviewCache.HasPreview;

    public static void Draw(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        if (!Enabled || source == null || !IsMultiPassSupportedType(typeKey))
            return;

        // Reset on plan-type / source change so the cache does not
        // briefly render against the wrong vehicle.
        if (_lastTypeKey != typeKey || _lastSourceId != source.Id)
        {
            _lastTypeKey = typeKey;
            _lastSourceId = source.Id;
            _passCount = 1;
            _splitMode = SplitMode.EqualBurnTime;
            MultiPassPreviewCache.ClearPreview();
        }

        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);

        // Active execution: pass count and split mode are locked at Start
        // time, and only the still-pending passes are meaningful to split
        // the remaining dV across.
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
        {
            DrawActive(source, typeKey, state, exec);
            return;
        }

        double totalDv = maneuver.DvCci.Length();
        double totalBurnTime = EstimateBurnTime(totalDv, state);

        ImGui.Spacing();
        DrawPassCountSelector();

        if (_passCount > 1)
            DrawSplitModeRadio();

        DrawAdvisoryIfApplicable(source.Orbit, totalDv, totalBurnTime);

        if (_passCount > 1)
        {
            MultiPassPreviewCache.UpdatePreviewIfStale(
                source, maneuver, typeKey, _passCount, _splitMode, state, totalDv);
            DrawPreviewFailureIfApplicable();
            DrawInsufficientFuelIfApplicable(totalDv, state);
            DrawPassList(firstPassDisplayNumber: 1);
            DrawSavingsLine(totalDv, totalBurnTime, source.Orbit?.Period ?? 0.0,
                MultiPassPreviewCache.PreviewPasses);
        }
        else
            MultiPassPreviewCache.ClearPreview();
    }

    private static void DrawActive(
        Vehicle source, string typeKey,
        SequenceBurnState state, MultiPassExecution exec)
    {
        int remaining = exec.PassCountTotal - exec.PassIndex;
        if (remaining <= 0) return;

        // Re-derive the maneuver from the locked intent (not from the
        // user-editable ManeuverToolsWindow.TargetAltitude): the displayed
        // pass list must match what the execution is actually targeting.
        OrbitManeuvers.ManeuverResult? lockedManeuver = exec.Intent.ComputeManeuver(source);
        if (lockedManeuver == null) return;

        double totalDv = lockedManeuver.Value.DvCci.Length();
        MultiPassPreviewCache.UpdatePreviewIfStale(
            source, lockedManeuver.Value, typeKey, remaining, exec.Mode, state, totalDv);
        DrawPreviewFailureIfApplicable();
        DrawInsufficientFuelIfApplicable(totalDv, state);
        DrawPassList(firstPassDisplayNumber: exec.PassIndex + 1);
        DrawSavingsLine(totalDv, EstimateBurnTime(totalDv, state),
            source.Orbit?.Period ?? 0.0, MultiPassPreviewCache.PreviewPasses);
    }

    public static void Render(Viewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source == null || source.Id != MultiPassPreviewCache.PreviewSourceId) return;

        // Active execution: stock renders the queued burn's orbit, so we
        // skip passes[0] and draw only the future-passes overlay.
        bool skipFirst = MultiPassRegistry.Has(source.Id);
        MultiPassRenderer.RenderPassOrbits(
            viewport, source, MultiPassPreviewCache.PreviewPasses, skipFirst);
    }

    /// <summary>Per-pass Ap/Pe/AN/DN/SOI/closest markers with first /
    /// final / intermediate styling. ImGui-phase counterpart of
    /// <see cref="Render"/>.</summary>
    public static void RenderMarkers(Viewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source == null || source.Id != MultiPassPreviewCache.PreviewSourceId) return;

        int firstPassDisplayNumber = 1;
        bool skipFirst = false;
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
        {
            firstPassDisplayNumber = exec.PassIndex + 1;
            skipFirst = true;
        }
        MultiPassMarkers.Draw(viewport, source,
            MultiPassPreviewCache.PreviewPasses,
            firstPassDisplayNumber, skipFirst);
    }

    /// <summary>Final-pass FlightPlan; what "Preview Flight Plan"
    /// shows in multi-pass mode.</summary>
    public static FlightPlan? LastPassFlightPlan
    {
        get
        {
            PassPreview[] passes = MultiPassPreviewCache.PreviewPasses;
            return passes.Length > 0 ? passes[passes.Length - 1].FlightPlan : null;
        }
    }

    public static void Reset()
    {
        _passCount = 1;
        _splitMode = SplitMode.EqualBurnTime;
        _lastTypeKey = null;
        _lastSourceId = null;
    }

    #region UI components

    private static void DrawPassCountSelector()
    {
        ImGui.Text("Passes:"u8);
        ImGui.SameLine();

        if (ImGuiHelper.DrawButton("<"u8, KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (_passCount > MinPasses)
            {
                _passCount--;
                MultiPassPreviewCache.Invalidate();
            }
        }
        ImGui.SameLine();
        ImGui.Text(_passCount.ToString(Inv));
        ImGui.SameLine();
        if (ImGuiHelper.DrawButton(">"u8, KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (_passCount < Splitter.MaxPasses)
            {
                _passCount++;
                MultiPassPreviewCache.Invalidate();
            }
        }
    }

    private static void DrawPassList(int firstPassDisplayNumber)
    {
        if (!MultiPassPreviewCache.HasPreview) return;

        PassPreview[] passes = MultiPassPreviewCache.PreviewPasses;
        if (passes.Length == 0) return;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        for (int i = 0; i < passes.Length; i++)
        {
            double dv = passes[i].DvVlf.Length();
            double t = passes[i].EstimatedBurnTimeSec;
            int n = firstPassDisplayNumber + i;
            string line = t > 0.5
                ? string.Format(Inv, "Pass {0}: {1:F0} m/s, {2:F0}s", n, dv, t)
                : string.Format(Inv, "Pass {0}: {1:F0} m/s", n, dv);
            ImGui.Text(line);
        }
        ImGui.PopStyleColor();
    }

    private static void DrawSplitModeRadio()
    {
        ImGui.Spacing();
        bool isBurnTime = _splitMode == SplitMode.EqualBurnTime;
        if (ImGui.RadioButton("Equal Burn Time"u8, isBurnTime) && !isBurnTime)
        {
            _splitMode = SplitMode.EqualBurnTime;
            MultiPassPreviewCache.Invalidate();
        }
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(
                "Each pass fires the engines for the same duration.\nEqualizes finite-burn arc length across passes\n(Oberth-optimal default)."u8);

        ImGui.SameLine();
        bool isEqualDv = _splitMode == SplitMode.EqualDv;
        if (ImGui.RadioButton("Equal Delta-V"u8, isEqualDv) && !isEqualDv)
        {
            _splitMode = SplitMode.EqualDv;
            MultiPassPreviewCache.Invalidate();
        }
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(
                "Each pass delivers the same delta-v magnitude"u8);
    }

    private static void DrawPreviewFailureIfApplicable()
    {
        if (!MultiPassPreviewCache.LastPreviewFailed) return;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 150, 50, 255));
        string reason = MultiPassPreviewCache.LastPreviewFailureReason ?? "unknown reason";
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Multi-pass preview incomplete: {0}.\n" +
            "Try fewer passes or a different split mode.",
            reason));
        ImGui.PopStyleColor();
    }

    // Warns when allocation sum < requested dV (vehicle is fuel-short).
    // Reads the cached sum so we do not re-run Splitter per frame.
    private static void DrawInsufficientFuelIfApplicable(
        double totalDv, SequenceBurnState state)
    {
        if (totalDv <= 0.0 || !state.HasUsableEngines) return;

        double sum = MultiPassPreviewCache.CachedAllocationsSum;
        if (double.IsNaN(sum)) return;

        // 0.5% tolerance to absorb floating-point drift from the
        // multi-stage Tsiolkovsky walk.
        if (sum >= totalDv * 0.995) return;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 150, 50, 255));
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Vehicle can only deliver ~{0:F0} m/s of the {1:F0} m/s required.\n" +
            "Multi-pass will run out of fuel before the goal is reached.",
            sum, totalDv));
        ImGui.PopStyleColor();
    }

    private static void DrawAdvisoryIfApplicable(
        Orbit orbit, double totalDv, double totalBurnTime)
    {
        if (orbit == null || !orbit.IsBound()) return;
        double period = orbit.Period;
        if (!(period > 0.0) || double.IsNaN(period)) return;
        if (!(totalBurnTime > 0.0)) return;

        double burnRatio = totalBurnTime / period;
        if (burnRatio <= SuggestThreshold) return;

        int suggestedN = ComputeSuggestedPassCount(burnRatio);
        double singlePassLoss = totalDv * MultiPassLoss.FiniteBurnLossFraction(burnRatio);
        double splitLoss = totalDv * MultiPassLoss.FiniteBurnLossFraction(burnRatio / suggestedN);
        double estimatedSavings = singlePassLoss - splitLoss;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 200, 60, 255));
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Burn duration ({0:F0}s) is {1:F0}%% of orbital period. " +
            "Splitting across {2} passes saves ~{3:F0} m/s.",
            totalBurnTime, burnRatio * 100.0, suggestedN, estimatedSavings));
        ImGui.PopStyleColor();
    }

    // Smallest N where per-pass loss is below the ceiling AND one more
    // split would save under 0.1% of total dV. Hard-capped at
    // SuggestMaxN to bound real-time wait; for very long burns this
    // may return SuggestMaxN with per-pass loss still above the
    // ceiling.
    private static int ComputeSuggestedPassCount(double burnRatio)
    {
        double lossN = MultiPassLoss.FiniteBurnLossFraction(burnRatio / SuggestMinN);
        for (int n = SuggestMinN; n < SuggestMaxN; n++)
        {
            double lossNext = MultiPassLoss.FiniteBurnLossFraction(burnRatio / (n + 1));
            if (lossN <= SuggestPerPassLossCeiling
                && lossN - lossNext <= SuggestMarginalSavingCeiling)
                return n;
            lossN = lossNext;
        }
        return SuggestMaxN;
    }

    // Loss-frame cumulative savings: sums per-pass dv * FBL(burnTime/period)
    // against the equivalent single-burn loss. Per-pass burn time and dV
    // come from the cached preview so EqualDv vs EqualBurnTime splits both
    // get accurate per-pass loss (not just the EqualBurnTime simplification).
    // Hidden when savings < 1 m/s: signals to the user that the chosen N
    // does not buy meaningful Oberth improvement (e.g. overkill N).
    private static void DrawSavingsLine(
        double totalDv, double singleBurnTime, double period, PassPreview[] passes)
    {
        if (passes.Length == 0) return;
        if (!(period > 0.0) || !(singleBurnTime > 0.0) || !(totalDv > 0.0)) return;

        double singleLoss = totalDv * MultiPassLoss.FiniteBurnLossFraction(singleBurnTime / period);
        double splitLoss = 0.0;
        for (int i = 0; i < passes.Length; i++)
        {
            double dv = passes[i].DvVlf.Length();
            double bt = passes[i].EstimatedBurnTimeSec;
            if (bt > 0.0)
                splitLoss += dv * MultiPassLoss.FiniteBurnLossFraction(bt / period);
        }
        double savings = singleLoss - splitLoss;
        if (savings < 1.0) return;

        ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        ImGui.Text(string.Format(Inv,
            "Total savings vs single burn: ~{0:F0} m/s", savings));
        ImGui.PopStyleColor();
    }

    #endregion

    // Total burn time for totalDv via multi-stage Tsiolkovsky.
    // Returns 0 when stage data is missing.
    private static double EstimateBurnTime(double totalDv, SequenceBurnState state)
    {
        if (totalDv <= 0.0 || !state.HasUsableEngines) return 0.0;
        var alloc = Splitter.Allocate(totalDv, 1, SplitMode.EqualDv, state);
        return alloc.Length > 0 ? alloc[0].EstimatedBurnTimeSec : 0.0;
    }
}
