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

    // Per-pass arc-fraction cap. Suggested N = ceil(burnRatio / cap).
    private const double SuggestPerPassCap = 0.10;

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

    public static bool IsApseBurnType(string typeKey) =>
        typeKey == KeySetApoapsis || typeKey == KeySetPeriapsis;

    /// <summary>Multi-pass selected and preview is usable.</summary>
    public static bool IsArmed(string typeKey) =>
        Enabled
        && _passCount > 1
        && IsApseBurnType(typeKey)
        && MultiPassPreviewCache.HasPreview;

    /// <summary>Multi-pass selected but planner could not produce a
    /// preview; Create should be disabled rather than fall back.</summary>
    public static bool WantsMultiPassButCannot(string typeKey) =>
        Enabled
        && _passCount > 1
        && IsApseBurnType(typeKey)
        && !MultiPassPreviewCache.HasPreview;

    public static void Draw(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        if (!Enabled || source == null || !IsApseBurnType(typeKey))
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
            DrawPassList();
        }
        else
            MultiPassPreviewCache.ClearPreview();
    }

    public static void Render(Viewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source == null || source.Id != MultiPassPreviewCache.PreviewSourceId) return;
        MultiPassRenderer.RenderPassOrbits(viewport, source, MultiPassPreviewCache.PreviewPasses);
    }

    /// <summary>Per-pass Ap/Pe/AN/DN/encounter markers via stock
    /// FlightPlan.DrawUi. ImGui-phase counterpart of <see cref="Render"/>.</summary>
    public static void RenderMarkers(Viewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source == null || source.Id != MultiPassPreviewCache.PreviewSourceId) return;

        PassPreview[] passes = MultiPassPreviewCache.PreviewPasses;
        for (int i = 0; i < passes.Length; i++)
        {
            var uiContext = new Astronomical.UiContext(
                viewport, source, Color.Green,
                TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0),
                ManeuverToolsWindow.GetSelectedTargetOrbiter());
            passes[i].FlightPlan.DrawUi(viewport, uiContext);
        }
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

    private static void DrawPassList()
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
            string line = t > 0.5
                ? string.Format(Inv, "Pass {0}: {1:F0} m/s, {2:F0}s", i + 1, dv, t)
                : string.Format(Inv, "Pass {0}: {1:F0} m/s", i + 1, dv);
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

        int suggestedN = Math.Clamp(
            (int)Math.Ceiling(burnRatio / SuggestPerPassCap),
            SuggestMinN, SuggestMaxN);

        // Closed-form finite-burn loss (Robbins / sinc approximation):
        //   dV_loss / D ~= (pi * burnRatio)^2 / 6
        // Splitting into N reduces the loss by a factor of 1/N^2.
        double singlePassLoss = totalDv * Math.Pow(Math.PI * burnRatio, 2.0) / 6.0;
        double splitLoss = singlePassLoss / (suggestedN * suggestedN);
        double estimatedSavings = Math.Max(0.0, singlePassLoss - splitLoss);

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 200, 60, 255));
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Burn duration ({0:F0}s) is {1:F0}%% of orbital period. " +
            "Splitting across {2} passes saves ~{3:F0} m/s.",
            totalBurnTime, burnRatio * 100.0, suggestedN, estimatedSavings));
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
