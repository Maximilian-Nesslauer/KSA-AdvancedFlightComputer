using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using static AdvancedFlightComputer.Features.ManeuverTools.ManeuverTools;

namespace AdvancedFlightComputer.Features.MultiPass;

// These controls keep the selected pass count, split mode, and advisory preview together.
internal static class MultiPassUI
{
    private const int MinPasses = 1;

    public static bool Enabled { get; set; }

    // The burn duration relative to the orbital period determines whether to suggest splitting.
    private const double SuggestThreshold = 0.15;

    private const double SuggestPerPassLossCeiling = 0.005;   // Allow 0.5 percent loss from finite burn duration per pass.
    private const double SuggestMarginalSavingCeiling = 0.001; // 0.1% of total dV gained per added pass

    private const int SuggestMinN = 2;
    private const int SuggestMaxN = 8;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static readonly string[] SplitModeLabels = { "EQUAL TIME", "EQUAL DV" };

    private static int _passCount = 1;
    private static SplitMode _splitMode = SplitMode.EqualBurnTime;

    private static string? _lastTypeKey;
    private static string? _lastSourceId;

    public static int PassCount => _passCount;
    public static SplitMode CurrentSplitMode => _splitMode;

    public static bool HasMultiPassPreview =>
        Enabled && _passCount > 1 && MultiPassPreviewCache.HasPreview;

    private static bool IsMultiPassSupportedType(string typeKey) =>
        typeKey == KeySetApoapsis
        || typeKey == KeySetPeriapsis
        || typeKey == KeyMatchInclination
        || typeKey == KeySetInclination
        || typeKey == KeyStockCircularizeApoapsis
        || typeKey == KeyStockCircularizePeriapsis;

    public static bool IsArmed(string typeKey) =>
        Enabled
        && _passCount > 1
        && IsMultiPassSupportedType(typeKey)
        && MultiPassPreviewCache.HasPreview;

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

        MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec);

        // Block another active intent before resetting the cache. This preserves its markers and prevents the selected planner from using its locked delta v.
        if (exec != null && exec.Intent.TypeKey != typeKey)
        {
            DrawBlockedByOtherExecution(exec);
            return;
        }

        // Reset when the transfer type or source changes so another maneuver cannot reuse this preview.
        if (_lastTypeKey != typeKey || _lastSourceId != source.Id)
        {
            _lastTypeKey = typeKey;
            _lastSourceId = source.Id;
            _passCount = 1;
            _splitMode = SplitMode.EqualBurnTime;
            MultiPassPreviewCache.ClearPreview();
        }

        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);

        // An active execution locks its pass count and split mode. Preview only the remaining passes.
        if (exec != null)
        {
            DrawActive(source, typeKey, state, exec);
            return;
        }

        double totalDv = maneuver.DvCci.Length();
        double totalBurnTime = EstimateBurnTime(totalDv, state);

        ImGui.Spacing();
        DrawPassCountSelector();

        if (_passCount > 1)
            DrawSplitModeSelector();

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

        // Use the locked intent rather than an editable target while execution is active.
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

    // Use the display name from TransferTypes. The caller draws execution status and cancellation controls separately.
    private static void DrawBlockedByOtherExecution(MultiPassExecution exec)
    {
        string typeKey = exec.Intent.TypeKey;
        string label = typeKey;
        foreach (TransferType t in TransferPlanner.TransferTypes)
        {
            if (t.GetKey() == typeKey)
            {
                label = t.GetName();
                break;
            }
        }

        ImGui.Spacing();
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "Vehicle is running a \"{0}\" multi-pass. " +
            "Switch the Plan Type back to \"{0}\" to view its passes.",
            label));
    }

    public static void Render(IViewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source == null || source.Id != MultiPassPreviewCache.PreviewSourceId) return;

        // Stock already draws the queued burn orbit.
        bool skipFirst = MultiPassRegistry.Has(source.Id);
        MultiPassRenderer.RenderPassOrbits(
            viewport, source, MultiPassPreviewCache.PreviewPasses, skipFirst);
    }

    public static void RenderMarkers(IViewport viewport, Vehicle source)
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
        ConsoleWidgets.BeginRow("PASSES".AsSpan());
        int passes = _passCount;
        if (ConsoleWidgets.SliderInt("AfcMpPasses".AsSpan(), ref passes, MinPasses,
                Splitter.MaxPasses, passes.ToString(Inv).AsSpan(), pending: false))
        {
            _passCount = passes;
            MultiPassPreviewCache.Invalidate();
        }
        ConsoleWidgets.EndRow();
    }

    private static void DrawPassList(int firstPassDisplayNumber)
    {
        if (!MultiPassPreviewCache.HasPreview) return;

        PassPreview[] passes = MultiPassPreviewCache.PreviewPasses;
        if (passes.Length == 0) return;

        for (int i = 0; i < passes.Length; i++)
        {
            double dv = passes[i].DvVlf.Length();
            double t = passes[i].EstimatedBurnTimeSec;
            string value = t > 0.5
                ? string.Format(Inv, "{0:F0} m/s, {1:F0}s", dv, t)
                : string.Format(Inv, "{0:F0} m/s", dv);
            ConsoleWidgets.Readout(
                string.Format(Inv, "PASS {0}", firstPassDisplayNumber + i).AsSpan(), value.AsSpan());
        }
    }

    private static void DrawSplitModeSelector()
    {
        ConsoleWidgets.BeginRow("SPLIT".AsSpan());
        int picked = ConsoleWidgets.Segmented("AfcMpSplit".AsSpan(), SplitModeLabels,
            _splitMode == SplitMode.EqualDv ? 1 : 0);
        if (ConsoleWidgets.RowHovered)
            ConsoleWidgets.Tooltip(
                "Equal burn time fires the engines for the same duration each pass, equalizing finite-burn arc length (Oberth-optimal default). Equal delta-v delivers the same magnitude each pass.".AsSpan());
        ConsoleWidgets.EndRow();

        SplitMode pickedMode = picked == 1 ? SplitMode.EqualDv : SplitMode.EqualBurnTime;
        if (picked >= 0 && pickedMode != _splitMode)
        {
            _splitMode = pickedMode;
            MultiPassPreviewCache.Invalidate();
        }
    }

    private static void DrawPreviewFailureIfApplicable()
    {
        if (!MultiPassPreviewCache.LastPreviewFailed) return;

        ImGui.Spacing();
        string reason = MultiPassPreviewCache.LastPreviewFailureReason ?? "unknown reason";
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "Multi-pass preview incomplete: {0}. " +
            "Try fewer passes or a different split mode.",
            reason));
    }

    // Cache the allocated delta v total so fuel checks do not run the splitter each frame.
    private static void DrawInsufficientFuelIfApplicable(
        double totalDv, SequenceBurnState state)
    {
        if (totalDv <= 0.0 || !state.HasUsableEngines) return;

        double sum = MultiPassPreviewCache.CachedAllocationsSum;
        if (double.IsNaN(sum)) return;

        // Allow a 0.5 percent tolerance for floating point differences in the staged Tsiolkovsky calculation.
        if (sum >= totalDv * 0.995) return;

        ImGui.Spacing();
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "Vehicle can only deliver ~{0:F0} m/s of the {1:F0} m/s required. " +
            "Multi-pass will run out of fuel before the goal is reached.",
            sum, totalDv));
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
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "Burn duration ({0:F0}s) is {1:F0}% of orbital period. " +
            "Splitting across {2} passes saves ~{3:F0} m/s.",
            totalBurnTime, burnRatio * 100.0, suggestedN, estimatedSavings));
    }

    // Choose the smallest pass count that meets the loss threshold and saves less than 0.1 percent with another pass. The maximum count can still leave the loss above the threshold.
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

    // Compare losses using each pass delta v and burn time so both split modes are supported. Hide savings below one meter per second.
    private static void DrawSavingsLine(
        double totalDv, double singleBurnTime, double period, PassPreview[] passes)
    {
        if (passes.Length == 0) return;
        if (!(period > 0.0) || !(singleBurnTime > 0.0) || !(totalDv > 0.0)) return;

        // Do not compare unreachable requested delta v with a split limited by available fuel. That would inflate savings.
        double allocSum = MultiPassPreviewCache.CachedAllocationsSum;
        if (!double.IsNaN(allocSum) && allocSum < totalDv * 0.995) return;

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

        ConsoleWidgets.Readout("SAVINGS VS SINGLE BURN".AsSpan(),
            string.Format(Inv, "~{0:F0} m/s", savings).AsSpan());
    }

    #endregion

    // Use the staged Tsiolkovsky calculation for total burn time. Missing propulsion data produces zero.
    private static double EstimateBurnTime(double totalDv, SequenceBurnState state)
    {
        if (totalDv <= 0.0 || !state.HasUsableEngines) return 0.0;
        var alloc = Splitter.Allocate(totalDv, 1, SplitMode.EqualDv, state);
        return alloc.Length > 0 ? alloc[0].EstimatedBurnTimeSec : 0.0;
    }
}
