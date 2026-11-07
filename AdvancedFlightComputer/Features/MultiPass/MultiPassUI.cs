using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.Ui.Logic;
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

    private static readonly string[] SplitModeLabels = { "EQUAL TIME", "EQUAL DELTA-V" };

    private static int _passCount = 1;
    private static SplitMode _splitMode = SplitMode.EqualBurnTime;

    private static string? _lastTypeKey;
    private static string? _lastSourceId;

    private static MultiPassExecution? _shortExec;
    private static double _shortMs;

    public static void NoteRunningShort(MultiPassExecution exec, double shortMs)
    {
        _shortExec = exec;
        _shortMs = shortMs;
    }

    public static double RunningShortMs(MultiPassExecution exec) => ReferenceEquals(_shortExec, exec) ? _shortMs : 0.0;

    public static int PassCount => _passCount;
    public static SplitMode CurrentSplitMode => _splitMode;

    public static bool HasMultiPassPreview =>
        Enabled && _passCount > 1 && MultiPassPreviewCache.HasPreview;

    private static bool IsMultiPassSupportedType(string typeKey) =>
        typeKey == KeySetApoapsis
        || typeKey == KeySetPeriapsis
        || typeKey == KeyMatchInclination
        || typeKey == KeySetInclination
        || typeKey == KeyCircularizeApoapsis
        || typeKey == KeyCircularizePeriapsis;

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

    /// <summary>Footer sentence while the passes of this tool cannot deliver the goal, or empty. The burn bars cannot show this, because a preview caps each pass at the fuel it has.</summary>
    public static string FuelShortFact(string typeKey, double totalDv)
    {
        if (!IsArmed(typeKey) || totalDv <= 0.0)
            return "";
        double sum = MultiPassPreviewCache.CachedAllocationsSum;
        return !double.IsNaN(sum) && sum < totalDv * 0.995 ? PageText.FuelShort(totalDv, sum) : "";
    }

    public static void Draw(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey, bool countRow = true)
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

        ResetOnNewTool(source, typeKey);

        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);

        // An active execution locks its pass count and split mode. Preview only the remaining passes.
        if (exec != null)
        {
            DrawActive(source, typeKey, state, exec);
            return;
        }

        double totalDv = maneuver.DvCci.Length();
        double totalBurnTime = EstimateBurnTime(totalDv, state);

        double period = source.Orbit?.Period ?? 0.0;
        bool matters = FormRules.PassesCanMatter(totalDv, totalBurnTime, period);
        bool savingKnown = double.IsFinite(FormRules.PassesSaving(totalDv, totalBurnTime, period));
        ConsoleUi.NoteShape("AfcMpPassesRow", countRow ? 1 : 0);
        if (countRow && PassCountRow("##AfcMpPasses", ref _passCount, matters ? "" : PageText.PassesRowDetail(_passCount, savingKnown), matters ? "" : PageText.PassesRowTip(savingKnown)))
            MultiPassPreviewCache.Invalidate();

        DrawAdvisoryIfApplicable(source.Orbit!, totalDv, totalBurnTime);

        if (_passCount > 1)
        {
            MultiPassPreviewCache.UpdatePreviewIfStale(
                source, maneuver, typeKey, _passCount, _splitMode, state, totalDv);
            DrawPreviewFailureIfApplicable();
            DrawSavingsLine(totalDv, totalBurnTime, source.Orbit?.Period ?? 0.0,
                MultiPassPreviewCache.PreviewPasses);
        }
        else
            MultiPassPreviewCache.ClearPreview();
    }

    public static void DrawRunning(Vehicle source, string typeKey)
    {
        if (!Enabled || !MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
            return;
        if (exec.Intent.TypeKey != typeKey)
        {
            DrawBlockedByOtherExecution(exec);
            return;
        }
        ResetOnNewTool(source, typeKey);
        DrawActive(source, typeKey, MultiPassPreviewCache.GetSequenceState(source), exec);
    }

    public static PassPreview[] RunningPasses(Vehicle source, MultiPassExecution exec)
    {
        int remaining = exec.PassCountTotal - exec.PassIndex;
        if (!Enabled || remaining <= 0)
            return Array.Empty<PassPreview>();
        string typeKey = exec.Intent.TypeKey;
        if (!IsMultiPassSupportedType(typeKey))
        {
            PassPreview[] planned = HohmannMultiPassUI.PreviewPassesFrom(source, exec.PassIndex);
            return planned.Length == remaining ? planned : Array.Empty<PassPreview>();
        }
        OrbitManeuvers.ManeuverResult? locked = exec.Intent.ComputeManeuver(source);
        if (locked == null)
            return Array.Empty<PassPreview>();
        MultiPassPreviewCache.UpdatePreviewIfStale(source, locked.Value, typeKey, remaining, exec.Mode,
            MultiPassPreviewCache.GetSequenceState(source), locked.Value.DvCci.Length());
        return MultiPassPreviewCache.PreviewPasses.Length == remaining ? MultiPassPreviewCache.PreviewPasses : Array.Empty<PassPreview>();
    }

    private static void ResetOnNewTool(Vehicle source, string typeKey)
    {
        if (_lastTypeKey == typeKey && _lastSourceId == source.Id)
            return;
        _lastTypeKey = typeKey;
        _lastSourceId = source.Id;
        _passCount = 1;
        _splitMode = SplitMode.EqualBurnTime;
        MultiPassPreviewCache.ClearPreview();
    }

    private static void DrawActive(
        Vehicle source, string typeKey,
        SequenceBurnState state, MultiPassExecution exec)
    {
        int remaining = exec.PassCountTotal - exec.PassIndex;
        if (remaining <= 0) return;

        _passCount = exec.PassCountTotal;

        // Use the locked intent rather than an editable target while execution is active.
        OrbitManeuvers.ManeuverResult? lockedManeuver = exec.Intent.ComputeManeuver(source);
        if (lockedManeuver == null) return;

        double totalDv = lockedManeuver.Value.DvCci.Length();
        MultiPassPreviewCache.UpdatePreviewIfStale(
            source, lockedManeuver.Value, typeKey, remaining, exec.Mode, state, totalDv);
        DrawPreviewFailureIfApplicable();
        double sum = MultiPassPreviewCache.CachedAllocationsSum;
        NoteRunningShort(exec, state.HasUsableEngines && totalDv > 0.0 && !double.IsNaN(sum) && sum < totalDv * 0.995 ? totalDv - sum : 0.0);
        DrawSavingsLine(totalDv, EstimateBurnTime(totalDv, state),
            source.Orbit?.Period ?? 0.0, MultiPassPreviewCache.PreviewPasses);
    }

    // The caller draws execution status and cancellation controls separately.
    private static void DrawBlockedByOtherExecution(MultiPassExecution exec)
    {
        ImGui.Spacing();
        // A Hohmann multi-pass is not an ORBIT tool, so its passes are on TRANSFER.
        if (OrbitToolCatalog.IsTool(exec.Intent.TypeKey))
            ConsoleUi.WarningWrapped(string.Format(Inv,
                "This craft runs passes of \"{0}\". Pick \"{0}\" again to view them.",
                OrbitToolCatalog.DisplayName(exec.Intent.TypeKey)));
        else
            ConsoleUi.WarningWrapped("This craft runs the passes of a transfer. TRANSFER shows them.");
    }

    public static string SplitSummary => _splitMode == SplitMode.EqualDv ? "Equal delta-v" : "Equal burn time";

    public static void DrawSplitRow(Vehicle source, string typeKey)
    {
        if (SplitRowShown(source, typeKey))
            DrawSplitModeSelector();
    }

    public static bool SplitRowShown(Vehicle source, string typeKey)
        => Enabled && source != null && _passCount > 1 && IsMultiPassSupportedType(typeKey)
           && _lastTypeKey == typeKey && _lastSourceId == source.Id && !MultiPassRegistry.Has(source.Id);

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
        _shortExec = null;
        _shortMs = 0.0;
    }

    #region UI components

    /// <summary>The PASSES row of the orbit tools and of the transfer page. Returns whether the count changed.</summary>
    internal static bool PassCountRow(string id, ref int count, string detail, string tip)
    {
        ConsoleUi.BeginRow("PASSES".AsSpan());
        bool detailUnder = detail.Length > 0 && !ConsoleUi.RowDetail(detail, ConsoleWidgets.RowControlWidth);
        if (tip.Length > 0 && ConsoleWidgets.RowHovered)
            ConsoleUi.Tooltip(tip.AsSpan());
        int passes = count;
        ImGui.SetNextItemWidth(ConsoleWidgets.RowControlWidth);
        bool edited = ImGui.InputInt(id, ref passes);
        ConsoleUi.EndRow();
        ConsoleUi.NoteShape("AfcPassesDetailUnder", detailUnder ? 1 : 0);
        if (detailUnder)
            ConsoleUi.Muted(detail.AsSpan());
        passes = Math.Clamp(passes, MinPasses, Splitter.MaxPasses);
        if (!edited || passes == count)
            return false;
        count = passes;
        return true;
    }

    private static void DrawSplitModeSelector()
    {
        ConsoleUi.BeginRow(FoldNames.Split.AsSpan());
        int picked = ConsoleUi.Segmented("AfcMpSplit", SplitModeLabels,
            _splitMode == SplitMode.EqualDv ? 1 : 0);
        if (ConsoleWidgets.RowHovered)
            ConsoleUi.Tooltip(
                "Equal burn time fires the engines for the same duration each pass, equalizing finite-burn arc length (Oberth-optimal default). Equal delta-v delivers the same magnitude each pass.".AsSpan());
        ConsoleUi.EndRow();

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
            "Pass preview incomplete: {0}. " +
            "Try fewer passes or the other choice of EACH PASS HAS.",
            reason));
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
        ConsoleUi.AdviceWrapped(ConsoleUi.EscapeFormat(string.Format(Inv,
            "Burn duration ({0:F0}s) is {1:F0}% of orbital period. " +
            "Splitting across {2} passes saves ~{3:F0} m/s.",
            totalBurnTime, burnRatio * 100.0, suggestedN, estimatedSavings)));
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

        ConsoleUi.Readout("SAVINGS VS SINGLE BURN".AsSpan(),
            string.Format(Inv, "~{0:F0} m/s", savings).AsSpan());
    }

    #endregion

    // Use the staged Tsiolkovsky calculation for total burn time. Missing propulsion data produces zero.
    internal static double EstimateBurnTime(double totalDv, SequenceBurnState state)
    {
        if (totalDv <= 0.0 || !state.HasUsableEngines) return 0.0;
        var alloc = Splitter.Allocate(totalDv, 1, SplitMode.EqualDv, state);
        return alloc.Length > 0 ? alloc[0].EstimatedBurnTimeSec : 0.0;
    }
}
