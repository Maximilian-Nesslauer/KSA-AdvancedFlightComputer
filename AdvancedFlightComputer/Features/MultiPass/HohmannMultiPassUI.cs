using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// The passes and the flyby of a transfer on the TRANSFER page, planned from the selection of the AFC transfer session. Quick tools use MultiPassUI.
internal static class HohmannMultiPassUI
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly string[] SplitModeLabels = { "EQUAL TIME", "EQUAL DELTA-V" };

    public static bool Enabled { get; set; }

    private static int _passCount = 1;
    // EqualBurnTime distributes the burn arc across the periapsis passes to reduce losses from finite burn duration. EqualDv distributes delta v evenly.
    private static SplitMode _splitMode = SplitMode.EqualBurnTime;

    // Quantization prevents each physics sample from rebuilding the preview. The pass index separates execution steps.
    private readonly record struct PreviewKey(
        string SourceId,
        string TargetId,
        long TFinalBucketSec,
        long DvMagBucket,
        long VInfBucket,
        long ApoTargetBucket,
        bool IsCrossParent,
        int PassCount,
        SplitMode Mode,
        int StartPassIndex,
        long MassBucket,
        bool FlybyOn,
        long FlybyRpBucket,
        FlybySide FlybySide,
        FlybyTargeting.DepartureKey? FlybyDeparture = null,
        int PhaseSignature = 0)
    {
        // Thrust changes mass and can shift the final time through the parking period. Ignore those fields while thrust is active, but retain changes to the transfer, flyby request, and pass index.
        public PreviewKey WithoutDrift() =>
            this with { TFinalBucketSec = 0, MassBucket = 0, PhaseSignature = 0,
                FlybyDeparture = FlybyDeparture?.WithoutParkingDrift() };
    }

    private static PreviewKey _cachedKey;
    private static bool _hasCachedPreview;
    private static PassPreviewResult _cachedPreview;
    private static int _autoClampedFromN;     // Zero means no clamp. A positive value preserves the requested count.
    // Keep the failure kind with the clamp reason so advice does not depend on the wording of the message.
    private static string? _autoClampReason;
    private static PassPlanFailure _autoClampKind;
    // A schedule shift can delay departure for a moon transfer within the same parent system. Zero means the selected departure time needs no shift.
    private static int _lastShiftKShift;
    private static double _cachedFuelSum = double.NaN;
    private static double _cachedFuelTotalDv = double.NaN;
    private static string? _lastSourceId;
    private static string? _lastTargetId;

    // A failed flyby retarget leaves the departure aimed at the center of the body. Keep this state so the UI can warn the user.
    private static bool _flybyRetargetFailed;
    private static FlybyTargeting.DepartureSolution? _plannedFlybyDeparture;

    public static void DrawInputs()
    {
        if (!Enabled || RunsOnSession()) return;

        if (!ShouldDraw(out Vehicle? source, out OrbitalTransfers.PorkChopEntry? entry,
                       out OrbitalTransfers.TransferInfo? info))
            return;

        // Reset the state when the source or target changes so geometry from another transfer cannot remain visible.
        string targetId = (info!.Target as Astronomical)?.Id ?? string.Empty;
        if (_lastSourceId != source!.Id || _lastTargetId != targetId)
        {
            _lastSourceId = source.Id;
            _lastTargetId = targetId;
            _passCount = 1;
            _splitMode = SplitMode.EqualBurnTime;
            InvalidatePreview();
        }

        try
        {
            DrawBodyInputs(source, entry!, info!);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("hohmann-inputs:" + ex.GetType().Name,
                $"[AFC] HohmannMultiPassUI.DrawInputs: source={source.Id} target={targetId} passes={_passCount} mode={_splitMode}: {ex}");
        }
    }

    public static void DrawResults()
    {
        if (!Enabled) return;

        if (TryDrawActiveFastPath()) return;

        if (!ShouldDraw(out Vehicle? source, out OrbitalTransfers.PorkChopEntry? entry,
                       out OrbitalTransfers.TransferInfo? info))
            return;

        try
        {
            DrawBodyResults(source!, entry!, info!);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("hohmann-inline:" + ex.GetType().Name,
                $"[AFC] HohmannMultiPassUI.DrawResults: source={source!.Id} passes={_passCount} mode={_splitMode}: {ex}");
        }
    }

    private static bool RunsOnSession()
        => TransferSession.Source is Vehicle source
           && MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec)
           && exec.Intent is HohmannTransferIntent;

    // A running transfer multi-pass shows its passes whatever the selection is, because a new calculation clears the selection.
    private static bool TryDrawActiveFastPath()
    {
        if (TransferSession.Source is not Vehicle source) return false;
        if (!MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
            return false;
        if (exec.Intent is not HohmannTransferIntent) return false;

        try { DrawActive(source, exec); }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("hohmann-inline-active:" + ex.GetType().Name,
                $"[AFC] HohmannMultiPassUI.DrawSection: source={source.Id} pass={exec.PassIndex + 1}/{exec.PassCountTotal} kind={exec.Intent.Kind}: {ex}");
        }
        return true;
    }

    public static string SplitSummary => _splitMode == SplitMode.EqualDv ? "Equal delta-v" : "Equal burn time";

    public static bool SplitRowShown => Enabled && _passCount > 1 && ShouldDraw(out Vehicle? source, out _, out _) && !MultiPassRegistry.Has(source!.Id);

    public static void DrawSplitRow() => DrawSplitModeSelector();

    public static int ChangedInFold => !Enabled || _passCount <= 1 || _splitMode == SplitMode.EqualBurnTime ? 0 : 1;

    /// <summary>The pass count the TRANSFER footer reads for its verb.</summary>
    public static int PassCount => _passCount;

    /// <summary>The passes of the preview for the burn bars, empty without a multi-pass preview.</summary>
    public static PassPreview[] PreviewPasses => _hasCachedPreview && !_cachedPreview.Failed ? _cachedPreview.Passes : Array.Empty<PassPreview>();

    public static PassPreview[] PreviewPassesFrom(Vehicle source, int startPassIndex)
        => _cachedKey.SourceId == source.Id && _cachedKey.StartPassIndex == startPassIndex ? PreviewPasses : Array.Empty<PassPreview>();

    public static void Reset()
    {
        _passCount = 1;
        _splitMode = SplitMode.EqualBurnTime;
        InvalidatePreview();
        _lastSourceId = null;
        _lastTargetId = null;
        HohmannMultiPassPlanner.ResetShiftCache();
    }

    private static void InvalidatePreview()
    {
        _hasCachedPreview = false;
        _cachedPreview = default;
        _cachedKey = default;
        _cachedFuelSum = double.NaN;
        _cachedFuelTotalDv = double.NaN;
        _autoClampedFromN = 0;
        _autoClampReason = null;
        _autoClampKind = PassPlanFailure.None;
        _lastShiftKShift = 0;
        _flybyRetargetFailed = false;
        _plannedFlybyDeparture = null;
    }

    // Clear the preview before registry removal can leave an overlay on the orbit after the burn.
    public static void OnExecutionEnded(string vehicleId)
    {
        if (!_hasCachedPreview) return;
        if (_cachedKey.SourceId != vehicleId) return;
        InvalidatePreview();
    }

    // The flyby preview key can keep a disposed source vehicle reachable.
    public static void OnVehicleDisposed(string vehicleId)
    {
        if (_cachedKey.SourceId != vehicleId
            && _plannedFlybyDeparture?.Key.Source.Id != vehicleId) return;
        InvalidatePreview();
    }

    // The cached pass count preserves the active overlay when the user switches away from the source and back.
    public static bool HasMultiPassPreview
    {
        get
        {
            if (!Enabled || !_hasCachedPreview || _cachedPreview.Failed) return false;
            if (_cachedPreview.Passes.Length == 0) return false;
            // Changing the source resets the selected pass count to one. The active preview retains its total pass count in the cache key, so the overlay can return when the source is selected again.
            return _passCount > 1 || _cachedKey.StartPassIndex > 0
                   || _cachedKey.PassCount > 1;
        }
    }

    // Stock draws the queued pass, so only the remaining passes are drawn here.
    public static void RenderOrbits(IViewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source.Id != _cachedKey.SourceId) return;

        bool skipFirst = MultiPassRegistry.Has(source.Id);
        MultiPassRenderer.RenderPassOrbits(
            viewport, source, _cachedPreview.Passes, skipFirst);
    }

    // Keep the original pass numbers when the preview contains only the remaining passes.
    public static void RenderMarkers(IViewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source.Id != _cachedKey.SourceId) return;

        int firstPassDisplayNumber = 1;
        bool skipFirst = false;
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
        {
            firstPassDisplayNumber = exec.PassIndex + 1;
            skipFirst = true;
        }
        MultiPassMarkers.Draw(viewport, source,
            _cachedPreview.Passes, firstPassDisplayNumber, skipFirst);
    }

    // True while RenderOrbits and RenderMarkers draw the passes for the TRANSFER session's source vehicle. The selected transfer is one burn of the whole transfer delta v, which multi-pass does not fly, so the TRANSFER page hides it then.
    public static bool IsDrawingPassOverlay()
        => ShouldRenderOverlay(out Vehicle? source)
           && source != null && source.Id == _cachedKey.SourceId;

    /// <summary>
    /// Whether the session's craft flies the final pass of a transfer multi-pass. Stock then draws that
    /// pass as the queued node of the flight plan, so the selected transfer is not drawn on top of it.
    /// </summary>
    internal static bool FinalPassQueued
        => TransferSession.Source is Vehicle source
           && MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec)
           && exec.Intent is HohmannTransferIntent
           && exec.PassIndex >= exec.PassCountTotal - 1;

    // Active execution survives a new calculation. An uncommitted preview draws only while TRANSFER is on screen with a result.
    public static bool ShouldRenderOverlay(out Vehicle? source)
    {
        source = null;
        if (!HasMultiPassPreview) return false;
        if (!TransferSession.ShowMap) return false;

        source = TransferSession.Source;
        if (source == null) return false;

        if (MultiPassRegistry.Has(source.Id))
            return true;
        return TransferSession.Calculated && TransferSession.PageVisible;
    }

    private static bool ShouldDraw(
        out Vehicle? source,
        out OrbitalTransfers.PorkChopEntry? entry,
        out OrbitalTransfers.TransferInfo? info)
    {
        source = null;
        entry = null;
        info = null;

        try
        {
            // The session clears the selection on a source or destination change, so a calculated result always matches its transfer information.
            if (!TransferSession.Calculated) return false;

            entry = TransferSession.SelectedEntry;
            if (entry == null) return false;

            info = TransferSession.Info;
            if (info == null || info.Target == null) return false;

            source = TransferSession.Source;
            return source != null && source.Orbit?.Parent != null;
        }
        catch
        {
            return false;
        }
    }

    private static void DrawBodyInputs(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        // An active intent owns its pending burn, so another intent gets no controls that could replace it.
        if (MultiPassRegistry.Has(source.Id))
            return;

        // Apply flyby targeting before splitting the departure so all passes use the requested aim.
        HohmannFlybyUI.DrawControls(source, info);

        double dv = entry.TransferData.TransferDvVlf.Length();
        double burnTime = MultiPassUI.EstimateBurnTime(dv, MultiPassPreviewCache.GetSequenceState(source));
        double period = source.Orbit?.Period ?? 0.0;
        bool matters = Ui.Logic.FormRules.PassesCanMatter(dv, burnTime, period);
        bool savingKnown = double.IsFinite(Ui.Logic.FormRules.PassesSaving(dv, burnTime, period));
        if (MultiPassUI.PassCountRow("##AfcHmpPasses", ref _passCount, matters ? "" : Ui.Logic.PageText.PassesRowDetail(_passCount, savingKnown), matters ? "" : Ui.Logic.PageText.PassesRowTip(savingKnown)))
            InvalidatePreview();

        if (_passCount > 1)
            UpdatePreviewIfStale(source, entry, info);
        ShowFlybyDeparture(source, entry, info);
    }

    private static void DrawBodyResults(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
        {
            DrawBlockedByOtherExecution(exec);
            return;
        }

        if (_passCount > 1)
        {
            DrawPreviewFailureIfApplicable();
            DrawAdvisoryIfApplicable();
            DrawFlybyMultipassNoteIfApplicable();
        }
        HohmannFlybyUI.DrawSelectedResult(entry);
        if (_passCount > 1 && _hasCachedPreview && !_cachedPreview.Failed)
        {
            DrawSpanInfo(source);
            DrawTotalsAndSavings(source, _cachedPreview.Passes,
                entry.TransferData.TransferDvVlf.Length(),
                source.Orbit?.Period ?? 0.0);
        }

        // Keep the clamp message visible even when only one pass remains. Clear it when the user changes the pass count or split mode.
        DrawAutoClampIfApplicable();
    }

    private static void ShowFlybyDeparture(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry, OrbitalTransfers.TransferInfo info)
    {
        if (_passCount <= 1)
            HohmannFlybyUI.ShowSingleDeparture(source, entry, info);
        else if (info.Target is IParentBody target)
        {
            PassPreview[] passes = _hasCachedPreview ? _cachedPreview.Passes : Array.Empty<PassPreview>();
            FlightPlan? finalPlan = !_cachedPreview.Failed && passes.Length > 0 ? passes[^1].FlightPlan : null;
            HohmannFlybyUI.ShowMultiPassDeparture(_plannedFlybyDeparture, finalPlan, target, entry);
        }
    }

    private static void DrawSpanInfo(Vehicle source)
    {
        if (source.Orbit == null) return;
        double tPark = source.Orbit.Period;
        if (!(tPark > 0.0)) return;
        if (!_hasCachedPreview || _cachedPreview.Failed
            || _cachedPreview.Passes.Length < 2) return;
        var passes = _cachedPreview.Passes;
        double spanSec = passes[passes.Length - 1].BurnTime.Seconds()
                         - passes[0].BurnTime.Seconds();
        if (!(spanSec > 0.0)) return;

        ConsoleUi.Readout("SPAN".AsSpan(), string.Format(Inv, "{0:F0} parking periods (~{1})",
            spanSec / tPark, FormatHelper.FormatDuration(spanSec)).AsSpan());
        if (_lastShiftKShift > 0)
        {
            double shiftSec = _lastShiftKShift * tPark;
            ConsoleUi.MutedWrapped(string.Format(Inv,
                "Final burn moved {0} parking orbits (~{1}) later so the " +
                "passes fit. The transfer is planned again for that time.",
                _lastShiftKShift, FormatHelper.FormatDuration(shiftSec)));
        }
    }

    private static void DrawSplitModeSelector()
    {
        ConsoleUi.BeginRow(FoldNames.Split.AsSpan());
        int picked = ConsoleUi.Segmented("AfcHmpSplit", SplitModeLabels,
            _splitMode == SplitMode.EqualDv ? 1 : 0);
        if (ConsoleWidgets.RowHovered)
            ConsoleUi.Tooltip(
                "Equal burn time fires the engines for the same duration each pass, equalizing finite-burn arc length (the literature-standard default for finite-burn loss). Equal delta-v delivers the same magnitude each pass: simpler to reason about, slightly less efficient for finite burns.".AsSpan());
        ConsoleUi.EndRow();

        SplitMode mode = picked == 1 ? SplitMode.EqualDv : SplitMode.EqualBurnTime;
        if (picked < 0 || mode == _splitMode) return;
        _splitMode = mode;
        InvalidatePreview();
    }

    #region Active execution

    private static void DrawActive(Vehicle source, MultiPassExecution exec)
    {
        ConsoleUi.Readout("PASSES RUN".AsSpan(),
            string.Format(Inv, "Pass {0} of {1}", exec.PassIndex + 1, exec.PassCountTotal).AsSpan());

        // Rebuild the active preview from the locked intent and current orbit. The pass index invalidates the cache after each completed pass.
        if (exec.Intent is HohmannTransferIntent intent)
            UpdatePreviewForActiveExec(source, exec, intent);

        // Show planning failures during execution so the user can understand why the overlay disappeared.
        DrawPreviewFailureIfApplicable();
        DrawAdvisoryIfApplicable();

    }

    private static void UpdatePreviewForActiveExec(
        Vehicle source, MultiPassExecution exec, HohmannTransferIntent intent)
    {
        if (Universe.CurrentSystem == null) return;
        if (!Universe.CurrentSystem.All.TryGet(intent.TargetId, out Astronomical? targetA))
            return;
        if (targetA is not IOrbiter target) return;

        var input = new HohmannMultiPassPlanner.HohmannPlanInput(
            Target: target,
            TFinal: new UniverseTime(intent.TFinalSec),
            DFinalVlf: intent.DFinalVlf,
            IsCrossParent: intent.IsCrossParent,
            VInfMs: intent.VInfMs,
            ApoTargetRadiusMeters: intent.ApoTargetRadiusMeters,
            Transit: intent.TransitSec is double transitSec ? new UniverseTime(transitSec) : null);

        var key = BuildKey(source, input,
            passCount: exec.PassCountTotal,
            mode: exec.Mode,
            startPassIndex: exec.PassIndex);
        if (_hasCachedPreview && key == _cachedKey) return;

        if (CanKeepPreviewDuringThrust(source, key))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        // The intent already contains the schedule shift in its locked final time. Only the live final pass advisory needs to be evaluated here.
        _cachedPreview = HohmannMultiPassPlanner.Plan(
            source, input, exec.PassCountTotal, exec.PassIndex,
            intent.ParkingPeriodSec, state, now, exec.Mode);
        _cachedKey = key;
        _hasCachedPreview = true;
    }

    private static void DrawBlockedByOtherExecution(MultiPassExecution exec)
    {
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "This craft runs {0} passes (pass {1} of {2}). " +
            "STOP PASSES in the AFC head ends it before a transfer can start.",
            exec.Intent.Kind, exec.PassIndex + 1, exec.PassCountTotal));
    }

    #endregion

    #region Pass count / split mode / preview

    private static void UpdatePreviewIfStale(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        UniverseTime now = Universe.GetElapsedTime();
        double parkingPeriodSec = source.Orbit?.Period ?? double.NaN;
        if (!(parkingPeriodSec > 0.0))
        {
            _passCount = 1;
            InvalidatePreview();
            return;
        }

        var raw = BuildBasePlanInput(source, entry, info);
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        var shift = HohmannMultiPassPlanner.PrepareShiftedInput(
            raw, source, info, _passCount, parkingPeriodSec, now, _splitMode, state);
        var key = BuildKey(source, shift.Input, _passCount, _splitMode, 0,
            ResolveFlybyTransit(shift, entry));
        if (_hasCachedPreview && key == _cachedKey)
        {
            _lastShiftKShift = shift.KShift;
            return;
        }
        if (CanKeepPreviewDuringThrust(source, key)) return;

        shift = ClampPassCount(source, info, raw, shift, state, parkingPeriodSec, now);
        // The cache key describes the shifted input and requested flyby. The retargeted result is derived from them.
        key = BuildKey(source, shift.Input, _passCount, _splitMode, 0,
            ResolveFlybyTransit(shift, entry));
        PlanPreview(source, entry, info, shift, key, state, parkingPeriodSec, now);
    }

    // Freeze drift from mass and time while thrust is active. Changes to other inputs or the pass index still invalidate the preview.
    private static bool CanKeepPreviewDuringThrust(Vehicle source, PreviewKey key)
        => _hasCachedPreview
           && MultiPassPreviewCache.ShouldFreezeForThrust(
               source, key.WithoutDrift() == _cachedKey.WithoutDrift());

    private static HohmannMultiPassPlanner.ShiftResult ClampPassCount(
        Vehicle source, OrbitalTransfers.TransferInfo info,
        HohmannMultiPassPlanner.HohmannPlanInput raw,
        HohmannMultiPassPlanner.ShiftResult shift, SequenceBurnState state,
        double parkingPeriodSec, UniverseTime now)
    {
        int requestedN = _passCount;
        _lastShiftKShift = shift.KShift;

        int clampedN = HohmannMultiPassPlanner.LargestFeasibleN(
            source, shift.Input, state, parkingPeriodSec, now, requestedN, _splitMode,
            out string? clampReason, out PassPlanFailure clampKind);

        if (clampedN < requestedN)
        {
            // Keep the warning until the user changes the pass count or split mode.
            _autoClampedFromN = Math.Max(_autoClampedFromN, requestedN);
            _autoClampReason = clampReason;
            _autoClampKind = clampKind;
            _passCount = clampedN;
            // Fewer passes require fewer parking periods.
            shift = HohmannMultiPassPlanner.PrepareShiftedInput(
                raw, source, info, _passCount, parkingPeriodSec, now, _splitMode, state);
            _lastShiftKShift = shift.KShift;

        }

        return shift;
    }

    private static void PlanPreview(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info, HohmannMultiPassPlanner.ShiftResult shift,
        PreviewKey key, SequenceBurnState state, double parkingPeriodSec, UniverseTime now)
    {
        // Apply the flyby offset before splitting the departure. Retain a failed retarget so the UI can warn about the unchanged aim.
        var planInput = MaybeApplyFlyby(
            source, info, shift.Input, ResolveFlybyTransit(shift, entry), out _flybyRetargetFailed);

        _cachedPreview = HohmannMultiPassPlanner.Plan(
            source, planInput, _passCount, startPassIndex: 0,
            parkingPeriodSec, state, now, _splitMode);
        // Prefer the advisory from the chained final trajectory. Use the shifted single burn scan only when the plan has no advisory.
        if (shift.ScanAdvisory != null && _cachedPreview.Advisory == null)
            _cachedPreview = _cachedPreview with { Advisory = shift.ScanAdvisory };
        _cachedKey = key;
        _hasCachedPreview = true;

        UpdateFuelEstimate(planInput.DFinalVlf.Length(), state);
    }

    private static void UpdateFuelEstimate(double totalDv, SequenceBurnState state)
    {
        if (totalDv > 0.0 && state.HasUsableEngines)
        {
            _cachedFuelSum = Splitter.SumDvCapacityMs(
                Splitter.Allocate(totalDv, _passCount, _splitMode, state));
            _cachedFuelTotalDv = totalDv;
        }
        else
        {
            _cachedFuelSum = double.NaN;
            _cachedFuelTotalDv = double.NaN;
        }
    }

    private static void DrawPreviewFailureIfApplicable()
    {
        if (!_hasCachedPreview || !_cachedPreview.Failed) return;
        ImGui.Spacing();
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "Pass preview incomplete: {0}.",
            _cachedPreview.FailureReason ?? "unknown reason"));
    }

    private static void DrawAdvisoryIfApplicable()
    {
        if (!_hasCachedPreview || _cachedPreview.Failed) return;
        if (_cachedPreview.Advisory == null) return;
        ImGui.Spacing();
        ConsoleUi.WarningWrapped(ConsoleUi.EscapeFormat(_cachedPreview.Advisory));
    }

    private static void DrawAutoClampIfApplicable()
    {
        if (_autoClampedFromN <= _passCount) return;
        ImGui.Spacing();

        // Use the failure kind so message edits cannot change the advice. Suggest a split mode only when it differs from the current mode.
        SplitMode otherMode = _splitMode == SplitMode.EqualBurnTime
            ? SplitMode.EqualDv
            : SplitMode.EqualBurnTime;
        string otherModeLabel = otherMode == SplitMode.EqualBurnTime
            ? "equal burn time"
            : "equal delta-v";

        string advice = _autoClampKind switch
        {
            PassPlanFailure.TimeBudget =>
                "Pick a later porkchop entry (arrow buttons / further right) "
                + "to gain time budget.",
            PassPlanFailure.SoiCeiling =>
                "More passes would push the intermediate orbit past the parent "
                + $"SOI envelope. Reduce passes, or try {otherModeLabel} for "
                + "tighter K.",
            PassPlanFailure.ParabolicVp =>
                "An intermediate orbit reaches the required departure speed too early. "
                + "Reduce passes, or try "
                + $"{otherModeLabel}.",
            PassPlanFailure.NonMonotonicK =>
                "The passes do not divide evenly at this count. One pass fewer usually works.",
            PassPlanFailure.KFloor =>
                "Each pass would be too small at this count; reduce passes.",
            PassPlanFailure.FuelShort =>
                "Vehicle has insufficient fuel for this transfer (reducing "
                + "passes will not help). Add fuel or pick a lower-energy departure.",
            _ =>
                $"Reduce passes, try {otherModeLabel}, or pick a later porkchop entry.",
        };

        ConsoleUi.WarningWrapped(string.Format(Inv,
            "{0} passes asked for, only {1} can fly from this departure. {2}",
            _autoClampedFromN, _passCount, advice));
        if (MultiPassDebug.Enabled && _autoClampReason != null)
            ConsoleUi.MutedWrapped(string.Format(Inv,
                "Debug: kind={0}, reason: {1}", _autoClampKind, _autoClampReason));
    }

    private static void DrawTotalsAndSavings(
        Vehicle source, PassPreview[] passes, double lambertDv, double tPark)
    {
        if (passes.Length == 0 || !(lambertDv > 0.0)) return;

        ConsoleUi.Readout("SINGLE BURN".AsSpan(),
            string.Format(Inv, "{0:F0} m/s", lambertDv).AsSpan());
        string? savingsLine = TryFormatRobbinsSavings(source, passes, lambertDv, tPark);
        if (savingsLine != null)
            ConsoleUi.Readout("SAVINGS VS SINGLE BURN".AsSpan(), savingsLine.AsSpan());
    }

    // A shifted transfer or a plan with insufficient fuel cannot be compared with the selected single burn. Using the parking period for every pass slightly overestimates savings.
    private static string? TryFormatRobbinsSavings(
        Vehicle source, PassPreview[] passes, double lambertDv, double tPark)
    {
        if (_lastShiftKShift > 0) return null;
        if (!(tPark > 0.0)) return null;

        // With insufficient fuel, the single burn loss uses unreachable delta v while the split uses available delta v. Comparing them would inflate savings.
        if (!double.IsNaN(_cachedFuelSum) && !double.IsNaN(_cachedFuelTotalDv)
            && _cachedFuelTotalDv > 0.0 && _cachedFuelSum < _cachedFuelTotalDv * 0.995)
            return null;

        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        if (!state.HasUsableEngines) return null;

        // The split mode has no effect for one pass. EqualDv matches the fuel estimate in MultiPassUI.
        PassAllocation[] singleAlloc = Splitter.Allocate(
            lambertDv, 1, SplitMode.EqualDv, state);
        double singleBurnTime = singleAlloc.Length > 0
            ? singleAlloc[0].EstimatedBurnTimeSec : 0.0;
        if (!(singleBurnTime > 0.0)) return null;

        double singleLoss = lambertDv
            * MultiPassLoss.FiniteBurnLossFraction(singleBurnTime / tPark);
        double splitLoss = 0.0;
        for (int i = 0; i < passes.Length; i++)
        {
            double dv = passes[i].DvVlf.Length();
            double bt = passes[i].EstimatedBurnTimeSec;
            if (bt > 0.0)
                splitLoss += dv
                    * MultiPassLoss.FiniteBurnLossFraction(bt / tPark);
        }
        double savings = singleLoss - splitLoss;
        if (savings < 1.0) return null;

        return string.Format(Inv, "~{0:F0} m/s", savings);
    }

    #endregion

    #region Interceptor handoff

    public static bool WantedMultiPassButPreviewFailed() =>
        Enabled && _passCount > 1 && _hasCachedPreview && _cachedPreview.Failed;

    public static string FuelShortFact()
    {
        if (!Enabled || _passCount <= 1 || (_hasCachedPreview && _cachedPreview.Failed))
            return "";
        if (double.IsNaN(_cachedFuelSum) || double.IsNaN(_cachedFuelTotalDv) || !(_cachedFuelTotalDv > 0.0))
            return "";
        return _cachedFuelSum < _cachedFuelTotalDv * 0.995 ? Ui.Logic.PageText.FuelShort(_cachedFuelTotalDv, _cachedFuelSum) : "";
    }

    public static bool WantsMultiPass => Enabled && _passCount > 1;

    public static bool TryGetArmedState(
        Vehicle vehicle, out int passCount, out HohmannTransferIntent? intent,
        out SplitMode mode)
    {
        passCount = 0;
        intent = null;
        mode = SplitMode.EqualBurnTime;
        if (!Enabled) return false;
        if (_passCount <= 1) return false;
        if (!_hasCachedPreview || _cachedPreview.Failed) return false;

        if (!ShouldDraw(out Vehicle? uiSource,
                out OrbitalTransfers.PorkChopEntry? entry,
                out OrbitalTransfers.TransferInfo? info))
            return false;
        if (uiSource == null || uiSource.Id != vehicle.Id) return false;

        UniverseTime now = Universe.GetElapsedTime();
        intent = BuildIntent(uiSource, entry!, info!, _passCount, now);
        if (intent == null) return false;

        passCount = _passCount;
        mode = _splitMode;
        return true;
    }

    #endregion

    #region Helpers

    private static HohmannMultiPassPlanner.HohmannPlanInput BuildBasePlanInput(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        // Compare parent IDs directly. TransferTask.Run can rewrite the source to the vehicle, which makes SameSoiTransfer unsuitable for this check.
        string? targetParentId = info.Target?.Parent?.Id;
        string? sourceParentId = source.Orbit?.Parent?.Id;
        bool isCrossParent = sourceParentId == null
            || targetParentId == null
            || sourceParentId != targetParentId;
        double vInfMs = 0.0;
        double apoTargetRadiusM = 0.0;

        if (isCrossParent)
        {
            // DepartureVelocityCci is hyperbolic excess in the CCI frame of the parking orbit parent. Its magnitude is unchanged by frame rotation.
            vInfMs = entry.TransferData.DepartureVelocityCci.Length();
        }
        else
        {
            // The first flight plan patch gives the apoapsis immediately after the burn. An unbound orbit gives NaN, which the planner handles.
            FlightPlan? fp = entry.FlightPlan;
            if (fp != null && fp.Patches.Count > 0 && source.Orbit != null)
            {
                double apo = fp.Patches[0].Orbit.Apoapsis;
                if (double.IsFinite(apo) && apo > source.Orbit.Periapsis)
                    apoTargetRadiusM = apo;
            }
        }

        return new HohmannMultiPassPlanner.HohmannPlanInput(
            Target: info.Target!,
            TFinal: entry.TransferData.Start,
            DFinalVlf: entry.TransferData.TransferDvVlf,
            IsCrossParent: isCrossParent,
            VInfMs: vInfMs,
            ApoTargetRadiusMeters: apoTargetRadiusM,
            Transit: entry.TransferData.Transit);
    }


    private static HohmannTransferIntent? BuildIntent(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info,
        int passCount, UniverseTime now)
    {
        if (source.Orbit?.Parent == null) return null;
        string targetId = (info.Target as Astronomical)?.Id ?? string.Empty;
        if (string.IsNullOrEmpty(targetId)) return null;

        // Lock the parking period with the intent. Reading the period after an earlier pass would change the schedule.
        double parkingPeriod = source.Orbit.Period;
        if (!(parkingPeriod > 0.0)) return null;

        // Apply the same schedule shift and split mode as the preview before locking the final time and direction. Otherwise the first pass can be scheduled too early.
        var raw = BuildBasePlanInput(source, entry, info);
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        var shift = HohmannMultiPassPlanner.PrepareShiftedInput(
            raw, source, info, passCount, parkingPeriod, now, _splitMode, state);
        var input = shift.Input;

        // Use the same flyby transit as the preview. Reject a failed retarget so the execution cannot silently depart toward an impact.
        input = MaybeApplyFlyby(
            source, info, input, ResolveFlybyTransit(shift, entry), out bool flybyFailed);
        if (flybyFailed) return null;

        return new HohmannTransferIntent
        {
            TargetId = targetId,
            ParentId = source.Orbit.Parent.Id,
            TFinalSec = input.TFinal.Seconds(),
            DFinalVlf = input.DFinalVlf,
            IsCrossParent = input.IsCrossParent,
            VInfMs = input.VInfMs,
            ApoTargetRadiusMeters = input.ApoTargetRadiusMeters,
            ParkingPeriodSec = parkingPeriod,
            TransitSec = input.Transit?.Seconds(),
        };
    }

    private static PreviewKey BuildKey(
        Vehicle source, HohmannMultiPassPlanner.HohmannPlanInput input,
        int passCount, SplitMode mode, int startPassIndex, UniverseTime? transit = null)
    {
        // Changing the flyby altitude or side must invalidate the preview.
        bool flybyOn = false;
        long flybyRpBucket = 0;
        FlybySide flybySide = FlybySide.Inner;
        FlybyTargeting.DepartureKey? departureKey = null;
        if (input.Target is IParentBody target
            && HohmannFlybyUI.TryGetRequest(target, out double rp, out FlybySide side))
        {
            flybyOn = true;
            flybyRpBucket = (long)(rp / 1000.0);
            flybySide = side;
            if (transit is UniverseTime duration)
                departureKey = FlybyTargeting.CaptureDepartureKey(
                    source, input.Target, input.TFinal, duration, rp, side);
        }

        return new PreviewKey(
            SourceId: source.Id,
            TargetId: (input.Target as Astronomical)?.Id ?? string.Empty,
            TFinalBucketSec: (long)input.TFinal.Seconds(),
            DvMagBucket: (long)input.DFinalVlf.Length(),
            VInfBucket: (long)input.VInfMs,
            ApoTargetBucket: (long)(input.ApoTargetRadiusMeters / 1000.0),
            IsCrossParent: input.IsCrossParent,
            PassCount: passCount,
            Mode: mode,
            StartPassIndex: startPassIndex,
            MassBucket: (long)(source.TotalMass / 100.0),
            FlybyOn: flybyOn,
            FlybyRpBucket: flybyRpBucket,
            FlybySide: flybySide,
            FlybyDeparture: departureKey,
            PhaseSignature: FiniteBurns.PhaseSignature(source));
    }

    // Preview and commit must use the same shifted transit.
    private static UniverseTime ResolveFlybyTransit(
        HohmannMultiPassPlanner.ShiftResult shift, OrbitalTransfers.PorkChopEntry entry)
        => shift.KShift > 0 ? shift.ShiftedTransit : entry.TransferData.Transit;

    // Transfers between different parent systems lock hyperbolic excess. Transfers within one parent system lock apoapsis radius. A failed retarget preserves the input and sets the failure flag.
    internal static HohmannMultiPassPlanner.HohmannPlanInput MaybeApplyFlyby(
        Vehicle source, OrbitalTransfers.TransferInfo info,
        HohmannMultiPassPlanner.HohmannPlanInput center, UniverseTime transit, out bool failed)
    {
        failed = false;
        _plannedFlybyDeparture = null;
        if (info.Target is not IParentBody target) return center;
        if (info.Target is not IOrbiter targetOrbiter) return center;
        if (!HohmannFlybyUI.TryGetRequest(target, out double rp, out FlybySide side))
            return center;

        var key = FlybyTargeting.CaptureDepartureKey(
            source, targetOrbiter, center.TFinal, transit, rp, side);
        _plannedFlybyDeparture = FlybyTargeting.GetDeparture(key);
        var outcome = _plannedFlybyDeparture.Outcome;
        if (outcome.Result == null)
        {
            failed = true;
            return center;
        }
        FlybyTargeting.FlybyResult f = outcome.Result.Value;

        if (center.IsCrossParent)
        {
            // Hyperbolic excess defines this departure. Apoapsis is unused.
            if (!(f.PlannerVInfMs > 0.0)) { failed = true; return center; }
            return center with
            {
                TFinal = f.BurnTime,
                DFinalVlf = f.DvVlf,
                VInfMs = f.PlannerVInfMs,
                Transit = FlybyTransit(f, center),
            };
        }

        // Apoapsis defines this departure. Hyperbolic excess is unused.
        if (!(f.PlannerApoTargetMeters > 0.0)) { failed = true; return center; }
        return center with
        {
            TFinal = f.BurnTime,
            DFinalVlf = f.DvVlf,
            ApoTargetRadiusMeters = f.PlannerApoTargetMeters,
            Transit = FlybyTransit(f, center),
        };
    }

    private static UniverseTime? FlybyTransit(
        FlybyTargeting.FlybyResult flyby, HohmannMultiPassPlanner.HohmannPlanInput center)
        => flyby.Arrival is UniverseTime arrival && arrival > flyby.BurnTime
            ? arrival - flyby.BurnTime
            : center.Transit;

    private static void DrawFlybyMultipassNoteIfApplicable()
    {
        if (!HohmannFlybyUI.FlybyRequested || !_flybyRetargetFailed) return;
        ImGui.Spacing();
        ConsoleUi.WarningWrapped("Flyby retarget failed for this multi-pass geometry; the split would still impact. Reduce passes, change the window, or flip the side.");
    }

    #endregion
}
