using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Hohmann keeps separate UI state because stock owns this window. Quick tools use MultiPassUI.
internal static class HohmannMultiPassUI
{
    private const int MinPasses = 1;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly string[] SplitModeLabels = { "EQUAL TIME", "EQUAL DV" };

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
        FlybyTargeting.DepartureKey? FlybyDeparture = null)
    {
        // Thrust changes mass and can shift the final time through the parking period. Ignore those fields while thrust is active, but retain changes to the transfer, flyby request, and pass index.
        public PreviewKey WithoutDrift() =>
            this with { TFinalBucketSec = 0, MassBucket = 0,
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

    private static int _lastFrameDrawn = -1;

    // Both window injection sites share one ImGui frame claim to prevent duplicate controls.
    public static void DrawInline()
    {
        if (!Enabled) return;
        int frame = ImGui.GetFrameCount();
        if (frame == _lastFrameDrawn) return;
        _lastFrameDrawn = frame;

        if (TryDrawActiveFastPath()) return;

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
            ImGui.Separator();
            DrawBody(source, entry!, info!);
            ImGui.Separator();
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("hohmann-inline:" + ex.GetType().Name,
                $"[AFC] HohmannMultiPassUI.DrawInline: source={source.Id} target={targetId} passes={_passCount} mode={_splitMode}: {ex}");
        }
    }

    // Keep active execution status available after closing the window clears the selected transfer.
    private static bool TryDrawActiveFastPath()
    {
        if (!StockPlanner.ShowPlanWindow) return false;
        if (StockPlanner.TransferTypeKey != ManeuverTools.ManeuverTools.KeyStockHohmann)
            return false;

        if (StockPlanner.SourceVehicle is not Vehicle source) return false;
        if (!MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
            return false;
        if (exec.Intent is not HohmannTransferIntent) return false;

        ImGui.Separator();
        try { DrawActive(source, exec); }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("hohmann-inline-active:" + ex.GetType().Name,
                $"[AFC] HohmannMultiPassUI.DrawInline: source={source.Id} pass={exec.PassIndex + 1}/{exec.PassCountTotal} kind={exec.Intent.Kind}: {ex}");
        }
        ImGui.Separator();
        return true;
    }

    public static void Reset()
    {
        _passCount = 1;
        _splitMode = SplitMode.EqualBurnTime;
        InvalidatePreview();
        _lastSourceId = null;
        _lastTargetId = null;
        _lastFrameDrawn = -1;
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

    internal static void ClearFlybyPreview()
    {
        if (_cachedKey.FlybyDeparture != null || _plannedFlybyDeparture != null)
            InvalidatePreview();
    }

    // Clear the preview before registry removal can leave an overlay on the orbit after the burn.
    public static void OnExecutionEnded(string vehicleId)
    {
        if (!_hasCachedPreview) return;
        if (_cachedKey.SourceId != vehicleId) return;
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

    // Stock draws the queued pass. It draws the final pass only while the selected transfer overlay is available.
    public static void RenderOrbits(IViewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source.Id != _cachedKey.SourceId) return;

        bool skipFirst = MultiPassRegistry.Has(source.Id);
        bool stockOwnsFinal = StockSelectedTransferOverlayActive();
        MultiPassRenderer.RenderPassOrbits(
            viewport, source, _cachedPreview.Passes,
            skipFirst, skipLast: stockOwnsFinal);
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
        bool stockOwnsFinal = StockSelectedTransferOverlayActive();
        MultiPassMarkers.Draw(viewport, source,
            _cachedPreview.Passes, firstPassDisplayNumber,
            skipFirst, skipLast: stockOwnsFinal);
    }

    // The flyby overlay hides the stock path aimed at the center, so MultiPass must draw the final pass.
    private static bool StockSelectedTransferOverlayActive()
    {
        if (HohmannFlybyUI.FlybyRequested) return false;
        return StockPlanner.ShowPlanWindow
               && StockPlanner.DisplaySelectedTransfer
               && StockPlanner.TransferCalculated;
    }

    // Active execution survives invalidation of the stock preview. An uncommitted preview must be discarded.
    public static bool ShouldRenderOverlay(out Vehicle? source)
    {
        source = null;
        if (!HasMultiPassPreview) return false;

        if (!StockPlanner.ShowPlanWindow) return false;
        if (!StockPlanner.DisplaySelectedTransfer) return false;
        if (StockPlanner.TransferTypeKey != ManeuverTools.ManeuverTools.KeyStockHohmann)
            return false;

        source = StockPlanner.SourceVehicle;
        if (source == null) return false;

        if (!MultiPassRegistry.Has(source.Id) && !StockPlanner.TransferCalculated)
            return false;

        return true;
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
            if (StockPlanner.TransferTypeKey != ManeuverTools.ManeuverTools.KeyStockHohmann)
                return false;

            if (!StockPlanner.ShowPlanWindow) return false;

            // The selected entry can survive a source or target change. Require TransferCalculated to avoid combining that entry with different transfer information.
            if (!StockPlanner.TransferCalculated) return false;

            entry = StockPlanner.SelectedEntry;
            if (entry == null) return false;

            info = StockPlanner.TransferInfo;
            if (info == null || info.Target == null) return false;

            source = StockPlanner.SourceVehicle;
            return source != null && source.Orbit?.Parent != null;
        }
        catch
        {
            return false;
        }
    }

    private static void DrawBody(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        // An active intent owns its pending burn. Show its status or block the controls before another intent can replace it.
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
        {
            if (exec.Intent is HohmannTransferIntent)
                DrawActive(source, exec);
            else
                DrawBlockedByOtherExecution(exec);
            return;
        }

        ConsoleWidgets.RegionHeader("MULTI-PASS DEPARTURE".AsSpan());
        ConsoleWidgets.Readout("DESTINATION".AsSpan(),
            ((info.Target as Astronomical)?.Id ?? "?").AsSpan());
        ConsoleWidgets.Readout("LAMBERT DV".AsSpan(),
            string.Format(Inv, "{0:F1} m/s", entry.TransferData.TransferDvVlf.Length()).AsSpan());

        // Apply flyby targeting before splitting the departure so all passes use the requested aim.
        HohmannFlybyUI.DrawControls(source, info);

        ImGui.Spacing();

        DrawPassCountSelector();

        if (_passCount > 1)
        {
            DrawSplitModeSelector();
            UpdatePreviewIfStale(source, entry, info);
            ShowFlybyDeparture(source, entry, info);
            DrawSpanInfo(source);
            DrawPreviewFailureIfApplicable();
            DrawInsufficientFuelIfApplicable();
            DrawAdvisoryIfApplicable();
            DrawFlybyMultipassNoteIfApplicable();
            DrawPassList();
            if (_hasCachedPreview && !_cachedPreview.Failed)
                DrawTotalsAndSavings(source, _cachedPreview.Passes,
                    entry.TransferData.TransferDvVlf.Length(),
                    source.Orbit?.Period ?? 0.0);
        }
        else
        {
            ShowFlybyDeparture(source, entry, info);
        }

        // Keep the clamp message visible even when only one pass remains. Clear it when the user changes the pass count or split mode.
        DrawAutoClampIfApplicable();

        bool flyby = HohmannFlybyUI.FlybyRequested;
        if (_passCount <= 1 && _autoClampedFromN <= _passCount)
        {
            ImGui.Spacing();
            ConsoleUi.MutedWrapped(flyby
                ? "N = 1: the Create button does a single flyby burn."
                : "N = 1: the Create button does a single Hohmann burn.");
        }
        else if (_passCount > 1)
        {
            ImGui.Spacing();
            ConsoleUi.MutedWrapped(string.Format(Inv,
                flyby
                    ? "Click Create to start the {0}-pass flyby execution."
                    : "Click Create to start the {0}-pass execution.",
                _passCount));
        }
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
        HohmannFlybyUI.DrawSelectedResult(entry);
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

        ConsoleWidgets.Readout("SPAN".AsSpan(), string.Format(Inv, "{0:F0} parking periods (~{1})",
            spanSec / tPark, FormatHelper.FormatDuration(spanSec)).AsSpan());
        if (_lastShiftKShift > 0)
        {
            double shiftSec = _lastShiftKShift * tPark;
            ConsoleUi.MutedWrapped(string.Format(Inv,
                "Final burn pushed {0} parking period(s) (~{1}) later so the " +
                "multi-pass schedule fits; transfer re-planned at the later time.",
                _lastShiftKShift, FormatHelper.FormatDuration(shiftSec)));
        }
    }

    private static void DrawSplitModeSelector()
    {
        ConsoleWidgets.BeginRow("SPLIT".AsSpan());
        int picked = ConsoleWidgets.Segmented("AfcHmpSplit".AsSpan(), SplitModeLabels,
            _splitMode == SplitMode.EqualDv ? 1 : 0);
        if (ConsoleWidgets.RowHovered)
            ConsoleWidgets.Tooltip(
                "Equal burn time fires the engines for the same duration each pass, equalizing finite-burn arc length (the literature-standard default for finite-burn loss). Equal delta-v delivers the same magnitude each pass: simpler to reason about, slightly less efficient for finite burns.".AsSpan());
        ConsoleWidgets.EndRow();

        SplitMode mode = picked == 1 ? SplitMode.EqualDv : SplitMode.EqualBurnTime;
        if (picked < 0 || mode == _splitMode) return;
        _splitMode = mode;
        InvalidatePreview();
    }

    #region Active execution

    private static void DrawActive(Vehicle source, MultiPassExecution exec)
    {
        ConsoleWidgets.Readout("MULTI-PASS ACTIVE".AsSpan(),
            string.Format(Inv, "PASS {0} OF {1}", exec.PassIndex + 1, exec.PassCountTotal).AsSpan());

        // Rebuild the active preview from the locked intent and current orbit. The pass index invalidates the cache after each completed pass.
        if (exec.Intent is HohmannTransferIntent intent)
            UpdatePreviewForActiveExec(source, exec, intent);

        // The first preview entry is the queued pass. Keep its displayed number relative to the full execution.
        DrawPassList(firstPassDisplayNumber: exec.PassIndex + 1);

        // Show planning failures during execution so the user can understand why the overlay disappeared.
        DrawPreviewFailureIfApplicable();
        DrawAdvisoryIfApplicable();

        // Stock hides its preview checkbox after the window is closed. This control lets the user restore the overlay without another calculation.
        if (!StockPlanner.TransferCalculated)
            DrawInlinePreviewToggle();

        ImGui.Spacing();
        if (ConsoleWidgets.DangerButton("CANCEL REMAINING PASSES".AsSpan()))
            CancelExecution(source, exec);
    }

    private static void DrawInlinePreviewToggle()
    {
        bool preview = StockPlanner.DisplaySelectedTransfer;
        if (ConsoleUi.CheckboxRow("PREVIEW SELECTED TRANSFER".AsSpan(),
                "AfcHmpPreview".AsSpan(), ref preview))
            StockPlanner.DisplaySelectedTransfer = preview;
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
            ApoTargetRadiusMeters: intent.ApoTargetRadiusMeters);

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
            "Vehicle is already running a {0} multi-pass ({1} of {2}). " +
            "Cancel it from its own plan window before starting a Hohmann.",
            exec.Intent.Kind, exec.PassIndex + 1, exec.PassCountTotal));
    }

    private static void CancelExecution(Vehicle source, MultiPassExecution exec)
    {
        Burn? pending = exec.TryResolveCurrentBurn(source.FlightComputer.BurnPlan);
        if (pending != null)
        {
            InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
            {
                Burn = pending,
                FlightComputer = source.FlightComputer,
                DeleteBurn = true,
            });
        }
        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] HohmannMultiPass: vehicle={source.Id} user cancelled at pass " +
                $"{exec.PassIndex + 1}/{exec.PassCountTotal}.");
        MultiPassRegistry.Remove(source.Id);
        PassCompletionPatch.OnRegistryRemovedExternally(source.Id);
        OnExecutionEnded(source.Id);
    }

    #endregion

    #region Pass count / split mode / preview

    private static void DrawPassCountSelector()
    {
        ConsoleWidgets.BeginRow("PASSES".AsSpan());
        int passes = _passCount;
        bool changed = ConsoleWidgets.SliderInt("AfcHmpPasses".AsSpan(), ref passes,
            MinPasses, Splitter.MaxPasses, passes.ToString(Inv).AsSpan(), pending: false);
        ConsoleWidgets.EndRow();
        if (!changed)
            return;

        _passCount = passes;
        InvalidatePreview();
    }

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
            "Multi-pass preview incomplete: {0}.",
            _cachedPreview.FailureReason ?? "unknown reason"));
    }

    private static void DrawAdvisoryIfApplicable()
    {
        if (!_hasCachedPreview || _cachedPreview.Failed) return;
        if (_cachedPreview.Advisory == null) return;
        ImGui.Spacing();
        ConsoleUi.WarningWrapped(_cachedPreview.Advisory);
    }

    private static void DrawInsufficientFuelIfApplicable()
    {
        if (_hasCachedPreview && _cachedPreview.Failed) return;
        if (double.IsNaN(_cachedFuelSum) || double.IsNaN(_cachedFuelTotalDv)) return;
        if (!(_cachedFuelTotalDv > 0.0)) return;
        if (_cachedFuelSum >= _cachedFuelTotalDv * 0.995) return;

        ImGui.Spacing();
        ConsoleUi.WarningWrapped(string.Format(Inv,
            "Vehicle can only deliver ~{0:F0} m/s of the {1:F0} m/s required. " +
            "Multi-pass will run out of fuel before the departure is reached.",
            _cachedFuelSum, _cachedFuelTotalDv));
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
                "Even with priors auto-capped at escape velocity the transfer "
                + $"is too high-energy for this N. Reduce passes, or try {otherModeLabel}.",
            PassPlanFailure.NonMonotonicK =>
                "Integer-sum rounding artifact at this N; reduce passes by one "
                + "(N-1 typically works).",
            PassPlanFailure.KFloor =>
                "Per-pass dV too small to be meaningful at this N; reduce passes.",
            PassPlanFailure.FuelShort =>
                "Vehicle has insufficient fuel for this transfer (reducing "
                + "passes will not help). Add fuel or pick a lower-energy departure.",
            _ =>
                $"Reduce passes, try {otherModeLabel}, or pick a later porkchop entry.",
        };

        ConsoleUi.WarningWrapped(string.Format(Inv,
            "{0} pass(es) requested, only {1} feasible at this departure entry. {2}",
            _autoClampedFromN, _passCount, advice));
        if (MultiPassDebug.Enabled && _autoClampReason != null)
            ConsoleUi.MutedWrapped(string.Format(Inv,
                "Debug: kind={0}, reason: {1}", _autoClampKind, _autoClampReason));
    }

    private static void DrawPassList(int firstPassDisplayNumber = 1)
    {
        if (!_hasCachedPreview) return;
        PassPreview[] passes = _cachedPreview.Passes;
        if (passes.Length == 0) return;

        for (int i = 0; i < passes.Length; i++)
        {
            double dv = passes[i].DvVlf.Length();
            double t = passes[i].EstimatedBurnTimeSec;
            string label = i == passes.Length - 1
                ? string.Format(Inv, "PASS {0} (FINAL)", firstPassDisplayNumber + i)
                : string.Format(Inv, "PASS {0}", firstPassDisplayNumber + i);
            string value = t > 0.5
                ? string.Format(Inv, "{0:F0} m/s, {1:F0}s", dv, t)
                : string.Format(Inv, "{0:F0} m/s", dv);
            ConsoleWidgets.Readout(label.AsSpan(), value.AsSpan());
        }
    }

    private static void DrawTotalsAndSavings(
        Vehicle source, PassPreview[] passes, double lambertDv, double tPark)
    {
        if (passes.Length == 0 || !(lambertDv > 0.0)) return;

        double sumDv = 0.0;
        for (int i = 0; i < passes.Length; i++)
            sumDv += passes[i].DvVlf.Length();

        ConsoleWidgets.Rule();
        ConsoleWidgets.Readout("TOTAL".AsSpan(),
            string.Format(Inv, "{0:F0} m/s", sumDv).AsSpan());
        ConsoleWidgets.Readout("LAMBERT".AsSpan(),
            string.Format(Inv, "{0:F0} m/s", lambertDv).AsSpan());
        string? savingsLine = TryFormatRobbinsSavings(source, passes, lambertDv, tPark);
        if (savingsLine != null) ConsoleUi.MutedWrapped(savingsLine);
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

        return string.Format(Inv,
            "Robbins savings estimate vs single burn: ~{0:F0} m/s", savings);
    }

    #endregion

    #region Interceptor handoff

    public static bool WantedMultiPassButPreviewFailed() =>
        Enabled && _passCount > 1 && _hasCachedPreview && _cachedPreview.Failed;

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
            ApoTargetRadiusMeters: apoTargetRadiusM);
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
            FlybyDeparture: departureKey);
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
            };
        }

        // Apoapsis defines this departure. Hyperbolic excess is unused.
        if (!(f.PlannerApoTargetMeters > 0.0)) { failed = true; return center; }
        return center with
        {
            TFinal = f.BurnTime,
            DFinalVlf = f.DvVlf,
            ApoTargetRadiusMeters = f.PlannerApoTargetMeters,
        };
    }

    private static void DrawFlybyMultipassNoteIfApplicable()
    {
        if (!HohmannFlybyUI.FlybyRequested || !_flybyRetargetFailed) return;
        ImGui.Spacing();
        ConsoleUi.WarningWrapped("Flyby retarget failed for this multi-pass geometry; the split would still impact. Reduce passes, change the window, or flip the side.");
    }

    #endregion
}
