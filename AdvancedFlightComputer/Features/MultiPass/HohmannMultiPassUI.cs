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

/// <summary>
/// Inline multi-pass section drawn INSIDE the stock Transfer Planning
/// window. Invoked via the IL transpiler in
/// <see cref="ManeuverTools.Patch_DrawPlanWindow_HohmannMultiPass"/>,
/// which injects a call to <see cref="DrawInline"/> right before stock's
/// own <c>DrawCorrectionTransfer</c> call so our section appears between
/// the Create button and the correction-burn / porkchop UI.
///
/// Three states:
///   * No active execution, N == 1: short hint pointing the user to
///     stock's Create button.
///   * No active execution, N >= 2: pass count, split mode, advisory,
///     pass list, "Create Multi-Pass" button.
///   * Active execution: "Pass X of N" status, "Cancel remaining" button.
///
/// Holds its own state (pass count, split mode, cached preview) so the
/// stock TransferPlanner state machine and our preview machinery don't
/// fight over <see cref="MultiPassUI"/>'s globals.
/// </summary>
internal static class HohmannMultiPassUI
{
    private const int MinPasses = 1;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly ImColor8 StatusGrey = new(120, 120, 120, 255);
    private static readonly ImColor8 ColorAmber = new(255, 200, 60, 255);
    private static readonly ImColor8 ColorOrange = new(255, 150, 50, 255);
    private static readonly float[] SingleColumnWidths = new float[] { 0.9f };

    public static bool Enabled { get; set; }

    private static int _passCount = 1;
    // EqualBurnTime is the literature-standard default for finite-burn
    // loss across N periapsis kicks (equal arc length per pass).
    // EqualDv is the simpler "uniform per-pass dV" alternative.
    private static SplitMode _splitMode = SplitMode.EqualBurnTime;

    // Cache key components - quantized so per-frame floating drift on
    // T_final / dV magnitude does not bust the cache. SplitMode is part of
    // the key because it drives per-pass dV through the real-K schedule
    // (Splitter.Allocate). StartPassIndex distinguishes init-phase
    // (always 0) from active-exec (= exec.PassIndex), so a pass completion
    // naturally busts the cache.
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
        FlybySide FlybySide);

    private static PreviewKey _cachedKey;
    private static bool _hasCachedPreview;
    private static PassPreviewResult _cachedPreview;
    private static int _autoClampedFromN;     // 0 = no clamp; >0 = user asked for this, got LargestFeasibleN
    // Failure reason + classifier at the requested N, captured during the
    // clamp probe. _autoClampKind drives the banner's advice switch
    // (PassPlanFailure is stable across reason-text edits); _autoClampReason
    // is shown to the user in debug builds for diagnostics only.
    private static string? _autoClampReason;
    private static PassPlanFailure _autoClampKind;
    // K_shift applied to fit the K-schedule for same-parent moon transfers
    // (LEO -> Luna, etc.). 0 = no shift applied (porkchop entry's TFinal was
    // already feasible). Surfaced in the UI so the user understands why
    // the multi-pass extends past the porkchop-selected cell.
    private static int _lastShiftKShift;
    private static double _cachedFuelSum = double.NaN;
    private static double _cachedFuelTotalDv = double.NaN;
    private static string? _lastSourceId;
    private static string? _lastTargetId;

    // True when the flyby option is on but the multi-pass retarget could not be
    // applied to the current geometry, so the split would depart center-aimed.
    // Surfaced as a warning so the user does not silently get an impact plan.
    private static bool _flybyRetargetFailed;

    // Per-frame dedup: two transpiler injections call DrawInline in
    // the same frame (see the DrawInline doc). First caller claims
    // the frame via ImGui.GetFrameCount, second short-circuits.
    private static int _lastFrameDrawn = -1;

    /// <summary>
    /// Inline-drawn into stock's "Transfer Planning" window by two
    /// transpilers (<see cref="ManeuverTools.Patch_DrawPlanWindow_HohmannMultiPass"/>
    /// and <see cref="Patch_DrawPlanWindow_HohmannFallback"/>). Both call
    /// sites land inside stock's outer Begin/End scope, so no window
    /// management here - just draw the section content. Per-frame dedup
    /// via <see cref="_lastFrameDrawn"/> keeps a single call wherever it
    /// fires first.
    /// </summary>
    public static void DrawInline()
    {
        if (!Enabled) return;
        int frame = ImGui.GetFrameCount();
        if (frame == _lastFrameDrawn) return;
        _lastFrameDrawn = frame;

        // Active-exec fast path: covers window-close/reopen during a
        // multi-pass run, where stock clears _selectedEntry and ShouldDraw
        // would early-return. Status + Cancel must still appear so the
        // user can see the run progress and cancel without re-Calculating.
        if (TryDrawActiveFastPath()) return;

        if (!ShouldDraw(out Vehicle? source, out OrbitalTransfers.PorkChopEntry? entry,
                       out OrbitalTransfers.TransferInfo? info))
            return;

        // Reset on source / target change so a stale preview from another
        // run doesn't render against the new geometry.
        string targetId = (info!.Target as Astronomical)?.Id ?? string.Empty;
        if (_lastSourceId != source!.Id || _lastTargetId != targetId)
        {
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(string.Format(Inv,
                    "[AFC] HohmannMultiPassUI: source/target change reset " +
                    "(was source='{0}' target='{1}', now source='{2}' target='{3}'); " +
                    "_passCount {4} -> 1",
                    _lastSourceId ?? "-", _lastTargetId ?? "-",
                    source.Id, targetId, _passCount));
            _lastSourceId = source.Id;
            _lastTargetId = targetId;
            _passCount = 1;
            _splitMode = SplitMode.EqualBurnTime;
            _hasCachedPreview = false;
            _cachedFuelSum = double.NaN;
            _cachedFuelTotalDv = double.NaN;
            _autoClampedFromN = 0;
            _autoClampReason = null;
            _autoClampKind = PassPlanFailure.None;
            _lastShiftKShift = 0;
        }

        try
        {
            ImGui.Separator();
            DrawBody(source, entry!, info!);
            ImGui.Separator();
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] HohmannMultiPassUI.DrawInline: {ex}");
        }
    }

    /// <summary>Returns true if active Hohmann exec was detected and the
    /// status / Cancel section was drawn. Resolves source via stock's
    /// <c>_sourceBody</c> only - independent of <c>_selectedEntry</c> -
    /// so the active-pass UI survives a Transfer Planning window close
    /// followed by a reopen (which clears _selectedEntry).</summary>
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
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannMultiPassUI.DrawInline (active fast path): {ex}");
        }
        ImGui.Separator();
        return true;
    }

    public static void Reset()
    {
        _passCount = 1;
        _splitMode = SplitMode.EqualBurnTime;
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
        _lastSourceId = null;
        _lastTargetId = null;
        _lastFrameDrawn = -1;
        HohmannMultiPassPlanner.ResetShiftCache();
    }

    /// <summary>Drops the cached preview when a Hohmann multi-pass exec
    /// ends on <paramref name="vehicleId"/>. Without this the cache
    /// outlives the registry entry: HasMultiPassPreview's PassCount > 1
    /// clause then keeps the OnPreRender overlay alive against whatever
    /// orbit the vehicle is on post-exec, until the init-flow's
    /// UpdatePreviewIfStale happens to bust the key.</summary>
    public static void OnExecutionEnded(string vehicleId)
    {
        if (!_hasCachedPreview) return;
        if (_cachedKey.SourceId != vehicleId) return;
        _hasCachedPreview = false;
        _cachedPreview = default;
        _cachedKey = default;
        _cachedFuelSum = double.NaN;
        _cachedFuelTotalDv = double.NaN;
        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] HohmannMultiPassUI.OnExecutionEnded: vehicle='{vehicleId}' " +
                "cleared cached preview.");
    }

    /// <summary>True when a successful multi-pass plan is cached and
    /// either the user picked N>1 in the init UI or the cached plan
    /// came from an active-exec recompute. The "same vehicle" check is
    /// enforced downstream in <see cref="RenderOrbits"/> /
    /// <see cref="RenderMarkers"/> via the cache's SourceId, so a stale
    /// cache for a vehicle that is no longer the source still returns
    /// true here but is filtered out at render time.</summary>
    public static bool HasMultiPassPreview
    {
        get
        {
            if (!Enabled || !_hasCachedPreview || _cachedPreview.Failed) return false;
            if (_cachedPreview.Passes.Length == 0) return false;
            // _cachedKey.PassCount > 1 covers the source-switch case during
            // active execution: switching the dropdown to another vehicle
            // and back resets _passCount to 1, but DrawActive rebuilds the
            // cache with PassCount = exec.PassCountTotal. Without this
            // clause the overlay would silently stop rendering after a
            // dropdown round-trip while the multi-pass is still running.
            return _passCount > 1 || _cachedKey.StartPassIndex > 0
                   || _cachedKey.PassCount > 1;
        }
    }

    /// <summary>3D orbit lines for the cached multi-pass preview. During
    /// active execution the first cached pass is the queued one and
    /// stock renders it via BurnPlan, so we skipFirst to avoid drawing
    /// it twice. skipLast tracks whether stock's selected-entry overlay
    /// will own the final-pass Lambert trajectory: that overlay is
    /// gated on <c>_transferCalculated</c>, which is false after F4
    /// close+reopen of the plan window; in that state we must draw the
    /// final pass ourselves or it disappears entirely (visible at low
    /// N=2,3 where skipFirst already eats the queued pass).</summary>
    public static void RenderOrbits(Viewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source.Id != _cachedKey.SourceId) return;

        bool skipFirst = MultiPassRegistry.Has(source.Id);
        bool stockOwnsFinal = StockSelectedTransferOverlayActive();
        MultiPassRenderer.RenderPassOrbits(
            viewport, source, _cachedPreview.Passes,
            skipFirst, skipLast: stockOwnsFinal);
    }

    /// <summary>Per-pass Ap/Pe/AN/DN/SOI/closest markers. ImGui-phase
    /// counterpart of <see cref="RenderOrbits"/>. firstPassDisplayNumber
    /// equals <c>exec.PassIndex + 1</c> during execution so labels reflect
    /// the absolute pass number in the original N-pass sequence
    /// (e.g. after pass 1 completes, the intermediate triangle still
    /// reads "Ap Pass 3" rather than restarting at 1). skipLast tracks
    /// stock's selected-transfer-marker availability (gated on
    /// <c>_transferCalculated</c>) for the same reason as
    /// <see cref="RenderOrbits"/>.</summary>
    public static void RenderMarkers(Viewport viewport, Vehicle source)
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

    /// <summary>True iff stock's selected-transfer overlay (3D lines
    /// + markers via <c>DrawSelectedTransfer</c> /
    /// <c>DrawSelectedTransferUi</c>) is going to render the final
    /// pass for us this frame. Drives the skipLast decision in
    /// <see cref="RenderOrbits"/> and <see cref="RenderMarkers"/>.
    /// Mirrors stock's own gate
    /// <c>ShowPlanWindow &amp;&amp; _displaySelectedTransfer
    /// &amp;&amp; _transferCalculated</c> from
    /// <see cref="TransferPlanner.OnPreRender"/>; the _displaySelected
    /// check is redundant with our render-path gating but explicit
    /// here so the helper is self-contained.</summary>
    private static bool StockSelectedTransferOverlayActive()
    {
        // With a flyby armed the flyby patch hides stock's overlay (it shows the
        // center-aimed impact), so the final pass has to be drawn here instead of
        // being delegated to stock.
        if (HohmannFlybyUI.FlybyRequested) return false;
        return StockPlanner.ShowPlanWindow
               && StockPlanner.DisplaySelectedTransfer
               && StockPlanner.TransferCalculated;
    }

    /// <summary>Shared gate for the 3D orbit overlay and the per-pass
    /// marker overlay (both <see cref="Patch_TransferPlanner_OnPreRender_Hohmann"/>
    /// and <see cref="Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers"/>).
    ///
    /// An active multi-pass exec is authoritative: we render based
    /// on the cached preview regardless of stock's
    /// <c>_transferCalculated</c> flag, because F4 close+reopen
    /// clears the flag while the exec stays alive. Without an
    /// active exec we require <c>_transferCalculated</c> so we do
    /// not paint stale init-cache orbits after the user changes
    /// source / destination / type (all of which stock clears the
    /// flag for, but none of which invalidate our preview
    /// cache).</summary>
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

    /// <summary>Whether the in-stock-window Hohmann state currently
    /// supports a multi-pass preview. False means draw nothing.</summary>
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

            // Gates out stale _selectedEntry that stock keeps across
            // source/destination changes (only nulls it on full window
            // close via ShowPlanWindow.set(false)). Without this check
            // we'd render with the previous target's entry data
            // against the new target's transferInfo.
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
        // Active execution branch: show status + Cancel for our Hohmann
        // intent, or a "blocked by other multi-pass" notice if a
        // different intent (apse, inclination, circularize) is already
        // running on this vehicle. Without the second arm, clicking
        // Create here would overwrite the other intent in the registry
        // and orphan its pending burn.
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
        {
            if (exec.Intent is HohmannTransferIntent)
                DrawActive(source, exec);
            else
                DrawBlockedByOtherExecution(exec);
            return;
        }

        ImGui.TextWrapped(string.Format(Inv,
            "Departure to {0}", (info.Target as Astronomical)?.Id ?? "?"));
        ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
        ImGui.TextWrapped(string.Format(Inv,
            "Lambert dV: {0:F1} m/s", entry.TransferData.TransferDvVlf.Length()));
        ImGui.PopStyleColor();

        // Flyby targeting sits between the transfer summary and the pass-count
        // splitting: it changes WHERE the departure aims (impact -> flyby), then
        // multi-pass optionally splits that same departure into perigee kicks.
        HohmannFlybyUI.DrawInline(source, entry, info);

        ImGui.Spacing();

        DrawPassCountSelector(source, entry, info);

        if (_passCount > 1)
        {
            DrawSplitModeRadio();
            UpdatePreviewIfStale(source, entry, info);
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

        // Auto-clamp banner stays visible AFTER the clamp drops _passCount
        // back to 1: the user needs to see why their > click had no
        // visible effect on the pass count. The < / > clicks explicitly
        // reset _autoClampedFromN, which is the only way to clear it.
        DrawAutoClampIfApplicable();

        bool flyby = HohmannFlybyUI.FlybyRequested;
        if (_passCount <= 1 && _autoClampedFromN <= _passCount)
        {
            ImGui.Spacing();
            ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
            ImGui.TextWrapped(flyby
                ? "N = 1: stock Create button does a single flyby burn."
                : "N = 1: stock Create button does a single Hohmann burn.");
            ImGui.PopStyleColor();
        }
        else if (_passCount > 1)
        {
            ImGui.Spacing();
            ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
            ImGui.TextWrapped(string.Format(Inv,
                flyby
                    ? "Click the stock Create button to start the {0}-pass flyby execution."
                    : "Click the stock Create button to start the {0}-pass execution.",
                _passCount));
            ImGui.PopStyleColor();
        }
    }

    /// <summary>"This multi-pass occupies X parking periods of warning
    /// time" - helps the user understand that picking N=4 isn't free,
    /// they're committing to a longer departure window. When the porkchop
    /// entry's start was too early to fit the K-schedule (same-parent
    /// moons) and we auto-shifted T_final forward, a second line surfaces
    /// the shift so the user understands why the final burn happens past
    /// the selected porkchop cell.
    ///
    /// Span is read from the cached preview's actual first/last burn
    /// times so it matches the SplitMode-derived real-K schedule rather
    /// than a closed-form integer estimate.</summary>
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

        ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
        ImGui.TextWrapped(string.Format(Inv,
            "Span: {0:F0} parking periods (~{1})",
            spanSec / tPark, FormatHelper.FormatDuration(spanSec)));
        if (_lastShiftKShift > 0)
        {
            double shiftSec = _lastShiftKShift * tPark;
            ImGui.TextWrapped(string.Format(Inv,
                "Final burn pushed {0} parking period(s) (~{1}) later so the " +
                "multi-pass schedule fits; transfer re-planned at the later time.",
                _lastShiftKShift, FormatHelper.FormatDuration(shiftSec)));
        }
        ImGui.PopStyleColor();
    }

    /// <summary>SplitMode selector. EqualBurnTime is the literature-standard
    /// default for finite-burn loss (each pass burns the engines for the
    /// same duration); EqualDv is the simpler "uniform per-pass dV"
    /// alternative. Drawn only when N &gt;= 2; mode change busts cached
    /// preview.</summary>
    private static void DrawSplitModeRadio()
    {
        ImGui.Spacing();
        bool isBurnTime = _splitMode == SplitMode.EqualBurnTime;
        if (ImGui.RadioButton("Equal Burn Time"u8, isBurnTime) && !isBurnTime)
        {
            _splitMode = SplitMode.EqualBurnTime;
            _hasCachedPreview = false;
            _cachedFuelSum = double.NaN;
            _cachedFuelTotalDv = double.NaN;
            _autoClampedFromN = 0;
            _autoClampReason = null;
            _autoClampKind = PassPlanFailure.None;
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    "[AFC] HohmannMultiPassUI: split mode -> EqualBurnTime");
        }
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(
                "Each pass fires the engines for the same duration.\nEqualises finite-burn arc length across passes\n(literature-standard default for finite-burn loss)."u8);

        ImGui.SameLine();
        bool isEqualDv = _splitMode == SplitMode.EqualDv;
        if (ImGui.RadioButton("Equal Delta-V"u8, isEqualDv) && !isEqualDv)
        {
            _splitMode = SplitMode.EqualDv;
            _hasCachedPreview = false;
            _cachedFuelSum = double.NaN;
            _cachedFuelTotalDv = double.NaN;
            _autoClampedFromN = 0;
            _autoClampReason = null;
            _autoClampKind = PassPlanFailure.None;
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    "[AFC] HohmannMultiPassUI: split mode -> EqualDv");
        }
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(
                "Each pass delivers the same delta-v magnitude.\nSimpler to reason about but slightly less efficient\nfor finite burns than Equal Burn Time."u8);
    }

    #region Active execution

    private static void DrawActive(Vehicle source, MultiPassExecution exec)
    {
        ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
        ImGui.Text(string.Format(Inv,
            "Multi-pass active: pass {0} of {1}",
            exec.PassIndex + 1, exec.PassCountTotal));
        ImGui.PopStyleColor();

        // Refresh preview from the locked intent state so the 3D overlay
        // reflects the chained orbit (post-prior-passes) rather than the
        // parking-orbit shape it had at init time. Cache key includes
        // exec.PassIndex so cache busts on pass completion.
        if (exec.Intent is HohmannTransferIntent intent)
            UpdatePreviewForActiveExec(source, exec, intent);

        // List remaining passes with planned dV / burn time. Cache
        // built from Plan(startPassIndex=exec.PassIndex), so passes[0]
        // is the currently queued pass; firstPassDisplayNumber maps
        // index 0 to the absolute pass number (PassIndex+1).
        DrawPassList(firstPassDisplayNumber: exec.PassIndex + 1);

        // Surface mid-exec planner failures (e.g. live-state drift makes
        // v_p_target unreachable) so the user understands why the orbit
        // overlay disappeared instead of seeing it silently vanish.
        DrawPreviewFailureIfApplicable();
        DrawAdvisoryIfApplicable();

        // Mirror stock's "Preview Selected Transfer" checkbox here
        // when stock's own copy is hidden (i.e., _transferCalculated
        // is false after F4 close+reopen). Without this the user has
        // no way to re-enable the 3D overlay without re-running
        // Calculate. Skipped when stock's checkbox is accessible to
        // avoid two coupled checkboxes that show the same state.
        if (!StockPlanner.TransferCalculated)
            DrawInlinePreviewToggle();

        ImGui.Spacing();
        if (ImGuiHelper.DrawButton("Cancel remaining passes"u8,
                KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Red))
            CancelExecution(source, exec);
    }

    /// <summary>Reads + writes stock's <c>_displaySelectedTransfer</c>
    /// field via reflection so toggling our copy stays in sync with
    /// any future user interaction with stock's own checkbox (when
    /// stock's section is visible again). Drawn only when stock's
    /// section is hidden; see the caller in <see cref="DrawActive"/>.
    /// Wrapped in BeginColumns / EndColumns because
    /// <see cref="ImGuiHelper.DrawCheckbox"/> needs that for the label
    /// to render inline with the box; without columns the label drops
    /// onto a second line.</summary>
    private static void DrawInlinePreviewToggle()
    {
        ImGui.Spacing();
        bool preview = StockPlanner.DisplaySelectedTransfer;
        bool previous = preview;
        ImGuiHelper.BeginColumns(2, SingleColumnWidths);
        ImGuiHelper.DrawCheckbox("Preview Selected Transfer"u8, ref preview, isChanged: false);
        ImGuiHelper.EndColumns();
        if (preview != previous)
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
            TFinal: new SimTime(intent.TFinalSec),
            DFinalVlf: intent.DFinalVlf,
            IsCrossParent: intent.IsCrossParent,
            VInfMs: intent.VInfMs,
            ApoTargetRadiusMeters: intent.ApoTargetRadiusMeters);

        var key = BuildKey(source, input,
            passCount: exec.PassCountTotal,
            mode: exec.Mode,
            startPassIndex: exec.PassIndex);
        if (_hasCachedPreview && key == _cachedKey) return;

        // Freeze during Auto burn so per-tick mass drift does not
        // recompute every physics tick. Pass completion increments
        // exec.PassIndex (part of the cache key via StartPassIndex),
        // busting the cache on the next frame. Mode transitions alone
        // do not change the key.
        if (_hasCachedPreview
            && source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto)
            return;

        SimTime now = Universe.GetElapsedSimTime();
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        // No PrepareShiftedInput / ScanAdvisory merge here: the shift was
        // applied at intent creation and the input's TFinal is locked.
        // Plan's own CheckFinalPassAdvisory still surfaces a live final-pass
        // impact on _cachedPreview.Advisory.
        _cachedPreview = HohmannMultiPassPlanner.Plan(
            source, input, exec.PassCountTotal, exec.PassIndex,
            intent.ParkingPeriodSec, state, now, exec.Mode);
        _cachedKey = key;
        _hasCachedPreview = true;

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewForActiveExec: vehicle='{0}' " +
                "passIndex={1}/{2} -> failed={3} reason='{4}' advisory='{5}' previewPasses={6}",
                source.Id, exec.PassIndex, exec.PassCountTotal,
                _cachedPreview.Failed, _cachedPreview.FailureReason ?? "-",
                _cachedPreview.Advisory ?? "-",
                _cachedPreview.Passes.Length));
    }

    private static void DrawBlockedByOtherExecution(MultiPassExecution exec)
    {
        ImGui.PushStyleColor(ImGuiCol.Text, ColorAmber);
        ImGui.TextWrapped(string.Format(Inv,
            "Vehicle is already running a {0} multi-pass ({1} of {2}).\n" +
            "Cancel it from its own plan window before starting a Hohmann.",
            exec.Intent.Kind, exec.PassIndex + 1, exec.PassCountTotal));
        ImGui.PopStyleColor();
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
        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] HohmannMultiPass: vehicle={source.Id} user cancelled at pass " +
                $"{exec.PassIndex + 1}/{exec.PassCountTotal}.");
        MultiPassRegistry.Remove(source.Id);
        PassCompletionPatch.OnRegistryRemovedExternally(source.Id);
        OnExecutionEnded(source.Id);
    }

    #endregion

    #region Pass count / split mode / preview

    private static void DrawPassCountSelector(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        ImGui.Text("Passes:"u8);
        ImGui.SameLine();
        if (ImGuiHelper.DrawButton("<"u8, KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (_passCount > MinPasses)
            {
                int before = _passCount;
                _passCount--;
                _hasCachedPreview = false;
                _cachedFuelSum = double.NaN;
                _cachedFuelTotalDv = double.NaN;
                _autoClampedFromN = 0;
                _autoClampReason = null;
                _autoClampKind = PassPlanFailure.None;
                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] HohmannMultiPassUI: < clicked, _passCount {before} -> {_passCount}.");
            }
        }
        ImGui.SameLine();
        ImGui.Text(_passCount.ToString(Inv));
        ImGui.SameLine();
        if (ImGuiHelper.DrawButton(">"u8, KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (_passCount < Splitter.MaxPasses)
            {
                int before = _passCount;
                _passCount++;
                _hasCachedPreview = false;
                _cachedFuelSum = double.NaN;
                _cachedFuelTotalDv = double.NaN;
                _autoClampedFromN = 0;
                _autoClampReason = null;
                _autoClampKind = PassPlanFailure.None;
                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] HohmannMultiPassUI: > clicked, _passCount {before} -> {_passCount}.");
            }
        }
    }

    private static void UpdatePreviewIfStale(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        SimTime now = Universe.GetElapsedSimTime();
        double parkingPeriodSec = source.Orbit?.Period ?? double.NaN;

        // Vehicle is no longer in a bound orbit (e.g. just completed a
        // multi-pass to Mars and is now hyperbolic; Orbit.Period returns
        // NaN). Force N=1 and bail before LargestFeasibleN runs probes
        // that would all fail with "parkingPeriodSec NaN" - those produce
        // confusing log spam plus an auto-clamp banner that misleads the
        // user about why multi-pass isn't available.
        if (!(parkingPeriodSec > 0.0))
        {
            if (_passCount > 1)
            {
                _passCount = 1;
                _autoClampedFromN = 0;
                _autoClampReason = null;
                _autoClampKind = PassPlanFailure.None;
            }
            _lastShiftKShift = 0;
            _hasCachedPreview = false;
            _cachedFuelSum = double.NaN;
            _cachedFuelTotalDv = double.NaN;
            _cachedPreview = default;
            _cachedKey = default;
            return;
        }

        var raw = BuildBasePlanInput(source, entry, info);
        // Through the shared cache: Analyze now runs the game's own drain
        // simulation, far too heavy for this per-frame pre-key-check site.
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        var shift = HohmannMultiPassPlanner.PrepareShiftedInput(
            raw, source, info, _passCount, parkingPeriodSec, now, _splitMode, state);

        var key = BuildKey(source, shift.Input,
            passCount: _passCount, mode: _splitMode, startPassIndex: 0);
        if (_hasCachedPreview && key == _cachedKey)
        {
            // PrepareShiftedInput's own cache made the call cheap, but the
            // planner-cache hit means we also avoid the Plan call below.
            _lastShiftKShift = shift.KShift;
            return;
        }

        // Freeze the cache during an Auto burn so mid-burn mass drift
        // does not recompute every physics tick. Accumulated drift in
        // quantized key fields (MassBucket etc.) busts the cache once
        // the freeze lifts; the mode transition itself is not in the key.
        if (_hasCachedPreview
            && source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto)
            return;

        int requestedN = _passCount;
        _lastShiftKShift = shift.KShift;

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewIfStale: vehicle='{0}' target='{1}' " +
                "requestedN={2} isCrossParent={3} vInf={4:F1}m/s apoTarget={5:F0}m " +
                "T_final={6:F0}s rawTFinal={7:F0}s K_shift={8} now={9:F0}s T_park={10:F1}s",
                source.Id, (info.Target as Astronomical)?.Id ?? "?",
                requestedN, shift.Input.IsCrossParent,
                shift.Input.VInfMs, shift.Input.ApoTargetRadiusMeters,
                shift.Input.TFinal.Seconds(), raw.TFinal.Seconds(),
                shift.KShift, now.Seconds(),
                parkingPeriodSec));

        int clampedN = HohmannMultiPassPlanner.LargestFeasibleN(
            source, shift.Input, state, parkingPeriodSec, now, requestedN, _splitMode,
            out string? clampReason, out PassPlanFailure clampKind);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewIfStale: LargestFeasibleN " +
                "requested={0} -> clamped={1} mode={2} kind={3} reason='{4}'",
                requestedN, clampedN, _splitMode, clampKind, clampReason ?? "-"));

        var planInput = shift.Input;
        if (clampedN < requestedN)
        {
            // _autoClampedFromN is sticky: clearing it here would reset
            // the banner on the next frame because requestedN already
            // equals clampedN (we wrote _passCount = clampedN). The < / >
            // buttons explicitly reset _autoClampedFromN, which is the
            // only way back to "no clamp warning".
            _autoClampedFromN = Math.Max(_autoClampedFromN, requestedN);
            _autoClampReason = clampReason;
            _autoClampKind = clampKind;
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(string.Format(Inv,
                    "[AFC] HohmannMultiPassUI.UpdatePreviewIfStale: AUTO-CLAMP " +
                    "_passCount {0} -> {1}, _autoClampedFromN={2}",
                    _passCount, clampedN, _autoClampedFromN));
            _passCount = clampedN;
            // Re-shift for the clamped passCount: a smaller N has a smaller
            // K_total, so the required shift shrinks. Without this we'd
            // over-shift the final plan by the difference between requestedN's
            // K_total and clampedN's K_total parking periods - feasible but
            // unnecessarily late.
            shift = HohmannMultiPassPlanner.PrepareShiftedInput(
                raw, source, info, _passCount, parkingPeriodSec, now, _splitMode, state);
            planInput = shift.Input;
            _lastShiftKShift = shift.KShift;
            // BuildKey depends on planInput.TFinal (bucketed); recompute so
            // the cached key matches the actual input we're planning with,
            // otherwise the next frame would bust the cache on a phantom delta.
            key = BuildKey(source, planInput,
                passCount: _passCount, mode: _splitMode, startPassIndex: 0);
        }

        // Bake the flyby offset into the (center-aimed) plan input so the
        // multi-pass split departs toward the flyby, not the impact. The retarget
        // failure is stored for DrawFlybyMultipassNoteIfApplicable to warn.
        planInput = MaybeApplyFlyby(
            source, info, planInput, ResolveFlybyTransit(shift, entry), out _flybyRetargetFailed);

        _cachedPreview = HohmannMultiPassPlanner.Plan(
            source, planInput, _passCount, startPassIndex: 0,
            parkingPeriodSec, state, now, _splitMode);
        // Merge the shifted-Lambert scan's advisory into the Plan result.
        // Plan's own advisory (chained final-pass FP) is more accurate than
        // the scan signal (single-burn FP at shifted geometry), so prefer it
        // when both fire; only surface the scan-only signal when Plan came
        // back clean.
        if (shift.ScanAdvisory != null && _cachedPreview.Advisory == null)
            _cachedPreview = _cachedPreview with { Advisory = shift.ScanAdvisory };
        _cachedKey = key;
        _hasCachedPreview = true;

        double fuelCheckDv = planInput.DFinalVlf.Length();
        if (fuelCheckDv > 0.0 && state.HasUsableEngines)
        {
            _cachedFuelSum = Splitter.SumDvCapacityMs(
                Splitter.Allocate(fuelCheckDv, _passCount, _splitMode, state));
            _cachedFuelTotalDv = fuelCheckDv;
        }
        else
        {
            _cachedFuelSum = double.NaN;
            _cachedFuelTotalDv = double.NaN;
        }

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewIfStale: Plan -> failed={0} " +
                "reason='{1}' advisory='{2}' previewPasses={3} _passCount(final)={4} " +
                "K_shift(final)={5}",
                _cachedPreview.Failed, _cachedPreview.FailureReason ?? "-",
                _cachedPreview.Advisory ?? "-",
                _cachedPreview.Passes.Length, _passCount, _lastShiftKShift));
    }

    private static void DrawPreviewFailureIfApplicable()
    {
        if (!_hasCachedPreview || !_cachedPreview.Failed) return;
        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ColorOrange);
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Multi-pass preview incomplete: {0}.",
            _cachedPreview.FailureReason ?? "unknown reason"));
        ImGui.PopStyleColor();
    }

    /// <summary>Soft warning when the final-pass FP (or the shifted-Lambert
    /// scan it was built from) intersects the parent body or crosses an
    /// unintended SOI. Stock's porkchop reject filter would have hidden
    /// this entry; we surface it instead of refusing to plan, so the user
    /// can inspect the previewed trajectory and decide whether to commit.
    /// Amber, less severe than the orange failure banner.</summary>
    private static void DrawAdvisoryIfApplicable()
    {
        if (!_hasCachedPreview || _cachedPreview.Failed) return;
        if (_cachedPreview.Advisory == null) return;
        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ColorAmber);
        ImGui.TextWrapped(string.Format(Inv,
            "[!] {0}", _cachedPreview.Advisory));
        ImGui.PopStyleColor();
    }

    private static void DrawInsufficientFuelIfApplicable()
    {
        if (_hasCachedPreview && _cachedPreview.Failed) return;
        if (double.IsNaN(_cachedFuelSum) || double.IsNaN(_cachedFuelTotalDv)) return;
        if (!(_cachedFuelTotalDv > 0.0)) return;
        if (_cachedFuelSum >= _cachedFuelTotalDv * 0.995) return;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ColorOrange);
        ImGui.TextWrapped(string.Format(Inv,
            "[!] Vehicle can only deliver ~{0:F0} m/s of the {1:F0} m/s required.\n" +
            "Multi-pass will run out of fuel before the departure is reached.",
            _cachedFuelSum, _cachedFuelTotalDv));
        ImGui.PopStyleColor();
    }

    private static void DrawAutoClampIfApplicable()
    {
        if (_autoClampedFromN <= _passCount) return;
        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ColorAmber);

        // PassPlanFailure-based advice. Driven by the planner's classifier
        // (PassPlanFailure) instead of substring-matching the human-
        // readable reason; that way reason-text edits don't silently
        // break the advice routing. Mode-aware: only suggest the OTHER
        // SplitMode (suggesting the user's current mode is a no-op).
        SplitMode otherMode = _splitMode == SplitMode.EqualBurnTime
            ? SplitMode.EqualDv
            : SplitMode.EqualBurnTime;
        string otherModeLabel = otherMode == SplitMode.EqualBurnTime
            ? "Equal Burn Time"
            : "Equal Delta-V";

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

        ImGui.TextWrapped(string.Format(Inv,
            "[!] {0} pass(es) requested, only {1} feasible at this departure entry.\n{2}",
            _autoClampedFromN, _passCount, advice));
        if (DebugConfig.MultiPass && _autoClampReason != null)
            ImGui.TextWrapped(string.Format(Inv,
                "Debug: kind={0}, reason: {1}", _autoClampKind, _autoClampReason));
        ImGui.PopStyleColor();
    }

    private static void DrawPassList(int firstPassDisplayNumber = 1)
    {
        if (!_hasCachedPreview) return;
        PassPreview[] passes = _cachedPreview.Passes;
        if (passes.Length == 0) return;

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        for (int i = 0; i < passes.Length; i++)
        {
            double dv = passes[i].DvVlf.Length();
            double t = passes[i].EstimatedBurnTimeSec;
            int n = firstPassDisplayNumber + i;
            string suffix = i == passes.Length - 1 ? " (final)" : "";
            string line = t > 0.5
                ? string.Format(Inv, "Pass {0}: {1:F0} m/s, {2:F0}s{3}", n, dv, t, suffix)
                : string.Format(Inv, "Pass {0}: {1:F0} m/s{2}", n, dv, suffix);
            ImGui.Text(line);
        }
        ImGui.PopStyleColor();
    }

    /// <summary>One line under the per-pass list - "Total: X m/s |
    /// Lambert: Y m/s" - plus an optional Robbins savings estimate
    /// when the comparison is meaningful. The total can exceed Lambert
    /// when the planner caps priors at the SOI envelope: extra dV gets
    /// dumped into a "fast escape but capped" intermediate orbit, the
    /// final pass adds the hyperbolic excess on top, and the trade is
    /// smaller finite-burn loss at the final pass. We surface that as
    /// a neutral comparison instead of framing it as "savings". The
    /// savings line itself goes via <see cref="TryFormatRobbinsSavings"/>,
    /// which gates on shift-state and fuel-state to avoid printing a
    /// misleading number.</summary>
    private static void DrawTotalsAndSavings(
        Vehicle source, PassPreview[] passes, double lambertDv, double tPark)
    {
        if (passes.Length == 0 || !(lambertDv > 0.0)) return;

        double sumDv = 0.0;
        for (int i = 0; i < passes.Length; i++)
            sumDv += passes[i].DvVlf.Length();

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
        ImGui.Text(string.Format(Inv,
            "Total: {0:F0} m/s | Lambert: {1:F0} m/s", sumDv, lambertDv));
        string? savingsLine = TryFormatRobbinsSavings(source, passes, lambertDv, tPark);
        if (savingsLine != null) ImGui.Text(savingsLine);
        ImGui.PopStyleColor();
    }

    /// <summary>Returns the formatted savings line, or null when the
    /// comparison is not meaningful for the current state.
    ///
    /// Skipped when <c>_lastShiftKShift &gt; 0</c>: PrepareShiftedInput
    /// re-solved Lambert at shifted geometry for same-parent moon
    /// transfers, so the multi-pass total is against a different transit
    /// than the user would have flown with stock's N=1 click. The
    /// outer "Total | Lambert" line is still meaningful as "what stock
    /// would burn vs what multi-pass burns" but the per-pass loss diff
    /// then conflates Oberth savings with geometry-change costs and
    /// stops being a clean savings claim.
    ///
    /// Approximation note: <paramref name="tPark"/> is used as the
    /// equivalent period for every pass. Chained orbits for pass k &gt; 0
    /// have larger SMA but the same periapsis, so omega_periapsis_k
    /// &gt; omega_park and the true equivalent period is shorter than
    /// tPark. Using tPark uniformly slightly over-reports savings for
    /// chained passes; the "~" prefix is the user-facing acknowledgment.
    /// Exposing v_peri_k per pass for an accurate fix is overkill for
    /// a UI helper.</summary>
    private static string? TryFormatRobbinsSavings(
        Vehicle source, PassPreview[] passes, double lambertDv, double tPark)
    {
        if (_lastShiftKShift > 0) return null;
        if (!(tPark > 0.0)) return null;

        // When fuel-short, singleLoss uses lambertDv (unreachable) while
        // splitLoss uses the fuel-limited per-pass dV, inflating savings.
        if (!double.IsNaN(_cachedFuelSum) && !double.IsNaN(_cachedFuelTotalDv)
            && _cachedFuelTotalDv > 0.0 && _cachedFuelSum < _cachedFuelTotalDv * 0.995)
            return null;

        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        if (!state.HasUsableEngines) return null;

        // SplitMode irrelevant at passCount = 1 (Splitter collapses to a
        // single fuel-drain walk). EqualDv is the codebase convention for
        // "doesn't care" - matches MultiPassUI.EstimateBurnTime.
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

    /// <summary>True when the user has selected N>1 but the preview
    /// failed (so the interceptor will fall through to a single burn).
    /// Lets the interceptor surface an alert instead of silently giving
    /// the user something different from what they asked for.</summary>
    public static bool WantedMultiPassButPreviewFailed() =>
        Enabled && _passCount > 1 && _hasCachedPreview && _cachedPreview.Failed;

    /// <summary>The user has a split selected (N &gt; 1), regardless of whether it
    /// could be armed. Lets the create interceptor tell the user when it falls back
    /// to a single burn for a reason the preview banner does not cover (e.g. the
    /// click-time intent rebuild failing after a successful preview).</summary>
    public static bool WantsMultiPass => Enabled && _passCount > 1;

    /// <summary>
    /// True when the inline UI is armed for a Hohmann multi-pass on
    /// <paramref name="vehicle"/>: pass count > 1, a valid cached preview,
    /// and stock state (transfer type / selected porkchop entry / source
    /// vehicle) matches. Used by <see cref="HohmannCreateInterceptor"/>
    /// to decide whether stock's Create button should fire multi-pass
    /// or fall through to a single burn.
    /// </summary>
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

        SimTime now = Universe.GetElapsedSimTime();
        intent = BuildIntent(uiSource, entry!, info!, _passCount, now);
        if (intent == null) return false;

        passCount = _passCount;
        mode = _splitMode;
        return true;
    }

    #endregion

    #region Helpers

    /// <summary>Reads the raw HohmannPlanInput off a stock porkchop entry,
    /// no multi-pass feasibility shift applied. For N=1 this is the only
    /// thing we need; for N >= 2 the result is then passed through
    /// <see cref="HohmannMultiPassPlanner.PrepareShiftedInput"/>.</summary>
    private static HohmannMultiPassPlanner.HohmannPlanInput BuildBasePlanInput(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        // Cross-parent = vehicle's parent body differs from target's parent
        // body. Examples: LEO -> Mars (Earth vs Sun), Mars-orbit -> Phobos
        // is SAME (both Mars). We cannot use OrbitalTransfers.SameSoiTransfer
        // here: stock's TransferTask.Run rewrites info.Source to info.Vehicle
        // for same-SOI transfers, and re-calling SameSoiTransfer on the
        // post-rewrite info returns false for both Luna and Mars cases. The
        // direct parent-id compare is unambiguous and stable.
        string? targetParentId = info.Target?.Parent?.Id;
        string? sourceParentId = source.Orbit?.Parent?.Id;
        bool isCrossParent = sourceParentId == null
            || targetParentId == null
            || sourceParentId != targetParentId;
        double vInfMs = 0.0;
        double apoTargetRadiusM = 0.0;

        if (isCrossParent)
        {
            // Cross-parent Lambert: EjectionVelocityCci is the hyperbolic
            // excess in the parking-orbit-parent's CCI frame (magnitude
            // invariant under the cci2Cce / cce2Cci transform stock applies).
            vInfMs = entry.TransferData.EjectionVelocityCci.Length();
        }
        else
        {
            // Same-parent Lambert: read the post-burn orbit's apoapsis
            // off the stock-generated flight plan. Patches[0] is the
            // immediate post-burn ellipse around the same parent. NaN
            // when the stock entry's first patch is hyperbolic (rare for
            // same-SOI but possible with high-energy porkchop picks);
            // the planner's "unbound transfer orbit" check then surfaces
            // the issue instead of silently passing NaN through.
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
        int passCount, SimTime now)
    {
        if (source.Orbit?.Parent == null) return null;
        string targetId = (info.Target as Astronomical)?.Id ?? string.Empty;
        if (string.IsNullOrEmpty(targetId)) return null;

        // Lock the parking period at intent creation. RecomputePass uses
        // this to anchor the K-schedule; reading vehicle.Orbit.Period
        // later would return the chained orbit's period and break the
        // chain alignment.
        double parkingPeriod = source.Orbit.Period;
        if (!(parkingPeriod > 0.0)) return null;

        // Build raw then apply the same multi-pass feasibility shift that
        // UpdatePreviewIfStale uses for the preview. T_final / DFinalVlf
        // must be read from the shifted input (NOT entry directly) so the
        // intent locks in the shifted geometry, otherwise RecomputePass
        // would use the un-shifted T_final and the K-schedule fails on
        // the first pass. Mode-aware: the K-total required for the shift
        // depends on SplitMode (EqualBurnTime needs more parking periods
        // than EqualDv for the same N), so the intent must use the same
        // mode the user picked.
        var raw = BuildBasePlanInput(source, entry, info);
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(source);
        var shift = HohmannMultiPassPlanner.PrepareShiftedInput(
            raw, source, info, passCount, parkingPeriod, now, _splitMode, state);
        var input = shift.Input;

        // Bake the flyby offset into the locked input so RecomputePass splits the
        // flyby departure across passes. Uses the same transit as the preview path
        // (via ResolveFlybyTransit) or the created plan diverges from the preview.
        // Refuse to lock a center-aimed (impact) intent when the user armed the
        // flyby: better to fall back to the stock burn with an alert than to
        // silently start a multi-pass that impacts.
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
        int passCount, SplitMode mode, int startPassIndex)
    {
        // Flyby request is part of the key so toggling the flyby / its altitude /
        // side busts the cached (center-aimed vs offset) preview.
        bool flybyOn = false;
        long flybyRpBucket = 0;
        FlybySide flybySide = FlybySide.Inner;
        if (input.Target is IParentBody target
            && HohmannFlybyUI.TryGetRequest(target, out double rp, out FlybySide side))
        {
            flybyOn = true;
            flybyRpBucket = (long)(rp / 1000.0);
            flybySide = side;
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
            FlybySide: flybySide);
    }

    /// <summary>The transit that the flyby retarget must aim at: the shifted
    /// candidate's transit after a moon-shift, otherwise the porkchop transit.
    /// Both the preview and the created intent must use the same value or the
    /// created plan diverges from the preview, so it is resolved in one place.</summary>
    private static SimTime ResolveFlybyTransit(
        HohmannMultiPassPlanner.ShiftResult shift, OrbitalTransfers.PorkChopEntry entry)
        => shift.KShift > 0 ? shift.ShiftedTransit : entry.TransferData.Transit;

    /// <summary>Bakes the flyby offset into a center-aimed multi-pass input so the
    /// split departs toward the flyby. Returns the input unchanged and sets
    /// <paramref name="failed"/> when the flyby is off (failed = false) or the
    /// retarget could not be applied (failed = true), so the caller can warn or
    /// refuse instead of silently planning an impact. The planner locks exactly
    /// one energy descriptor: the hyperbolic ejection excess for cross-parent, or
    /// the post-burn apoapsis for same-parent. Math lives in <see cref="FlybyTargeting"/>.</summary>
    private static HohmannMultiPassPlanner.HohmannPlanInput MaybeApplyFlyby(
        Vehicle source, OrbitalTransfers.TransferInfo info,
        HohmannMultiPassPlanner.HohmannPlanInput center, SimTime transit, out bool failed)
    {
        failed = false;
        if (info.Target is not IParentBody target) return center;
        if (info.Target is not IOrbiter targetOrbiter) return center;
        if (!HohmannFlybyUI.TryGetRequest(target, out double rp, out FlybySide side))
            return center;

        var outcome = FlybyTargeting.ComputeFlybyDeparture(
            source, targetOrbiter, center.TFinal, transit, rp, side);
        if (outcome.Result == null)
        {
            failed = true;
            return center;
        }
        FlybyTargeting.FlybyResult f = outcome.Result.Value;

        if (center.IsCrossParent)
        {
            // Cross-parent locks the hyperbolic ejection excess; apoapsis unused.
            if (!(f.PlannerVInfMs > 0.0)) { failed = true; return center; }
            return center with
            {
                TFinal = f.BurnTime,
                DFinalVlf = f.DvVlf,
                VInfMs = f.PlannerVInfMs,
            };
        }

        // Same-parent locks the post-burn apoapsis; v_inf unused.
        if (!(f.PlannerApoTargetMeters > 0.0)) { failed = true; return center; }
        return center with
        {
            TFinal = f.BurnTime,
            DFinalVlf = f.DvVlf,
            ApoTargetRadiusMeters = f.PlannerApoTargetMeters,
        };
    }

    /// <summary>Warns when a multi-pass split is armed with the flyby option on
    /// but the retarget failed, so the plan would depart center-aimed (impact).</summary>
    private static void DrawFlybyMultipassNoteIfApplicable()
    {
        if (!HohmannFlybyUI.FlybyRequested || !_flybyRetargetFailed) return;
        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ColorOrange);
        ImGui.TextWrapped("[!] Flyby retarget failed for this multi-pass geometry; the split would still impact. Reduce passes, change the window, or flip the side.");
        ImGui.PopStyleColor();
    }

    #endregion
}
