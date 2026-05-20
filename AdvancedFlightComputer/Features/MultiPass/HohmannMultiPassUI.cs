using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
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

    public static bool Enabled { get; set; }

    private static int _passCount = 1;

    // Cache key components - quantized so per-frame floating drift on
    // T_final / dV magnitude does not bust the cache. SplitMode is gone
    // for Hohmann: per-pass dV is determined entirely by the K-integer
    // sequence in HohmannMultiPassPlanner, not by user split-mode choice.
    // StartPassIndex distinguishes init-phase (always 0) from active-exec
    // (= exec.PassIndex), so a pass completion naturally busts the cache.
    private readonly record struct PreviewKey(
        string SourceId,
        string TargetId,
        long TFinalBucketSec,
        long DvMagBucket,
        long VInfBucket,
        long ApoTargetBucket,
        bool IsCrossParent,
        int PassCount,
        int StartPassIndex,
        long MassBucket);

    private static PreviewKey _cachedKey;
    private static bool _hasCachedPreview;
    private static PassPreviewResult _cachedPreview;
    private static int _autoClampedFromN;     // 0 = no clamp; >0 = user asked for this, got LargestFeasibleN
    // K_shift applied to fit the K-schedule for same-parent moon transfers
    // (LEO -> Luna, etc.). 0 = no shift applied (porkchop entry's TFinal was
    // already feasible). Surfaced in the UI so the user understands why
    // the multi-pass extends past the porkchop-selected cell.
    private static int _lastShiftKShift;
    private static string? _lastSourceId;
    private static string? _lastTargetId;

    /// <summary>
    /// Inline-drawn into stock's "Transfer Planning" window by the
    /// transpiler in Patch_DrawPlanWindow_HohmannMultiPass. We're already
    /// inside stock's Begin/End so no window-management here - just draw
    /// the section content.
    /// </summary>
    public static void DrawInline()
    {
        if (!Enabled) return;

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
            _hasCachedPreview = false;
            _autoClampedFromN = 0;
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
        if (!(bool)GameReflection.TransferPlanner_showPlanWindow!.GetValue(null)!)
            return false;
        var transferType = (TransferType)GameReflection.TransferPlanner_transferType!
            .GetValue(null)!;
        if (transferType.GetKey() != "Hohmann") return false;

        var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!
            .GetValue(null)!;
        if (sourceBody.Body is not Vehicle source) return false;
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
        _hasCachedPreview = false;
        _cachedPreview = default;
        _cachedKey = default;
        _autoClampedFromN = 0;
        _lastShiftKShift = 0;
        _lastSourceId = null;
        _lastTargetId = null;
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
    /// it twice. skipLast is always true: stock's selected-entry overlay
    /// owns the final-pass Lambert trajectory; rendering ours on top
    /// produced a visible double-line with slight discrepancies.</summary>
    public static void RenderOrbits(Viewport viewport, Vehicle source)
    {
        if (!HasMultiPassPreview) return;
        if (source.Id != _cachedKey.SourceId) return;

        bool skipFirst = MultiPassRegistry.Has(source.Id);
        MultiPassRenderer.RenderPassOrbits(
            viewport, source, _cachedPreview.Passes,
            skipFirst, skipLast: true);
    }

    /// <summary>Per-pass Ap/Pe/AN/DN/SOI/closest markers. ImGui-phase
    /// counterpart of <see cref="RenderOrbits"/>. firstPassDisplayNumber
    /// equals <c>exec.PassIndex + 1</c> during execution so labels reflect
    /// the absolute pass number in the original N-pass sequence
    /// (e.g. after pass 1 completes, the intermediate triangle still
    /// reads "Ap Pass 3" rather than restarting at 1). skipLast=true
    /// matches <see cref="RenderOrbits"/>; stock's <c>FlightPlan.DrawUi</c>
    /// labels the final pass (e.g. "Escape 0").</summary>
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
        MultiPassMarkers.Draw(viewport, source,
            _cachedPreview.Passes, firstPassDisplayNumber,
            skipFirst, skipLast: true);
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
            var transferType = (TransferType)GameReflection.TransferPlanner_transferType!
                .GetValue(null)!;
            if (transferType.GetKey() != "Hohmann") return false;

            if (!(bool)GameReflection.TransferPlanner_showPlanWindow!.GetValue(null)!)
                return false;

            entry = GameReflection.TransferPlanner_selectedEntry!.GetValue(null)
                as OrbitalTransfers.PorkChopEntry;
            if (entry == null) return false;

            info = GameReflection.TransferPlanner_transferInfo!.GetValue(null)
                as OrbitalTransfers.TransferInfo;
            if (info == null || info.Target == null) return false;

            var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!
                .GetValue(null)!;
            source = sourceBody.Body as Vehicle;
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
        ImGui.Spacing();

        DrawPassCountSelector(source, entry, info);

        if (_passCount > 1)
        {
            UpdatePreviewIfStale(source, entry, info);
            DrawSpanInfo(source);
            DrawPreviewFailureIfApplicable();
            DrawPassList();
        }

        // Auto-clamp banner stays visible AFTER the clamp drops _passCount
        // back to 1: the user needs to see why their > click had no
        // visible effect on the pass count. The < / > clicks explicitly
        // reset _autoClampedFromN, which is the only way to clear it.
        DrawAutoClampIfApplicable();

        if (_passCount <= 1 && _autoClampedFromN <= _passCount)
        {
            ImGui.Spacing();
            ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
            ImGui.TextWrapped("N = 1: stock Create button does a single Hohmann burn.");
            ImGui.PopStyleColor();
        }
        else if (_passCount > 1)
        {
            ImGui.Spacing();
            ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
            ImGui.TextWrapped(string.Format(Inv,
                "Click the stock Create button to start the {0}-pass execution.",
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
    /// the selected porkchop cell.</summary>
    private static void DrawSpanInfo(Vehicle source)
    {
        if (source.Orbit == null) return;
        double tPark = source.Orbit.Period;
        if (!(tPark > 0.0)) return;
        double spanSec = HohmannMultiPassPlanner.GetSpanSeconds(tPark, _passCount);
        if (spanSec <= 0.0) return;

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

        // Surface mid-exec planner failures (e.g. live-state drift makes
        // v_p_target unreachable) so the user understands why the orbit
        // overlay disappeared instead of seeing it silently vanish.
        DrawPreviewFailureIfApplicable();

        ImGui.Spacing();
        if (ImGuiHelper.DrawButton("Cancel remaining passes"u8,
                KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Red))
            CancelExecution(source, exec);
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
            startPassIndex: exec.PassIndex);
        if (_hasCachedPreview && key == _cachedKey) return;

        // Freeze during Auto burn so the per-tick mass drift does not
        // recompute every physics tick; Auto -> Manual on pass completion
        // naturally invalidates the key on the next frame.
        if (_hasCachedPreview
            && source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto)
            return;

        SimTime now = Universe.GetElapsedSimTime();
        SequenceBurnState state = SequenceBurnState.Analyze(source);
        _cachedPreview = HohmannMultiPassPlanner.Plan(
            source, input, exec.PassCountTotal, exec.PassIndex,
            intent.ParkingPeriodSec, state, now);
        _cachedKey = key;
        _hasCachedPreview = true;

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewForActiveExec: vehicle='{0}' " +
                "passIndex={1}/{2} -> failed={3} reason='{4}' previewPasses={5}",
                source.Id, exec.PassIndex, exec.PassCountTotal,
                _cachedPreview.Failed, _cachedPreview.FailureReason ?? "-",
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
        PassCompletionPatch.OnRegistryRemovedExternally(source.Id);
        MultiPassRegistry.Remove(source.Id);
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
                _autoClampedFromN = 0;
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
                _autoClampedFromN = 0;
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
            }
            _lastShiftKShift = 0;
            _hasCachedPreview = false;
            _cachedPreview = default;
            _cachedKey = default;
            return;
        }

        var raw = BuildBasePlanInput(source, entry, info);
        var shift = HohmannMultiPassPlanner.PrepareShiftedInput(
            raw, source, info, _passCount, parkingPeriodSec, now);

        var key = BuildKey(source, shift.Input,
            passCount: _passCount, startPassIndex: 0);
        if (_hasCachedPreview && key == _cachedKey)
        {
            // PrepareShiftedInput's own cache made the call cheap, but the
            // planner-cache hit means we also avoid the Plan call below.
            _lastShiftKShift = shift.KShift;
            return;
        }

        // Freeze the cache during an Auto burn so a mid-burn-mass drift
        // doesn't recompute every physics tick; the Auto -> Manual
        // transition will naturally bust the cache on the next frame.
        if (_hasCachedPreview
            && source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto)
            return;

        SequenceBurnState state = SequenceBurnState.Analyze(source);
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
            source, shift.Input, state, parkingPeriodSec, now, requestedN);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewIfStale: LargestFeasibleN " +
                "requested={0} -> clamped={1}",
                requestedN, clampedN));

        var planInput = shift.Input;
        if (clampedN < requestedN)
        {
            // _autoClampedFromN is sticky: clearing it here would reset
            // the banner on the next frame because requestedN already
            // equals clampedN (we wrote _passCount = clampedN). The < / >
            // buttons explicitly reset _autoClampedFromN, which is the
            // only way back to "no clamp warning".
            _autoClampedFromN = Math.Max(_autoClampedFromN, requestedN);
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
                raw, source, info, _passCount, parkingPeriodSec, now);
            planInput = shift.Input;
            _lastShiftKShift = shift.KShift;
            // BuildKey depends on planInput.TFinal (bucketed); recompute so
            // the cached key matches the actual input we're planning with,
            // otherwise the next frame would bust the cache on a phantom delta.
            key = BuildKey(source, planInput,
                passCount: _passCount, startPassIndex: 0);
        }

        _cachedPreview = HohmannMultiPassPlanner.Plan(
            source, planInput, _passCount, startPassIndex: 0,
            parkingPeriodSec, state, now);
        _cachedKey = key;
        _hasCachedPreview = true;

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannMultiPassUI.UpdatePreviewIfStale: Plan -> failed={0} " +
                "reason='{1}' previewPasses={2} _passCount(final)={3} K_shift(final)={4}",
                _cachedPreview.Failed, _cachedPreview.FailureReason ?? "-",
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

    private static void DrawAutoClampIfApplicable()
    {
        if (_autoClampedFromN <= _passCount) return;
        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ColorAmber);
        ImGui.TextWrapped(string.Format(Inv,
            "[!] {0} pass(es) requested, only {1} feasible at this departure time.\n" +
            "Multi-pass needs ~{0} parking-orbit periods of warning time before " +
            "the final burn; pick a later porkchop entry (arrow buttons / click " +
            "further right on the porkchop) to gain time budget.",
            _autoClampedFromN, _passCount));
        ImGui.PopStyleColor();
    }

    private static void DrawPassList()
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
            string suffix = i == passes.Length - 1 ? " (final)" : "";
            string line = t > 0.5
                ? string.Format(Inv, "Pass {0}: {1:F0} m/s, {2:F0}s{3}", i + 1, dv, t, suffix)
                : string.Format(Inv, "Pass {0}: {1:F0} m/s{2}", i + 1, dv, suffix);
            ImGui.Text(line);
        }
        ImGui.PopStyleColor();
    }

    #endregion

    #region Interceptor handoff

    /// <summary>True when the user has selected N>1 but the preview
    /// failed (so the interceptor will fall through to a single burn).
    /// Lets the interceptor surface an alert instead of silently giving
    /// the user something different from what they asked for.</summary>
    public static bool WantedMultiPassButPreviewFailed() =>
        Enabled && _passCount > 1 && _hasCachedPreview && _cachedPreview.Failed;

    /// <summary>
    /// True when the inline UI is armed for a Hohmann multi-pass on
    /// <paramref name="vehicle"/>: pass count > 1, a valid cached preview,
    /// and stock state (transfer type / selected porkchop entry / source
    /// vehicle) matches. Used by <see cref="HohmannCreateInterceptor"/>
    /// to decide whether stock's Create button should fire multi-pass
    /// or fall through to a single burn.
    /// </summary>
    public static bool TryGetArmedState(
        Vehicle vehicle, out int passCount, out HohmannTransferIntent? intent)
    {
        passCount = 0;
        intent = null;
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
            // the planner's "non-positive vpTarget" check then surfaces
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
        // the first pass.
        var raw = BuildBasePlanInput(source, entry, info);
        var shift = HohmannMultiPassPlanner.PrepareShiftedInput(
            raw, source, info, passCount, parkingPeriod, now);
        var input = shift.Input;

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
        int passCount, int startPassIndex)
    {
        return new PreviewKey(
            SourceId: source.Id,
            TargetId: (input.Target as Astronomical)?.Id ?? string.Empty,
            TFinalBucketSec: (long)input.TFinal.Seconds(),
            DvMagBucket: (long)input.DFinalVlf.Length(),
            VInfBucket: (long)input.VInfMs,
            ApoTargetBucket: (long)(input.ApoTargetRadiusMeters / 1000.0),
            IsCrossParent: input.IsCrossParent,
            PassCount: passCount,
            StartPassIndex: startPassIndex,
            MassBucket: (long)(source.TotalMass / 100.0));
    }

    #endregion
}
