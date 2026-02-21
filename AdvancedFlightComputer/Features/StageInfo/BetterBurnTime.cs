using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.StageInfo;

/// <summary>
/// Extends the Staging window with per-stage Delta V, TWR, burn time, ISP,
/// and a fuel progress bar. Also shows total Delta V in a footer.
///
/// Since StagingWindow is a private nested class of Staging, we use manual
/// Harmony patching to replace its DrawContent method. The Prefix re-implements
/// the original rendering and adds our data inline (progress bar on the stage
/// header line, info text when expanded).
///
/// Analysis results are cached and refreshed periodically (~0.5s) to avoid
/// running StageAnalyzer every frame.
/// </summary>
static class BetterBurnTime
{
    private const int UpdateIntervalFrames = 30;

    private static VehicleBurnAnalysis? _cachedAnalysis;
    private static Dictionary<int, StageBurnInfo>? _stageInfoLookup;
    private static BurnAnalysis? _cachedBurnAnalysis;
    private static Dictionary<int, BurnStageAllocation>? _burnAllocationLookup;
    private static string? _lastVehicleId;
    private static int _framesSinceUpdate;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static readonly ImColor8 ColorInsufficient = new ImColor8(255, 60, 60, 255);

    /// <summary>
    /// Returns a gradient color based on how much of a stage's dV the burn
    /// consumes. ratio=0 (green) -> 0.5 (yellow) -> 1.0 (red).
    /// </summary>
    private static ImColor8 GetBurnGradientColor(float ratio)
    {
        ratio = Math.Clamp(ratio, 0f, 1f);
        byte r, g, b;
        if (ratio <= 0.5f)
        {
            // Green (80,220,80) -> Yellow (255,220,0)
            float t = ratio * 2f;
            r = (byte)(80 + 175 * t);
            g = 220;
            b = (byte)(80 - 80 * t);
        }
        else
        {
            // Yellow (255,220,0) -> Red (255,60,60)
            float t = (ratio - 0.5f) * 2f;
            r = 255;
            g = (byte)(220 - 160 * t);
            b = (byte)(60 * t);
        }
        return new ImColor8(r, g, b, 255);
    }

    private static MethodInfo? _drawThruster;
    private static MethodInfo? _drawEngine;
    private static MethodInfo? _drawDecoupler;

    /// <summary>
    /// Sets up the manual Harmony patch on StagingWindow.DrawContent and caches
    /// reflection handles for the private DrawComponent&lt;T&gt; method.
    /// Called from Mod.OnFullyLoaded after PatchAll.
    /// </summary>
    public static bool ApplyPatches(Harmony harmony)
    {
        var windowType = typeof(Staging).GetNestedType("StagingWindow",
            BindingFlags.NonPublic);
        if (windowType == null)
        {
            DefaultCategory.Log.Error(
                "[AFC] BetterBurnTime: StagingWindow type not found - game version changed?");
            return false;
        }

        var drawContent = windowType.GetMethod("DrawContent",
            BindingFlags.Public | BindingFlags.Instance);
        if (drawContent == null)
        {
            DefaultCategory.Log.Error(
                "[AFC] BetterBurnTime: DrawContent method not found - game version changed?");
            return false;
        }

        var drawComponentOpen = windowType.GetMethod("DrawComponent",
            BindingFlags.NonPublic | BindingFlags.Instance);
        if (drawComponentOpen == null)
        {
            DefaultCategory.Log.Error(
                "[AFC] BetterBurnTime: DrawComponent method not found - game version changed?");
            return false;
        }

        _drawThruster = drawComponentOpen.MakeGenericMethod(typeof(ThrusterController));
        _drawEngine = drawComponentOpen.MakeGenericMethod(typeof(EngineController));
        _drawDecoupler = drawComponentOpen.MakeGenericMethod(typeof(Decoupler));

        harmony.Patch(drawContent,
            prefix: new HarmonyMethod(typeof(BetterBurnTime), nameof(DrawContentPrefix)));

        if (Mod.DebugMode)
            DefaultCategory.Log.Debug("[AFC] BetterBurnTime: patched StagingWindow.DrawContent");

        return true;
    }

    public static void Reset()
    {
        _cachedAnalysis = null;
        _stageInfoLookup = null;
        _cachedBurnAnalysis = null;
        _burnAllocationLookup = null;
        _lastVehicleId = null;
        _framesSinceUpdate = 0;
    }

    #region Cache Management

    private static void UpdateCache(Vehicle vehicle)
    {
        string vehicleId = vehicle.Id;

        if (vehicleId != _lastVehicleId)
        {
            _lastVehicleId = vehicleId;
            _framesSinceUpdate = UpdateIntervalFrames;
        }

        _framesSinceUpdate++;
        if (_framesSinceUpdate < UpdateIntervalFrames)
            return;

        _framesSinceUpdate = 0;
        _cachedAnalysis = StageAnalyzer.Analyze(vehicle);

        _stageInfoLookup = new Dictionary<int, StageBurnInfo>();
        foreach (var stage in _cachedAnalysis.Value.Stages)
            _stageInfoLookup[stage.StageNumber] = stage;

        UpdateBurnAnalysisCache(vehicle);
    }

    private static void UpdateBurnAnalysisCache(Vehicle vehicle)
    {
        BurnTarget? burn = vehicle.FlightComputer.Burn;
        if (burn == null || _cachedAnalysis == null)
        {
            _cachedBurnAnalysis = null;
            _burnAllocationLookup = null;
            return;
        }

        float requiredDv = burn.DeltaVToGoCci.Length();
        if (requiredDv <= 0f)
        {
            _cachedBurnAnalysis = null;
            _burnAllocationLookup = null;
            return;
        }

        _cachedBurnAnalysis = StageAnalyzer.AnalyzeBurn(_cachedAnalysis.Value, requiredDv);

        _burnAllocationLookup = new Dictionary<int, BurnStageAllocation>();
        foreach (var alloc in _cachedBurnAnalysis.Value.StageAllocations)
            _burnAllocationLookup[alloc.StageNumber] = alloc;
    }

    #endregion

    #region DrawContent Replacement

    /// <summary>
    /// Replaces StagingWindow.DrawContent. Re-implements the original stage tree
    /// rendering and adds per-stage info (progress bar, Delta V, TWR, burn time,
    /// ISP) plus a total footer.
    /// </summary>
    static bool DrawContentPrefix(object __instance, Viewport viewport)
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null)
            return false;

        UpdateCache(vehicle);

        float footerHeight = ImGui.GetTextLineHeightWithSpacing() + 4f;
        float tableHeight = ImGui.GetContentRegionAvail().Y - footerHeight;
        if (tableHeight < 50f)
            tableHeight = 50f;

        ImGuiTableFlags flags = ImGuiTableFlags.BordersV
            | ImGuiTableFlags.Resizable
            | ImGuiTableFlags.RowBg
            | ImGuiTableFlags.BordersOuterH
            | ImGuiTableFlags.NoBordersInBody
            | ImGuiTableFlags.ScrollY;

        if (!ImGui.BeginTable("stages"u8, 1, flags,
                new float2?(new float2(0f, tableHeight))))
            return false;

        ImGui.TableSetupScrollFreeze(0, 1);
        ImGui.TableSetupColumn(vehicle.Id, ImGuiTableColumnFlags.NoHide);
        ImGui.TableHeadersRow();

        StageList stageList = vehicle.Parts.StageList;
        for (int i = 0; i < stageList.Stages.Length; i++)
        {
            Stage stage = vehicle.Parts.StageList.Stages[i];
            if (stage.Parts.IsEmpty)
                continue;

            ImGui.TableNextRow();
            ImGui.TableNextColumn();

            ImGuiTreeNodeFlags treeFlags = ImGuiTreeNodeFlags.DefaultOpen
                | ImGuiTreeNodeFlags.FramePadding
                | ImGuiTreeNodeFlags.DrawLinesToNodes;

            string text = $"Stage {stage.StageNumber}";
            bool activated = stage.Activated;
            if (!activated)
                ImGui.PushStyleColor(ImGuiCol.Text,
                    ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));

            bool expanded = ImGui.TreeNodeEx(text, treeFlags);
            stage.Highlight = ImGui.IsItemHovered();

            DrawStageProgressBar(stage.StageNumber);

            if (expanded)
            {
                DrawStageInfoLine(stage.StageNumber);

                ImGui.TableNextRow();
                ImGui.TableNextColumn();
                ReadOnlySpan<Part> parts = stage.Parts;
                for (int j = 0; j < parts.Length; j++)
                {
                    Part part = parts[j];
                    if (part.HasAny<ThrusterController>()
                        || part.HasAny<Decoupler>()
                        || part.HasAny<EngineController>())
                    {
                        bool partExpanded = ImGui.TreeNodeEx(part.DisplayName, treeFlags);
                        part.Highlighted = stage.Highlight || ImGui.IsItemHovered();
                        if (partExpanded)
                        {
                            ImGui.TableNextRow();
                            ImGui.TableNextColumn();
                            InvokeDrawComponent(_drawThruster, __instance, part);
                            InvokeDrawComponent(_drawEngine, __instance, part);
                            InvokeDrawComponent(_drawDecoupler, __instance, part);
                            ImGui.TreePop();
                        }
                    }
                }
                ImGui.TreePop();
            }

            if (!activated)
                ImGui.PopStyleColor();
        }

        ImGui.EndTable();

        DrawTotalFooter();

        return false;
    }

    private static readonly object[] _invokeArgs = new object[1];

    private static void InvokeDrawComponent(MethodInfo? method, object instance, Part part)
    {
        if (method == null) return;
        _invokeArgs[0] = part;
        method.Invoke(instance, _invokeArgs);
    }

    #endregion

    #region Stage Info Rendering

    private static void DrawStageProgressBar(int stageNumber)
    {
        if (_stageInfoLookup == null) return;
        if (!_stageInfoLookup.TryGetValue(stageNumber, out var info)) return;
        if (info.EngineCount == 0) return;

        ImGui.SameLine();
        float availWidth = ImGui.GetContentRegionAvail().X;
        float pctTextWidth = ImGui.CalcTextSize("100% fuel"u8).X + 8f;
        float barWidth = availWidth - pctTextWidth;
        if (barWidth < 30f) return;

        float lineHeight = ImGui.GetTextLineHeight();
        float barHeight = lineHeight * 0.6f;
        float yOffset = (lineHeight - barHeight) * 0.5f;

        float2 cursor = ImGui.GetCursorPos();
        ImGui.SetCursorPos(new float2(cursor.X, cursor.Y + yOffset));
        ImGui.ProgressBar(info.FuelFraction,
            new float2?(new float2(barWidth, barHeight)), ""u8);
        ImGui.SameLine();
        ImGui.SetCursorPosY(cursor.Y);
        string pctText = string.Format(Inv, "{0}% fuel", (int)(info.FuelFraction * 100f));
        ImGui.Text(pctText);
    }

    private static void DrawStageInfoLine(int stageNumber)
    {
        if (_stageInfoLookup == null) return;
        if (!_stageInfoLookup.TryGetValue(stageNumber, out var info)) return;
        if (info.EngineCount == 0) return;

        ImGui.TableNextRow();
        ImGui.TableNextColumn();
        ImGui.Indent();
        ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));

        float spacing = ImGui.GetStyle().ItemSpacing.X;
        float availWidth = ImGui.GetContentRegionAvail().X;
        float lineX = 0f;

        string dvText = string.Format(Inv, "Delta V: {0:N0} m/s", info.DeltaV);
        string twrText = string.Format(Inv, "TWR: {0:F2}", info.Twr);
        string burnTimeText = string.Format(Inv, "Burn Time: {0}", FormatBurnTime(info.BurnTime));
        string ispText = string.Format(Inv, "ISP: {0:F0}s", info.Isp);

        // When a burn is planned and this stage is involved, color the Delta V
        // text and show how much dV the burn needs from this stage.
        BurnStageAllocation? alloc = null;
        if (_burnAllocationLookup != null
            && _burnAllocationLookup.TryGetValue(stageNumber, out var a))
            alloc = a;

        if (alloc != null)
        {
            float ratio = alloc.Value.StageTotalDv > 0f
                ? alloc.Value.AllocatedDv / alloc.Value.StageTotalDv
                : 1f;
            ImColor8 burnColor = GetBurnGradientColor(ratio);

            DrawInfoSegmentColored(dvText, burnColor, ref lineX, availWidth, spacing);

            string needsText = string.Format(Inv, "needs {0:N0} m/s", alloc.Value.AllocatedDv);
            DrawInfoSegmentColored(needsText, burnColor, ref lineX, availWidth, spacing);
        }
        else
        {
            DrawInfoSegment(dvText, ref lineX, availWidth, spacing, isFirst: true);
        }

        DrawInfoSegment(twrText, ref lineX, availWidth, spacing, isFirst: false);
        DrawInfoSegment(burnTimeText, ref lineX, availWidth, spacing, isFirst: false);
        DrawInfoSegment(ispText, ref lineX, availWidth, spacing, isFirst: false);

        ImGui.PopStyleColor();
        ImGui.Unindent();
    }

    private static void DrawInfoSegment(string text, ref float lineX,
        float availWidth, float spacing, bool isFirst)
    {
        DrawInfoSegmentColored(text, null, ref lineX, availWidth, spacing, isFirst);
    }

    private static void DrawInfoSegmentColored(string text, ImColor8? color,
        ref float lineX, float availWidth, float spacing, bool isFirst = false)
    {
        float2 textSize = ImGui.CalcTextSize(text);

        if (lineX > 0f && lineX + textSize.X <= availWidth)
            ImGui.SameLine(0f, spacing * 2f);
        else if (lineX > 0f)
            lineX = 0f;

        if (color != null)
            ImGui.PushStyleColor(ImGuiCol.Text, color.Value);
        ImGui.Text(text);
        if (color != null)
            ImGui.PopStyleColor();

        lineX += textSize.X + spacing * 2f;
    }

    private static void DrawTotalFooter()
    {
        if (_cachedAnalysis == null) return;
        var analysis = _cachedAnalysis.Value;
        if (analysis.Stages.Count == 0) return;

        ImGui.Separator();

        if (_cachedBurnAnalysis != null)
        {
            var burn = _cachedBurnAnalysis.Value;
            string totalText = string.Format(Inv,
                "Total Delta V: {0:N0} m/s", analysis.TotalDeltaV);
            ImGui.Text(totalText);
            ImGui.SameLine();
            ImGui.Text("|");
            ImGui.SameLine();

            if (burn.IsSufficient)
            {
                string burnText = string.Format(Inv,
                    "Burn: {0:N0} m/s  Burn Time: {1}",
                    burn.RequiredDv, FormatBurnTime(burn.TotalBurnTime));
                ImGui.Text(burnText);
            }
            else
            {
                string burnText = string.Format(Inv,
                    "Burn: {0:N0} m/s  INSUFFICIENT", burn.RequiredDv);
                ImGui.PushStyleColor(ImGuiCol.Text, ColorInsufficient);
                ImGui.Text(burnText);
                ImGui.PopStyleColor();
            }
        }
        else
        {
            string totalText = string.Format(Inv,
                "Total Delta V: {0:N0} m/s  Burn Time: {1}",
                analysis.TotalDeltaV, FormatBurnTime(analysis.TotalBurnTime));
            ImGui.Text(totalText);
        }
    }

    #endregion

    #region Corrected Burn Duration

    /// <summary>
    /// Returns the cached multi-stage burn time, or null if no burn analysis
    /// is available. Used by Patch_CorrectedBurnDuration to override the
    /// single-stage BurnDuration computed by the game's UpdateBurnTarget.
    /// </summary>
    internal static float? GetCorrectedBurnDuration()
    {
        return _cachedBurnAnalysis?.TotalBurnTime;
    }

    #endregion

    #region Formatting

    private static string FormatBurnTime(float seconds)
    {
        if (seconds < 60f)
            return $"{seconds:F0}s";
        if (seconds < 3600f)
        {
            int m = (int)(seconds / 60f);
            int s = (int)(seconds % 60f);
            return s > 0 ? $"{m}m {s}s" : $"{m}m";
        }
        int h = (int)(seconds / 3600f);
        int min = (int)((seconds % 3600f) / 60f);
        return min > 0 ? $"{h}h {min}m" : $"{h}h";
    }

    #endregion
}

/// <summary>
/// Overrides BurnDuration and IgnitionTime with multi-stage values after the
/// background job's FlightComputer copy is applied on the main thread.
///
/// The game's UpdateBurnTarget (worker thread) computes a single-stage
/// BurnDuration using only the currently active engines. This postfix
/// replaces it with the multi-stage burn time from our StageAnalyzer,
/// which accounts for all future stages.
///
/// This corrects the BURN TIME and START BURN IN gauge rollers, which
/// read from FlightComputer.Burn on the main thread during the render pass.
///
/// The worker thread's ignition decision uses its own freshly-computed
/// single-stage value (slightly off for multi-stage burns). This is
/// acceptable because auto-staging handles burn continuation.
/// </summary>
[HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
static class Patch_CorrectedBurnDuration
{
    static void Postfix(Vehicle __instance)
    {
        if (__instance != Program.ControlledVehicle) return;

        FlightComputer fc = __instance.FlightComputer;
        if (fc.Burn == null) return;

        float? corrected = BetterBurnTime.GetCorrectedBurnDuration();
        if (corrected == null || corrected.Value <= 0f) return;

        fc.Burn.BurnDuration = corrected.Value;
        fc.Burn.IgnitionTime = fc.Burn.ImpulsiveInstant - 0.5 * (double)fc.Burn.BurnDuration;
    }
}
