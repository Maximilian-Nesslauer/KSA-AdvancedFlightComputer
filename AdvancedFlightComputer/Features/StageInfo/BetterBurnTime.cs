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
    private static string? _lastVehicleId;
    private static int _framesSinceUpdate;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

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

        string[] segments =
        [
            string.Format(Inv, "Delta V: {0:N0} m/s", info.DeltaV),
            string.Format(Inv, "TWR: {0:F2}", info.Twr),
            string.Format(Inv, "Burn Time: {0}", FormatBurnTime(info.BurnTime)),
            string.Format(Inv, "ISP: {0:F0}s", info.Isp)
        ];

        float spacing = ImGui.GetStyle().ItemSpacing.X;
        float availWidth = ImGui.GetContentRegionAvail().X;
        float lineX = 0f;

        for (int i = 0; i < segments.Length; i++)
        {
            float2 textSize = ImGui.CalcTextSize(segments[i]);
            float blockWidth = textSize.X + (i < segments.Length - 1 ? spacing * 2f : 0f);

            if (i > 0 && lineX + textSize.X <= availWidth)
            {
                ImGui.SameLine(0f, spacing * 2f);
            }
            else if (i > 0)
            {
                lineX = 0f;
            }

            ImGui.Text(segments[i]);
            lineX += blockWidth;
        }

        ImGui.PopStyleColor();
        ImGui.Unindent();
    }

    private static void DrawTotalFooter()
    {
        if (_cachedAnalysis == null) return;
        var analysis = _cachedAnalysis.Value;
        if (analysis.Stages.Count == 0) return;

        ImGui.Separator();
        string totalText = string.Format(Inv,
            "Total Delta V: {0:N0} m/s  Burn Time: {1}",
            analysis.TotalDeltaV, FormatBurnTime(analysis.TotalBurnTime));
        ImGui.Text(totalText);
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
