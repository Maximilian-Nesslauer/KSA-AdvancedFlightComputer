using System;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using AdvancedFlightComputer.Core;
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
/// Pure rendering code - reads all analysis data from StageAnalysisCache.
/// Since StagingWindow is a private nested class of Staging, we use manual
/// Harmony patching to replace its DrawContent method.
/// </summary>
static class StageInfoPanel
{
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
    /// Applies all StageInfo Harmony patches. Called from Mod.cs after
    /// GameReflection.ValidateStageInfo() passes. Includes:
    /// - StagingWindow.DrawContent replacement (manual patch)
    /// - Worker-thread ignition timing fix (manual patch)
    /// - Corrected burn duration postfix
    /// - StageAnalyzerDebug patches (debug-only analysis logging)
    /// </summary>
    public static bool ApplyPatches(Harmony harmony)
    {
        _drawThruster = GameReflection.StagingWindow_DrawComponentOpen!
            .MakeGenericMethod(typeof(ThrusterController));
        _drawEngine = GameReflection.StagingWindow_DrawComponentOpen!
            .MakeGenericMethod(typeof(EngineController));
        _drawDecoupler = GameReflection.StagingWindow_DrawComponentOpen!
            .MakeGenericMethod(typeof(Decoupler));

        harmony.Patch(GameReflection.StagingWindow_DrawContent!,
            prefix: new HarmonyMethod(typeof(StageInfoPanel), nameof(DrawContentPrefix)));

        if (GameReflection.FlightComputer_UpdateBurnTarget != null)
        {
            harmony.Patch(GameReflection.FlightComputer_UpdateBurnTarget,
                postfix: new HarmonyMethod(typeof(Patch_WorkerIgnitionTiming),
                    nameof(Patch_WorkerIgnitionTiming.Postfix)));
        }
        else
        {
            DefaultCategory.Log.Warning(
                "[AFC] FlightComputer.UpdateBurnTarget not found - worker ignition timing fix disabled.");
        }

        harmony.CreateClassProcessor(typeof(Patch_CorrectedBurnDuration)).Patch();
        harmony.CreateClassProcessor(typeof(StageAnalyzerDebug.Patch_AnalyzeAfterStaging)).Patch();
        harmony.CreateClassProcessor(typeof(StageAnalyzerDebug.Patch_InitialAnalysis)).Patch();

        if (DebugConfig.StageInfo)
            DefaultCategory.Log.Debug("[AFC] StageInfo: all patches applied.");

        return true;
    }

    #region DrawContent Replacement

    /// <summary>
    /// Replaces StagingWindow.DrawContent. Re-implements the original stage tree
    /// rendering and adds per-stage info (progress bar, Delta V, TWR, burn time,
    /// ISP) plus a total footer. Reads all data from StageAnalysisCache.
    /// </summary>
    static bool DrawContentPrefix(object __instance, Viewport viewport)
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null)
            return false;

        StageAnalysisCache.MarkPanelActive();

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

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("DrawContentPrefix", Stopwatch.GetTimestamp() - perfStart);
#endif
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
        if (!StageAnalysisCache.TryGetStageInfo(stageNumber, out var info)) return;
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
        string pctText = string.Format(Inv, "{0}% fuel", (int)MathF.Round(info.FuelFraction * 100f));
        ImGui.Text(pctText);
    }

    private static void DrawStageInfoLine(int stageNumber)
    {
        if (!StageAnalysisCache.TryGetStageInfo(stageNumber, out var info)) return;
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

        BurnStageAllocation? alloc = null;
        if (StageAnalysisCache.TryGetBurnAllocation(stageNumber, out var a))
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
        var analysis = StageAnalysisCache.Analysis;
        if (analysis == null) return;
        var stages = analysis.Value;
        if (stages.Stages.Count == 0) return;

        ImGui.Separator();

        var burnAnalysis = StageAnalysisCache.BurnAnalysis;
        if (burnAnalysis != null)
        {
            var burn = burnAnalysis.Value;
            string totalText = string.Format(Inv,
                "Total Delta V: {0:N0} m/s", stages.TotalDeltaV);
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
                stages.TotalDeltaV, FormatBurnTime(stages.TotalBurnTime));
            ImGui.Text(totalText);
        }
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
