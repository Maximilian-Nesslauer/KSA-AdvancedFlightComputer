using System;
using System.Collections.Generic;
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
/// Supports atmospheric thrust display via configurable display modes.
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

    private static readonly string[] ModeLabels = { "Auto", "VAC", "ASL", "VAC + ASL", "Planning" };
    private static bool _initialSizeApplied;

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

        if (!_initialSizeApplied)
        {
            _initialSizeApplied = true;
            float2 screen = ImGui.GetMainViewport().Size;
            ImGui.SetWindowSize(new float2(screen.X * 0.11f, screen.Y * 0.225f));
        }

        StageAnalysisCache.MarkPanelActive();

        DrawModeSelector();

        bool hasSecondary = StageAnalysisCache.SecondaryAnalysis != null;
        float footerLines = hasSecondary ? 2f : 1f;
        float footerHeight = ImGui.GetTextLineHeightWithSpacing() * footerLines + 4f;
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

    #region Mode Selector

    private static void DrawModeSelector()
    {
        ImGui.PushItemWidth(110f);

        string currentLabel = ModeLabels[(int)StageInfoSettings.Mode];

        if (ImGui.BeginCombo("##StageInfoMode"u8, currentLabel))
        {
            for (int i = 0; i < ModeLabels.Length; i++)
            {
                bool isSelected = (int)StageInfoSettings.Mode == i;
                if (ImGui.Selectable(ModeLabels[i], isSelected))
                    StageInfoSettings.Mode = (StageDisplayMode)i;
            }
            ImGui.EndCombo();
        }
        ImGui.PopItemWidth();

        if (StageInfoSettings.Mode == StageDisplayMode.Planning)
        {
            ImGui.SameLine();
            DrawBodySelector();
        }
    }

    private static void DrawBodySelector()
    {
        List<Astronomical> bodies = StageInfoSettings.GetCelestialBodies();
        if (bodies.Count == 0)
        {
            ImGui.Text("(no bodies)");
            return;
        }

        if (StageInfoSettings.SelectedBodyId == null)
            StageInfoSettings.SelectedBodyId = bodies[0].Id;

        string currentName = StageInfoSettings.SelectedBodyId;

        ImGui.PushItemWidth(140f);
        if (ImGui.BeginCombo("##PlanningBody"u8, currentName))
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                string bodyId = bodies[i].Id;
                bool isSelected = bodyId == StageInfoSettings.SelectedBodyId;
                if (ImGui.Selectable(bodyId, isSelected))
                    StageInfoSettings.SelectedBodyId = bodyId;
            }
            ImGui.EndCombo();
        }
        ImGui.PopItemWidth();
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

        string primaryLabel = StageAnalysisCache.PrimaryLabel;
        BurnStageAllocation? primaryAlloc = null;
        if (StageAnalysisCache.TryGetBurnAllocation(stageNumber, out var pa))
            primaryAlloc = pa;

        bool hasSecondary = StageAnalysisCache.TryGetSecondaryStageInfo(stageNumber, out var secondaryInfo)
            && secondaryInfo.EngineCount > 0;

        bool primaryDimmed = hasSecondary && !StageAnalysisCache.IsPrimaryCurrentCondition;
        DrawSingleStageInfoLine(info, primaryLabel, primaryAlloc, primaryDimmed);

        if (hasSecondary)
        {
            string secondaryLabel = StageAnalysisCache.SecondaryLabel ?? "";
            BurnStageAllocation? secondaryAlloc = null;
            if (StageAnalysisCache.TryGetSecondaryBurnAllocation(stageNumber, out var sa))
                secondaryAlloc = sa;

            bool secondaryDimmed = StageAnalysisCache.IsPrimaryCurrentCondition;
            DrawSingleStageInfoLine(secondaryInfo, secondaryLabel, secondaryAlloc, secondaryDimmed);
        }
    }

    private static void DrawSingleStageInfoLine(StageBurnInfo info, string label,
        BurnStageAllocation? alloc, bool isDimmed)
    {
        ImGui.TableNextRow();
        ImGui.TableNextColumn();
        ImGui.Indent();

        if (isDimmed)
        {
            var dimColor = ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled);
            dimColor.W *= 0.6f;
            ImGui.PushStyleColor(ImGuiCol.Text, dimColor);
        }
        else
        {
            ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        }

        float spacing = ImGui.GetStyle().ItemSpacing.X;
        float availWidth = ImGui.GetContentRegionAvail().X;
        float lineX = 0f;

        if (alloc != null)
        {
            float ratio = alloc.Value.StageTotalDv > 0f
                ? alloc.Value.AllocatedDv / alloc.Value.StageTotalDv
                : 1f;
            ImColor8 burnColor = isDimmed
                ? new ImColor8(180, 180, 180, 160)
                : GetBurnGradientColor(ratio);

            string allocText = string.IsNullOrEmpty(label)
                ? string.Format(Inv, "Burn allocated {0:N0} / {1:N0} m/s stage deltaV",
                    alloc.Value.AllocatedDv, info.DeltaV)
                : string.Format(Inv, "{0} Burn allocated {1:N0} / {2:N0} m/s stage deltaV",
                    label, alloc.Value.AllocatedDv, info.DeltaV);
            DrawInfoSegmentColored(allocText, burnColor, ref lineX, availWidth, spacing);
        }
        else
        {
            string dvText = string.IsNullOrEmpty(label)
                ? string.Format(Inv, "Delta V: {0:N0} m/s", info.DeltaV)
                : string.Format(Inv, "{0} Delta V: {1:N0} m/s", label, info.DeltaV);
            DrawInfoSegment(dvText, ref lineX, availWidth, spacing, isFirst: true);
        }

        string twrText = string.Format(Inv, "TWR: {0:F2}", info.Twr);
        string burnTimeText = string.Format(Inv, "Burn: {0}", FormatBurnTime(info.BurnTime));
        string ispText = string.Format(Inv, "ISP: {0:F0}s", info.Isp);

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

        string primaryLabel = StageAnalysisCache.PrimaryLabel;
        bool hasSecondary = StageAnalysisCache.SecondaryAnalysis != null;
        bool primaryDimmed = hasSecondary && !StageAnalysisCache.IsPrimaryCurrentCondition;

        DrawTotalLine(stages, StageAnalysisCache.BurnAnalysis, primaryLabel, primaryDimmed);

        if (hasSecondary)
        {
            var secondary = StageAnalysisCache.SecondaryAnalysis!.Value;
            string secondaryLabel = StageAnalysisCache.SecondaryLabel ?? "";
            bool secondaryDimmed = StageAnalysisCache.IsPrimaryCurrentCondition;

            DrawTotalLine(secondary, StageAnalysisCache.SecondaryBurnAnalysis,
                secondaryLabel, secondaryDimmed);
        }
    }

    private static void DrawTotalLine(VehicleBurnAnalysis stages, BurnAnalysis? burnAnalysis,
        string label, bool isDimmed)
    {
        if (isDimmed)
        {
            var dimColor = ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled);
            dimColor.W *= 0.6f;
            ImGui.PushStyleColor(ImGuiCol.Text, dimColor);
        }

        string prefix = string.IsNullOrEmpty(label) ? "" : label + " ";

        if (burnAnalysis != null)
        {
            var burn = burnAnalysis.Value;
            string totalText = string.Format(Inv,
                "{0}Total Delta V: {1:N0} m/s", prefix, stages.TotalDeltaV);
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
                "{0}Total Delta V: {1:N0} m/s  Burn Time: {2}",
                prefix, stages.TotalDeltaV, FormatBurnTime(stages.TotalBurnTime));
            ImGui.Text(totalText);
        }

        if (isDimmed)
            ImGui.PopStyleColor();
    }

    #endregion

    #region Formatting

    private static string FormatBurnTime(float seconds)
        => Core.FormatHelper.FormatDuration(seconds);

    #endregion
}
