using System;
using System.Collections.Generic;
using System.Globalization;
using AutoStage.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AutoStage;

/// <summary>
/// Delay settings in the pinned Part Window, one block per (module kind,
/// sequence) the part fires in. A tower with a motor and two mounts on three
/// rows gets three blocks; a part that fires nothing draws none.
/// </summary>
[HarmonyPatch(typeof(Part), nameof(Part.DrawPartInfo))]
static class PartWindowPatch
{
    // Shared scratch, safe only because Part.DrawPartInfo never nests.
    private static readonly List<int> _engineSequences = new();
    private static readonly List<int> _decouplerSequences = new();

    static void Postfix(Part __instance)
    {
        try
        {
            DrawDelays(__instance);
        }
        catch (Exception ex)
        {
            LogHelper.ErrorOnce("PartWindow.Draw",
                $"[AutoStage] PartWindow draw error: {ex.Message}");
        }
    }

    private static void DrawDelays(Part part)
    {
        if (!Mod.IgnitionDelayAvailable)
            return;

        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null || part.Tree != vehicle.Parts)
            return;

        CollectSequences(part);
        if (_engineSequences.Count == 0 && _decouplerSequences.Count == 0)
            return;

        Config.LoadVehicleOverrides(vehicle.Id);

        ImGui.Spacing();
        ImGui.Separator();
        ImGui.Spacing();

        string partName = part.Template.DisplayName;

        foreach (int sequence in _engineSequences)
            DrawDelayBlock(part, vehicle, sequence, DelayKind.Engine, "Ignition Delay", partName);

        foreach (int sequence in _decouplerSequences)
            DrawDelayBlock(part, vehicle, sequence, DelayKind.Decoupler, "Decoupler Delay", partName);
    }

    /// <summary>Ascending, deduplicated. Sequence 0 means "no row".</summary>
    private static void CollectSequences(Part part)
    {
        _engineSequences.Clear();
        _decouplerSequences.Clear();

        foreach (ISequenced module in part.GetSubtreeSequencedModules())
        {
            int sequence = module.Sequence;
            if (sequence <= 0) continue;

            // Both named positively: an else-branch would file a future third
            // kind as a decoupler, drawing a row staging then ignores.
            List<int>? target =
                module is EngineController ? _engineSequences :
                module is Decoupler ? _decouplerSequences : null;
            if (target != null && !target.Contains(sequence))
                target.Add(sequence);
        }

        _engineSequences.Sort();
        _decouplerSequences.Sort();
    }

    private static void DrawDelayBlock(Part part, Vehicle vehicle, int sequence, DelayKind kind,
        string title, string partName)
    {
        double effectiveDelay = kind == DelayKind.Engine
            ? Config.GetSequenceEngineDelay(vehicle, sequence)
            : Config.GetSequenceDecouplerDelay(vehicle, sequence);
        double partDefault = kind == DelayKind.Engine
            ? Config.ComputeSequenceEngineDelay(vehicle, sequence)
            : Config.ComputeSequenceDecouplerDelay(vehicle, sequence);
        bool hasOverride = kind == DelayKind.Engine
            ? Config.HasSequenceEngineOverride(vehicle, sequence)
            : Config.HasSequenceDecouplerOverride(vehicle, sequence);

        // Sequence in the id, not just the label: two blocks of one kind would
        // otherwise share it and both edit whichever drew first.
        ImGui.PushID(string.Format(CultureInfo.InvariantCulture,
            "AutoStageDelay_{0}_{1}", kind, sequence));

        ImGui.Text(string.Format(CultureInfo.InvariantCulture,
            "{0} - {1} (Seq {2})", title, partName, sequence));

        // Which of the part's modules this row fires.
        DrawCoveredModules(part, sequence, kind);

        ImGui.Spacing();

        float delayValue = (float)effectiveDelay;
        ImGui.SetNextItemWidth(120f);
        if (ImGui.InputFloat("###val"u8, ref delayValue, 0.1f, 1.0f, "%.1f"))
        {
            if (kind == DelayKind.Engine)
                Config.SetSequenceEngineOverride(vehicle, sequence, delayValue);
            else
                Config.SetSequenceDecouplerOverride(vehicle, sequence, delayValue);
        }
        if (ImGui.IsItemDeactivatedAfterEdit())
            Config.FlushPendingSaves();
        ImGui.SameLine();
        ImGui.TextDisabled("seconds"u8);

        if (hasOverride)
        {
            string source = string.Format(CultureInfo.InvariantCulture,
                "override (default: {0:F1} s)", partDefault);
            ImGui.TextColored(new float4(1f, 0.8f, 0.2f, 1f), source);

            if (ImGui.SmallButton("Reset to default"u8))
            {
                if (kind == DelayKind.Engine)
                    Config.ClearSequenceEngineOverride(vehicle, sequence);
                else
                    Config.ClearSequenceDecouplerOverride(vehicle, sequence);
                Config.FlushPendingSaves();
            }
        }
        else
        {
            string source = string.Format(CultureInfo.InvariantCulture,
                "part default ({0:F1} s)", partDefault);
            ImGui.TextDisabled(source);
        }

        ImGui.Spacing();
        ImGui.PopID();
    }

    private static void DrawCoveredModules(Part part, int sequence, DelayKind kind)
    {
        string? covered = null;
        int extra = 0;
        foreach (ISequenced module in part.InSequence(sequence))
        {
            if (!SequencedModules.Matches(module, kind)) continue;
            if (covered == null)
                covered = SequencedModules.Describe(module);
            else
                extra++;
        }

        if (covered == null)
            return;

        ImGui.TextDisabled(extra > 0
            ? string.Format(CultureInfo.InvariantCulture, "fires {0} and {1} more", covered, extra)
            : string.Format(CultureInfo.InvariantCulture, "fires {0}", covered));
    }
}
