using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Delay settings in the pinned part window, one block per (module kind, sequence) the part fires in.
// A tower with a motor and two mounts on three rows gets three blocks.
[HarmonyPatch(typeof(Part), nameof(Part.DrawPartInfo), new Type[0])]
internal static class StagingDelayPartWindow
{
    // Shared scratch, safe because Part.DrawPartInfo never nests.
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
            LogHelper.WarnOnce("autostage-part-window", $"[AFC] AutoStage part window draw failed for '{__instance.Id}': {ex}");
        }
    }

    private static void DrawDelays(Part part)
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null || part.Tree != vehicle.Parts)
            return;

        CollectSequences(part);
        if (_engineSequences.Count == 0 && _decouplerSequences.Count == 0)
            return;

        StagingConfig.LoadVehicleOverrides(vehicle.Id);

        ImGui.Spacing();
        ImGui.Separator();
        ImGui.Spacing();

        foreach (int sequence in _engineSequences)
            DrawDelayBlock(part, vehicle, sequence, DelayKind.Engine, "IGNITION DELAY");
        foreach (int sequence in _decouplerSequences)
            DrawDelayBlock(part, vehicle, sequence, DelayKind.Decoupler, "DECOUPLER DELAY");

        if (ImGui.SmallButton("OPEN AFC##AutoStageDelays"u8))
            Ui.Console.Pages.StagingPage.OpenWithDelays();
    }

    // Ascending, deduplicated. Sequence 0 means "no row".
    private static void CollectSequences(Part part)
    {
        _engineSequences.Clear();
        _decouplerSequences.Clear();

        foreach (ISequenced module in part.GetSubtreeSequencedModules())
        {
            int sequence = module.Sequence;
            if (sequence <= 0)
                continue;

            List<int>? target =
                module is EngineController ? _engineSequences :
                module is Decoupler ? _decouplerSequences : null;
            if (target != null && !target.Contains(sequence))
                target.Add(sequence);
        }

        _engineSequences.Sort();
        _decouplerSequences.Sort();
    }

    private static void DrawDelayBlock(Part part, Vehicle vehicle, int sequence, DelayKind kind, string title)
    {
        bool engine = kind == DelayKind.Engine;
        double effectiveDelay = engine
            ? StagingConfig.GetSequenceEngineDelay(vehicle, sequence)
            : StagingConfig.GetSequenceDecouplerDelay(vehicle, sequence);
        double partDefault = engine
            ? StagingConfig.ComputeSequenceEngineDelay(vehicle, sequence)
            : StagingConfig.ComputeSequenceDecouplerDelay(vehicle, sequence);
        bool hasOverride = engine
            ? StagingConfig.HasSequenceEngineOverride(vehicle, sequence)
            : StagingConfig.HasSequenceDecouplerOverride(vehicle, sequence);

        // The sequence is part of the id, or two blocks of one kind would both edit whichever drew first.
        ImGui.PushID(string.Format(CultureInfo.InvariantCulture, "AutoStageDelay_{0}_{1}", kind, sequence));
        ImGui.TextWrapped(Core.ConsoleUi.EscapeFormat(string.Format(CultureInfo.InvariantCulture, "{0}, SEQ {1}", title, sequence)));
        DrawCoveredModules(part, sequence, kind);
        ImGui.Spacing();

        float delayValue = (float)effectiveDelay;
        ImGui.SetNextItemWidth(120f);
        if (ImGui.InputFloat("###val"u8, ref delayValue, 0.1f, 1.0f, "%.1f"))
        {
            if (engine)
                StagingConfig.SetSequenceEngineOverride(vehicle, sequence, delayValue);
            else
                StagingConfig.SetSequenceDecouplerOverride(vehicle, sequence, delayValue);
        }
        if (ImGui.IsItemDeactivatedAfterEdit())
            StagingConfig.FlushPendingSaves();
        ImGui.SameLine();
        ImGui.TextDisabled("s"u8);

        if (hasOverride)
        {
            ImGui.TextDisabled(string.Format(CultureInfo.InvariantCulture, "Changed, default {0:F1} s", partDefault));
            if (ImGui.SmallButton("RESET TO DEFAULT"u8))
            {
                if (engine)
                    StagingConfig.ClearSequenceEngineOverride(vehicle, sequence);
                else
                    StagingConfig.ClearSequenceDecouplerOverride(vehicle, sequence);
                StagingConfig.FlushPendingSaves();
            }
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
            if (!SequencedModules.Matches(module, kind))
                continue;
            if (covered == null)
                covered = SequencedModules.Describe(module);
            else
                extra++;
        }

        if (covered == null || extra == 0)
            return;

        ImGui.TextDisabled(string.Format(CultureInfo.InvariantCulture, "Fires {0} and {1} more", covered, extra));
    }
}
