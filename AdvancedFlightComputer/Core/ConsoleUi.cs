using System;
using System.Collections.Generic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Label-and-control rows built on the game's ConsoleWidgets, mirroring the
/// private row helpers stock uses in TransferPlanner. The mod's UI is drawn
/// inside stock windows (or windows shaped like them), so it lays out through
/// the same widgets rather than raw ImGui, which would render at a different
/// size, color and column position than everything around it.
///
/// Labels are passed through as given; stock writes them upper case.
/// </summary>
internal static class ConsoleUi
{
    /// <summary>One-line status text in the muted body colour, for the
    /// "nothing to do here" cases that are not a row.</summary>
    public static void Muted(ReadOnlySpan<char> text)
    {
        ConsoleStyle.PushValueFont();
        ImGui.TextColored(in ConsoleStyle.TextMuted, text);
        ConsoleStyle.PopFont();
    }

    public static void Positive(ReadOnlySpan<char> text)
    {
        ConsoleStyle.PushValueFont();
        ImGui.TextColored(in ConsoleStyle.Positive, text);
        ConsoleStyle.PopFont();
    }

    public static void Warning(ReadOnlySpan<char> text)
    {
        ConsoleStyle.PushValueFont();
        ImGui.TextColored(in ConsoleStyle.Pending, text);
        ConsoleStyle.PopFont();
    }

    public static void DangerWrapped(string text)
    {
        ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.Danger);
        ImGui.TextWrapped(text);
        ImGui.PopStyleColor();
    }

    public static void WarningWrapped(string text)
    {
        ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.Pending);
        ImGui.TextWrapped(text);
        ImGui.PopStyleColor();
    }

    public static void MutedWrapped(string text)
    {
        ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.TextMuted);
        ImGui.TextWrapped(text);
        ImGui.PopStyleColor();
    }

    public static bool CheckboxRow(ReadOnlySpan<char> label, ReadOnlySpan<char> id, ref bool value,
        ReadOnlySpan<char> tooltip = default)
    {
        ConsoleWidgets.BeginRow(label);
        bool changed = ConsoleWidgets.Checkbox(id, ref value, pending: false);
        if (tooltip.Length > 0 && ConsoleWidgets.RowHovered)
            ConsoleWidgets.Tooltip(tooltip);
        ConsoleWidgets.EndRow();
        return changed;
    }

    public static bool ComboRow<T>(ReadOnlySpan<char> label, ReadOnlySpan<char> id,
        ref T value, IReadOnlyList<T> options) where T : IComboable
    {
        ConsoleWidgets.BeginRow(label);
        bool changed = ComboControl(id, ref value, options);
        ConsoleWidgets.EndRow();
        return changed;
    }

    private static bool ComboControl<T>(ReadOnlySpan<char> id, ref T value,
        IReadOnlyList<T> options) where T : IComboable
    {
        bool changed = false;
        int current = -1;
        for (int i = 0; i < options.Count; i++)
        {
            if (options[i].GetKey().Equals(value.GetKey(), StringComparison.OrdinalIgnoreCase))
            {
                current = i;
                value = options[i];
                break;
            }
        }
        if (current < 0 && options.Count > 0)
            current = 0;

        string preview = current >= 0 ? options[current].GetName() : "N/A";
        if (!ConsoleWidgets.BeginComboControl(id, preview.AsSpan(), pending: false))
            return false;

        for (int i = 0; i < options.Count; i++)
        {
            T option = options[i];
            bool selected = option.GetKey().Equals(value.GetKey(), StringComparison.OrdinalIgnoreCase);
            if (ImGui.Selectable(option.GetName(), selected, ImGuiSelectableFlags.None, (float2?)null))
            {
                value = option;
                changed = true;
            }
        }
        ConsoleWidgets.EndComboControl();
        return changed;
    }

    /// <summary>String-list combo for options that are not IComboable. Returns
    /// the newly selected index, or -1 when nothing was picked this frame.</summary>
    public static int ComboRow(ReadOnlySpan<char> label, ReadOnlySpan<char> id,
        int activeIndex, IReadOnlyList<string> options)
    {
        ConsoleWidgets.BeginRow(label);
        int picked = -1;
        string preview = (activeIndex >= 0 && activeIndex < options.Count)
            ? options[activeIndex] : "N/A";
        if (ConsoleWidgets.BeginComboControl(id, preview.AsSpan(), pending: false))
        {
            for (int i = 0; i < options.Count; i++)
            {
                if (ImGui.Selectable(options[i], i == activeIndex, ImGuiSelectableFlags.None, (float2?)null))
                    picked = i;
            }
            ConsoleWidgets.EndComboControl();
        }
        ConsoleWidgets.EndRow();
        return picked;
    }

    /// <summary>InputDouble hosted in a row. Kept an InputDouble rather than a
    /// ConsoleWidgets drag control because these fields take typed absolute
    /// values (altitudes, angles) that a drag range cannot express.</summary>
    public static bool InputDoubleRow(ReadOnlySpan<char> label, ImString id, ref double value,
        double step, double stepFast, ImString format)
    {
        ConsoleWidgets.BeginRow(label);
        ImGui.SetNextItemWidth(ConsoleWidgets.RowControlWidth);
        bool changed = ImGui.InputDouble(id, ref value, step, stepFast, format,
            ImGuiInputTextFlags.CharsDecimal);
        ConsoleWidgets.EndRow();
        return changed;
    }

    public static bool InputIntRow(ReadOnlySpan<char> label, ImString id, ref int value,
        int step, int stepFast)
    {
        ConsoleWidgets.BeginRow(label);
        ImGui.SetNextItemWidth(ConsoleWidgets.RowControlWidth);
        bool changed = ImGui.InputInt(id, ref value, step, stepFast, ImGuiInputTextFlags.CharsDecimal);
        ConsoleWidgets.EndRow();
        return changed;
    }
}
