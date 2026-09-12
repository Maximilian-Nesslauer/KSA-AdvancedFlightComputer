#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The two-column rows the guidance tabs are built from.
//
// ImGuiHelper.BeginRegion puts the body in two columns, the label at 33 percent and the control
// at 67 percent, and its own widgets follow the idiom below. These match it for the types it does
// not cover: it has DrawFloat but nothing for double, and the guidance state is double throughout.
//
// Ids are explicit rather than ImGuiHelper's _widgetId counter, because that counter is global and
// reset by ImGuiHelper.StartFrame, which the game's own windows call, so a mod incrementing it
// would collide with them.
public static partial class GuidanceWindow
{
    private static bool GaugeRow(string label, string id, ref double value)
    {
        ImGui.Text(label);
        ImGui.NextColumn();
        ImGui.PushItemWidth(-1f);
        bool changed = ImGui.InputDouble(id, ref value);
        ImGui.PopItemWidth();
        ImGui.NextColumn();
        return changed;
    }

    private static bool GaugeRowCheck(string label, string id, ref bool value)
    {
        ImGui.Text(label);
        ImGui.NextColumn();
        bool changed = ImGui.Checkbox(id, ref value);
        ImGui.NextColumn();
        return changed;
    }

    private static void GaugeRowText(string label, string value)
    {
        ImGui.Text(label);
        ImGui.NextColumn();
        ImGui.Text(value);
        ImGui.NextColumn();
    }

    private static void GaugeRowText(string label, string value, float4 colour)
    {
        ImGui.Text(label);
        ImGui.NextColumn();
        ImGui.TextColored(colour, value);
        ImGui.NextColumn();
    }
}
