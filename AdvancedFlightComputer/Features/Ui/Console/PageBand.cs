using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

internal static class PageBand
{
    private static float InnerGap => 4f * ImGuiHelper.InterfaceScale;

    public static float Height
    {
        get
        {
            ConsoleUi.PushTitle();
            float title = ImGui.GetFontSize();
            ConsoleStyle.PopFont();
            return InnerGap * 2f + title;
        }
    }

    public static void DrawStrip(ImDrawListPtr dl, float left, float right, float top, float height)
    {
        dl.AddRectFilled(new float2(left, top), new float2(right, top + height), ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelRaisedU32));
        uint hair = ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32);
        dl.AddLine(new float2(left, top), new float2(right, top), hair, ConsoleStyle.WindowBorderThicknessPx);
        dl.AddLine(new float2(left, top + height), new float2(right, top + height), hair, ConsoleStyle.WindowBorderThicknessPx);
    }

    public static void DrawRule(ImDrawListPtr dl, float left, float right, float y)
        => dl.AddLine(new float2(left, y), new float2(right, y), ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32), ConsoleStyle.WindowBorderThicknessPx);

    public static void Draw(ConsolePage page, float2 origin, float width)
    {
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float y = origin.Y + InnerGap;

        ConsoleUi.PushTitle();
        string name = ConsolePages.Label(page);
        dl.AddText(new float2(origin.X, y), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32), name);
        ConsoleStyle.PopFont();

        ConsoleUi.NoteRow("AfcPageBand", origin, new float2(width, Height));
    }
}
