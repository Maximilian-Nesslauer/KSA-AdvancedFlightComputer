using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>
/// The amber band while the craft's aero data is being made: GENERATING AERO DATA with an estimate of the time left (KsaAeroSweep.AeroDataRemaining), or LOADING AERO DATA while a cached or saved sweep is handed over. A mode that starts on the body table flies without its fins or flaps until it is done, so it says so where the separation band stands, under the stock altitude counter, with the console closed too (2026-11-25, the user's ask). Amber: a wait, not a fault. Under the failure and separation bands when they show.
/// </summary>
internal static class AeroDataBandWindow
{
    private const string WindowId = "afc-aero-data-band";
    private const float PaddingPx = 8f;
    private const float GapPx = 4f;

    public static void Draw(ConsoleFrame f)
    {
        if (f.Subject == null)
            return;
        (double seconds, bool loading) = KsaAeroSweep.AeroDataRemaining(f.Subject.Id);
        if (!(seconds > 0.0))
            return;
        string text = loading
            ? "LOADING AERO DATA"
            : "GENERATING AERO DATA: READY IN ~" + Fmt.Time(Math.Ceiling(seconds)) + " (ESTIMATE)";

        ConsoleStyle.EnsureInitialized();
        float scale = ImGuiHelper.InterfaceScale;
        float pad = PaddingPx * scale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        ConsoleStyle.PushValueFont();
        float2 textSize = ImGui.CalcTextSize(text);
        ConsoleStyle.PopFont();

        var size = new float2(pad + textSize.X + pad, Math.Max(ConsoleStyle.FrameHeightPx, textSize.Y) + pad);
        (float centerX, float top) = FailureBandWindow.Anchor(in main, scale);
        float below = float.IsFinite(SeparationBandWindow.DrawnBottom) ? SeparationBandWindow.DrawnBottom
            : FailureBandWindow.DrawnBottom;
        if (float.IsFinite(below))
            top = below + GapPx * scale;
        float x = Math.Clamp(centerX - size.X * 0.5f, main.Pos.X, Math.Max(main.Pos.X, main.Pos.X + main.Size.X - size.X));
        var min = new float2(x, top);

        ImGui.SetNextWindowPos(min);
        ImGui.SetNextWindowSize(size);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, float2.Zero);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, 0f);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowMinSize, float2.Zero);
        ImGui.Begin(WindowId, FailureBandWindow.Flags);
        try
        {
            ImGui.GetWindowDrawList().AddRectFilled(min, min + size, ConsoleStyle.ApplyAlpha(Palette.Warn.U32At(1f)), ConsoleStyle.FrameRoundingPx);
            ImGui.SetCursorScreenPos(new float2(min.X + pad, min.Y + (size.Y - textSize.Y) * 0.5f));
            ConsoleStyle.PushValueFont();
            ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.Panel);
            ImGui.Text(text);
            ImGui.PopStyleColor();
            ConsoleStyle.PopFont();
        }
        finally
        {
            ImGui.End();
            ImGui.PopStyleVar(3);
        }
    }
}
