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
/// The green band of a coming separation: SEPARATION T-, in the last seconds before a stage comes off (StagingCountdown.BandFromS), as its own small window where the failure band stands, under the stock altitude counter, so it shows with the console closed. Green, not red: a separation is the flight going to plan. Under a failure band when both show.
/// </summary>
internal static class SeparationBandWindow
{
    private const string WindowId = "afc-separation-band";
    private const float PaddingPx = 8f;
    private const float GapBelowFailurePx = 4f;

    /// <summary>The band's bottom edge as last drawn, NaN when it did not show: the aero data band stands under it (AeroDataBandWindow).</summary>
    internal static float DrawnBottom = float.NaN;

    public static void Draw(ConsoleFrame f)
    {
        DrawnBottom = float.NaN;
        if (f.Subject == null)
            return;
        double seconds = GuidanceWindow.SeparationSecondsOf(f.Subject);
        if (!StagingCountdown.BandShows(seconds))
            return;
        string text = StagingCountdown.BandText(seconds);

        ConsoleStyle.EnsureInitialized();
        float scale = ImGuiHelper.InterfaceScale;
        float pad = PaddingPx * scale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        ConsoleStyle.PushValueFont();
        float2 textSize = ImGui.CalcTextSize(text);
        ConsoleStyle.PopFont();

        var size = new float2(pad + textSize.X + pad, Math.Max(ConsoleStyle.FrameHeightPx, textSize.Y) + pad);
        (float centerX, float top) = FailureBandWindow.Anchor(in main, scale);
        if (float.IsFinite(FailureBandWindow.DrawnBottom))
            top = FailureBandWindow.DrawnBottom + GapBelowFailurePx * scale;
        float x = Math.Clamp(centerX - size.X * 0.5f, main.Pos.X, Math.Max(main.Pos.X, main.Pos.X + main.Size.X - size.X));
        var min = new float2(x, top);
        DrawnBottom = min.Y + size.Y;

        // Steady until the last seconds, then it pulses, once a second.
        float alpha = StagingCountdown.BandPulses(seconds) ? 0.65f + 0.35f * MathF.Cos((float)ImGui.GetTime() * MathF.Tau) : 1f;

        ImGui.SetNextWindowPos(min);
        ImGui.SetNextWindowSize(size);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, float2.Zero);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, 0f);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowMinSize, float2.Zero);
        ImGui.Begin(WindowId, FailureBandWindow.Flags);
        try
        {
            ImGui.GetWindowDrawList().AddRectFilled(min, min + size, ConsoleStyle.ApplyAlpha(Palette.Good.U32At(alpha)), ConsoleStyle.FrameRoundingPx);
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
