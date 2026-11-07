using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The red band of a failure, as its own small window under the stock altitude counter, so it shows with the console closed and leaves the counter free.</summary>
internal static class FailureBandWindow
{
    private const string WindowId = "afc-failure-band";
    private const string AltitudeCanvasId = "Altitude";
    private const float GapBelowCounter = 6f;
    private const float PaddingPx = 8f;

    // At interface scale 1, under the place of the counter in stock Gauges.xml, for a counter that does not draw.
    private const float TopWithoutCounter = 110f;

    private const ImGuiWindowFlags Flags = ImGuiWindowFlags.NoDecoration | ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoSavedSettings
        | ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoBackground;

    public static void Draw(ConsoleFrame f)
    {
        Vehicle? subject = f.Subject;
        if (subject == null || f.FailureWhat.Length == 0)
            return;

        BandFacts facts = f.FailureFacts();
        string text = FailureBand.Text(in facts);

        ConsoleStyle.EnsureInitialized();
        float scale = ImGuiHelper.InterfaceScale;
        float pad = PaddingPx * scale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        ConsoleStyle.PushWidgetStyle();
        try
        {
            float okW = ConsoleWidgets.ButtonWidth("OK".AsSpan());
            float wrap = Math.Max(1f, main.Size.X * 0.5f);
            ConsoleStyle.PushValueFont();
            float2 textSize = ImGui.CalcTextSize(text, false, wrap);
            ConsoleStyle.PopFont();

            var size = new float2(pad + textSize.X + pad + okW + pad, Math.Max(ConsoleStyle.FrameHeightPx, textSize.Y) + pad);
            (float centerX, float top) = Anchor(in main, scale);
            float x = Math.Clamp(centerX - size.X * 0.5f, main.Pos.X, Math.Max(main.Pos.X, main.Pos.X + main.Size.X - size.X));
            var min = new float2(x, top);

            ImGui.SetNextWindowPos(min);
            ImGui.SetNextWindowSize(size);
            ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, float2.Zero);
            ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, 0f);
            ImGui.PushStyleVar(ImGuiStyleVar.WindowMinSize, float2.Zero);
            ImGui.Begin(WindowId, Flags);
            try
            {
                ImGui.GetWindowDrawList().AddRectFilled(min, min + size, ConsoleStyle.ApplyAlpha(ConsoleStyle.DangerU32), ConsoleStyle.FrameRoundingPx);

                var textAt = new float2(min.X + pad, min.Y + (size.Y - textSize.Y) * 0.5f);
                ImGui.SetCursorScreenPos(textAt);
                ConsoleStyle.PushValueFont();
                ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + wrap);
                ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.Panel);
                ImGui.Text(text);
                ImGui.PopStyleColor();
                ImGui.PopTextWrapPos();
                ConsoleStyle.PopFont();
                if (f.FailureDetail.Length > 0 && ImGui.IsWindowHovered() && ImGui.IsMouseHoveringRect(in textAt, textAt + textSize))
                    ConsoleUi.Tooltip(Fmt.Sentence(f.FailureDetail).AsSpan());

                ImGui.SetCursorScreenPos(new float2(min.X + size.X - pad - okW, min.Y + (size.Y - ConsoleStyle.FrameHeightPx) * 0.5f));
                if (ConsoleUi.BandButton("OK", "AfcFailureOk", new float2(okW, ConsoleStyle.FrameHeightPx)))
                    AfcConsole.AcknowledgeFailure();
                if (ImGui.IsItemHovered())
                    ConsoleUi.Tooltip(PageText.FailureOkTip.AsSpan());
            }
            finally
            {
                ImGui.End();
                ImGui.PopStyleVar(3);
            }
        }
        finally
        {
            ConsoleStyle.PopWidgetStyle();
        }
    }

    // The center of the counter and the top of the band, from the place GaugeCanvas.OnDrawUi gives the stock Altitude gauge, so a counter the pilot moved keeps its band.
    private static (float CenterX, float Top) Anchor(in ImGuiViewportPtr main, float scale)
    {
        try
        {
            foreach (GaugeCanvas canvas in GaugeCanvas.AllCanvases)
            {
                if (canvas.Id != AltitudeCanvasId)
                    continue;
                if (!canvas.Enabled || !canvas.IsContextVisible())
                    break;
                float2 at = main.Pos + canvas.GetPixelsMin();
                float2 size = canvas.GetPixelsSize();
                if (size.X > 0f && size.Y > 0f)
                    return (at.X + size.X * 0.5f, at.Y + size.Y + GapBelowCounter * scale);
                break;
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-failure-anchor:" + ex.GetType().Name, $"[AFC] Reading the place of the altitude counter failed: {ex.Message}");
        }
        return (main.Pos.X + main.Size.X * 0.5f, main.Pos.Y + TopWithoutCounter * scale);
    }
}
