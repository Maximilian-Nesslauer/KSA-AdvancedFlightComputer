using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Ui.Canvas;

/// <summary>What the player did on a picture this frame.</summary>
internal struct CanvasInput
{
    public bool Hovered;
    public bool Click;
    public float2 Mouse;
}

/// <summary>
/// A zoomable view of a world rectangle with separate X and Y scales, because a flight profile is far
/// wider than it is high. Y grows upward on screen. The view fits itself to what the picture asks for
/// until the player zooms with the wheel, and a middle click gives the fit back.
/// One invisible button over the picture keeps a left drag from moving the window, and the picture
/// takes only the mouse wheel while it is hovered, never the keyboard: Program.OnKey drops every key
/// while ImGui wants the keyboard, so a picture that took it would also swallow F4.
/// </summary>
internal sealed class CanvasView
{
    private const double ZoomStep = 1.15;
    private const double MinSpan = 1e-3;

    public double X0, X1 = 1.0, Y0, Y1 = 1.0;
    public float2 Min, Max;

    /// <summary>True once the player zoomed. The fit then leaves the view alone.</summary>
    public bool UserView;

    public float Width => Max.X - Min.X;
    public float Height => Max.Y - Min.Y;
    public double ScaleX => Width / Math.Max(X1 - X0, MinSpan);
    public double ScaleY => Height / Math.Max(Y1 - Y0, MinSpan);

    public void SetRect(float2 min, float2 max)
    {
        Min = min;
        Max = max;
    }

    public float Sx(double x) => (float)(Min.X + (x - X0) * ScaleX);
    public float Sy(double y) => (float)(Max.Y - (y - Y0) * ScaleY);
    public float2 ToScreen(double x, double y) => new float2(Sx(x), Sy(y));
    public double ToWorldX(float sx) => X0 + (sx - Min.X) / ScaleX;
    public double ToWorldY(float sy) => Y0 + (Max.Y - sy) / ScaleY;

    /// <summary>Applies the picture's fit unless the player has zoomed.</summary>
    public void Fit(double x0, double x1, double y0, double y1)
    {
        if (UserView)
            return;
        X0 = x0;
        X1 = Math.Max(x1, x0 + MinSpan);
        Y0 = y0;
        Y1 = Math.Max(y1, y0 + MinSpan);
    }

    private void ZoomAbout(float2 screen, double factorX, double factorY)
    {
        double wx = ToWorldX(screen.X);
        X0 = wx - (wx - X0) / factorX;
        X1 = wx + (X1 - wx) / factorX;
        double wy = ToWorldY(screen.Y);
        Y0 = wy - (wy - Y0) / factorY;
        Y1 = wy + (Y1 - wy) / factorY;
        UserView = true;
    }

    /// <summary>Submits the invisible button over the picture and reads the wheel zoom and the middle-click fit. A zoomable picture says so in its head lane (Picture.ZoomHint).</summary>
    public CanvasInput Capture(ReadOnlySpan<byte> id, bool zoom)
    {
        var input = new CanvasInput { Mouse = ImGui.GetMousePos() };
        ImGui.SetCursorScreenPos(Min);
        float2 size = new float2(Math.Max(Width, 1f), Math.Max(Height, 1f));
        bool released = ImGui.InvisibleButton(id, in size);
        bool hovered = ImGui.IsItemHovered();
        input.Hovered = hovered;
        input.Click = released;
        if (!hovered || !zoom)
            return input;

        ImGui.SetItemKeyOwner(ImGuiKey.MouseWheelY);
        ImGuiIOPtr io = ImGui.GetIO();
        float wheel = io.MouseWheel;
        if (wheel != 0f)
        {
            io.MouseWheel = 0f;
            double f = Math.Pow(ZoomStep, wheel);
            ZoomAbout(input.Mouse, f, io.KeyShift ? 1.0 : f);
        }
        if (ImGui.IsMouseClicked(ImGuiMouseButton.Middle))
            UserView = false;
        return input;
    }

    public void Reset() => UserView = false;
}
