using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The place of the console. By default it docks where the stock Transfer Planner opens (see DockLayout), and a place the player gives it is saved in ui.toml, because ConsoleStyle.BeginWindowCore adds NoSavedSettings.</summary>
internal static class Dock
{
    private const float MinVisiblePx = 64f;
    private const string BurnPanelId = "BurnControl";

    // The stock panels under the console in the right column (Gauges.xml), which show with their context: BurnControl with a planned burn, RendezvousControl with a target, AutopilotSettings on a craft.
    private static readonly string[] PanelsBelow = { BurnPanelId, "RendezvousControl", "AutopilotSettings" };

    private static bool _oldSizeDone;
    private static bool _wideGrowsRight;
    private static float _placedLeft = float.NaN;

    public static void Reset()
    {
        _oldSizeDone = false;
        _wideGrowsRight = false;
        _placedLeft = float.NaN;
    }

    /// <summary>Sets the next window position and returns the size for ConsoleStyle.BeginWindow with ImGuiCond.Always.</summary>
    public static float2 Place(string key, bool wide, bool force, float height)
    {
        ForgetOldSizeOnce(key);
        float scale = ImGuiHelper.InterfaceScale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        DockRect dock = DockLayout.Console(main.Pos.X, main.Pos.Y, main.Size.X, main.Size.Y, scale, wide: false);

        float right = UiConfig.GetFloat(key + ".right", dock.Right);
        float top = UiConfig.GetFloat(key + ".y", dock.Y);
        float narrowW = UiConfig.GetFloat(key + ".w", dock.W);
        float h = float.IsFinite(height) ? height : dock.H;

        (float minW, float maxW) = DockLayout.WidthLimits(scale, wide, main.Size.X);
        float wideW = UiConfig.GetFloat(key + ".ww", DockLayout.WideWidth * scale);
        float w = Math.Clamp(wide ? wideW : narrowW, minW, maxW);
        h = Math.Max(DockLayout.MinHeight * scale, h);

        float2 workMin = main.WorkPos;
        float2 workMax = main.WorkPos + main.WorkSize;
        float x;
        if (wide)
        {
            (float narrowMin, float narrowMax) = DockLayout.WidthLimits(scale, false, main.Size.X);
            float narrow = Math.Clamp(narrowW, narrowMin, narrowMax);
            _wideGrowsRight = DockLayout.WideGrowsRight(right - narrow, right, workMin.X, workMax.X);
            x = DockLayout.WideLeft(right, narrow, w, workMin.X, workMax.X);
        }
        else
            x = Math.Clamp(right - w, workMin.X - w + MinVisiblePx, Math.Max(workMin.X, workMax.X - MinVisiblePx));
        float y = Math.Clamp(top, workMin.Y, Math.Max(workMin.Y, workMax.Y - MinVisiblePx));
        if (force || !float.IsFinite(_placedLeft))
            _placedLeft = x;

        ImGui.SetNextWindowPos(new float2(x, y), force ? ImGuiCond.Always : ImGuiCond.Appearing, (float2?)null);
        // ImGui truncates a size it is given, so a whole height comes back unchanged and any difference is the resize handle.
        return new float2(w, MathF.Ceiling(h));
    }

    /// <summary>A ui.toml can hold a height and a width that were written on every frame and are no choice of the player.</summary>
    public static void ForgetOldSizeOnce(string key)
    {
        if (_oldSizeDone)
            return;
        _oldSizeDone = true;
        string marker = key + ".h_is_maximum";
        if (UiConfig.GetBool(marker, false))
            return;
        UiConfig.Remove(key + ".h");
        if (MathF.Abs(UiConfig.GetFloat(key + ".w", float.NaN) - DockLayout.OldNarrowWidth * ImGuiHelper.InterfaceScale) <= 1f)
            UiConfig.Remove(key + ".w");
        UiConfig.Set(marker, true);
    }

    public static (float Left, float Width) NarrowPlace(string key)
    {
        float scale = ImGuiHelper.InterfaceScale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        DockRect dock = DockLayout.Console(main.Pos.X, main.Pos.Y, main.Size.X, main.Size.Y, scale, wide: false);
        (float minW, float maxW) = DockLayout.WidthLimits(scale, false, main.Size.X);
        float width = Math.Clamp(UiConfig.GetFloat(key + ".w", dock.W), minW, maxW);
        return (UiConfig.GetFloat(key + ".right", dock.Right) - width, width);
    }

    public static void Constrain(bool wide)
    {
        float scale = ImGuiHelper.InterfaceScale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        (float minW, float maxW) = DockLayout.WidthLimits(scale, wide, main.Size.X);
        float2 size = ImGui.GetWindowSize();
        float w = Math.Clamp(size.X, minW, maxW);
        float h = Math.Max(DockLayout.MinHeight * scale, size.Y);
        if (MathF.Abs(w - size.X) > 0.5f || MathF.Abs(h - size.Y) > 0.5f)
            ImGui.SetWindowSize(new float2(w, h));
    }

    public static void Record(string key, bool wide)
    {
        float2 pos = ImGui.GetWindowPos();
        float2 size = ImGui.GetWindowSize();
        Store(key + ".y", pos.Y);
        if (!wide)
        {
            Store(key + ".right", pos.X + size.X);
            Store(key + ".w", size.X);
            _placedLeft = pos.X;
            return;
        }
        Store(key + ".ww", size.X);
        // ImGui truncates a position it is given, so a difference under a pixel is no move.
        if (MathF.Abs(pos.X - _placedLeft) <= 1.5f)
            return;
        _placedLeft = pos.X;
        float scale = ImGuiHelper.InterfaceScale;
        (float minW, float maxW) = DockLayout.WidthLimits(scale, false, ImGui.GetMainViewport().Size.X);
        float narrow = Math.Clamp(UiConfig.GetFloat(key + ".w", DockLayout.NarrowWidth * scale), minW, maxW);
        Store(key + ".right", DockLayout.NarrowRight(pos.X, size.X, narrow, _wideGrowsRight));
    }

    public static void RecordMaximumHeight(string key, float height) => Store(key + ".h", height);

    public static void ForgetPlacement(string key)
    {
        UiConfig.Remove(key + ".right");
        UiConfig.Remove(key + ".y");
        UiConfig.Remove(key + ".w");
        UiConfig.Remove(key + ".ww");
        UiConfig.Remove(key + ".h");
    }

    /// <summary>Whether the pilot dragged the height, which the window then keeps.</summary>
    public static bool PilotHeight(string key) => float.IsFinite(UiConfig.GetFloat(key + ".h", float.NaN));

    /// <summary>The heights the window being drawn may take at its present place, over the stock panels that are on the screen now.</summary>
    public static (float Floor, float Limit) HeightRange(string key, in ConsoleLayout layout, float rows)
    {
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        float left = layout.WindowPos.X;
        Span<DockRect> panels = stackalloc DockRect[PanelsBelow.Length];
        int count = 0;
        foreach (string id in PanelsBelow)
        {
            if (Panel(id, whileHidden: false) is DockRect panel)
                panels[count++] = panel;
        }
        return DockLayout.HeightRange(layout.WindowPos.Y, left, left + layout.WindowSize.X, main.Pos.Y + main.Size.Y,
            ImGuiHelper.InterfaceScale, panels.Slice(0, count), UiConfig.GetFloat(key + ".h", float.NaN), rows);
    }

    /// <summary>While <paramref name="roomForMore"/> holds, the form is open and the page may grow past the place of a stock burn panel that hides.</summary>
    public static (float Floor, float Limit) FlyingHeightRange(in ConsoleLayout layout, float limit, bool roomForMore, float rows)
    {
        float left = layout.WindowPos.X;
        return DockLayout.FlyingHeightRange(limit, layout.WindowPos.Y, left, left + layout.WindowSize.X, ImGuiHelper.InterfaceScale,
            roomForMore ? null : Panel(BurnPanelId, whileHidden: true), rows);
    }

    // A stock gauge on the screen as GaugeCanvas.OnDrawUi places it, or null while it is off.
    private static DockRect? Panel(string id, bool whileHidden)
    {
        if (Program.IsEditorOpen)
            return null;
        try
        {
            if (!whileHidden && Program.Instance?.ShouldDrawVehicleCanvas() != true)
                return null;
            foreach (GaugeCanvas canvas in GaugeCanvas.AllCanvases)
            {
                if (canvas.Id != id)
                    continue;
                if (!canvas.Enabled || (!whileHidden && !canvas.IsContextVisible()))
                    return null;
                float2 min = ImGui.GetMainViewport().Pos + canvas.GetPixelsMin();
                float2 size = canvas.GetPixelsSize();
                return size.X > 0f && size.Y > 0f ? new DockRect(min.X, min.Y, size.X, size.Y) : null;
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-dock-panel:" + ex.GetType().Name, $"[AFC] Reading the place of the {id} panel failed: {ex.Message}");
        }
        return null;
    }

    private static void Store(string key, float value)
    {
        if (!float.IsFinite(value))
            return;
        float old = UiConfig.GetFloat(key, float.NaN);
        if (float.IsNaN(old) || MathF.Abs(old - value) > 1f)
            UiConfig.Set(key, MathF.Round(value));
    }
}
