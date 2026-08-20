using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// Shared plumbing for panels built on KSA.ImGauge — the immediate-mode face of the
// gauge renderer the game's own HUD uses.
//
// The shell pattern these support is stock's own, from KSA.GameSettings: an ImGauge
// window for the chrome (dressed box, screws, header), then an ordinary ImGui body
// wrapped in ImGaugeDressing.PushGaugeWidgetStyle so plain widgets take the gauge
// palette, with KSA.ImGuiHelper.BeginRegion for collapsible two-column sections.
//
// Gotchas the signatures don't tell you:
//   * Label and Button take at most 16 chars; Label THROWS on the 17th.
//   * The gauge font is uppercase A-Z, 0-9 and " . - + / \ _" only — no '%', no ':'.
//     (This applies to ImGauge primitives, NOT to ordinary ImGui text in the body.)
//   * Offset and size are normalised to screen WIDTH on BOTH axes.
//   * Button is cursor-relative; Box, Label and Screw are absolute.
//   * BeginWindow hardcodes NoMove, so dragging means moving OffsetUv ourselves.
public static partial class PoweredGuidanceWindow
{
    // Layout constants, matching ImGaugeDressing's so a hand-rolled panel lines up
    // with the stock popups. All are fractions of one canvas unit.
    private const float GaugeMarginUv = 0.0114f;
    private const float GaugeHeaderTopUv = 0.019f;
    private const float GaugeHeaderHeightUv = 0.01577f;
    private const float GaugeSpacingUv = 0.0076f;
    private const float GaugeBottomMarginUv = 0.019f;

    /// <summary>One canvas unit in pixels — the screen width, for both axes.</summary>
    private static float GaugeUnit() => ScreenReference.UvToPixels(new float2(1f, 0f)).X;

    /// <summary>Clamps to what the gauge font can pack. Label throws past 16.</summary>
    private static string Gauge(string s)
    {
        s = s.ToUpperInvariant();
        return s.Length > 16 ? s.Substring(0, 16) : s;
    }

    // A Box reads only BackgroundColor.rgb; a Label reads TextColor plus
    // BackgroundColor including its alpha, and Default's is transparent.
    private static ImGaugeStyle GaugeFill(float3 rgb) => new ImGaugeStyle(
        ImGaugeStyle.Default.TextColor, new float4(rgb, 1f),
        ImGaugeStyle.Default.IdleColor, ImGaugeStyle.Default.ActiveColor,
        ImGaugeStyle.Default.TextScale, false);

    private static ImGaugeStyle GaugeText(float3 rgb, float scale)
        => ImGaugeStyle.Default.WithText(rgb, scale);

    /// <summary>
    /// A button face in a chosen colour. ActiveColor is the HOVER colour and the
    /// pressed look lasts only while held, so a lit/latched button means setting
    /// IdleColor — that is the only channel that survives between frames.
    /// </summary>
    private static ImGaugeStyle GaugeButton(float3 idle, float textScale) => new ImGaugeStyle(
        new float3(0f, 0f, 0f), ImGaugeStyle.Default.BackgroundColor,
        idle, ImGaugeStyle.Default.ActiveColor, textScale, false);

    /// <summary>
    /// The drag ImGauge doesn't provide: an invisible ImGui button over a handle
    /// area, feeding the mouse delta back into the window's OffsetUv. Clamped to the
    /// viewport — there is no layout save on this path, so a panel dragged off screen
    /// would be gone for good.
    /// </summary>
    private static void GaugeDrag(string id, ref float2 offsetUv, float2 pos, float2 size,
                                  float2 handlePos, float2 handleSize)
    {
        ImGui.SetCursorScreenPos(handlePos);
        ImGui.InvisibleButton(id, handleSize);
        if (!ImGui.IsItemActive())
            return;

        float2 d = ImGui.GetMouseDragDelta(ImGuiMouseButton.Left, 0f);
        if (d.X == 0f && d.Y == 0f)
            return;
        ImGui.ResetMouseDragDelta(ImGuiMouseButton.Left);

        ImGuiViewportPtr vp = ImGui.GetMainViewport();
        float2 vpPos = vp.Pos, vpSize = vp.Size;
        float x = Math.Clamp(pos.X + d.X, vpPos.X, Math.Max(vpPos.X, vpPos.X + vpSize.X - size.X));
        float y = Math.Clamp(pos.Y + d.Y, vpPos.Y, Math.Max(vpPos.Y, vpPos.Y + vpSize.Y - size.Y));

        // Offset moves 1:1 with pixels, scaled into canvas units.
        float unit = GaugeUnit();
        offsetUv = new float2(offsetUv.X + (x - pos.X) / unit, offsetUv.Y + (y - pos.Y) / unit);
    }

    // --- two-column rows ----------------------------------------------------
    //
    // ImGuiHelper.BeginRegion puts the body in two columns — label at 33%, control
    // at 67% — and its own widgets follow the idiom below. These match it for the
    // types it doesn't cover: it has DrawFloat but nothing for double, and the
    // guidance state is double throughout.
    //
    // Ids are explicit rather than ImGuiHelper's _widgetId counter, because that
    // counter is global and reset by ImGuiHelper.StartFrame, which the game's own
    // windows call — a mod incrementing it would collide with them.

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
