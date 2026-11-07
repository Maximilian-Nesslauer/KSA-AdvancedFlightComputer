using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Canvas;

/// <summary>The box a picture draws into this frame, with its draw list and the interface scale.</summary>
internal readonly struct PictureFrame
{
    public readonly float2 Min;
    public readonly float2 Max;
    public readonly ImDrawListPtr Dl;
    public readonly float Scale;

    public PictureFrame(float2 min, float2 max, ImDrawListPtr dl, float scale)
    {
        Min = min;
        Max = max;
        Dl = dl;
        Scale = scale;
    }

    public float Width => Max.X - Min.X;
    public float Height => Max.Y - Min.Y;
    public float2 Center => (Min + Max) * 0.5f;
    public PicBox Box => new(Min.X, Min.Y, Width, Height);
}

internal enum SwatchKind { Line, Thick, Dashed, LongDash, Dotted, Cross, Filled, Frame, Diamond, Ring, Tick, Beam }

internal readonly struct PlotClip : IDisposable
{
    private readonly ImDrawListPtr _dl;

    public PlotClip(ImDrawListPtr dl, in PicBox plot)
    {
        _dl = dl;
        dl.PushClipRect(new float2(plot.X, plot.Y), new float2(plot.Right, plot.Bottom), true);
    }

    public void Dispose() => _dl.PopClipRect();
}

/// <summary>Picture.Begin as it is made and Picture.End as it is disposed, for a using statement.</summary>
internal readonly struct PictureScope : IDisposable
{
    public readonly PictureFrame Frame;

    public PictureScope(float height) => Frame = Picture.Begin(height);

    public void Dispose() => Picture.End(in Frame);
}

/// <summary>The frame every console picture shares: small folder tabs on the top edge with the views of the picture, or its name in a plain caption where it has one view, the switches of the picture beside...</summary>
internal static class Picture
{
    // Where the active tab opens the top edge of the next frame.
    private static float _gapX0, _gapX1, _gapY;
    private static bool _gapSet;

    public static float EmptyHeight => DockLayout.EmptyPictureHeight * ImGuiHelper.InterfaceScale;

    public static bool Shows(string id, bool hasData)
    {
        ConsoleUi.NoteShape(id, hasData ? 1 : 0);
        return hasData;
    }

    /// <summary>The low frame of a picture with nothing to show, with one sentence.</summary>
    public static void EmptyState(string sentence)
    {
        using (PictureScope pic = Open(EmptyHeight))
        {
            Empty(in pic.Frame, sentence);
        }
    }

    public static PictureScope Open(float height) => new(height);

    // The room between a frame and what draws in it, on each side.
    private static float FrameInset => ConsoleStyle.BezelInsetPx + 1f;

    /// <summary>The height to give so the inside of the frame is <paramref name="inside"/> pixels high.</summary>
    public static float FrameHeight(float inside) => inside + FrameInset * 2f;

    /// <summary>A frame of the width <see cref="Begin"/> gives, to measure lanes before the height of a picture is known.</summary>
    public static PictureFrame Probe()
    {
        float inset = FrameInset;
        return new PictureFrame(default, new float2(Math.Max(40f * ImGuiHelper.InterfaceScale, ConsoleStyle.ContentAvailWidth()) - inset * 2f, 1f), ImGui.GetWindowDrawList(), ImGuiHelper.InterfaceScale);
    }

    /// <summary>Starts a picture of <paramref name="height"/> pixels at the full content width.</summary>
    public static PictureFrame Begin(float height)
    {
        float scale = ImGuiHelper.InterfaceScale;
        float w = Math.Max(40f * scale, ConsoleStyle.ContentAvailWidth());
        float2 min = ImGui.GetCursorScreenPos();
        float2 size = new float2(w, Math.Max(40f * scale, height));
        ConsoleWidgets.InputWell(min, size);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        if (_gapSet && Math.Abs(_gapY - min.Y) < 1.5f)
        {
            float border = ConsoleStyle.WindowBorderThicknessPx;
            dl.AddRectFilled(new float2(_gapX0 + border, min.Y - 1f), new float2(_gapX1 - border, min.Y + ConsoleStyle.BezelInsetPx + border + 1f), ConsoleStyle.ApplyAlpha(ConsoleStyle.InsetU32));
        }
        _gapSet = false;
        float inset = FrameInset;
        float2 innerMin = min + new float2(inset, inset);
        float2 innerMax = min + size - new float2(inset, inset);
        dl.PushClipRect(innerMin, innerMax, true);
        return new PictureFrame(innerMin, innerMax, dl, scale);
    }

    public static void End(in PictureFrame f)
    {
        f.Dl.PopClipRect();
        float inset = FrameInset;
        ImGui.SetCursorScreenPos(new float2(f.Min.X - inset, f.Max.Y + inset));
        ImGui.Dummy(new float2(f.Width + 2f * inset, 0f));
        ImGui.Spacing();
    }

    #region Lanes

    /// <summary>It holds no curve that needs room, so its low frame keeps its lanes.</summary>
    public const float StripPlot = 16f;

    public static PictureLanes Lanes(in PictureFrame f, float head, float left = 0f, float right = 0f, float bottom = 0f, float minPlot = 48f)
        => PictureLayout.Split(f.Box, head, left, right, bottom, minPlot * f.Scale);

    private static float SmallFontSize
    {
        get
        {
            ConsoleUi.PushSmall();
            float size = ImGui.GetFontSize();
            ConsoleStyle.PopFont();
            return size;
        }
    }

    public static float TextLaneHeight(int rows = 1) => (SmallFontSize + 3f * ImGuiHelper.InterfaceScale) * rows + 2f * ImGuiHelper.InterfaceScale;

    /// <summary>The height of the head lane: the big number if the picture has one, one row of small text if it has a short text or a hint for a zoom, and a thin margin otherwise.</summary>
    public static float HeadHeight(in PictureFrame f, string big, bool text = false)
    {
        if (!string.IsNullOrEmpty(big))
        {
            ScreenRect box = BigNumberBox(in f, big);
            return box.H + 2f * f.Scale;
        }
        return text ? TextLaneHeight() : 3f * f.Scale;
    }

    public static float TickWidth(string a, string b = "") => Math.Max(ConsoleUi.SmallWidth(a), ConsoleUi.SmallWidth(b)) + 8f * ImGuiHelper.InterfaceScale;

    private static float HeadRoom(in PictureFrame f, in PicBox head, string big)
        => string.IsNullOrEmpty(big) ? head.W - 8f * f.Scale : BigNumberBox(in f, big).X - head.X - 8f * f.Scale;

    public static bool HeadTextFits(in PictureFrame f, in PicBox head, string text, string big = "")
        => ConsoleUi.SmallWidth(text) <= HeadRoom(in f, in head, big);

    public static void HeadText(in PictureFrame f, in PicBox head, string text, ImColor8 color, string big = "")
    {
        if (string.IsNullOrEmpty(text) || head.H <= 0f)
            return;
        float room = HeadRoom(in f, in head, big);
        ConsoleUi.PushSmall();
        string shown = ConsoleUi.FitEllipsis(text, room);
        f.Dl.AddText(new float2(head.X + 4f * f.Scale, head.Y + (head.H - ImGui.GetFontSize()) * 0.5f), color, shown);
        ConsoleStyle.PopFont();
    }

    public static void ZoomHint(in PictureFrame f, in PicBox head, bool hovered, string big = "")
    {
        if (hovered)
            HeadText(in f, in head, PageText.ZoomHint, Palette.Dim.Solid, big);
    }

    public static void LaneText(in PictureFrame f, in PicBox lane, string text, ImColor8 color, int align = -1)
    {
        if (string.IsNullOrEmpty(text) || lane.H <= 0f || lane.W <= 0f)
            return;
        ConsoleUi.PushSmall();
        float pad = 4f * f.Scale;
        string shown = ConsoleUi.FitEllipsis(text, lane.W - pad * 2f);
        float w = ImGui.CalcTextSize(shown).X;
        float x = align < 0 ? lane.X + pad : align > 0 ? lane.Right - w - pad : lane.X + (lane.W - w) * 0.5f;
        f.Dl.AddText(new float2(x, lane.Y + (lane.H - ImGui.GetFontSize()) * 0.5f), color, shown);
        ConsoleStyle.PopFont();
    }

    public static void GuideY(in PictureFrame f, in PicBox plot, float y, ImColor8 color)
        => Glyphs.DashedLine(f.Dl, new float2(plot.X, y), new float2(plot.Right, y), color, 1f * f.Scale, 3f * f.Scale, 3f * f.Scale);

    #endregion

    #region Axes

    public static void AxisX(in PictureFrame f, in PictureLanes lanes, double lo, double hi, bool time, Func<double, float> toX, Func<double, string> text)
    {
        Span<double> ticks = stackalloc double[12];
        double step = time ? AxisTicks.TimeStep(hi - lo, 4) : AxisTicks.NiceStep(hi - lo, 4);
        int n = AxisTicks.Fill(lo, hi, step, ticks);
        ConsoleUi.PushSmall();
        float tickW = 0f;
        for (int i = 0; i < n; i++)
            tickW = Math.Max(tickW, ImGui.CalcTextSize(text(ticks[i])).X);
        ConsoleStyle.PopFont();
        int most = Math.Clamp((int)(lanes.Plot.W / (tickW + 14f * f.Scale)), 1, 8);
        step = time ? AxisTicks.TimeStep(hi - lo, most) : AxisTicks.NiceStep(hi - lo, most);
        n = AxisTicks.Fill(lo, hi, step, ticks);
        TicksX(in f, in lanes, ticks.Slice(0, n), toX, text);
    }

    public static void AxisY(in PictureFrame f, in PictureLanes lanes, double lo, double hi, Func<double, float> toY, Func<double, string> text, double step = double.NaN)
    {
        int most = Math.Clamp((int)(lanes.Plot.H / (SmallFontSize * 2.4f)), 1, 8);
        Span<double> ticks = stackalloc double[12];
        int n = AxisTicks.Fill(lo, hi, double.IsFinite(step) ? step : AxisTicks.NiceStep(hi - lo, most), ticks);
        TicksY(in f, in lanes, ticks.Slice(0, n), toY, text);
    }

    public static void TicksX(in PictureFrame f, in PictureLanes lanes, ReadOnlySpan<double> ticks, Func<double, float> toX, Func<double, string> text)
    {
        PicBox plot = lanes.Plot, lane = lanes.Bottom;
        float s = f.Scale;
        ConsoleUi.PushSmall();
        float lastLo = float.PositiveInfinity, lastHi = float.NegativeInfinity;
        float y = lane.Y + (lane.H - ImGui.GetFontSize()) * 0.5f;
        foreach (double v in ticks)
        {
            float x = toX(v);
            if (x < plot.X - 1f || x > plot.Right + 1f)
                continue;
            f.Dl.AddLine(new float2(x, plot.Bottom), new float2(x, plot.Bottom + 3f * s), Palette.Dim.A(0.8f), 1f * s);
            if (lane.H <= 0f)
                continue;
            string t = text(v);
            float w = ImGui.CalcTextSize(t).X;
            float at = Math.Clamp(x - w * 0.5f, lane.X + 2f * s, lane.Right - w - 2f * s);
            if (at < lastHi + 6f * s && at + w > lastLo - 6f * s)
                continue;
            f.Dl.AddText(new float2(at, y), Palette.Dim.Solid, t);
            lastLo = Math.Min(lastLo, at);
            lastHi = Math.Max(lastHi, at + w);
        }
        ConsoleStyle.PopFont();
    }

    public static void TicksY(in PictureFrame f, in PictureLanes lanes, ReadOnlySpan<double> ticks, Func<double, float> toY, Func<double, string> text)
    {
        PicBox plot = lanes.Plot, lane = lanes.Left;
        float s = f.Scale;
        ConsoleUi.PushSmall();
        float font = ImGui.GetFontSize();
        float lastY = float.NegativeInfinity;
        foreach (double v in ticks)
        {
            float y = toY(v);
            if (y < plot.Y - 1f || y > plot.Bottom + 1f)
                continue;
            f.Dl.AddLine(new float2(plot.X, y), new float2(plot.Right, y), Palette.Grid.A(0.3f), 1f * s);
            if (lane.W <= 0f)
                continue;
            string t = text(v);
            if (t.Length == 0)
                continue;
            float w = ImGui.CalcTextSize(t).X;
            float top = Math.Clamp(y - font * 0.5f, lane.Y, Math.Max(lane.Y, lane.Bottom - font));
            if (Math.Abs(top - lastY) < font)
                continue;
            f.Dl.AddText(new float2(lane.Right - w - 4f * s, top), Palette.Dim.Solid, t);
            lastY = top;
        }
        ConsoleStyle.PopFont();
    }

    #endregion

    /// <summary>A text on a dim plate, drawn unformatted, so free text cannot reach a format call.</summary>
    public static void Text(in PictureFrame f, float2 at, string text, ImColor8 color)
    {
        if (string.IsNullOrEmpty(text))
            return;
        float2 size = ImGui.CalcTextSize(text);
        float pad = 2f * f.Scale;
        float x = Math.Clamp(at.X, f.Min.X + pad, Math.Max(f.Min.X + pad, f.Max.X - size.X - pad));
        float y = Math.Clamp(at.Y, f.Min.Y + pad, Math.Max(f.Min.Y + pad, f.Max.Y - size.Y - pad));
        var corner = new float2(x, y);
        f.Dl.AddRectFilled(corner - new float2(pad, pad * 0.5f), corner + size + new float2(pad, pad * 0.5f), Palette.Plate, 2f);
        f.Dl.AddText(corner + new float2(pad * 0.5f, pad * 0.5f), Palette.Shadow, text);
        f.Dl.AddText(corner, color, text);
    }

    public static ScreenRect BigNumber(in PictureFrame f, string text, ImColor8 color)
    {
        ScreenRect box = BigNumberBox(in f, text);
        if (box.W <= 0f)
            return box;
        float pad = 3f * f.Scale;
        PushBigFont(in f, text);
        f.Dl.AddText(new float2(box.X + pad, box.Y + pad), color, text);
        ConsoleStyle.PopFont();
        return box;
    }

    public static ScreenRect BigNumberBox(in PictureFrame f, string text)
    {
        if (string.IsNullOrEmpty(text))
            return default;
        PushBigFont(in f, text);
        float2 size = ImGui.CalcTextSize(text);
        ConsoleStyle.PopFont();
        float pad = 6f * f.Scale;
        var at = new float2(f.Max.X - size.X - pad, f.Min.Y + pad * 0.5f);
        return new ScreenRect(at.X - pad * 0.5f, at.Y - pad * 0.5f, size.X + pad, size.Y + pad);
    }

    private static void PushBigFont(in PictureFrame f, string text)
    {
        ConsoleUi.PushAnswer();
        bool fits = ImGui.CalcTextSize(text).X <= f.Width * 0.55f;
        if (fits)
            return;
        ConsoleStyle.PopFont();
        ConsoleUi.PushTitle();
        fits = ImGui.CalcTextSize(text).X <= f.Width * 0.7f;
        if (fits)
            return;
        ConsoleStyle.PopFont();
        ConsoleUi.PushRow();
    }

    /// <summary>A long one wraps inside the frame.</summary>
    public static void Empty(in PictureFrame f, string text)
    {
        float wrap = Math.Max(1f, f.Width - 12f * f.Scale);
        float2 size = ImGui.CalcTextSize(text, false, wrap);
        ImGui.SetCursorScreenPos(f.Center - size * 0.5f);
        ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + wrap);
        ImGui.PushStyleColor(ImGuiCol.Text, Palette.Dim.F4);
        ImGui.Text(text);
        ImGui.PopStyleColor();
        ImGui.PopTextWrapPos();
    }

    #region Tabs

    private static float TabHeight => SmallFontSize + 7f * ImGuiHelper.InterfaceScale;

    /// <summary>The row on top of a picture: the views of the picture as folder tabs on the edge of its frame at the left, or its name in a plain dim caption where it has one view, because a tab with nothing to...</summary>
    public static int Header(string title, ReadOnlySpan<string> views, ref int view, string idPrefix,
        ReadOnlySpan<string> switches = default, ReadOnlySpan<bool> on = default, ReadOnlySpan<string> tips = default,
        ReadOnlySpan<string> disabledReasons = default, ReadOnlySpan<string> whenTags = default, float reserve = 0f, ReadOnlySpan<string> viewCards = default)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float avail = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float gap = ImGui.GetStyle().ItemSpacing.X;

        float right = RightWidth(switches, gap) + reserve;

        float bottom = origin.Y + h;
        if (views.Length > 1)
        {
            int picked = FolderTabs(views, view, idPrefix + "VIEW", origin.X, bottom, avail - right - gap, viewCards);
            if (picked >= 0)
                view = picked;
        }
        else if (title.Length > 0)
            Caption(title, origin.X, origin.Y, h);

        int clicked = -1;
        float x = origin.X + Math.Max(0f, avail - right);
        for (int i = 0; i < switches.Length; i++)
        {
            ImGui.SetCursorScreenPos(new float2(x, origin.Y));
            string tag = i < whenTags.Length ? whenTags[i] : "";
            string reason = i < disabledReasons.Length ? disabledReasons[i] : "";
            bool state = i < on.Length && on[i];
            if (ConsoleUi.Switch(switches[i], idPrefix + "Sw" + i, ref state, i < tips.Length ? tips[i] : "", tag, reason))
                clicked = i;
            x += ConsoleUi.SwitchWidth(switches[i]) + gap * 2f;
        }

        ImGui.SetCursorScreenPos(new float2(origin.X, bottom));
        return clicked;
    }

    private static void Caption(string title, float x, float y, float h)
    {
        ConsoleUi.PushSmall();
        ImGui.GetWindowDrawList().AddText(new float2(x + 6f * ImGuiHelper.InterfaceScale, y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), title);
        ConsoleStyle.PopFont();
    }

    /// <summary>The plain head of a picture that shows a stock thing: one word in the label ink and the STOCK chip after it, as on the row of TARGET, over the frame that draws next.</summary>
    public static void StockCaption(string word)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float h = TabHeight;
        float scale = ImGuiHelper.InterfaceScale;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        ConsoleUi.PushSmall();
        float x = origin.X + 6f * scale;
        dl.AddText(new float2(x, origin.Y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32), word);
        x += ImGui.CalcTextSize(word).X + 6f * scale;
        ConsoleStyle.PopFont();
        var chipAt = new float2(x, origin.Y);
        float chipW = ConsoleUi.OwnerChipAt(dl, chipAt, h, Owner.Stock, "STOCK");
        ConsoleUi.StockChipCard(chipAt, new float2(chipW, h));
        ImGui.SetCursorScreenPos(new float2(origin.X, origin.Y + h));
    }

    // The tabs from x = left with their bottom on the frame edge.
    private static int FolderTabs(ReadOnlySpan<string> tabs, int active, string id, float left, float bottom, float room, ReadOnlySpan<string> cards = default)
    {
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float scale = ImGuiHelper.InterfaceScale;
        float rounding = ConsoleStyle.FrameRoundingPx;
        float border = ConsoleStyle.WindowBorderThicknessPx;
        float h = TabHeight;
        bool windowHovered = ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows);
        int clicked = -1;
        ConsoleUi.PushSmall();
        try
        {
            float font = ImGui.GetFontSize();
            float pad = 7f * scale;
            float gap = 2f * scale;
            float start = 6f * scale;
            float textW = 0f;
            for (int i = 0; i < tabs.Length; i++)
                textW += ImGui.CalcTextSize(tabs[i]).X;
            float need = textW + (pad * 2f) * tabs.Length + gap * Math.Max(0, tabs.Length - 1) + start;
            if (need > room && tabs.Length > 0)
                pad = Math.Max(2f * scale, pad - (need - room) / (2f * tabs.Length));

            float x = left + start;
            for (int i = 0; i < tabs.Length; i++)
            {
                float w = ImGui.CalcTextSize(tabs[i]).X + pad * 2f;
                var pos = new float2(x, bottom - h);
                var size = new float2(w, h);
                bool isActive = i == active;
                ImGui.SetCursorScreenPos(pos);
                if (ImGui.InvisibleButton(id + "##Tab" + i, size))
                {
                    clicked = i;
                    ConsoleUi.ShapeChanged = true;
                }
                ConsoleUi.NoteRow(id, i, pos, size);
                bool hovered = windowHovered && ImGui.IsMouseHoveringRect(in pos, pos + size);
                var max = new float2(x + w, bottom + 1f);
                if (isActive)
                {
                    dl.AddRectFilled(pos, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.InsetU32), rounding, ImDrawFlags.RoundCornersTop);
                    dl.AddRect(pos, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.FrameU32), rounding, ImDrawFlags.RoundCornersTop, border);
                    _gapX0 = x;
                    _gapX1 = x + w;
                    _gapY = bottom;
                    _gapSet = true;
                }
                else
                {
                    if (hovered)
                        dl.AddRectFilled(pos, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.RowHoverWashU32), rounding, ImDrawFlags.RoundCornersTop);
                    dl.AddRect(pos, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32), rounding, ImDrawFlags.RoundCornersTop, border);
                }
                uint ink = ConsoleStyle.ApplyAlpha(isActive ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextMutedU32);
                dl.AddText(pos + new float2(pad, (h - font) * 0.5f), ink, tabs[i]);
                if (hovered && i < cards.Length && cards[i].Length > 0)
                {
                    ConsoleStyle.PopFont();
                    ConsoleUi.Card(tabs[i], cards[i]);
                    ConsoleUi.PushSmall();
                }
                x += w + gap;
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
        return clicked;
    }

    private static float RightWidth(ReadOnlySpan<string> switches, float gap)
    {
        float w = 0f;
        for (int i = 0; i < switches.Length; i++)
            w += ConsoleUi.SwitchWidth(switches[i]) + (i > 0 ? gap * 2f : 0f);
        return w;
    }

    #endregion

    #region Legend

    public readonly record struct Swatch(string Name, Rgb Colour, SwatchKind Kind = SwatchKind.Line, float Alpha = 1f);

    public static float LegendWidth(ReadOnlySpan<string> names)
    {
        float w = 0f;
        foreach (string name in names)
            w = Math.Max(w, ConsoleUi.SmallWidth(name));
        return w + 22f * ImGuiHelper.InterfaceScale;
    }

    public static void Legend(in PictureFrame f, in PicBox lane, ReadOnlySpan<Swatch> swatches) => DrawLegend(in f, in lane, swatches, inRow: false);

    public static void LegendRow(in PictureFrame f, in PicBox lane, ReadOnlySpan<Swatch> swatches) => DrawLegend(in f, in lane, swatches, inRow: true);

    // A column of swatches stops at the bottom of the lane, a row of them at its right edge, and a row skips a swatch with no name.
    private static void DrawLegend(in PictureFrame f, in PicBox lane, ReadOnlySpan<Swatch> swatches, bool inRow)
    {
        if (lane.W <= 0f || lane.H <= 0f)
            return;
        float s = f.Scale;
        ConsoleUi.PushSmall();
        try
        {
            float font = ImGui.GetFontSize();
            float rowH = font + 3f * s;
            float mark = 12f * s;
            float x = lane.X + 4f * s;
            float y = lane.Y + 2f * s;
            foreach (Swatch swatch in swatches)
            {
                float w = 0f, mid, textY;
                if (inRow)
                {
                    if (string.IsNullOrEmpty(swatch.Name))
                        continue;
                    w = ImGui.CalcTextSize(swatch.Name).X;
                    if (x + mark + 4f * s + w > lane.Right)
                        break;
                    mid = lane.Y + lane.H * 0.5f;
                    textY = mid - font * 0.5f;
                }
                else
                {
                    if (y + rowH > lane.Bottom)
                        break;
                    mid = y + rowH * 0.5f;
                    textY = y + (rowH - font) * 0.5f;
                }
                DrawSample(in f, swatch, x, mid, mark);
                f.Dl.AddText(new float2(x + mark + 4f * s, textY), Palette.Dim.Solid, swatch.Name);
                if (inRow)
                    x += mark + 4f * s + w + 14f * s;
                else
                    y += rowH;
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    private static void DrawSample(in PictureFrame f, in Swatch swatch, float x, float midY, float mark)
    {
        float s = f.Scale;
        ImDrawListPtr dl = f.Dl;
        ImColor8 c = swatch.Colour.A(swatch.Alpha);
        switch (swatch.Kind)
        {
            case SwatchKind.Dashed:
                Glyphs.DashedLine(dl, new float2(x, midY), new float2(x + mark, midY), c, 1.5f * s, 3f * s, 2f * s);
                break;
            case SwatchKind.LongDash:
                Glyphs.DashedLine(dl, new float2(x, midY), new float2(x + mark, midY), c, 1.5f * s, 5f * s, 2f * s);
                break;
            case SwatchKind.Dotted:
                Glyphs.DashedLine(dl, new float2(x, midY), new float2(x + mark, midY), c, 1f * s, 1f * s, 2f * s);
                break;
            case SwatchKind.Tick:
                dl.AddLine(new float2(x + mark * 0.5f, midY - 5f * s), new float2(x + mark * 0.5f, midY + 5f * s), c, 1.5f * s);
                break;
            case SwatchKind.Beam:
                Glyphs.Beam(dl, x + mark * 0.5f, midY - 6f * s, midY + 6f * s, 2.5f * s, 1.5f * s, c);
                break;
            case SwatchKind.Thick:
                dl.AddLine(new float2(x, midY), new float2(x + mark, midY), c, 3f * s);
                break;
            case SwatchKind.Cross:
                Glyphs.Cross(dl, new float2(x + mark * 0.5f, midY), 3.5f * s, c, 1.5f * s);
                break;
            case SwatchKind.Filled:
                dl.AddRectFilled(new float2(x + 3f * s, midY - 4f * s), new float2(x + mark - 3f * s, midY + 4f * s), c, 1f);
                break;
            case SwatchKind.Frame:
                dl.AddRect(new float2(x + 3f * s, midY - 4f * s), new float2(x + mark - 3f * s, midY + 4f * s), c, 1f, ImDrawFlags.None, 1.5f * s);
                break;
            case SwatchKind.Diamond:
                Glyphs.Diamond(dl, new float2(x + mark * 0.5f, midY), 4f * s, c);
                break;
            case SwatchKind.Ring:
                Glyphs.Ring(dl, new float2(x + mark * 0.5f, midY), 4f * s, c, 1.5f * s);
                break;
            default:
                dl.AddLine(new float2(x, midY), new float2(x + mark, midY), c, 1.5f * s);
                break;
        }
    }

    #endregion

    /// <summary>The line under a picture frame: a text at the left, such as the stage line, and the small verbs of the picture at the right (CALCULATE, CANCEL, CLEAR), with at most one link after them.</summary>
    public static int LegendLine(string legend, ReadOnlySpan<string> verbs, ReadOnlySpan<string> tips, string idPrefix,
        ReadOnlySpan<bool> disabled = default, string link = "", string linkTip = "", bool linkStock = false, string key = "", Rgb keyHue = default)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float avail = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float right = 0f;
        for (int i = 0; i < verbs.Length; i++)
            right += ConsoleWidgets.ButtonWidth(verbs[i].AsSpan()) + (i > 0 ? gap : 0f);
        float linkW = link.Length > 0 ? ConsoleUi.LinkWidth(link) : 0f;
        float chipW = linkW > 0f && linkStock ? ConsoleUi.OwnerChipWidth(Owner.Stock, "STOCK") + gap : 0f;
        if (linkW > 0f)
            right += (right > 0f ? gap * 3f : 0f) + chipW + linkW;
        float roomEnd = origin.X + avail - right - gap;

        float x = origin.X, y = origin.Y;
        ConsoleStyle.PushLabelFont();
        try
        {
            uint muted = ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32);
            if (legend.Length > 0)
            {
                float scale = ImGuiHelper.InterfaceScale;
                float legendW = ImGui.CalcTextSize(legend).X;
                float keyW = key.Length > 0 ? 24f * scale + ImGui.CalcTextSize(key).X : 0f;
                if (x + legendW + keyW > roomEnd && right > 0f)
                {
                    y += h;
                    roomEnd = origin.X + avail;
                }
                ConsoleUi.FitText(new float2(x, y), roomEnd - x, h, legend, muted);
                if (keyW > 0f && x + legendW + keyW <= roomEnd)
                {
                    ImDrawListPtr dl = ImGui.GetWindowDrawList();
                    float keyX = x + legendW + 12f * scale;
                    float mid = y + h * 0.5f;
                    dl.AddRectFilled(new float2(keyX, mid - 4f * scale), new float2(keyX + 8f * scale, mid + 4f * scale), ConsoleStyle.ApplyAlpha(keyHue.U32));
                    dl.AddText(new float2(keyX + 12f * scale, mid - ImGui.GetFontSize() * 0.5f), muted, key);
                }
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }

        int clicked = -1;
        float vx = origin.X + Math.Max(0f, avail - right);
        for (int i = 0; i < verbs.Length; i++)
        {
            ImGui.SetCursorScreenPos(new float2(vx, origin.Y));
            bool off = disabled.Length > i && disabled[i];
            bool hit;
            using (new ImGuiDisabledScope(off))
                hit = ConsoleWidgets.Button(verbs[i].AsSpan(), (idPrefix + "Verb" + i).AsSpan(), default);
            if (tips.Length > i && tips[i].Length > 0 && ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
                ConsoleUi.Card(verbs[i], tips[i]);
            if (hit && !off)
                clicked = i;
            vx += ConsoleWidgets.ButtonWidth(verbs[i].AsSpan()) + gap;
        }
        if (linkW > 0f)
        {
            if (chipW > 0f)
            {
                var chipAt = new float2(origin.X + avail - chipW + gap, origin.Y);
                ConsoleUi.OwnerChipAt(ImGui.GetWindowDrawList(), chipAt, h, Owner.Stock, "STOCK");
                ConsoleUi.StockChipCard(chipAt, new float2(chipW - gap, h));
            }
            ImGui.SetCursorScreenPos(new float2(origin.X + avail - chipW - linkW, origin.Y));
            if (ConsoleUi.Link(link, idPrefix + "Link", linkTip))
                clicked = verbs.Length;
        }

        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(avail, y - origin.Y + h));
        return clicked;
    }
}
