using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Canvas;

internal sealed class Labels
{
    private const int MostSegments = 240;

    private readonly LabelPlacer _placer = new();
    private ImDrawListPtr _dl;
    private float _pad;
    private float _inset;

    public void Begin(ImDrawListPtr dl, in PicBox plot, float scale)
    {
        _dl = dl;
        _pad = 2f * scale;
        _inset = 3f * scale;
        _placer.Reset(in plot);
    }

    public void Curve(ReadOnlySpan<float2> points)
    {
        if (points.Length < 2)
            return;
        int stride = Math.Max(1, points.Length / MostSegments);
        int last = 0;
        for (int i = stride; i < points.Length; i += stride)
        {
            _placer.ReserveSegment(points[last].X, points[last].Y, points[i].X, points[i].Y);
            last = i;
        }
        if (last != points.Length - 1)
            _placer.ReserveSegment(points[last].X, points[last].Y, points[^1].X, points[^1].Y);
    }

    public void Mark(float2 c, float r) => _placer.Reserve(new PicBox(c.X - r, c.Y - r, r * 2f, r * 2f));

    public void Reserve(in PicBox box) => _placer.Reserve(in box);

    public float2 Size(string text)
    {
        ConsoleUi.PushSmall();
        float2 size = ImGui.CalcTextSize(text);
        ConsoleStyle.PopFont();
        return size + new float2(_pad * 2f, _pad);
    }

    public bool Point(string text, float2 point, float offset, ImColor8 color)
    {
        ConsoleUi.PushSmall();
        try
        {
            float2 size = ImGui.CalcTextSize(text);
            bool placed = _placer.TakePoint(point.X, point.Y, offset + _pad, size.X + _pad * 2f, size.Y + _pad, out PicBox box);
            return Finish(text, placed, in box, color);
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    public bool Guide(string text, float y, ImColor8 color, bool under = false)
    {
        ConsoleUi.PushSmall();
        try
        {
            float2 size = ImGui.CalcTextSize(text);
            bool placed = _placer.TakeOnGuide(y, size.X + _pad * 2f, size.Y + _pad, _inset, out PicBox box, under);
            return Finish(text, placed, in box, color);
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    public bool Spots(string text, ReadOnlySpan<float2> corners, ImColor8 color)
    {
        ConsoleUi.PushSmall();
        try
        {
            float2 size = ImGui.CalcTextSize(text);
            foreach (float2 corner in corners)
            {
                var box = new PicBox(corner.X, corner.Y, size.X + _pad * 2f, size.Y + _pad);
                if (_placer.Take(in box))
                    return Finish(text, true, in box, color);
            }
            return Finish(text, false, default, color);
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    private bool Finish(string text, bool placed, in PicBox box, ImColor8 color)
    {
        if (placed)
            Draw(text, in box, color);
        return placed;
    }

    private void Draw(string text, in PicBox box, ImColor8 color)
    {
        _dl.AddRectFilled(new float2(box.X, box.Y), new float2(box.Right, box.Bottom), Palette.Plate, _pad);
        var at = new float2(box.X + _pad, box.Y + _pad * 0.5f);
        _dl.AddText(at + new float2(_pad * 0.5f, _pad * 0.5f), Palette.Shadow, text);
        _dl.AddText(at, color, text);
    }
}
