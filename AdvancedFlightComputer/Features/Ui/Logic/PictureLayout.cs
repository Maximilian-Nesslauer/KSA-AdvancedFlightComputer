using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct PicBox(float X, float Y, float W, float H)
{
    public float Right => X + W;
    public float Bottom => Y + H;

    public bool Overlaps(in PicBox o) => X < o.Right && Right > o.X && Y < o.Bottom && Bottom > o.Y;

    public bool Inside(in PicBox o) => X >= o.X && Y >= o.Y && Right <= o.Right && Bottom <= o.Bottom;

    public bool Crossed(float x0, float y0, float x1, float y1)
    {
        float dx = x1 - x0, dy = y1 - y0;
        float t0 = 0f, t1 = 1f;
        return Edge(-dx, x0 - X, ref t0, ref t1) && Edge(dx, Right - x0, ref t0, ref t1)
            && Edge(-dy, y0 - Y, ref t0, ref t1) && Edge(dy, Bottom - y0, ref t0, ref t1);
    }

    private static bool Edge(float p, float q, ref float t0, ref float t1)
    {
        if (p == 0f)
            return q >= 0f;
        float r = q / p;
        if (p < 0f)
        {
            if (r > t1)
                return false;
            if (r > t0)
                t0 = r;
        }
        else
        {
            if (r < t0)
                return false;
            if (r < t1)
                t1 = r;
        }
        return true;
    }
}

internal readonly record struct PictureLanes(PicBox Head, PicBox Left, PicBox Plot, PicBox Right, PicBox Bottom);

internal static class PictureLayout
{
    /// <summary>Cuts a frame into its lanes. A plot needs <paramref name="minPlot"/> pixels each way, so a small frame gives up a lane: the legend first, then the y ticks, and the x ticks before the head.</summary>
    public static PictureLanes Split(PicBox frame, float head, float left, float right, float bottom, float minPlot)
    {
        if (frame.W - left - right < minPlot)
            right = 0f;
        if (frame.W - left - right < minPlot)
            left = 0f;
        if (frame.H - head - bottom < minPlot)
            bottom = 0f;
        if (frame.H - head - bottom < minPlot)
            head = 0f;
        float midY = frame.Y + head;
        float midH = Math.Max(0f, frame.H - head - bottom);
        return new PictureLanes(
            new PicBox(frame.X, frame.Y, frame.W, head),
            new PicBox(frame.X, midY, left, midH),
            new PicBox(frame.X + left, midY, Math.Max(0f, frame.W - left - right), midH),
            new PicBox(frame.Right - right, midY, right, midH),
            new PicBox(frame.X, frame.Bottom - bottom, frame.W, bottom));
    }
}

internal sealed class LabelPlacer
{
    private readonly List<PicBox> _taken = new();
    private readonly List<(float X0, float Y0, float X1, float Y1)> _curves = new();
    private PicBox _area;

    public void Reset(in PicBox area)
    {
        _area = area;
        _taken.Clear();
        _curves.Clear();
    }

    public void Reserve(in PicBox box)
    {
        if (box.W > 0f && box.H > 0f)
            _taken.Add(box);
    }

    public void ReserveSegment(float x0, float y0, float x1, float y1) => _curves.Add((x0, y0, x1, y1));

    public bool IsFree(in PicBox box)
    {
        if (!box.Inside(in _area))
            return false;
        foreach (PicBox other in _taken)
        {
            if (box.Overlaps(in other))
                return false;
        }
        foreach ((float x0, float y0, float x1, float y1) in _curves)
        {
            if (box.Crossed(x0, y0, x1, y1))
                return false;
        }
        return true;
    }

    public bool Take(in PicBox box)
    {
        if (!IsFree(in box))
            return false;
        _taken.Add(box);
        return true;
    }

    public bool TakeFirst(ReadOnlySpan<PicBox> spots, out PicBox placed)
    {
        foreach (PicBox spot in spots)
        {
            if (Take(in spot))
            {
                placed = spot;
                return true;
            }
        }
        placed = default;
        return false;
    }

    /// <summary>A free place near the mark at (<paramref name="px"/>, <paramref name="py"/>): the four corners around it first, then above and below it, then further out on its right.</summary>
    public bool TakePoint(float px, float py, float offset, float w, float h, out PicBox placed)
    {
        float dy = offset * 0.4f;
        Span<PicBox> spots =
        [
            new PicBox(px + offset, py + dy, w, h),
            new PicBox(px + offset, py - dy - h, w, h),
            new PicBox(px - offset - w, py + dy, w, h),
            new PicBox(px - offset - w, py - dy - h, w, h),
            new PicBox(px - w * 0.5f, py - offset - h, w, h),
            new PicBox(px - w * 0.5f, py + offset, w, h),
            new PicBox(px + offset, py + offset + h, w, h),
            new PicBox(px + offset, py - offset - h * 2f, w, h),
        ];
        return TakeFirst(spots, out placed);
    }

    public bool TakeOnGuide(float y, float w, float h, float gap, out PicBox placed, bool underFirst = false)
    {
        float left = _area.X + gap, right = _area.Right - w - gap;
        var overLeft = new PicBox(left, y - h - 1f, w, h);
        var overRight = new PicBox(right, y - h - 1f, w, h);
        var underLeft = new PicBox(left, y + 1f, w, h);
        var underRight = new PicBox(right, y + 1f, w, h);
        Span<PicBox> spots = underFirst ? [underLeft, underRight, overLeft, overRight] : [overLeft, overRight, underLeft, underRight];
        return TakeFirst(spots, out placed);
    }

    public int Mark => _taken.Count;

    public void Rollback(int mark)
    {
        if (mark < _taken.Count)
            _taken.RemoveRange(mark, _taken.Count - mark);
    }
}
