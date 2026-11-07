using System;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Ui.Canvas;

internal readonly struct ScreenRect
{
    public readonly float X;
    public readonly float Y;
    public readonly float W;
    public readonly float H;

    public ScreenRect(float x, float y, float w, float h)
    {
        X = x;
        Y = y;
        W = w;
        H = h;
    }

    public float Right => X + W;
    public float Bottom => Y + H;

    public bool Intersects(in ScreenRect o)
    {
        return X < o.Right && Right > o.X && Y < o.Bottom && Bottom > o.Y;
    }

    public bool Contains(float2 p) => p.X >= X && p.X <= Right && p.Y >= Y && p.Y <= Bottom;
}

// A flat screen-space grid over the canvas, so the per-frame culling stays linear in the number of labels. Each cell heads a linked list of entries in shared arrays, which grow when needed and are otherwise reused, so a frame allocates nothing. A rect that reaches past the canvas lands in the border cells, and the overlap test is exact anyway.
internal sealed class ScreenGrid
{
    private readonly float _bucket;
    private float _originX;
    private float _originY;
    private int _cols;
    private int _rows;
    private int[] _head = Array.Empty<int>();
    private int[] _next = new int[256];
    private int[] _entryRect = new int[256];
    private ScreenRect[] _rects = new ScreenRect[128];
    private int _entryCount;
    private int _rectCount;

    public ScreenGrid(float bucket)
    {
        _bucket = bucket;
    }

    public void Reset(float2 min, float2 max)
    {
        _originX = min.X;
        _originY = min.Y;
        _cols = Math.Max(1, (int)MathF.Ceiling((max.X - min.X) / _bucket) + 1);
        _rows = Math.Max(1, (int)MathF.Ceiling((max.Y - min.Y) / _bucket) + 1);
        int cells = _cols * _rows;
        if (_head.Length < cells)
            _head = new int[cells];
        Array.Fill(_head, -1, 0, cells);
        _entryCount = 0;
        _rectCount = 0;
    }

    public void Insert(in ScreenRect rect)
    {
        if (_rectCount == _rects.Length)
            Array.Resize(ref _rects, _rects.Length * 2);
        int index = _rectCount++;
        _rects[index] = rect;
        CellRange(in rect, out int c0, out int c1, out int r0, out int r1);
        for (int r = r0; r <= r1; r++)
        {
            for (int c = c0; c <= c1; c++)
            {
                if (_entryCount == _next.Length)
                {
                    Array.Resize(ref _next, _next.Length * 2);
                    Array.Resize(ref _entryRect, _entryRect.Length * 2);
                }
                int cell = r * _cols + c;
                int entry = _entryCount++;
                _entryRect[entry] = index;
                _next[entry] = _head[cell];
                _head[cell] = entry;
            }
        }
    }

    public bool AnyOverlap(in ScreenRect rect)
    {
        CellRange(in rect, out int c0, out int c1, out int r0, out int r1);
        for (int r = r0; r <= r1; r++)
        {
            for (int c = c0; c <= c1; c++)
            {
                for (int entry = _head[r * _cols + c]; entry >= 0; entry = _next[entry])
                {
                    if (rect.Intersects(in _rects[_entryRect[entry]]))
                        return true;
                }
            }
        }
        return false;
    }

    private void CellRange(in ScreenRect rect, out int c0, out int c1, out int r0, out int r1)
    {
        c0 = Cell(rect.X - _originX, _cols);
        c1 = Cell(rect.Right - _originX, _cols);
        r0 = Cell(rect.Y - _originY, _rows);
        r1 = Cell(rect.Bottom - _originY, _rows);
    }

    private int Cell(float v, int count)
    {
        return Math.Clamp((int)MathF.Floor(v / _bucket), 0, count - 1);
    }
}
