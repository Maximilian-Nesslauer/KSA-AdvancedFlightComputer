using System;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Ui.Canvas;

/// <summary>An RGB colour that hands out ImGui colours at any alpha.</summary>
internal readonly struct Rgb
{
    public readonly byte R, G, B;

    public Rgb(byte r, byte g, byte b)
    {
        R = r;
        G = g;
        B = b;
    }

    public static implicit operator Rgb(Rgb8 c) => new(c.R, c.G, c.B);

    public ImColor8 Solid => new ImColor8(R, G, B, 255);

    public ImColor8 A(float alpha) => new ImColor8(R, G, B, (byte)Math.Clamp((int)(alpha * 255f + 0.5f), 0, 255));

    public uint U32 => (uint)(R | (G << 8) | (B << 16)) | 0xFF000000u;

    public uint U32At(float alpha) => (uint)(R | (G << 8) | (B << 16)) | ((uint)Math.Clamp((int)(alpha * 255f + 0.5f), 0, 255) << 24);

    public float4 F4 => new float4(R / 255f, G / 255f, B / 255f, 1f);
}

/// <summary>Gold is the lit verb and belongs to the stock button and footer colour ConsoleStyle.Pending, so no picture uses it.</summary>
internal static class Palette
{
    public static readonly Rgb Afc = ColorRules.Afc;
    public static Rgb Plan => Afc;
    public static readonly Rgb Flown = ColorRules.Flown;
    public static readonly Rgb Target = ColorRules.Target;
    public static readonly Rgb Stock = ColorRules.Stock;
    public static readonly Rgb StockDim = ColorRules.StockDim;
    public static readonly Rgb WarnOnBand = ColorRules.WarnOnBand;
    public static readonly Rgb BadOnBand = ColorRules.BadOnBand;
    public static readonly Rgb DimOnBand = ColorRules.DimOnBand;

    /// <summary>The same as ConsoleStyle.Positive, so a picture and a row say a good result in one green.</summary>
    public static readonly Rgb Good = ColorRules.Good;

    /// <summary>It is clearly away from the gold of the verb, which ConsoleStyle.Pending holds.</summary>
    public static readonly Rgb Warn = ColorRules.Warn;

    public static readonly Rgb WarnDim = ColorRules.WarnDim;

    /// <summary>The same as ConsoleStyle.Danger, so a picture and a row say an error in one red.</summary>
    public static readonly Rgb Bad = ColorRules.Bad;

    /// <summary>A crash on a row that is not chosen.</summary>
    public static readonly Rgb BadDim = ColorRules.BadDim;

    public static readonly Rgb Air = ColorRules.Air;
    public static readonly Rgb QAlpha = ColorRules.QAlpha;
    public static readonly Rgb Changed = ColorRules.Changed;
    public static readonly Rgb Grid = new(70, 80, 95);
    public static readonly Rgb Text = new(205, 214, 224);
    public static readonly Rgb Dim = new(125, 138, 153);

    // The plate and the shadow behind a picture text.
    public static readonly ImColor8 Plate = new ImColor8(8, 10, 14, 190);
    public static readonly ImColor8 Shadow = new ImColor8(0, 0, 0, 210);

    /// <summary>The colour of stage of the burn order, counted from 0, shared by the staging bar, the boxes of STAGING and every stage mark (ColorRules.Stages).</summary>
    public static ImColor8 Stage(int index) => ((Rgb)ColorRules.Stage(index)).Solid;

    public static Rgb Tone(PlanTone tone) => tone switch
    {
        PlanTone.Good => Good,
        PlanTone.Warn => Warn,
        _ => Text,
    };
}

/// <summary>
/// The glyphs, as pure functions of a draw list, a point and a size. Every stroke follows the size it is given, so a glyph scales with the interface.
/// </summary>
internal static class Glyphs
{
    // No clip at all, for the legend and other short strokes.
    private static readonly float2 NoClipMin = new float2(-1e30f, -1e30f), NoClipMax = new float2(1e30f, 1e30f);

    public static void Diamond(ImDrawListPtr dl, float2 c, float r, ImColor8 col)
    {
        float2 top = c + new float2(0f, -r), right = c + new float2(r, 0f);
        float2 bottom = c + new float2(0f, r), left = c + new float2(-r, 0f);
        dl.AddTriangleFilled(top, right, bottom, col);
        dl.AddTriangleFilled(top, bottom, left, col);
    }

    public static void Ring(ImDrawListPtr dl, float2 c, float r, ImColor8 col, float thick)
        => dl.AddCircle(c, r, col, 0, thick);

    public static void Dot(ImDrawListPtr dl, float2 c, float r, ImColor8 col)
        => dl.AddCircleFilled(c, r, col);

    public static void Cross(ImDrawListPtr dl, float2 c, float r, ImColor8 col, float thick)
    {
        dl.AddLine(c + new float2(-r, -r), c + new float2(r, r), col, thick);
        dl.AddLine(c + new float2(-r, r), c + new float2(r, -r), col, thick);
    }

    public static void Beam(ImDrawListPtr dl, float x, float top, float bottom, float mark, float thick, ImColor8 col)
    {
        dl.AddLine(new float2(x, top), new float2(x, bottom), col, thick);
        dl.AddTriangleFilled(new float2(x - mark, top), new float2(x + mark, top), new float2(x, top + mark), col);
        dl.AddTriangleFilled(new float2(x - mark, bottom), new float2(x, bottom - mark), new float2(x + mark, bottom), col);
    }

    /// <summary>The launch pad: a tower on a base.</summary>
    public static void Pad(ImDrawListPtr dl, float2 c, float r, ImColor8 col)
    {
        dl.AddTriangleFilled(c + new float2(-r, 0f), c + new float2(r, 0f), c + new float2(0f, -r * 0.7f), col);
        dl.AddLine(c + new float2(r * 0.35f, 0f), c + new float2(r * 0.35f, -r * 1.8f), col, MathF.Max(1f, r * 0.2f));
    }

    /// <summary>A landing site: a ring with a crosshair.</summary>
    public static void Site(ImDrawListPtr dl, float2 c, float r, ImColor8 col, float thick)
    {
        dl.AddCircle(c, r, col, 0, thick);
        dl.AddLine(c + new float2(-r * 1.5f, 0f), c + new float2(r * 1.5f, 0f), col, thick);
        dl.AddLine(c + new float2(0f, -r * 1.5f), c + new float2(0f, r * 1.5f), col, thick);
    }

    public static void DashedLine(ImDrawListPtr dl, float2 a, float2 b, ImColor8 col, float thick, float dash, float gap)
        => DashedLine(dl, a, b, col, thick, dash, gap, NoClipMin, NoClipMax);

    /// <summary>A dashed line that emits dashes only inside the clip rectangle, so a line that runs far off screen at a deep zoom costs only what shows.</summary>
    public static void DashedLine(ImDrawListPtr dl, float2 a, float2 b, ImColor8 col, float thick, float dash, float gap, float2 clipMin, float2 clipMax)
    {
        float2 d = b - a;
        double len = Math.Sqrt((double)d.X * d.X + (double)d.Y * d.Y);
        if (len < 0.5)
            return;
        DashSegment(dl, a, d, len, 0.0, col, thick, dash, dash + gap, clipMin, clipMax);
    }

    public static void DashedPolyline(ImDrawListPtr dl, ReadOnlySpan<float2> pts, ImColor8 col, float thick, float dash, float gap)
        => DashedPolyline(dl, pts, col, thick, dash, gap, NoClipMin, NoClipMax);

    /// <summary>A dashed polyline whose dash pattern runs on across the vertices, so short segments still read as dashed. Only the parts inside the clip rectangle emit dashes.</summary>
    public static void DashedPolyline(ImDrawListPtr dl, ReadOnlySpan<float2> pts, ImColor8 col, float thick, float dash, float gap, float2 clipMin, float2 clipMax)
    {
        double period = dash + gap;
        double phase = 0.0;
        for (int i = 1; i < pts.Length; i++)
        {
            float2 a = pts[i - 1];
            float2 d = pts[i] - a;
            double len = Math.Sqrt((double)d.X * d.X + (double)d.Y * d.Y);
            if (len < 1e-3)
                continue;
            DashSegment(dl, a, d, len, phase, col, thick, dash, period, clipMin, clipMax);
            phase = (phase + len) % period;
        }
    }

    /// <summary>
    /// The dashes of one segment from <paramref name="a"/> along <paramref name="d"/>, where the pattern is <paramref name="phase"/> pixels into its period at <paramref name="a"/>. The segment is first cut to the clip rectangle, and the loop starts at the first dash that reaches into it.
    /// </summary>
    private static void DashSegment(ImDrawListPtr dl, float2 a, float2 d, double len, double phase, ImColor8 col, float thick, double dash, double period,
        float2 clipMin, float2 clipMax)
    {
        if (!ClipSegment(a, d, clipMin, clipMax, out double t0, out double t1))
            return;
        double s0 = t0 * len, s1 = t1 * len;
        double ux = d.X / len, uy = d.Y / len;
        // Dash k covers [k period - phase, k period - phase + dash] along the segment.
        double k = Math.Floor((s0 + phase) / period);
        for (double start = k * period - phase; start < s1; start += period)
        {
            double from = Math.Max(start, s0), to = Math.Min(start + dash, s1);
            if (to > from)
                dl.AddLine(new float2((float)(a.X + ux * from), (float)(a.Y + uy * from)),
                    new float2((float)(a.X + ux * to), (float)(a.Y + uy * to)), col, thick);
        }
    }

    /// <summary>The part of the segment a + t d, t in [0, 1], that lies inside the rectangle, by the Liang-Barsky parametric clip.</summary>
    private static bool ClipSegment(float2 a, float2 d, float2 min, float2 max, out double t0, out double t1)
    {
        t0 = 0.0;
        t1 = 1.0;
        return ClipEdge(-d.X, a.X - min.X, ref t0, ref t1) && ClipEdge(d.X, max.X - a.X, ref t0, ref t1)
            && ClipEdge(-d.Y, a.Y - min.Y, ref t0, ref t1) && ClipEdge(d.Y, max.Y - a.Y, ref t0, ref t1);
    }

    private static bool ClipEdge(double p, double q, ref double t0, ref double t1)
    {
        if (p == 0.0)
            return q >= 0.0;
        double r = q / p;
        if (p < 0.0)
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
