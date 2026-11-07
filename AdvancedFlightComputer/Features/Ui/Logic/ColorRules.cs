using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct Rgb8(byte R, byte G, byte B);

internal enum RowTone { Normal, Dim, Bright }

internal enum PlanTone { Plain, Good, Warn }

/// <summary>The values of the named colours of the console, which Canvas.Palette hands out to the draw calls, and the rules about them.</summary>
internal static class ColorRules
{
    public static readonly Rgb8 Afc = new(80, 200, 255);

    public static readonly Rgb8 Flown = new(235, 235, 235);

    public static readonly Rgb8 Target = new(180, 140, 255);

    public static readonly Rgb8 Stock = new(206, 199, 178);

    public static readonly Rgb8 StockDim = new(131, 127, 115);

    /// <summary>The plate of a chosen row of a table: ConsoleStyle.PanelRaised with 13 % of the AFC colour over it.</summary>
    public static readonly Rgb8 ChosenBand = new(37, 54, 64);

    public static readonly Rgb8 WarnOnBand = new(214, 142, 70);

    public static readonly Rgb8 BadOnBand = new(255, 112, 96);

    public static readonly Rgb8 DimOnBand = new(156, 162, 170);

    /// <summary>The same as ConsoleStyle.Positive.</summary>
    public static readonly Rgb8 Good = new(92, 201, 128);

    public static readonly Rgb8 Warn = new(255, 160, 60);

    public static readonly Rgb8 WarnDim = new(186, 122, 56);

    /// <summary>The same as ConsoleStyle.Danger.</summary>
    public static readonly Rgb8 Bad = new(255, 99, 82);

    public static readonly Rgb8 BadDim = new(204, 96, 84);

    /// <summary>It belongs to the stock button and footer colour ConsoleStyle.Pending, so no picture uses it.</summary>
    public static readonly Rgb8 Gold = new(242, 191, 76);

    public static readonly Rgb8 Air = new(70, 130, 220);

    public static readonly Rgb8 QAlpha = new(190, 232, 255);

    public static readonly Rgb8 Changed = new(120, 225, 190);

    /// <summary>The same as ConsoleStyle.Panel.</summary>
    public static readonly Rgb8 Panel = new(18, 19, 21);

    /// <summary>The same as ConsoleStyle.TextPrimary.</summary>
    public static readonly Rgb8 Bright = new(209, 213, 217);

    public static readonly Rgb8[] Stages =
    {
        new(214, 120, 170),
        new(70, 175, 165),
        new(115, 130, 170),
        new(155, 170, 90),
        new(178, 124, 92),
        new(160, 118, 196),
    };

    public static Rgb8 Stage(int index) => Stages[((index % Stages.Length) + Stages.Length) % Stages.Length];

    /// <summary>The relative luminance of a colour, by the WCAG 2 formula.</summary>
    public static double Luminance(Rgb8 c)
    {
        static double Linear(byte v)
        {
            double s = v / 255.0;
            return s <= 0.03928 ? s / 12.92 : Math.Pow((s + 0.055) / 1.055, 2.4);
        }
        return 0.2126 * Linear(c.R) + 0.7152 * Linear(c.G) + 0.0722 * Linear(c.B);
    }

    /// <summary>The contrast of two colours, by the WCAG 2 formula, from 1 to 21.</summary>
    public static double Contrast(Rgb8 a, Rgb8 b)
    {
        double la = Luminance(a), lb = Luminance(b);
        return (Math.Max(la, lb) + 0.05) / (Math.Min(la, lb) + 0.05);
    }

    public static Rgb8 InkOn(Rgb8 fill) => Contrast(fill, Panel) >= Contrast(fill, Bright) ? Panel : Bright;

    /// <summary>The hue of a colour in degrees, from 0 to 360.</summary>
    public static double Hue(Rgb8 c)
    {
        double r = c.R / 255.0, g = c.G / 255.0, b = c.B / 255.0;
        double max = Math.Max(r, Math.Max(g, b)), min = Math.Min(r, Math.Min(g, b));
        double d = max - min;
        if (d <= 0.0)
            return 0.0;
        double h = max == r ? (g - b) / d : max == g ? (b - r) / d + 2.0 : (r - g) / d + 4.0;
        return (h * 60.0 + 360.0) % 360.0;
    }

    public static double Saturation(Rgb8 c)
    {
        double max = Math.Max(c.R, Math.Max(c.G, c.B)), min = Math.Min(c.R, Math.Min(c.G, c.B));
        return max > 0.0 ? (max - min) / max : 0.0;
    }

    public static double Distance(Rgb8 a, Rgb8 b)
    {
        double dr = a.R - b.R, dg = a.G - b.G, db = a.B - b.B;
        return Math.Sqrt(dr * dr + dg * dg + db * db);
    }

    public static RowTone RowToneOf(bool chosen, bool dimmed) => chosen ? RowTone.Bright : dimmed ? RowTone.Dim : RowTone.Normal;

    public static PlanTone MissTone(double km, bool flying, bool far, double insideKm)
    {
        if (!double.IsFinite(km))
            return PlanTone.Plain;
        if (far)
            return PlanTone.Warn;
        return flying && km <= insideKm ? PlanTone.Good : PlanTone.Plain;
    }

    public static int AfcWordAt(string mark) => mark.IndexOf("AFC", StringComparison.Ordinal);
}
