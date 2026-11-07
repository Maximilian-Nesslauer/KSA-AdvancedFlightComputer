using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// The porkchop plot as a binned grid instead of a texture. The 292 x 292 results of a transfer
/// calculation are binned into 73 x 73 cells of 4 x 4 entries, each cell keeps the lowest usable value
/// of its block and the entry that has it, the value is mapped to one of 32 colour levels on a log
/// scale between the smallest and the largest value, and equal neighbours in a row merge into one run.
/// The result is built once per calculation and drawn as at most 5329 rectangles, so no GPU texture
/// is needed. Rows are the departure time and columns the time of flight, as in the stock plot.
/// </summary>
internal sealed class PorkchopRaster
{
    public const int Size = 292;
    public const int Block = 4;
    public const int Cells = Size / Block;
    public const int Levels = 32;

    /// <summary>It draws in the neutral colour of a frame, so it reads as a hole and not as a bad value.</summary>
    public const byte NoLevel = byte.MaxValue;

    public const int NothingDims = Levels;

    /// <summary>A run of equal cells in one row, from <see cref="Col0"/> to <see cref="Col1"/> inclusive.</summary>
    public readonly record struct Run(int Row, int Col0, int Col1, byte Level);

    private readonly List<Run> _runs = new();
    private readonly int[] _bestRow = new int[Cells * Cells];
    private readonly int[] _bestCol = new int[Cells * Cells];

    public IReadOnlyList<Run> Runs => _runs;
    public double MinValue { get; private set; } = double.NaN;
    public double MaxValue { get; private set; } = double.NaN;

    public bool HasNoValue { get; private set; }

    private static readonly (float R, float G, float B)[] RampStops =
    {
        (253f, 231f, 37f),
        (94f, 201f, 98f),
        (33f, 145f, 140f),
        (59f, 82f, 139f),
        (68f, 1f, 84f),
    };

    private const float UnpaidShare = 0.7f;

    /// <summary>
    /// Builds the grid. <paramref name="value"/> reads the colour value of entry (row, col), and
    /// <paramref name="usable"/> says whether the entry is a transfer at all: not infeasible and not
    /// an impact. A value that is not finite and positive is not usable either, because the scale is
    /// logarithmic.
    /// </summary>
    public static PorkchopRaster Build(Func<int, int, double> value, Func<int, int, bool> usable)
    {
        var raster = new PorkchopRaster();
        var cellValue = new double[Cells * Cells];
        double min = double.MaxValue, max = double.MinValue;

        for (int cr = 0; cr < Cells; cr++)
        {
            for (int cc = 0; cc < Cells; cc++)
            {
                double best = double.NaN;
                int bestR = -1, bestC = -1;
                for (int r = cr * Block; r < cr * Block + Block; r++)
                {
                    for (int c = cc * Block; c < cc * Block + Block; c++)
                    {
                        if (!usable(r, c))
                            continue;
                        double v = value(r, c);
                        if (!double.IsFinite(v) || !(v > 0.0))
                            continue;
                        if (double.IsNaN(best) || v < best)
                        {
                            best = v;
                            bestR = r;
                            bestC = c;
                        }
                    }
                }
                int i = cr * Cells + cc;
                cellValue[i] = best;
                raster._bestRow[i] = bestR;
                raster._bestCol[i] = bestC;
                if (!double.IsNaN(best))
                {
                    min = Math.Min(min, best);
                    max = Math.Max(max, best);
                }
                else
                    raster.HasNoValue = true;
            }
        }

        bool any = min <= max;
        raster.MinValue = any ? min : double.NaN;
        raster.MaxValue = any ? max : double.NaN;

        for (int cr = 0; cr < Cells; cr++)
        {
            int start = 0;
            byte current = LevelOf(cellValue[cr * Cells], min, max);
            for (int cc = 1; cc <= Cells; cc++)
            {
                byte level = cc < Cells ? LevelOf(cellValue[cr * Cells + cc], min, max) : (byte)0;
                if (cc < Cells && level == current)
                    continue;
                raster._runs.Add(new Run(cr, start, cc - 1, current));
                if (cc < Cells)
                {
                    start = cc;
                    current = level;
                }
            }
        }
        return raster;
    }

    /// <summary>The colour level of a value on the log scale from <paramref name="min"/> to <paramref name="max"/>.</summary>
    public static byte LevelOf(double v, double min, double max)
    {
        if (double.IsNaN(v) || !(min <= max))
            return NoLevel;
        double t = max > min ? (Math.Log(v) - Math.Log(min)) / (Math.Log(max) - Math.Log(min)) : 0.0;
        int level = (int)(Math.Clamp(t, 0.0, 1.0) * Levels);
        return (byte)Math.Min(Levels - 1, level);
    }

    public static (byte R, byte G, byte B) LevelColor(byte level)
        => level == NoLevel ? ((byte)71, (byte)75, (byte)82) : RampColor((level + 0.5) / Levels);

    public static (byte R, byte G, byte B) RampColor(double t)
    {
        double at = Math.Clamp(t, 0.0, 1.0) * (RampStops.Length - 1);
        int stop = Math.Min((int)at, RampStops.Length - 2);
        float f = (float)(at - stop);
        (float R, float G, float B) a = RampStops[stop], b = RampStops[stop + 1];
        return ((byte)(a.R + (b.R - a.R) * f + 0.5f), (byte)(a.G + (b.G - a.G) * f + 0.5f), (byte)(a.B + (b.B - a.B) * f + 0.5f));
    }

    public static (byte R, byte G, byte B) LegendColor(int index, int blocks, double paidShare = 1.0)
    {
        double t = (index + 0.5) / Math.Max(1, blocks);
        (byte r, byte g, byte b) = RampColor(t);
        return t < paidShare ? (r, g, b) : ((byte)(r * UnpaidShare), (byte)(g * UnpaidShare), (byte)(b * UnpaidShare));
    }

    public double PaidShare(double onBoardMs)
    {
        if (BudgetBelow(onBoardMs))
            return 0.0;
        double share = LegendShare(onBoardMs);
        return double.IsFinite(share) ? share : 1.0;
    }

    public int PaidLevel(double onBoardMs)
    {
        if (!double.IsFinite(onBoardMs) || !double.IsFinite(MinValue) || onBoardMs >= MaxValue)
            return NothingDims;
        return onBoardMs < MinValue ? 0 : LevelOf(onBoardMs, MinValue, MaxValue);
    }

    public bool BudgetBelow(double onBoardMs) => double.IsFinite(onBoardMs) && double.IsFinite(MinValue) && onBoardMs < MinValue;

    public bool BudgetAbove(double onBoardMs) => double.IsFinite(onBoardMs) && double.IsFinite(MaxValue) && onBoardMs > MaxValue;

    public static (byte R, byte G, byte B) CellColor(byte level, int paidLevel)
    {
        (byte r, byte g, byte b) = LevelColor(level);
        return level == NoLevel || level < paidLevel ? (r, g, b) : ((byte)(r * UnpaidShare), (byte)(g * UnpaidShare), (byte)(b * UnpaidShare));
    }

    /// <summary>Where <paramref name="value"/> stands along the legend, from 0 to 1, or NaN outside its range.</summary>
    public double LegendShare(double value)
    {
        if (!double.IsFinite(value) || !double.IsFinite(MinValue) || !(MaxValue > MinValue) || value < MinValue || value > MaxValue)
            return double.NaN;
        return (Math.Log(value) - Math.Log(MinValue)) / (Math.Log(MaxValue) - Math.Log(MinValue));
    }

    public static bool MarkShows(int markRow, int markCol, int selectedRow, int selectedCol, double ringEntries)
    {
        if (markRow < 0)
            return false;
        if (selectedRow < 0)
            return true;
        double dr = markRow - selectedRow, dc = markCol - selectedCol;
        return dr * dr + dc * dc > ringEntries * ringEntries;
    }

    public static (double Row, double Col)? MarkPlace(int markRow, int markCol, int selectedRow, int selectedCol, double ringEntries, int side)
    {
        if (markRow < 0)
            return null;
        return MarkShows(markRow, markCol, selectedRow, selectedCol, ringEntries)
            ? (markRow + 0.5, markCol + 0.5)
            : (selectedRow + 0.5, selectedCol + 0.5 + side * ringEntries);
    }

    /// <summary>The full resolution entry a cell stands for, or (-1, -1) for a cell with no usable entry.</summary>
    public (int Row, int Col) EntryOf(int cellRow, int cellCol)
    {
        if (cellRow < 0 || cellRow >= Cells || cellCol < 0 || cellCol >= Cells)
            return (-1, -1);
        int i = cellRow * Cells + cellCol;
        return (_bestRow[i], _bestCol[i]);
    }
}
