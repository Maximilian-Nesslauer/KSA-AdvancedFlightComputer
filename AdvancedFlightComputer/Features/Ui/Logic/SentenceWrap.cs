using System;
using System.Collections.Generic;
using System.Text;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>The text wrap of ImGui also breaks after a period or a comma, so a decimal can stand on two rows, and it leaves one word alone on the last row.</summary>
internal static class SentenceWrap
{
    public delegate float SpanMeasure(ReadOnlySpan<char> text);

    private readonly record struct Slice(int Start, int Length);

    [ThreadStatic]
    private static List<Slice>? _slices;

    [ThreadStatic]
    private static StringBuilder? _sb;

    public static string Break(string text, float width, Func<string, float> measure)
        => Break(text, width, (SpanMeasure)(span => measure(span.ToString())));

    public static string Break(string text, float width, SpanMeasure measure)
    {
        ReadOnlySpan<char> all = text.AsSpan();
        if (text.Length == 0 || (all.IndexOf('\n') < 0 && measure(all) <= width))
            return text;

        List<Slice> rows = _slices ??= new List<Slice>(16);
        rows.Clear();

        int pStart = 0;
        while (true)
        {
            int nl = text.IndexOf('\n', pStart);
            int pEnd = nl >= 0 ? nl : text.Length;
            WrapParagraph(text, pStart, pEnd, width, measure, rows);
            if (nl < 0)
                break;
            pStart = nl + 1;
        }

        if (rows.Count == 1 && rows[0].Start == 0 && rows[0].Length == text.Length)
            return text;

        StringBuilder sb = _sb ??= new StringBuilder(text.Length + rows.Count);
        sb.Clear();
        sb.EnsureCapacity(text.Length + rows.Count);
        for (int i = 0; i < rows.Count; i++)
        {
            if (i > 0)
                sb.Append('\n');
            Slice s = rows[i];
            if (s.Length > 0)
                sb.Append(text, s.Start, s.Length);
        }
        return sb.ToString();
    }

    private static void WrapParagraph(string text, int pStart, int pEnd, float width, SpanMeasure measure, List<Slice> rows)
    {
        int first = rows.Count;
        int rowStart = pStart;
        int rowLen = 0;
        int wStart = pStart;

        while (true)
        {
            int sp = text.IndexOf(' ', wStart, pEnd - wStart);
            int wEnd = sp >= 0 ? sp : pEnd;
            int wLen = wEnd - wStart;

            if (rowLen == 0)
            {
                rowStart = wStart;
                rowLen = wLen;
            }
            else
            {
                int triedLen = wEnd - rowStart;
                if (measure(text.AsSpan(rowStart, triedLen)) > width)
                {
                    rows.Add(new Slice(rowStart, rowLen));
                    rowStart = wStart;
                    rowLen = wLen;
                }
                else
                {
                    rowLen = triedLen;
                }
            }

            if (sp < 0)
                break;
            wStart = sp + 1;
        }
        rows.Add(new Slice(rowStart, rowLen));
        KeepCompany(text, rows, first, width, measure);
    }

    public static string LastRow(string broken)
    {
        int end = broken.LastIndexOf('\n');
        return end >= 0 ? broken.Substring(end + 1) : broken;
    }

    private static void KeepCompany(string text, List<Slice> rows, int first, float width, SpanMeasure measure)
    {
        int last = rows.Count - 1;
        if (last <= first)
            return;
        Slice lastRow = rows[last];
        if (lastRow.Length > 0 && text.IndexOf(' ', lastRow.Start, lastRow.Length) >= 0)
            return;
        Slice above = rows[last - 1];
        if (above.Length == 0)
            return;
        int space = text.LastIndexOf(' ', above.Start + above.Length - 1, above.Length);
        if (space <= above.Start || text.IndexOf(' ', above.Start, above.Length) == space)
            return;
        int movedStart = space + 1;
        int movedLen = (lastRow.Start + lastRow.Length) - movedStart;
        if (movedStart + movedLen <= text.Length && lastRow.Start == above.Start + above.Length + 1)
        {
            if (measure(text.AsSpan(movedStart, movedLen)) > width)
                return;
            rows[last - 1] = new Slice(above.Start, space - above.Start);
            rows[last] = new Slice(movedStart, movedLen);
        }
    }
}
