using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>The text wrap of ImGui also breaks after a period or a comma, so a decimal can stand on two rows, and it leaves one word alone on the last row.</summary>
internal static class SentenceWrap
{
    public static string Break(string text, float width, Func<string, float> measure)
    {
        if (text.Length == 0 || (text.IndexOf('\n') < 0 && measure(text) <= width))
            return text;
        var rows = new List<string>();
        foreach (string paragraph in text.Split('\n'))
        {
            int first = rows.Count;
            string row = "";
            foreach (string word in paragraph.Split(' '))
            {
                string tried = row.Length == 0 ? word : row + " " + word;
                if (row.Length > 0 && measure(tried) > width)
                {
                    rows.Add(row);
                    row = word;
                }
                else
                    row = tried;
            }
            rows.Add(row);
            KeepCompany(rows, first, width, measure);
        }
        return string.Join("\n", rows);
    }

    public static string LastRow(string broken)
    {
        int end = broken.LastIndexOf('\n');
        return end >= 0 ? broken.Substring(end + 1) : broken;
    }

    private static void KeepCompany(List<string> rows, int first, float width, Func<string, float> measure)
    {
        int last = rows.Count - 1;
        if (last <= first || rows[last].IndexOf(' ') >= 0)
            return;
        string above = rows[last - 1];
        int space = above.LastIndexOf(' ');
        if (space <= 0 || above.IndexOf(' ') == space)
            return;
        string moved = above.Substring(space + 1) + " " + rows[last];
        if (measure(moved) > width)
            return;
        rows[last - 1] = above.Substring(0, space);
        rows[last] = moved;
    }
}
