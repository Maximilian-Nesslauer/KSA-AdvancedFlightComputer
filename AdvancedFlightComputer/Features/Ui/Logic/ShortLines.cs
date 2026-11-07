using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// Short log lines. The stock console window shows every Debug and Info line on screen for a few
/// seconds with no wrap and no background, so a line longer than <see cref="Max"/> characters runs
/// across the HUD.
/// </summary>
internal static class ShortLines
{
    /// <summary>The longest message a short line carries after the log prefix that names the mod and the craft.</summary>
    public const int Max = 70;

    /// <summary><paramref name="text"/> cut to at most <paramref name="max"/> characters, ending in "..." when it was cut.</summary>
    public static string Cut(string? text, int max)
    {
        text ??= "";
        if (text.Length <= max)
            return text;
        return text.Substring(0, Math.Max(0, max - 3)).TrimEnd() + "...";
    }

    /// <summary>
    /// <paramref name="text"/> broken into parts of at most <paramref name="width"/> characters, at
    /// the last space that fits. A word longer than the width, such as a file path, is broken
    /// inside the word. No part is empty.
    /// </summary>
    public static List<string> Split(string? text, int width)
    {
        var parts = new List<string>();
        if (string.IsNullOrEmpty(text))
            return parts;
        width = Math.Max(1, width);
        int start = 0;
        while (start < text.Length && text[start] == ' ')
            start++;
        while (start < text.Length)
        {
            int length = Math.Min(width, text.Length - start);
            if (start + length < text.Length)
            {
                int space = text.LastIndexOf(' ', start + length, length);
                if (space > start)
                    length = space - start;
            }
            parts.Add(text.Substring(start, length).TrimEnd());
            start += length;
            while (start < text.Length && text[start] == ' ')
                start++;
        }
        return parts;
    }
}
