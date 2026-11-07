using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>Plain lines of one craft within a moment are one event, and the newest stands for it (StatusSelector.SameEvent).</summary>
internal static class ConsoleMessages
{
    private const int Capacity = 16;

    // A warning or a failure that keeps coming back, such as a refusal on every click, toasts once in this many seconds.
    private const double ToastIntervalS = 30.0;

    internal readonly record struct Entry(string Time, Severity Severity, string Text, string CraftId, string Shape, DateTime AtUtc, bool FromStatus);

    private static readonly List<Entry> _entries = new();
    private static readonly HashSet<string> _current = new(StringComparer.Ordinal);
    private static readonly HashSet<string> _previous = new(StringComparer.Ordinal);
    private static readonly StringBuilder _shape = new();
    private static readonly Dictionary<string, DateTime> _toasted = new(StringComparer.Ordinal);

    public static IReadOnlyList<Entry> Entries => _entries;

    /// <summary>Records each line of this frame that was not there the frame before.</summary>
    public static void Observe(IReadOnlyList<StatusLine> lines, string craftId, bool consoleShown)
    {
        _current.Clear();
        foreach (StatusLine line in lines)
        {
            if (string.IsNullOrEmpty(line.Text))
                continue;
            // The severity is part of the key, so a short step error that lasts and becomes an error is recorded and toasted again.
            string key = craftId + "\n" + (int)line.Severity + "\n" + line.Text;
            if (!_current.Add(key) || _previous.Contains(key))
                continue;
            Record(line, craftId, fromStatus: true);
            if (!consoleShown && line.Severity is Severity.Warning or Severity.Error)
                Toast(line, craftId);
        }
        _previous.Clear();
        _previous.UnionWith(_current);
    }

    /// <summary>Records a toast, so it stays readable in the drawer after it faded.</summary>
    public static void RecordAlert(string text, string craftId, bool warning)
        => Record(new StatusLine(warning ? Severity.Warning : Severity.Info, text, ""), craftId, fromStatus: false);

    private static void Record(StatusLine line, string craftId, bool fromStatus)
    {
        string shape = Shape(line.Text);
        for (int i = 0; i < _entries.Count; i++)
        {
            if (_entries[i].CraftId == craftId && _entries[i].Shape == shape)
            {
                if (line.Severity == Severity.Info && _entries[i].Text == line.Text)
                    return;
                _entries.RemoveAt(i);
                break;
            }
        }
        DateTime nowUtc = DateTime.UtcNow;
        if (fromStatus && _entries.Count > 0 && _entries[0].FromStatus
            && StatusSelector.SameEvent(_entries[0].Severity, line.Severity, _entries[0].CraftId == craftId, (nowUtc - _entries[0].AtUtc).TotalSeconds))
            _entries.RemoveAt(0);
        _entries.Insert(0, new Entry(DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture), line.Severity, line.Text,
            craftId, shape, nowUtc, fromStatus));
        if (_entries.Count > Capacity)
            _entries.RemoveAt(_entries.Count - 1);
    }

    private static void Toast(StatusLine line, string craftId)
    {
        string key = craftId + "\n" + Shape(line.Text);
        DateTime now = DateTime.UtcNow;
        if (_toasted.TryGetValue(key, out DateTime last) && (now - last).TotalSeconds < ToastIntervalS)
            return;
        _toasted[key] = now;
        string text = craftId.Length > 0 ? craftId + ": " + line.Text : line.Text;
        if (line.Severity == Severity.Error)
            TimedAlert.CreateWarning(text);
        else
            TimedAlert.Create(text, Color.Yellow, 5.0);
    }

    // The text with every run of digits made one '#', so two lines that differ only in their numbers compare equal.
    private static string Shape(string text)
    {
        _shape.Clear();
        bool inNumber = false;
        foreach (char c in text)
        {
            if (char.IsAsciiDigit(c))
            {
                if (!inNumber)
                    _shape.Append('#');
                inNumber = true;
                continue;
            }
            inNumber = false;
            _shape.Append(c);
        }
        return _shape.ToString();
    }

    public static void Reset()
    {
        _entries.Clear();
        _current.Clear();
        _previous.Clear();
        _toasted.Clear();
    }
}
