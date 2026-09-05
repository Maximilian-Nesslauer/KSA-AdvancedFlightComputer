#if DEBUG
using System.Diagnostics;
using System.Globalization;
using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Per-method Stopwatch accumulator for debug builds. It keeps the call count and the total, min
/// and max elapsed ticks per name and reports them every <see cref="ReportIntervalSeconds"/> and
/// on <see cref="Reset"/>. A call site is <c>using var _ = new PerfTracker.Scope("MethodName");</c>
/// inside <c>#if DEBUG</c>.
/// </summary>
internal static class PerfTracker
{
    // Long enough that the summary does not dominate the session log while the plan window is open.
    private const double ReportIntervalSeconds = 30.0;

    private static readonly Dictionary<string, PerfData> _entries = new();
    private static readonly List<string> _orderedKeys = new();
    private static long _windowStart = Stopwatch.GetTimestamp();

    private struct PerfData
    {
        public int Count;
        public long TotalTicks;
        public long MinTicks;
        public long MaxTicks;
    }

    /// <summary>Dispose is a no-op when <see cref="DebugConfig.Performance"/> is off.</summary>
    internal readonly ref struct Scope
    {
        private readonly string _name;
        private readonly long _start;

        public Scope(string name)
        {
            _name = name;
            _start = DebugConfig.Performance ? Stopwatch.GetTimestamp() : -1;
        }

        public void Dispose()
        {
            if (_start < 0) return;
            Record(_name, Stopwatch.GetTimestamp() - _start);
        }
    }

    public static void Record(string name, long elapsedTicks)
    {
        if (_entries.TryGetValue(name, out var data))
        {
            data.Count++;
            data.TotalTicks += elapsedTicks;
            if (elapsedTicks < data.MinTicks) data.MinTicks = elapsedTicks;
            if (elapsedTicks > data.MaxTicks) data.MaxTicks = elapsedTicks;
            _entries[name] = data;
        }
        else
        {
            _entries[name] = new PerfData
            {
                Count = 1,
                TotalTicks = elapsedTicks,
                MinTicks = elapsedTicks,
                MaxTicks = elapsedTicks
            };
            _orderedKeys.Add(name);
        }

        if (WindowSeconds() >= ReportIntervalSeconds)
            Report();
    }

    /// <summary>Flushes the current window so a short session or a teardown still reports.</summary>
    public static void Reset() => Report();

    private static void Report()
    {
        double elapsed = WindowSeconds();
        foreach (string key in _orderedKeys)
        {
            if (!_entries.TryGetValue(key, out var data) || data.Count == 0)
                continue;

            DefaultCategory.Log.Debug(string.Format(
                CultureInfo.InvariantCulture,
                "[AFC] Perf ({0:F1}s): {1} avg={2:F3}ms min={3:F3}ms max={4:F3}ms ({5} calls)",
                elapsed, key, TicksToMs(data.TotalTicks / data.Count), TicksToMs(data.MinTicks),
                TicksToMs(data.MaxTicks), data.Count));
        }

        _entries.Clear();
        _orderedKeys.Clear();
        _windowStart = Stopwatch.GetTimestamp();
    }

    private static double WindowSeconds()
        => (Stopwatch.GetTimestamp() - _windowStart) / (double)Stopwatch.Frequency;

    private static double TicksToMs(long ticks)
        => ticks * 1000.0 / Stopwatch.Frequency;
}
#endif
