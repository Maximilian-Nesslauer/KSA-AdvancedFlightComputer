#if DEBUG
using System.Diagnostics;
using System.Globalization;
using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Lightweight per-method Stopwatch accumulator for debug builds.
/// Tracks call count and total/min/max elapsed ticks; reports a per-
/// window summary every <see cref="ReportIntervalSeconds"/> seconds and
/// then resets the counters.
///
/// Call site:
/// <code>
/// #if DEBUG
/// using var _ = new PerfTracker.Scope("MethodName");
/// #endif
/// </code>
/// </summary>
internal static class PerfTracker
{
    private const double ReportIntervalSeconds = 5.0;

    private static readonly Dictionary<string, PerfData> _entries = new();
    private static readonly List<string> _orderedKeys = new();
    private static long _lastReportTimestamp = Stopwatch.GetTimestamp();

    private struct PerfData
    {
        public int Count;
        public long TotalTicks;
        public long MinTicks;
        public long MaxTicks;
    }

    /// <summary>Stack-allocated scope: ctor takes the start timestamp
    /// (or -1 if <see cref="DebugConfig.Performance"/> is off, making
    /// Dispose a no-op).</summary>
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

        MaybeReport();
    }

    private static void MaybeReport()
    {
        long now = Stopwatch.GetTimestamp();
        double elapsed = (now - _lastReportTimestamp) / (double)Stopwatch.Frequency;
        if (elapsed < ReportIntervalSeconds)
            return;

        _lastReportTimestamp = now;

        foreach (string key in _orderedKeys)
        {
            if (!_entries.TryGetValue(key, out var data) || data.Count == 0)
                continue;

            double avgMs = TicksToMs(data.TotalTicks / data.Count);
            double minMs = TicksToMs(data.MinTicks);
            double maxMs = TicksToMs(data.MaxTicks);

            DefaultCategory.Log.Debug(string.Format(
                CultureInfo.InvariantCulture,
                "[AFC] Perf ({0:F1}s): {1} avg={2:F3}ms min={3:F3}ms max={4:F3}ms ({5} calls)",
                elapsed, key, avgMs, minMs, maxMs, data.Count));
        }

        _entries.Clear();
        _orderedKeys.Clear();
    }

    private static double TicksToMs(long ticks)
    {
        return ticks * 1000.0 / Stopwatch.Frequency;
    }

    public static void Reset()
    {
        _entries.Clear();
        _orderedKeys.Clear();
        _lastReportTimestamp = Stopwatch.GetTimestamp();
    }
}
#endif
