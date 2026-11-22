#if DEBUG
using System.Diagnostics;
using System.Globalization;
using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Per-method Stopwatch accumulator for debug builds. It keeps the call count and the total, min
/// and max elapsed ticks per name and reports them every <see cref="ReportIntervalSeconds"/> and
/// on <see cref="Reset"/>, with the garbage collector's work over the same window. A single call
/// over <see cref="SpikeMs"/> is logged as it happens, with the share of it the collector paused
/// for; nested scopes end first, so a spike's parts are logged ahead of the whole. A call site is
/// <c>using var _ = new PerfTracker.Scope("MethodName");</c>; release builds get a no-op Scope.
/// </summary>
internal static class PerfTracker
{
    // Long enough that the summary does not dominate the session log while the measured code runs every frame.
    private const double ReportIntervalSeconds = 30.0;

    /// <summary>A call this long is logged on its own, ms: three frames at 60 fps.</summary>
    private const double SpikeMs = 50.0;

    /// <summary>At most this many spike lines per report window, so a run of hitches cannot flood the log.</summary>
    private const int SpikesPerWindow = 40;

    private static readonly Dictionary<string, PerfData> _entries = new();
    private static readonly List<string> _orderedKeys = new();
    private static readonly Dictionary<string, long> _gapLast = new();
    private static long _windowStart = Stopwatch.GetTimestamp();
    private static int _spikes;
    private static int _gc0 = GC.CollectionCount(0), _gc1 = GC.CollectionCount(1), _gc2 = GC.CollectionCount(2);
    private static TimeSpan _gcPause = GC.GetTotalPauseDuration();
    private static long _allocated = GC.GetTotalAllocatedBytes(false);

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
        private readonly long _pauseStart;

        public Scope(string name)
        {
            _name = name;
            _start = DebugConfig.Performance ? Stopwatch.GetTimestamp() : -1;
            _pauseStart = _start < 0 ? 0 : GC.GetTotalPauseDuration().Ticks;
        }

        public void Dispose()
        {
            if (_start < 0) return;
            Record(_name, Stopwatch.GetTimestamp() - _start, _pauseStart);
        }
    }

    /// <summary>
    /// The wall-clock time since the last call of the same name, recorded under that name: called once per step of the focused craft, its max is the longest the game went between our steps, whoever held it up.
    /// </summary>
    public static void Gap(string name)
    {
        if (!DebugConfig.Performance) return;
        long now = Stopwatch.GetTimestamp();
        if (_gapLast.TryGetValue(name, out long last))
            Record(name, now - last, -1);
        _gapLast[name] = now;
    }

    public static void Record(string name, long elapsedTicks) => Record(name, elapsedTicks, -1);

    private static void Record(string name, long elapsedTicks, long pauseStartTicks)
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

        double ms = TicksToMs(elapsedTicks);
        if (ms >= SpikeMs && _spikes < SpikesPerWindow)
        {
            _spikes++;
            string gc = pauseStartTicks >= 0
                ? string.Format(CultureInfo.InvariantCulture, ", GC paused {0:F1}ms of it",
                    (GC.GetTotalPauseDuration().Ticks - pauseStartTicks) / (double)TimeSpan.TicksPerMillisecond)
                : " (gap between steps)";
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] Perf spike: {0} {1:F1}ms{2}; GC gen0/1/2 so far {3}/{4}/{5}",
                name, ms, gc, GC.CollectionCount(0), GC.CollectionCount(1), GC.CollectionCount(2)));
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
                "[AFC] Perf ({0:F1}s): {1} avg={2:F3}ms min={3:F3}ms max={4:F3}ms total={5:F0}ms ({6} calls)",
                elapsed, key, TicksToMs(data.TotalTicks / data.Count), TicksToMs(data.MinTicks),
                TicksToMs(data.MaxTicks), TicksToMs(data.TotalTicks), data.Count));
        }

        // The whole process's collector, ours and the game's alike: a gen2 count that climbs with our scopes' totals points at our allocation.
        int gc0 = GC.CollectionCount(0), gc1 = GC.CollectionCount(1), gc2 = GC.CollectionCount(2);
        TimeSpan pause = GC.GetTotalPauseDuration();
        long allocated = GC.GetTotalAllocatedBytes(false);
        if (_orderedKeys.Count > 0)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] Perf ({0:F1}s): GC gen0/1/2 +{1}/+{2}/+{3}, paused {4:F0}ms, allocated {5:F0} MB, {6} spikes",
                elapsed, gc0 - _gc0, gc1 - _gc1, gc2 - _gc2, (pause - _gcPause).TotalMilliseconds,
                (allocated - _allocated) / 1048576.0, _spikes));
        _gc0 = gc0; _gc1 = gc1; _gc2 = gc2; _gcPause = pause; _allocated = allocated;

        _entries.Clear();
        _orderedKeys.Clear();
        _spikes = 0;
        _windowStart = Stopwatch.GetTimestamp();
    }

    private static double WindowSeconds()
        => (Stopwatch.GetTimestamp() - _windowStart) / (double)Stopwatch.Frequency;

    private static double TicksToMs(long ticks)
        => ticks * 1000.0 / Stopwatch.Frequency;
}
#else
namespace AdvancedFlightComputer.Core;

/// <summary>Release builds time nothing: the scope compiles to nothing, so call sites need no #if.</summary>
internal static class PerfTracker
{
    internal readonly ref struct Scope
    {
        public Scope(string name) { }
        public void Dispose() { }
    }

    public static void Gap(string name) { }
}
#endif
