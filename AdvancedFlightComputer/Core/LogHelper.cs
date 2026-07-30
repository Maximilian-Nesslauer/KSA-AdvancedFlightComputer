using System.Diagnostics;
using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Log deduplication for messages that would otherwise spam the console.
/// WarnOnce and DebugOnce share ONE key set, so keys must be unique
/// across both levels (a collision silently drops whichever message
/// fires second); a given key fires once per mod load. ThrottleAllows is
/// per-interval instead: its key re-arms after the given spacing.
/// Reset() drops all of it on unload so a re-loaded mod starts fresh.
///
/// Both containers are locked because the callers are not all on one thread: the
/// porkchop worker reaches WarnOnce through <c>OrbitalTransfers.AlignmentTime</c>
/// (<c>TransferTask</c>'s constructor queues its Run on the ThreadPool), while
/// the plan-window draw reaches it from that same method and from
/// <c>TransferPlanner.PopulateWithPlanets</c>. HashSet is not safe for concurrent
/// Add, and an Add that grows the buckets while another thread walks them can
/// read a half-updated chain. The logging call stays outside the lock; off-thread
/// logging itself is fine, since stock logs from that same work item.
/// </summary>
internal static class LogHelper
{
    private static readonly HashSet<string> _loggedOnce = new();
    private static readonly Dictionary<string, long> _throttleLastTimestamp = new();
    private static readonly object _gate = new();

    /// <summary>
    /// Logs a warning only on its first occurrence for a given key.
    /// Subsequent calls with the same key are silently dropped.
    /// </summary>
    public static void WarnOnce(string key, string message)
    {
        lock (_gate)
        {
            if (!_loggedOnce.Add(key)) return;
        }
        DefaultCategory.Log.Warning(message);
    }

    /// <summary>
    /// Debug-level counterpart of <see cref="WarnOnce"/>. Used by patch-time
    /// diagnostics: Harmony re-runs every transpiler on a method whenever
    /// another patch is applied to or removed from it, so an ungated success
    /// line repeats per re-run (and again during unpatch at unload).
    /// </summary>
    public static void DebugOnce(string key, string message)
    {
        lock (_gate)
        {
            if (!_loggedOnce.Add(key)) return;
        }
        DefaultCategory.Log.Debug(message);
    }

    /// <summary>
    /// True at most once per <paramref name="minIntervalSec"/> of real time
    /// for a given key. Gates diagnostics whose trigger re-fires per frame
    /// while an input drifts continuously (e.g. a preview replanned during a
    /// burn), so the log keeps a sample of the stream instead of the flood.
    /// </summary>
    public static bool ThrottleAllows(string key, double minIntervalSec)
    {
        long now = Stopwatch.GetTimestamp();
        lock (_gate)
        {
            if (_throttleLastTimestamp.TryGetValue(key, out long last)
                && now - last < minIntervalSec * Stopwatch.Frequency)
                return false;
            _throttleLastTimestamp[key] = now;
            return true;
        }
    }

    public static void Reset()
    {
        lock (_gate)
        {
            _loggedOnce.Clear();
            _throttleLastTimestamp.Clear();
        }
    }
}
