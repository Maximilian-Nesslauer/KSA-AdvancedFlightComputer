using System.Diagnostics;
using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Log deduplication. A key fires once per mod load through <see cref="WarnOnce"/> or
/// <see cref="DebugOnce"/>, each level with its own key set, and <see cref="ThrottleAllows"/>
/// re-arms its key after the given spacing. Locked because the porkchop worker reaches WarnOnce
/// through <c>OrbitalTransfers.AlignmentTime</c> while the plan-window draw reaches it on the
/// main thread. The log call itself stays outside the lock.
/// </summary>
internal static class LogHelper
{
    private static readonly HashSet<string> _warned = new();
    private static readonly HashSet<string> _debugged = new();
    private static readonly Dictionary<string, long> _throttleLastTimestamp = new();
    private static readonly object _gate = new();

    public static void WarnOnce(string key, string message)
    {
        if (FirstTime(_warned, key))
            DefaultCategory.Log.Warning(message);
    }

    /// <summary>For patch-time diagnostics. Harmony re-runs every transpiler on a method whenever
    /// another patch is applied to or removed from it, so an ungated success line would repeat
    /// per re-run.</summary>
    public static void DebugOnce(string key, string message)
    {
        if (FirstTime(_debugged, key))
            DefaultCategory.Log.Debug(message);
    }

    /// <summary>True at most once per <paramref name="minIntervalSec"/> of real time for a key, so
    /// a diagnostic whose trigger re-fires per frame logs a sample of the stream instead of the
    /// flood.</summary>
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
            _warned.Clear();
            _debugged.Clear();
            _throttleLastTimestamp.Clear();
        }
    }

    private static bool FirstTime(HashSet<string> seen, string key)
    {
        lock (_gate)
            return seen.Add(key);
    }
}
