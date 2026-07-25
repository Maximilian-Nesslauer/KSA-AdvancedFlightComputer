using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Log deduplication for warnings that would otherwise spam the console.
/// A given key fires once per mod load; Reset() drops the set on unload
/// so a re-loaded mod sees fresh warnings.
///
/// The set is locked because the callers are not all on one thread: the porkchop
/// worker reaches WarnOnce through <c>OrbitalTransfers.AlignmentTime</c>
/// (<c>TransferTask</c>'s constructor queues its Run on the ThreadPool), while
/// the plan-window draw reaches it from that same method and from
/// <c>TransferPlanner.PopulateWithPlanets</c>. HashSet is not safe for concurrent
/// Add, and an Add that grows the buckets while another thread walks them can
/// read a half-updated chain. The logging call stays outside the lock; off-thread
/// logging itself is fine, since stock logs from that same work item.
/// </summary>
internal static class LogHelper
{
    private static readonly HashSet<string> _loggedWarnings = new();
    private static readonly object _gate = new();

    /// <summary>
    /// Logs a warning only on its first occurrence for a given key.
    /// Subsequent calls with the same key are silently dropped.
    /// </summary>
    public static void WarnOnce(string key, string message)
    {
        lock (_gate)
        {
            if (!_loggedWarnings.Add(key)) return;
        }
        DefaultCategory.Log.Warning(message);
    }

    public static void Reset()
    {
        lock (_gate)
        {
            _loggedWarnings.Clear();
        }
    }
}
