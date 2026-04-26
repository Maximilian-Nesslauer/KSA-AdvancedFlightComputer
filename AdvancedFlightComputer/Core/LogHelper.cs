using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Log deduplication for warnings that would otherwise spam the console.
///
/// Keys are kept for the lifetime of the mod, so a warning fires at most
/// once per session even if the underlying state recovers and degrades
/// again. Reset() clears the set on mod unload so reloading sees fresh
/// warnings.
/// </summary>
internal static class LogHelper
{
    private static readonly HashSet<string> _loggedWarnings = new();

    /// <summary>
    /// Logs a warning only on its first occurrence for a given key.
    /// Subsequent calls with the same key are silently dropped.
    /// </summary>
    public static void WarnOnce(string key, string message)
    {
        if (_loggedWarnings.Add(key))
            DefaultCategory.Log.Warning(message);
    }

    public static void Reset() => _loggedWarnings.Clear();
}
