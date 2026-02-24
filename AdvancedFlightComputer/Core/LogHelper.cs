using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Utility for log deduplication. Prevents repeated warnings from spamming
/// the console by tracking which messages have already been logged.
/// </summary>
static class LogHelper
{
    private static readonly HashSet<string> _loggedWarnings = new();

    /// <summary>
    /// Logs a warning only on its first occurrence. Subsequent calls with the
    /// same key are silently ignored until Reset() is called.
    /// </summary>
    public static void WarnOnce(string key, string message)
    {
        if (_loggedWarnings.Add(key))
            DefaultCategory.Log.Warning(message);
    }

    /// <summary>
    /// Clears all tracked warnings. Called on mod unload so warnings
    /// re-fire after a reload.
    /// </summary>
    public static void Reset() => _loggedWarnings.Clear();
}
