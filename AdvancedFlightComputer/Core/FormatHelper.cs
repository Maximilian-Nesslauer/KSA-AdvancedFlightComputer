namespace AdvancedFlightComputer.Core;

static class FormatHelper
{
    /// <summary>
    /// Formats a duration in seconds as a human-readable string.
    /// Returns "past" for negative values, "N/A" for NaN/Infinity.
    /// Uses the largest appropriate unit: seconds, minutes, hours, days.
    /// </summary>
    public static string FormatDuration(double seconds)
    {
        if (double.IsNaN(seconds) || double.IsInfinity(seconds))
            return "N/A";
        if (seconds < 0)
            return "past";
        if (seconds < 60.0)
            return $"{seconds:F0}s";
        if (seconds < 3600.0)
        {
            int m = (int)(seconds / 60.0);
            int s = (int)(seconds % 60.0);
            return s > 0 ? $"{m}m {s}s" : $"{m}m";
        }
        if (seconds < 86400.0)
        {
            int h = (int)(seconds / 3600.0);
            int min = (int)((seconds % 3600.0) / 60.0);
            return min > 0 ? $"{h}h {min}m" : $"{h}h";
        }
        int d = (int)(seconds / 86400.0);
        int hr = (int)((seconds % 86400.0) / 3600.0);
        return hr > 0 ? $"{d}d {hr}h" : $"{d}d";
    }
}
