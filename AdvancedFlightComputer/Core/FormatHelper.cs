namespace AdvancedFlightComputer.Core;

internal static class FormatHelper
{
    /// <summary>
    /// Formats a duration in seconds as a human-readable string with the
    /// largest fitting unit (s, m, h, d). Negative durations are returned
    /// with a leading "-"; NaN/Infinity returns "N/A".
    /// </summary>
    public static string FormatDuration(double seconds)
    {
        if (double.IsNaN(seconds) || double.IsInfinity(seconds))
            return "N/A";

        string sign = seconds < 0 ? "-" : "";
        double abs = System.Math.Abs(seconds);

        if (abs < 60.0)
            return $"{sign}{abs:F0}s";
        if (abs < 3600.0)
        {
            int m = (int)(abs / 60.0);
            int s = (int)(abs % 60.0);
            return s > 0 ? $"{sign}{m}m {s}s" : $"{sign}{m}m";
        }
        if (abs < 86400.0)
        {
            int h = (int)(abs / 3600.0);
            int min = (int)((abs % 3600.0) / 60.0);
            return min > 0 ? $"{sign}{h}h {min}m" : $"{sign}{h}h";
        }
        int d = (int)(abs / 86400.0);
        int hr = (int)((abs % 86400.0) / 3600.0);
        return hr > 0 ? $"{sign}{d}d {hr}h" : $"{sign}{d}d";
    }
}
