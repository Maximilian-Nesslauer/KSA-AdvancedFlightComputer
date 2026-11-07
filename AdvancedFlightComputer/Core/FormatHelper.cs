using AdvancedFlightComputer.Features.Ui.Logic;

namespace AdvancedFlightComputer.Core;

internal static class FormatHelper
{
    /// <summary>Seconds in the one time form of the console, with a leading "-" for negative values and "N/A" for NaN or infinity.</summary>
    public static string FormatDuration(double seconds) => Fmt.Time(seconds);
}
