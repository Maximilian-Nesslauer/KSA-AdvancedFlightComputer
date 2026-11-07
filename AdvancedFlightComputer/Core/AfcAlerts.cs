using System;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// One alert call for the whole mod: the stock toast (TimedAlert.Create or CreateWarning) and an entry
/// in the console message history, named by craft. The history keeps what a toast showed for a few
/// seconds, so a player who looked away can read it in the head drawer.
/// </summary>
internal static class AfcAlerts
{
    /// <summary>Receives every alert for the message history. The console sets it, so this file stays free of the UI.</summary>
    internal static Action<string, string, bool>? History;

    public static void Info(string text, byte4 color, double seconds, Vehicle? vehicle = null)
    {
        TimedAlert.Create(text, color, seconds);
        Record(text, vehicle, warning: false);
    }

    public static void Warning(string text, Vehicle? vehicle = null)
    {
        TimedAlert.CreateWarning(text);
        Record(text, vehicle, warning: true);
    }

    private static void Record(string text, Vehicle? vehicle, bool warning)
    {
        try
        {
            History?.Invoke(text, (vehicle ?? Program.ControlledVehicle)?.Id ?? "", warning);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-alert-history:" + ex.GetType().Name, $"[AFC] Alert history failed: {ex.Message}");
        }
    }
}
