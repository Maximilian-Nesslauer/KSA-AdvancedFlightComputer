using System;
using System.Globalization;
using System.Linq;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class Fmt
{
    public static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private const double SecondsOnlyLimit = 120.0;
    private const double DistanceKmFrom = 10000.0;

    public static string Time(double seconds)
    {
        if (!double.IsFinite(seconds))
            return "N/A";
        string sign = seconds < 0.0 ? "-" : "";
        double abs = Math.Abs(seconds);
        if (abs < SecondsOnlyLimit)
            return sign + abs.ToString("F0", Inv) + " s";
        long whole = (long)abs;
        if (whole < 3600)
            return sign + (whole / 60).ToString(Inv) + "m " + (whole % 60).ToString("00", Inv) + "s";
        if (whole < 86400)
            return sign + (whole / 3600).ToString(Inv) + "h " + (whole % 3600 / 60).ToString("00", Inv) + "m";
        long hours = whole % 86400 / 3600;
        return sign + (whole / 86400).ToString(Inv) + "d" + (hours > 0 ? " " + hours.ToString(Inv) + "h" : "");
    }

    public static string TimeFine(double seconds)
        => double.IsFinite(seconds) && Math.Abs(seconds) < SecondsOnlyLimit ? seconds.ToString("F1", Inv) + " s" : Time(seconds);

    /// <summary>A time span for a place with little room: under a minute in seconds, under an hour in whole minutes ("30m"), and above that as <see cref="Time"/>.</summary>
    public static string TimeShort(double seconds)
    {
        double minutes = Math.Round(Math.Abs(seconds) / 60.0);
        return Math.Abs(seconds) >= 60.0 && minutes < 60.0 ? (seconds < 0.0 ? "-" : "") + ((long)minutes).ToString(Inv) + "m" : Time(seconds);
    }

    /// <summary>The delta-v of one burn, to the tenth, because a correction can be well under 1 m/s.</summary>
    public static string Dv(double ms) => ms.ToString("F1", Inv) + " m/s";

    /// <summary>A budget of delta-v, such as a stage or what is on board, in whole m/s.</summary>
    public static string DvWhole(double ms) => ms.ToString("F0", Inv) + " m/s";

    public static string Speed(double ms) => ms.ToString("F1", Inv) + " m/s";

    public static string Distance(double meters)
        => Math.Abs(meters) < DistanceKmFrom ? meters.ToString("F0", Inv) + " m" : (meters / 1000.0).ToString("F1", Inv) + " km";

    public static string Height(double meters) => Math.Abs(meters) < 100.0 ? Meters(meters) : Distance(meters);

    public static string Meters(double meters) => meters.ToString("F1", Inv) + " m";

    public static string Km(double km) => km.ToString("F0", Inv) + " km";

    public static string KmFine(double km) => km.ToString("F1", Inv) + " km";

    public static string Orbit(double peKm, double apKm) => peKm.ToString("F0", Inv) + " x " + apKm.ToString("F0", Inv) + " km";

    public static string Deg(double degrees) => degrees.ToString("F1", Inv) + " deg";

    public static string LatLon(double latDeg, double lonDeg)
        => "lat " + latDeg.ToString("F3", Inv) + ", lon " + lonDeg.ToString("F3", Inv);

    public static string Percent(double fraction) => (fraction * 100.0).ToString("F0", Inv) + " %";

    public static string Tons(double kg) => (kg / 1000.0).ToString("F1", Inv) + " t";

    public static string Kg(double kg) => kg.ToString("F0", Inv) + " kg";

    public static string Mass(double kg) => Math.Abs(kg) < 1000.0 ? Kg(kg) : Tons(kg);

    public static bool TryParseTime(string text, out double seconds)
    {
        seconds = 0.0;
        string[] parts = (text ?? "").Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length == 0)
            return false;
        bool any = false;
        for (int i = 0; i < parts.Length; i++)
        {
            string part = parts[i];
            int digits = part.Length;
            while (digits > 0 && char.IsLetter(part[digits - 1]))
                digits--;
            string unit = part.Substring(digits);
            if (unit.Length == 0 && i + 1 < parts.Length && parts[i + 1].All(char.IsLetter))
                unit = parts[++i];
            if (!double.TryParse(part.AsSpan(0, digits), NumberStyles.Float, Inv, out double value) || value < 0.0 || !double.IsFinite(value))
                return false;
            double scale = unit.ToLowerInvariant() switch
            {
                "" or "s" => 1.0,
                "m" => 60.0,
                "h" => 3600.0,
                "d" => 86400.0,
                _ => double.NaN,
            };
            if (double.IsNaN(scale))
                return false;
            seconds += value * scale;
            any = true;
        }
        return any;
    }

    public static string Fixed(double value, int digits)
    {
        double rounded = Math.Round(value, digits);
        return (rounded == 0.0 ? 0.0 : rounded).ToString("F" + digits.ToString(Inv), Inv);
    }

    public static string Kpa(double pascal) => (pascal / 1000.0).ToString("F1", Inv) + " kPa";

    public static string Mn(double newton) => (newton / 1e6).ToString("F2", Inv) + " MN";

    public static string Ratio(double value) => value.ToString("F2", Inv);

    public static string Sentence(string text)
    {
        if (string.IsNullOrEmpty(text) || !char.IsLower(text[0]))
            return text;
        return char.ToUpperInvariant(text[0]) + text.Substring(1);
    }
}
