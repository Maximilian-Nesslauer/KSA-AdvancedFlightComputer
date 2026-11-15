using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>How steep the ground at the landing site is against what the craft can stand on, and the words of the LAND page for it.</summary>
internal enum SlopeVerdict { Unknown, Fits, TipsOver, CannotStand }

internal static class SiteSlopeText
{
    public const string FindVerb = "FIND FLAT";
    public const string SearchingVerb = "SEARCHING";
    public const string MoveVerb = "MOVE SITE";

    /// <summary>The verdict of a slope against the tip-over limit and the limit above which no heading stands, all in degrees. A limit that is not known gives no verdict.</summary>
    public static SlopeVerdict Verdict(double slopeDeg, double tipDeg, double standDeg)
    {
        if (!double.IsFinite(slopeDeg) || !double.IsFinite(tipDeg) || !double.IsFinite(standDeg))
            return SlopeVerdict.Unknown;
        if (slopeDeg > standDeg)
            return SlopeVerdict.CannotStand;
        return slopeDeg > tipDeg ? SlopeVerdict.TipsOver : SlopeVerdict.Fits;
    }

    /// <summary>The verb of the search button: FIND FLAT for the first radius, and the distance for a wider one.</summary>
    public static string FindVerbFor(double radiusM, double firstRadiusM)
        => radiusM <= firstRadiusM ? FindVerb : FormattableString.Invariant($"FIND IN {radiusM / 1000.0:0.0} km");

    /// <summary>The hover card of the search button when the last answer still tips the craft over.</summary>
    public static string WiderCard(double answerDeg, double tipDeg, double radiusM)
        => FormattableString.Invariant($"The flattest ground found is {answerDeg:F1} deg, and this craft goes over above {tipDeg:F1} deg. The next search looks {radiusM:F0} m around the site.");

    /// <summary>The hover card of the search button when the widest search has its answer and that still tips the craft over.</summary>
    public static string WidestCard(double answerDeg, double tipDeg, double radiusM)
        => FormattableString.Invariant($"The flattest ground within {radiusM:F0} m of the site is {answerDeg:F1} deg, and this craft goes over above {tipDeg:F1} deg. No search looks further. Pick another site.");

    public static string SlopeLabel(double slopeDeg) => FormattableString.Invariant($"GROUND {slopeDeg:F1} DEG");

    public static string LimitDetail(double tipDeg) => FormattableString.Invariant($"TIPS OVER ABOVE {tipDeg:F1}");

    /// <summary>The hover card of the slope row: what the numbers are and the rule behind the limit.</summary>
    public static string Card(double slopeDeg, double tipDeg, double standDeg, double footprintM) => FormattableString.Invariant(
        $"The slope of the ground at the site, from terrain samples over the footprint of the legs ({footprintM:F1} m from the axis). This craft goes over above {tipDeg:F1} deg: it lands upright, meets the ground with the uphill feet first and falls onto the downhill feet, and above that slope the fall carries it over them, for the worst heading of the legs and for any propellant left at the touchdown, also with the lowest tanks run empty first, which keeps the centre of mass highest. No heading stands above {standDeg:F1} deg. {FindVerb} looks for flatter ground near the site.");

    /// <summary>The footer fact of a site that the craft cannot land on or that tips it over, or empty.</summary>
    public static string Fact(SlopeVerdict verdict, double slopeDeg, double tipDeg, double standDeg) => verdict switch
    {
        SlopeVerdict.CannotStand => FormattableString.Invariant($"GROUND {slopeDeg:F0} DEG: NO STAND ABOVE {standDeg:F0} DEG. {FindVerb}."),
        SlopeVerdict.TipsOver => FormattableString.Invariant($"GROUND {slopeDeg:F0} DEG: TIPS OVER ABOVE {tipDeg:F0} DEG. {FindVerb}."),
        _ => "",
    };

    /// <summary>The refusal of DEORBIT + LAND for a site on which no heading of the craft stands, or empty.</summary>
    public static string Refusal(SlopeVerdict verdict, double slopeDeg, double standDeg)
        => verdict == SlopeVerdict.CannotStand
            ? FormattableString.Invariant($"The ground at the site slopes {slopeDeg:F1} deg, and this craft cannot stand on more than {standDeg:F1} deg with any heading. Pick another site or use {FindVerb}.")
            : "";

    public static string Searching(double radiusM, double share) => FormattableString.Invariant($"SEARCHING {radiusM:F0} m: {share * 100.0:F0} %");

    /// <summary>The result of a search: the flattest point, how far and which way, or that the site is already as flat as anything near it.</summary>
    public static string Found(double slopeDeg, double distanceM, double bearingDeg, double siteSlopeDeg, double radiusM)
        => IsBetter(slopeDeg, siteSlopeDeg)
            ? FormattableString.Invariant($"FLAT {slopeDeg:F1} DEG, {distanceM:F0} m {Compass(bearingDeg)}")
            : FormattableString.Invariant($"NO FLATTER GROUND IN {radiusM:F0} m");

    /// <summary>A point counts as flatter when it is a tenth of a degree or more below the site.</summary>
    public static bool IsBetter(double slopeDeg, double siteSlopeDeg)
        => double.IsFinite(slopeDeg) && (!double.IsFinite(siteSlopeDeg) || slopeDeg <= siteSlopeDeg - 0.1);

    /// <summary>One of eight compass words for a bearing in degrees clockwise from north.</summary>
    public static string Compass(double bearingDeg)
    {
        string[] words = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
        double b = ((bearingDeg % 360.0) + 360.0) % 360.0;
        return words[(int)Math.Floor((b + 22.5) / 45.0) % 8];
    }
}
