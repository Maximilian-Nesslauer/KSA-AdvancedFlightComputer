using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum CraftSituation { Unknown, Ground, Orbit, CoastingUp, FallingAir, FallingNoAir, Escaping }

/// <summary>An altitude that is not known is NaN.</summary>
internal readonly record struct SituationFacts(
    CraftSituation Where, string Body = "", double PeKm = double.NaN, double ApKm = double.NaN, bool Thrust = false, bool HasAir = false,
    double HeightM = double.NaN, double ImpactS = double.NaN);

internal static class SituationText
{
    public const string YouFly = "You fly.";

    public static CraftSituation Classify(bool contact, bool bound, double periapsis, double nearSurfaceRadius, double radialSpeed, bool hasAir)
    {
        if (contact)
            return CraftSituation.Ground;
        if (double.IsNaN(periapsis) || double.IsNaN(nearSurfaceRadius) || double.IsNaN(radialSpeed))
            return CraftSituation.Unknown;
        bool clears = periapsis > nearSurfaceRadius;
        bool rising = radialSpeed > 0.0;
        if (bound && clears)
            return CraftSituation.Orbit;
        if (!bound && (clears || rising))
            return CraftSituation.Escaping;
        if (rising)
            return CraftSituation.CoastingUp;
        return hasAir ? CraftSituation.FallingAir : CraftSituation.FallingNoAir;
    }

    public static bool IsFalling(CraftSituation where) => where is CraftSituation.FallingAir or CraftSituation.FallingNoAir;

    public static bool ImpactKnown(in SituationFacts f) => IsFalling(f.Where) && double.IsFinite(f.ImpactS) && f.ImpactS > 0.0;

    public static (int Start, int Length) ImpactWords(string line)
    {
        int at = line.IndexOf("Impact in ", StringComparison.Ordinal);
        if (at < 0)
            return (-1, 0);
        int end = line.IndexOf('.', at);
        return (at, (end < 0 ? line.Length : end + 1) - at);
    }

    private static string FallWord(in SituationFacts f)
        => f.Thrust && !(double.IsFinite(f.ImpactS) && f.ImpactS > 0.0) ? "Descending" : "Falling";

    private static string FallFacts(in SituationFacts f, bool height = true, bool impact = true)
    {
        string h = height && double.IsFinite(f.HeightM) ? Fmt.Height(f.HeightM) : "";
        string t = impact && double.IsFinite(f.ImpactS) && f.ImpactS > 0.0 ? "Impact in " + Fmt.Time(f.ImpactS) : "";
        return h.Length > 0 && t.Length > 0 ? h + ". " + t : h + t;
    }

    public static string Place(in SituationFacts f, bool orbitShown = false)
    {
        string state = f.Where switch
        {
            CraftSituation.Ground => "On the ground",
            CraftSituation.Orbit => "In orbit",
            CraftSituation.CoastingUp => f.Thrust ? "Climbing" : "Coasting up",
            CraftSituation.FallingAir or CraftSituation.FallingNoAir => FallWord(in f),
            CraftSituation.Escaping => "On an escape path",
            _ => "",
        };
        if (state.Length == 0)
            return "";
        string body = f.Body.Length > 0 ? ", " + f.Body : "";
        string fall = IsFalling(f.Where) ? FallFacts(in f) : "";
        string detail = f.Where switch
        {
            CraftSituation.FallingAir or CraftSituation.FallingNoAir when fall.Length > 0 => ", " + fall,
            CraftSituation.Orbit when !orbitShown && double.IsFinite(f.PeKm) && double.IsFinite(f.ApKm) => ", " + Fmt.Orbit(f.PeKm, f.ApKm),
            CraftSituation.CoastingUp when !orbitShown && double.IsFinite(f.ApKm) => ", Ap " + Fmt.Km(f.ApKm),
            CraftSituation.FallingNoAir => ", no air",
            _ => "",
        };
        return state + body + detail;
    }

    public static string Line(in SituationFacts f, string who = YouFly)
    {
        string place = Place(in f);
        return place.Length > 0 ? place + ". " + who : who;
    }

    public static string LineThatFits(in SituationFacts f, Func<string, float> measure, float room, string who = YouFly)
    {
        string full = Line(in f, who);
        string fall = IsFalling(f.Where) ? FallFacts(in f) : "";
        if (fall.Length == 0 || measure(full) <= room)
            return full;
        string state = FallWord(in f);
        string noBody = state + ", " + fall + ". " + who;
        if (measure(noBody) <= room)
            return noBody;
        string impact = FallFacts(in f, height: false);
        string noHeight = impact.Length > 0 ? state + ". " + impact + ". " + who : state + ". " + who;
        if (measure(noHeight) <= room)
            return noHeight;
        string noWord = impact.Length > 0 ? impact + ". " + who : who;
        return measure(noWord) <= room ? noWord : who;
    }

    public static string MessageLine(string message, Func<string, float> measure, float room, string who = YouFly)
    {
        string after = message + " " + who;
        if (measure(after) <= room)
            return after;
        string brief = PageText.FinishedBrief(message) + " " + who;
        return brief != after && measure(brief) <= room ? brief : message;
    }

    public static string EngineLeftLine(in SituationFacts f, double? percent)
        => IsFalling(f.Where) && percent is double p && p >= 0.5 ? "Engine at " + p.ToString("F0", Fmt.Inv) + " %." : "";

    public static string FlyingPlace(in SituationFacts f, bool orbitShown = false, bool heightShown = false, bool impactShown = false)
    {
        switch (f.Where)
        {
            case CraftSituation.CoastingUp:
                return !orbitShown && double.IsFinite(f.ApKm) ? Joined(f.Body, "Ap " + Fmt.Km(f.ApKm)) : f.Body;
            case CraftSituation.FallingAir:
            case CraftSituation.FallingNoAir:
                return Joined(f.Body, FallFacts(in f, !heightShown, !impactShown));
            default:
                return Place(in f, orbitShown);
        }
    }

    private static string Joined(string body, string detail)
        => body.Length > 0 && detail.Length > 0 ? body + ", " + detail : body + detail;
}
