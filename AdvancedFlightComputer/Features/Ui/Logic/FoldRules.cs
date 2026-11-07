using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct FoldPart(string Name, bool Shows, Action? Draw = null, string Summary = "");

internal static class FoldNames
{
    public const string Plane = "PLANE";
    public const string Insertion = "INSERTION";
    public const string Turn = "TURN";
    public const string ConvexPlan = "SOLVED";
    public const string InMap = PageText.ShowInMap;
    public const string VehicleHeight = "COM";
    public const string Approach = "APPROACH";
    public const string Burn = "BURN";
    public const string Glide = "GLIDE";
    public const string BurnLimit = "BURN LIMIT";
    public const string HoverLimits = "HOVER LIMITS";
    public const string InWorld = PageText.ShowInWorld;
    public const string Search = "SEARCH";
    public const string Site = "SITE";
    public const string Passes = "PASSES";
    public const string Split = "PER PASS";
    public const string CraftTarget = "CRAFT TARGET";
    public const string FlightTime = "TOF";
    public const string Flyby = "FLYBY";
    public const string Allocator = "THRUSTER MIX";
    public const string Attitude = "ATTITUDE";
    public const string ThisCraft = "GNC SWITCHES";
    public const string NewCraft = "DEFAULTS FOR NEW CRAFT";
    public const string FlightPlan = "FLIGHT PLAN";
    public const string Distance = "DISTANCE";
    public const string Terrain = "TERRAIN";
    public const string TelemetryLog = "TELEMETRY LOG";
    public const string Impact = "IMPACT";
    public const string Feasibility = "FEASIBILITY";
    public const string Deorbit = "DEORBIT";
}

internal static class FoldRules
{
    public static bool HasRows(ReadOnlySpan<FoldPart> parts)
    {
        foreach (FoldPart part in parts)
            if (part.Shows && part.Name.Length > 0)
                return true;
        return false;
    }

    public static string Contents(ReadOnlySpan<FoldPart> parts)
    {
        var names = new List<string>(parts.Length);
        foreach (FoldPart part in parts)
            if (part.Shows && part.Name.Length > 0)
                names.Add((part.Summary.Length > 0 ? part.Summary : part.Name).ToLowerInvariant());
        return string.Join(", ", names);
    }

    public static string Label(string name, ReadOnlySpan<FoldPart> parts, int changed = 0)
    {
        string contents = Contents(parts);
        string count = changed > 0 ? changed.ToString(Fmt.Inv) + " CHANGED" : "";
        string detail = contents.Length > 0 && count.Length > 0 ? contents + " (" + count + ")" : contents + count;
        return detail.Length > 0 ? name + ": " + detail.ToUpperInvariant() : name;
    }

    public static string FitSummary(string detail, float room, Func<string, float> width)
    {
        if (width(detail) <= room)
            return detail;
        string tail = "";
        string body = detail;
        int count = detail.LastIndexOf(" (", StringComparison.Ordinal);
        if (count > 0 && detail.EndsWith(" CHANGED)", StringComparison.Ordinal))
        {
            tail = detail.Substring(count);
            body = detail.Substring(0, count);
        }
        for (int comma = body.LastIndexOf(", ", StringComparison.Ordinal); comma > 0; comma = body.LastIndexOf(", ", StringComparison.Ordinal))
        {
            body = body.Substring(0, comma);
            if (width(body + tail) <= room)
                return body + tail;
        }
        for (int space = body.LastIndexOf(' '); space > 0; space = body.LastIndexOf(' '))
        {
            body = body.Substring(0, space);
            if (width(body) <= room)
                return body;
        }
        return width(body) <= room ? body : "";
    }

    public static string WholeOrNone(string detail, float room, Func<string, float> width) => width(detail) <= room ? detail : "";

    public static (string Name, string Detail) Split(string label)
    {
        int colon = label.IndexOf(": ", StringComparison.Ordinal);
        if (colon <= 0)
            return (label, "");
        for (int i = 0; i < colon; i++)
            if (char.IsLower(label[i]))
                return (label, "");
        return (label.Substring(0, colon), Fmt.Sentence(label.Substring(colon + 2)));
    }
}
