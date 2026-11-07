using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum CircularizeAt { Apoapsis, Periapsis }

/// <summary>
/// The five quick orbit tools and the six string keys they are known by, one for each side of CIRCULARIZE. The keys are the plan type
/// keys the tools had in the transfer planner, so the multi-pass intents and multipass.toml keep
/// reading the same strings.
/// </summary>
internal static class OrbitToolCatalog
{
    public const string KeySetPeriapsis = "AFC Set Periapsis";
    public const string KeySetApoapsis = "AFC Set Apoapsis";
    public const string KeyMatchInclination = "AFC Match Inclination";
    public const string KeySetInclination = "AFC Set Inclination";
    public const string KeyCircularizeApoapsis = "Circularize Apoapsis";
    public const string KeyCircularizePeriapsis = "Circularize Periapsis";

    public static readonly string[] AtLabels = { "APOAPSIS", "PERIAPSIS" };

    public const double CircularEccentricity = 0.001;

    public const string CueCircularize = "CIRCULARIZE";

    public static string? KeyOfCue(string subPage) => subPage == CueCircularize ? KeyCircularizeApoapsis : null;

    public static string StartKey(double eccentricity)
        => eccentricity < CircularEccentricity ? KeySetApoapsis : KeyCircularizeApoapsis;

    /// <summary>The five tools of the TOOL tabs above the MANEUVER picture. CIRCULARIZE takes its side from the AT row.</summary>
    public static readonly string[] ToolLabels = { "CIRCULARIZE", "SET AP", "SET PE", "MATCH", "SET INC" };

    /// <summary>What each tool of <see cref="ToolLabels"/> does, for the hover card of its tab.</summary>
    public static readonly string[] ToolDescriptions =
    {
        "Makes the orbit round at the point you pick.",
        "Moves apoapsis to the value you set.",
        "Moves periapsis to the value you set.",
        "Turns the orbit plane onto the plane of the target.",
        "Turns the orbit to the inclination you set.",
    };

    /// <summary>What the tool of a TOOL tab does, or empty for an index that is no tool.</summary>
    public static string Description(int tool) => tool >= 0 && tool < ToolDescriptions.Length ? ToolDescriptions[tool] : "";

    /// <summary>The TOOL segment of a key, -1 for a key that is no tool.</summary>
    public static int ToolIndex(string? key) => key switch
    {
        KeyCircularizeApoapsis or KeyCircularizePeriapsis => 0,
        KeySetApoapsis => 1,
        KeySetPeriapsis => 2,
        KeyMatchInclination => 3,
        KeySetInclination => 4,
        _ => -1,
    };

    /// <summary>The key of a TOOL segment, with the circularize side from <paramref name="at"/>.</summary>
    public static string KeyForTool(int index, CircularizeAt at) => index switch
    {
        0 => at == CircularizeAt.Apoapsis ? KeyCircularizeApoapsis : KeyCircularizePeriapsis,
        2 => KeySetPeriapsis,
        3 => KeyMatchInclination,
        4 => KeySetInclination,
        _ => KeySetApoapsis,
    };

    /// <summary>Whether a tool turns the orbit plane, so the picture is the plane sketch instead of the orbit sketch.</summary>
    public static bool IsPlaneTool(string? key) => key == KeyMatchInclination || key == KeySetInclination;

    /// <summary>The name a player reads for a tool key; any other key is returned as it is.</summary>
    public static string DisplayName(string key) => key switch
    {
        KeySetPeriapsis => "SET PERIAPSIS",
        KeySetApoapsis => "SET APOAPSIS",
        KeyMatchInclination => "MATCH INCLINATION",
        KeySetInclination => "SET INCLINATION",
        KeyCircularizeApoapsis => "CIRCULARIZE AT APOAPSIS",
        KeyCircularizePeriapsis => "CIRCULARIZE AT PERIAPSIS",
        _ => key,
    };

    public static bool IsTool(string? key)
        => key == KeySetPeriapsis || key == KeySetApoapsis || key == KeyMatchInclination
           || key == KeySetInclination || key == KeyCircularizeApoapsis || key == KeyCircularizePeriapsis;

    /// <summary>Reads the circularize side from a circularize key. <paramref name="at"/> keeps its value for any other key.</summary>
    public static void CircularizeSide(string? key, ref CircularizeAt at)
    {
        if (key == KeyCircularizeApoapsis)
            at = CircularizeAt.Apoapsis;
        else if (key == KeyCircularizePeriapsis)
            at = CircularizeAt.Periapsis;
    }
}
