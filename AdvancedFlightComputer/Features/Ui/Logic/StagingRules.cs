using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class StagingRules
{
    // The stock panel prints no delta-v under 1 m/s.
    private const double SpentDvMs = 0.5;

    public const string IgnitionWord = "IGNITION";
    public const string DecouplerWord = "DECOUPLER";

    public static bool HoldsDv(double dvMs) => dvMs > SpentDvMs;

    public static bool HasBox(bool activated, int number, int activeSequence, double dvMs)
        => !activated || (number == activeSequence && dvMs > SpentDvMs);

    /// <summary>The answer of the picture: "11702 m/s, 9 sequences".</summary>
    public static string Total(double dvMs, int sequences)
        => Fmt.DvWhole(dvMs) + ", " + sequences.ToString(Fmt.Inv) + (sequences == 1 ? " SEQUENCE" : " SEQUENCES");

    public static string BoxLabel(int number, bool next, bool withWord, bool burns = false)
        => (next ? "> " : "") + (withWord ? "SEQ " : "") + number.ToString(Fmt.Inv) + (burns ? " BURNS" : "");

    public static string Delay(string word, double seconds)
        => (word.Length > 0 ? word + " " : "") + "+" + seconds.ToString("F1", Fmt.Inv) + " s";

    public static bool DelayShows(bool burns, double seconds) => !burns && seconds > 0.0;

    /// <summary>What a box says under its name: the delta-v it holds in m/s, or a dash, because the caption of the picture promises a value for every box.</summary>
    public static string BoxDv(double dvMs) => HoldsDv(dvMs) ? dvMs.ToString("F0", Fmt.Inv) : "-";

    public static string Card(int number, double dvMs, bool active, bool next, double ignitionS, double decouplerS)
    {
        var lines = new List<string> { "SEQ " + number.ToString(Fmt.Inv) + ": " + Fmt.DvWhole(dvMs) + "." };
        if (active)
            lines.Add("Burns now.");
        if (next)
            lines.Add("Fires next.");
        if (ignitionS > 0.0)
            lines.Add("Engines light " + ignitionS.ToString("F1", Fmt.Inv) + " s after the staging trigger.");
        if (decouplerS > 0.0)
            lines.Add("Decouplers fire " + decouplerS.ToString("F1", Fmt.Inv) + " s after the staging trigger.");
        return string.Join(" ", lines);
    }
}
