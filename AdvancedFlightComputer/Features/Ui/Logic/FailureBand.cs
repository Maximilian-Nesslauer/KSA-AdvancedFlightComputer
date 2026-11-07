using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum EngineNow { Off, On, StockAuto }

/// <summary><see cref="ImpactSeconds"/> is NaN when the time to impact is not known.</summary>
internal readonly record struct BandFacts(string What, EngineNow Engine, string Who, double ImpactSeconds = double.NaN);

internal static class FailureBand
{
    public static string Text(in BandFacts f)
    {
        string text = f.What + ". " + EngineWords(f.Engine) + ". " + FlownBy.Sentence(f.Who) + ".";
        return double.IsFinite(f.ImpactSeconds) && f.ImpactSeconds > 0.0
            ? text + " " + Fmt.Time(f.ImpactSeconds) + " to impact."
            : text;
    }

    public static string RowLine(in BandFacts f, Func<string, float> measure, float room)
    {
        string full = Text(in f);
        if (measure(full) <= room)
            return full;
        string impact = double.IsFinite(f.ImpactSeconds) && f.ImpactSeconds > 0.0 ? " " + Fmt.Time(f.ImpactSeconds) + " to impact." : "";
        string who = " " + FlownBy.Sentence(f.Who) + ".";
        string noEngine = f.What + "." + who + impact;
        if (measure(noEngine) <= room)
            return noEngine;
        string shortWhat = ShortWhat(f.What);
        string noReason = shortWhat + "." + who + impact;
        if (measure(noReason) <= room)
            return noReason;
        string whatAndTime = shortWhat + "." + impact;
        if (measure(whatAndTime) <= room)
            return whatAndTime;
        return shortWhat.Replace(" guidance", "", StringComparison.Ordinal) + "." + impact;
    }

    public static bool StripStays(bool busy, bool failureWaits, bool failureAnswered, in SituationFacts f)
        => busy || failureWaits || (failureAnswered && SituationText.ImpactKnown(in f));

    public static string ToImpact(double seconds) => Fmt.Time(seconds) + " to impact.";

    public static string ShortWhat(string what)
        => what.StartsWith("G-FOLD ", StringComparison.Ordinal) ? "G-FOLD failed" : what;

    public static string EngineWords(EngineNow engine) => engine switch
    {
        EngineNow.On => "Engine on",
        EngineNow.StockAuto => "Stock AUTO has the engine",
        _ => "Engine off",
    };
}

internal readonly record struct CraftFailure(string What, string Detail, string Line = "", int Serial = 0);

internal sealed class FailureAcks
{
    private readonly Dictionary<string, HashSet<(string What, int Serial)>> _acknowledged = new(StringComparer.Ordinal);

    public void Update(string craftId, IReadOnlyList<CraftFailure> current)
    {
        if (!_acknowledged.TryGetValue(craftId, out var seen))
            return;
        seen.RemoveWhere(id =>
        {
            for (int i = 0; i < current.Count; i++)
            {
                if (current[i].What == id.What && current[i].Serial == id.Serial)
                    return false;
            }
            return true;
        });
        if (seen.Count == 0)
            _acknowledged.Remove(craftId);
    }

    public bool IsAcknowledged(string craftId, string what, int serial)
        => what.Length > 0 && _acknowledged.TryGetValue(craftId, out var seen) && seen.Contains((what, serial));

    public void Acknowledge(string craftId, string what, int serial)
    {
        if (what.Length == 0)
            return;
        if (!_acknowledged.TryGetValue(craftId, out var seen))
            _acknowledged[craftId] = seen = new HashSet<(string What, int Serial)>();
        seen.Add((what, serial));
    }

    public void Clear() => _acknowledged.Clear();
}
