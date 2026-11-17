using System;
using System.Collections.Generic;
using System.Globalization;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// Defaults of guidance settings that differ by body. The defaults of <see cref="GuidanceSettingSpecs"/> hold on every body,
/// and a body named here overrides single settings on top of them. A craft takes the defaults of the body whose sphere of
/// influence it is in (see BodySettings.Follow), so a value the player never changed is the default of where the craft is.
/// The built-in table is extended or overridden by body-defaults.toml in the mod folder, one table per body id:
/// <code>
/// [Luna]
/// ConvexHandoverAltKm = 10
/// FlyConvexAscent = false
/// </code>
/// </summary>
internal static class BodyDefaults
{
    internal const string FileName = "body-defaults.toml";

    private static readonly Dictionary<string, Dictionary<string, double>> BuiltInDoubles = new(StringComparer.Ordinal)
    {
        ["Luna"] = new(StringComparer.Ordinal)
        {
            // No air to climb through: the solved climb has nothing to shape past the pitch-over, so UPFG takes it low.
            [GuidanceSettingSpecs.ConvexHandoverAltKm.Key] = 10.0,
            // A low lunar orbit, not the 200 x 420 km of the home body.
            [GuidanceSettingSpecs.PeKm.Key] = 30.0,
            [GuidanceSettingSpecs.ApKm.Key] = 50.0,
        },
    };

    private static readonly Dictionary<string, Dictionary<string, bool>> BuiltInBools = new(StringComparer.Ordinal);

    private static Dictionary<string, Dictionary<string, double>> _doubles = Copy(BuiltInDoubles);
    private static Dictionary<string, Dictionary<string, bool>> _bools = Copy(BuiltInBools);

    /// <summary>The default of <paramref name="spec"/> on <paramref name="body"/>: the body's override, or the default of every body. An empty or null body is no body.</summary>
    public static double DefaultOf(in SettingSpec spec, string? body)
        => !string.IsNullOrEmpty(body) && _doubles.TryGetValue(body, out var table) && table.TryGetValue(spec.Key, out double value) ? value : spec.Default;

    public static bool DefaultOf(in BoolSpec spec, string? body)
        => !string.IsNullOrEmpty(body) && _bools.TryGetValue(body, out var table) && table.TryGetValue(spec.Key, out bool value) ? value : spec.Default;

    /// <summary><paramref name="spec"/> with the default it has on <paramref name="body"/>, for every check, reset and ghost text that reads a default.</summary>
    public static SettingSpec On(in SettingSpec spec, string? body) => Overrides(body, spec.Key) ? spec with { Default = DefaultOf(spec, body) } : spec;

    public static BoolSpec On(in BoolSpec spec, string? body) => Overrides(body, spec.Key) ? spec with { Default = DefaultOf(spec, body) } : spec;

    /// <summary>Whether <paramref name="body"/> has its own default for the setting <paramref name="key"/>.</summary>
    public static bool Overrides(string? body, string key)
        => !string.IsNullOrEmpty(body)
           && ((_doubles.TryGetValue(body, out var d) && d.ContainsKey(key)) || (_bools.TryGetValue(body, out var b) && b.ContainsKey(key)));

    /// <summary>The tooltip line of a default, naming the body when the default is the body's own.</summary>
    public static string GhostText(in SettingSpec spec, string? body)
    {
        SettingSpec on = On(spec, body);
        string text = SettingMath.GhostText(on);
        return Overrides(body, spec.Key) ? BodyWord(body!) + " " + text : text;
    }

    public static string GhostText(in BoolSpec spec, string? body)
    {
        BoolSpec on = On(spec, body);
        string text = SettingMath.GhostText(on);
        return Overrides(body, spec.Key) ? BodyWord(body!) + " " + text : text;
    }

    private static string BodyWord(string body) => body.ToUpperInvariant();

    /// <summary>Puts back the built-in table alone.</summary>
    public static void Reset()
    {
        _doubles = Copy(BuiltInDoubles);
        _bools = Copy(BuiltInBools);
    }

    /// <summary>
    /// Lays the lines of a body-defaults file over the built-in table, replacing any file read before. A key that is no
    /// setting, a value that does not parse and a number out of the range of its setting are skipped, and
    /// <paramref name="unread"/> names them, empty when everything was taken. Returns the count of values taken.
    /// </summary>
    public static int Load(IEnumerable<string> lines, out string unread)
    {
        var doubles = Copy(BuiltInDoubles);
        var bools = Copy(BuiltInBools);
        var skipped = new List<string>();
        int taken = 0;
        string? body = null;
        int lineNo = 0;

        foreach (string raw in lines)
        {
            lineNo++;
            string line = StripComment(raw).Trim();
            if (line.Length == 0)
                continue;
            if (line[0] == '[')
            {
                body = line.Length > 2 && line[^1] == ']' && line[1] != '[' ? Unquote(line[1..^1].Trim()) : null;
                if (string.IsNullOrEmpty(body))
                    skipped.Add($"line {lineNo}: table {line}");
                continue;
            }
            int eq = line.IndexOf('=');
            if (eq <= 0 || body == null)
            {
                skipped.Add($"line {lineNo}: {line}");
                continue;
            }
            string key = line[..eq].Trim();
            string value = line[(eq + 1)..].Trim();
            if (TryFindDouble(key, out SettingSpec spec))
            {
                if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out double number)
                    && double.IsFinite(number) && number >= spec.Min && number <= spec.Max)
                {
                    Table(doubles, body)[key] = number;
                    taken++;
                }
                else
                    skipped.Add($"{body}.{key} = {value} (out of {spec.Min.ToString(CultureInfo.InvariantCulture)}..{spec.Max.ToString(CultureInfo.InvariantCulture)} or no number)");
            }
            else if (TryFindBool(key, out _))
            {
                if (bool.TryParse(value, out bool on))
                {
                    Table(bools, body)[key] = on;
                    taken++;
                }
                else
                    skipped.Add($"{body}.{key} = {value} (not true or false)");
            }
            else
                skipped.Add($"{body}.{key} (no such setting)");
        }

        _doubles = doubles;
        _bools = bools;
        unread = string.Join("; ", skipped);
        return taken;
    }

    private static bool TryFindDouble(string key, out SettingSpec found)
    {
        foreach (SettingSpec spec in GuidanceSettingSpecs.AllDoubles)
            if (spec.Key == key)
            {
                found = spec;
                return true;
            }
        found = default;
        return false;
    }

    private static bool TryFindBool(string key, out BoolSpec found)
    {
        foreach (BoolSpec spec in GuidanceSettingSpecs.AllBools)
            if (spec.Key == key)
            {
                found = spec;
                return true;
            }
        found = default;
        return false;
    }

    private static Dictionary<string, T> Table<T>(Dictionary<string, Dictionary<string, T>> all, string body)
    {
        if (!all.TryGetValue(body, out var table))
            all[body] = table = new Dictionary<string, T>(StringComparer.Ordinal);
        return table;
    }

    private static Dictionary<string, Dictionary<string, T>> Copy<T>(Dictionary<string, Dictionary<string, T>> from)
    {
        var copy = new Dictionary<string, Dictionary<string, T>>(StringComparer.Ordinal);
        foreach (var (body, table) in from)
            copy[body] = new Dictionary<string, T>(table, StringComparer.Ordinal);
        return copy;
    }

    private static string StripComment(string line)
    {
        int hash = line.IndexOf('#');
        return hash >= 0 ? line[..hash] : line;
    }

    private static string Unquote(string text)
        => text.Length >= 2 && text[0] == '"' && text[^1] == '"' ? text[1..^1] : text;
}
