using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// HARNESS EXPERIMENTS ON BELLYFLOP'S PLAN, as TouchdownExperiment's on touchdown's: an environment variable AFC_XBF_&lt;name&gt; sets the BellyflopConfig property of that name on every config bellyflop builds (BellyflopConfigNow), a number or true/false - AFC_XBF_BankWeight=0.1, AFC_XBF_EndLevelS=30. Read once per process. Unset, as in the game, it changes nothing.
///  WHY (2026-11-27). Bellyflop's shaping (bank against yaw, levelling by the flip) is tried across many headless flights from one build, side by side in the harness profiles.
/// </summary>
internal static class BellyflopExperiment
{
    private const string Prefix = "AFC_XBF_";
    private static List<(PropertyInfo Property, object Value)>? _overrides;

    /// <summary>The overrides in force, "name=value; ..." for the log, or empty.</summary>
    internal static string Text { get; private set; } = "";

    internal static BellyflopConfig Apply(BellyflopConfig cfg)
    {
        _overrides ??= Read();
        foreach ((PropertyInfo property, object value) in _overrides)
            property.SetValue(cfg, value);
        return cfg;
    }

    /// <summary>A flight-side number by name (AFC_XBF_&lt;name&gt;), or the fallback where unset.</summary>
    internal static double Number(string name, double fallback)
        => double.TryParse(Environment.GetEnvironmentVariable(Prefix + name), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : fallback;

    private static List<(PropertyInfo, object)> Read()
    {
        var list = new List<(PropertyInfo, object)>();
        var words = new List<string>();
        foreach (System.Collections.DictionaryEntry entry in Environment.GetEnvironmentVariables())
        {
            string key = entry.Key as string ?? "";
            if (!key.StartsWith(Prefix, StringComparison.OrdinalIgnoreCase))
                continue;
            string name = key[Prefix.Length..];
            string raw = (entry.Value as string ?? "").Trim();
            PropertyInfo? property = typeof(BellyflopConfig).GetProperty(name, BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase);
            if (property?.SetMethod == null)
                continue;
            object? value = property.PropertyType == typeof(bool) ? bool.TryParse(raw, out bool b) ? b : raw == "1" ? true : raw == "0" ? false : null
                : property.PropertyType == typeof(double) && double.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out double d) ? d
                : property.PropertyType == typeof(int) && int.TryParse(raw, NumberStyles.Integer, CultureInfo.InvariantCulture, out int i) ? i
                : null;
            if (value == null)
                continue;
            list.Add((property, value));
            words.Add(FormattableString.Invariant($"{property.Name}={value}"));
        }
        Text = string.Join("; ", words);
        return list;
    }
}
