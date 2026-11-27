using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// HARNESS EXPERIMENTS ON THE TOUCHDOWN PLAN: an environment variable AFC_XCFG_&lt;name&gt; sets the Scvx6DofConfig property of that name on every touchdown config the setup builds (Ksa6DofSetup.TryBuild), a number, or true/false for a switch - AFC_XCFG_SinkPerM=0.5, AFC_XCFG_TipLimit=false. Read once per process. Unset, as in the game, it changes nothing.
///  WHY (2026-11-27 campaign). The user asked for the day's touchdown ideas to be turned on and off one at a time across many headless flights; one build that takes any setting from the run spares a build and deploy per variant, and the harness profiles run side by side.
/// </summary>
internal static class TouchdownExperiment
{
    private const string Prefix = "AFC_XCFG_";
    private static List<(PropertyInfo Property, object Value)>? _overrides;

    /// <summary>The overrides in force, "name=value; ..." for the log, or empty.</summary>
    internal static string Text { get; private set; } = "";

    internal static void Apply(Scvx6DofConfig cfg)
    {
        _overrides ??= Read();
        foreach ((PropertyInfo property, object value) in _overrides)
            property.SetValue(cfg, value);
    }

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
            PropertyInfo? property = typeof(Scvx6DofConfig).GetProperty(name, BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase);
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
