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
        // AFC_XVC_Q / AFC_XVC_W: the virtual control's attitude (quaternion components) and body-rate scales, rad/s (Ksa6DofSetup.VirtualControlAttitudeScale, VirtualControlRateScaleRadS), which live in an array.
        if (cfg.VirtualControlScale is { Length: >= 13 } scale)
        {
            if (Number("AFC_XVC_Q") is double q)
                for (int i = 6; i <= 9; i++)
                    scale[i] = q;
            if (Number("AFC_XVC_W") is double w)
                for (int i = 10; i <= 12; i++)
                    scale[i] = w;
        }
    }

    private static double? Number(string name)
        => double.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) && v > 0.0 ? v : null;

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
