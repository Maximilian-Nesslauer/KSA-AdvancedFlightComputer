using System;
using System.Globalization;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum UiTier { Player, Expert, Tuning, Diagnostics }

/// <summary>
/// One numeric guidance input. <see cref="Key"/> is the name of the field on
/// unit, and <see cref="Format"/> is an ImGui printf format.
/// </summary>
internal readonly record struct SettingSpec(
    string Key, string Label, string Unit, UiTier Tier,
    double Default, double Min, double Max, string Format, string Help);

internal readonly record struct BoolSpec(string Key, string Label, UiTier Tier, bool Default, string Help);

internal static class SettingMath
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static double Clamp(in SettingSpec spec, double value)
    {
        if (double.IsNaN(value))
            return spec.Default;
        return Math.Clamp(value, spec.Min, spec.Max);
    }

    // Relative, so a large default such as a 500 m gate and a small one such as a 0.002 share both compare at their own scale.
    public static bool IsChanged(in SettingSpec spec, double value)
        => Math.Abs(value - spec.Default) > 1e-9 * Math.Max(1.0, Math.Abs(spec.Default));

    public static bool IsChanged(in BoolSpec spec, bool value) => value != spec.Default;

    public const string ShareUnit = "frac";

    public const string PercentFormat = "%.1f";

    public static bool ShowsPercent(in SettingSpec spec) => spec.Unit == ShareUnit;

    public static string ShownUnit(in SettingSpec spec) => ShowsPercent(spec) ? "%" : spec.Unit;

    public static string ShownFormat(in SettingSpec spec) => ShowsPercent(spec) ? PercentFormat : spec.Format;

    public static double ToShown(in SettingSpec spec, double value) => ShowsPercent(spec) ? value * 100.0 : value;

    public static double FromShown(in SettingSpec spec, double shown) => ShowsPercent(spec) ? shown / 100.0 : shown;

    public static string DisplayLabel(in SettingSpec spec)
    {
        string unit = ShownUnit(spec);
        return unit.Length > 0 ? spec.Label + " (" + unit + ")" : spec.Label;
    }

    public static string GhostText(in SettingSpec spec)
    {
        string number = DefaultValue(spec);
        string unit = ShownUnit(spec);
        return unit.Length > 0
            ? "DEFAULT " + number + " " + unit
            : "DEFAULT " + number;
    }

    public static string GhostText(in BoolSpec spec) => spec.Default ? "DEFAULT ON" : "DEFAULT OFF";

    public static string DefaultValue(in SettingSpec spec) => ToShown(spec, spec.Default).ToString(DotNetFormat(ShownFormat(spec)), Inv);

    public static string DefaultValue(in BoolSpec spec) => spec.Default ? "ON" : "OFF";

    /// <summary>Turns a printf format such as "%.2f" into the .NET format "F2". Anything else prints in the general format.</summary>
    public static string DotNetFormat(string printf)
    {
        if (printf.Length >= 4 && printf[0] == '%' && printf[1] == '.' && printf[^1] == 'f'
            && int.TryParse(printf.AsSpan(2, printf.Length - 3), NumberStyles.None, Inv, out int digits))
            return "F" + digits.ToString(Inv);
        return "G";
    }
}
