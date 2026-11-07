using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class TuneRules
{
    public static readonly TuneSet[] Order = { TuneSet.Ascent, TuneSet.Recover, TuneSet.Landing, TuneSet.Diag, TuneSet.Afc };

    public const int DividerAt = 3;

    private const double NearLimitShare = 0.95;

    public static bool NearLimit(double value, double limit) => limit > 0.0 && value >= limit * NearLimitShare;

    public static bool HoldsValues(TuneSet set) => set is TuneSet.Ascent or TuneSet.Recover or TuneSet.Landing;

    public static string CountText(int changed) => changed > 0 ? changed.ToString(Fmt.Inv) : "";

    public static bool ResetSetShows(int changed, int rowResets) => changed >= 2 || (changed == 1 && rowResets == 0);

    public static TuneSet Shown(TuneSet saved, bool setsWork, bool pickedHere)
        => setsWork || pickedHere ? saved : TuneSet.Afc;

    private const double PlotNoiseShare = 1e-4;
    private const double PlotZero = 1e-9;

    public static string PlotValue(double value, double scale)
    {
        if (!double.IsFinite(value))
            return "-";
        double abs = Math.Abs(value);
        if (abs < PlotZero || abs < Math.Abs(scale) * PlotNoiseShare)
            return "0";
        string scientific = abs.ToString("E2", Fmt.Inv);
        int exponent = int.Parse(scientific.AsSpan(scientific.IndexOf('E') + 1), System.Globalization.NumberStyles.AllowLeadingSign, Fmt.Inv);
        return Fmt.Fixed(value, Math.Clamp(2 - exponent, 0, 6));
    }
}
