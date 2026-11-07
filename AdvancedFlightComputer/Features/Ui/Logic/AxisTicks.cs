using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class AxisTicks
{
    private static readonly double[] Mantissas = { 1.0, 2.0, 5.0, 10.0 };

    // The steps of a time axis in seconds: round in seconds, in minutes, in hours and in days.
    private static readonly double[] TimeSteps =
    {
        1.0, 2.0, 5.0, 10.0, 15.0, 30.0, 60.0, 120.0, 300.0, 600.0, 900.0, 1800.0, 3600.0, 7200.0, 10800.0, 21600.0, 43200.0, 86400.0,
    };

    private const double SecondsPerDay = 86400.0;

    public static double NiceStep(double span, int most)
    {
        if (!(span > 0.0) || most < 1)
            return 1.0;
        double raw = span / most;
        double power = Math.Pow(10.0, Math.Floor(Math.Log10(raw)));
        foreach (double m in Mantissas)
        {
            if (m * power >= raw * (1.0 - 1e-9))
                return m * power;
        }
        return 10.0 * power;
    }

    public static double TimeStep(double span, int most)
    {
        if (!(span > 0.0) || most < 1)
            return 1.0;
        double raw = span / most;
        foreach (double step in TimeSteps)
        {
            if (step >= raw)
                return step;
        }
        return SecondsPerDay * NiceStep(span / SecondsPerDay, most);
    }

    public static int Fill(double lo, double hi, double step, Span<double> ticks)
    {
        if (!(step > 0.0) || !(hi >= lo) || !double.IsFinite(lo) || !double.IsFinite(hi))
            return 0;
        double first = Math.Ceiling(lo / step - 1e-9) * step;
        int n = 0;
        while (n < ticks.Length)
        {
            double v = first + n * step;
            if (v > hi + step * 1e-9)
                break;
            ticks[n++] = Math.Abs(v) < step * 1e-9 ? 0.0 : v;
        }
        return n;
    }

    public static (double Lo, double Hi) Outward(double lo, double hi, double step)
        => step > 0.0 ? (Math.Floor(lo / step + 1e-9) * step, Math.Ceiling(hi / step - 1e-9) * step) : (lo, hi);

    public static int RootTicks(double top, Span<double> ticks)
    {
        if (!(top > 0.0) || ticks.Length < 4)
            return 0;
        ticks[0] = 0.0;
        ticks[1] = OneFigure(top / 16.0);
        ticks[2] = OneFigure(top / 4.0);
        ticks[3] = OneFigure(top);
        return 4;
    }

    public static int BrokenTicks(double near, double far, Span<double> ticks)
    {
        if (!(near > 0.0))
            return 0;
        int n = Fill(0.0, near, NiceStep(near, 3), ticks);
        for (double v = Math.Pow(10.0, Math.Ceiling(Math.Log10(near) + 1e-9)); v <= far && n < ticks.Length; v *= 10.0)
        {
            if (v > near)
                ticks[n++] = v;
        }
        return n;
    }

    private static double OneFigure(double v)
    {
        if (!(v > 0.0))
            return 0.0;
        double power = Math.Pow(10.0, Math.Floor(Math.Log10(v)));
        return Math.Round(v / power) * power;
    }

    public static string Number(double v) => Math.Round(v, 6).ToString("0.###", Fmt.Inv);

    public static string Km(double km) => km == 0.0 ? "0" : Number(km) + " km";

    public static string Distance(double meters)
        => meters == 0.0 ? "0" : Math.Abs(meters) < 1000.0 ? Number(meters) + " m" : Number(meters / 1000.0) + " km";

    public static string Time(double seconds)
    {
        if (seconds == 0.0)
            return "0";
        string time = Fmt.Time(seconds);
        return time.EndsWith(" 00s", StringComparison.Ordinal) || time.EndsWith(" 00m", StringComparison.Ordinal) ? time.Substring(0, time.Length - 4) : time;
    }

    public static string Degrees(double deg) => Number(deg) + " deg";

    public static string Percent(double percent) => Number(percent) + " %";
}
