using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// How fast a propellant mass is falling, kg/s, measured from its own samples: the same mass a staging cue compares, so throttle, the g-limit and a booster burning beside its core are all in the rate. A rise in the mass, a new <c>key</c> (another cue or stage model) or a gap in the samples starts the measurement again.
/// </summary>
internal sealed class DrainRate
{
    /// <summary>The shortest span one rate is taken over, s of sim time. Shorter spans are waited out, so a step-rate caller gets the same rate as a slow one.</summary>
    public const double MinSpanS = 0.5;

    /// <summary>A gap longer than this between samples, s, starts again: the drain before it says nothing about the drain now.</summary>
    public const double GapS = 5.0;

    /// <summary>The weight of each new rate in the running one.</summary>
    public const double Smoothing = 0.5;

    private double _mass = double.NaN;
    private double _time = double.NaN;
    private int _key;

    /// <summary>The rate, kg/s, NaN until two samples MinSpanS apart.</summary>
    public double KgPerS { get; private set; } = double.NaN;

    public void Reset()
    {
        _mass = double.NaN;
        _time = double.NaN;
        KgPerS = double.NaN;
    }

    public double Update(double time, double mass, int key)
    {
        if (!double.IsFinite(time) || !double.IsFinite(mass))
        {
            Reset();
            return double.NaN;
        }
        if (key != _key || !double.IsFinite(_mass) || mass > _mass || time < _time || time - _time > GapS)
        {
            Reset();
            _key = key;
            _mass = mass;
            _time = time;
            return double.NaN;
        }
        double span = time - _time;
        if (span < MinSpanS)
            return KgPerS;
        double rate = (_mass - mass) / span;
        KgPerS = double.IsFinite(KgPerS) ? KgPerS + Smoothing * (rate - KgPerS) : rate;
        _mass = mass;
        _time = time;
        return KgPerS;
    }
}

/// <summary>
/// The countdown to the next separation: the soonest of the cues that will fire a staging row that drops parts, counted to the moment the parts come off (the row's decoupler delay included), not to the next engine's ignition.
/// </summary>
internal static class StagingCountdown
{
    /// <summary>A drain slower than this, kg/s, is an engine off or a craft coasting: no separation is coming from it.</summary>
    public const double MinDrainKgPerS = 0.1;

    /// <summary>Seconds until <paramref name="marginKg"/> of propellant is gone at <paramref name="drainKgPerS"/>, plus the row's <paramref name="delayS"/>. A margin already used up counts as now. NaN when it cannot be known.</summary>
    public static double Seconds(double marginKg, double drainKgPerS, double delayS = 0.0)
    {
        if (!double.IsFinite(marginKg) || !double.IsFinite(drainKgPerS) || drainKgPerS < MinDrainKgPerS)
            return double.NaN;
        return Math.Max(0.0, marginKg) / drainKgPerS + (double.IsFinite(delayS) ? Math.Max(0.0, delayS) : 0.0);
    }

    /// <summary>The sooner of two countdowns, either NaN when unknown.</summary>
    public static double Soonest(double a, double b)
        => double.IsNaN(a) ? b : double.IsNaN(b) ? a : Math.Min(a, b);

    /// <summary>A countdown taken at <paramref name="takenAt"/>, read at <paramref name="now"/>: it runs down with sim time between updates and stops at zero.</summary>
    public static double Now(double seconds, double takenAt, double now)
        => double.IsFinite(seconds) && double.IsFinite(takenAt) && double.IsFinite(now) ? Math.Max(0.0, seconds - Math.Max(0.0, now - takenAt)) : double.NaN;

    /// <summary>The head value: SEPARATION T- with room, SEP T- without. It ranks after the periapsis, so a narrow head drops it before the orbit.</summary>
    public static HeadValue Head(double seconds)
        => new("SEPARATION T-" + Fmt.Time(seconds), 1, "SEP T-" + Fmt.Time(seconds));
}
