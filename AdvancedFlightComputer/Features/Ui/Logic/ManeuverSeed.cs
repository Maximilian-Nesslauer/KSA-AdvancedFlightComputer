using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum SeedPhase { Unknown, Ground, Orbit, Suborbital, Escaping }

/// <summary>A target field of a MANEUVER tool starts on an even value that differs from the orbit now and that the form accepts, so the page never opens in a refusal and never offers a burn of 0.0 m/s.</summary>
internal static class ManeuverSeed
{
    private const double MinStepKm = 10.0;
    private const double IncStepDeg = 5.0;

    public const double AtTargetKm = 0.1;

    public const double AtTargetDeg = 0.06;

    public static SeedPhase PhaseOf(CraftSituation where) => where switch
    {
        CraftSituation.Ground => SeedPhase.Ground,
        CraftSituation.Orbit => SeedPhase.Orbit,
        CraftSituation.CoastingUp or CraftSituation.FallingAir or CraftSituation.FallingNoAir => SeedPhase.Suborbital,
        CraftSituation.Escaping => SeedPhase.Escaping,
        _ => SeedPhase.Unknown,
    };

    public static bool ApseFits(bool setPeriapsis, double targetKm, double peKm, double apKm, double marginKm)
        => setPeriapsis ? targetKm < apKm - marginKm : targetKm > peKm + marginKm;

    /// <summary>Whether the target is the apsis the tool moves, so the burn would be about 0 m/s.</summary>
    public static bool AtApse(bool setPeriapsis, double targetKm, double peKm, double apKm)
        => Math.Abs(targetKm - (setPeriapsis ? peKm : apKm)) < AtTargetKm;

    public static bool AtInclination(double targetDeg, double currentDeg) => Math.Abs(targetDeg - currentDeg) < AtTargetDeg;

    public static double ApoapsisKm(double peKm, double apKm, double marginKm)
        => NextEvenAbove(Math.Max(apKm, peKm + marginKm));

    public static double PeriapsisKm(double peKm, double apKm, double marginKm)
    {
        double ceiling = apKm - marginKm;
        double up = peKm < MinStepKm ? Step(Math.Max(0.0, apKm)) : NextEvenAbove(peKm);
        if (up < ceiling)
            return up;
        double down = NextEvenBelow(peKm);
        return down < ceiling ? down : Math.Max(0.0, ceiling);
    }

    public static double InclinationDeg(double currentDeg)
    {
        double current = double.IsFinite(currentDeg) ? Math.Clamp(currentDeg, 0.0, 180.0) : 0.0;
        double next = (Math.Floor(current / IncStepDeg) + 1.0) * IncStepDeg;
        if (next - current < 1.0)
            next += IncStepDeg;
        return next <= 180.0 ? next : (Math.Ceiling(current / IncStepDeg) - 1.0) * IncStepDeg;
    }

    public static double NextEvenAbove(double km)
    {
        double from = double.IsFinite(km) ? Math.Max(0.0, km) : 0.0;
        double step = Step(from);
        double next = (Math.Floor(from / step) + 1.0) * step;
        return next - from < step * 0.1 ? next + step : next;
    }

    public static double NextEvenBelow(double km)
    {
        double from = double.IsFinite(km) ? Math.Max(0.0, km) : 0.0;
        double step = Step(from);
        return Math.Max(0.0, (Math.Ceiling(from / step) - 1.0) * step);
    }

    private static double Step(double km)
        => km < MinStepKm ? MinStepKm : Math.Pow(10.0, Math.Floor(Math.Log10(km)));
}
