using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>A stage of the stage list as the TWR row reads it: its full thrust at the pad's air pressure, N, its mass when it starts, kg, and whether it still burns an engine that burns at lift-off.</summary>
internal readonly record struct TwrStage(double ThrustN, double MassKg, bool BurnsLiftoffEngine);

/// <summary>
/// The thrust-to-weight ratio of the first stage before a launch. The first stage is the lift-off stage and every stage after it that still burns one of its engines, such as a core that burns on after its boosters drop; it ends at the first stage that burns none. <see cref="LowestStage"/> indexes the stage list, -1 when the first stage has one stage only.
/// </summary>
internal readonly record struct LaunchTwrFacts(double LiftoffTwr, double LowestTwr, int LowestStage)
{
    public bool Known => double.IsFinite(LiftoffTwr);

    public bool CannotLiftOff => Known && LiftoffTwr < 1.0;

    /// <summary>The row shows: the craft cannot lift off, or the first stage falls under its weight from a later stage on.</summary>
    public bool Shows => CannotLiftOff || (Known && LowestStage > 0 && LowestTwr < 1.0);
}

internal static class LaunchTwr
{
    public static readonly LaunchTwrFacts Unknown = new(double.NaN, double.NaN, -1);

    /// <param name="gravity">The local acceleration of gravity at the pad, m/s^2.</param>
    public static LaunchTwrFacts Of(IReadOnlyList<TwrStage> stages, double gravity)
    {
        if (stages.Count == 0 || !(gravity > 0.0))
            return Unknown;
        double liftoff = Ratio(stages[0], gravity);
        if (!double.IsFinite(liftoff))
            return Unknown;
        double lowest = double.NaN;
        int lowestStage = -1;
        for (int i = 1; i < stages.Count && stages[i].BurnsLiftoffEngine; i++)
        {
            double twr = Ratio(stages[i], gravity);
            if (double.IsFinite(twr) && !(twr >= lowest))
            {
                lowest = twr;
                lowestStage = i;
            }
        }
        return new LaunchTwrFacts(liftoff, lowest, lowestStage);
    }

    private static double Ratio(in TwrStage s, double gravity)
        => s.MassKg > 0.0 && s.ThrustN >= 0.0 ? s.ThrustN / (s.MassKg * gravity) : double.NaN;
}
