namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The max-q a planner retries at when every seed climbed through the max-q it was given, so the problem likely has no feasible trajectory. The retry never goes above a caller-given ceiling under the dynamic pressure at which a vehicle breaks up.
/// </summary>
public static class AscentMaxQ
{
    public const double RelaxFactor = 1.15;

    public const double Step = 5000.0;

    public static bool OutOfReach(AscentSolution s, double qMax)
        => double.IsFinite(s.SeedLeastMaxQ) && s.SeedLeastMaxQ > 0.98 * qMax;

    /// <summary>The ceiling rounded down to a <see cref="Step"/>, Pa.</summary>
    public static double Highest(double ceiling) => Math.Floor(ceiling / Step) * Step;

    /// <summary>The max-q to retry at, Pa. A retry helps only when this is above the max-q that was tried.</summary>
    public static double Relaxed(double seedLeastMaxQ, double ceiling)
        => Math.Min(Math.Ceiling(RelaxFactor * seedLeastMaxQ / Step) * Step, Highest(ceiling));
}
