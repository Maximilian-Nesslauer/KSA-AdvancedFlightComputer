namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>How a booster comes home, or does not.</summary>
public enum BoosterReturnMode
{
    /// <summary>No target: it falls where its coast takes it, and keeps no reserve.</summary>
    Expendable,
    /// <summary>A landing site set by the player: a boostback puts its impact there.</summary>
    Site,
    /// <summary>Optimal return (a drone ship): no boostback, a landing where its coast comes down, and only the landing burn kept (AscentRecovery.OptimalReturn).</summary>
    OptimalReturn,
}

/// <summary>
/// A booster's way down from a plan's separation: its path over the ground and where it lands.
///
/// <see cref="Ground"/> holds x y z per point, fixed to the ground in the problem's frame at lift-off (BoostbackOracle.Path), and <see cref="Time"/> each point's time from lift-off. <see cref="Impact"/> is the landing point in the same frame: the site for a booster sent to one, and the drone ship's point for an optimal return. Not <see cref="Ok"/>, with <see cref="Note"/> saying why, when the booster has no separation in the plan or never comes down.
/// </summary>
public sealed record BoosterReturnTrack(BoosterReturnMode Mode, bool Ok, string Note, double[] Ground, double[] Time, double[] Impact,
                                        double BoostbackDv, double SeparationTime)
{
    public int Points => Time.Length;
}

/// <summary>The return paths of a plan's boosters, from the plan's own separations.</summary>
public static class BoosterReturn
{
    /// <summary>
    /// The way down of the booster <paramref name="r"/> describes, from where plan <paramref name="s"/> of problem <paramref name="p"/> separates it: the last node of its stage.
    ///   - A booster sent to a site flies the boostback that lands it there (BoostbackOracle.Boostback), after the coast to the burn.
    ///   - An optimal return and an expendable booster fly none: the path is the coast, and its impact is where they come down.
    /// The site of <paramref name="r"/> is only used for <see cref="BoosterReturnMode.Site"/>; for the others it gives the surface radius the coast ends at.
    /// </summary>
    public static BoosterReturnTrack Compute(AscentProblem p, AscentRecovery r, BoosterReturnMode mode, AscentSolution s)
        => Compute(p.Mu, p.BodyRadius, p.Omega, r, mode, s);

    /// <summary>The same, from the body alone: gravitational parameter, m^3/s^2, mean radius, m, and spin rate, rad/s.</summary>
    public static BoosterReturnTrack Compute(double mu, double bodyRadius, double omega, AscentRecovery r, BoosterReturnMode mode, AscentSolution s)
    {
        int k = s.Nodes > 0 ? Array.LastIndexOf(s.NodeStage, r.Stage) : -1;
        // The last stage reaches the insertion: nothing separates at its end.
        if (k < 0 || r.Stage >= s.BurnTime.Length - 1)
            return Failed(mode, "no separation in the plan");

        ReadOnlySpan<double> pos = s.Position.AsSpan(k * 3, 3), vel = s.Velocity.AsSpan(k * 3, 3);
        double tSep = s.Time[k];
        var oracle = new BoostbackOracle(mu, bodyRadius, omega, r);
        double[]? impulse = null;
        double dvMag = 0.0;
        if (mode == BoosterReturnMode.Site)
        {
            impulse = new double[3];
            if (!oracle.Boostback(pos, vel, tSep, impulse, out dvMag, out _))
                return Failed(mode, "no boostback lands it on its site: " + oracle.LastFailure);
        }

        var ground = new List<double>();
        var times = new List<double>();
        if (!oracle.Path(pos, vel, tSep, impulse, ground, times, out double[] impact))
            return Failed(mode, oracle.LastFailure);
        return new BoosterReturnTrack(mode, true, "", ground.ToArray(), times.ToArray(), impact, dvMag, tSep);
    }

    private static BoosterReturnTrack Failed(BoosterReturnMode mode, string note)
        => new(mode, false, note, [], [], [], double.NaN, double.NaN);
}
