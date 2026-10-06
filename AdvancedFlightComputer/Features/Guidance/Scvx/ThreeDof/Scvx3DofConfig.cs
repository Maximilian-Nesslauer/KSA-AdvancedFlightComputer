namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Problem shape, limits and SCvx weights for the glide-and-burn subproblem.
///
/// Defaults mirror 3dof.py so the C# result can be diffed against the script. Everything the script does not have defaults to OFF, so the default config assembles exactly the script's problem; the guidance switches those on for flight.
/// </summary>
public sealed class Scvx3DofConfig
{
    private const int NX = PointMass3Dof.NX;

    public int Nodes { get; init; } = 30;

    /// <summary>
    /// Intervals in the glide phase, at the front of the grid. The glide is intervals 0..K-1 (nodes 0..K) with the engine off, the burn intervals K..N-2 (nodes K..N-1); node K is the ignition instant and belongs to both.
    ///
    /// ZERO MEANS NO GLIDE: the same class then assembles the burn-only problem the guidance switches to once ignition is committed, with no glide duration variable at all.
    /// </summary>
    public int GlideIntervals { get; init; } = 12;

    public bool HasGlide => GlideIntervals > 0;
    public int BurnIntervals => Nodes - 1 - GlideIntervals;

    public double ThrottleFloor { get; init; } = 0.40;

    /// <summary>
    /// Highest throttle the PLAN may use. Settable, so the guidance can hold back headroom before ignition and release it after without rebuilding the solver: a min-fuel landing burn rides full throttle, and a committed ignition time with no headroom left cannot absorb arriving a little late.
    /// </summary>
    public double ThrottleCeiling { get; set; } = 1.0;

    /// <summary>Angle-of-attack limit, degrees, at nodes faster than <see cref="AlphaSpeedThreshold"/>.</summary>
    public double AlphaMaxDeg { get; init; } = 10.0;

    /// <summary>Below this speed the airflow direction means nothing, so neither the angle-of-attack limit nor the retrograde row applies. 3dof.py's V_THRESH.</summary>
    public double AlphaSpeedThreshold { get; init; } = 5.0;

    /// <summary>
    /// q-alpha limit, Pa per radian of sin(alpha), or 0 for none. Where set, the node's angle-of-attack radius is min(sin(AlphaMax), QAlphaMax / q_bar): a large angle at the bottom of a burn, where the air is slow, loads nothing, while the same angle at ignition can. Off by default, as in the script.
    /// </summary>
    public double QAlphaMax { get; init; }

    /// <summary>Keep the body axis against the airflow, b . v_hat &lt;= 0: the booster flies engine first.</summary>
    public bool Retrograde { get; init; } = true;

    /// <summary>
    /// Per-channel bounds on the state from node 1 onward; infinite entries are off. Node 0 is pinned to the measured state by an equality, so a bound it already breaks would make the problem infeasible rather than expensive - the reason the 6-DOF's path constraints start at node 1 too.
    ///
    /// The ground floor is a minimum on r_z and the dry mass a minimum on m. 3dof.py's "do not overshoot the pad" pair is v_x &lt;= 2 and r_x &gt;= -10.
    /// </summary>
    public double[] StateMin { get; init; } = Filled(double.NegativeInfinity);
    public double[] StateMax { get; init; } = Filled(double.PositiveInfinity);

    /// <summary>
    /// Body axis at the final node, a unit vector, or null for free. Upright for a landing: the vehicle arrives standing on its engine.
    /// </summary>
    public double[]? TerminalAttitude { get; init; }

    /// <summary>
    /// Attitude rate limit, rad/s, or 0 for none: |b_k+1 - b_k| &lt;= rate * (interval duration). The plan's attitude is flown by the flight computer, which turns at a finite rate; a point-mass model otherwise assumes it turns instantly.
    ///
    /// The duration is sigma * dtau, and sigma is a VARIABLE, so the bound is an exact second-order cone with sigma in its head - no linearisation.
    /// </summary>
    public double AttitudeRateMax { get; init; }

    /// <summary>
    /// Allocate the row that holds node 0's body axis within a cone about a given direction, set per solve through <see cref="Scvx3DofSolver.SetAttitudeAnchor"/>. Node 0's attitude is a control, not a measured state, so without this a plan may open with an instant rotation.
    /// </summary>
    public bool AttitudeAnchor { get; init; }

    /// <summary>
    /// Penalty per metre on missing the terminal position, and per m/s on the terminal velocity, or 0 to keep each hard. L1, so exact: above a finite weight the slack is zero whenever the target is reachable, and it only opens when the alternative is no plan at all. See Scvx6DofConfig.TerminalMissWeight.
    /// </summary>
    public double TerminalMissWeight { get; init; }
    public double TerminalSpeedWeight { get; init; }

    public double RhoVc { get; init; } = 1e5;
    public double WThrottleRate { get; init; } = 0.05;
    public double WAttitudeRate { get; init; } = 0.10;

    /// <summary>Proximal weight on (X - Xbar)/Xscale, for conditioning without biasing sigma. See Scvx6DofConfig.ProximalWeight. Off by default, as in the script.</summary>
    public double ProximalWeight { get; init; }

    /// <summary>
    /// Phase-duration bounds, seconds. SETTABLE, so the guidance can pin a duration per cycle (min == max) without rebuilding the solver: that is how the glide's duration is committed once ignition is close, and how the burn's is counted down after it.
    /// </summary>
    public double GlideSigmaMin { get; set; } = 0.0;
    public double GlideSigmaMax { get; set; } = 40.0;
    public double BurnSigmaMin { get; set; } = 5.0;
    public double BurnSigmaMax { get; set; } = 50.0;
    public double SigmaScale { get; init; } = 30.0;

    /// <summary>State scales. 3dof.py's Xscale, whose mass entry is its m0.</summary>
    public double[] XScale { get; init; } = [1000, 1000, 6000, 300, 300, 300, 28000];

    public double SinAlphaMax => Math.Sin(AlphaMaxDeg * Math.PI / 180.0);
    public bool SoftTerminal => TerminalMissWeight > 0.0 || TerminalSpeedWeight > 0.0;

    private static double[] Filled(double v)
    {
        var a = new double[NX];
        Array.Fill(a, v);
        return a;
    }
}
