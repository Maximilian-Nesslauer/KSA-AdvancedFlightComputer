namespace Scvx;

/// <summary>
/// Vehicle limits and SCvx weights for the 6-DOF subproblem.
///
/// Defaults mirror 6dof.py so the C# result can be diffed against the Python
/// oracle. They are NOT flight values — for flight these must come from the
/// vehicle and the target body (thrust and Isp from the staging model, inertia
/// and mass live, gravity from the body being landed on, gimbal limit from the
/// actual parts). See the constants-drift guard in python_ref/loop_ref.py: the
/// scenario travels through loop_ref.csv, but everything here is hand-mirrored
/// and will silently desynchronise if only one side is edited.
/// </summary>
public sealed class Scvx6DofConfig
{
    public int Nodes { get; init; } = 30;

    public double Tmax { get; init; } = 6.0e6;
    public double ThrottleFloor { get; init; } = 0.40;          // Tmin = floor * Tmax
    public double GimbalMaxDeg { get; init; } = 10.0;
    public double TauRollMax { get; init; } = 1.0e5;
    public double TiltMaxDeg { get; init; } = 30.0;
    public double GroundFloor { get; init; } = -1.0;            // X[n,2] >= this

    public double RhoVc { get; init; } = 1e5;                   // virtual-control penalty
    public double WDu { get; init; } = 0.2;                     // control-rate smoothing
    public double WW { get; init; } = 1.0;                      // angular-rate damping
    /// <summary>
    /// PROXIMAL weight: penalise deviation from the REFERENCE trajectory,
    /// rho * ||(X - Xbar)/Xscale||^2, rather than deviation from zero.
    ///
    /// Exists purely for CONDITIONING. The WDu and WW regularisers were doing two
    /// jobs at once: shaping the answer (badly — both get cheaper as sigma grows, so
    /// they pin burn time at its upper bound) and adding positive-definite mass to
    /// P's diagonal. Turning them down fixes the trajectory and TRIPLES the ADMM
    /// iteration count, because P loses that mass and SCS's convergence rate degrades.
    ///
    /// A proximal term restores exactly the same conditioning WITHOUT the bias: it is
    /// centred on the current reference, so it does not prefer slow rotation or long
    /// burns, and it vanishes at convergence where X = Xbar. This is the standard
    /// proximal-SCvx formulation rather than an invention.
    ///
    /// DEFAULT 0 so the reference validation and the constants drift guard compare
    /// against exactly the Python problem; only flight turns it on.
    /// </summary>
    public double ProximalWeight { get; init; }

    /// <summary>
    /// Burn-time bounds. SETTABLE, not init-only, so a fixed-time formulation can pin
    /// sigma per cycle (SigmaMin == SigmaMax) without rebuilding the solver — which
    /// would throw away the ADMM warm start and turn a 25 ms update into 1.3 s.
    /// The subproblem writes these into its cone rows on every Assemble, so a change
    /// takes effect on the next solve.
    /// </summary>
    public double SigmaMin { get; set; } = 5.0;
    public double SigmaMax { get; set; } = 25.0;
    public double SigmaScale { get; init; } = 12.0;

    public double[] XScale { get; init; } =
        [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];
    public double[]? UScale { get; init; }                      // defaults from Tmax/gimbal

    /// <summary>
    /// Glideslope angle above the HORIZONTAL, in degrees, measured at the target.
    /// Zero (the default) disables the constraint entirely and costs nothing — no
    /// variables, no rows.
    ///
    /// Constrains the path to a cone opening upward from the target:
    /// ||r_xy - target_xy|| &lt;= cot(angle) * (r_z - target_z). Equivalently, every
    /// node must sit at least this many degrees above the horizontal plane through
    /// the target, so a LARGER angle is a TIGHTER cone and a steeper approach.
    /// Same convention and same formula as the G-FOLD path already in the mod
    /// (PoweredGuidanceOverlay draws the cone with cot = 1/tan of this angle), so
    /// the two agree and the overlay draws what the solver enforces.
    ///
    /// The plan respects exactly this number, so set it a couple of degrees tighter
    /// than the terrain actually requires. An optimum that RIDES a constraint
    /// boundary is the dangerous case: any disturbance then puts the vehicle
    /// outside, and with no margin the next re-solve starts from a violated state.
    /// The slack below keeps that recoverable, but margin keeps it from happening.
    /// </summary>
    public double GlideSlopeDeg { get; init; }

    /// <summary>
    /// Maximum ALLOWED climb rate, m/s, applied from node 1 onward. Negative
    /// disables the constraint entirely.
    ///
    /// Deliberately a small POSITIVE number rather than a hard zero. The intent is
    /// "do not balloon", and a hard v_z &lt;= 0 turns every transient - a gust, a
    /// tracking wobble, the pitch-over right after ignition - into a constraint
    /// the vehicle is already violating. The tolerance costs nothing visually and
    /// removes a cliff.
    /// </summary>
    public double VzMax { get; init; } = -1.0;

    /// <summary>
    /// Penalties on the glideslope and climb-rate slacks, per metre and per m/s of
    /// violation.
    ///
    /// BOTH CONSTRAINTS ARE SOFT, and that is the whole point. Node 0 is pinned by
    /// an equality to the measured state, so a HARD constraint that the vehicle is
    /// already violating makes the problem infeasible by construction - not
    /// "expensive", not "suboptimal", but unsolvable, with no plan at all to fly.
    /// This exact failure already bit this codebase once, when the trust-region box
    /// applied at node 0 and any vehicle more than 11.5 degrees off vertical could
    /// not be planned for. Both constraints therefore start at node 1, AND carry a
    /// penalised slack so that even a genuinely unreachable corridor degrades into
    /// an expensive plan rather than no plan.
    ///
    /// The danger is the two together, not either alone: outside the cone and too
    /// low, the only way back inside is to climb - which the climb-rate constraint
    /// forbids. Hard versions of both can trap the vehicle in a region with no
    /// feasible exit. Soft versions cannot.
    ///
    /// These are L1 (linear) penalties, so they are EXACT: above a finite threshold
    /// the solution is identical to the hard-constrained one, rather than trading a
    /// little violation for a little objective the way a quadratic penalty would.
    /// </summary>
    public double GlideSlopeWeight { get; init; } = 1e4;
    public double VzWeight { get; init; } = 1e4;

    /// <summary>
    /// Penalty per metre on missing the terminal POSITION, or 0 to keep it a hard
    /// equality.
    ///
    /// A hard terminal constraint says "arrive exactly there, exactly at rest". When
    /// that is not achievable - which reachability makes routine, not exotic - the
    /// problem is INFEASIBLE and the solver returns nothing. But a booster on the way
    /// down cannot decline to land, so "no plan" is not a safe answer; it just means
    /// flying an older plan that is getting worse.
    ///
    /// Softening the position lets the optimiser answer "land 40 m off" instead of
    /// refusing. The order of things worth giving up on a descent is fuel optimality,
    /// then landing precision, then landing softly - so precision is the right thing
    /// to trade, and terminal VELOCITY stays hard because arriving at rest is the part
    /// that matters.
    ///
    /// L1 again, so it is exact: above a finite weight the slack sits at zero whenever
    /// the target is actually reachable, and the answer is identical to the hard
    /// problem. It only opens when the alternative is no answer at all.
    /// </summary>
    public double TerminalMissWeight { get; init; }

    /// <summary>
    /// Penalty per m/s on missing the terminal VELOCITY, or 0 to keep it hard.
    ///
    /// Softening position alone does not help the case it was meant for. When a
    /// vehicle cannot stop in time, what is unreachable is arriving AT REST, not
    /// arriving THERE - so the position slack sits unused and the solver still fakes
    /// the terminal condition with virtual control, which is what the defect gate then
    /// refuses. Measured: hard 5.27 m of defect, position-soft 5.22 m. No help.
    ///
    /// Weighted far more heavily than the position miss, because the ordering of what
    /// to concede on a descent is fuel, then precision, then softness. This is the
    /// last thing to give up, and it should only open when the alternative is a plan
    /// that claims a touchdown speed the vehicle cannot achieve.
    /// </summary>
    public double TerminalSpeedWeight { get; init; }

    public bool SoftTerminal => TerminalMissWeight > 0.0 || TerminalSpeedWeight > 0.0;

    public bool GlideSlopeEnabled => GlideSlopeDeg > 0.0;
    public bool VzLimitEnabled => VzMax >= 0.0;

    /// <summary>cot of the glideslope half-angle: the cone's horizontal run per unit of height.</summary>
    public double CotGlideSlope => 1.0 / Math.Tan(Math.Clamp(GlideSlopeDeg, 1e-3, 89.999) * Math.PI / 180.0);

    public double Tmin => ThrottleFloor * Tmax;
    public double TanGimbal => Math.Tan(GimbalMaxDeg * Math.PI / 180.0);
    public double CosTilt => Math.Cos(TiltMaxDeg * Math.PI / 180.0);

    public double[] ResolvedUScale => UScale ??
        [Tmax * TanGimbal, Tmax * TanGimbal, Tmax, TauRollMax];
}
