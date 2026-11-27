namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Vehicle limits and SCvx weights for the 6-DOF subproblem.
///
/// Defaults mirror 6dof.py so the C# result can be diffed against the Python
/// oracle. They are NOT flight values - for flight these must come from the
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

    /// <summary>
    /// THE PLAN'S THRUST CEILING as a share of full thrust, 1 for all of it: the plan may ask for at most this, and the attitude and thrust feedback has the rest (tube MPC's constraint tightening). The throttle is still T / Tmax.
    ///  WHY (2026-11-27 campaign). A propellant-best plan ends its burn on full thrust; the lunar lander came down its last 4.5 s at 100 % throttle from 18 m/s at 49 m, so nothing was left for the craft falling behind its plan, and the re-solves slid the braking later still.
    /// </summary>
    public double PlanThrustShare { get; set; } = 1.0;
    public double GimbalMaxDeg { get; init; } = 10.0;

    /// <summary>
    /// Where the gimbal cone is centred, as lateral thrust per newton of axial thrust (tdx / T, tdy / T): the deflection that holds the engines' own moment (Dynamics6Dof.Params.ThrustMoment*, tdx = m_y T / L, tdy = -m_x T / L). The cone, ||(tdx, tdy) - T (x, y)|| &lt;= T tan(gimbal), is the authority the gimbals have AROUND that deflection.
    ///  WHY. The cone's width is the gimbals' capability in their weakest direction. For one engine off the stack's axis that is the direction that drags roll in, which only the verniers can take out; the direction that holds its own moment needs none and has several times as much. Centred on zero, the trim of one 10 deg engine of a triple plate needed 324 % of the cone: every plan was asked for a deflection it could not have, the standby could not publish, and touchdown guidance took over at 38 m (2026-11-21). Zero for a balanced cluster, which is the cone it had.
    /// </summary>
    public double GimbalTrimX { get; init; }
    public double GimbalTrimY { get; init; }
    public double GimbalTrim => Math.Sqrt(GimbalTrimX * GimbalTrimX + GimbalTrimY * GimbalTrimY);

    /// <summary>
    /// How fast the gimbal can move, as full deflections (GimbalMaxDeg) per second; 0 for no limit. KSA 5901's gimbals slew at Gimbal's rate cap, NaturalFrequency / (5 e) of their range a second - 2 a second for the stock engines, zero to full deflection in half a second - where the plan took the lateral thrust as free to jump between nodes: at 50 nodes it could swing ten times faster than the gimbal follows. The gimbal's angle - each lateral thrust over the axial, td/T - may change between neighbouring nodes by no more than this rate times the interval times the cone's tangent, linearised about the reference (Scvx6DofSubproblemScs). WHY THE ANGLE (2026-11-27): the limit was on the lateral thrust itself, |td_{k+1} - td_k| against the cone at the mean thrust, which is the angle's rate only while the thrust holds. The last plans of the Starship landings swung the thrust between 0.9 and 2.4 MN in a few tenths of a second, so with the lateral thrust inside its limit the angle moved at 30-120 deg/s against the gimbal's 20.
    /// </summary>
    public double GimbalSlewPerS { get; init; }
    public double TauRollMax { get; init; } = 1.0e5;

    /// <summary>
    /// Roll torque scales with the axial thrust, |tau_roll| &lt;= TauRollMax * T / Tmax, rather than TauRollMax at any thrust: the roll torque comes from gimbals, which turn thrust into torque, so at a third of full thrust there is a third of the roll authority. A fixed bound promised a landing booster several times the roll it had (2026-10-08), the plan rolled at rates the vehicle could not follow, and every re-solve from there was refused. Off for a vehicle with a roll actuator of its own.
    /// </summary>
    public bool RollTorqueFollowsThrust { get; init; }

    /// <summary>
    /// Leaves the heading and the roll rate free at the end of the plan, for a craft that has no roll torque: the last node is held upright (qx = qy = 0) with no pitch or yaw rate, and its qw, qz and wz are what the roll motion makes them. Such a craft keeps the roll rate it enters with. An end state with a fixed heading and no roll rate is then out of reach of every control except the coupling of the pitch and yaw rates through unequal transverse inertias, which is weak and makes the subproblems badly conditioned: a plan that uses it swings the thrust axis through the whole burn, and a re-solve costs ten or more times the solver iterations.
    /// The reference has to roll with the craft as well, see ColdSeed6Dof.KeepRoll.
    /// </summary>
    public bool TerminalRollFree { get; init; }
    public double TiltMaxDeg { get; init; } = 30.0;
    public double GroundFloor { get; init; } = -1.0;            // X[n,2] >= this

    public double RhoVc { get; init; } = 1e5;                   // virtual-control penalty

    /// <summary>
    /// VIRTUAL CONTROL PRICED PER SECOND, s, or 0 for per interval: an interval shorter than this has its virtual control priced up by this over its length (at most <see cref="VirtualControlShortCap"/> times), so the penalty is RhoVc * sum (w / scale)^2 * max(1, this / dt) - the integral of the squared rate, against the interval it was sized on. Longer intervals keep their price.
    ///  WHY (2026-11-27). Priced per interval, a plan that moves its state at some rate its dynamics do not make pays c^2 T^2 / N over T seconds in N intervals: as a 15-node landing plan shrank from 4.5 s to 1.2 s, the same cheating got 14 times cheaper, and the last plans of the Starship landings turned their attitude 3-40 deg/s faster or slower than their own rates rather than pay the soft limits' L1 prices. 0.35 s is the interval the per-channel tolerances were sized on (Ksa6DofGuidance.GateReferenceIntervalS).
    /// </summary>
    ///  Briefly switched off on 2026-11-27 (91be2da), blamed for a lunar lander that padded its late plans with a hover at the contact height. Wrongly: those plans were padded with 0.96 s intervals too, where this adds nothing; the sideways-speed cone held them to 0.3 m/s at the ground with a metre still to go (VehicleAutopilotState.SixDofSideSpeedPerM, now off by default).
    public double VirtualControlRefIntervalS { get; set; } = 0.35;

    /// <summary>The most an interval's virtual control is priced up for being short (VirtualControlRefIntervalS): a fiftieth of it, 7 ms, and under.</summary>
    public const double VirtualControlShortCap = 50.0;

    /// <summary>The factor on an interval's virtual-control price for its length <paramref name="dt"/>, s: 1 at VirtualControlRefIntervalS or longer, for a zero-length interval (a coast after ignition) and when off.</summary>
    public double VirtualControlIntervalWeight(double dt)
        => VirtualControlRefIntervalS > 0.0 && dt > 1e-9 ? Math.Clamp(VirtualControlRefIntervalS / dt, 1.0, VirtualControlShortCap) : 1.0;

    /// <summary>
    /// The scale each state channel's virtual control is priced on, NX long, or null for <see cref="XScale"/>: the dynamics penalty is RhoVc * sum (w / scale)^2, in the subproblem and in the merit alike. An entry of 0 or less keeps that channel on XScale.
    ///  DECOUPLED FROM XScale, which also sets the column scaling and the trust region, so a channel's price can be chosen for what a jump in it means without touching either - the ascent's DScale (launch3dof.py's Dscale). The convergence test still reads the defect on XScale.
    ///  Why it matters: priced on XScale, a velocity jump is measured against the speed at engage, and the quadratic makes a small jump repeated on every interval nearly free - cheaper, below some size per interval, than the L1 terminal slack it can stand in for. On the 2026-10-09 7 t booster a plan carried 0.18 m/s of phantom braking on every interval, about 9 m/s over the burn, at a twentieth of the price of arriving that much faster.
    /// </summary>
    ///  SETTABLE, through Scvx6DofSolver.SetVirtualControlScale and nowhere else: the guidance stages it, loose while a plan is being found and tight once one is being refined (Ksa6DofGuidance's pricing stage).
    public double[]? VirtualControlScale { get; set; }

    /// <summary>The scale channel <paramref name="i"/>'s virtual control is priced on: <see cref="VirtualControlScale"/> where it sets one, <see cref="XScale"/> otherwise.</summary>
    public double VirtualControlScaleOf(int i)
        => VirtualControlScale is { } s && i < s.Length && s[i] > 0.0 ? s[i] : XScale[i];

    /// <summary>
    /// THE NEAR-TERM PRICE: the first <see cref="VirtualControlNearIntervals"/> intervals priced on this scale instead (NX long, null or entries of 0 or less for none), so the intervals about to be flown, which the flight's gate holds to its tolerance in full, are priced as tight as it is, whatever <see cref="VirtualControlScale"/> is for the rest. SETTABLE as VirtualControlScale is, through Scvx6DofSolver.SetVirtualControlScale.
    /// </summary>
    public double[]? VirtualControlNearScale { get; set; }
    public int VirtualControlNearIntervals { get; set; }

    /// <summary>The scale channel <paramref name="i"/>'s virtual control is priced on over interval <paramref name="k"/>: the near-term price on the first VirtualControlNearIntervals, VirtualControlScaleOf after.</summary>
    public double VirtualControlScaleOf(int i, int k)
        => k < VirtualControlNearIntervals && VirtualControlNearScale is { } s && i < s.Length && s[i] > 0.0 ? s[i] : VirtualControlScaleOf(i);

    public double WDu { get; init; } = 0.2;                     // control-rate smoothing

    /// <summary>
    /// Price on the RCS torque, RcsWeight * sum over nodes and controls of (u_rcs / authority)^2. The propellant alone barely prices it, so without this the split between the gimbals and the RCS, and between a control's + and - halves, was all but free, and the subproblem had flat directions: on 2026-11-19 SCS took 2-8 times the iterations it had without RCS. The square makes the split unique and leans on the gimbals, which cost no propellant, before the thrusters.
    /// </summary>
    public double RcsWeight { get; init; } = 0.0;

    /// <summary>
    /// THE CONTROL SURFACES (Dynamics6Dof.IFIN): each fin's deflection range, degrees, and how fast it may move, deg/s, as the plan may use them; zeros pin a slot at zero (a craft with fewer fins, or none). The subproblem bounds each fin control between its min and max at every node and its change between nodes by the rate times the interval.
    /// </summary>
    public double[]? FinMinDeg { get; init; }
    public double[]? FinMaxDeg { get; init; }
    public double[]? FinRateDegS { get; init; }

    /// <summary>Price on the fins' deflection, FinWeight * sum over nodes and fins of (deflection / range)^2: far below the RCS's, so the fins are spent first, and enough to make the split between the fins unique and keep idle ones home.</summary>
    public double FinWeight { get; init; } = 1.0e-3;

    /// <summary>The craft's control surfaces as the plan flies them, one fin control each (Dynamics6Dof.IFIN on): as many as the fin arrays are long. Must match the dynamics' (Dynamics6Dof.Params.FinCount).</summary>
    public int FinCount => Math.Max(FinMinDeg?.Length ?? 0, Math.Max(FinMaxDeg?.Length ?? 0, FinRateDegS?.Length ?? 0));

    /// <summary>The control count: the base controls and one per fin.</summary>
    public int NU => Dynamics6Dof.NUBase + FinCount;

    /// <summary>Fin <paramref name="i"/>'s range, degrees; (0, 0) for none.</summary>
    public (double Min, double Max) FinBounds(int i)
    {
        double lo = FinMinDeg is { } a && i < a.Length && double.IsFinite(a[i]) ? Math.Min(a[i], 0.0) : 0.0;
        double hi = FinMaxDeg is { } b && i < b.Length && double.IsFinite(b[i]) ? Math.Max(b[i], 0.0) : 0.0;
        return (lo, hi);
    }

    /// <summary>Fin <paramref name="i"/>'s rate, deg/s; 0 for none (or no limit where the fin has a range: see the subproblem).</summary>
    public double FinRate(int i) => FinRateDegS is { } r && i < r.Length && double.IsFinite(r[i]) && r[i] > 0.0 ? r[i] : 0.0;
    public double WW { get; init; } = 1.0;                      // angular-rate damping

    /// <summary>
    /// A rate price on the COAST's nodes, on top of WW: CoastRateWeight * sum over coast nodes 1 .. CoastIntervals-1 of ||w||^2. 0 for none.
    ///  WHY. The coast burns nothing, so fuel - the only term that cares about the path's shape - is flat across every glide that reaches an equally good ignition, and the air's lift steers it sideways for free. Re-solves then wandered: on the 2026-11-20 14:51 Earth flight entry's plan, the seed, ran 7.6 m off the straight line to the pad; the plans went to +27 m, then -14, -23, -47 m, and the booster flew a 20 m sideways bend in the coast and back in the burn. Of glides that cost the same, the one that turns least is the straight one, so a small price on turning picks it; set against fuel (Ksa6DofSetup), small enough that a turn that saves fuel still wins.
    /// </summary>
    public double CoastRateWeight { get; init; }
    /// <summary>
    /// PROXIMAL weight: penalise deviation from the REFERENCE trajectory,
    /// rho * ||(X - Xbar)/Xscale||^2, rather than deviation from zero.
    ///
    /// Exists purely for CONDITIONING. The WDu and WW regularisers were doing two
    /// jobs at once: shaping the answer (badly - both get cheaper as sigma grows, so
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
    /// sigma per cycle (SigmaMin == SigmaMax) without rebuilding the solver - which
    /// would throw away the ADMM warm start and turn a 25 ms update into 1.3 s.
    /// The subproblem writes these into its cone rows on every Assemble, so a change
    /// takes effect on the next solve.
    /// </summary>
    public double SigmaMin { get; set; } = 5.0;
    public double SigmaMax { get; set; } = 25.0;
    public double SigmaScale { get; init; } = 12.0;

    /// <summary>
    /// THE COAST BEFORE IGNITION, as a number of intervals, or 0 for none (the burn starts at node 0, as before).
    ///  The first CoastIntervals intervals are flown with the engine off (Dynamics6Dof.Params.EngineOff): nodes 0 .. CoastIntervals-1 have no thrust, and node CoastIntervals is the IGNITION, where the thrust steps to the burn's. The coast has its own free duration, the second time variable beside the burn's sigma, so the plan chooses when to light as well as how to burn: entry guidance's glide and burn split, on the full attitude. Attitude in the coast is the RCS's and the air's; there is no gimbal torque without thrust.
    ///  One node grid for both phases, so the problem keeps its shape as ignition nears and passes: a warm start carries from a re-solve to the next, through ignition (<see cref="CoastSigmaFixed"/>).
    /// </summary>
    public int CoastIntervals { get; init; }

    public bool HasCoast => CoastIntervals > 0;

    /// <summary>The coast's duration bounds, s. SETTABLE like sigma's.</summary>
    public double CoastSigmaMin { get; set; }
    public double CoastSigmaMax { get; set; } = 30.0;

    /// <summary>
    /// NO LATER IGNITION, s, or negative for none: once touchdown has the craft (Ksa6DofGuidance.ArmIgnitionCap), the ignition no later than the earliest a published plan - or the plan in hand at the hand-over - has promised, plus this (Ksa6DofGuidance.CoastNow): a re-solve can light the engine earlier but not later. A shrinking horizon on the ignition. 0.2 s by default (Ksa6DofSetup).
    ///  WHY (2026-11-27 campaign). The coast stays free until the ignition locks, and each re-solve of a Starship's approach pushed the ignition a little later - the 17:29 landing planned to light 4.9 s after the hand-over and lit 7.7 s after, the plan's touchdown sliding 7 s later over the coast - so the burn began lower and faster and ended on its limits. The literature's receding-horizon fix: keep the time-to-go the plan promised.
    /// </summary>
    public double CoastGrowthCapS { get; set; } = -1.0;

    /// <summary>
    /// NO LATER TOUCHDOWN, s, or negative for none: once touchdown has the craft (Ksa6DofGuidance.ArmIgnitionCap), a re-solve's burn ends no later than the earliest end a published plan - or the plan in hand at the hand-over - has promised, plus this (Ksa6DofGuidance.KeepEndNoLater). A re-solve can land sooner but not later: the burn time's own shrinking horizon, as NO LATER IGNITION is the coast's.
    ///  WHY (2026-11-27 in-game Moon landing, 20:49). The burn time is free on every re-solve, and the lander's plans stretched their end from 4.6 s to go at 8.5 m over the legs to 9 s at 2.6 m and still 7.8 s at 1.2 m, creeping at 0-0.1 m/s rather than taking out the last 0.7 m of miss with a lean; the cut supervisor read it as the stall it was, cut at 1.2 m, and the lander fell to the ground at 1.8 m/s.
    /// </summary>
    public double EndGrowthCapS { get; set; } = -1.0;

    /// <summary>
    /// THE IGNITION LOCK: the coast's duration pinned to this, s, or null to leave it free. A fuel-optimal plan always lights as late as it can, so a free ignition time slides later on every re-solve; locked a second or so out, the craft commits to it. After ignition the coast is pinned at zero: its nodes sit at the start, the problem keeps its shape, and the warm start carries on.
    /// </summary>
    public double? CoastSigmaFixed { get; set; }

    public double[] XScale { get; init; } =
        [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];
    public double[]? UScale { get; init; }                      // defaults from Tmax/gimbal

    /// <summary>
    /// Glideslope angle above the HORIZONTAL, in degrees, measured at the target.
    /// Zero (the default) disables the constraint entirely and costs nothing - no
    /// variables, no rows.
    ///
    /// Constrains the path to a cone opening upward from the target:
    /// ||r_xy - target_xy|| &lt;= cot(angle) * (r_z - target_z). Equivalently, every
    /// node must sit at least this many degrees above the horizontal plane through
    /// the target, so a LARGER angle is a TIGHTER cone and a steeper approach.
    /// Same convention and same formula as the G-FOLD path already in the mod
    /// (The guidance overlay draws the cone with cot = 1/tan of this angle), so
    /// the two agree and the overlay draws what the solver enforces.
    ///
    /// The plan respects exactly this number, so set it a couple of degrees tighter
    /// than the terrain actually requires. An optimum that RIDES a constraint
    /// boundary is the dangerous case: any disturbance then puts the vehicle
    /// outside, and with no margin the next re-solve starts from a violated state.
    /// The slack below keeps that recoverable, but margin keeps it from happening.
    ///
    /// A vehicle that enters outside the cone gets an allowance that shrinks to
    /// zero at the target, so the plan closes on the cone instead of asking the
    /// first interval to jump inside it.
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
    /// The length the terminal miss is priced on, m: TerminalMissWeight per this many metres of miss. 0 prices it on the position scale (XScale), as it was.
    ///  PRICED ON A FIXED LENGTH, the miss costs the same per metre wherever the plan starts. On the position scale - the craft's distance from the target when the solver was built - it cost a third as much per metre on a plan handed over 760 m up as on one from 240 m, and offline the landings missed by 10 m against 3 m (--6dof-coast, 2026-11-20).
    /// </summary>
    public double TerminalMissScaleM { get; init; }

    /// <summary>The speed the terminal velocity miss is priced on, m/s; 0 prices it on the velocity scale (XScale). See TerminalMissScaleM.</summary>
    public double TerminalSpeedScaleMs { get; init; }

    /// <summary>The length a metre of terminal miss is priced on.</summary>
    public double TerminalMissPriceScale => TerminalMissScaleM > 0.0 ? TerminalMissScaleM : XScale[Dynamics6Dof.IR];

    /// <summary>The speed an m/s of terminal velocity miss is priced on.</summary>
    public double TerminalSpeedPriceScale => TerminalSpeedScaleMs > 0.0 ? TerminalSpeedScaleMs : XScale[Dynamics6Dof.IV];

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
    public double TerminalSpeedWeight { get; set; }

    /// <summary>The terminal miss's price on the HEIGHT axis, as a multiple of TerminalMissWeight's (1: the same as across). Raised with the speed's price, so a squeezed plan still arrives a little fast rather than stopping high, where a craft that cannot throttle below its weight floats and climbs (2026-10-08).</summary>
    public double TerminalHeightMissFactor { get; set; } = 1.0;

    /// <summary>
    /// THE END ATTITUDE, SOFT: the price per degree of tilt left at the end (the end quaternion's x and y components, TerminalAttitudePriceQuat each), or 0 to keep it hard. With <see cref="TerminalRateWeight"/>, per deg/s of pitch and yaw rate left at the end.
    ///  WHY (2026-11-27). Upright with no pitch or yaw rate at the end was hard. In the last second or two of the Starship landings the plans already swung the gimbal at its planned slew on 12-14 of 14 intervals and could not stop an 8 deg/s turn in time; the only slack left was virtual control, so the plans' attitudes ran 5-12 deg off their own rates, and the craft, following physics, touched down at 5-10 deg still turning. Soft, a plan that cannot end upright says so, and the flight is not asked to fly attitudes nothing produces. Priced above the position miss (the user's call: there is margin in where it lands, none in how it lands) and with the speed.
    /// </summary>
    public double TerminalAttitudeWeight { get; set; }

    /// <summary>The end's pitch and yaw rate, soft: the price per deg/s left at the end, or 0 to keep it hard. See TerminalAttitudeWeight.</summary>
    public double TerminalRateWeight { get; set; }

    /// <summary>A degree of tilt as an end quaternion component, sin(0.5 deg): what TerminalAttitudeWeight is priced on.</summary>
    public static readonly double TerminalAttitudePriceQuat = Math.Sin(0.5 * Math.PI / 180.0);

    /// <summary>A degree a second in rad/s: what TerminalRateWeight is priced on.</summary>
    public const double TerminalRatePriceRadS = Math.PI / 180.0;

    public bool SoftTerminal => TerminalMissWeight > 0.0 || TerminalSpeedWeight > 0.0 || TerminalAttitudeWeight > 0.0 || TerminalRateWeight > 0.0;

    /// <summary>The end speed's price ACROSS (x, y), as a multiple of TerminalSpeedWeight's; the height axis keeps the whole. Low, so a plan does not chase the last of the sideways speed with a lean and a turn it cannot finish: the tip-over line (TipLimit) says how much sideways speed the touchdown can carry.</summary>
    public double TerminalSideSpeedFactor { get; set; } = 1.0;

    /// <summary>
    /// THE TIP-OVER LINE at the end (2026-11-27, the user's): the touchdown may carry at most TipShare (80 %) of the energy that tips the craft over its skirt's or feet's edge. With R the edge's distance from the axis, H the centre of mass's height over the contact plane, I the pitch-yaw inertia, m the end mass, d = sqrt(R^2 + H^2), alpha = atan(R / H) and K = sqrt(TipShare (I + m d^2) m g d), that is, for tilts short of alpha,
    /// <c>(m H |v_xy| + I |w_xy|) / K + 2 |q_xy| &lt;= alpha</c>
    /// at the last node (the barrier is m g d (alpha - theta)^2 / 2 to within a percent there, so the line is straight). Three cones and one row, soft with its own slack in radians of the line, priced TipWeight a degree. On from the build (the pattern is fixed then); the numbers - TipRadiusM, TipHeightM, TipInertiaKgM2, TipGravity - are the flight's, set each re-solve, and the line is loose until they are.
    /// </summary>
    public bool TipLimit { get; set; }
    public double TipRadiusM { get; set; }
    public double TipHeightM { get; set; }
    public double TipInertiaKgM2 { get; set; }
    public double TipGravity { get; set; } = 9.81;
    public double TipShare { get; set; } = 0.8;
    public double TipWeight { get; set; } = 2.0e4;

    /// <summary>The line's gain K, N m s a radian, and alpha, rad, at end mass <paramref name="massKg"/>; false while the numbers are missing (the line is then loose).</summary>
    public bool TipLine(double massKg, out double k, out double alpha)
    {
        double r = TipRadiusM, h = TipHeightM, d = Math.Sqrt(r * r + h * h);
        alpha = Math.Atan2(Math.Max(r, 0.0), Math.Max(h, 1e-6));
        k = Math.Sqrt(Math.Max(TipShare, 0.0) * (TipInertiaKgM2 + massKg * d * d) * massKg * TipGravity * d);
        return TipLimit && r > 0.0 && h > 0.0 && TipInertiaKgM2 > 0.0 && massKg > 0.0 && TipGravity > 0.0 && double.IsFinite(k) && k > 0.0;
    }

    /// <summary>
    /// THE LAST SECONDS PRICED FOR SMOOTHNESS (2026-11-27, the user's idea): from TerminalPhaseS to go, each node's tilt and pitch-yaw rate cost TerminalPhaseTiltWeight a degree squared second and TerminalPhaseRateWeight a (deg/s) squared second, ramped in as the square of how far into those seconds it is. Far out the plan flies for propellant; near the end, leaning and turning are dear, so it kills the sideways speed early or lands with a little drift rather than leaning late and swinging back through upright. 0 for none.
    /// </summary>
    public double TerminalPhaseS { get; set; }
    public double TerminalPhaseTiltWeight { get; set; }
    public double TerminalPhaseRateWeight { get; set; }

    /// <summary>Node <paramref name="k"/>'s share of the last-seconds price, s: the ramp at its time to go, times the plan time it stands for. 0 in the coast and outside TerminalPhaseS.</summary>
    public double TerminalPhaseShare(int k, int n, int coastIntervals, double sigma, double coastSigma)
    {
        if (!(TerminalPhaseS > 0.0) || k < coastIntervals || !(sigma > 0.0))
            return 0.0;
        double end = Scvx6DofSolver.NodeTime(n - 1, n, coastIntervals, sigma, coastSigma);
        double toGo = end - Scvx6DofSolver.NodeTime(k, n, coastIntervals, sigma, coastSigma);
        double ramp = Math.Clamp(1.0 - toGo / TerminalPhaseS, 0.0, 1.0);
        double dt = sigma / Math.Max(n - 1 - coastIntervals, 1);
        double span = k == coastIntervals || k == n - 1 ? 0.5 * dt : dt;
        return ramp * ramp * span;
    }

    /// <summary>A degree of tilt as the quaternion's x or y component and a degree a second in rad/s: what the last-seconds prices are per.</summary>
    public static readonly double PhaseTiltScaleQuat = Math.Sin(0.5 * Math.PI / 180.0);
    public const double PhaseRateScaleRadS = Math.PI / 180.0;

    public bool GlideSlopeEnabled => GlideSlopeDeg > 0.0;
    public bool VzLimitEnabled => VzMax >= 0.0;

    /// <summary>
    /// Largest THRUST acceleration the plan may use, m/s^2, or 0 for none: T &lt;= AccelMax * m at every node. Exact - both are variables and the row is linear in them - and soft like the glideslope, with an L1 slack at <see cref="AccelWeight"/>, so a craft that needs more to stop gets it rather than no plan. Keeps a high thrust-to-weight booster from planning its landing as a few seconds at full thrust, which is where lag costs the most.
    /// </summary>
    public double AccelMax { get; init; }

    /// <summary>
    /// The acceleration slack's price per newton over the limit, normalised by Tmax, per node. LOW ENOUGH THAT BRAKING HARDER BEATS ARRIVING FAST: a m/s bought over the limit costs about (AccelWeight / 1e4) x (velocity scale / full-thrust acceleration) / node spacing times a m/s of touchdown speed, and at 1e4 that was 2.2 on the 2026-10-09 booster (95 m/s, 3.8 MN on 31 t, 0.35 s nodes) - in game it coasted, held 3.00 g to the end and hit the pad at 10.9 m/s.
    ///  300 keeps that ratio under about 0.3 from 50 nodes down to 5 on the 7-30 t boosters, and flew well in game on 2026-10-09. The ratio grows with finer nodes and lighter thrust; a much different vehicle could need it lower.
    ///  NOT VERIFIED TO BIND EVERYWHERE: offline, --path's acceleration case no longer holds the limit at 300 (peak 24.1 m/s^2 against 12.3, IterationLimit) where 1e4 held it exactly - a flat weight's meaning depends on the scales, which differ there.
    /// </summary>
    public double AccelWeight { get; init; } = 300;
    public bool AccelLimitEnabled => AccelMax > 0.0;

    /// <summary>
    /// THE TURN-RATE LIMIT, rad/s, or 0 for none: |w_k| &lt;= RateMaxRadS at every node from node 1 on, the body rate's whole magnitude, roll included. An exact second-order cone, soft like the glideslope with an L1 slack at <see cref="RateWeight"/>, so a craft handed over turning faster than this is brought back under it rather than given no plan.
    ///  WHY. Before the RCS, the gimbals' small authority kept the plan's turns gentle. With the RCS the craft can turn hard, nothing in the objective objects (the rate penalty WW is tuned small in flight), and the cheapest plan used it: on 2026-11-20 the booster swung out to 23 deg of tilt and 25 deg/s of roll with the RCS slammed between its limits.
    /// </summary>
    public double RateMaxRadS { get; init; }

    /// <summary>The turn-rate slack's price per rad/s over the limit, per node.</summary>
    public double RateWeight { get; init; } = 1e3;

    public bool RateLimitEnabled => RateMaxRadS > 0.0;

    /// <summary>
    /// Whether the turn-rate limit counts the roll rate. Off for a plan with a coast before ignition: there the air can roll the craft harder than its RCS can stop - 40 kN m against 22 on the 2026-11-20 booster, which spun at 30 deg/s under entry guidance in the headless harness - and a limit on a rate nothing can hold only makes the plan infeasible. The pitch and yaw rates, which steer the craft, stay limited.
    /// </summary>
    public bool RateLimitIncludesRoll { get; init; } = true;

    /// <summary>Components of the body rate the turn-rate limit takes: 3 with the roll, 2 without.</summary>
    public int RateLimitComponents => RateLimitIncludesRoll ? 3 : 2;

    /// <summary>cot of the glideslope half-angle: the cone's horizontal run per unit of height.</summary>
    public double CotGlideSlope => 1.0 / Math.Tan(Math.Clamp(GlideSlopeDeg, 1e-3, 89.999) * Math.PI / 180.0);

    /// <summary>
    /// THE PROGRESSIVE GLIDESLOPE: the heights over the target, m, ascending, at which the funnel's half-angle from vertical, seen from the target (as GLIDE SLOPE's angle is), is <see cref="GlideFunnelHalfAngleDeg"/>: the offset allowed there is h tan(half-angle). Null or empty for the plain cone. Between two heights the offset allowed changes linearly, below the first it is the first's cone, and above the last it widens at least at the last's half-angle. The plain cone of <see cref="GlideSlopeDeg"/> stays as the outer bound. Flight sets one craft length, 300 m and 1000 m (Ksa6DofSetup.GlideFunnelHeights).
    ///  WHY (#335, 2026-11-27). The plan's cost is mostly fuel, which is flat across many shapes, so the plain 10 deg cone - 5.7 m of offset per metre of height, 113 m at 20 m - left the shape to wherever SCvx's local optimum fell: one Starship landing lit 109 m out and swept in at 20 m/s sideways through 50 m, another lit 561 m out. Real landings come down the last stretch nearly vertical.
    ///  NOT CONVEX: a funnel that narrows towards the ground is a trumpet. The subproblem takes its tangent at each node's reference height (an affine bound on the offset, a second-order cone as before), and the merit prices the true funnel, as SCvx linearises every non-convex limit. Its slopes are made non-decreasing in height, so it is convex in height and every tangent lies inside it.
    /// </summary>
    public double[]? GlideFunnelHeightsM { get; init; }

    /// <summary>The progressive glideslope's half-angles from vertical at <see cref="GlideFunnelHeightsM"/>, deg.</summary>
    public double[]? GlideFunnelHalfAngleDeg { get; init; }

    /// <summary>
    /// The price of the progressive glideslope's narrowing beyond the plain cone, per <see cref="GlideFunnelPriceScaleM"/> of offset, as <see cref="GlideSlopeWeight"/> prices the cone: low, so the funnel SHAPES a plan that can follow it and gives way for one that cannot, rather than enforcing a corridor. Past the plain cone the glideslope's own price applies.
    ///  Offline (2026-11-27), at the cone's own price the funnel took a gimbal-limited booster that ends 8-15 m off the pad (--6dof-lift) from 40 plans kept to none.
    /// </summary>
    public double GlideFunnelWeight { get; init; } = 10.0;

    /// <summary>The length the funnel's price is per, m: fixed, as the terminal miss's is (TerminalMissScaleM), so a metre outside costs the same however far away the plan starts. On the position scale - the craft's distance at setup, about 240 m in the offline checks and 2 km for a Starship at standby - it would barely touch the Starship.</summary>
    public const double GlideFunnelPriceScaleM = 250.0;

    /// <summary>
    /// THE SIDEWAYS-SPEED CONE, 1/s, or 0 for none: the plan's horizontal speed at each node from node 1 is at most this times its height over the target, plus <see cref="SideSpeedFloorMs"/> - 0.3 allows 3.3 m/s at 10 m and 0.9 m/s at 2 m - so the sideways speed is gone before the last metres and the craft comes down them straight, as a booster that lands 0.2 m/s sideways does. Soft, with its own slack at <see cref="SideSpeedWeight"/>. Exactly convex: a second-order cone in the state, so it needs no linearisation.
    ///  WHY (#335, 2026-11-27). The position funnel limits how far off the craft may be, not how fast it moves, and the plans met it and left the sideways speed to the last second: two Starship landings passed over the target with their legs 7-9 m up at 2.9-3.8 m/s. The last plans then stopped it by leaning one way and pushing the other with the gimbal; one engine's gimbal could not both turn the craft and push it, the attitude feedback's corrections pushed the other way, and both touched down sliding at 3.3-4.2 m/s and toppled.
    /// </summary>
    public double SideSpeedPerM { get; set; }

    /// <summary>The sideways speed the cone allows at the target's height, m/s.</summary>
    public double SideSpeedFloorMs { get; set; } = 0.3;

    /// <summary>The price of sideways speed over the cone, per <see cref="SideSpeedPriceScaleMs"/> per node.</summary>
    public double SideSpeedWeight { get; set; } = 10.0;

    /// <summary>The speed the sideways-speed cone's price is per, m/s: fixed, as the funnel's length is.</summary>
    public const double SideSpeedPriceScaleMs = 20.0;

    public bool SideSpeedEnabled => SideSpeedPerM > 0.0;

    /// <summary>
    /// THE SINK-RATE FUNNEL, 1/s, or 0 for none: the plan's speed down at each node from node 1 is at most this times its height over the target plus <see cref="SinkFloorMs"/> - 0.5 with a 2 m/s floor allows 12 m/s at 20 m and 3 m/s at 2 m. Soft, its slack at <see cref="SinkWeight"/> per <see cref="SideSpeedPriceScaleMs"/> per node. Linear in the state.
    ///  WHY (2026-11-27 campaign). A propellant-best plan brakes as late as it can: every Starship landing of the day came down its last 20-30 m at 19-24 m/s on full throttle, leaving no time to stand the craft up and take out its sideways speed, and the re-solves slid the braking later still. This keeps the end of the burn gentle, so the last seconds have room.
    /// </summary>
    public double SinkPerM { get; set; }

    /// <summary>The speed down the sink-rate funnel allows at the target's height, m/s.</summary>
    public double SinkFloorMs { get; set; } = 2.0;

    /// <summary>The price of speed down over the funnel, per <see cref="SideSpeedPriceScaleMs"/> per node.</summary>
    public double SinkWeight { get; set; } = 200.0;

    public bool SinkFunnelEnabled => SinkPerM > 0.0;

    /// <summary>
    /// THE FIRST GIMBAL COMMAND'S ANCHOR, N, or null for none: (tdx, tdy, band) - node 0's lateral thrust may differ from (tdx, tdy) by at most band on each axis. Set by the flight for a warm re-solve to the command it is flying now, with band what the gimbal slews in two re-solve cycles (Ksa6DofGuidance.FirstGimbalAnchor), so a new plan carries on from where the gimbal is going rather than jumping. Settable between solves, as VirtualControlScale is.
    ///  WHY (#335, 2026-11-27). Each re-solve started from the measured state but its first gimbal command was free, so every new plan, about every 0.15 s, stepped the feedforward by 1-5 deg - faster than the gimbal slews (about 20 deg/s). In the last 5 s of two Starship landings 15 of 22 and 23 of 35 too-fast steps came at a new plan; the gimbal lagged, the attitude fell 5-7 deg behind, re-solves were refused, and the attitude feedback's deflection pushed the craft sideways into a topple.
    /// </summary>
    public double[]? FirstGimbalAnchor { get; set; }

    public bool GlideSlopeProgressive => IsProgressive(GlideFunnelHeightsM, GlideFunnelHalfAngleDeg);

    /// <summary>The glideslope's horizontal run per metre of height at <paramref name="heightM"/> over the target.</summary>
    public double GlideSlopeRunPerMAt(double heightM) => RunPerM(GlideSlopeDeg, GlideFunnelHeightsM, GlideFunnelHalfAngleDeg, heightM);

    /// <summary>The largest horizontal offset from the target the glideslope allows at <paramref name="heightM"/> over it, m: the integral of <see cref="GlideSlopeRunPerMAt"/> from 0, so the plain cone's cot x height when not progressive. Negative below the target, as the cone's is.</summary>
    public double GlideSlopeRadiusAt(double heightM) => Radius(GlideSlopeDeg, GlideFunnelHeightsM, GlideFunnelHalfAngleDeg, heightM);

    /// <summary>Whether funnel knots are given at all: matching, non-empty, heights ascending and non-negative, half-angles between 0 and 90 deg.</summary>
    public static bool IsProgressive(double[]? heightsM, double[]? halfAngleDeg)
    {
        if (heightsM is not { Length: > 0 } || halfAngleDeg is null || halfAngleDeg.Length != heightsM.Length)
            return false;
        for (int i = 0; i < heightsM.Length; i++)
        {
            if (!(heightsM[i] >= 0.0) || !(halfAngleDeg[i] > 0.0 && halfAngleDeg[i] < 90.0) || (i > 0 && !(heightsM[i] > heightsM[i - 1])))
                return false;
        }
        return true;
    }

    private static double Cot(double deg) => 1.0 / Math.Tan(Math.Clamp(deg, 1e-3, 89.999) * Math.PI / 180.0);

    /// <summary>The funnel's slope on each stretch, m of offset per m of height: stretch 0 from the target to the first height, i from height i-1 to i, and the last above the last height. From the offsets at the heights, h tan(half-angle); made non-decreasing so the funnel stays convex (a stretch that would narrow upward is widened), and no wider than the plain cone.</summary>
    private static double[] Slopes(double plainDeg, double[] heightsM, double[] halfAngleDeg)
    {
        int n = heightsM.Length;
        double cap = Cot(plainDeg);
        var slope = new double[n + 1];
        double prevH = 0.0, prevR = 0.0;
        for (int i = 0; i < n; i++)
        {
            double r = heightsM[i] * Math.Tan(halfAngleDeg[i] * Math.PI / 180.0);
            slope[i] = heightsM[i] > prevH ? (r - prevR) / (heightsM[i] - prevH) : Math.Tan(halfAngleDeg[i] * Math.PI / 180.0);
            prevH = heightsM[i];
            prevR = r;
        }
        slope[n] = Math.Max(slope[n - 1], Math.Tan(halfAngleDeg[n - 1] * Math.PI / 180.0));
        for (int i = 0; i <= n; i++)
            slope[i] = Math.Min(Math.Max(slope[i], i > 0 ? slope[i - 1] : 0.0), cap);
        return slope;
    }

    /// <summary>The run per metre of height of a funnel at <paramref name="heightM"/> (see <see cref="GlideSlopeRunPerMAt"/>): its slope on that stretch. For the drawings and the log, which have no config.</summary>
    public static double RunPerM(double plainDeg, double[]? heightsM, double[]? halfAngleDeg, double heightM)
    {
        if (!IsProgressive(heightsM, halfAngleDeg))
            return Cot(plainDeg);
        double[] slope = Slopes(plainDeg, heightsM!, halfAngleDeg!);
        for (int i = 0; i < heightsM!.Length; i++)
            if (heightM < heightsM[i])
                return slope[i];
        return slope[heightsM.Length];
    }

    /// <summary>The radius of a funnel at <paramref name="heightM"/> (see <see cref="GlideSlopeRadiusAt"/>): the offset allowed, the sum of its slopes over the stretches below.</summary>
    public static double Radius(double plainDeg, double[]? heightsM, double[]? halfAngleDeg, double heightM)
    {
        if (!IsProgressive(heightsM, halfAngleDeg))
            return Cot(plainDeg) * heightM;
        double[] slope = Slopes(plainDeg, heightsM!, halfAngleDeg!);
        if (heightM <= 0.0)
            return slope[0] * heightM;
        double radius = 0.0, below = 0.0;
        for (int i = 0; i < heightsM!.Length; i++)
        {
            double top = Math.Min(heightM, heightsM[i]);
            radius += slope[i] * (top - below);
            if (heightM <= heightsM[i])
                return radius;
            below = heightsM[i];
        }
        return radius + slope[heightsM.Length] * (heightM - below);
    }

    public double Tmin => ThrottleFloor * Tmax;
    public double TanGimbal => Math.Tan(GimbalMaxDeg * Math.PI / 180.0);
    public double CosTilt => Math.Cos(TiltMaxDeg * Math.PI / 180.0);

    /// <summary>
    /// Radius of the tilt constraint expressed as a NORM BALL on the quaternion's
    /// (qx, qy), which is what the subproblem actually enforces.
    ///
    /// The tilt of the body z-axis from vertical is acos(R22) with
    /// R22 = 1 - 2(qx^2 + qy^2), so
    ///
    ///     tilt &lt;= tiltMax
    ///       &lt;=&gt;  1 - 2(qx^2 + qy^2) &gt;= cos(tiltMax)
    ///       &lt;=&gt;  qx^2 + qy^2 &lt;= (1 - cos(tiltMax)) / 2
    ///       &lt;=&gt;  ||(qx, qy)|| &lt;= sin(tiltMax / 2)
    ///
    /// - a CONVEX second-order cone in the original variables, exact, and with no
    /// dependence on the reference trajectory. See the tilt block in
    /// Scvx6DofSubproblemScs.AssembleCone for why that matters: the linearisation
    /// this replaces had a gradient of (-4*qx, -4*qy), which is exactly zero for a
    /// vertical vehicle.
    ///
    /// Written as sin(tiltMax/2) rather than sqrt((1 - cos(tiltMax)) / 2). They are
    /// the same number, but the half-angle form does not lose precision to
    /// cancellation in 1 - cos for small angles.
    ///
    /// Valid across the whole 0-180 degree range: at 180 it evaluates to 1, and
    /// ||(qx, qy)|| &lt;= 1 holds for every unit quaternion, so the constraint goes
    /// vacuous exactly where it should.
    /// </summary>
    public double SinHalfTilt => Math.Sin(0.5 * TiltMaxDeg * Math.PI / 180.0);

    /// <summary>
    /// The largest RCS torque about each body axis, N m, positive way and negative way (as magnitudes): Dynamics6Dof.RcsModel.MaxPosNm and MaxNegNm. The subproblem bounds each axis's control between -neg and +pos, so null, or zeros, pin it at zero.
    /// </summary>
    public double[]? RcsMaxPosNm { get; init; }
    public double[]? RcsMaxNegNm { get; init; }

    /// <summary>
    /// The share of the craft's torque the plan's limits are: GimbalMaxDeg (in torque, the tangent), TauRollMax and the RCS bounds are this much of the hardware's, the rest kept for the feedback that holds the plan's attitude between re-solves (the flight's AttitudeFeedback6Dof). 1 for none kept. The solver does not read it; the flight divides by it to recover the hardware's limits.
    /// </summary>
    public double PlanTorqueShare { get; init; } = 1.0;

    /// <summary>The share of the RCS's torque the RCS bounds are (Ksa6DofSetup.PlanSurfaceShare), at most PlanTorqueShare: the flight divides by it to recover the RCS's whole torque for the feedback. 1 for none kept.</summary>
    public double PlanRcsShare { get; init; } = 1.0;

    private static double Bound(double[]? r, int a)
        => r is { } v && a < v.Length && double.IsFinite(v[a]) && v[a] > 0.0 ? v[a] : 0.0;

    /// <summary>RCS axis <paramref name="a"/>'s bounds, N m: the most it may give the positive way, and the negative way as a magnitude; zeros for none.</summary>
    public (double Pos, double Neg) RcsBounds(int a) => (Bound(RcsMaxPosNm, a), Bound(RcsMaxNegNm, a));

    /// <summary>
    /// TORQUE DEMAND (a prototype, issue #222): the plan asks for one torque per body axis and a mixer outside the plan splits it between the gimbals and the RCS (and later the fins). The three torque controls (Dynamics6Dof.IRCS) then carry the WHOLE control torque; the gimbals' lateral thrust and roll torque controls are pinned at zero; the side force the torque brings is Dynamics6Dof.RcsModel.ForcePerNm, an effective factor the caller sets from the mixer's split.
    ///  What the plan sees of the mixer is its envelope: per axis, the gimbals' torque per newton of axial thrust (<see cref="DemandTorquePerN"/> on pitch and yaw, <see cref="DemandRollPerN"/> on roll) times the thrust, plus the RCS's own bounds (<see cref="RcsBounds"/>); and how fast that torque can change, the gimbals' slew (GimbalSlewPerS) on their share plus what the RCS can jump.
    ///  WHY. With the gimbals and the RCS both free in the plan, two torque sources that cost about the same stand in for each other, and SCS takes 5-20 times the iterations on the warm re-solves (2026-11-19/20). One torque per axis leaves nothing to trade.
    /// </summary>
    public bool TorqueDemand { get; init; }

    /// <summary>The gimbals' pitch and yaw torque per newton of axial thrust, N m per N (engine arm times tan of the gimbal limit), for <see cref="TorqueDemand"/>.</summary>
    public double DemandTorquePerN { get; init; }

    /// <summary>The gimbals' roll torque per newton of axial thrust, N m per N, for <see cref="TorqueDemand"/>.</summary>
    public double DemandRollPerN { get; init; }

    /// <summary>The control torque's bounds on axis <paramref name="a"/> at axial thrust <paramref name="thrust"/>: the RCS's alone, or in <see cref="TorqueDemand"/> the gimbals' share at that thrust plus the RCS's.</summary>
    public (double Pos, double Neg) ControlTorqueBounds(int a, double thrust)
    {
        (double pos, double neg) = RcsBounds(a);
        if (!TorqueDemand)
            return (pos, neg);
        double g = (a < 2 ? DemandTorquePerN : DemandRollPerN) * Math.Max(thrust, 0.0);
        return (pos + g, neg + g);
    }

    /// <summary>
    /// The engine's arm from the centre of mass to the gimbal pivot, m (Dynamics6Dof.Params.LArm), for the RCS controls' scale; 0 leaves it out.
    /// </summary>
    public double GimbalArm { get; init; }

    /// <summary>
    /// The control scales: the gimbal, thrust and roll ones from UScale or Tmax, then one per RCS control: ALL the control torque on that axis, the RCS's authority plus the gimbals' at full thrust (Tmax tan(gimbal) GimbalArm on pitch and yaw, TauRollMax on roll), or the roll scale for a control pinned at zero (any positive figure would do there).
    ///  WHY THE AXIS'S WHOLE TORQUE AND NOT THE RCS'S ALONE. Scaled by its own authority, an RCS control's trust region (TrustRegion x scale) was a tenth of what the RCS can give per iteration, and the warm re-solves kept running into it: SCS took 5-20 times the iterations it needs with the gimbals alone, often its cap (2026-11-19/20). On the axis's whole torque (about 3x the RCS's on the 2026-11-18 booster) it took 325 at 3e-4 and 100-775 at RealTimeEpsWithRcs, against 350-575 for the gimbals alone, and landed as well (offline --6dof-rcs). The torques the plan trades against each other share one scale.
    /// </summary>
    public double[] ResolvedUScale
    {
        get
        {
            // A craft with no gimbal has a zero cone, which pins its gimbal controls; their scale still has to be positive, so they take a hundredth of the thrust.
            // The lateral thrust reaches the cone's centre plus its width (GimbalTrimX/Y), so the scale takes both.
            double gimbalScale = Tmax * TanGimbal > 0.0 ? Tmax * (TanGimbal + GimbalTrim) : 0.01 * Tmax;
            double[] head = UScale ?? [gimbalScale, gimbalScale, Tmax, TauRollMax];
            if (head.Length >= NU)
                return head;
            var all = new double[NU];
            Array.Copy(head, all, Math.Min(head.Length, Dynamics6Dof.IRCS));
            double fallback = TauRollMax > 0.0 && double.IsFinite(TauRollMax) ? TauRollMax : 1.0;
            for (int a = 0; a < Dynamics6Dof.NRCS; a++)
            {
                (double pos, double neg) = ControlTorqueBounds(a, Tmax);
                double max = Math.Max(pos, neg);
                // TorqueDemand's bounds already hold the gimbals' share.
                double gimbal = TorqueDemand || !(GimbalArm > 0.0) ? 0.0
                    : a < 2 ? Tmax * TanGimbal * GimbalArm : Math.Max(TauRollMax, 0.0);
                all[Dynamics6Dof.IRCS + a] = max > 0.0 ? max + gimbal : fallback;
            }
            // A fin's scale is its range, degrees; a fin with none, pinned at zero, any positive figure.
            for (int i = 0; i < FinCount; i++)
            {
                (double lo, double hi) = FinBounds(i);
                double range = Math.Max(-lo, hi);
                all[Dynamics6Dof.IFIN + i] = range > 0.0 ? range : 1.0;
            }
            return all;
        }
    }
}
