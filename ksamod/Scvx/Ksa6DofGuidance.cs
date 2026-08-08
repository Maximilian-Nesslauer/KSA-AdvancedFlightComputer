using System;
using Brutal.Numerics;
using KSA;
using Scvx;

/// <summary>
/// 6-DOF powered descent as straight model-predictive control.
///
/// Every cycle: re-solve from the LIVE vehicle state, then apply the optimiser's own
/// controls, interpolated along the fresh trajectory. That is the whole algorithm.
/// There is no trajectory tracking, no PD, no attitude reference — MPC gets its
/// feedback from re-solving at the measured state, which is the point of it.
///
/// The optimiser's control IS the actuator command: u = (tdx, tdy, T, tau_roll) is a
/// gimbal deflection, an axial thrust and a roll torque. Applying it means
///     torque  = r_T x T_body = (LArm*tdy, -LArm*tdx, tau_roll)
///     thrust  = T, in NEWTONS
/// and nothing else. Converting that thrust into a KSA throttle needs the vehicle's
/// LIVE capability and so belongs at the KSA boundary, not here — see Command. Attitude then evolves from the torque that deflection produces,
/// exactly as the model's own dynamics say it will.
///
/// NODE 0 IS THE VEHICLE, and that has to be enforced rather than assumed. The
/// subproblem pins it as an equality (`X[0][i] = x0[i]`), but the plan we read back
/// is the SCvx REFERENCE trajectory, which only advances on an ACCEPTED step. Reseed
/// sets that reference to the previous plan shifted forward — whose node 0 is the old
/// plan's node `shift`, NOT the vehicle. So a cycle where the ratio test accepts
/// nothing leaves a plan anchored an interval away from the vehicle, and the commands
/// are read at the wrong point on it. AnchorOffsetM measures exactly that, and a plan
/// is refused if it is not anchored.
///
/// NOT THREADED. A re-solve costs ~33 ms in the caller's frame.
/// </summary>
public sealed class Ksa6DofGuidance
{
    private const int NX = 14;
    private const int NU = 4;

    private readonly Scvx6DofConfig _cfg;
    private readonly Dynamics6Dof.Params _dyn;
    private readonly Scvx6DofSolver _solver;
    private readonly int _n;

    private double[] _planX = [];
    private double[] _planU = [];
    private double _planSigma;
    private double _solveTime;          // sim time at which node 0 was the vehicle

    public ScvxStatus Status { get; private set; } = ScvxStatus.Failed;
    public string Error { get; private set; } = "";
    public int LastIterations { get; private set; }
    public int AcceptedSteps { get; private set; }
    public double LastSolveMs { get; private set; }
    public int SolveCount { get; private set; }
    public bool HasPlan => _planX.Length > 0;

    /// <summary>Plan duration (the solver's free final time), seconds.</summary>
    public double Sigma => _planSigma;

    /// <summary>Seconds since the solve that produced the current plan.</summary>
    public double PlanElapsed { get; private set; }

    /// <summary>
    /// Distance between the plan's node 0 and the state it was solved for. Must be
    /// ~0 — node 0 is an equality constraint. Anything else means the reference did
    /// not advance and the plan is stale.
    /// </summary>
    public double AnchorOffsetM { get; private set; }

    /// <summary>
    /// Worst dynamics defect on the accepted plan, scaled by XScale. THE test of
    /// whether the plan is physical at all.
    ///
    /// The dynamics are imposed as X[k+1] = X[k] + 0.5*dtau*sigma*(g_k + g_k+1) + Wv[k]
    /// where Wv is VIRTUAL CONTROL — a slack variable, penalised by RhoVc but not
    /// constrained to zero. Until SCvx converges, Wv is non-zero and the trajectory
    /// DOES NOT OBEY THE DYNAMICS: the state teleports between nodes on fictitious
    /// forces. Such a plan cannot be flown at any thrust, which is what "the vehicle
    /// chases a solution it cannot fly" looks like from the cockpit.
    /// </summary>
    public double LastDefect { get; private set; } = double.PositiveInfinity;

    /// <summary>Defect below which the plan counts as physically realisable.</summary>
    public double DefectTolerance => _solver.DefectTolerance;

    /// <summary>
    /// The same defect expressed in METRES — LastDefect * XScale — which is what the
    /// flight gate actually judges. See Finish for why the scaled figure is the wrong
    /// yardstick on an approach.
    /// </summary>
    public double LastDefectM { get; private set; } = double.PositiveInfinity;

    /// <summary>
    /// Largest position defect, in metres, that still counts as a flyable plan.
    ///
    /// Measured across a full descent (Scvx.Console --defect), a well-resolved plan
    /// sits around 0.1 m regardless of range, so 1 m is comfortably permissive
    /// without accepting anything meaningfully unphysical. It DOES bite on a badly
    /// under-resolved plan, which is the case worth catching: at 235 m altitude the
    /// absolute defect is 0.13 m at N=50, 0.45 m at N=20 and 2.36 m at N=10.
    /// </summary>
    public double MaxDefectM { get; set; } = 1.0;

    /// <summary>
    /// The same gate for a COLD solve, deliberately far looser.
    ///
    /// The two cases have completely different costs for refusing. A refused
    /// re-solve falls back on the previous plan, which is very nearly as good - it
    /// is one cadence tick stale - so being strict there is cheap. A refused COLD
    /// solve leaves NO plan at all: the guidance simply does not engage, and the
    /// vehicle gets nothing.
    ///
    /// A few metres of defect on a first plan is not a reason to refuse to fly. MPC
    /// re-solves ten times a second from the measured state, so the first plan only
    /// has to be a usable starting point - the loop refines it long before the
    /// vehicle has travelled far. Some solution is better than no solution.
    /// </summary>
    public double ColdMaxDefectM { get; set; } = 15.0;

    /// <summary>The Tmax the PLAN was built against — a fallback divisor only; see Command.</summary>
    public double Tmax => _cfg.Tmax;

    /// <summary>Sigma bounds, so the UI can show when burn time is being DICTATED by a bound rather than chosen.</summary>
    public double SigmaMin => _cfg.SigmaMin;
    public double SigmaMax => _cfg.SigmaMax;

    /// <summary>Consecutive failed Update calls, used to stop paying for a retry that is not working.</summary>
    private int _consecutiveFailures;

    /// <summary>True when the last Update needed the wide-trust-region retry — that retry is what turns a ~30 ms solve into ~500 ms.</summary>
    public bool FellBack { get; private set; }

    /// <summary>
    /// Update the inertia used by the dynamics. KSA's TotalMassPropsBody.Inertia is
    /// LIVE — it is rebuilt whenever propellant changes — so a value captured at
    /// engage goes stale over a burn that spends a meaningful fraction of the wet
    /// mass. That produces a SYSTEMATIC torque error: the plan is computed against a
    /// heavier vehicle than the one flying, so every commanded torque is wrong in the
    /// same direction, and MPC cannot correct it because re-anchoring the state does
    /// not touch the model. Call this each cycle; it is three field writes.
    /// </summary>
    public void SetInertia(double ixx, double iyy, double izz)
    {
        if (ixx > 0.0) _dyn.Ixx = ixx;
        if (iyy > 0.0) _dyn.Iyy = iyy;
        if (izz > 0.0) _dyn.Izz = izz;
    }

    /// <summary>Inertia currently in the model, for the readout.</summary>
    public double3 Inertia => new(_dyn.Ixx, _dyn.Iyy, _dyn.Izz);

    private double3 _baseGravity;
    private bool _haveBaseGravity;

    /// <summary>
    /// Unmodelled acceleration, site frame, m/s^2 — added to the model's gravity so
    /// the OPTIMISER plans around it rather than fighting it.
    ///
    /// This is offset-free MPC, and it is the piece that was missing. Plain MPC
    /// corrects the STATE every cycle but keeps planning with the same wrong model,
    /// so a persistent force it does not know about is re-encountered identically on
    /// every replan: the plan promises to arrive, the vehicle falls short, the next
    /// plan promises again. Nothing about re-solving fixes a model error.
    ///
    /// Measured on a real descent, the model was short by about 2.2 m/s^2 in the
    /// vertical — a fitted 10.3% thrust shortfall on top of gravity being 9.4% low
    /// (10.74 measured against 9.82 from Mu/r^2). That is roughly a fifth of the
    /// vehicle's net climb authority, applied continuously, which is more than
    /// enough to turn an approach into an overshoot and then an orbit.
    ///
    /// Estimating the RESIDUAL rather than any individual term is deliberate: it
    /// needs no theory about which of gravity, thrust calibration or aerodynamics is
    /// responsible, and it picks up drag for free — including the way drag falls off
    /// as speed comes off, since the estimate simply follows it down.
    /// </summary>
    public double3 AccelBias { get; private set; }

    public void SetAccelBias(double3 bias)
    {
        if (!_haveBaseGravity)
        {
            _baseGravity = new double3(_dyn.Gx, _dyn.Gy, _dyn.Gz);
            _haveBaseGravity = true;
        }
        if (!double.IsFinite(bias.X) || !double.IsFinite(bias.Y) || !double.IsFinite(bias.Z))
            return;

        AccelBias = bias;
        _dyn.Gx = _baseGravity.X + bias.X;
        _dyn.Gy = _baseGravity.Y + bias.Y;
        _dyn.Gz = _baseGravity.Z + bias.Z;
    }

    /// <summary>Gravity the config was built with, before any bias — for the readout.</summary>
    public double3 BaseGravity => _haveBaseGravity ? _baseGravity : new double3(_dyn.Gx, _dyn.Gy, _dyn.Gz);

    /// <summary>
    /// Fly a FIXED burn time instead of letting the solver choose it.
    ///
    /// Free final time makes the dynamics BILINEAR in (sigma, x, u) — sigma multiplies
    /// f(x,u) in the collocation — which is a first-class nonconvexity and the root of
    /// several separate pathologies: sigma pinning at its bounds, the regularisers
    /// biasing the trajectory (BOTH W_DU and W_W get cheaper as sigma grows, so they
    /// push it to the ceiling), loitering to fill an over-long burn, and plan-to-plan
    /// inconsistency as sigma jumps between MPC cycles.
    ///
    /// Pinning sigma removes all of that at once. In particular the regularisers can
    /// no longer buy anything by stretching time, so they can be set for smoothness
    /// and conditioning without distorting the answer.
    ///
    /// This is what the 3-DOF Gfold planner already does — fixed time-of-flight per
    /// solve with an outer bracket-and-golden-section search over it — and is the
    /// classic powered-descent formulation.
    /// </summary>
    public bool FixedTime { get; set; } = true;

    /// <summary>Burn time committed at engage; receding horizon counts down from it.</summary>
    public double CommittedSigma { get; private set; }

    /// <summary>Floor for the counted-down burn time, so the last cycles before arrival cannot divide by ~0.</summary>
    public double MinimumBurnTime { get; init; } = 1.0;

    /// <summary>
    /// ADMM iteration caps. THIS IS WHAT STOPS THE GAME FREEZING.
    ///
    /// SCS defaults to 100,000 iterations per subproblem — an offline-validation
    /// number. Measured in closed loop, an uncapped worst-case subproblem ran 31,800
    /// iterations / 1.7 s against a mean of ~400, and several of those back to back on
    /// the sim thread is a multi-second stall: the game stops responding and is killed.
    ///
    /// Capping is free here. Same closed-loop run, cap 2000 vs uncapped: identical
    /// miss (2.1 m), BETTER path (1.27 vs 1.36 x direct), same plan jump, worst case
    /// 561 ms instead of 1709 ms. It is safe because SCS reports a truncated solve as
    /// SolvedInaccurate, HitIterationLimit catches it, and the SCvx loop already
    /// treats that as a subproblem failure and shrinks the trust region — so the cap
    /// turns a stall into a smaller step.
    ///
    /// Cold needs more than warm: a cap of 1000 fails the cold solve outright.
    /// </summary>
    public int ColdAdmmCap { get; init; } = 20_000;
    public int WarmAdmmCap { get; init; } = 2_000;

    /// One retry budget for a subproblem that merely ran out of iterations. Keeps the
    /// common case fast on the low cap while giving the occasional hard subproblem
    /// enough room to finish, instead of shrinking the trust region and spiralling.
    public int EscalatedAdmmCap { get; init; } = 12_000;

    /// <summary>
    /// Wall-clock budget per subproblem, milliseconds. THE CAP THAT ACTUALLY MATTERS.
    ///
    /// An ITERATION cap does not bound TIME, because the cost of one ADMM iteration
    /// scales with problem size: measured, ~0.04 ms at N=30 but ~0.77 ms at N=80,
    /// twenty times more, since the KKT factorisation grows. So the 2000-iteration cap
    /// that kept N=30 under 60 ms is 1.5 SECONDS at N=80 — which is exactly the
    /// "periodic 2000 ms solve" seen in flight: the common case converges in ~50
    /// iterations and is a few ms, while the occasional hard subproblem takes its full
    /// budget and stalls the frame.
    ///
    /// So the iteration cap is DERIVED each cycle from a measured cost per iteration.
    /// That adapts automatically to node count, vehicle and machine, instead of being
    /// a constant that is only correct for one problem size.
    /// </summary>
    public double SubproblemBudgetMs { get; init; } = 40.0;

    /// <summary>Budget for the one escalated retry. Larger, but still bounded.</summary>
    public double EscalatedBudgetMs { get; init; } = 200.0;

    /// <summary>
    /// Wall-clock ceiling for a WHOLE guidance cycle, ms - the number that decides
    /// whether the sim thread stutters.
    ///
    /// The per-subproblem budgets above bound one solve; a cycle runs up to
    /// maxIterations of them, so they authorised 5 x 40 ms, and 5 x 200 ms once the
    /// escalated retry fired. Measured in flight log 20260807-103232: p50 108 ms and
    /// p99 432 ms per cycle against a 100 ms cadence. Nothing was exceeding its
    /// stated budget; the stated budget was simply not a budget for the thing that
    /// hitches.
    ///
    /// 25 ms is a quarter of the cadence, so a solve and the frame it lands in still
    /// leave room for the game. Iterations beyond the first are refinement - measured
    /// defect over that whole flight was 0.02-0.13 m against a 1 m gate - so trading
    /// them for frame time is cheap.
    /// </summary>
    public double CycleBudgetMs { get; set; } = 25.0;

    /// <summary>Measured cost of one ADMM iteration, ms. Seeded pessimistically and refined from real solves.</summary>
    public double MsPerAdmmIteration { get; private set; } = 0.05;

    /// <summary>
    /// WHERE the last reported defect actually is. The magnitude on its own cannot be
    /// read: DefectNorm is a max over all fourteen state channels, each divided by its
    /// OWN scale, and LastDefectM then multiplies it by the POSITION scale to put it in
    /// metres. That is a real distance only when the worst channel happens to be a
    /// position. When it is a body rate, "3900 m" is a rate error in rad/s times a
    /// length in metres, and reading it as a spatial error points the investigation at
    /// entirely the wrong thing.
    /// </summary>
    public int LastDefectChannel { get; private set; } = -1;

    /// <summary>Interval (between node k and k+1) carrying the worst defect, or -1.</summary>
    public int LastDefectNode { get; private set; } = -1;

    public string LastDefectChannelName => Scvx6DofSolver.ChannelName(LastDefectChannel);
    public string LastDefectGroup => Scvx6DofSolver.ChannelGroup(LastDefectChannel);

    /// <summary>
    /// The worst defect in ITS OWN units - scaled error times that channel's scale -
    /// which is the number that can actually be interpreted. Metres for a position,
    /// m/s for a velocity, rad/s for a body rate, dimensionless for a quaternion.
    /// </summary>
    public double LastDefectRaw => LastDefectChannel >= 0 && LastDefectChannel < _cfg.XScale.Length
        ? LastDefect * _cfg.XScale[LastDefectChannel]
        : double.NaN;

    public string LastDefectUnits => Scvx6DofSolver.ChannelGroup(LastDefectChannel) switch
    {
        "position" => "m",
        "velocity" => "m/s",
        "attitude" => "(quat)",
        "rate" => "rad/s",
        "mass" => "kg",
        _ => "",
    };

    /// <summary>Subproblems that needed the escalated budget, for the readout.</summary>
    public int Escalations => _solver.Escalations;

    /// <summary>Plan node count, for the overlay.</summary>
    public int Nodes => _n;

    public ReadOnlySpan<double> PlanState => _planX;
    public ReadOnlySpan<double> PlanControl => _planU;

    public Ksa6DofGuidance(Scvx6DofConfig cfg, Dynamics6Dof.Params dyn)
    {
        _cfg = cfg;
        _dyn = dyn;
        _n = cfg.Nodes;
        _solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        _solver.MaxSubproblemIterations = ColdAdmmCap;
        _solver.EscalatedSubproblemIterations = EscalatedAdmmCap;
    }

    /// <summary>Cold solve from a straight-line seed. ~1.7 s, so do it during a coast.</summary>
    public bool Plan(double[] x0, double[] xf, double sigmaSeed, double simNow, int maxIterations = 25)
    {
        BuildColdSeed(x0, xf, out double[] xSeed, out double[] uSeed);
        _xf = (double[])xf.Clone();
        _solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed);
        return Finish(x0, simNow, maxIterations, cold: true);
    }

    /// <summary>
    /// The straight-line cold seed, shared by Plan and BeginCold.
    ///
    /// Attitude and rates interpolate FROM THE MEASURED STATE rather than jumping to
    /// identity - see the note inside. That is a feasibility requirement, not a
    /// quality one.
    /// </summary>
    private void BuildColdSeed(double[] x0, double[] xf, out double[] xSeed, out double[] uSeed)
    {
        xSeed = new double[_n * NX];
        uSeed = new double[_n * NU];
        double m0 = x0[13];
        for (int k = 0; k < _n; k++)
        {
            double t = (double)k / (_n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }

            // ATTITUDE AND RATES MUST INTERPOLATE FROM THE MEASURED STATE, not jump
            // to identity. This is a FEASIBILITY requirement, not a quality one.
            //
            // The trust region is a box on every node INCLUDING NODE 0
            // (|X[k][i] - xbar[k][i]| <= tr * XScale[i]), and node 0 is simultaneously
            // pinned by the equality X[0] = x0. Together those demand
            // |x0[i] - xSeed[0][i]| <= tr * XScale[i]. With identity seeded into the
            // quaternion and zero into the rates — XScale 1, tr 0.1 — the cold solve
            // is INFEASIBLE for any vehicle more than ~11.5 deg off vertical or
            // rotating faster than 0.1 rad/s. Not slow: infeasible, immediately, and
            // untouched by relaxing any of the physical constraints.
            //
            // The Python reference never shows this because it starts at exactly
            // q = identity, omega = 0, so its seed matches node 0 perfectly.
            Slerp(x0.AsSpan(6, 4), xf.AsSpan(6, 4), t, xSeed.AsSpan(k * NX + 6, 4));
            for (int i = 0; i < 3; i++)
                xSeed[k * NX + 10 + i] = x0[10 + i] * (1.0 - t);   // spin down to zero

            xSeed[k * NX + 13] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * Math.Abs(_dyn.Gz);    // ~hover axial thrust
        }

        // Belt and braces: node 0 IS the measured state, exactly. Interpolation should
        // already give this at t=0, but the equality and the trust region leave no
        // slack at all here, so it is not worth depending on floating-point luck.
        Array.Copy(x0, 0, xSeed, 0, NX);

    }

    /// <summary>
    /// Cold solve from a SUPPLIED seed rather than the built-in straight line.
    ///
    /// SCvx refines a reference, it does not search for one, so the seed sets both how
    /// many iterations the cold solve needs and which local solution it walks toward.
    /// See Ksa6DofGfoldSeed for the one that matters: a convex 3-DOF solve of the same
    /// landing, which costs a few milliseconds and gets position, velocity, mass, burn
    /// time and thrust direction all approximately right.
    /// </summary>
    public bool PlanFromSeed(double[] x0, double[] xf, double[] xSeed, double[] uSeed,
                             double sigmaSeed, double simNow, int maxIterations = 25)
    {
        if (xSeed.Length != _n * NX || uSeed.Length != _n * NU)
            return false;

        // Node 0 must be the measured state exactly - the equality pins it and the
        // trust region applies there too, so a seed that disagrees is infeasible
        // rather than merely inaccurate.
        Array.Copy(x0, 0, xSeed, 0, NX);

        _xf = (double[])xf.Clone();
        if (FixedTime) PinSigma(sigmaSeed);
        _solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed);
        return Finish(x0, simNow, maxIterations, cold: true);
    }

    /// <summary>
    /// Begin a cold solve WITHOUT running it, so the iterations can be spread over
    /// several frames by StepCold.
    ///
    /// Blocking for a whole cold solve costs 130-280 ms in the caller's frame, which
    /// on the sim thread is a visible hitch. Spreading it is possible because SCvx is
    /// iterative by construction and the solver keeps all its state between calls -
    /// Iterate() is just one pass of the loop that Solve() runs in a batch.
    /// </summary>
    public void BeginCold(double[] x0, double[] xf, double sigmaSeed)
    {
        BuildColdSeed(x0, xf, out double[] xSeed, out double[] uSeed);
        _xf = (double[])xf.Clone();
        if (FixedTime) PinSigma(sigmaSeed);
        _solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed);
        Status = ScvxStatus.IterationLimit;
        Error = "converging";
        _consecutiveFailures = 0;
        _lastColdIterationS = 0.0;
        _coldIterations = 0;
        NeedsMoreNodes = false;
        _solver.SubproblemEps = ColdSubproblemEps;
    }

    /// <summary>
    /// Advance a spread cold solve by up to <paramref name="iterations"/> SCvx steps,
    /// re-anchored at the CURRENT vehicle state.
    ///
    /// Re-anchoring every frame is what makes the vehicle continuing to fall a
    /// non-issue: it is the same thing Update does at every cadence tick, so the drift
    /// is absorbed rather than invalidating the work so far.
    ///
    /// The iterations are also PACED - see ColdIterationIntervalS - so the handful it
    /// takes lands across about a second rather than on consecutive frames.
    ///
    /// Returns true once the plan is good enough to fly, at which point the caller
    /// should switch to Update.
    ///
    /// HANDS OVER AT THE WARM GATE, not the cold one. The cold gate exists so a first
    /// plan is not refused outright, but handing the warm loop a plan it will
    /// immediately reject is worse than not engaging: the loop refuses every cycle,
    /// the plan goes stale, the vehicle diverges from it, and each refusal costs MORE
    /// than a success because it runs the wide-trust-region retry. Flight log
    /// 20260807-093052 did exactly that - handed over at 14.48 m against a 1 m warm
    /// gate, then accepted 6 re-solves out of 103 while burning 250-500 ms a cycle.
    ///
    /// Iterating further is nearly free here, because the whole point of spreading is
    /// that the iterations are already amortised over frames. Only if it stalls does
    /// the cold gate apply, so a hard case still flies rather than never engaging.
    /// </summary>
    public bool StepCold(double[] x0, double simNow, int iterations = 1)
    {
        // PACING. An SCvx iteration is the indivisible unit of work here, so the cost
        // of one is whatever it is; what can be chosen is how close together they land.
        // Back to back on consecutive frames, three iterations read as ONE freeze of
        // their combined length. Spaced a few hundred milliseconds apart they read as
        // three brief hitches with normal frames in between, which is far less
        // noticeable even though the total work - and the total wall time - is the same
        // or worse.
        //
        // Paced on WALL CLOCK, not sim time. What is being protected is the frame
        // rate, which is a wall-clock property: sim time stalls when the game is
        // paused or running slow and races under time warp, so pacing on it either
        // never fires - one measured run managed a single iteration in 164 frames
        // and then hit the give-up limit - or fires every frame and spreads nothing.
        // Between iterations this returns immediately, so those frames cost nothing.
        double wallNow = _wallClock.Elapsed.TotalSeconds;
        if (_coldIterations > 0 && wallNow - _lastColdIterationS < ColdIterationIntervalS)
            return false;
        _lastColdIterationS = wallNow;
        _coldIterations++;

        // The flight budget, not a smaller one. An SCvx iteration is indivisible, so
        // cutting it short here would only mean throwing the frame's work away.
        ApplyTimeBudget(SubproblemBudgetMs, EscalatedBudgetMs);

        // Re-anchor at the vehicle before iterating, exactly as Update does.
        if (_solver.IterationCount > 0)
        {
            double[] xs = (double[])_solver.ReferenceX.Clone();
            double[] us = (double[])_solver.ReferenceU.Clone();
            Array.Copy(x0, 0, xs, 0, NX);
            _solver.Reseed(x0, xs, us, _solver.Sigma, trustRegion: _solver.TrustRegion);
        }

        var sw = System.Diagnostics.Stopwatch.StartNew();
        for (int i = 0; i < Math.Max(iterations, 1); i++)
            _solver.Iterate();
        double iterationMs = sw.Elapsed.TotalMilliseconds;

        // Accept against the WARM gate first. Only fall back to the looser cold gate
        // once the iteration count says it has stopped improving, so a genuinely hard
        // case still engages instead of converging forever. Finish does no solving
        // here - the 0 budget means it runs the gates over what Iterate just produced.
        // STALLED ABOVE THE WARM GATE MEANS TOO FEW NODES, NOT "GOOD ENOUGH".
        //
        // The loose cold gate exists so a first plan is not refused outright, but
        // handing the warm loop a plan it will reject is worse than not engaging - the
        // whole point of hlanding over at the warm gate. Flight log 20260808-104651 did
        // exactly this three times: cold solve settles at 7.87 m against the 15 m cold
        // gate, reports CONVERGED, and the warm loop then refuses that same plan
        // fifteen times at 7.8 m before cold-restarting to do it again.
        //
        // The solver's own message says what is wrong - "needs more iterations, or more
        // nodes" - and iterations have already stalled, so it is nodes. The caller can
        // act on that; settling cannot.
        bool stalled = _coldIterations >= ColdStallIterations;
        bool ok = Finish(x0, simNow, 0);
        NeedsMoreNodes = !ok && stalled && LastDefectM > MaxDefectM;

        // AFTER Finish, which overwrites both from the solver. The SOLVER's iteration
        // count is useless here - StepCold re-anchors through Reseed before every
        // iteration and Reseed zeroes IterationCount - so it read 1 no matter how many
        // ran, and ColdStallIterations above could never be reached. Its solve time is
        // likewise the time of a zero-iteration Finish, not of the work done.
        LastIterations = _coldIterations;
        LastSolveMs = iterationMs;

        // Back to flight accuracy for everything after this. The looser tolerance is
        // only defensible while the answer is a seed for the next solve rather than a
        // trajectory the vehicle is about to fly.
        if (ok)
            _solver.SubproblemEps = Scvx6DofSolver.RealTimeEps;
        return ok;
    }

    /// <summary>
    /// Accept the current cold plan against the LOOSE gate. Last resort, for when
    /// there are no more nodes to add: an imperfect plan the loop may refuse still
    /// beats no plan at all on a vehicle that is already falling.
    /// </summary>
    public bool AcceptCold(double[] x0, double simNow) => Finish(x0, simNow, 0, cold: true);

    /// <summary>
    /// Set when a cold solve has stopped improving while still above the warm gate.
    /// The caller should give it more nodes rather than accept the plan or restart at
    /// the same count - both of those just repeat the same solve.
    /// </summary>
    public bool NeedsMoreNodes { get; private set; }

    /// <summary>
    /// Iterations after which a spread cold solve is judged to have stopped improving.
    /// Generous, because spread iterations are cheap - the cost is bounded per FRAME,
    /// not in total.
    /// </summary>
    public int ColdStallIterations { get; set; } = 12;

    /// <summary>
    /// Sim-time gap between iterations of a spread cold solve, in seconds.
    ///
    /// Sets how the cold solve FEELS, not how long it takes. At 0.25 s the measured
    /// three-to-four-iteration cold solve lands across about a second, as four short
    /// hitches separated by fifteen normal frames, instead of four consecutive heavy
    /// frames that merge into one visible freeze.
    ///
    /// The cost of waiting is that the vehicle falls further before the plan is
    /// flyable - about 4 m/s of accumulated descent per second on Earth. That is
    /// absorbed rather than wasted, because every iteration re-anchors node 0 at the
    /// measured state, so the extra fall changes the problem slightly rather than
    /// invalidating the work.
    /// </summary>
    public double ColdIterationIntervalS { get; set; } = 0.25;

    /// <summary>
    /// SCS tolerance for the cold solve, looser than the flight one.
    ///
    /// A cold solve is looking for a STARTING POINT, and the warm loop refines it ten
    /// times a second afterwards, so solving each subproblem to flight accuracy is
    /// work spent on a trajectory that is about to be replaced. Measured on the log
    /// 20260807-103232 entry at 10 nodes: 1e-4 peaks at 20 ms per iteration, 3e-4 at
    /// 17 ms, 1e-3 at 15 ms.
    ///
    /// 3e-4 rather than 1e-3, and the reason is a floor rather than a preference: at 5
    /// nodes, 1e-3 does not converge at all - 40 iterations and 207 m of defect, while
    /// 3e-4 lands in 6 iterations with 0.59 m. A small problem has less redundancy to
    /// absorb a sloppy solve, and the node ladder is heading that way, so the tolerance
    /// has to be safe at the SMALL end rather than optimal at the large one.
    ///
    /// Restored to the flight tolerance the moment the plan hands over, because from
    /// then on the answer is flown rather than refined.
    /// </summary>
    public double ColdSubproblemEps { get; set; } = 3e-4;

    private double _lastColdIterationS;
    private int _coldIterations;
    private readonly System.Diagnostics.Stopwatch _wallClock = System.Diagnostics.Stopwatch.StartNew();

    /// <summary>
    /// How far the vehicle is from where its own plan says it should be, right now.
    ///
    /// This is the number that decides whether a WARM start is still worth having. A
    /// warm start is an advantage only while the vehicle is near its plan; past some
    /// drift the stale plan is a WORSE seed than a straight line from the current
    /// state, because the straight line is at least self-consistent while the stale
    /// one asserts a trajectory the vehicle is no longer on. At that point re-solving
    /// from it is paying to use a bad guess.
    /// </summary>
    public double PlanDriftM { get; private set; }

    /// <summary>Distance from the vehicle to the plan's prediction for right now.</summary>
    public double MeasureDrift(double[] x0, double simNow)
    {
        if (!HasPlan) return double.PositiveInfinity;
        double dt = _planSigma / (_n - 1);
        double t = Math.Max(0.0, simNow - _solveTime);
        double s = Math.Clamp(t / dt, 0.0, _n - 1.001);
        int k = (int)s;
        double f = s - k;
        // NOT Lerp() - that one strides by NU because every other caller passes
        // _planU. Using it on _planX reads element k*4+i, so from node 1 onward it
        // returns velocity and quaternion components as if they were positions. That
        // put ~6% of altitude of fictitious drift on a plan solved milliseconds ago,
        // which tripped the 5%-of-altitude restart limit on literally every plan.
        int k1 = Math.Min(k + 1, _n - 1);
        double d = 0.0;
        for (int i = 0; i < 3; i++)
        {
            double px = _planX[k * NX + i] * (1.0 - f) + _planX[k1 * NX + i] * f;
            d += (px - x0[i]) * (px - x0[i]);
        }
        PlanDriftM = Math.Sqrt(d);
        return PlanDriftM;
    }

    private double[] _xf = [];

    /// <summary>
    /// Seed this guidance from ANOTHER one's plan, resampled onto this node count.
    /// Used when the node schedule steps down: the node count is fixed at
    /// construction (it sizes the frozen sparsity pattern), so a change means a new
    /// solver, and a new solver would otherwise start cold.
    ///
    /// What survives the transition and what does not is the whole point:
    ///
    ///   LOST - the ADMM iterate. ScsWorkspace length-checks its stored x/y/s
    ///   against the new problem dimensions and discards them, so this cannot be
    ///   carried across however much we would like it to.
    ///
    ///   KEPT - the reference TRAJECTORY, which is the seed that actually matters.
    ///   It is just a time series, so it resamples by interpolation. It is what
    ///   keeps the cold solve feasible (see the trust-region note in Plan) and what
    ///   stops the plan jumping on the transition cycle.
    ///
    /// The transition solve is therefore ADMM-cold but trajectory-warm, and it
    /// happens at the SMALLER node count, where a solve is cheap anyway - measured
    /// p50 4 ms at N=30 against 13 ms at N=80.
    /// </summary>
    /// <summary>
    /// JUDGED AT THE WARM GATE, not the cold one, for the same reason StepCold is.
    /// A node step accepted at the loose cold tolerance commits a plan that the warm
    /// loop will then reject on every single cycle: flight log 20260807-101448
    /// stepped 50 -> 30 nodes with 1.13 m of defect, passed the 15 m cold gate, and
    /// every cycle afterwards read "no solve this cycle" against the 1 m warm gate
    /// while the vehicle flew an ageing open-loop plan.
    ///
    /// The coarser rung is an OPTIMISATION - it exists to make solves cheaper. If it
    /// cannot produce a plan the loop will actually fly, declining the step and
    /// staying at the finer count costs only solve time, which is the thing the step
    /// was trying to save. That is the right way round.
    /// </summary>
    public bool SeedFrom(Ksa6DofGuidance prev, double[] x0, double simNow, int maxIterations = 5)
    {
        if (prev == null || !prev.HasPlan)
            return false;

        var xs = new double[_n * NX];
        var us = new double[_n * NU];
        int src = prev._n;

        for (int k = 0; k < _n; k++)
        {
            // Same normalised time in both plans, so this is a pure resample of the
            // trajectory rather than a reinterpretation of it.
            double t = (double)k / (_n - 1) * (src - 1);
            int i0 = Math.Clamp((int)Math.Floor(t), 0, src - 1);
            int i1 = Math.Min(i0 + 1, src - 1);
            double a = t - i0;

            for (int i = 0; i < NX; i++)
                xs[k * NX + i] = prev._planX[i0 * NX + i] * (1.0 - a) + prev._planX[i1 * NX + i] * a;
            for (int j = 0; j < NU; j++)
                us[k * NU + j] = prev._planU[i0 * NU + j] * (1.0 - a) + prev._planU[i1 * NU + j] * a;

            // Componentwise interpolation does not preserve unit norm, and the
            // dynamics assume it does.
            Normalize(xs.AsSpan(k * NX + 6, 4));
        }

        // Node 0 is the measured state exactly, for the same reason as in Plan: the
        // equality and the trust region leave no slack there at all.
        Array.Copy(x0, 0, xs, 0, NX);

        _xf = (double[])prev._xf.Clone();
        double sigma = Math.Max(_cfg.SigmaMin, prev._planSigma - Math.Max(0.0, simNow - prev._solveTime));
        if (FixedTime) PinSigma(sigma);
        _solver.Initialize(x0, _xf, xs, us, sigma);
        return Finish(x0, simNow, maxIterations);
    }

    private static void Normalize(Span<double> q)
    {
        double m = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (m < 1e-12) { q[0] = 1.0; q[1] = q[2] = q[3] = 0.0; return; }
        for (int i = 0; i < 4; i++) q[i] /= m;
    }

    /// <summary>
    /// Constrain sigma to a single value. A hair of slack either side rather than an
    /// exact equality: SigmaMin == SigmaMax to the last bit is a knife-edge for the
    /// solver's feasibility check, and costs nothing to avoid.
    /// </summary>
    private void PinSigma(double sigma)
    {
        _cfg.SigmaMin = sigma * (1.0 - 1e-9);
        _cfg.SigmaMax = sigma * (1.0 + 1e-9);
    }

    /// <summary>
    /// Cold solve with a search over burn time, mirroring Gfold's bracket. Each sample
    /// warm-starts from the previous one rather than starting cold, so the sweep costs
    /// far less than N independent solves.
    ///
    /// Selection is by MERIT among samples that are actually PHYSICAL (defect within
    /// tolerance) — a shorter burn that only "wins" because it failed to converge is
    /// not a win.
    /// </summary>
    public bool PlanSearch(double[] x0, double[] xf, double sigmaGuess, double simNow,
                           int samples = 5, int iterationsPerSample = 20)
    {
        double lo = Math.Max(sigmaGuess * 0.6, 1.0);
        double hi = Math.Max(sigmaGuess * 1.8, lo + 1.0);

        double[] bestX = null, bestU = null;
        double bestSigma = 0.0, bestMerit = double.PositiveInfinity;
        bool seeded = false;
        SearchLog = "";

        for (int i = 0; i < samples; i++)
        {
            double sigma = samples == 1 ? sigmaGuess : lo + (hi - lo) * i / (samples - 1);
            PinSigma(sigma);

            if (!seeded)
            {
                if (!Plan(x0, xf, sigma, simNow, iterationsPerSample))
                {
                    SearchLog += $"{sigma:F1}s fail  ";
                    continue;
                }
                seeded = true;
            }
            else
            {
                _solver.Reseed(x0, _planX, _planU, sigma, trustRegion: _solver.TrustRegionMax);
                if (!Finish(x0, simNow, iterationsPerSample, cold: true))
                {
                    SearchLog += $"{sigma:F1}s fail  ";
                    continue;
                }
            }

            double merit = _solver.Cost;
            SearchLog += $"{sigma:F1}s J={merit:F4}  ";
            if (merit < bestMerit)
            {
                bestMerit = merit;
                bestSigma = sigma;
                bestX = (double[])_planX.Clone();
                bestU = (double[])_planU.Clone();
            }
        }

        if (bestX == null)
        {
            Error = "burn-time search found no physical plan: " + SearchLog;
            return false;
        }

        _planX = bestX;
        _planU = bestU;
        _planSigma = bestSigma;
        CommittedSigma = bestSigma;
        _solveTime = simNow;
        PinSigma(bestSigma);
        Error = "";
        return true;
    }

    /// <summary>What the burn-time search tried and what it cost, for the readout.</summary>
    public string SearchLog { get; private set; } = "";

    /// <summary>
    /// One MPC step: re-solve from the live state. The previous solution, shifted
    /// forward, seeds it — that is what keeps the warm start good and the solve at
    /// ~33 ms — but the ANSWER is anchored at the vehicle by the initial-state
    /// equality, not at the shifted seed.
    /// </summary>
    /// <param name="maxIterations">
    /// SCvx iterations per cycle. Measured in closed loop at N=50 with dispersion, a
    /// budget of 1 is indistinguishable from 5 in tracking (miss 1.8 vs 1.7 m, path
    /// 1.29 vs 1.28 x direct, plan jump 1.3 m for both) for a fifth of the work — the
    /// standard real-time iteration scheme. Held at 5 for now anyway: that measurement
    /// was taken against the harness dynamics, and the thrust-shortfall question is
    /// still open, so this is not the moment to also cut the loop's convergence
    /// margin. Drop it to 1 once the model is trusted.
    /// </param>
    public bool Update(double[] x0, double simNow, int maxIterations = 5)
    {
        if (!HasPlan)
            return false;

        // Bounded worst case from here on: this runs inside the guidance loop.
        ApplyTimeBudget(SubproblemBudgetMs, EscalatedBudgetMs);

        double dt = _planSigma / (_n - 1);
        double elapsed = Math.Max(0.0, simNow - _solveTime);
        int shift = Math.Clamp((int)Math.Round(elapsed / dt), 0, _n - 2);

        var xs = new double[_n * NX];
        var us = new double[_n * NU];
        for (int k = 0; k < _n; k++)
        {
            int src = Math.Min(k + shift, _n - 1);
            Array.Copy(_planX, src * NX, xs, k * NX, NX);
            Array.Copy(_planU, src * NU, us, k * NU, NU);
        }

        // Seed node 0 with the MEASURED state. It is what the equality will force
        // anyway, and starting the reference there means even a cycle that accepts no
        // step still hands back a plan anchored at the vehicle rather than at the old
        // plan's node `shift`.
        Array.Copy(x0, 0, xs, 0, NX);

        // FIXED TIME: count the COMMITTED burn time down rather than letting the
        // solver re-choose it every cycle. Re-choosing is what made successive plans
        // change shape instead of merely advancing, and it is what let the
        // regularisers stretch sigma to make themselves cheaper.
        double sigma;
        if (FixedTime)
        {
            sigma = Math.Max(_planSigma - elapsed, MinimumBurnTime);
            PinSigma(sigma);
        }
        else
        {
            sigma = Math.Max(_cfg.SigmaMin, _planSigma - elapsed);
        }
        FellBack = false;
        _solver.Reseed(x0, xs, us, sigma, trustRegion: 0.05);
        if (Finish(x0, simNow, maxIterations))
        {
            _consecutiveFailures = 0;
            return true;
        }

        // WIDE-TRUST-REGION RETRY, but NOT on every failure.
        //
        // It is the right response to a one-off divergence: the tight region above
        // assumes the vehicle is near its previous plan, and once it is not, the
        // problem is infeasible until the region opens up. But it costs a second
        // reseed and three times the iterations, so doing it on every failure makes a
        // REFUSAL more expensive than a SUCCESS - and once the loop starts failing
        // repeatedly that is precisely backwards. Flight log 20260807-093052 spent
        // 250-500 ms per cycle this way while accepting 6 re-solves in 103.
        //
        // If the previous cycle already tried it and still failed, it is not going to
        // work this cycle either: something structural is wrong, and the caller's
        // circuit breaker should cold-restart instead. So fail fast and cheap.
        if (_consecutiveFailures >= 2)
        {
            _consecutiveFailures++;
            FellBack = false;
            return false;
        }

        FellBack = true;
        _consecutiveFailures++;
        _solver.Reseed(x0, xs, us, sigma, trustRegion: _solver.TrustRegionMax);
        if (Finish(x0, simNow, maxIterations * 3))
        {
            _consecutiveFailures = 0;
            return true;
        }
        return false;
    }

    /// <summary>
    /// Convert the wall-clock budgets into iteration caps using the measured cost per
    /// ADMM iteration. Floors exist because a cap so low that nothing can converge
    /// just burns the budget failing; the escalation path is what covers a subproblem
    /// that genuinely needs more.
    /// </summary>
    private void ApplyTimeBudget(double budgetMs, double escalatedMs)
    {
        double perIter = Math.Max(MsPerAdmmIteration, 1e-4);

        // ONE CAP, NOT TWO. The two-stage scheme ran a subproblem to the small cap
        // and, if it truncated, ran the SAME subproblem again with the big one - so a
        // truncating solve paid for both. It was built for one-off truncation, but
        // measured in flight it fired on 77% of solves (213 escalations in 149
        // cycles), which means the small cap was not a budget, it was a tax.
        //
        // SCS stops at its tolerance, not at the cap, so a subproblem that converges
        // is completely unaffected by how high the cap is - raising it costs nothing
        // on the common path and saves the wasted first run on the uncommon one. What
        // bounds the cycle now is CycleBudgetMs, which bounds the thing that actually
        // matters.
        int cap = (int)Math.Clamp(Math.Max(budgetMs, escalatedMs) / perIter, 400, 100_000);
        _solver.MaxSubproblemIterations = cap;
        _solver.EscalatedSubproblemIterations = cap;   // equal, so no retry is issued
    }

    private bool Finish(double[] x0, double simNow, int maxIterations, bool cold = false)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        Status = _solver.Solve(maxIterations, CycleBudgetMs);
        LastSolveMs = sw.Elapsed.TotalMilliseconds;

        // Refine the cost-per-iteration estimate from what this solve actually did.
        // Exponential average so one outlier cannot swing the budget, but it still
        // tracks a change in node count or vehicle within a few cycles.
        int admm = 0;
        foreach (ScvxIteration it in _solver.Trace)
            admm += it.SolverIterations;
        if (admm > 50 && LastSolveMs > 0.0)
        {
            double sample = LastSolveMs / admm;
            MsPerAdmmIteration = 0.8 * MsPerAdmmIteration + 0.2 * sample;
        }
        LastIterations = _solver.IterationCount;
        SolveCount++;

        AcceptedSteps = 0;
        foreach (ScvxIteration it in _solver.Trace)
            if (it.Accepted)
                AcceptedSteps++;

        if (Status is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
        {
            Error = string.IsNullOrEmpty(_solver.LastFailureReason) ? Status.ToString() : _solver.LastFailureReason;
            return false;
        }

        // Worst defect over the trace — the LAST entry is the accepted reference's.
        // The CHANNEL comes from the same entry, deliberately: reading it from the
        // solver's latest evaluation instead would let the magnitude and its location
        // come from different iterations and quietly disagree.
        LastDefect = double.PositiveInfinity;
        LastDefectChannel = -1;
        LastDefectNode = -1;
        for (int i = _solver.Trace.Count - 1; i >= 0; i--)
            if (_solver.Trace[i].Accepted)
            {
                LastDefect = _solver.Trace[i].DefectNorm;
                LastDefectChannel = _solver.Trace[i].DefectChannel;
                LastDefectNode = _solver.Trace[i].DefectNode;
                break;
            }

        double[] xNew = _solver.ReferenceX;
        AnchorOffsetM = Math.Sqrt(
            (xNew[0] - x0[0]) * (xNew[0] - x0[0]) +
            (xNew[1] - x0[1]) * (xNew[1] - x0[1]) +
            (xNew[2] - x0[2]) * (xNew[2] - x0[2]));

        // Refuse a plan that is not anchored at the vehicle. Flying one means reading
        // the controls at the wrong point of the trajectory, which looks like the
        // vehicle ignoring its plan. One metre of slack for solver tolerance.
        if (AnchorOffsetM > 1.0)
        {
            Error = $"plan not anchored to the vehicle ({AnchorOffsetM:F1} m off) - " +
                    $"{AcceptedSteps} accepted step(s)";
            return false;
        }

        // REFUSE AN UNPHYSICAL PLAN. IterationLimit was previously accepted outright,
        // which shipped whatever the loop happened to have reached — including
        // trajectories still carrying large virtual control. Flying one means asking
        // the vehicle to reproduce motion that no force produced, so it saturates
        // thrust and falls further behind on every cycle. Better to keep the previous
        // plan and say so.
        // THE GATE IS IN METRES, NOT IN SCALED UNITS.
        //
        // DefectNorm is max|defect| / XScale, and XScale's position entries are L,
        // the range to the target — which is exactly the thing that shrinks on an
        // approach. A fixed scaled tolerance therefore means 1e-3 * L METRES, so the
        // gate silently tightens as the vehicle closes in: measured, it allows 1.07 m
        // of defect at 1 km, 0.26 m at 235 m and 0.04 m at 50 m.
        //
        // Nothing about the plan got worse. Measured at N=50 the ABSOLUTE defect is
        // flat down the whole descent (0.04, 0.01, 0.13, 0.07, 0.06 m); only the
        // ruler moved. Past ~100 m the solver was producing perfectly flyable
        // trajectories - centimetres of discrepancy over a 10 s plan - and having
        // them rejected for being centimetres off.
        //
        // That is why the trajectory "starts sensible and becomes a loop": a rejected
        // re-solve leaves _planX and _solveTime untouched, so the vehicle keeps
        // flying an ageing open-loop plan while the commands are read further and
        // further along it. The failure is the gate, not the guidance.
        //
        // Judged in metres this asks the question that actually matters — is the
        // trajectory flyable — and the answer no longer depends on how close the
        // target happens to be.
        LastDefectM = LastDefect * _cfg.XScale[Dynamics6Dof.IR];
        double gate = cold ? ColdMaxDefectM : MaxDefectM;
        if (!(LastDefectM <= gate))
        {
            Error = $"plan is not physical - dynamics defect {LastDefectM:F2} m " +
                    $"exceeds {gate:F2} m after {_solver.IterationCount} iters " +
                    $"({AcceptedSteps} accepted), worst on {LastDefectChannelName} " +
                    $"({LastDefectGroup}) at interval {LastDefectNode} = " +
                    $"{LastDefectRaw:G3} {LastDefectUnits}. " +
                    "Needs more iterations, or more nodes.";
            return false;
        }

        _planX = (double[])xNew.Clone();
        _planU = (double[])_solver.ReferenceU.Clone();
        _planSigma = _solver.Sigma;
        _solveTime = simNow;
        Error = "";
        return true;
    }

    /// <summary>
    /// The optimiser's control at this instant: body torque (N·m, MODEL body axes)
    /// and throttle in [0,1], interpolated along the current plan.
    ///
    /// Read at (now - solveTime), so immediately after a solve this is node 0's
    /// control — the control the optimiser chose FOR THE VEHICLE'S ACTUAL STATE.
    /// </summary>
    public bool Command(double simNow, out double3 torqueModel, out double thrustN)
    {
        torqueModel = default;
        thrustN = 0.0;
        if (!HasPlan)
            return false;

        double dt = _planSigma / (_n - 1);
        double t = Math.Max(0.0, simNow - _solveTime);
        PlanElapsed = t;

        double sNode = Math.Clamp(t / dt, 0.0, _n - 1.001);
        int k = (int)sNode;
        double f = sNode - k;

        double tdx = Lerp(_planU, 0, k, f);
        double tdy = Lerp(_planU, 1, k, f);
        double thrust = Lerp(_planU, 2, k, f);
        double tauRoll = Lerp(_planU, 3, k, f);

        // tau = r_T x T_body with r_T = (0,0,-LArm), i.e. the engine below the centre
        // of mass — the model's own gimbal-torque relation, verbatim.
        torqueModel = new double3(_dyn.LArm * tdy, -_dyn.LArm * tdx, tauRoll);
        LastLateralForce = new double2(tdx, tdy);

        // AXIAL THRUST IN NEWTONS, not a throttle fraction.
        //
        // This used to return thrust / _cfg.Tmax, and that was the systematic error
        // behind the descend-until-it-loops behaviour. Tmax is fixed when the plan is
        // built, so the moment the vehicle's real capability differs from it — a
        // different ambient pressure, an engine out, propellant starvation — every
        // commanded thrust is wrong by exactly that ratio. It is invisible to the
        // MPC too: re-solving corrects the STATE, but the error is in the actuator
        // mapping, so each new plan is executed just as wrongly as the last.
        //
        // Newtons is the honest unit for the optimiser to speak in. Converting to a
        // throttle needs the vehicle's LIVE capability, which only the caller can
        // measure, so that conversion belongs at the KSA boundary and not here.
        // TOTAL thrust magnitude, not just the axial component.
        //
        // The model's control is a VECTOR, u = (tdx, tdy, T, tau_roll): T is the
        // AXIAL thrust and tdx/tdy are the lateral components the gimbal produces.
        // KSA's throttle sets the total thrust magnitude along the gimballed nozzle,
        // so commanding T alone delivers a vector of length T, whose axial part is
        // only T*cos(deflection) - short by exactly the cosine loss.
        //
        // Commanding |u| instead makes the axial part |u| * T/|u| = T and the
        // lateral parts tdx, tdy: precisely the force vector the optimiser chose.
        //
        // Measured on a real descent this is worth almost nothing - the deflections
        // were 0.1 to 2.2 degrees, so cos is 0.999 - and it is NOT the explanation
        // for the 10.3% thrust shortfall seen there. It is fixed because it is
        // wrong, not because it is large.
        thrustN = Math.Sqrt(thrust * thrust + tdx * tdx + tdy * tdy);
        thrustN = Math.Max(thrustN, 0.0);
        return true;
    }

    /// <summary>
    /// Shortest-arc quaternion interpolation, scalar-first. Sign-corrected first: q
    /// and -q are the same rotation, so without that the interpolation can take the
    /// long way round through zero and produce a degenerate mid-path attitude.
    /// Falls back to normalised lerp when the endpoints are nearly parallel, where
    /// the slerp formula divides by ~0.
    /// </summary>
    private static void Slerp(ReadOnlySpan<double> a, ReadOnlySpan<double> b, double t, Span<double> outQ)
    {
        double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        double sign = dot < 0.0 ? -1.0 : 1.0;
        dot = Math.Abs(dot);

        double wa, wb;
        if (dot > 0.9995)
        {
            wa = 1.0 - t;
            wb = t;
        }
        else
        {
            double theta = Math.Acos(Math.Clamp(dot, -1.0, 1.0));
            double s = Math.Sin(theta);
            wa = Math.Sin((1.0 - t) * theta) / s;
            wb = Math.Sin(t * theta) / s;
        }

        double n = 0.0;
        for (int i = 0; i < 4; i++)
        {
            outQ[i] = wa * a[i] + wb * sign * b[i];
            n += outQ[i] * outQ[i];
        }
        n = Math.Sqrt(n);
        if (n < 1e-12)
        {
            outQ[0] = 1.0; outQ[1] = outQ[2] = outQ[3] = 0.0;
            return;
        }
        for (int i = 0; i < 4; i++)
            outQ[i] /= n;
    }

    private double Lerp(double[] a, int off, int k, double f)
    {
        int k1 = Math.Min(k + 1, _n - 1);
        return a[k * NU + off] * (1 - f) + a[k1 * NU + off] * f;
    }

    /// <summary>
    /// Objective breakdown at the current plan. Fuel SHOULD dominate — if a
    /// regulariser is comparable to or larger than it, the optimiser is no longer
    /// solving min-fuel, and because both regularisers get cheaper as sigma grows
    /// the visible symptom is burn time pinned at its upper bound.
    /// </summary>
    public void ObjectiveTerms(out double fuel, out double controlSmoothing, out double rateDamping)
    {
        fuel = controlSmoothing = rateDamping = 0.0;
        if (!HasPlan)
            return;

        double m0 = _planX[13];
        fuel = (m0 - _planX[(_n - 1) * NX + 13]) / Math.Max(m0, 1.0);

        double[] us = _cfg.ResolvedUScale;
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                double d = (_planU[(k + 1) * NU + j] - _planU[k * NU + j]) / us[j];
                controlSmoothing += _cfg.WDu * d * d;
            }

        for (int k = 0; k < _n; k++)
            for (int i = 0; i < 3; i++)
            {
                double w = _planX[k * NX + 10 + i];
                rateDamping += _cfg.WW * w * w;
            }
    }

    /// <summary>
    /// The LATERAL thrust the model believes it is producing, (tdx, tdy) in model
    /// body axes, N. The model rigidly couples this to pitch/yaw torque through a
    /// SINGLE engine at LArm: tau = r_T x T_body. The real vehicle does not — the
    /// allocator makes the requested torque using every gimbal it has, including
    /// verniers, and its lateral force is whatever that geometry gives.
    ///
    /// If the two disagree, the plan's TRANSLATIONAL dynamics are wrong even though
    /// attitude tracks: the vehicle gets a different sideways push than planned.
    /// Compare against KsaGimbalControl.LastAllocation.AchievedForce.
    /// </summary>
    public double2 LastLateralForce { get; private set; }

    /// <summary>Plan node 0 in model coordinates — where the plan believes the vehicle is.</summary>
    public double3 PlanOrigin => HasPlan
        ? new double3(_planX[0], _planX[1], _planX[2])
        : default;

    /// <summary>How far the vehicle has drifted from the plan it is flying. Pure diagnostics — nothing acts on it.</summary>
    public void Diagnostics(double[] x, out double posErrM, out double velErrMs, out double attErrDeg)
    {
        posErrM = velErrMs = attErrDeg = 0.0;
        if (!HasPlan)
            return;

        double dt = _planSigma / (_n - 1);
        double sNode = Math.Clamp(PlanElapsed / dt, 0.0, _n - 1.001);
        int k = (int)sNode;
        double f = sNode - k;
        int k1 = Math.Min(k + 1, _n - 1);

        double Sx(int off) => _planX[k * NX + off] * (1 - f) + _planX[k1 * NX + off] * f;

        posErrM = Math.Sqrt(
            (Sx(0) - x[0]) * (Sx(0) - x[0]) +
            (Sx(1) - x[1]) * (Sx(1) - x[1]) +
            (Sx(2) - x[2]) * (Sx(2) - x[2]));
        velErrMs = Math.Sqrt(
            (Sx(3) - x[3]) * (Sx(3) - x[3]) +
            (Sx(4) - x[4]) * (Sx(4) - x[4]) +
            (Sx(5) - x[5]) * (Sx(5) - x[5]));

        double pw = Sx(6), pxq = Sx(7), pyq = Sx(8), pzq = Sx(9);
        double nrm = Math.Sqrt(pw * pw + pxq * pxq + pyq * pyq + pzq * pzq);
        if (nrm < 1e-12)
            return;
        KsaFrameBridge.QuatToMatrix(pw / nrm, pxq / nrm, pyq / nrm, pzq / nrm, out _, out _, out double3 planZ);
        KsaFrameBridge.QuatToMatrix(x[6], x[7], x[8], x[9], out _, out _, out double3 curZ);
        double d = Math.Clamp(double3.Dot(double3.Normalize(planZ), double3.Normalize(curZ)), -1.0, 1.0);
        attErrDeg = Math.Acos(d) * 180.0 / Math.PI;
    }
}
