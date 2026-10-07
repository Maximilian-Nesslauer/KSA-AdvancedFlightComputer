using AdvancedFlightComputer.Guidance.Scvx.SixDof;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

public enum Phase3Dof
{
    /// <summary>Nothing planned yet.</summary>
    Idle,
    /// <summary>The cold glide-and-burn solve is running; the caller flies its own glide meanwhile.</summary>
    Converging,
    /// <summary>Glide and burn, ignition time free: the plan decides when to light.</summary>
    Glide,
    /// <summary>Ignition committed, engine still off: the burn-only problem plans from the state predicted at ignition.</summary>
    Locked,
    /// <summary>Engine lit, burn-only problem from the measured state.</summary>
    Burn,
}

/// <summary>
/// When the solver's units are sized to the problem. See <see cref="Guidance3Dof.Settings.Scaling"/>.
/// </summary>
public enum ScalingMode
{
    /// <summary>Never: the config's scales throughout, as 3dof.py.</summary>
    Fixed,
    /// <summary>When a solver starts - the two-phase one at the cold solve, the burn-only one at the lock - as the 6-DOF sizes at engage and at each node-ladder rebuild.</summary>
    AtLock,
    /// <summary>Before every solve, from what is left.</summary>
    EverySolve,
}

/// <summary>
/// One published plan: immutable, so the sim thread can read it while the next one is being solved.
///
/// Node 0 sits at <see cref="StartTime"/>, which is the measurement time for every plan but the burn plans made while ignition is locked - those start at the committed ignition, in the future, from the state predicted there.
/// </summary>
public sealed record Plan3Dof(
    double[] X, double[] U, int Nodes, int GlideIntervals,
    double SigmaGlide, double SigmaBurn, double StartTime, double SolveTime,
    Phase3Dof Phase, double GatedDefect)
{
    public double IgnitionTime => StartTime + SigmaGlide;
    public double EndTime => StartTime + SigmaGlide + SigmaBurn;

    /// <summary>State and control at sim time <paramref name="simTime"/>, clamped to the plan's span.</summary>
    public void Sample(double simTime, Span<double> x, Span<double> u)
        => Resample3Dof.Sample(X, U, Nodes, GlideIntervals, SigmaGlide, SigmaBurn, simTime - StartTime, x, u);
}

/// <summary>
/// The receding-horizon glide-and-burn guidance, independent of the game so it can be flown closed loop offline.
///
/// THREE PHASES, AND THE IGNITION BETWEEN THEM.
///
/// GLIDE. Each cycle re-solves the two-phase problem from the measured state, both durations free, seeded with the last plan resampled onto the new horizon. The glide's duration is the time to ignition, and it is the optimiser's to choose.
///
/// LOCK. When the time to ignition falls to <see cref="Settings.LockSeconds"/>, the ignition time is COMMITTED and the glide phase is dropped - not at ignition, a few seconds before it, so the change of problem never coincides with lighting the engine. From then on the burn-only problem (the same subproblem class with no glide intervals) plans from the state PREDICTED at the committed ignition: the measured state carried through the remaining seconds of glide, every cycle, with the vehicle turning towards the burn's opening attitude. That is still closed loop - a dispersion before ignition shows up in the prediction and the burn plans for it - but the ignition time stops moving, which is the one decision not worth re-making against a model whose bias the solver cannot null: re-solving it into the last seconds would chase a target that moves every time it looks. Node 0's body axis is held within what the vehicle can turn to by ignition, a cone that closes onto the measured attitude as ignition arrives.
///
/// BURN. At ignition the prediction horizon is zero, so node 0 is simply the measured state and the cone has closed: nothing switches. The burn is re-solved from the measured state each cycle with the full throttle released.
///
/// The swap is cheap because the burn-only problem is seeded with the burn the two-phase plan already holds; offline it takes four iterations. Until it has a plan of its own, the two-phase plan's burn, re-timed to the committed ignition, is published in its place, so there is never a moment without one.
///
/// A PLAN IS PUBLISHED ONLY IF IT IS FLYABLE: its true dynamics defect, weighted by how far ahead each interval is, must be inside the per-channel tolerances, as the 6-DOF gate does. A refused cycle keeps the previous plan.
/// </summary>
public sealed class Guidance3Dof
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    public sealed record Settings
    {
        /// <summary>Time to ignition at which it is committed and the glide phase dropped, s.</summary>
        public double LockSeconds { get; init; } = 4.0;

        /// <summary>Throttle the plan may use before ignition is committed, and after. The difference is the headroom a committed ignition needs to absorb arriving late.</summary>
        public double GlideThrottleCeiling { get; init; } = 0.9;
        public double BurnThrottleCeiling { get; init; } = 1.0;

        public int ColdIterations { get; init; } = 150;

        /// <summary>
        /// ADMM iterations per subproblem, cold and warm, and the one escalated retry of a subproblem that only ran out of them. A bounded worst case is what keeps a solve from stalling for seconds; a truncated solve counts as a failure, never as a step. See Ksa6DofGuidance.ColdAdmmCap.
        /// </summary>
        public int ColdAdmmCap { get; init; } = 20_000;
        public int WarmAdmmCap { get; init; } = 2_000;
        public int EscalatedAdmmCap { get; init; } = 8_000;
        public int WarmIterations { get; init; } = 3;
        public int SwapIterations { get; init; } = 12;

        /// <summary>Wall-clock budget per warm cycle, and for a cycle after a refusal, ms. See Ksa6DofGuidance.CycleBudgetMs.</summary>
        public double CycleBudgetMs { get; init; } = 60.0;
        public double RecoveryBudgetMs { get; init; } = 200.0;

        /// <summary>Trust region a warm cycle starts from.</summary>
        public double WarmTrustRegion { get; init; } = 0.05;

        /// <summary>Seconds of turning node 0's body axis is allowed from the measured attitude, at the rate limit, outside the lock.</summary>
        public double AnchorSeconds { get; init; } = 1.0;

        /// <summary>Smallest anchor cone, radians: the attitude loop is never exactly on its command.</summary>
        public double AnchorFloor { get; init; } = 2.0 * Math.PI / 180;

        /// <summary>Floor for the counted-down burn, s.</summary>
        public double MinimumBurnTime { get; init; } = 0.25;

        /// <summary>Per-channel defect tolerance for publishing, SI: m, m/s, kg.</summary>
        public double[] DefectTolerance { get; init; } = [2, 2, 2, 0.5, 0.5, 0.5, 5];

        /// <summary>Seconds of plan judged at full strength, and the allowance at its far end. See Scvx6DofSolver.WeightedDefect.</summary>
        public double CommitHorizonS { get; init; } = 2.0;
        public double HorizonFarSlack { get; init; } = 25.0;

        /// <summary>
        /// When the solver's units are resized to the problem left - see <see cref="Rescale"/>. Under test: the three modes are compared by --3dof-mpc --scaling all.
        /// </summary>
        public ScalingMode Scaling { get; init; } = ScalingMode.EverySolve;

        /// <summary>Smallest length scale, m, so the last metres of a burn are not divided by nearly nothing.</summary>
        public double MinLengthScale { get; init; } = 30.0;

        /// <summary>
        /// Bounds on the duration scale, s. It is set ONCE, at the cold solve, from the seed's burn time - not resized with the time left, see <see cref="Rescale"/>.
        /// </summary>
        public double MinSigmaScale { get; init; } = 5.0;
        public double MaxSigmaScale { get; init; } = 30.0;

        /// <summary>Integration step for the ignition prediction, s.</summary>
        public double PredictionStep { get; init; } = 0.05;
    }

    private readonly Settings _set;
    private readonly Scvx3DofSolver _two, _burn;
    private double[] _xf = new double[6];
    private bool _burnStarted;
    private volatile Plan3Dof? _published;

    public Phase3Dof Phase { get; private set; } = Phase3Dof.Idle;
    public Plan3Dof? Published => _published;
    public double CommittedIgnition { get; private set; } = double.NaN;
    public string Error { get; private set; } = "";
    public int ConsecutiveRefusals { get; private set; }
    public int LastIterations { get; private set; }
    public double LastSolveMs { get; private set; }

    /// <summary>ADMM iterations the last solve spent, over all its SCvx iterations.</summary>
    public int LastAdmmIterations { get; private set; }
    public double LastGatedDefect { get; private set; } = double.NaN;
    public string LastGatedChannel { get; private set; } = "-";
    public ScvxStatus LastStatus { get; private set; }

    /// <summary>The model the next solve uses; swap it between cycles to hand in corrected aerodynamics.</summary>
    public KsaPointMassModel Model { get; set; }

    public Scvx3DofConfig GlideConfig => _two.Config;
    public Scvx3DofConfig BurnConfig => _burn.Config;
    public Settings Options => _set;

    public Guidance3Dof(Scvx3DofConfig glideConfig, Scvx3DofConfig burnConfig, KsaPointMassModel model, Settings? settings = null)
    {
        if (!glideConfig.HasGlide) throw new ArgumentException("the glide config needs glide intervals");
        if (burnConfig.HasGlide) throw new ArgumentException("the burn config must have no glide intervals");
        if (!burnConfig.AttitudeAnchor) throw new ArgumentException("the burn config must allocate the attitude anchor");
        _set = settings ?? new Settings();
        Model = model;
        _two = new Scvx3DofSolver(glideConfig, model) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        _burn = new Scvx3DofSolver(burnConfig, model) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        SetAdmmCaps(_set.WarmAdmmCap);
    }

    // ------------------------------------------------------------------- cold

    /// <summary>
    /// Seed and start the cold glide-and-burn solve from <paramref name="x0"/>, measured at <paramref name="now"/>. Run it with <see cref="StepCold"/>, as many iterations a call as the caller can spare.
    /// </summary>
    public bool BeginCold(double[] x0, double[] xf, double now, Seed3Dof.Options seed)
    {
        _xf = (double[])xf.Clone();
        _two.Model = Model;
        _two.Config.ThrottleCeiling = _set.GlideThrottleCeiling;
        _two.SetAttitudeAnchor([0, 0, 0], Math.PI);
        _xf = (double[])xf.Clone();
        if (!Seed3Dof.TryBuild(Model, _two.Config, x0, xf, seed, out double[] xs, out double[] us,
                               out double sg, out double sb))
        {
            Error = "no seed";
            return false;
        }
        if (_set.Scaling != ScalingMode.Fixed)
        {
            Rescale(_two.Config, x0);
            // The duration scale, once, for both solvers: the seed's burn time, as the 6-DOF sizes its own from the burn time at engage.
            _two.Config.SigmaScale = _burn.Config.SigmaScale = Math.Clamp(sb, _set.MinSigmaScale, _set.MaxSigmaScale);
        }
        _two.Initialize(x0, xf, xs, us, sg, sb);
        _two.MaxSubproblemIterations = _two.EscalatedSubproblemIterations = _set.ColdAdmmCap;
        _coldStart = now;
        _coldIterations = 0;
        _burnStarted = false;
        CommittedIgnition = double.NaN;
        ConsecutiveRefusals = 0;
        _published = null;
        Phase = Phase3Dof.Converging;
        Error = "converging";
        return true;
    }

    private double _coldStart;
    private int _coldIterations;

    /// <summary>
    /// Advance the cold solve by up to <paramref name="iterations"/> SCvx iterations. Publishes and moves to the glide on convergence; false while still converging, and once it has given up (see <see cref="Error"/>).
    /// </summary>
    public bool StepCold(int iterations)
    {
        if (Phase != Phase3Dof.Converging) return Phase == Phase3Dof.Glide;
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus st = _two.Solve(iterations);
        _coldIterations += _two.Trace.Count == 0 ? 0 : iterations;
        LastSolveMs = sw.Elapsed.TotalMilliseconds;
        LastIterations = _two.IterationCount;
        LastStatus = st;
        if (st == ScvxStatus.Converged && Gate(_two, out double gated))
        {
            SetAdmmCaps(_set.WarmAdmmCap);
            Publish(_two, _coldStart, _coldStart, Phase3Dof.Glide, gated);
            Phase = Phase3Dof.Glide;
            Error = "";
            return true;
        }
        if (st is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed || _two.IterationCount >= _set.ColdIterations)
        {
            Error = $"cold solve gave up after {_two.IterationCount} iterations ({st}, defect gate {LastGatedDefect:F1}x on {LastGatedChannel})";
            Phase = Phase3Dof.Idle;
        }
        return false;
    }

    // ------------------------------------------------------------------- warm

    /// <summary>
    /// One receding-horizon cycle from the measured state. <paramref name="attitude"/> is the measured thrust axis in the model frame. Moves through the lock and into the burn as the committed times arrive; true if a new plan was published.
    /// </summary>
    public bool Update(double[] x0, double[] attitude, double now)
    {
        Plan3Dof? plan = _published;
        if (plan == null) return false;

        return Phase switch
        {
            Phase3Dof.Glide => plan.IgnitionTime - now <= _set.LockSeconds
                ? Lock(plan, x0, attitude, now)
                : UpdateGlide(plan, x0, attitude, now),
            Phase3Dof.Locked => now >= CommittedIgnition
                ? StartBurn(x0, attitude, now)
                : UpdateLocked(plan, x0, attitude, now),
            Phase3Dof.Burn => UpdateBurn(plan, x0, attitude, now),
            _ => false,
        };
    }

    private bool UpdateGlide(Plan3Dof plan, double[] x0, double[] attitude, double now)
    {
        double elapsed = Math.Max(0.0, now - plan.StartTime);
        double sigG = plan.SigmaGlide - elapsed;
        double sigB = plan.SigmaBurn;
        Scvx3DofConfig cfg = _two.Config;
        var xs = new double[cfg.Nodes * NX];
        var us = new double[cfg.Nodes * NU];
        Resample3Dof.Resample(plan.X, plan.U, plan.Nodes, plan.GlideIntervals, plan.SigmaGlide, plan.SigmaBurn, elapsed,
                              cfg.Nodes, cfg.GlideIntervals, sigG, sigB, cfg.ThrottleFloor, xs, us);
        Array.Copy(x0, xs, NX);
        OpenOn(us, attitude);

        _two.Model = Model;
        cfg.ThrottleCeiling = _set.GlideThrottleCeiling;
        _two.SetAttitudeAnchor(attitude, AnchorCone(_set.AnchorSeconds));
        return Solve(_two, x0, xs, us, sigG, sigB, now, now, Phase3Dof.Glide, _set.WarmIterations, reseed: true);
    }

    /// <summary>
    /// Commit the ignition and drop the glide: publish the two-phase plan's burn re-timed to the committed ignition, then solve the burn-only problem from the predicted ignition state.
    /// </summary>
    private bool Lock(Plan3Dof plan, double[] x0, double[] attitude, double now)
    {
        CommittedIgnition = Math.Max(plan.IgnitionTime, now);
        Scvx3DofConfig cfg = _burn.Config;
        var xs = new double[cfg.Nodes * NX];
        var us = new double[cfg.Nodes * NU];
        Resample3Dof.Resample(plan.X, plan.U, plan.Nodes, plan.GlideIntervals, plan.SigmaGlide, plan.SigmaBurn, plan.SigmaGlide,
                              cfg.Nodes, 0, 0.0, plan.SigmaBurn, cfg.ThrottleFloor, xs, us);
        // The stand-in: the plan's own burn, starting at the committed ignition.
        _published = new Plan3Dof((double[])xs.Clone(), (double[])us.Clone(), cfg.Nodes, 0, 0.0, plan.SigmaBurn,
                                  CommittedIgnition, now, Phase3Dof.Locked, plan.GatedDefect);
        Phase = Phase3Dof.Locked;
        _burnStarted = false;
        return UpdateLocked(_published, x0, attitude, now);
    }

    private bool UpdateLocked(Plan3Dof plan, double[] x0, double[] attitude, double now)
    {
        Scvx3DofConfig cfg = _burn.Config;
        double toGo = Math.Max(0.0, CommittedIgnition - now);
        var xs = (double[])plan.X.Clone();
        var us = (double[])plan.U.Clone();
        double[] opening = plan.U.AsSpan(0, 3).ToArray();
        double[] predicted = PredictIgnition(x0, attitude, opening, toGo);
        Array.Copy(predicted, xs, NX);

        _burn.Model = Model;
        cfg.ThrottleCeiling = _set.BurnThrottleCeiling;
        // What the vehicle can turn to by ignition, closing onto the measured attitude as ignition arrives.
        _burn.SetAttitudeAnchor(attitude, Math.Max(cfg.AttitudeRateMax * toGo, _set.AnchorFloor));
        bool first = !_burnStarted;
        _burnStarted = true;
        return Solve(_burn, predicted, xs, us, 0.0, plan.SigmaBurn, CommittedIgnition, now, Phase3Dof.Locked,
                     first ? _set.SwapIterations : _set.WarmIterations, reseed: !first);
    }

    private bool StartBurn(double[] x0, double[] attitude, double now)
    {
        Phase = Phase3Dof.Burn;
        return UpdateBurn(_published!, x0, attitude, now);
    }

    private bool UpdateBurn(Plan3Dof plan, double[] x0, double[] attitude, double now)
    {
        Scvx3DofConfig cfg = _burn.Config;
        double elapsed = Math.Max(0.0, now - plan.StartTime);
        double sigB = Math.Max(plan.SigmaBurn - elapsed, _set.MinimumBurnTime);
        var xs = new double[cfg.Nodes * NX];
        var us = new double[cfg.Nodes * NU];
        Resample3Dof.Resample(plan.X, plan.U, plan.Nodes, plan.GlideIntervals, plan.SigmaGlide, plan.SigmaBurn, elapsed,
                              cfg.Nodes, 0, 0.0, sigB, cfg.ThrottleFloor, xs, us);
        Array.Copy(x0, xs, NX);
        OpenOn(us, attitude);

        _burn.Model = Model;
        cfg.ThrottleCeiling = _set.BurnThrottleCeiling;
        // A burn that runs short of its floor keeps that floor rather than asking the bounds for a burn shorter than they allow.
        cfg.BurnSigmaMin = Math.Min(cfg.BurnSigmaMin, _set.MinimumBurnTime);
        _burn.SetAttitudeAnchor(attitude, AnchorCone(_set.AnchorSeconds));
        bool reseed = _burnStarted;
        _burnStarted = true;
        return Solve(_burn, x0, xs, us, 0.0, sigB, now, now, Phase3Dof.Burn, _set.WarmIterations, reseed);
    }

    /// <summary>
    /// Seed node 0's body axis with the anchor - the attitude last commanded - so the reference already satisfies the anchor and the plan opens where the command is. The resampled plan's own opening attitude is a cycle stale.
    /// </summary>
    private static void OpenOn(double[] us, double[] attitude)
    {
        double len = Math.Sqrt(attitude[0] * attitude[0] + attitude[1] * attitude[1] + attitude[2] * attitude[2]);
        if (!(len > 0.5)) return;
        for (int j = 0; j < 3; j++) us[PointMass3Dof.IB + j] = attitude[j] / len;
    }

    private bool Solve(Scvx3DofSolver solver, double[] x0, double[] xs, double[] us, double sigG, double sigB,
                       double startTime, double now, Phase3Dof phase, int iterations, bool reseed)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        // Every solve, or only when this solver starts, which for the burn is the lock.
        if (_set.Scaling == ScalingMode.EverySolve || (_set.Scaling == ScalingMode.AtLock && !reseed))
            Rescale(solver.Config, x0);
        double tr = ConsecutiveRefusals > 0 ? solver.TrustRegionMax : _set.WarmTrustRegion;
        if (reseed)
        {
            solver.SetTerminal(_xf);
            solver.Reseed(x0, xs, us, sigG, sigB, tr);
        }
        else
            solver.Initialize(x0, _xf, xs, us, sigG, sigB, tr);

        double budget = ConsecutiveRefusals > 0 ? _set.RecoveryBudgetMs : _set.CycleBudgetMs;
        LastStatus = solver.Solve(iterations, budget);
        LastSolveMs = sw.Elapsed.TotalMilliseconds;
        LastIterations = solver.IterationCount;
        LastAdmmIterations = solver.Trace.Sum(t => t.SolverIterations);

        if (!solver.Trace.Any(t => t.Solved) || !Gate(solver, out double gated))
        {
            ConsecutiveRefusals++;
            Error = solver.Trace.Any(t => t.Solved)
                ? $"plan refused: defect {LastGatedDefect:F1}x tolerance on {LastGatedChannel}"
                : "plan refused: " + solver.LastFailureReason;
            return false;
        }
        Publish(solver, startTime, now, phase, gated);
        ConsecutiveRefusals = 0;
        Error = "";
        return true;
    }

    /// <summary>
    /// Size the solver's units to the problem left, before every solve (see <see cref="ScalingMode"/>): a length scale per axis, a speed scale and the present mass.
    ///
    /// NOT THE DURATION SCALE. It also sizes the trust region on the two durations - each may move tr x SigmaScale per iteration - and sized from the time left it was 80-90 s through the glide: four seconds of ignition time per iteration, three iterations a cycle, on a cost that barely cares when the engine lights. Every re-solve published a different timeline, the ignition time wandered two seconds either way, and the glide attitude wandered with it. It is set once at the cold solve instead, from the burn time, as the 6-DOF's is: a fraction of a second per iteration, so the timeline moves only when the problem moves it.
    ///
    /// PER AXIS, NOT ISOTROPIC as the 6-DOF's. A glide is far taller than it is wide - 30 km of height against a few of range - and one length for all three axes sized by the height left the cross-range to resolve in 30 km units: offline, a 2 km cross-range cold solve that converges on per-axis scales stopped converging. The horizontal scale is the horizontal range to the aim point, floored at a tenth of the height, since the path can swing wider than the straight line; the vertical scale is the height.
    /// </summary>
    private void Rescale(Scvx3DofConfig cfg, double[] x0)
    {
        double dx = x0[0] - _xf[0], dy = x0[1] - _xf[1], dz = x0[2] - _xf[2];
        double height = Math.Max(Math.Abs(dz), _set.MinLengthScale);
        double across = Math.Max(Math.Max(Math.Sqrt(dx * dx + dy * dy), 0.1 * height), _set.MinLengthScale);
        double g = Model.Mu / Math.Pow(Model.MeanRadius + Model.Altitude(x0), 2);
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocity = Math.Max(Math.Max(speed, Math.Sqrt(height * g)), 1.0);
        cfg.XScale = [across, across, height, velocity, velocity, velocity, Math.Max(x0[PointMass3Dof.IM], 1.0)];
    }

    private bool Gate(Scvx3DofSolver solver, out double gated)
    {
        int commit = 0;
        while (commit < solver.Nodes - 1 && solver.NodeTime(commit + 1) <= _set.CommitHorizonS) commit++;
        Span<double> tol = stackalloc double[NX];
        for (int i = 0; i < NX; i++) tol[i] = _set.DefectTolerance[i];
        gated = solver.WeightedDefect(tol, commit, _set.HorizonFarSlack, out int ch, out _, out _);
        LastGatedDefect = gated;
        LastGatedChannel = Scvx3DofSolver.ChannelName(ch);
        return gated <= 1.0;
    }

    private void Publish(Scvx3DofSolver solver, double startTime, double now, Phase3Dof phase, double gated)
    {
        _published = new Plan3Dof((double[])solver.ReferenceX.Clone(), (double[])solver.ReferenceU.Clone(),
                                  solver.Nodes, solver.GlideIntervals, solver.SigmaGlide, solver.SigmaBurn,
                                  startTime, now, phase, gated);
    }

    private void SetAdmmCaps(int cap)
    {
        foreach (Scvx3DofSolver solver in new[] { _two, _burn })
        {
            solver.MaxSubproblemIterations = cap;
            solver.EscalatedSubproblemIterations = Math.Max(cap, _set.EscalatedAdmmCap);
        }
    }

    private double AnchorCone(double seconds)
        => Math.Max(_two.Config.AttitudeRateMax > 0.0 ? _two.Config.AttitudeRateMax * seconds : Math.PI, _set.AnchorFloor);

    // ------------------------------------------------------------- prediction

    /// <summary>
    /// The state at ignition: the measured state carried engine-off through the seconds left, with the body axis turning from the measured attitude towards the burn's opening attitude at the rate limit - which is what the vehicle is commanded to do in that time.
    /// </summary>
    public double[] PredictIgnition(double[] x0, double[] attitude, double[] opening, double seconds)
    {
        double[] x = (double[])x0.Clone();
        if (!(seconds > 0.0)) return x;
        double rate = _burn.Config.AttitudeRateMax > 0.0 ? _burn.Config.AttitudeRateMax : 10.0;
        int steps = Math.Max(1, (int)Math.Ceiling(seconds / _set.PredictionStep));
        double h = seconds / steps;
        double[] b = Normalise(attitude);
        double[] target = Normalise(opening);
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX];
        double[] tmp = new double[NX];
        for (int s = 0; s < steps; s++)
        {
            double[] u = [b[0], b[1], b[2], 0.0];
            PointMass3Dof.Eval(Model, x, u, k1);
            for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
            PointMass3Dof.Eval(Model, tmp, u, k2);
            for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
            PointMass3Dof.Eval(Model, tmp, u, k3);
            for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
            PointMass3Dof.Eval(Model, tmp, u, k4);
            for (int i = 0; i < NX; i++) x[i] += h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
            b = Slew(b, target, rate * h);
        }
        return x;
    }

    /// <summary>Turn <paramref name="from"/> towards <paramref name="to"/> by at most <paramref name="maxAngle"/>.</summary>
    public static double[] Slew(double[] from, double[] to, double maxAngle)
    {
        double cos = Math.Clamp(from[0] * to[0] + from[1] * to[1] + from[2] * to[2], -1.0, 1.0);
        double angle = Math.Acos(cos);
        if (angle <= maxAngle || angle < 1e-9) return (double[])to.Clone();
        double s = Math.Sin(angle);
        double wa = Math.Sin(angle - maxAngle) / s, wb = Math.Sin(maxAngle) / s;
        return Normalise([wa * from[0] + wb * to[0], wa * from[1] + wb * to[1], wa * from[2] + wb * to[2]]);
    }

    private static double[] Normalise(double[] v)
    {
        double len = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        return len > 1e-12 ? [v[0] / len, v[1] / len, v[2] / len] : [0, 0, 1];
    }
}

/// <summary>
/// The lumped corrections to the model, estimated from what the vehicle is measured to feel - offset-free MPC, but multiplicative: an additive bias, as the 6-DOF carries, would be wrong over a horizon where the dynamic pressure changes tenfold.
///
/// IN THE GLIDE THE MEASUREMENT IS CLEAN: the engine is off, so the non-gravitational acceleration is the aerodynamic force and nothing else. Its component against the airflow divided by the model's drag is a drag scale, its component across the airflow projected on the model's lift a lift scale.
///
/// UNDER THRUST the aerodynamic scales are frozen at what the glide learned, and what is left along the body axis once the scaled aerodynamics are taken away is the engine's: divided by the model's thrust it is a thrust scale. A min-fuel burn ends its flare at full throttle, so an engine a few percent weak, unlearned, arrives at the aim point fast with nothing in hand to stop.
///
/// Each is low-passed and clamped, and multiplied into the model the planner uses; see <see cref="Apply"/>.
/// </summary>
public sealed class AeroScaleEstimator
{
    public double DragScale { get; private set; } = 1.0;
    public double LiftScale { get; private set; } = 1.0;
    public double ThrustScale { get; private set; } = 1.0;

    public double TimeConstant { get; init; } = 3.0;
    public double ThrustTimeConstant { get; init; } = 1.0;
    public double MinDynamicPressure { get; init; } = 300.0;
    public double MinScale { get; init; } = 0.5;
    public double MaxScale { get; init; } = 2.0;
    public double MinThrustScale { get; init; } = 0.7;
    public double MaxThrustScale { get; init; } = 1.3;

    /// <summary>Lift is only measured where the model's lift is at least this fraction of its drag; below it the ratio is noise over nothing.</summary>
    public double MinLiftFraction { get; init; } = 0.1;

    public void Reset()
    {
        DragScale = LiftScale = ThrustScale = 1.0;
    }

    /// <summary>The nominal model with the learned scales applied.</summary>
    public KsaPointMassModel Apply(KsaPointMassModel nominal) => nominal with
    {
        DragScale = DragScale,
        LiftScale = LiftScale,
        VacuumThrust = nominal.VacuumThrust * ThrustScale,
        ExitArea = nominal.ExitArea * ThrustScale,
    };

    /// <param name="nominal">The model with every scale at 1.</param>
    /// <param name="x">Measured state, model frame.</param>
    /// <param name="b">Measured body axis.</param>
    /// <param name="throttle">Throttle the engine was commanded to, 0 with it off.</param>
    /// <param name="measured">Measured non-gravitational acceleration - aerodynamics and thrust - m/s^2, model frame.</param>
    public void Update(KsaPointMassModel nominal, double[] x, double[] b, double throttle, double[] measured, double dt)
    {
        if (!(dt > 0.0)) return;
        double vx = x[3], vy = x[4], vz = x[5];
        double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);

        if (throttle > KsaPointMassModel.EngineOnThrottle)
        {
            double thrustAccel = nominal.Thrust(throttle, nominal.Altitude(x)) / x[PointMass3Dof.IM];
            if (!(thrustAccel > 0.5)) return;
            double[] aero = PredictedAero(nominal with { DragScale = DragScale, LiftScale = LiftScale }, x, b);
            double along = (measured[0] - aero[0]) * b[0] + (measured[1] - aero[1]) * b[1] + (measured[2] - aero[2]) * b[2];
            double blendT = 1.0 - Math.Exp(-dt / ThrustTimeConstant);
            ThrustScale = Math.Clamp(ThrustScale + (along / thrustAccel - ThrustScale) * blendT, MinThrustScale, MaxThrustScale);
            return;
        }

        if (nominal.DynamicPressure(x) < MinDynamicPressure || !(sp > 1.0)) return;
        double[] a = PredictedAero(nominal, x, b);
        double[] h = [vx / sp, vy / sp, vz / sp];

        double predDrag = -(a[0] * h[0] + a[1] * h[1] + a[2] * h[2]);
        double measDrag = -(measured[0] * h[0] + measured[1] * h[1] + measured[2] * h[2]);
        double blend = 1.0 - Math.Exp(-dt / TimeConstant);
        if (predDrag > 1e-3)
            DragScale = Math.Clamp(DragScale + (measDrag / predDrag - DragScale) * blend, MinScale, MaxScale);

        double[] predLift = [a[0] + predDrag * h[0], a[1] + predDrag * h[1], a[2] + predDrag * h[2]];
        double[] measLift = [measured[0] + measDrag * h[0], measured[1] + measDrag * h[1], measured[2] + measDrag * h[2]];
        double pl2 = predLift[0] * predLift[0] + predLift[1] * predLift[1] + predLift[2] * predLift[2];
        if (pl2 > Math.Pow(MinLiftFraction * predDrag, 2) && pl2 > 1e-6)
        {
            double ratio = (measLift[0] * predLift[0] + measLift[1] * predLift[1] + measLift[2] * predLift[2]) / pl2;
            LiftScale = Math.Clamp(LiftScale + (ratio - LiftScale) * blend, MinScale, MaxScale);
        }
    }

    /// <summary>The model's aerodynamic acceleration at a state and attitude, engine off: its derivative less the same with the air taken away.</summary>
    public static double[] PredictedAero(KsaPointMassModel model, double[] x, double[] b)
    {
        Span<double> f = stackalloc double[PointMass3Dof.NX];
        Span<double> f0 = stackalloc double[PointMass3Dof.NX];
        double[] u = [b[0], b[1], b[2], 0.0];
        PointMass3Dof.Eval(model, x, u, f);
        PointMass3Dof.Eval(model with { Drag = null, LiftSlope = null }, x, u, f0);
        return [f[3] - f0[3], f[4] - f0[4], f[5] - f0[5]];
    }

    /// <summary>The model's non-gravitational acceleration - aerodynamics and thrust - at a state, attitude and throttle.</summary>
    public static double[] NonGravitational(KsaPointMassModel model, double[] x, double[] b, double throttle)
    {
        Span<double> f = stackalloc double[PointMass3Dof.NX];
        Span<double> f0 = stackalloc double[PointMass3Dof.NX];
        PointMass3Dof.Eval(model, x, [b[0], b[1], b[2], throttle], f);
        PointMass3Dof.Eval(model with { Drag = null, LiftSlope = null }, x, [b[0], b[1], b[2], 0.0], f0);
        return [f[3] - f0[3], f[4] - f0[4], f[5] - f0[5]];
    }
}
