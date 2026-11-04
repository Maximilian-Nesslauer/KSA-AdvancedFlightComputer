namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>One SCvx iteration, as launch3dof.py prints it.</summary>
public readonly record struct AscentIteration(
    int Index,
    bool Solved,
    bool Accepted,
    double Rho,
    double TrustRegion,       // after this iteration's update
    double[] SigmaSeconds,    // of the reference after this iteration
    double Predicted,
    double DefectNorm,        // of the candidate
    double PathViolation,
    double TerminalViolation,
    double DeltaV,
    double Cost,              // true merit of the candidate
    int SolverIterations,
    double SolveMs,
    double ElapsedMs,
    string Status);

public enum AscentStatus
{
    /// <summary>The model predicts no further gain and every feasibility gate passes.</summary>
    Converged,
    /// <summary>Six rejections or failures in a row at the smallest trust region.</summary>
    Stalled,
    /// <summary>Ran out of iterations.</summary>
    IterationLimit,
    /// <summary>Stopped by the caller.</summary>
    Cancelled,
    /// <summary>No flyable seed, or an invalid problem.</summary>
    Failed,
}

/// <summary>
/// The converged ascent, back in SI and in the frame the problem was posed in, one entry per node. Immutable once built, so it can be handed from the solver thread to the game.
/// </summary>
public sealed class AscentSolution
{
    public required AscentStatus Status { get; init; }
    public required string Message { get; init; }
    public required int Nodes { get; init; }
    public required int[] NodeStage { get; init; }
    /// <summary>Seconds from lift-off. Two nodes share each staging instant.</summary>
    public required double[] Time { get; init; }
    public required double[] Position { get; init; }   // 3 per node, m
    public required double[] Velocity { get; init; }   // 3 per node, m/s
    public required double[] Mass { get; init; }       // kg
    public required double[] Throttle { get; init; }   // u, 3 per node
    public required double[] Thrust { get; init; }     // N delivered: the liquid engines' full-throttle thrust at the node's altitude times |u|, plus the solids'
    /// <summary>Full-throttle thrust at each node, N: the liquid engines at full throttle plus the solids, which is what the vehicle could deliver there.</summary>
    public double[] FullThrust { get; init; } = [];
    /// <summary>The solids' share of each node's thrust, N; zero where none burn.</summary>
    public double[] SolidThrust { get; init; } = [];
    public required double[] DynamicPressure { get; init; }  // Pa
    public required double[] QAlpha { get; init; }           // Pa rad
    /// <summary>Angle between the thrust axis and the air-relative velocity at each node, deg: what the drag's attitude term reads.</summary>
    public double[] AngleOfAttackDeg { get; init; } = [];
    public required double[] BurnTime { get; init; }   // s per stage
    public required double[] StageDeltaV { get; init; } // m/s per stage, at the reference exhaust velocity
    public required double[] TerminalResidual { get; init; } // SI: m, m/s, m^2/s, m, m/s
    public required double MaxDefectPosition { get; init; }  // m
    public required double MaxDefectVelocity { get; init; }  // m/s
    public required double MaxDefectMass { get; init; }      // kg
    public required double KickDeg { get; init; }
    /// <summary>What the seed search noticed, empty if nothing: a kick scan that bottomed out, or path limits no gravity turn meets.</summary>
    public string SeedNote { get; init; } = "";
    /// <summary>The lowest peak dynamic pressure any seed flew, Pa - close to the least this vehicle can manage at full throttle. NaN when there was no seed.</summary>
    public double SeedLeastMaxQ { get; init; } = double.NaN;
    public required int Iterations { get; init; }
    public required int Accepted { get; init; }
    public required double SolveSeconds { get; init; }
    public required double SeedSeconds { get; init; }
    public required IReadOnlyList<AscentIteration> Trace { get; init; }

    /// <summary>The returned reference passes every feasibility gate: dynamics defect, path limits and insertion.</summary>
    public bool GatesMet { get; init; }
    /// <summary>The gain the model still predicted at the last iteration, as kilograms of lift-off mass (the merit's fuel term is the final mass over the lift-off mass). NaN when no subproblem solved.</summary>
    public double RemainingGainKg { get; init; } = double.NaN;

    public bool Converged => Status == AscentStatus.Converged;

    /// <summary>
    /// Fit to fly: converged, or stalled at the smallest trust region on a reference that passes every feasibility gate with less than <see cref="AscentSettings.StallAcceptGain"/> still predicted. Such a stall is a plan the solver cannot prove optimal, not one it cannot fly; see <see cref="AscentSettings.StallAcceptGain"/>.
    /// </summary>
    public bool Usable => Converged || StallUsable;
    public bool StallUsable { get; init; }

    /// <summary>With a booster brought home (AscentProblem.Recovery): the reserve the plan keeps in it, kg; what it needs from the plan's separation, kg; that need's boostback impulse and landing burn, m/s; and when it separates, s from lift-off. NaN without one.</summary>
    public double RecoveryReserveKg { get; init; } = double.NaN;
    public double RecoveryRequiredKg { get; init; } = double.NaN;
    public double BoostbackDv { get; init; } = double.NaN;
    public double LandingDv { get; init; } = double.NaN;
    public double SeparationTime { get; init; } = double.NaN;

    /// <summary>How far the plan turned the target plane about the spin axis, radians, east positive (AscentProblem.PlaneTurnMin): a launch delay of -PlaneTurn / Omega for a plane fixed in space, or a LAN change for one that is not. The insertion residuals are taken in the turned plane.</summary>
    public double PlaneTurn { get; init; }

    /// <summary>
    /// The same plan turned about the spin axis (z) by <paramref name="angle"/>, radians, east positive: positions, velocities and thrust directions; everything else is unchanged, since nothing else depends on where round the axis the plan is. A plan solved from a pad and flown <c>t</c> later, from where the spin has carried the pad, is this turned by Omega t.
    /// </summary>
    public AscentSolution RotatedAboutSpin(double angle)
    {
        if (angle == 0.0 || Nodes == 0)
            return this;
        double c = Math.Cos(angle), s = Math.Sin(angle);
        double[] Turn(double[] v)
        {
            var o = (double[])v.Clone();
            for (int k = 0; k + 2 < o.Length; k += 3)
            {
                double x = v[k], y = v[k + 1];
                o[k] = c * x - s * y;
                o[k + 1] = s * x + c * y;
            }
            return o;
        }
        return new AscentSolution
        {
            Status = Status, Message = Message, Nodes = Nodes, NodeStage = NodeStage, Time = Time,
            Position = Turn(Position), Velocity = Turn(Velocity), Mass = Mass, Throttle = Turn(Throttle),
            Thrust = Thrust, FullThrust = FullThrust, SolidThrust = SolidThrust, DynamicPressure = DynamicPressure,
            QAlpha = QAlpha, AngleOfAttackDeg = AngleOfAttackDeg, BurnTime = BurnTime, StageDeltaV = StageDeltaV,
            TerminalResidual = TerminalResidual, MaxDefectPosition = MaxDefectPosition, MaxDefectVelocity = MaxDefectVelocity,
            MaxDefectMass = MaxDefectMass, KickDeg = KickDeg, SeedNote = SeedNote, SeedLeastMaxQ = SeedLeastMaxQ,
            Iterations = Iterations, Accepted = Accepted, SolveSeconds = SolveSeconds, SeedSeconds = SeedSeconds, Trace = Trace,
            GatesMet = GatesMet, RemainingGainKg = RemainingGainKg, StallUsable = StallUsable, PlaneTurn = PlaneTurn + angle,
            RecoveryReserveKg = RecoveryReserveKg, RecoveryRequiredKg = RecoveryRequiredKg, BoostbackDv = BoostbackDv,
            LandingDv = LandingDv, SeparationTime = SeparationTime,
        };
    }
    public double FinalMass => Mass[Nodes - 1];
    public double TotalDeltaV => StageDeltaV.Sum();
}

/// <summary>
/// Minimum-propellant multi-stage ascent by successive convexification: a port of launch3dof.py's solver onto the mod's own conic machinery (Clarabel, as the script uses through CVXPY) and forward-mode AD (as the script uses JAX).
///
/// THE LOOP IS THE SCRIPT'S, TERM FOR TERM. Linearise about the reference, solve the subproblem inside the trust box, and judge the step on a merit that has the same terms and weights as the subproblem objective but with each approximation replaced by the quantity it approximates - the true collocation defect for the virtual control, the true insertion miss for its slacks, the true violation of the true limit for each path slack. At the reference the two agree term by term, which is what lets predicted and actual reductions be compared. Accept on any true decrease, shrink below 0.25, grow above 0.7 if the step used the box. Converged when the model predicts less than PredictedTolerance of gain AND the reference passes every feasibility gate - never on step size alone, which passes automatically once the box has shrunk below the tolerance.
///
/// Offline only: a solve is seconds of work. Run it on a worker thread and pass a token to cancel between iterations.
/// </summary>
public static class AscentScvx
{
    private const int NX = AscentCase.NX;
    private const int NU = AscentCase.NU;

    public static AscentSolution Solve(AscentProblem problem, Action<AscentIteration>? onIteration = null,
                                       Action<string>? onPhase = null, CancellationToken cancel = default)
    {
        var total = System.Diagnostics.Stopwatch.StartNew();
        string invalid = problem.Validate();
        if (invalid.Length > 0)
            return Failed(problem, invalid);

        var c = new AscentCase(problem);
        AscentSettings st = c.Settings;

        // A BOOSTER BROUGHT HOME STARTS FROM THE PLAN WITH ITS RESERVE AS SET. SCvx is local, and the reserve and the separation feed each other - a booster that keeps more separates slower and needs less - so the plan it reaches depends on where it starts. From a gravity-turn seed far short of the need, the slack's price mends it in the most local way, bending the climb to cheapen the boostback: on the 2stage_new stack that reached 7.84 t to orbit where a fixed 7.06 t reserve that comes home reaches 8.01 t. Solved first with the reserve held where it was set - an efficient climb for that reserve - and freed from there, the solve only trades the reserve against that climb.
        AscentSolution? warm = null;
        if (problem.Recovery != null && st.RecoveryWarmStart)
        {
            onPhase?.Invoke($"booster reserve: planning first with it held at {problem.Recovery.InitialReserveKg / 1000.0:F2} t");
            AscentSolution held = Solve(problem.WithRecovery(null), onIteration, onPhase, cancel);
            if (held.Status == AscentStatus.Cancelled)
                return Failed(problem, "cancelled", AscentStatus.Cancelled);
            if (held.Nodes > 0 && held.GatesMet)
                warm = held;
            // A held plan short of its own booster's need is a climb that cannot bring it home, and freeing the reserve from there bends the climb to make up for it: from none on the 2stage_new stack that reached 7.81 t to orbit, where a fixed 7.02 t reserve that comes home reaches 8.02 t. So it is held once more, at that plan's own need, and freed from the climb that carries it.
            if (warm != null)
            {
                int ks = c.Ends[c.RecStage];
                BoostbackOracle.Need need = c.Oracle!.Evaluate(warm.Position.AsSpan(ks * 3, 3), warm.Velocity.AsSpan(ks * 3, 3), warm.Time[ks], null, gradient: false);
                double kept = problem.Recovery.InitialReserveKg;
                double most = (c.RMaxC + c.R0C) * c.Dyn.MU;
                if (need.Ok && need.RequiredKg > kept + 0.02 * need.RequiredKg + 50.0)
                {
                    double again = Math.Min(need.RequiredKg, most);
                    onPhase?.Invoke($"booster reserve: that plan's booster needs {need.RequiredKg / 1000.0:F2} t; planning again with it held there");
                    AscentProblem moved = problem.WithInitialReserve(again);
                    AscentSolution held2 = Solve(moved.WithRecovery(null), onIteration, onPhase, cancel);
                    if (held2.Status == AscentStatus.Cancelled)
                        return Failed(problem, "cancelled", AscentStatus.Cancelled);
                    if (held2.Nodes > 0 && held2.GatesMet)
                    {
                        problem = moved;
                        c = new AscentCase(problem);
                        st = c.Settings;
                        held = held2;
                        warm = held2;
                    }
                }
            }
            onPhase?.Invoke(warm != null
                ? $"booster reserve: freeing it from that plan ({held.Message}, {held.FinalMass / 1000.0:F2} t to orbit)"
                : $"booster reserve: that plan did not hold ({held.Message}), freeing it from the seed instead");
        }

        AscentSeed.Result seed;
        if (warm != null)
        {
            seed = WarmStart(c, warm);
        }
        else
        {
            onPhase?.Invoke("searching for a seed");
            seed = AscentSeed.Search(c);
            if (!seed.Ok)
                return Failed(problem, "no gravity-turn seed stays flyable" + (seed.Note.Length > 0 ? $" ({seed.Note})" : ""));
        }

        double seedSeconds = total.Elapsed.TotalSeconds;
        if (cancel.IsCancellationRequested)
            return Failed(problem, "cancelled", AscentStatus.Cancelled);

        double[] xbar = seed.X, ubar = seed.U, sigBar = (double[])seed.Sigma.Clone();
        double phiBar = warm != null ? Math.Clamp(warm.PlaneTurn, c.PlaneTurnMin, c.PlaneTurnMax) : c.PlaneTurn0;
        double drBar = 0.0;
        Merit refMerit = TrueCost(c, xbar, ubar, sigBar, phiBar, drBar, null);
        // The booster's need at the reference, with its slopes: what the recovery row linearises. Recomputed only when the reference moves.
        BoostbackOracle.Need needRef = default;
        bool needStale = true;
        onPhase?.Invoke($"seed: kick {seed.KickDeg:F3} deg after {seed.Evaluations} flights{(seed.Note.Length > 0 ? " (" + seed.Note + ")" : "")}, "
                      + $"merit {refMerit.J:E3}, defect {refMerit.DefectNorm:E2}, path {refMerit.PathViolation:E2}, insertion {refMerit.TerminalViolation:E2}");
        var lin = new Linearization(c.N);
        var trace = new List<AscentIteration>();
        double tr = st.TrustInitial;
        int stall = 0, accepted = 0;
        double lastPred = double.NaN;
        AscentStatus status = AscentStatus.IterationLimit;
        string message = "";

        onPhase?.Invoke("solving");
        int it = 0;
        while (it < st.MaxIterations)
        {
            if (cancel.IsCancellationRequested)
            {
                status = AscentStatus.Cancelled;
                message = "cancelled";
                break;
            }
            var sw = System.Diagnostics.Stopwatch.StartNew();

            lin.Fill(c, xbar, ubar);
            if (c.RecStage >= 0 && needStale)
            {
                double[] rs = new double[3], vs = new double[3];
                c.Separation(xbar, sigBar, rs, vs, out double ts);
                needRef = c.Oracle!.Evaluate(rs, vs, ts, refMerit.Need.Ok ? refMerit.Need.Impulse : null, gradient: true);
                needStale = false;
            }
            var act = new List<int>();
            for (int k = 0; k < c.N; k++)
                if (lin.Q[k] > st.QActive) act.Add(k);

            var sub = new AscentSubproblem(c, [.. act]);
            AscentSubproblem.Solved sol;
            try
            {
                sol = sub.Solve(lin, xbar, ubar, sigBar, phiBar, tr, drBar, needRef);
            }
            // A missing solver is not a numerical failure, so a smaller box cannot help. The job reports it.
            catch (DllNotFoundException)
            {
                throw;
            }
            catch (Exception e)
            {
                sol = new AscentSubproblem.Solved(false, "exception: " + e.Message, [], [], [], [], [], [], [], [], [], 0, 0.0);
            }

            if (!sol.Ok)
            {
                // Every nonconvex constraint carries a slack, so the subproblem is feasible by construction: a failure is numerical. Retry with a smaller, better-conditioned box.
                tr = Math.Max(st.TrustMin, tr * st.Shrink);
                if (tr <= st.TrustMin * 1.001) stall++;
                var failed = new AscentIteration(it, false, false, double.NaN, tr, Seconds(c, sigBar),
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    sol.SolverIterations, sol.SolveMs, sw.Elapsed.TotalMilliseconds, "solve failed: " + sol.Status);
                trace.Add(failed);
                onIteration?.Invoke(failed);
                it++;
                if (stall >= st.StallMax)
                {
                    status = AscentStatus.Stalled;
                    message = "the solver keeps failing at the smallest trust region" + StallNote(c, refMerit, lastPred);
                    break;
                }
                continue;
            }

            // ---- ratio test: predicted (linearised) vs actual (true nonlinear) merit reduction.
            double jLin = (1.0 - sol.X[(c.N - 1) * NX + 6]) + Smoothing(c, sol.U)
                + st.RhoVirtualControl * SumSquaresScaled(sol.Wv, c.DScale)
                + st.RhoTerminal * SumSquares(sol.STerm)
                + st.RhoPath * (sol.Sq.Sum() + sol.Sqa.Sum() + sol.Spro.Sum() + sol.Sg.Sum())
                + TurnCost(st, sol.Phi)
                + st.RhoRecovery * sol.SRec;
            Merit cand = TrueCost(c, sol.X, sol.U, sol.Sigma, sol.Phi, sol.DR, refMerit.Need.Ok ? refMerit.Need.Impulse : null);
            double pred = refMerit.J - jLin;
            double actual = refMerit.J - cand.J;
            lastPred = pred;

            double dX = MaxScaledDiff(sol.X, xbar, c.XScale);
            double dU = MaxScaledDiff(sol.U, ubar, [1.0, 1.0, 1.0]);
            double dS = 0.0;
            for (int s = 0; s < c.S; s++) dS = Math.Max(dS, Math.Abs(sol.Sigma[s] - sigBar[s]) / c.SigScale);
            double dP = Math.Abs(sol.Phi - phiBar) / st.PlaneTurnTrust;
            double dRr = c.RecStage >= 0 ? Math.Abs(sol.DR - drBar) / st.RecoveryTrust : 0.0;
            double used = Math.Max(Math.Max(Math.Max(dX, dP), dRr), Math.Max(dU, dS));

            double rho;
            bool accept, shrink, grow;
            if (pred > 1e-12)
            {
                rho = actual / pred;
                accept = rho > st.RhoAccept;
                shrink = rho < st.RhoShrink;
                grow = rho >= st.RhoGrow && used >= 0.8 * tr;
            }
            else
            {
                // The model predicts no gain inside this box: never divide two negatives into a "good" ratio, judge on the true change alone.
                rho = double.NaN;
                accept = actual > 0.0;
                shrink = !accept;
                grow = false;
            }

            if (accept)
            {
                xbar = sol.X;
                ubar = sol.U;
                sigBar = sol.Sigma;
                phiBar = sol.Phi;
                drBar = sol.DR;
                needStale = true;
                refMerit = cand;
                accepted++;
            }
            if (shrink) tr = Math.Max(st.TrustMin, tr * st.Shrink);
            else if (grow) tr = Math.Min(st.TrustMax, tr * st.Grow);
            stall = accept ? 0 : stall + (tr <= st.TrustMin * 1.001 ? 1 : 0);

            var rec = new AscentIteration(it, true, accept, rho, tr, Seconds(c, sigBar), pred,
                cand.DefectNorm, cand.PathViolation, cand.TerminalViolation, DeltaV(c, sol.X),
                cand.J, sol.SolverIterations, sol.SolveMs, sw.Elapsed.TotalMilliseconds,
                (accept ? "accept" : "reject") + (sol.Status == "Optimal" ? "" : $" [{sol.Status}]"));
            trace.Add(rec);
            onIteration?.Invoke(rec);
            it++;

            // The feasibility gates are those of the current REFERENCE, the last accepted iterate.
            bool feasible = GatesMet(st, refMerit);
            if (pred < st.PredictedTolerance && feasible)
            {
                status = AscentStatus.Converged;
                message = $"converged after {it} iterations";
                break;
            }
            if (stall >= st.StallMax)
            {
                status = AscentStatus.Stalled;
                message = $"stalled: {st.StallMax} rejections in a row at the smallest trust region, feasibility gates "
                        + (feasible ? "met" : "not met") + StallNote(c, refMerit, lastPred);
                break;
            }
        }
        if (status == AscentStatus.IterationLimit)
            message = $"no convergence in {st.MaxIterations} iterations";

        bool gates = GatesMet(st, refMerit);
        return Build(c, xbar, ubar, sigBar, phiBar, drBar, refMerit.Need, status, message, seed, it, accepted,
            total.Elapsed.TotalSeconds, seedSeconds, trace, gates,
            status == AscentStatus.Stalled && StallAcceptable(st, refMerit, lastPred),
            lastPred * c.Dyn.MU);
    }

    private static bool GatesMet(AscentSettings st, Merit m)
        => m.DefectNorm < st.DefectTolerance && m.PathViolation < st.PathTolerance && m.TerminalViolation < st.TerminalTolerance
           && m.RecoveryShort < st.RecoveryTolerance;

    /// <summary>A stall that is flown anyway: its reference passes every gate, and the model's last promise is small. See <see cref="AscentSettings.StallAcceptGain"/>.</summary>
    private static bool StallAcceptable(AscentSettings st, Merit m, double lastPred)
        => GatesMet(st, m) && double.IsFinite(lastPred) && lastPred < st.StallAcceptGain;

    /// <summary>What a stall's message adds when the plan is flown anyway: how far the model still thought it could go.</summary>
    private static string StallNote(AscentCase c, Merit m, double lastPred)
        => StallAcceptable(c.Settings, m, lastPred)
            ? $", so the plan is usable: the last step still promised {Math.Max(lastPred, 0.0) * c.Dyn.MU:F1} kg"
            : "";

    // ------------------------------------------------------------------ merit

    /// <summary>The merit and the reference's gates; with a booster brought home, how short its reserve falls of its need (canonical) and that need.</summary>
    private readonly record struct Merit(double J, double DefectNorm, double PathViolation, double TerminalViolation,
                                         double RecoveryShort = 0.0, BoostbackOracle.Need Need = default);

    /// <summary>
    /// J = fuel + smoothing + virtual-control penalty + insertion penalty + path penalty, with the TRUE nonlinear dynamics and constraints, each term of the same form and weight as its linearised counterpart in the subproblem.
    /// </summary>
    /// <summary>A converged plan of the same problem as the reference to start from, in the case's canonical units: the same nodes, since a booster brought home changes no node.</summary>
    private static AscentSeed.Result WarmStart(AscentCase c, AscentSolution w)
    {
        AscentDynamics d = c.Dyn;
        var x = new double[c.N * NX];
        var u = new double[c.N * NU];
        for (int k = 0; k < c.N; k++)
        {
            for (int i = 0; i < 3; i++)
            {
                x[k * NX + i] = w.Position[k * 3 + i] / d.LU;
                x[k * NX + 3 + i] = w.Velocity[k * 3 + i] / d.VU;
                u[k * NU + i] = w.Throttle[k * 3 + i];
            }
            x[k * NX + 6] = w.Mass[k] / d.MU;
        }
        var sig = new double[c.S];
        for (int s = 0; s < c.S; s++)
            sig[s] = w.BurnTime[s] / d.TU;
        return new AscentSeed.Result(true, x, u, sig, w.KickDeg, 0.0, 0, w.SeedNote, w.SeedLeastMaxQ);
    }

    private static double SepTime(AscentCase c, double[] sig)
    {
        double t = 0.0;
        for (int s = 0; s <= c.RecStage; s++) t += sig[s] * c.Dyn.TU;
        return t;
    }

    /// <summary>The plane turn's tie-break price, as the subproblem's objective has it (AscentSettings.PlaneTurnPrice).</summary>
    private static double TurnCost(AscentSettings st, double phi) => st.PlaneTurnPrice * Math.Abs(phi);

    private static Merit TrueCost(AscentCase c, double[] x, double[] u, double[] sig, double phi, double dr, double[]? impulse)
    {
        AscentSettings st = c.Settings;
        int n = c.N;
        var f = new double[n * NX];
        for (int k = 0; k < n; k++)
            c.Dyn.Eval(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), k, f.AsSpan(k * NX, NX));

        double defect = 0.0, worst = 0.0;
        foreach (int k in c.Coll)
        {
            int s = c.NodeStage[k];
            double half = 0.5 * c.Dtau[k] * sig[s];
            for (int i = 0; i < NX; i++)
            {
                double d = x[(k + 1) * NX + i] - x[k * NX + i] - half * (f[k * NX + i] + f[(k + 1) * NX + i]);
                double scaled = d / c.DScale[i];
                defect += scaled * scaled;
                worst = Math.Max(worst, Math.Abs(scaled));
            }
        }

        Span<double> res = stackalloc double[5];
        c.TerminalResidual(x, phi, res);
        double term = 0.0, termWorst = 0.0;
        for (int j = 0; j < 5; j++)
        {
            term += res[j] * res[j];
            termWorst = Math.Max(termWorst, Math.Abs(res[j]));
        }

        double path = 0.0, pathWorst = 0.0;
        for (int k = 0; k < n; k++)
        {
            double qa = c.Dyn.QAlpha(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), c.NodeStage[k]);
            double q = c.Dyn.QValue(x.AsSpan(k * NX, NX));
            double vqa = Math.Max(0.0, qa / c.QAlphaMax - 1.0);
            double vq = Math.Max(0.0, q / c.QMax - 1.0);
            // Thrust against the airflow, where the subproblem forbids it (AscentSubproblem): the true -v_rel_hat . u.
            double vpro = q > st.QActive ? Math.Max(0.0, -AirAlong(c, x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU))) : 0.0;
            path += vqa + vq + vpro;
            pathWorst = Math.Max(pathWorst, Math.Max(Math.Max(vqa, vq), vpro));
        }
        // Thrust acceleration over the limit, where a stage has one (AscentSubproblem): the true thrust at the node's altitude.
        foreach (int k in c.LimitedNodes)
        {
            c.Dyn.ThrustAt(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), k, out _, out double delivered, out _);
            double mass = Math.Max(x[k * NX + AscentDynamics.IM], 1e-12);
            double vg = Math.Max(0.0, delivered / c.Dyn.FU / (c.AccC[c.NodeStage[k]] * mass) - 1.0);
            path += vg;
            pathWorst = Math.Max(pathWorst, vg);
        }

        double j0 = (1.0 - x[(n - 1) * NX + 6]) + Smoothing(c, u)
            + st.RhoVirtualControl * defect + st.RhoTerminal * term + st.RhoPath * path + TurnCost(st, phi);
        // The booster's shortfall: the true need at this plan's separation, against what it keeps. A booster that cannot land at all is short the whole stack.
        double shortC = 0.0;
        BoostbackOracle.Need need = default;
        if (c.RecStage >= 0)
        {
            double[] rs = new double[3], vs = new double[3];
            c.Separation(x, sig, rs, vs, out double ts);
            need = c.Oracle!.Evaluate(rs, vs, ts, impulse, gradient: false);
            shortC = need.Ok ? Math.Max(0.0, need.RequiredKg / c.Dyn.MU - (c.R0C + dr)) : 1.0;
            j0 += st.RhoRecovery * shortC;
        }
        return new Merit(j0, worst, pathWorst, termWorst, shortC, need);
    }

    /// <summary>The throttle vector's component along the air-relative velocity: |u| cos(alpha).</summary>
    private static double AirAlong(AscentCase c, ReadOnlySpan<double> x, ReadOnlySpan<double> u)
    {
        double[] air = c.AirVelocity(x);
        double an = AscentCase.Norm3(air);
        return an > 1e-12 ? (air[0] * u[0] + air[1] * u[1] + air[2] * u[2]) / an : 0.0;
    }

    private static double Smoothing(AscentCase c, double[] u)
    {
        double s = 0.0;
        for (int k = 0; k < c.N - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                double d = u[(k + 1) * NU + j] - u[k * NU + j];
                s += d * d;
            }
        return c.Settings.SmoothingWeight * s;
    }

    private static double SumSquaresScaled(double[] v, double[] scale)
    {
        double s = 0.0;
        for (int i = 0; i < v.Length; i++)
        {
            double d = v[i] / scale[i % scale.Length];
            s += d * d;
        }
        return s;
    }

    private static double SumSquares(double[] v)
    {
        double s = 0.0;
        foreach (double x in v) s += x * x;
        return s;
    }

    private static double MaxScaledDiff(double[] a, double[] b, double[] scale)
    {
        double w = 0.0;
        for (int i = 0; i < a.Length; i++)
            w = Math.Max(w, Math.Abs((a[i] - b[i]) / scale[i % scale.Length]));
        return w;
    }

    private static double[] Seconds(AscentCase c, double[] sig)
    {
        var s = new double[sig.Length];
        for (int i = 0; i < sig.Length; i++) s[i] = sig[i] * c.Dyn.TU;
        return s;
    }

    /// <summary>Ideal dV over the burns, at each stage's reference exhaust velocity - the script's dv readout.</summary>
    private static double DeltaV(AscentCase c, double[] x)
    {
        double dv = 0.0;
        for (int s = 0; s < c.S; s++)
        {
            double m0 = x[c.Starts[s] * NX + 6], m1 = x[c.Ends[s] * NX + 6];
            if (m0 > 0.0 && m1 > 0.0)
                dv += c.Problem.Stages[s].ExhaustVelocity * Math.Log(m0 / m1);
        }
        return dv;
    }

    // --------------------------------------------------------------- results

    private static AscentSolution Build(AscentCase c, double[] x, double[] u, double[] sig, double phi,
                                        double dr, BoostbackOracle.Need need, AscentStatus status, string message, AscentSeed.Result seed,
                                        int iterations, int accepted, double seconds, double seedSeconds,
                                        List<AscentIteration> trace, bool gatesMet, bool stallUsable, double remainingGainKg)
    {
        int n = c.N;
        AscentDynamics d = c.Dyn;
        var time = new double[n];
        for (int k = 0; k < n - 1; k++)
            time[k + 1] = time[k] + (c.CollRow[k] >= 0 ? sig[c.NodeStage[k]] * c.Dtau[k] * d.TU : 0.0);

        var pos = new double[n * 3];
        var vel = new double[n * 3];
        var mass = new double[n];
        var thr = new double[n * 3];
        var thrust = new double[n];
        var fullThrust = new double[n];
        var solidThrust = new double[n];
        var q = new double[n];
        var qa = new double[n];
        var aoa = new double[n];
        for (int k = 0; k < n; k++)
        {
            for (int i = 0; i < 3; i++)
            {
                pos[k * 3 + i] = x[k * NX + i] * d.LU;
                vel[k * 3 + i] = x[k * NX + 3 + i] * d.VU;
                thr[k * 3 + i] = u[k * NU + i];
            }
            mass[k] = x[k * NX + 6] * d.MU;
            d.ThrustAt(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), k, out double liquidFull, out thrust[k], out solidThrust[k]);
            fullThrust[k] = liquidFull + solidThrust[k];
            q[k] = d.QValue(x.AsSpan(k * NX, NX));
            qa[k] = d.QAlpha(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), c.NodeStage[k]);
            aoa[k] = d.AngleOfAttackDeg(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), c.NodeStage[k]);
        }

        // True defects of the reference, per channel, in SI.
        var f = new double[n * NX];
        for (int k = 0; k < n; k++)
            d.Eval(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), k, f.AsSpan(k * NX, NX));
        double dp = 0, dvv = 0, dm = 0;
        foreach (int k in c.Coll)
        {
            double half = 0.5 * c.Dtau[k] * sig[c.NodeStage[k]];
            for (int i = 0; i < NX; i++)
            {
                double e = Math.Abs(x[(k + 1) * NX + i] - x[k * NX + i] - half * (f[k * NX + i] + f[(k + 1) * NX + i]));
                if (i < 3) dp = Math.Max(dp, e * d.LU);
                else if (i < 6) dvv = Math.Max(dvv, e * d.VU);
                else dm = Math.Max(dm, e * d.MU);
            }
        }

        var res = new double[5];
        c.TerminalResidual(x, phi, res);
        res[0] *= d.LU;
        res[1] *= d.VU;
        res[2] *= d.LU * d.VU;
        res[3] *= d.LU;
        res[4] *= d.VU;

        var burn = new double[c.S];
        var dv = new double[c.S];
        for (int s = 0; s < c.S; s++)
        {
            burn[s] = sig[s] * d.TU;
            double m0 = x[c.Starts[s] * NX + 6], m1 = x[c.Ends[s] * NX + 6];
            dv[s] = m0 > 0 && m1 > 0 ? c.Problem.Stages[s].ExhaustVelocity * Math.Log(m0 / m1) : 0.0;
        }

        return new AscentSolution
        {
            Status = status,
            Message = message,
            Nodes = n,
            NodeStage = (int[])c.NodeStage.Clone(),
            Time = time,
            Position = pos,
            Velocity = vel,
            Mass = mass,
            Throttle = thr,
            Thrust = thrust,
            FullThrust = fullThrust,
            SolidThrust = solidThrust,
            DynamicPressure = q,
            QAlpha = qa,
            AngleOfAttackDeg = aoa,
            BurnTime = burn,
            StageDeltaV = dv,
            TerminalResidual = res,
            MaxDefectPosition = dp,
            MaxDefectVelocity = dvv,
            MaxDefectMass = dm,
            KickDeg = seed.KickDeg,
            SeedNote = seed.Note,
            SeedLeastMaxQ = seed.LeastMaxQ,
            Iterations = iterations,
            Accepted = accepted,
            SolveSeconds = seconds,
            SeedSeconds = seedSeconds,
            Trace = trace,
            PlaneTurn = phi,
            RecoveryReserveKg = c.RecStage >= 0 ? (c.R0C + dr) * c.Dyn.MU : double.NaN,
            RecoveryRequiredKg = c.RecStage >= 0 && need.Ok ? need.RequiredKg : double.NaN,
            BoostbackDv = c.RecStage >= 0 && need.Ok ? need.BoostbackDv : double.NaN,
            LandingDv = c.Oracle?.LandingDv ?? double.NaN,
            SeparationTime = c.RecStage >= 0 ? SepTime(c, sig) : double.NaN,
            GatesMet = gatesMet,
            StallUsable = stallUsable,
            RemainingGainKg = remainingGainKg,
        };
    }

    private static AscentSolution Failed(AscentProblem p, string message, AscentStatus status = AscentStatus.Failed)
        => new()
        {
            Status = status,
            Message = message,
            Nodes = 0,
            NodeStage = [],
            Time = [],
            Position = [],
            Velocity = [],
            Mass = [],
            Throttle = [],
            Thrust = [],
            DynamicPressure = [],
            QAlpha = [],
            BurnTime = [],
            StageDeltaV = [],
            TerminalResidual = [],
            MaxDefectPosition = double.NaN,
            MaxDefectVelocity = double.NaN,
            MaxDefectMass = double.NaN,
            KickDeg = double.NaN,
            Iterations = 0,
            Accepted = 0,
            SolveSeconds = 0,
            SeedSeconds = 0,
            Trace = [],
        };
}
