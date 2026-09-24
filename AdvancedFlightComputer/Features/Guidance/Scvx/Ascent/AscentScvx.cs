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
    public required double[] Thrust { get; init; }     // N, full-throttle thrust at the node's altitude times |u|
    public required double[] DynamicPressure { get; init; }  // Pa
    public required double[] QAlpha { get; init; }           // Pa rad
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

    public bool Converged => Status == AscentStatus.Converged;
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

        onPhase?.Invoke("searching for a seed");
        AscentSeed.Result seed = AscentSeed.Search(c);
        double seedSeconds = total.Elapsed.TotalSeconds;
        if (!seed.Ok)
            return Failed(problem, "no gravity-turn seed stays flyable" + (seed.Note.Length > 0 ? $" ({seed.Note})" : ""));
        if (cancel.IsCancellationRequested)
            return Failed(problem, "cancelled", AscentStatus.Cancelled);

        double[] xbar = seed.X, ubar = seed.U, sigBar = (double[])seed.Sigma.Clone();
        Merit refMerit = TrueCost(c, xbar, ubar, sigBar);
        onPhase?.Invoke($"seed: kick {seed.KickDeg:F3} deg after {seed.Evaluations} flights{(seed.Note.Length > 0 ? " (" + seed.Note + ")" : "")}, "
                      + $"merit {refMerit.J:E3}, defect {refMerit.DefectNorm:E2}, path {refMerit.PathViolation:E2}, insertion {refMerit.TerminalViolation:E2}");
        var lin = new Linearization(c.N);
        var trace = new List<AscentIteration>();
        double tr = st.TrustInitial;
        int stall = 0, accepted = 0;
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
            var act = new List<int>();
            for (int k = 0; k < c.N; k++)
                if (lin.Q[k] > st.QActive) act.Add(k);

            var sub = new AscentSubproblem(c, [.. act]);
            AscentSubproblem.Solved sol;
            try
            {
                sol = sub.Solve(lin, xbar, ubar, sigBar, tr);
            }
            catch (Exception e)
            {
                sol = new AscentSubproblem.Solved(false, "exception: " + e.Message, [], [], [], [], [], [], [], 0, 0.0);
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
                    message = "the solver keeps failing at the smallest trust region";
                    break;
                }
                continue;
            }

            // ---- ratio test: predicted (linearised) vs actual (true nonlinear) merit reduction.
            double jLin = (1.0 - sol.X[(c.N - 1) * NX + 6]) + Smoothing(c, sol.U)
                + st.RhoVirtualControl * SumSquaresScaled(sol.Wv, c.DScale)
                + st.RhoTerminal * SumSquares(sol.STerm)
                + st.RhoPath * (sol.Sq.Sum() + sol.Sqa.Sum());
            Merit cand = TrueCost(c, sol.X, sol.U, sol.Sigma);
            double pred = refMerit.J - jLin;
            double actual = refMerit.J - cand.J;

            double dX = MaxScaledDiff(sol.X, xbar, c.XScale);
            double dU = MaxScaledDiff(sol.U, ubar, [1.0, 1.0, 1.0]);
            double dS = 0.0;
            for (int s = 0; s < c.S; s++) dS = Math.Max(dS, Math.Abs(sol.Sigma[s] - sigBar[s]) / c.SigScale);
            double used = Math.Max(dX, Math.Max(dU, dS));

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
            bool feasible = refMerit.DefectNorm < st.DefectTolerance
                         && refMerit.PathViolation < st.PathTolerance
                         && refMerit.TerminalViolation < st.TerminalTolerance;
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
                        + (feasible ? "met" : "not met");
                break;
            }
        }
        if (status == AscentStatus.IterationLimit)
            message = $"no convergence in {st.MaxIterations} iterations";

        return Build(c, xbar, ubar, sigBar, status, message, seed, it, accepted,
            total.Elapsed.TotalSeconds, seedSeconds, trace);
    }

    // ------------------------------------------------------------------ merit

    private readonly record struct Merit(double J, double DefectNorm, double PathViolation, double TerminalViolation);

    /// <summary>
    /// J = fuel + smoothing + virtual-control penalty + insertion penalty + path penalty, with the TRUE nonlinear dynamics and constraints, each term of the same form and weight as its linearised counterpart in the subproblem.
    /// </summary>
    private static Merit TrueCost(AscentCase c, double[] x, double[] u, double[] sig)
    {
        AscentSettings st = c.Settings;
        int n = c.N;
        var f = new double[n * NX];
        for (int k = 0; k < n; k++)
            c.Dyn.Eval(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), c.NodeStage[k], f.AsSpan(k * NX, NX));

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
        c.TerminalResidual(x, res);
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
            path += vqa + vq;
            pathWorst = Math.Max(pathWorst, Math.Max(vqa, vq));
        }

        double j0 = (1.0 - x[(n - 1) * NX + 6]) + Smoothing(c, u)
            + st.RhoVirtualControl * defect + st.RhoTerminal * term + st.RhoPath * path;
        return new Merit(j0, worst, pathWorst, termWorst);
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

    private static AscentSolution Build(AscentCase c, double[] x, double[] u, double[] sig,
                                        AscentStatus status, string message, AscentSeed.Result seed,
                                        int iterations, int accepted, double seconds, double seedSeconds,
                                        List<AscentIteration> trace)
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
        var q = new double[n];
        var qa = new double[n];
        for (int k = 0; k < n; k++)
        {
            for (int i = 0; i < 3; i++)
            {
                pos[k * 3 + i] = x[k * NX + i] * d.LU;
                vel[k * 3 + i] = x[k * NX + 3 + i] * d.VU;
                thr[k * 3 + i] = u[k * NU + i];
            }
            mass[k] = x[k * NX + 6] * d.MU;
            double um = AscentCase.Norm3(u.AsSpan(k * NU, NU));
            thrust[k] = d.MaxThrustAt(x.AsSpan(k * NX, NX), c.NodeStage[k]) * um;
            q[k] = d.QValue(x.AsSpan(k * NX, NX));
            qa[k] = d.QAlpha(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), c.NodeStage[k]);
        }

        // True defects of the reference, per channel, in SI.
        var f = new double[n * NX];
        for (int k = 0; k < n; k++)
            d.Eval(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), c.NodeStage[k], f.AsSpan(k * NX, NX));
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
        c.TerminalResidual(x, res);
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
            DynamicPressure = q,
            QAlpha = qa,
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
