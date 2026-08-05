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
///     throttle = T / Tmax
/// and nothing else. Attitude then evolves from the torque that deflection produces,
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

    /// <summary>Sigma bounds, so the UI can show when burn time is being DICTATED by a bound rather than chosen.</summary>
    public double SigmaMin => _cfg.SigmaMin;
    public double SigmaMax => _cfg.SigmaMax;

    /// <summary>True when the last Update needed the wide-trust-region retry — that retry is what turns a ~30 ms solve into ~500 ms.</summary>
    public bool FellBack { get; private set; }

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
    }

    /// <summary>Cold solve from a straight-line seed. ~1.7 s, so do it during a coast.</summary>
    public bool Plan(double[] x0, double[] xf, double sigmaSeed, double simNow, int maxIterations = 25)
    {
        var xSeed = new double[_n * NX];
        var uSeed = new double[_n * NU];
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

        _xf = (double[])xf.Clone();
        _solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed);
        return Finish(x0, simNow, maxIterations);
    }

    private double[] _xf = [];

    /// <summary>
    /// One MPC step: re-solve from the live state. The previous solution, shifted
    /// forward, seeds it — that is what keeps the warm start good and the solve at
    /// ~33 ms — but the ANSWER is anchored at the vehicle by the initial-state
    /// equality, not at the shifted seed.
    /// </summary>
    public bool Update(double[] x0, double simNow, int maxIterations = 5)
    {
        if (!HasPlan)
            return false;

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

        double sigma = Math.Max(_cfg.SigmaMin, _planSigma - elapsed);
        FellBack = false;
        _solver.Reseed(x0, xs, us, sigma, trustRegion: 0.05);
        if (Finish(x0, simNow, maxIterations))
            return true;

        FellBack = true;

        // The tight trust region above assumes the vehicle is near its previous plan.
        // Once it has genuinely diverged that makes the problem infeasible, and a
        // failed solve would otherwise leave the OLD plan in place — flying a stale
        // trajectory, which is the failure mode this whole class exists to avoid.
        _solver.Reseed(x0, xs, us, sigma, trustRegion: _solver.TrustRegionMax);
        return Finish(x0, simNow, maxIterations * 3);
    }

    private bool Finish(double[] x0, double simNow, int maxIterations)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        Status = _solver.Solve(maxIterations);
        LastSolveMs = sw.Elapsed.TotalMilliseconds;
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
        LastDefect = double.PositiveInfinity;
        for (int i = _solver.Trace.Count - 1; i >= 0; i--)
            if (_solver.Trace[i].Accepted)
            {
                LastDefect = _solver.Trace[i].DefectNorm;
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
        if (!(LastDefect <= _solver.DefectTolerance))
        {
            Error = $"plan is not physical - dynamics defect {LastDefect:E2} " +
                    $"exceeds {_solver.DefectTolerance:E0} after {_solver.IterationCount} iters " +
                    $"({AcceptedSteps} accepted). Needs more iterations or an easier problem.";
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
    public bool Command(double simNow, out double3 torqueModel, out double throttle)
    {
        torqueModel = default;
        throttle = 0.0;
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
        throttle = Math.Clamp(thrust / _cfg.Tmax, 0.0, 1.0);
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
