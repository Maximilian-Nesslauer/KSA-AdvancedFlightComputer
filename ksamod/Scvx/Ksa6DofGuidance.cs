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
            xSeed[k * NX + 6] = 1.0;                              // identity quaternion
            xSeed[k * NX + 13] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * Math.Abs(_dyn.Gz);    // ~hover axial thrust
        }

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
    public bool Update(double[] x0, double simNow, int maxIterations = 8)
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
        _solver.Reseed(x0, xs, us, sigma, trustRegion: 0.05);
        if (Finish(x0, simNow, maxIterations))
            return true;

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

    private double Lerp(double[] a, int off, int k, double f)
    {
        int k1 = Math.Min(k + 1, _n - 1);
        return a[k * NU + off] * (1 - f) + a[k1 * NU + off] * f;
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
