using System.Diagnostics;
using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// The orbital entry planner: chooses vertical lift over drag against energy so the vehicle reaches the hand-over over its target, inside the heating, dynamic-pressure and load limits, with the least heat load.
///
///   minimise   Q/Q_seed + w_du sum((u_{k+1}-u_k)/U_s)^2 + penalties
///   subject to x_0 = measured; x_{k+1} = Phi(x_k, u_k, u_{k+1}) (linearised); s_N = 0 (soft);
///              r_N in the hand-over box (soft); r_k >= floor(e_k) (soft);
///              L/D cos(bank_max) &lt;= u_k &lt;= L/D cos(bank_min); |u_{k+1}-u_k| within the roll rate.
///
/// Only the dynamics are nonlinear, so this is SCvx with a light touch: exact discretisation by RK4 multiple shooting, Jacobians by central differences, virtual control and L1 slacks priced as exact penalties, an L-infinity trust region accepted on the true merit. Each subproblem is a small QP solved by Clarabel. The heat-load objective, 1/(C_D sqrt(rho)) per unit energy, is convex in altitude for an exponential atmosphere and enters as its second-order expansion.
/// </summary>
public sealed class EntryPlanner
{
    const int NX = EntryDynamics.NX;
    const int IR = EntryDynamics.IR, IG = EntryDynamics.IG, IS = EntryDynamics.IS;

    // Scales for the trust region, penalties and the gate: 5 km, 1 deg, 20 km of range; 0.1 of vertical L/D.
    static double[] StateScale(double r0) => [5_000.0 / r0, Math.PI / 180.0, 20_000.0 / r0];
    const double US = 0.1;

    public sealed class Result
    {
        public EntryPlan? Plan { get; init; }
        public string Why { get; init; } = "";
    }

    /// <summary>The final energy from the hand-over altitude and Mach.</summary>
    public static double FinalEnergy(EntryModel model, EntryConfig cfg)
    {
        double rf = 1.0 + cfg.HandoverAltitude / model.BodyRadius;
        double vf = cfg.HandoverMach * model.SpeedOfSound(cfg.HandoverAltitude) / model.SpeedUnit;
        return EntryDynamics.Energy(rf, vf);
    }

    /// <summary>
    /// Solve from the state (e0, x0). <paramref name="warm"/>, if given, seeds the control; otherwise a constant-bank seed is found by bisection on range.
    /// </summary>
    public Result Solve(EntryModel model, EntryConfig cfg, double e0, ReadOnlySpan<double> x0In, EntryPlan? warm)
    {
        var sw = Stopwatch.StartNew();
        var dyn = new EntryDynamics(model);
        double r0m = model.BodyRadius;
        double[] xs = StateScale(r0m);
        double[] x0 = x0In.ToArray();
        double ef = FinalEnergy(model, cfg);
        if (e0 >= ef) return new Result { Why = "past the hand-over energy" };

        int n = Math.Max(cfg.Nodes, 8);
        double[] e = Grid(e0, ef, x0[IR], cfg, r0m, n);

        // Fixed per-node data: the floors depend on energy alone.
        var floor = new double[n];
        for (int k = 0; k < n; k++) floor[k] = dyn.FloorRadius(e[k], cfg);
        double rfMin = 1.0 + (cfg.HandoverAltitude - cfg.HandoverAltitudeTol) / r0m;
        double rfMax = 1.0 + (cfg.HandoverAltitude + cfg.HandoverAltitudeTol) / r0m;
        double cosMax = Math.Cos(cfg.BankMaxDeg * Math.PI / 180.0);
        double cosMin = Math.Cos(cfg.BankMinDeg * Math.PI / 180.0);
        double rollRate = cfg.RollRateDegS * Math.PI / 180.0 * model.TimeUnit * cfg.PlanRollRateShare;

        // ---- Seed ----
        var x = new double[n, NX];
        var u = new double[n];
        bool cold = warm == null;
        if (warm != null)
        {
            for (int k = 0; k < n; k++) u[k] = warm.At(e[k]).U;
            Shoot(dyn, cfg, e, x0, u, x);
        }
        else
        {
            ColdSeed(dyn, cfg, e, x0, cosMin, cosMax, u, x);
        }
        ClampToBounds(dyn, e, x, u, cosMin, cosMax);
        Shoot(dyn, cfg, e, x0, u, x);

        var c = TrapezoidWeights(e);
        // The corridor at each node, for the margin term: the range left on the most drag and on the glide.
        CorridorAt(dyn, cfg, e, floor, cosMin, out double[] sShort, out double[] sLong);
        double widest = 0;
        for (int k = 0; k < n; k++) widest = Math.Max(widest, 0.5 * (sLong[k] - sShort[k]));
        var corridor = new CorridorMargin(sShort, sLong, Math.Max(0.25 * widest, 50_000.0 / r0m));
        double qRef = 0;
        for (int k = 0; k < n; k++) qRef += c[k] * dyn.HeatPerEnergy(e[k], x[k, IR]);
        qRef = Math.Max(qRef, 1e-12);

        // The plan before, on this grid, for the consistency term: a re-solve stays near what is being flown.
        double[]? uPrev = null;
        if (warm != null)
        {
            uPrev = new double[n];
            for (int k = 0; k < n; k++) uPrev[k] = warm.At(e[k]).U;
        }

        var lin = new Linearisation(n);
        double eta = cold ? 2.0 : 1.0;
        int maxIter = cold ? cfg.ColdIterations : cfg.WarmIterations;
        int iter = 0;
        double maxDefect = double.PositiveInfinity;
        double lastStep = double.PositiveInfinity;

        for (; iter < maxIter; iter++)
        {
            Linearise(dyn, cfg, e, x, u, lin);
            maxDefect = MaxScaledDefect(lin, xs, n);
            // Done once the plan is well inside the gate and the steps have become small: the rest of the iterations would only polish it.
            if (iter > 0 && maxDefect <= 0.25 * cfg.GateDefect && lastStep < 0.02
                && Math.Abs(x[n - 1, IS]) * r0m <= 0.25 * cfg.GateRangeM)
                break;
            double jRef = Merit(dyn, cfg, e, x, u, lin.Defect, c, qRef, floor, rfMin, rfMax, xs, uPrev, corridor);

            var sub = BuildAndSolve(dyn, cfg, e, x0, x, u, lin, c, qRef, floor, rfMin, rfMax,
                                    cosMin, cosMax, rollRate, xs, eta, uPrev, corridor, out double jLin);
            if (sub == null)
            {
                eta *= 0.5;
                if (eta < 1e-3) break;
                continue;
            }
            var (xc, uc) = sub.Value;

            var defC = new double[n - 1, NX];
            Defects(dyn, cfg, e, xc, uc, defC);
            double jCand = Merit(dyn, cfg, e, xc, uc, defC, c, qRef, floor, rfMin, rfMax, xs, uPrev, corridor);

            double predicted = jRef - jLin;
            double actual = jRef - jCand;
            double step = MaxScaledStep(x, u, xc, uc, xs, n);
            if (predicted <= 1e-9 * (1 + Math.Abs(jRef)))
            {
                if (actual >= 0) { Copy(xc, uc, x, u); }
                break;
            }
            double ratio = actual / predicted;
            if (ratio < 0.1)
            {
                eta *= 0.5;
                if (eta < 1e-3) break;
                continue;
            }
            Copy(xc, uc, x, u);
            lastStep = step;
            if (ratio < 0.25) eta *= 0.5;
            else if (ratio > 0.7) eta = Math.Min(eta * 2.0, 20.0);
            // A small step ends the solve only once the defect is inside the gate: before that the next iterations still close it.
            if (step < 1e-3 && MaxScaledDefectOf(dyn, cfg, e, x, u, xs) <= cfg.GateDefect) { iter++; break; }
        }

        Linearise(dyn, cfg, e, x, u, lin);
        maxDefect = MaxScaledDefect(lin, xs, n);
        double rangeErr = x[n - 1, IS] * r0m;
        bool ok = maxDefect <= cfg.GateDefect && Math.Abs(rangeErr) <= cfg.GateRangeM;

        var plan = Publish(dyn, cfg, e, x, u, floor, cosMin, ok, iter, maxDefect, rangeErr,
                           sw.Elapsed.TotalMilliseconds, cold);
        string why = ok ? "" : maxDefect > cfg.GateDefect
            ? $"defect {maxDefect:F3} over the gate"
            : $"misses the target by {rangeErr / 1000:F1} km";
        return new Result { Plan = plan, Why = why };
    }

    // ---------------------------------------------------------------- grid and seed

    /// <summary>Nodes evenly spaced in speed at a mean radius, so the low-speed end is not starved.</summary>
    static double[] Grid(double e0, double ef, double r0, EntryConfig cfg, double bodyR, int n)
    {
        double rBar = 0.5 * (r0 + 1.0 + cfg.HandoverAltitude / bodyR);
        var e = new double[n];
        double v0 = EntryDynamics.Speed(e0, rBar), vf = EntryDynamics.Speed(ef, rBar);
        bool bySpeed = v0 > vf && vf > 0;
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            if (bySpeed)
            {
                double v = v0 + (vf - v0) * t;
                e[k] = 1.0 / rBar - 0.5 * v * v;
            }
            else e[k] = e0 + (ef - e0) * t;
        }
        e[0] = e0;
        e[n - 1] = ef;
        return e;
    }

    static void ColdSeed(EntryDynamics dyn, EntryConfig cfg, double[] e, double[] x0,
                         double cosMin, double cosMax, double[] u, double[,] x)
    {
        int n = e.Length;
        double FinalRange(double lambda)
        {
            ShootConstantBank(dyn, cfg, e, x0, lambda, u, x);
            return x[n - 1, IS];
        }
        double lo = cosMax, hi = cosMin;     // more lift flies further, leaving less range to go
        double fLo = FinalRange(lo), fHi = FinalRange(hi);
        double best;
        if (fHi > 0) best = hi;              // short even fully lifted
        else if (fLo < 0) best = lo;         // long even at the largest bank
        else
        {
            for (int i = 0; i < 30; i++)
            {
                double mid = 0.5 * (lo + hi);
                if (FinalRange(mid) > 0) lo = mid; else hi = mid;
            }
            best = 0.5 * (lo + hi);
        }
        ShootConstantBank(dyn, cfg, e, x0, best, u, x);
    }

    /// <summary>Fly a constant cos(bank): u_k = lambda L/D at each node's Mach.</summary>
    static void ShootConstantBank(EntryDynamics dyn, EntryConfig cfg, double[] e, double[] x0, double lambda, double[] u, double[,] x)
    {
        int n = e.Length;
        Span<double> xa = stackalloc double[NX];
        Span<double> xb = stackalloc double[NX];
        x0.CopyTo(xa);
        for (int j = 0; j < NX; j++) x[0, j] = x0[j];
        u[0] = lambda * dyn.LiftToDrag(e[0], x0[IR]);
        for (int k = 0; k < n - 1; k++)
        {
            dyn.Propagate(e[k], e[k + 1], xa, u[k], u[k], cfg.SubSteps, xb);
            u[k + 1] = lambda * dyn.LiftToDrag(e[k + 1], xb[IR]);
            for (int j = 0; j < NX; j++) x[k + 1, j] = xb[j];
            xb.CopyTo(xa);
        }
    }

    /// <summary>Single shooting with the first-order-hold control.</summary>
    static void Shoot(EntryDynamics dyn, EntryConfig cfg, double[] e, double[] x0, double[] u, double[,] x)
    {
        int n = e.Length;
        Span<double> xa = stackalloc double[NX];
        Span<double> xb = stackalloc double[NX];
        x0.CopyTo(xa);
        for (int j = 0; j < NX; j++) x[0, j] = x0[j];
        for (int k = 0; k < n - 1; k++)
        {
            dyn.Propagate(e[k], e[k + 1], xa, u[k], u[k + 1], cfg.SubSteps, xb);
            for (int j = 0; j < NX; j++) x[k + 1, j] = xb[j];
            xb.CopyTo(xa);
        }
    }

    static void ClampToBounds(EntryDynamics dyn, double[] e, double[,] x, double[] u, double cosMin, double cosMax)
    {
        for (int k = 0; k < e.Length; k++)
        {
            double ld = dyn.LiftToDrag(e[k], x[k, IR]);
            u[k] = Math.Clamp(u[k], ld * cosMax, ld * cosMin);
        }
    }

    static double[] TrapezoidWeights(double[] e)
    {
        int n = e.Length;
        var c = new double[n];
        for (int k = 0; k < n - 1; k++)
        {
            double h = e[k + 1] - e[k];
            c[k] += 0.5 * h;
            c[k + 1] += 0.5 * h;
        }
        return c;
    }

    // ---------------------------------------------------------------- linearisation

    sealed class Linearisation(int n)
    {
        public readonly double[,,] A = new double[n - 1, NX, NX];
        public readonly double[,] Bm = new double[n - 1, NX];
        public readonly double[,] Bp = new double[n - 1, NX];
        public readonly double[,] Phi = new double[n - 1, NX];
        public readonly double[,] Defect = new double[n - 1, NX];
    }

    /// <summary>The intervals in turn, or spread over the thread pool where <see cref="EntryConfig.ParallelLinearise"/> allows it. In the game the planner keeps to its own thread: pool work there has waited seconds behind the game's own.</summary>
    static void For(EntryConfig cfg, int count, Action<int> body)
    {
        if (cfg.ParallelLinearise)
            Parallel.For(0, count, body);
        else
            for (int k = 0; k < count; k++) body(k);
    }

    static readonly double[] Hx = [1e-7, 1e-6, 1e-7];
    const double Hu = 1e-5;

    static void Linearise(EntryDynamics dyn, EntryConfig cfg, double[] e, double[,] x, double[] u, Linearisation lin)
    {
        int n = e.Length;
        For(cfg, n - 1, k =>
        {
            Span<double> xk = stackalloc double[NX];
            Span<double> xp = stackalloc double[NX];
            Span<double> fp = stackalloc double[NX];
            Span<double> fm = stackalloc double[NX];
            for (int j = 0; j < NX; j++) xk[j] = x[k, j];
            dyn.Propagate(e[k], e[k + 1], xk, u[k], u[k + 1], cfg.SubSteps, fp);
            for (int j = 0; j < NX; j++)
            {
                lin.Phi[k, j] = fp[j];
                lin.Defect[k, j] = fp[j] - x[k + 1, j];
            }
            for (int i = 0; i < NX; i++)
            {
                xk.CopyTo(xp);
                xp[i] += Hx[i];
                dyn.Propagate(e[k], e[k + 1], xp, u[k], u[k + 1], cfg.SubSteps, fp);
                xk.CopyTo(xp);
                xp[i] -= Hx[i];
                dyn.Propagate(e[k], e[k + 1], xp, u[k], u[k + 1], cfg.SubSteps, fm);
                for (int j = 0; j < NX; j++) lin.A[k, j, i] = (fp[j] - fm[j]) / (2 * Hx[i]);
            }
            dyn.Propagate(e[k], e[k + 1], xk, u[k] + Hu, u[k + 1], cfg.SubSteps, fp);
            dyn.Propagate(e[k], e[k + 1], xk, u[k] - Hu, u[k + 1], cfg.SubSteps, fm);
            for (int j = 0; j < NX; j++) lin.Bm[k, j] = (fp[j] - fm[j]) / (2 * Hu);
            dyn.Propagate(e[k], e[k + 1], xk, u[k], u[k + 1] + Hu, cfg.SubSteps, fp);
            dyn.Propagate(e[k], e[k + 1], xk, u[k], u[k + 1] - Hu, cfg.SubSteps, fm);
            for (int j = 0; j < NX; j++) lin.Bp[k, j] = (fp[j] - fm[j]) / (2 * Hu);
        });
    }

    static void Defects(EntryDynamics dyn, EntryConfig cfg, double[] e, double[,] x, double[] u, double[,] def)
    {
        int n = e.Length;
        For(cfg, n - 1, k =>
        {
            Span<double> xk = stackalloc double[NX];
            Span<double> f = stackalloc double[NX];
            for (int j = 0; j < NX; j++) xk[j] = x[k, j];
            dyn.Propagate(e[k], e[k + 1], xk, u[k], u[k + 1], cfg.SubSteps, f);
            for (int j = 0; j < NX; j++) def[k, j] = f[j] - x[k + 1, j];
        });
    }

    static double MaxScaledDefect(Linearisation lin, double[] xs, int n)
    {
        double m = 0;
        for (int k = 0; k < n - 1; k++)
            for (int j = 0; j < NX; j++) m = Math.Max(m, Math.Abs(lin.Defect[k, j]) / xs[j]);
        return m;
    }

    static double MaxScaledDefectOf(EntryDynamics dyn, EntryConfig cfg, double[] e, double[,] x, double[] u, double[] xs)
    {
        int n = e.Length;
        var def = new double[n - 1, NX];
        Defects(dyn, cfg, e, x, u, def);
        double m = 0;
        for (int k = 0; k < n - 1; k++)
            for (int j = 0; j < NX; j++) m = Math.Max(m, Math.Abs(def[k, j]) / xs[j]);
        return m;
    }

    static double MaxScaledStep(double[,] x, double[] u, double[,] xc, double[] uc, double[] xs, int n)
    {
        double m = 0;
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < NX; j++) m = Math.Max(m, Math.Abs(xc[k, j] - x[k, j]) / xs[j]);
            m = Math.Max(m, Math.Abs(uc[k] - u[k]) / US);
        }
        return m;
    }

    static void Copy(double[,] xs, double[] us, double[,] xd, double[] ud)
    {
        Array.Copy(xs, xd, xs.Length);
        Array.Copy(us, ud, us.Length);
    }

    // ---------------------------------------------------------------- merit

    /// <summary>The true cost of a trajectory: heat, smoothness and exact penalties on what it violates.</summary>
    static double Merit(EntryDynamics dyn, EntryConfig cfg, double[] e, double[,] x, double[] u, double[,] def,
                        double[] c, double qRef, double[] floor, double rfMin, double rfMax, double[] xs, double[]? uPrev, CorridorMargin corridor)
    {
        int n = e.Length;
        double heat = 0;
        for (int k = 0; k < n; k++) heat += c[k] * dyn.HeatPerEnergy(e[k], x[k, IR]);
        double j = cfg.HeatWeight * heat / qRef + ShapeCost(cfg, u, uPrev) + corridor.Cost(cfg, x, n);
        double pen = 0;
        for (int k = 0; k < n - 1; k++)
            for (int i = 0; i < NX; i++) pen += Math.Abs(def[k, i]) / xs[i];
        pen += SlackPenalty(x, floor, rfMin, rfMax, xs, n);
        return j + cfg.PenaltyWeight * pen;
    }

    /// <summary>The corridor's middle and half-width at each node, rad of range to go, for the margin term.</summary>
    sealed class CorridorMargin(double[] sShort, double[] sLong, double halfFloor)
    {
        public readonly double[] Mid = Mids(sShort, sLong);
        public readonly double[] Half = Halves(sShort, sLong, halfFloor);

        static double[] Mids(double[] a, double[] b) { var m = new double[a.Length]; for (int k = 0; k < a.Length; k++) m[k] = 0.5 * (a[k] + b[k]); return m; }

        // Floored: the corridor closes to nothing at the hand-over, and weighted by 1/half^2 the last nodes would be stiff enough that a warm re-solve could not close its defects (2026-11-21 offline: 82 of 113 refused).
        static double[] Halves(double[] a, double[] b, double floor)
        {
            var m = new double[a.Length];
            for (int k = 0; k < a.Length; k++) m[k] = Math.Max(0.5 * (b[k] - a[k]), floor);
            return m;
        }

        /// <summary>sum over the nodes before the last of ((s - mid) / half)^2 / n: zero in the middle, one per node at an edge. The last node is the hand-over, where the range to go is zero by the terminal condition.</summary>
        public double Cost(EntryConfig cfg, double[,] x, int n)
        {
            if (!(cfg.CorridorWeight > 0)) return 0;
            double j = 0;
            for (int k = 0; k < n - 1; k++)
            {
                double d = (x[k, IS] - Mid[k]) / Half[k];
                j += d * d;
            }
            return cfg.CorridorWeight * j / n;
        }
    }

    /// <summary>The range left from each node to the hand-over flying the most drag (the floors) and the glide (the lateral-reserve bank), rad: EntryPlanner.Corridor's quadrature, node by node.</summary>
    static void CorridorAt(EntryDynamics dyn, EntryConfig cfg, double[] e, double[] floor, double cosMin, out double[] sShort, out double[] sLong)
    {
        int n = e.Length;
        sShort = new double[n];
        sLong = new double[n];
        var iS = new double[n];
        var iL = new double[n];
        for (int k = 0; k < n; k++)
        {
            iS[k] = 1.0 / Math.Max(dyn.Drag(e[k], floor[k]), EntryDynamics.DragFloor);
            iL[k] = 1.0 / Math.Max(dyn.Drag(e[k], dyn.EquilibriumGlideRadius(e[k], cosMin, cfg)), EntryDynamics.DragFloor);
        }
        for (int k = n - 2; k >= 0; k--)
        {
            double h = e[k + 1] - e[k];
            sShort[k] = sShort[k + 1] + 0.5 * h * (iS[k] + iS[k + 1]);
            sLong[k] = sLong[k + 1] + 0.5 * h * (iL[k] + iL[k + 1]);
        }
    }

    /// <summary>The plan's shape terms: the change, the curvature, and the distance from the plan before (warm only), all in vertical L/D over U_s.</summary>
    static double ShapeCost(EntryConfig cfg, double[] u, double[]? uPrev)
    {
        int n = u.Length;
        double j = 0;
        for (int k = 0; k < n - 1; k++)
        {
            double du = (u[k + 1] - u[k]) / US;
            j += cfg.SmoothWeight * du * du;
        }
        for (int k = 1; k < n - 1; k++)
        {
            double d2 = (u[k + 1] - 2 * u[k] + u[k - 1]) / US;
            j += cfg.CurvatureWeight * d2 * d2;
        }
        if (uPrev != null)
            for (int k = 0; k < n; k++)
            {
                double dp = (u[k] - uPrev[k]) / US;
                j += cfg.ConsistencyWeight * dp * dp / n;
            }
        return j;
    }

    static double SlackPenalty(double[,] x, double[] floor, double rfMin, double rfMax, double[] xs, int n)
    {
        double pen = 0;
        for (int k = 0; k < n; k++) pen += Math.Max(0, floor[k] - x[k, IR]) / xs[IR];
        pen += Math.Abs(x[n - 1, IS]) / xs[IS];
        double rN = x[n - 1, IR];
        pen += (Math.Max(0, rN - rfMax) + Math.Max(0, rfMin - rN)) / xs[IR];
        return pen;
    }

    // ---------------------------------------------------------------- subproblem

    /// <summary>
    /// Variables: x (3N), u (N), w (3(N-1)), |w| bounds t (3(N-1)), floor slacks f (N), terminal range s+ s-, terminal radius slack rr.
    /// </summary>
    static (double[,], double[])? BuildAndSolve(
        EntryDynamics dyn, EntryConfig cfg, double[] e, double[] x0, double[,] xr, double[] ur, Linearisation lin,
        double[] c, double qRef, double[] floor, double rfMin, double rfMax,
        double cosMin, double cosMax, double rollRate, double[] xs, double eta, double[]? uPrev, CorridorMargin corridor, out double jLin)
    {
        jLin = double.PositiveInfinity;
        int n = e.Length;
        int nI = n - 1;
        int iX = 0, iU = NX * n, iW = iU + n, iT = iW + NX * nI, iF = iT + NX * nI;
        int iSp = iF + n, iSm = iSp + 1, iRr = iSm + 1;
        int nv = iRr + 1;
        int X(int k, int j) => iX + NX * k + j;
        int W(int k, int j) => iW + NX * k + j;
        int T(int k, int j) => iT + NX * k + j;

        var cvec = new double[nv];
        var P = new SparseCcs(nv, nv);

        // Heat load, second-order in r about the reference, normalised.
        double wq = cfg.HeatWeight / qRef;
        var heatF0 = new double[n];
        var heatF1 = new double[n];
        var heatF2 = new double[n];
        for (int k = 0; k < n; k++)
        {
            double r = xr[k, IR];
            const double dr = 2e-6;
            double f0 = dyn.HeatPerEnergy(e[k], r);
            double fP = dyn.HeatPerEnergy(e[k], r + dr), fM = dyn.HeatPerEnergy(e[k], r - dr);
            double f1 = (fP - fM) / (2 * dr);
            double f2 = Math.Max(f1 * f1 / Math.Max(f0, 1e-300), 0);   // exact for an exponential atmosphere, and never concave
            heatF0[k] = f0; heatF1[k] = f1; heatF2[k] = f2;
            cvec[X(k, IR)] += wq * c[k] * (f1 - f2 * r);
            P.Add(X(k, IR), X(k, IR), wq * c[k] * f2);
        }

        // Smoothness on u.
        double ws = cfg.SmoothWeight / (US * US);
        var pu = new double[n];
        for (int k = 0; k < n - 1; k++)
        {
            pu[k] += 2 * ws;
            pu[k + 1] += 2 * ws;
            P.Add(iU + k, iU + k + 1, -2 * ws);
        }

        // Proximal term.
        double rp = cfg.ProximalWeight;
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < NX; j++)
            {
                double s2 = xs[j] * xs[j];
                P.Add(X(k, j), X(k, j), 2 * rp / s2);
                cvec[X(k, j)] += -2 * rp * xr[k, j] / s2;
            }
            pu[k] += 2 * rp / (US * US);
            cvec[iU + k] += -2 * rp * ur[k] / (US * US);
        }
        // Curvature: (u_{k-1} - 2 u_k + u_{k+1})^2, each term's Hessian 2 w v v' with v = (1, -2, 1), upper triangle.
        double wc = cfg.CurvatureWeight / (US * US);
        for (int k = 1; k < n - 1; k++)
        {
            pu[k - 1] += 2 * wc;
            pu[k] += 8 * wc;
            pu[k + 1] += 2 * wc;
            P.Add(iU + k - 1, iU + k, -4 * wc);
            P.Add(iU + k, iU + k + 1, -4 * wc);
            P.Add(iU + k - 1, iU + k + 1, 2 * wc);
        }
        // Consistency with the plan before: (u_k - u_prev,k)^2 / n.
        if (uPrev != null)
        {
            double wp = cfg.ConsistencyWeight / (US * US * n);
            for (int k = 0; k < n; k++)
            {
                pu[k] += 2 * wp;
                cvec[iU + k] += -2 * wp * uPrev[k];
            }
        }
        for (int k = 0; k < n; k++) P.Add(iU + k, iU + k, pu[k]);
        // The corridor margin: (s_k - mid_k)^2 / half_k^2 / n on the range to go, every node before the hand-over.
        if (cfg.CorridorWeight > 0)
            for (int k = 0; k < n - 1; k++)
            {
                double w = cfg.CorridorWeight / (n * corridor.Half[k] * corridor.Half[k]);
                P.Add(X(k, IS), X(k, IS), 2 * w);
                cvec[X(k, IS)] += -2 * w * corridor.Mid[k];
            }

        // Penalties.
        double lam = cfg.PenaltyWeight;
        for (int k = 0; k < nI; k++)
            for (int j = 0; j < NX; j++) cvec[T(k, j)] = lam / xs[j];
        for (int k = 0; k < n; k++) cvec[iF + k] = lam / xs[IR];
        cvec[iSp] = lam / xs[IS];
        cvec[iSm] = lam / xs[IS];
        cvec[iRr] = lam / xs[IR];

        // ---- Equalities ----
        int nEq = NX + NX * nI + 1;
        var A = new SparseCcs(nEq, nv);
        var b = new double[nEq];
        int row = 0;
        for (int j = 0; j < NX; j++) { A.Add(row, X(0, j), 1.0); b[row++] = x0[j]; }
        for (int k = 0; k < nI; k++)
        {
            for (int j = 0; j < NX; j++)
            {
                A.Add(row, X(k + 1, j), 1.0);
                double rhs = lin.Phi[k, j];
                for (int i = 0; i < NX; i++)
                {
                    A.Add(row, X(k, i), -lin.A[k, j, i]);
                    rhs -= lin.A[k, j, i] * xr[k, i];
                }
                A.Add(row, iU + k, -lin.Bm[k, j]);
                A.Add(row, iU + k + 1, -lin.Bp[k, j]);
                rhs -= lin.Bm[k, j] * ur[k] + lin.Bp[k, j] * ur[k + 1];
                A.Add(row, W(k, j), -1.0);
                b[row++] = rhs;
            }
        }
        A.Add(row, X(n - 1, IS), 1.0);
        A.Add(row, iSp, -1.0);
        A.Add(row, iSm, 1.0);
        b[row++] = 0.0;

        // ---- Inequalities G v <= h ----
        var gRows = new List<(int Col, double Val)[]>();
        var h = new List<double>();
        void Le(double rhs, params (int, double)[] terms) { gRows.Add(terms); h.Add(rhs); }

        for (int k = 0; k < nI; k++)
            for (int j = 0; j < NX; j++)
            {
                Le(0, (W(k, j), 1.0), (T(k, j), -1.0));
                Le(0, (W(k, j), -1.0), (T(k, j), -1.0));
            }
        for (int k = 0; k < n; k++)
        {
            Le(-floor[k], (X(k, IR), -1.0), (iF + k, -1.0));
            Le(0, (iF + k, -1.0));
        }
        for (int k = 0; k < n; k++)
        {
            double ld = dyn.LiftToDrag(e[k], xr[k, IR]);
            Le(ld * cosMin, (iU + k, 1.0));
            Le(-ld * cosMax, (iU + k, -1.0));
        }
        for (int k = 0; k < nI; k++)
        {
            double eM = 0.5 * (e[k] + e[k + 1]);
            double rM = 0.5 * (xr[k, IR] + xr[k + 1, IR]);
            double dM = Math.Max(dyn.Drag(eM, rM), EntryDynamics.DragFloor);
            double vM = Math.Max(EntryDynamics.Speed(eM, rM), 1e-4);
            double dt = (e[k + 1] - e[k]) / (dM * vM);
            double du = Math.Max(dyn.LiftToDrag(eM, rM) * rollRate * dt, 1e-4);
            Le(du, (iU + k + 1, 1.0), (iU + k, -1.0));
            Le(du, (iU + k + 1, -1.0), (iU + k, 1.0));
        }
        Le(rfMax, (X(n - 1, IR), 1.0), (iRr, -1.0));
        Le(-rfMin, (X(n - 1, IR), -1.0), (iRr, -1.0));
        Le(0, (iRr, -1.0));
        Le(0, (iSp, -1.0));
        Le(0, (iSm, -1.0));
        for (int k = 1; k < n; k++)
            for (int j = 0; j < NX; j++)
            {
                Le(xr[k, j] + eta * xs[j], (X(k, j), 1.0));
                Le(-xr[k, j] + eta * xs[j], (X(k, j), -1.0));
            }
        for (int k = 0; k < n; k++)
        {
            Le(ur[k] + eta * US, (iU + k, 1.0));
            Le(-ur[k] + eta * US, (iU + k, -1.0));
        }

        var G = new SparseCcs(gRows.Count, nv);
        for (int i = 0; i < gRows.Count; i++)
            foreach (var (col, val) in gRows[i]) G.Add(i, col, val);

        var problem = new ConicProblem
        {
            C = cvec, P = P, A = A, B = b, G = G, H = [.. h],
            PositiveOrthantDim = gRows.Count,
        };
        ConicResult res;
        try { res = ClarabelSolver.Solve(problem); }
        catch (Exception) { return null; }
        if (!res.IsOptimal) return null;

        var v = res.X;
        var xc = new double[n, NX];
        var uc = new double[n];
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < NX; j++) xc[k, j] = v[X(k, j)];
            uc[k] = v[iU + k];
        }

        // The model's prediction of the merit at the candidate: heat expansion, smoothness, |w| and the slacks it chose.
        double jl = 0;
        for (int k = 0; k < n; k++)
        {
            double d = xc[k, IR] - xr[k, IR];
            jl += wq * c[k] * (heatF0[k] + heatF1[k] * d + 0.5 * heatF2[k] * d * d);
        }
        jl += ShapeCost(cfg, uc, uPrev) + corridor.Cost(cfg, xc, n);
        double pen = 0;
        for (int k = 0; k < nI; k++)
            for (int j = 0; j < NX; j++) pen += Math.Abs(v[W(k, j)]) / xs[j];
        pen += SlackPenalty(xc, floor, rfMin, rfMax, xs, n);
        jLin = jl + cfg.PenaltyWeight * pen;
        return (xc, uc);
    }

    // ---------------------------------------------------------------- output

    static EntryPlan Publish(EntryDynamics dyn, EntryConfig cfg, double[] e, double[,] x, double[] u, double[] floor,
                             double cosMin, bool ok, int iter, double maxDefect, double rangeErr, double ms, bool cold)
    {
        int n = e.Length;
        var m = dyn.Model;
        double r0 = m.BodyRadius, g0 = m.G0, vs = m.SpeedUnit;
        var R = new double[n]; var Gm = new double[n]; var S = new double[n]; var U = (double[])u.Clone();
        var D = new double[n]; var V = new double[n]; var H = new double[n]; var Hd = new double[n];
        var Ma = new double[n]; var LD = new double[n];
        var DMax = new double[n]; var DMin = new double[n]; var FloorH = new double[n];
        var invDMax = new double[n]; var invDMin = new double[n];
        for (int k = 0; k < n; k++)
        {
            R[k] = x[k, IR]; Gm[k] = x[k, IG]; S[k] = x[k, IS];
            double v = EntryDynamics.Speed(e[k], R[k]);
            V[k] = v * vs;
            H[k] = (R[k] - 1) * r0;
            D[k] = dyn.Drag(e[k], R[k]) * g0;
            Hd[k] = V[k] * Math.Sin(Gm[k]);
            Ma[k] = m.Mach(H[k], V[k]);
            LD[k] = m.Ld(Ma[k]);
            FloorH[k] = (floor[k] - 1) * r0;
            double dMax = dyn.Drag(e[k], floor[k]);
            double rEq = dyn.EquilibriumGlideRadius(e[k], cosMin, cfg);
            double dMin = dyn.Drag(e[k], rEq);
            DMax[k] = dMax * g0;
            DMin[k] = dMin * g0;
            invDMax[k] = 1.0 / Math.Max(dMax, EntryDynamics.DragFloor);
            invDMin[k] = 1.0 / Math.Max(dMin, EntryDynamics.DragFloor);
        }
        // Reachable range from each node: s' = -1/D with gamma small, integrated back from the hand-over.
        var shortM = new double[n];
        var longM = new double[n];
        for (int k = n - 2; k >= 0; k--)
        {
            double h = e[k + 1] - e[k];
            shortM[k] = shortM[k + 1] + 0.5 * h * (invDMax[k] + invDMax[k + 1]) * r0;
            longM[k] = longM[k + 1] + 0.5 * h * (invDMin[k] + invDMin[k + 1]) * r0;
        }
        return new EntryPlan
        {
            E = (double[])e.Clone(), R = R, Gamma = Gm, S = S, U = U,
            DragMs2 = D, SpeedMs = V, AltitudeM = H, HdotMs = Hd, Mach = Ma, LiftToDrag = LD,
            DragMaxMs2 = DMax, DragMinMs2 = DMin, FloorAltitudeM = FloorH,
            RangeShortM = shortM, RangeLongM = longM,
            BodyRadius = r0, Converged = ok, Iterations = iter, MaxDefect = maxDefect,
            TerminalRangeErrorM = rangeErr, SolveMs = ms, Cold = cold,
        };
    }

    // ---------------------------------------------------------------- corridor

    /// <summary>
    /// The range still reachable from energy <paramref name="e0"/> to the hand-over, m, without a solve: flying the most drag the limits allow (the altitude floors) gives the shortest, the equilibrium glide at the lateral-reserve bank the longest. Range per unit energy is 1/D with the flight-path angle small, so each is one quadrature of the model (the TRAJ display's two lines, which Publish draws the same way). It leaves out the pull-out after the entry interface and the roll between the extremes, so the real corridor is a little narrower, most at the start.
    /// </summary>
    public static (double ShortM, double LongM) Corridor(EntryModel model, EntryConfig cfg, double e0, int nodes = 40)
    {
        var dyn = new EntryDynamics(model);
        double ef = FinalEnergy(model, cfg);
        if (!(e0 < ef))
            return (0.0, 0.0);
        int n = Math.Max(nodes, 8);
        double[] e = Grid(e0, ef, 1.0 + 80_000.0 / model.BodyRadius, cfg, model.BodyRadius, n);
        double cosMin = Math.Cos(cfg.BankMinDeg * Math.PI / 180.0);
        double shortR = 0, longR = 0, prevS = 0, prevL = 0;
        for (int k = 0; k < n; k++)
        {
            double iS = 1.0 / Math.Max(dyn.Drag(e[k], dyn.FloorRadius(e[k], cfg)), EntryDynamics.DragFloor);
            double iL = 1.0 / Math.Max(dyn.Drag(e[k], dyn.EquilibriumGlideRadius(e[k], cosMin, cfg)), EntryDynamics.DragFloor);
            if (k > 0)
            {
                double h = e[k] - e[k - 1];
                shortR += 0.5 * h * (iS + prevS);
                longR += 0.5 * h * (iL + prevL);
            }
            prevS = iS;
            prevL = iL;
        }
        return (shortR * model.BodyRadius, longR * model.BodyRadius);
    }

    // ---------------------------------------------------------------- pre-entry

    /// <summary>
    /// Coast lift-up in the time domain until drag reaches the start value, and return the planner's start state there. Null if the vehicle never meets the air (or leaves it first).
    /// </summary>
    public static (double E, double[] X, double SecondsAhead)? PredictStart(
        EntryModel model, EntryConfig cfg, double radiusM, double speedMs, double gammaRad, double rangeRad)
    {
        var dyn = new EntryDynamics(model);
        double r0 = model.BodyRadius, g0 = model.G0;
        double r = radiusM / r0, v = speedMs / model.SpeedUnit, g = gammaRad, s = rangeRad;
        double startD = cfg.PlanStartDrag / g0;
        double dt = 0.5 / model.TimeUnit;
        double tMax = 4 * 3600.0 / model.TimeUnit;
        double t = 0;
        double rTop = 1.0 + model.AtmosphereTop / r0;
        bool wasInside = r < rTop;

        void Deriv(double rr, double vv, double gg, out double dr, out double dv, out double dg, out double ds)
        {
            double h = (rr - 1) * r0;
            double d = model.DragAccel(h, vv * model.SpeedUnit) / g0;
            double ld = model.Ld(model.Mach(h, vv * model.SpeedUnit)) * Math.Cos(cfg.PreStartBankDeg * Math.PI / 180.0);
            double vSafe = Math.Max(vv, 1e-4);
            model.RotationTerms(rr, vv, gg, out double vRot, out double vgRot);
            dr = vv * Math.Sin(gg);
            dv = -d - Math.Sin(gg) / (rr * rr) + vRot;
            dg = (ld * d + (vv * vv - 1.0 / rr) * Math.Cos(gg) / rr + vgRot) / vSafe;
            ds = -vv * Math.Cos(gg) / rr;
        }

        while (t < tMax)
        {
            double e = EntryDynamics.Energy(r, v);
            if (dyn.Drag(e, r) >= startD)
                return (e, [r, g, s], t * model.TimeUnit);
            if (r < 1.0) return null;
            bool inside = r < rTop;
            if (wasInside && !inside && g > 0) return null;   // skipped back out
            wasInside |= inside;

            Deriv(r, v, g, out double r1, out double v1, out double g1, out double s1);
            Deriv(r + 0.5 * dt * r1, v + 0.5 * dt * v1, g + 0.5 * dt * g1, out double r2, out double v2, out double g2, out double s2);
            Deriv(r + 0.5 * dt * r2, v + 0.5 * dt * v2, g + 0.5 * dt * g2, out double r3, out double v3, out double g3, out double s3);
            Deriv(r + dt * r3, v + dt * v3, g + dt * g3, out double r4, out double v4, out double g4, out double s4);
            r += dt / 6 * (r1 + 2 * r2 + 2 * r3 + r4);
            v += dt / 6 * (v1 + 2 * v2 + 2 * v3 + v4);
            g += dt / 6 * (g1 + 2 * g2 + 2 * g3 + g4);
            s += dt / 6 * (s1 + 2 * s2 + 2 * s3 + s4);
            t += dt;
        }
        return null;
    }
}
