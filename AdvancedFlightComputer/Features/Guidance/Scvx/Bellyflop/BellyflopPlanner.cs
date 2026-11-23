using System.Diagnostics;
using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

/// <summary>
/// Bellyflop's planner: chooses the airflow's angles on the belly (pitch alpha, bank beta) and the yaw over a free flight time so a belly-first craft falls onto its site, as near belly-first as it can, turning the yaw only once bank is pinned.
///
///   minimise   w_m |r_xy,N| + w_e mean(alpha^2 + beta^2) + w_psi mean|psi| + w_r sum(d alpha^2 + d beta^2 + d psi^2) + penalties
///   subject to x_0 = measured; x_{k+1} = Phi(x_k, u_k, u_{k+1}, sigma) (linearised); h(r_N) = target (soft);
///              |alpha|, |beta| &lt;= limit (widened at the start to the attitude flown, closing at the rate);
///              |u_{k+1} - u_k| &lt;= rate sigma / (N - 1); u_0 within the rate of the attitude flown.
///
/// The orbital entry planner's pattern in time: RK4 multiple shooting over the free time sigma, Jacobians by central differences, virtual control and L1 slacks priced as exact penalties, an L-infinity trust region accepted on the true merit, one Clarabel QP per iteration.
/// </summary>
public sealed class BellyflopPlanner
{
    const int NX = BellyflopModel.NX, NU = BellyflopModel.NU;
    static readonly double[] XS = [1000, 1000, 1000, 10, 10, 10];
    const double Deg = Math.PI / 180.0;
    const double US = Deg;          // angles' scale: a degree
    const double UT = 5 * Deg;      // the trust region's unit for angles
    const double SS = 10.0;         // flight time's scale, s

    public sealed class Result
    {
        public BellyflopPlan? Plan { get; init; }
        public string Why { get; init; } = "";
    }

    /// <summary>
    /// Plan from the state <paramref name="x0"/> at <paramref name="now"/>, the craft flying (<paramref name="alphaNow"/>, <paramref name="betaNow"/>, <paramref name="psiNow"/>). <paramref name="warm"/>, if given, seeds the controls and flight time; otherwise the seed eases the angles to belly-first at the rate and falls to the target altitude.
    /// </summary>
    public Result Solve(BellyflopModel model, BellyflopConfig cfg, ReadOnlySpan<double> x0In, double now,
                        double alphaNow, double betaNow, double psiNow, BellyflopPlan? warm)
    {
        var sw = Stopwatch.StartNew();
        int n = Math.Max(cfg.Nodes, 6);
        double[] x0 = x0In.ToArray();
        if (model.Altitude(x0) <= cfg.TargetAltitude)
            return new Result { Why = "at or under the target altitude" };
        double rate = cfg.AngleRateDegS * Deg, yawRate = cfg.YawRateDegS * Deg;
        double angleMax = cfg.AngleMaxDeg * Deg, pitchUp = Math.Max(cfg.PitchUpMaxDeg, cfg.AngleMaxDeg) * Deg, bankMax = cfg.BankMaxDeg * Deg;

        var x = new double[n, NX];
        var u = new double[n, NU];
        double sigma;
        bool cold = warm == null || !(warm.EndTime - now > cfg.SigmaMin);
        if (!cold)
        {
            sigma = Math.Clamp(warm!.EndTime - now, cfg.SigmaMin, cfg.SigmaMax);
            for (int k = 0; k < n; k++)
            {
                var (a, b, p) = warm.ControlAt(now + sigma * k / (n - 1));
                u[k, 0] = a; u[k, 1] = b; u[k, 2] = p;
            }
            u[0, 0] = alphaNow; u[0, 1] = betaNow; u[0, 2] = psiNow;
        }
        else
        {
            sigma = SeedTime(model, cfg, x0, alphaNow, betaNow, psiNow, rate);
            for (int k = 0; k < n; k++)
            {
                double t = sigma * k / (n - 1);
                u[k, 0] = Ease(alphaNow, rate * t);
                u[k, 1] = Ease(betaNow, rate * t);
                u[k, 2] = Ease(psiNow, yawRate * t);
            }
        }
        Shoot(model, cfg, x0, u, sigma, x);

        // The angle limits at each node: the configured limit, widened at the start to an attitude flown beyond it and closing at the rate. Pitch has its own limit each way: nose up as far as the entry flies, nose down only a little.
        var limA = new double[n];
        var limADown = new double[n];
        var limB = new double[n];
        for (int k = 0; k < n; k++)
        {
            double t = sigma * k / (n - 1);
            limA[k] = Math.Max(pitchUp, alphaNow - rate * t);
            limADown[k] = Math.Max(angleMax, -alphaNow - rate * t);
            limB[k] = Math.Max(bankMax, Math.Abs(betaNow) - rate * t);
        }

        var lin = new Linearisation(n);
        double eta = cold ? 2.0 : 1.0;
        int maxIter = cold ? cfg.ColdIterations : cfg.WarmIterations;
        int iter = 0;
        double lastStep = double.PositiveInfinity;
        for (; iter < maxIter; iter++)
        {
            Linearise(model, cfg, x, u, sigma, lin);
            double defect = MaxScaledDefect(lin.Defect, n);
            if (iter > 0 && defect <= 0.25 * cfg.GateDefect && lastStep < 0.02 && Miss(x, n) <= 0.25 * cfg.GateMissM)
                break;
            double jRef = Merit(model, cfg, x, u, lin.Defect, n);
            var sub = BuildAndSolve(model, cfg, x0, x, u, sigma, lin, eta, limA, limADown, limB, rate, yawRate,
                                    alphaNow, betaNow, psiNow, out double jLin);
            if (sub == null)
            {
                eta *= 0.5;
                if (eta < 1e-3) break;
                continue;
            }
            var (xc, uc, sc) = sub.Value;
            var defC = new double[n - 1, NX];
            Defects(model, cfg, xc, uc, sc, defC);
            double jCand = Merit(model, cfg, xc, uc, defC, n);
            double predicted = jRef - jLin, actual = jRef - jCand;
            double step = MaxScaledStep(x, u, sigma, xc, uc, sc, n);
            if (predicted <= 1e-9 * (1 + Math.Abs(jRef)))
            {
                if (actual >= 0) { Copy(xc, uc, x, u); sigma = sc; }
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
            sigma = sc;
            lastStep = step;
            if (ratio < 0.25) eta *= 0.5;
            else if (ratio > 0.7) eta = Math.Min(eta * 2.0, 20.0);
        }

        // Publish the shot trajectory of the final controls: what flying them gives, defects and all.
        var xp = new double[n, NX];
        Shoot(model, cfg, x0, u, sigma, xp);
        var finalDef = new double[n - 1, NX];
        Defects(model, cfg, x, u, sigma, finalDef);
        double maxDef = MaxScaledDefect(finalDef, n);
        double miss = Miss(xp, n);
        bool ok = maxDef <= cfg.GateDefect && miss <= cfg.GateMissM;
        var plan = new BellyflopPlan
        {
            StartTime = now, Sigma = sigma, X = xp, U = (double[,])u.Clone(), Converged = ok,
            Iterations = iter, MaxDefect = maxDef, MissM = miss, SolveMs = sw.Elapsed.TotalMilliseconds, Cold = cold,
        };
        string why = ok ? "" : maxDef > cfg.GateDefect ? $"defect {maxDef:F3} over the gate" : $"misses the site by {miss / 1000:F2} km";
        return new Result { Plan = plan, Why = why };
    }

    static double Ease(double from, double by) => Math.Abs(from) <= by ? 0.0 : from - Math.Sign(from) * by;

    /// <summary>
    /// How far over the ground a belly-first fall covers to the target altitude, m, from a hand-over at <paramref name="altitude"/> (above the model's datum), <paramref name="speed"/> and flight-path angle <paramref name="gammaRad"/>, the airflow's angle on the belly easing from <paramref name="alphaStart"/> to zero at the angle rate: the range the entry leaves bellyflop to fly (EntryConfig.TerminalRangeM). The fall is flown along the model's bearing.
    /// </summary>
    public static double FallRange(BellyflopModel model, BellyflopConfig cfg, double altitude, double speed, double gammaRad, double alphaStart)
    {
        double bx = model.BearingX, by = model.BearingY;
        double[] x0 = [0, 0, altitude, speed * Math.Cos(gammaRad) * bx, speed * Math.Cos(gammaRad) * by, speed * Math.Sin(gammaRad)];
        Span<double> x = stackalloc double[NX];
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX], xt = stackalloc double[NX];
        x0.CopyTo(x);
        double rate = cfg.AngleRateDegS * Deg;
        const double h = 0.5;
        for (double t = 0; t < cfg.SigmaMax; t += h)
        {
            if (model.Altitude(x) <= cfg.TargetAltitude)
                break;
            double aA = Ease(alphaStart, rate * t), aB = Ease(alphaStart, rate * (t + h / 2)), aC = Ease(alphaStart, rate * (t + h));
            model.Derivative(x, aA, 0, 0, k1);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + 0.5 * h * k1[j];
            model.Derivative(xt, aB, 0, 0, k2);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + 0.5 * h * k2[j];
            model.Derivative(xt, aB, 0, 0, k3);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + h * k3[j];
            model.Derivative(xt, aC, 0, 0, k4);
            for (int j = 0; j < NX; j++) x[j] += h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
        }
        return Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
    }

    /// <summary>The seed's flight time: easing the angles to belly-first and falling to the target altitude, s.</summary>
    static double SeedTime(BellyflopModel model, BellyflopConfig cfg, double[] x0, double a0, double b0, double p0, double rate)
    {
        Span<double> x = stackalloc double[NX];
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX], xt = stackalloc double[NX];
        x0.CopyTo(x);
        const double h = 0.5;
        double t = 0;
        while (t < cfg.SigmaMax)
        {
            double hPrev = model.Altitude(x);
            double aA = Ease(a0, rate * t), bA = Ease(b0, rate * t), pA = Ease(p0, rate * t);
            double aB = Ease(a0, rate * (t + h / 2)), bB = Ease(b0, rate * (t + h / 2)), pB = Ease(p0, rate * (t + h / 2));
            double aC = Ease(a0, rate * (t + h)), bC = Ease(b0, rate * (t + h)), pC = Ease(p0, rate * (t + h));
            model.Derivative(x, aA, bA, pA, k1);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + 0.5 * h * k1[j];
            model.Derivative(xt, aB, bB, pB, k2);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + 0.5 * h * k2[j];
            model.Derivative(xt, aB, bB, pB, k3);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + h * k3[j];
            model.Derivative(xt, aC, bC, pC, k4);
            for (int j = 0; j < NX; j++) x[j] += h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
            t += h;
            double hNow = model.Altitude(x);
            if (hNow <= cfg.TargetAltitude)
            {
                double f = (hPrev - cfg.TargetAltitude) / Math.Max(hPrev - hNow, 1e-9);
                return Math.Clamp(t - h + f * h, cfg.SigmaMin, cfg.SigmaMax);
            }
        }
        return cfg.SigmaMax;
    }

    // ---------------------------------------------------------------- shooting

    /// <summary>One interval of RK4 over normalised time [k, k+1]/(N-1) with first-order-hold controls, the time scaled by sigma.</summary>
    static void Propagate(BellyflopModel model, int subSteps, int n, ReadOnlySpan<double> xa, ReadOnlySpan<double> ua, ReadOnlySpan<double> ub,
                          double sigma, Span<double> xb)
    {
        Span<double> x = stackalloc double[NX];
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX], xt = stackalloc double[NX];
        xa.CopyTo(x);
        double dt = sigma / (n - 1) / subSteps;
        for (int s = 0; s < subSteps; s++)
        {
            double f0 = (double)s / subSteps, fm = (s + 0.5) / subSteps, f1 = (s + 1.0) / subSteps;
            double a0 = ua[0] + f0 * (ub[0] - ua[0]), b0 = ua[1] + f0 * (ub[1] - ua[1]), p0 = ua[2] + f0 * (ub[2] - ua[2]);
            double aM = ua[0] + fm * (ub[0] - ua[0]), bM = ua[1] + fm * (ub[1] - ua[1]), pM = ua[2] + fm * (ub[2] - ua[2]);
            double a1 = ua[0] + f1 * (ub[0] - ua[0]), b1 = ua[1] + f1 * (ub[1] - ua[1]), p1 = ua[2] + f1 * (ub[2] - ua[2]);
            model.Derivative(x, a0, b0, p0, k1);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + 0.5 * dt * k1[j];
            model.Derivative(xt, aM, bM, pM, k2);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + 0.5 * dt * k2[j];
            model.Derivative(xt, aM, bM, pM, k3);
            for (int j = 0; j < NX; j++) xt[j] = x[j] + dt * k3[j];
            model.Derivative(xt, a1, b1, p1, k4);
            for (int j = 0; j < NX; j++) x[j] += dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
        }
        x.CopyTo(xb);
    }

    static void Shoot(BellyflopModel model, BellyflopConfig cfg, double[] x0, double[,] u, double sigma, double[,] x)
    {
        int n = x.GetLength(0);
        Span<double> xa = stackalloc double[NX], xb = stackalloc double[NX], ua = stackalloc double[NU], ub = stackalloc double[NU];
        x0.CopyTo(xa);
        for (int j = 0; j < NX; j++) x[0, j] = x0[j];
        for (int k = 0; k < n - 1; k++)
        {
            for (int i = 0; i < NU; i++) { ua[i] = u[k, i]; ub[i] = u[k + 1, i]; }
            Propagate(model, cfg.SubSteps, n, xa, ua, ub, sigma, xb);
            for (int j = 0; j < NX; j++) x[k + 1, j] = xb[j];
            xb.CopyTo(xa);
        }
    }

    // ---------------------------------------------------------------- linearisation

    sealed class Linearisation(int n)
    {
        public readonly double[,,] A = new double[n - 1, NX, NX];
        public readonly double[,,] Bm = new double[n - 1, NX, NU];
        public readonly double[,,] Bp = new double[n - 1, NX, NU];
        public readonly double[,] Bs = new double[n - 1, NX];
        public readonly double[,] Phi = new double[n - 1, NX];
        public readonly double[,] Defect = new double[n - 1, NX];
    }

    static readonly double[] Hx = [1.0, 1.0, 1.0, 0.01, 0.01, 0.01];
    const double Hu = 1e-4, Hs = 1e-3;

    static void Linearise(BellyflopModel model, BellyflopConfig cfg, double[,] x, double[,] u, double sigma, Linearisation lin)
    {
        int n = x.GetLength(0);
        Span<double> xk = stackalloc double[NX], xp = stackalloc double[NX], fp = stackalloc double[NX], fm = stackalloc double[NX];
        Span<double> ua = stackalloc double[NU], ub = stackalloc double[NU], up = stackalloc double[NU];
        for (int k = 0; k < n - 1; k++)
        {
            for (int j = 0; j < NX; j++) xk[j] = x[k, j];
            for (int i = 0; i < NU; i++) { ua[i] = u[k, i]; ub[i] = u[k + 1, i]; }
            Propagate(model, cfg.SubSteps, n, xk, ua, ub, sigma, fp);
            for (int j = 0; j < NX; j++)
            {
                lin.Phi[k, j] = fp[j];
                lin.Defect[k, j] = fp[j] - x[k + 1, j];
            }
            for (int i = 0; i < NX; i++)
            {
                xk.CopyTo(xp); xp[i] += Hx[i];
                Propagate(model, cfg.SubSteps, n, xp, ua, ub, sigma, fp);
                xk.CopyTo(xp); xp[i] -= Hx[i];
                Propagate(model, cfg.SubSteps, n, xp, ua, ub, sigma, fm);
                for (int j = 0; j < NX; j++) lin.A[k, j, i] = (fp[j] - fm[j]) / (2 * Hx[i]);
            }
            for (int i = 0; i < NU; i++)
            {
                ua.CopyTo(up); up[i] += Hu;
                Propagate(model, cfg.SubSteps, n, xk, up, ub, sigma, fp);
                ua.CopyTo(up); up[i] -= Hu;
                Propagate(model, cfg.SubSteps, n, xk, up, ub, sigma, fm);
                for (int j = 0; j < NX; j++) lin.Bm[k, j, i] = (fp[j] - fm[j]) / (2 * Hu);
                ub.CopyTo(up); up[i] += Hu;
                Propagate(model, cfg.SubSteps, n, xk, ua, up, sigma, fp);
                ub.CopyTo(up); up[i] -= Hu;
                Propagate(model, cfg.SubSteps, n, xk, ua, up, sigma, fm);
                for (int j = 0; j < NX; j++) lin.Bp[k, j, i] = (fp[j] - fm[j]) / (2 * Hu);
            }
            Propagate(model, cfg.SubSteps, n, xk, ua, ub, sigma + Hs, fp);
            Propagate(model, cfg.SubSteps, n, xk, ua, ub, sigma - Hs, fm);
            for (int j = 0; j < NX; j++) lin.Bs[k, j] = (fp[j] - fm[j]) / (2 * Hs);
        }
    }

    static void Defects(BellyflopModel model, BellyflopConfig cfg, double[,] x, double[,] u, double sigma, double[,] def)
    {
        int n = x.GetLength(0);
        Span<double> xk = stackalloc double[NX], f = stackalloc double[NX], ua = stackalloc double[NU], ub = stackalloc double[NU];
        for (int k = 0; k < n - 1; k++)
        {
            for (int j = 0; j < NX; j++) xk[j] = x[k, j];
            for (int i = 0; i < NU; i++) { ua[i] = u[k, i]; ub[i] = u[k + 1, i]; }
            Propagate(model, cfg.SubSteps, n, xk, ua, ub, sigma, f);
            for (int j = 0; j < NX; j++) def[k, j] = f[j] - x[k + 1, j];
        }
    }

    static double MaxScaledDefect(double[,] def, int n)
    {
        double m = 0;
        for (int k = 0; k < n - 1; k++)
            for (int j = 0; j < NX; j++) m = Math.Max(m, Math.Abs(def[k, j]) / XS[j]);
        return m;
    }

    static double MaxScaledStep(double[,] x, double[,] u, double s, double[,] xc, double[,] uc, double sc, int n)
    {
        double m = Math.Abs(sc - s) / SS;
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < NX; j++) m = Math.Max(m, Math.Abs(xc[k, j] - x[k, j]) / XS[j]);
            for (int i = 0; i < NU; i++) m = Math.Max(m, Math.Abs(uc[k, i] - u[k, i]) / UT);
        }
        return m;
    }

    static double Miss(double[,] x, int n) => Math.Sqrt(x[n - 1, 0] * x[n - 1, 0] + x[n - 1, 1] * x[n - 1, 1]);

    static void Copy(double[,] xs, double[,] us, double[,] xd, double[,] ud)
    {
        Array.Copy(xs, xd, xs.Length);
        Array.Copy(us, ud, us.Length);
    }

    // ---------------------------------------------------------------- merit

    /// <summary>The control terms: pitch and bank (mean of squares), yaw (mean of magnitudes) and the change between nodes, in degrees.</summary>
    static double ControlCost(BellyflopConfig cfg, double[,] u, int n)
    {
        double j = 0;
        for (int k = 0; k < n; k++)
        {
            double a = u[k, 0] / US, b = u[k, 1] / US, p = u[k, 2] / US;
            j += (cfg.EffortWeight * (a * a + b * b) + cfg.YawWeight * Math.Abs(p)) / n;
        }
        for (int k = 0; k < n - 1; k++)
            for (int i = 0; i < NU; i++)
            {
                double d = (u[k + 1, i] - u[k, i]) / US;
                j += cfg.RateWeight * d * d;
            }
        return j;
    }

    static double Merit(BellyflopModel model, BellyflopConfig cfg, double[,] x, double[,] u, double[,] def, int n)
    {
        double j = ControlCost(cfg, u, n);
        j += cfg.MissWeight * (Math.Abs(x[n - 1, 0]) + Math.Abs(x[n - 1, 1])) / 1000.0;
        double pen = 0;
        for (int k = 0; k < n - 1; k++)
            for (int i = 0; i < NX; i++) pen += Math.Abs(def[k, i]) / XS[i];
        Span<double> xn = stackalloc double[NX];
        for (int i = 0; i < NX; i++) xn[i] = x[n - 1, i];
        pen += Math.Abs(model.Altitude(xn) - cfg.TargetAltitude) / XS[2];
        return j + cfg.PenaltyWeight * pen;
    }

    // ---------------------------------------------------------------- subproblem

    /// <summary>
    /// Variables: x (6N), u (3N), sigma, w (6(N-1)), |w| bounds t (6(N-1)), |psi| bounds y (N), the end's x and y split into positive and negative parts (4), the end altitude's slack pair (2).
    /// </summary>
    static (double[,], double[,], double)? BuildAndSolve(
        BellyflopModel model, BellyflopConfig cfg, double[] x0, double[,] xr, double[,] ur, double sr, Linearisation lin, double eta,
        double[] limA, double[] limADown, double[] limB, double rate, double yawRate, double alphaNow, double betaNow, double psiNow, out double jLin)
    {
        jLin = double.PositiveInfinity;
        int n = xr.GetLength(0), nI = n - 1;
        int iX = 0, iU = NX * n, iS = iU + NU * n, iW = iS + 1, iT = iW + NX * nI, iY = iT + NX * nI;
        int iMx = iY + n, iMy = iMx + 2, iZa = iMy + 2;
        int nv = iZa + 2;
        int X(int k, int j) => iX + NX * k + j;
        int U(int k, int i) => iU + NU * k + i;
        int W(int k, int j) => iW + NX * k + j;
        int T(int k, int j) => iT + NX * k + j;

        var c = new double[nv];
        var P = new SparseCcs(nv, nv);
        var pd = new double[nv];

        // Pitch and bank: a mean of squares in degrees; yaw: a mean of magnitudes; the change between nodes.
        double we = cfg.EffortWeight / (n * US * US);
        for (int k = 0; k < n; k++)
        {
            pd[U(k, 0)] += 2 * we;
            pd[U(k, 1)] += 2 * we;
            c[iY + k] = cfg.YawWeight / (n * US);
        }
        double wr = cfg.RateWeight / (US * US);
        for (int k = 0; k < nI; k++)
            for (int i = 0; i < NU; i++)
            {
                pd[U(k, i)] += 2 * wr;
                pd[U(k + 1, i)] += 2 * wr;
                P.Add(U(k, i), U(k + 1, i), -2 * wr);
            }
        // Proximal.
        double rp = cfg.ProximalWeight;
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < NX; j++)
            {
                double s2 = XS[j] * XS[j];
                pd[X(k, j)] += 2 * rp / s2;
                c[X(k, j)] += -2 * rp * xr[k, j] / s2;
            }
            for (int i = 0; i < NU; i++)
            {
                pd[U(k, i)] += 2 * rp / (US * US);
                c[U(k, i)] += -2 * rp * ur[k, i] / (US * US);
            }
        }
        pd[iS] += 2 * rp / (SS * SS);
        c[iS] += -2 * rp * sr / (SS * SS);
        for (int i = 0; i < nv; i++)
            if (pd[i] != 0) P.Add(i, i, pd[i]);

        // Penalties and the miss.
        double lam = cfg.PenaltyWeight;
        for (int k = 0; k < nI; k++)
            for (int j = 0; j < NX; j++) c[T(k, j)] = lam / XS[j];
        double wm = cfg.MissWeight / 1000.0;
        c[iMx] = c[iMx + 1] = c[iMy] = c[iMy + 1] = wm;
        c[iZa] = c[iZa + 1] = lam / XS[2];

        // ---- Equalities ----
        int nEq = NX + NX * nI + 3;
        var A = new SparseCcs(nEq, nv);
        var b = new double[nEq];
        int row = 0;
        for (int j = 0; j < NX; j++) { A.Add(row, X(0, j), 1.0); b[row++] = x0[j]; }
        for (int k = 0; k < nI; k++)
            for (int j = 0; j < NX; j++)
            {
                A.Add(row, X(k + 1, j), 1.0);
                double rhs = lin.Phi[k, j];
                for (int i = 0; i < NX; i++)
                {
                    A.Add(row, X(k, i), -lin.A[k, j, i]);
                    rhs -= lin.A[k, j, i] * xr[k, i];
                }
                for (int i = 0; i < NU; i++)
                {
                    A.Add(row, U(k, i), -lin.Bm[k, j, i]);
                    A.Add(row, U(k + 1, i), -lin.Bp[k, j, i]);
                    rhs -= lin.Bm[k, j, i] * ur[k, i] + lin.Bp[k, j, i] * ur[k + 1, i];
                }
                A.Add(row, iS, -lin.Bs[k, j]);
                rhs -= lin.Bs[k, j] * sr;
                A.Add(row, W(k, j), -1.0);
                b[row++] = rhs;
            }
        // The end over the site: x_N = mx+ - mx-, y_N = my+ - my-.
        A.Add(row, X(n - 1, 0), 1.0); A.Add(row, iMx, -1.0); A.Add(row, iMx + 1, 1.0); b[row++] = 0.0;
        A.Add(row, X(n - 1, 1), 1.0); A.Add(row, iMy, -1.0); A.Add(row, iMy + 1, 1.0); b[row++] = 0.0;
        // The end at the target altitude, linearised: h(x_r) + up.(x - x_r) = target + za+ - za-.
        {
            double px = xr[n - 1, 0], py = xr[n - 1, 1], pz = xr[n - 1, 2] + model.BodyRadius;
            double pr = Math.Sqrt(px * px + py * py + pz * pz);
            double hr = pr - model.BodyRadius;
            double gx = px / pr, gy = py / pr, gz = pz / pr;
            A.Add(row, X(n - 1, 0), gx); A.Add(row, X(n - 1, 1), gy); A.Add(row, X(n - 1, 2), gz);
            A.Add(row, iZa, -1.0); A.Add(row, iZa + 1, 1.0);
            b[row++] = cfg.TargetAltitude - hr + gx * xr[n - 1, 0] + gy * xr[n - 1, 1] + gz * xr[n - 1, 2];
        }

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
            Le(0, (U(k, 2), 1.0), (iY + k, -1.0));
            Le(0, (U(k, 2), -1.0), (iY + k, -1.0));
            Le(limA[k], (U(k, 0), 1.0));
            Le(limADown[k], (U(k, 0), -1.0));
            Le(limB[k], (U(k, 1), 1.0));
            Le(limB[k], (U(k, 1), -1.0));
        }
        // The first node near the attitude flown.
        double slack = cfg.StartSlackS;
        Le(alphaNow + rate * slack, (U(0, 0), 1.0)); Le(-alphaNow + rate * slack, (U(0, 0), -1.0));
        Le(betaNow + rate * slack, (U(0, 1), 1.0)); Le(-betaNow + rate * slack, (U(0, 1), -1.0));
        Le(psiNow + yawRate * slack, (U(0, 2), 1.0)); Le(-psiNow + yawRate * slack, (U(0, 2), -1.0));
        // Rates: |u_{k+1} - u_k| <= rate sigma / (N - 1), linear in sigma.
        double dtau = 1.0 / (n - 1);
        for (int k = 0; k < nI; k++)
        {
            for (int i = 0; i < 2; i++)
            {
                Le(0, (U(k + 1, i), 1.0), (U(k, i), -1.0), (iS, -rate * dtau));
                Le(0, (U(k + 1, i), -1.0), (U(k, i), 1.0), (iS, -rate * dtau));
            }
            Le(0, (U(k + 1, 2), 1.0), (U(k, 2), -1.0), (iS, -yawRate * dtau));
            Le(0, (U(k + 1, 2), -1.0), (U(k, 2), 1.0), (iS, -yawRate * dtau));
        }
        for (int s = 0; s < 2; s++)
        {
            Le(0, (iMx + s, -1.0)); Le(0, (iMy + s, -1.0)); Le(0, (iZa + s, -1.0));
        }
        Le(cfg.SigmaMax, (iS, 1.0));
        Le(-cfg.SigmaMin, (iS, -1.0));
        // Trust region.
        for (int k = 1; k < n; k++)
            for (int j = 0; j < NX; j++)
            {
                Le(xr[k, j] + eta * XS[j], (X(k, j), 1.0));
                Le(-xr[k, j] + eta * XS[j], (X(k, j), -1.0));
            }
        for (int k = 0; k < n; k++)
            for (int i = 0; i < NU; i++)
            {
                Le(ur[k, i] + eta * UT, (U(k, i), 1.0));
                Le(-ur[k, i] + eta * UT, (U(k, i), -1.0));
            }
        Le(sr + eta * SS, (iS, 1.0));
        Le(-sr + eta * SS, (iS, -1.0));

        var G = new SparseCcs(gRows.Count, nv);
        for (int i = 0; i < gRows.Count; i++)
            foreach (var (col, val) in gRows[i]) G.Add(i, col, val);
        var problem = new ConicProblem { C = c, P = P, A = A, B = b, G = G, H = [.. h], PositiveOrthantDim = gRows.Count };
        ConicResult res;
        try { res = ClarabelSolver.Solve(problem); }
        catch (Exception) { return null; }
        if (!res.IsOptimal) return null;

        var v = res.X;
        var xc = new double[n, NX];
        var uc = new double[n, NU];
        for (int k = 0; k < n; k++)
        {
            for (int j = 0; j < NX; j++) xc[k, j] = v[X(k, j)];
            for (int i = 0; i < NU; i++) uc[k, i] = v[U(k, i)];
        }
        double sc = v[iS];
        // The model's prediction of the merit: the control terms, the miss and the penalties it chose.
        double jl = ControlCost(cfg, uc, n) + wm * (Math.Abs(xc[n - 1, 0]) + Math.Abs(xc[n - 1, 1]));
        double pen = 0;
        for (int k = 0; k < nI; k++)
            for (int j = 0; j < NX; j++) pen += Math.Abs(v[W(k, j)]) / XS[j];
        pen += (v[iZa] + v[iZa + 1]) / XS[2];
        jLin = jl + lam * pen;
        return (xc, uc, sc);
    }
}
