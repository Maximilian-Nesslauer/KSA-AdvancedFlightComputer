namespace Gfold;

// The G-FOLD powered-descent problems (Açıkmeşe & Ploen; Blackmore), ported
// from the reference Python (GFOLD_static_p3p4.py) into ECOS standard conic
// form. Time of flight is an input — wrap with a search over tf if needed.
//
//   Problem 3 (minimum landing error): minimize ||r(tf) - rf||, final
//   altitude pinned to zero.
//   Problem 4 (minimum fuel): maximize final mass, landing point pinned to
//   the point P3 found (or any chosen reachable point).
//
// Decision variables per node n (N nodes):
//   x[6]  position r, velocity v          (leapfrog/trapezoid dynamics)
//   u[3]  thrust acceleration Tc/m
//   z     ln(mass)                         (convexified mass dynamics)
//   s     slack with ||u|| <= s            (lossless convexification)
// plus, for P3, one epigraph variable t bounding the landing error norm.
//
// Deviations from the reference, both deliberate:
//  - The glideslope cone uses the horizontal components (y,z) against
//    altitude x: ||(r-rf)_{y,z}|| <= (x-rf_x)/tan(gs). The Python's "fast"
//    line normed components [0:2] (altitude and y), which contradicts its
//    own commented-out general form.
//  - The thrust lower bound rows mirror the Python exactly (upper bound +
//    z box bounds only; the paper's quadratic lower-bound cut is omitted
//    there too, its comment notwithstanding).
// Options that change the formulation away from the literal reference. Defaults
// reproduce the reference exactly (so the Python cross-validation still holds);
// the live mod turns both on for real-time receding-horizon guidance.
public sealed record GfoldOptions
{
    // Enforce the engine's thrust FLOOR (rho1 <= ||T||), the paper's quadratic
    // lower-bound cut the reference script omits. Without it min-fuel coasts
    // then suicide-burns, so the first node has ~zero thrust — useless to fly
    // node-by-node. With it the descent thrusts continuously and node 0 is a
    // real, trackable command.
    public bool EnforceLowerThrust { get; init; }

    // Drop the "first thrust points straight up" boundary condition. That suits
    // the start of a one-shot trajectory but, re-solved every cycle, it forces
    // every commanded node-0 thrust vertical — no horizontal steering.
    public bool FreeInitialThrust { get; init; }

    // Skip the path inequalities (glideslope, velocity cap, thrust pointing) on
    // node 0. The initial state is pinned by equality, so imposing an inequality
    // it might violate (a fast/high/shallow handoff outside the glideslope cone
    // or above the speed cap) makes EVERY tf infeasible — a spurious failure.
    // The trajectory still has to satisfy the constraints from node 1 on.
    public bool RelaxInitialPath { get; init; }

    // Min-error (P3) tiebreaker: a small "prefer less fuel" weight added to the
    // landing-error objective. Pure min-error is indifferent to thrust, so with a
    // forced thrust floor the solver dumps the mandatory thrust sideways in an
    // arbitrary direction (flat, rotating, never throttling down). Among equally
    // accurate trajectories this picks the minimum-thrust one — throttle-down and
    // sensible — at no accuracy cost. Tiny by design: a pure tiebreaker.
    public double LandingFuelReg { get; init; }

    // Thrust-slew smoothing (min-fuel / P4 only): a penalty on the L2 norm of the
    // stacked thrust-vector differences ||u[n+1]-u[n]|| added to the objective, so the
    // solver spreads direction/throttle changes out over time instead of demanding
    // rapid slews the 6-DOF autopilot can't track. A soft regularizer — larger values
    // trade a little fuel for a smoother command; 0 disables it (unchanged behaviour).
    public double SlewReg { get; init; }

    public static readonly GfoldOptions Reference = new();
    public static readonly GfoldOptions RealTime = new()
    {
        EnforceLowerThrust = true, FreeInitialThrust = true, RelaxInitialPath = true,
        LandingFuelReg = 0.001,
    };

    // For committed-trajectory tracking: NO thrust floor, so the min-fuel plan can
    // coast (throttle down) where optimal and brake where needed — the caller flies
    // the whole trajectory by time index rather than node 0, so the coast arc is
    // followed instead of frozen.
    public static readonly GfoldOptions Descent = new()
    { FreeInitialThrust = true, RelaxInitialPath = true };
}

public static class GfoldPlanner
{
    public static GfoldTrajectory SolveMinError(GfoldParams p, double tf, int nodes,
                                                bool verbose = false, GfoldOptions? options = null)
        => Solve(p, tf, nodes, fixedLanding: null, verbose, options ?? GfoldOptions.Reference);

    public static GfoldTrajectory SolveMinFuel(GfoldParams p, double tf, int nodes,
                                               double[] landingPoint, bool verbose = false,
                                               GfoldOptions? options = null)
        => Solve(p, tf, nodes, fixedLanding: landingPoint, verbose, options ?? GfoldOptions.Reference);

    public sealed record SearchResult(GfoldTrajectory Trajectory, double TimeOfFlight,
                                      double FuelUsed, int Solves);

    // Search over time of flight for the minimum-fuel landing: fuel(tf) is
    // +inf where the target is unreachable (P3 misses) and U-shaped where it
    // is (too fast costs dv, too slow costs gravity losses), so a coarse
    // bracket plus golden-section refinement finds the minimum reliably.
    // Each evaluation is a P3 (reachability) + P4 (min fuel) solve pair.
    //
    // Whether the answer fits the fuel actually aboard is deliberately the
    // caller's check (FuelUsed vs params.FuelMass): the formulation has no
    // fuel-budget constraint, faithfully to the reference, and the overshoot
    // amount is useful go/no-go information.
    public static SearchResult? SearchMinFuel(GfoldParams p, int nodes,
                                              double landingToleranceM = 10.0,
                                              double tfToleranceS = 0.25,
                                              double? tfLo = null, double? tfHi = null,
                                              GfoldOptions? options = null)
    {
        GfoldOptions opt = options ?? GfoldOptions.Reference;
        int solves = 0;
        var cache = new Dictionary<double, (double Fuel, GfoldTrajectory? Traj)>();

        (double Fuel, GfoldTrajectory? Traj) Eval(double tf)
        {
            if (cache.TryGetValue(tf, out var hit))
                return hit;
            (double, GfoldTrajectory?) result;
            try
            {
                solves++;
                GfoldTrajectory p3 = SolveMinError(p, tf, nodes, options: opt);
                if (!IsUsable(p3.Status) || p3.LandingErrorNorm > landingToleranceM)
                {
                    result = (double.PositiveInfinity, null);
                }
                else
                {
                    solves++;
                    GfoldTrajectory p4 = SolveMinFuel(p, tf, nodes, p3.LandingPoint, options: opt);
                    result = IsUsable(p4.Status)
                        ? (p4.FuelUsed, p4)
                        : (double.PositiveInfinity, null);
                }
            }
            catch (ArgumentException)
            {
                result = (double.PositiveInfinity, null); // tf outside physical range
            }
            cache[tf] = result;
            return result;
        }

        // Coarse scan between the physical bounds (or a caller-supplied window,
        // e.g. to keep a terminal-descent search in a sensible range rather than
        // the full fuel-limited horizon).
        double lo = Math.Max(tfLo ?? p.TfMin, 1.0);
        double hi = Math.Min(tfHi ?? double.MaxValue, p.TfMax * 0.99);
        if (hi <= lo)
            return null;
        const int coarse = 8;
        double bestTf = double.NaN, bestFuel = double.PositiveInfinity;
        var grid = new double[coarse];
        for (int i = 0; i < coarse; i++)
        {
            grid[i] = lo + (hi - lo) * (i + 0.5) / coarse;
            double fuel = Eval(grid[i]).Fuel;
            if (fuel < bestFuel) { bestFuel = fuel; bestTf = grid[i]; }
        }
        if (double.IsInfinity(bestFuel))
            return null; // no tf reaches the target at all

        // Golden-section inside the bracket around the best coarse point.
        int k = Array.IndexOf(grid, bestTf);
        double a = k > 0 ? grid[k - 1] : lo;
        double b = k < coarse - 1 ? grid[k + 1] : hi;
        const double phi = 0.6180339887498949;
        double x1 = b - phi * (b - a);
        double x2 = a + phi * (b - a);
        double f1 = Eval(x1).Fuel;
        double f2 = Eval(x2).Fuel;
        while (b - a > tfToleranceS)
        {
            if (f1 <= f2)
            {
                b = x2; x2 = x1; f2 = f1;
                x1 = b - phi * (b - a);
                f1 = Eval(x1).Fuel;
            }
            else
            {
                a = x1; x1 = x2; f1 = f2;
                x2 = a + phi * (b - a);
                f2 = Eval(x2).Fuel;
            }
        }

        var all = cache.Where(e => e.Value.Traj != null).OrderBy(e => e.Value.Fuel).First();
        return new SearchResult(all.Value.Traj!, all.Key, all.Value.Fuel, solves);
    }

    private static bool IsUsable(EcosStatus s) =>
        s is EcosStatus.Optimal or EcosStatus.OptimalInaccurate;

    private static GfoldTrajectory Solve(GfoldParams P, double tf, int N,
                                         double[]? fixedLanding, bool verbose, GfoldOptions opt)
    {
        if (N < 4)
            throw new ArgumentException("need at least 4 nodes");
        if (tf >= P.TfMax)
            throw new ArgumentException(
                $"tf={tf:F1}s exceeds fuel-limited maximum {P.TfMax:F1}s");

        bool p3 = fixedLanding == null;
        double dtPhys = tf / (N - 1);

        // Nondimensionalize: solve in units where lengths, velocities and
        // accelerations are all O(1) (length scale ~ the problem size, time
        // scale such that gravity is ~1). ECOS's interior point breaks down
        // ("unreliable search direction") on the raw SI problem — metre-scale
        // coordinates against unit-scale ln-mass rows condition the KKT system
        // badly — and returns visibly suboptimal iterates. In scaled units it
        // converges cleanly. Mass stays in kg: every mass term enters through
        // ln(m) or the invariant combination alpha*r*t.
        double lenScale = Math.Max(1000.0, Math.Sqrt(P.R0.Sum(x => x * x)));
        double timeScale = Math.Sqrt(lenScale / P.GravityMag);
        double velScale = lenScale / timeScale;
        double accScale = lenScale / (timeScale * timeScale);

        double dt = dtPhys / timeScale;
        double alpha = P.Alpha * accScale * timeScale;   // alpha' = alpha L/T
        double r2Acc = P.R2 / accScale;                  // thrust bounds as scaled accel
        double r1Acc = P.R1 / accScale;
        double vMax = P.VMax / velScale;
        double cosPoint = Math.Cos(P.PointingMaxDeg * Math.PI / 180.0);
        double cotGs = 1.0 / Math.Tan(P.GlideSlopeDeg * Math.PI / 180.0);
        double[] g = [-P.GravityMag / accScale, 0, 0];
        double[] rf = [P.Rf[0] / lenScale, P.Rf[1] / lenScale, P.Rf[2] / lenScale];
        double[] r0 = [P.R0[0] / lenScale, P.R0[1] / lenScale, P.R0[2] / lenScale];
        double[] v0 = [P.V0[0] / velScale, P.V0[1] / velScale, P.V0[2] / velScale];
        double[] vf = [P.Vf[0] / velScale, P.Vf[1] / velScale, P.Vf[2] / velScale];
        double[]? landScaled = fixedLanding?.Select(x => x / lenScale).ToArray();

        // --- variable layout ---
        int IX(int n, int i) => n * 6 + i;        // i: 0..2 position, 3..5 velocity
        int IU(int n, int i) => 6 * N + n * 3 + i;
        int IZ(int n) => 9 * N + n;
        int IS(int n) => 10 * N + n;
        int IT = 11 * N;                          // P3 epigraph variable
        // Thrust-slew smoothing adds one variable bounding the L2 norm of the stacked
        // thrust differences. Min-fuel (P4) only, so it never distorts P3 reachability.
        // In P4 there is no IT, so its slot (11*N) is reused for the slew variable.
        bool smooth = !p3 && opt.SlewReg > 0;
        int IQ = 11 * N + (p3 ? 1 : 0);           // slew-norm variable (when smoothing)
        int nv = 11 * N + (p3 ? 1 : 0) + (smooth ? 1 : 0);

        // --- equality constraints  A x = b ---
        // Boundary rows: r0 (3) + v0 (3) + vf (3) + s_end (1) + u_end (3) +
        // z0 (1) = 14, plus u_start (3) unless the initial thrust is free, then
        // the landing rows and the dynamics.
        int pEq = 14 + (opt.FreeInitialThrust ? 0 : 3) + (p3 ? 1 : 3) + 7 * (N - 1);
        var A = new SparseCcs(pEq, nv);
        var b = new double[pEq];
        int row = 0;

        for (int i = 0; i < 3; i++) // initial position
        { A.Add(row, IX(0, i), 1); b[row++] = r0[i]; }
        for (int i = 0; i < 3; i++) // initial velocity
        { A.Add(row, IX(0, 3 + i), 1); b[row++] = v0[i]; }
        for (int i = 0; i < 3; i++) // final velocity ("don't forget to slow down, buddy!")
        { A.Add(row, IX(N - 1, 3 + i), 1); b[row++] = vf[i]; }

        A.Add(row, IS(N - 1), 1); b[row++] = 0; // thrust slack ends at zero
        // thrust starts (optional) and ends pointing straight up: u = s * (1,0,0)
        if (!opt.FreeInitialThrust)
        {
            A.Add(row, IU(0, 0), 1); A.Add(row, IS(0), -1); b[row++] = 0;
            A.Add(row, IU(0, 1), 1); b[row++] = 0;
            A.Add(row, IU(0, 2), 1); b[row++] = 0;
        }
        A.Add(row, IU(N - 1, 0), 1); A.Add(row, IS(N - 1), -1); b[row++] = 0;
        A.Add(row, IU(N - 1, 1), 1); b[row++] = 0;
        A.Add(row, IU(N - 1, 2), 1); b[row++] = 0;

        A.Add(row, IZ(0), 1); b[row++] = Math.Log(P.WetMass); // z(0) = ln(m_wet)

        if (p3)
        {
            // Reach the target altitude, floating only the horizontal landing point
            // to minimize the miss. rf_x is 0 for a ground landing and > 0 for an
            // above-the-pad arrival (Option B) — hardcoding 0 here made every P3
            // miss by the arrival altitude, so SearchMinFuel judged it unreachable.
            A.Add(row, IX(N - 1, 0), 1); b[row++] = rf[0]; // reach the target altitude
        }
        else
        {
            for (int i = 0; i < 3; i++) // land exactly where P3 landed
            { A.Add(row, IX(N - 1, i), 1); b[row++] = landScaled![i]; }
        }

        // dynamics (trapezoidal / leapfrog, constant gravity)
        for (int n = 0; n < N - 1; n++)
        {
            for (int i = 0; i < 3; i++)
            {
                // v[n+1] = v[n] + dt/2 (u[n] + u[n+1]) + dt g
                A.Add(row, IX(n + 1, 3 + i), 1);
                A.Add(row, IX(n, 3 + i), -1);
                A.Add(row, IU(n, i), -dt / 2);
                A.Add(row, IU(n + 1, i), -dt / 2);
                b[row++] = dt * g[i];
            }
            for (int i = 0; i < 3; i++)
            {
                // r[n+1] = r[n] + dt/2 (v[n] + v[n+1])
                A.Add(row, IX(n + 1, i), 1);
                A.Add(row, IX(n, i), -1);
                A.Add(row, IX(n + 1, 3 + i), -dt / 2);
                A.Add(row, IX(n, 3 + i), -dt / 2);
                b[row++] = 0;
            }
            // z[n+1] = z[n] - alpha dt/2 (s[n] + s[n+1])
            A.Add(row, IZ(n + 1), 1);
            A.Add(row, IZ(n), -1);
            A.Add(row, IS(n), alpha * dt / 2);
            A.Add(row, IS(n + 1), alpha * dt / 2);
            b[row++] = 0;
        }

        // --- cone constraints  G x + s = h,  s in R+^l x SOC(q...) ---
        // Path inequalities (pointing/glideslope/velocity) run from node p0: when
        // relaxing the initial state, skip node 0 so a fast/shallow handoff can't
        // make every tf infeasible. Thrust magnitude (lossless ||u|| <= s) and the
        // ground constraint stay on every node.
        int p0 = opt.RelaxInitialPath ? 1 : 0;
        int pathNodes = (N - 1) - p0;
        int lp = pathNodes               // thrust pointing
               + 3 * Math.Max(N - 2, 0)  // thrust upper bound + z box, n = 1..N-2
               + (N - 1);                // altitude >= 0
        int mIneq = lp
            + 3 * pathNodes // glideslope cones, Q3
            + 4 * pathNodes // velocity cones, Q4
            + 4 * (N - 1)   // thrust magnitude cones, Q4
            + (opt.EnforceLowerThrust ? 3 * Math.Max(N - 2, 0) : 0) // thrust floor cones
            + (p3 ? 4 : 0) // landing-error epigraph cone, Q4
            + (smooth ? 1 + 3 * (N - 1) : 0); // thrust-slew smoothing cone, Q(1+3(N-1))

        var G = new SparseCcs(mIneq, nv);
        var h = new double[mIneq];
        row = 0;

        // thrust pointing: cos(p_cs) s[n] - u_x[n] <= 0
        for (int n = p0; n < N - 1; n++)
        {
            G.Add(row, IS(n), cosPoint);
            G.Add(row, IU(n, 0), -1);
            h[row++] = 0;
        }

        // convexified thrust bound + ln-mass box (reference eq. 34-36)
        for (int n = 1; n < N - 1; n++)
        {
            // alpha' * r' * t' == alpha * r * t, so these mass terms are the
            // same numbers as in physical units.
            double z0Term = P.WetMass - alpha * r2Acc * n * dt;
            double z1Term = P.WetMass - alpha * r1Acc * n * dt;
            if (z0Term <= 0)
                throw new ArgumentException($"tf={tf:F1}s too long: full-throttle mass non-positive at node {n}");
            double z0 = Math.Log(z0Term);
            double z1 = Math.Log(z1Term);
            double mu2 = r2Acc / z0Term;

            // s[n] <= mu2 (1 - (z[n] - z0))
            G.Add(row, IS(n), 1);
            G.Add(row, IZ(n), mu2);
            h[row++] = mu2 * (1 + z0);
            // z0 <= z[n] <= z1
            G.Add(row, IZ(n), -1);
            h[row++] = -z0;
            G.Add(row, IZ(n), 1);
            h[row++] = z1;
        }

        // stay above ground: x[n] >= 0  ("no, this is not the Boring Company!")
        for (int n = 0; n < N - 1; n++)
        {
            G.Add(row, IX(n, 0), -1);
            h[row++] = 0;
        }

        var soc = new List<int>();

        // glideslope: ||(r - rf)_{y,z}|| <= cot(gs) (x - rf_x)
        for (int n = p0; n < N - 1; n++)
        {
            G.Add(row, IX(n, 0), -cotGs);
            h[row++] = -cotGs * rf[0];
            G.Add(row, IX(n, 1), -1);
            h[row++] = -rf[1];
            G.Add(row, IX(n, 2), -1);
            h[row++] = -rf[2];
            soc.Add(3);
        }

        // velocity: ||v[n]|| <= V_max
        for (int n = p0; n < N - 1; n++)
        {
            h[row++] = vMax; // first cone row has no variable terms
            for (int i = 0; i < 3; i++)
            {
                G.Add(row, IX(n, 3 + i), -1);
                h[row++] = 0;
            }
            soc.Add(4);
        }

        // thrust magnitude: ||u[n]|| <= s[n]
        for (int n = 0; n < N - 1; n++)
        {
            G.Add(row, IS(n), -1);
            h[row++] = 0;
            for (int i = 0; i < 3; i++)
            {
                G.Add(row, IU(n, i), -1);
                h[row++] = 0;
            }
            soc.Add(4);
        }

        // Thrust floor (continuous thrust): s[n] >= mu1*(1 - w + w^2/2), w = z - z0,
        // mu1 = rho1/z0Term. The RHS is convex quadratic in z, so it becomes a
        // rotated cone L >= u^2 (L affine, u = sqrt(mu1/2)*w) written as the SOC
        // ||(L-1, 2u)|| <= L+1, with L = s[n] + mu1*z[n] - mu1*(1+z0). Applied on
        // the same interior nodes as the box (n = 1..N-2, where z0 <= z <= z1
        // keeps w >= 0 and the Taylor cut conservative).
        if (opt.EnforceLowerThrust)
        {
            for (int n = 1; n < N - 1; n++)
            {
                double z0Term = P.WetMass - alpha * r2Acc * n * dt;
                double z0 = Math.Log(z0Term);
                double mu1 = r1Acc / z0Term;
                double k = Math.Sqrt(mu1 / 2.0);

                // cone row 0:  L + 1  = s + mu1*z + (1 - mu1*(1+z0))
                G.Add(row, IS(n), -1);
                G.Add(row, IZ(n), -mu1);
                h[row++] = 1 - mu1 * (1 + z0);
                // cone row 1:  L - 1  = s + mu1*z - (1 + mu1*(1+z0))
                G.Add(row, IS(n), -1);
                G.Add(row, IZ(n), -mu1);
                h[row++] = -1 - mu1 * (1 + z0);
                // cone row 2:  2u = 2k*(z - z0)
                G.Add(row, IZ(n), -2 * k);
                h[row++] = -2 * k * z0;
                soc.Add(3);
            }
        }

        if (p3)
        {
            // epigraph: ||r(tf) - rf|| <= t, minimized
            G.Add(row, IT, -1);
            h[row++] = 0;
            for (int i = 0; i < 3; i++)
            {
                G.Add(row, IX(N - 1, i), -1);
                h[row++] = -rf[i];
            }
            soc.Add(4);
        }

        // Thrust-slew smoothing cone: q >= || (u[1]-u[0], ..., u[N-1]-u[N-2]) ||. With
        // c[IQ] = SlewReg in the objective, minimizing pulls the stacked thrust
        // differences toward zero, spreading changes out into a gradual command.
        if (smooth)
        {
            G.Add(row, IQ, -1);
            h[row++] = 0;                          // cone top: s0 = q
            for (int n = 0; n < N - 1; n++)
                for (int i = 0; i < 3; i++)
                {
                    G.Add(row, IU(n + 1, i), -1);
                    G.Add(row, IU(n, i), 1);
                    h[row++] = 0;                  // s = u[n+1,i] - u[n,i]
                }
            soc.Add(1 + 3 * (N - 1));
        }

        // --- objective ---
        var c = new double[nv];
        if (p3)
        {
            c[IT] = 1;                          // minimize landing error norm
            if (opt.LandingFuelReg > 0)         // ... with a min-fuel tiebreaker
                c[IZ(N - 1)] = -opt.LandingFuelReg;
        }
        else
        {
            c[IZ(N - 1)] = -1;                  // maximize final ln(mass) = min fuel
        }
        if (smooth)
            c[IQ] = opt.SlewReg;                // penalize the thrust-slew L2 norm

        var problem = new EcosProblem
        {
            C = c,
            G = G,
            H = h,
            A = A,
            B = b,
            PositiveOrthantDim = lp,
            SocDims = soc.ToArray(),
        };

        EcosResult result = EcosSolver.Solve(problem, verbose);
        return Extract(result, N, dtPhys, P.Rf, lenScale, velScale, accScale);
    }

    private static GfoldTrajectory Extract(EcosResult result, int N, double dt, double[] rf,
                                           double lenScale, double velScale, double accScale)
    {
        var position = new double[N][];
        var velocity = new double[N][];
        var accel = new double[N][];
        var sigma = new double[N];
        var mass = new double[N];

        if (result.X.Length > 0)
        {
            double[] x = result.X;
            for (int n = 0; n < N; n++)
            {
                position[n] = [x[n * 6] * lenScale, x[n * 6 + 1] * lenScale, x[n * 6 + 2] * lenScale];
                velocity[n] = [x[n * 6 + 3] * velScale, x[n * 6 + 4] * velScale, x[n * 6 + 5] * velScale];
                accel[n] =
                [
                    x[6 * N + n * 3] * accScale,
                    x[6 * N + n * 3 + 1] * accScale,
                    x[6 * N + n * 3 + 2] * accScale,
                ];
                mass[n] = Math.Exp(x[9 * N + n]);
                sigma[n] = x[10 * N + n] * accScale;
            }
        }
        else
        {
            for (int n = 0; n < N; n++)
            {
                position[n] = [0, 0, 0];
                velocity[n] = [0, 0, 0];
                accel[n] = [0, 0, 0];
            }
        }

        double[] landing = result.X.Length > 0 ? position[N - 1] : [double.NaN, double.NaN, double.NaN];
        double err = Math.Sqrt(
            (landing[0] - rf[0]) * (landing[0] - rf[0]) +
            (landing[1] - rf[1]) * (landing[1] - rf[1]) +
            (landing[2] - rf[2]) * (landing[2] - rf[2]));

        return new GfoldTrajectory
        {
            Status = result.Status,
            Dt = dt,
            Position = position,
            Velocity = velocity,
            AccelCmd = accel,
            Sigma = sigma,
            Mass = mass,
            LandingPoint = landing,
            LandingErrorNorm = err,
            Iterations = result.Iterations,
        };
    }
}
