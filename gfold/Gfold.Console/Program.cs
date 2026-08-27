using Gfold;

// Replicates the reference Python P3_P4 flow on the Mars static test case:
// Problem 3 (minimum landing error) finds the best reachable landing point,
// Problem 4 (minimum fuel) re-solves pinned to that point.
//
//   usage: Gfold.Console [tf_seconds] [nodes] [--verbose]
//
// Writes gfold_p3.csv / gfold_p4.csv next to the executable and verifies the
// returned trajectories against the constraints (dynamics, thrust bounds,
// velocity cap, glideslope) independently of the solver's own residuals.

double tf = 81.0;
int nodes = 120;
bool verbose = args.Contains("--verbose");
double[] positional = args.Where(a => double.TryParse(a, out _)).Select(double.Parse).ToArray();
if (positional.Length > 0) tf = positional[0];
if (positional.Length > 1) nodes = (int)positional[1];

// --check <csv>: audit an externally produced trajectory (e.g. the Python
// reference's) against this implementation's constraint set — settles
// "formulation mismatch vs solver accuracy" questions decisively.
int checkIdx = Array.IndexOf(args, "--check");
if (checkIdx >= 0)
    return CheckCsv(args[checkIdx + 1], tf, new GfoldParams());

// --stress: a fast, shallow handoff state (high horizontal speed, low elevation,
// speed above the cap) that violates the node-0 path constraints — the kind that
// made the live mod fail immediately. Reference options should be infeasible;
// RealTime (relaxed initial path) should solve.
if (args.Contains("--stress"))
{
    double[] r0 = [800, 4000, 0];   // 800 m up, 4 km downrange -> ~11 deg elevation
    double[] v0 = [-20, -130, 0];   // 131 m/s, above the 90 m/s cap, nearly horizontal
    double userVMax = 90, userGlide = 30, userPoint = 45;

    // Raw (what the literal user knobs give) vs. adapted (what KsaGfold.BuildParams
    // computes: open each path constraint just enough to admit the handoff state).
    double speed = Math.Sqrt(v0.Sum(x => x * x));
    double elev = Math.Atan2(r0[0], Math.Sqrt(r0[1] * r0[1] + r0[2] * r0[2])) * 180 / Math.PI;
    double retro = Math.Acos(Math.Clamp(-v0[0] / speed, -1, 1)) * 180 / Math.PI;
    double vMaxEff = Math.Max(userVMax, 1.3 * speed);
    double glideEff = Math.Clamp(Math.Min(userGlide, elev - 5), 1, userGlide);
    double pointEff = Math.Clamp(Math.Max(userPoint, retro + 15), userPoint, 89);
    Console.WriteLine($"handoff: elev {elev:F0} deg, speed {speed:F0} m/s, retro tilt {retro:F0} deg");
    Console.WriteLine($"adapted: VMax {vMaxEff:F0}, glide {glideEff:F0}, pointing {pointEff:F0}\n");

    foreach ((string name, GfoldParams pp, GfoldOptions o) in new[]
    {
        ("Reference (raw knobs)", new GfoldParams { R0 = r0, V0 = v0, VMax = userVMax,
            GlideSlopeDeg = userGlide, PointingMaxDeg = userPoint }, GfoldOptions.Reference),
        ("RealTime (adapted)",    new GfoldParams { R0 = r0, V0 = v0, VMax = vMaxEff,
            GlideSlopeDeg = glideEff, PointingMaxDeg = pointEff }, GfoldOptions.RealTime),
    })
    {
        GfoldPlanner.SearchResult? s = GfoldPlanner.SearchMinFuel(
            pp, 60, tfLo: 4, tfHi: 120, options: o);
        Console.WriteLine(s == null
            ? $"{name,-22} -> no feasible tf"
            : $"{name,-22} -> tf {s.TimeOfFlight:F1} s, fuel {s.FuelUsed:F1} kg");
    }
    return 0;
}

// --ab: run the SAME assembled problem through ECOS and SCS and compare. This is the
// whole point of the migration branch — the two backends share GfoldPlanner's assembly,
// its nondimensionalisation and its extraction, so anything that differs here is the
// solver and nothing else.
//
// Reported per case: status, wall time, iterations (NOT comparable between an interior
// point method and a first-order one — they count different things), and the
// trajectory-level quantities the caller actually acts on. Objective agreement is table
// stakes; what decides whether SCS can replace ECOS is whether SearchMinFuel reaches
// the same time of flight, because that search gates on a 10 m landing tolerance and
// then picks between neighbouring tf values on fuel alone.
if (args.Contains("--ab"))
{
    Console.WriteLine($"ECOS {EcosSolver.NativeVersion}   SCS {ScsSolver.NativeVersion}");
    Console.WriteLine($"case: Mars static, tf={tf}s N={nodes}");
    Console.WriteLine();

    // Guard the one mechanical step of the conversion before trusting anything below:
    // a vertical stack in a column-major format is an interleave, not a concatenation,
    // and getting it wrong would hand SCS a different problem while still solving.
    if (!VStackSelfTest())
        return Fail("SparseCcs.VStack disagrees with a dense reference");
    Console.WriteLine("VStack self-test: ok");
    Console.WriteLine();

    var abP = new GfoldParams();

    (GfoldTrajectory Traj, long Ms) Run(ConicBackend backend, Func<GfoldTrajectory> f)
    {
        GfoldPlanner.Backend = backend;
        var w = System.Diagnostics.Stopwatch.StartNew();
        GfoldTrajectory t = f();
        w.Stop();
        return (t, w.ElapsedMilliseconds);
    }

    double MaxDiff(double[][] a, double[][] b) =>
        a.Zip(b, (x, y) => x.Zip(y, (u, v) => Math.Abs(u - v)).Max()).Max();

    // --- Problem 3, then Problem 4 pinned to each backend's own P3 answer ---
    var e3 = Run(ConicBackend.Ecos, () => GfoldPlanner.SolveMinError(abP, tf, nodes));
    var s3 = Run(ConicBackend.Scs, () => GfoldPlanner.SolveMinError(abP, tf, nodes));

    Console.WriteLine("P3 (minimum landing error)");
    Console.WriteLine($"  ECOS  [{e3.Traj.Status,-18}] {e3.Ms,6} ms  {e3.Traj.Iterations,7} it  " +
                      $"err {e3.Traj.LandingErrorNorm,8:F3} m  fuel {e3.Traj.FuelUsed,7:F2} kg");
    Console.WriteLine($"  SCS   [{s3.Traj.Status,-18}] {s3.Ms,6} ms  {s3.Traj.Iterations,7} it  " +
                      $"err {s3.Traj.LandingErrorNorm,8:F3} m  fuel {s3.Traj.FuelUsed,7:F2} kg");
    if (e3.Traj.IsUsable && s3.Traj.IsUsable)
    {
        Console.WriteLine($"  max |diff|: pos {MaxDiff(e3.Traj.Position, s3.Traj.Position):E2} m, " +
                          $"vel {MaxDiff(e3.Traj.Velocity, s3.Traj.Velocity):E2} m/s, " +
                          $"acc {MaxDiff(e3.Traj.AccelCmd, s3.Traj.AccelCmd):E2} m/s^2, " +
                          $"landing {Dist(e3.Traj.LandingPoint, s3.Traj.LandingPoint):E2} m");
    }

    var e4 = Run(ConicBackend.Ecos, () => GfoldPlanner.SolveMinFuel(abP, tf, nodes, e3.Traj.LandingPoint));
    var s4 = Run(ConicBackend.Scs, () => GfoldPlanner.SolveMinFuel(abP, tf, nodes, e3.Traj.LandingPoint));
    Console.WriteLine();
    Console.WriteLine("P4 (minimum fuel, both pinned to ECOS's P3 landing point)");
    Console.WriteLine($"  ECOS  [{e4.Traj.Status,-18}] {e4.Ms,6} ms  {e4.Traj.Iterations,7} it  " +
                      $"fuel {e4.Traj.FuelUsed,7:F2} kg");
    Console.WriteLine($"  SCS   [{s4.Traj.Status,-18}] {s4.Ms,6} ms  {s4.Traj.Iterations,7} it  " +
                      $"fuel {s4.Traj.FuelUsed,7:F2} kg");
    if (e4.Traj.IsUsable && s4.Traj.IsUsable)
        Console.WriteLine($"  fuel difference: {Math.Abs(e4.Traj.FuelUsed - s4.Traj.FuelUsed):F4} kg, " +
                          $"max |acc| diff {MaxDiff(e4.Traj.AccelCmd, s4.Traj.AccelCmd):E2} m/s^2");

    // --- tolerance sweep: what does accuracy cost, and where does it stop buying? ---
    Console.WriteLine();
    Console.WriteLine("SCS tolerance sweep (P4, pinned)");
    Console.WriteLine("      eps    status                 ms      iters      fuel kg   d(fuel) vs ECOS");
    foreach (double eps in new[] { 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9 })
    {
        GfoldPlanner.Backend = ConicBackend.Scs;
        GfoldPlanner.ScsEps = eps;
        var w = System.Diagnostics.Stopwatch.StartNew();
        GfoldTrajectory t = GfoldPlanner.SolveMinFuel(abP, tf, nodes, e3.Traj.LandingPoint);
        w.Stop();
        double dFuel = t.FuelUsed - e4.Traj.FuelUsed;
        Console.WriteLine($"  {eps,7:E0}    {t.Status,-18} {w.ElapsedMilliseconds,6}  {t.Iterations,9}  " +
                          $"{t.FuelUsed,10:F3}   {dFuel,10:F4}");
    }
    GfoldPlanner.ScsEps = null;

    // --- the decision-level test: does the tf search land in the same place? ---
    Console.WriteLine();
    Console.WriteLine("SearchMinFuel (the test that actually matters)");
    // Swept for SCS as well, because the search is where tolerance stops being an
    // accuracy question and becomes a pass/fail one: every tf is gated on Problem 3
    // clearing a 10 m landing tolerance and on the solve being usable at all, so a
    // tolerance tight enough to truncate rejects EVERY tf and the search reports no
    // feasible time of flight — a total failure produced purely by a settings choice.
    GfoldPlanner.Backend = ConicBackend.Ecos;
    var ew = System.Diagnostics.Stopwatch.StartNew();
    GfoldPlanner.SearchResult? eRes = GfoldPlanner.SearchMinFuel(abP, nodes);
    ew.Stop();
    Console.WriteLine(eRes == null
        ? $"  ECOS            -> no feasible tf ({ew.ElapsedMilliseconds} ms)"
        : $"  ECOS            -> tf {eRes.TimeOfFlight,6:F2} s  fuel {eRes.FuelUsed,8:F2} kg  " +
          $"{eRes.Solves,3} solves  {ew.ElapsedMilliseconds,6} ms");

    GfoldPlanner.Backend = ConicBackend.Scs;
    foreach (double eps in new[] { 1e-4, 1e-5, 1e-6, 1e-7 })
    {
        GfoldPlanner.ScsEps = eps;
        var w = System.Diagnostics.Stopwatch.StartNew();
        GfoldPlanner.SearchResult? r = GfoldPlanner.SearchMinFuel(abP, nodes);
        w.Stop();
        string tail = eRes != null && r != null
            ? $"   d(tf) {r.TimeOfFlight - eRes.TimeOfFlight,+6:F2} s  d(fuel) {r.FuelUsed - eRes.FuelUsed,+7:F2} kg"
            : "";
        Console.WriteLine(r == null
            ? $"  SCS eps {eps,7:E0} -> no feasible tf ({w.ElapsedMilliseconds} ms)"
            : $"  SCS eps {eps,7:E0} -> tf {r.TimeOfFlight,6:F2} s  fuel {r.FuelUsed,8:F2} kg  " +
              $"{r.Solves,3} solves  {w.ElapsedMilliseconds,6} ms{tail}");
    }
    GfoldPlanner.ScsEps = null;

    GfoldPlanner.Backend = ConicBackend.Ecos;
    return 0;
}

// --degen: a near-target, descending state where min-error is degenerate. Show
// that plain min-error dumps thrust sideways at the floor, while the regularized
// version (RealTime) points it up and throttles toward hover.
if (args.Contains("--degen"))
{
    var dp = new GfoldParams
    {
        R0 = [1500, 150, 0],  // 1.5 km up, ~150 m downrange of the pad
        V0 = [-30, 0, 0],     // descending 30 m/s, no horizontal motion
    };
    // The committed-tracking config: min-fuel, NO thrust floor. Confirm the profile
    // throttles down (low/zero) then brakes, with thrust pointing roughly up.
    GfoldPlanner.SearchResult? sr = GfoldPlanner.SearchMinFuel(
        dp, 40, tfLo: 4, tfHi: 120, options: GfoldOptions.Descent);
    if (sr == null) { Console.WriteLine("no feasible descent"); return 1; }
    GfoldTrajectory tr = sr.Trajectory;
    Console.WriteLine($"tf = {sr.TimeOfFlight:F1} s, fuel {sr.FuelUsed:F1} kg\n  t    alt    thr%   deg-from-vert");
    for (int i = 0; i < tr.Nodes; i += tr.Nodes / 10)
    {
        double[] u = tr.AccelCmd[i];
        double mag = Math.Sqrt(u.Sum(a => a * a));
        double thrPct = mag * tr.Mass[i] / dp.ThrustMax * 100;
        double deg = mag > 1e-6 ? Math.Acos(Math.Clamp(u[0] / mag, -1, 1)) * 180 / Math.PI : 0;
        Console.WriteLine($"  {i * tr.Dt,4:F0}  {tr.Position[i][0],5:F0}  {thrPct,5:F0}   {deg,5:F0}");
    }
    return 0;
}

// --realtime: solve min-fuel with the live-mod options (thrust floor + free
// initial direction) and report the thrust profile, to confirm continuous,
// node-0-trackable thrust (no coast arc).
if (args.Contains("--realtime"))
{
    var rp = new GfoldParams();
    GfoldTrajectory t = GfoldPlanner.SolveMinFuel(
        rp, tf, nodes, [0.0, 0.0, 0.0], options: GfoldOptions.RealTime);
    Console.WriteLine($"RealTime min-fuel [{t.Status}] fuel {t.FuelUsed:F1} kg");
    double rtMin = double.MaxValue, rtMax = 0;
    for (int i = 0; i < t.Nodes; i++)
    {
        double thr = Math.Sqrt(t.AccelCmd[i].Sum(a => a * a)) * t.Mass[i];
        rtMin = Math.Min(rtMin, thr);
        rtMax = Math.Max(rtMax, thr);
    }
    double node0 = Math.Sqrt(t.AccelCmd[0].Sum(a => a * a)) * t.Mass[0];
    Console.WriteLine($"thrust over trajectory: {rtMin:F0} .. {rtMax:F0} N (floor {rp.R1:F0}, ceil {rp.R2:F0})");
    Console.WriteLine($"node-0 thrust: {node0:F0} N  (this is what the mod commands first)");
    return node0 > rp.R1 * 0.9 ? 0 : 1;
}

// --search: find the minimum-fuel time of flight instead of using a fixed tf.
if (args.Contains("--search"))
{
    var sp = new GfoldParams();
    Console.WriteLine($"ECOS {EcosSolver.NativeVersion} | tf search in [{sp.TfMin:F1}, {sp.TfMax:F1}] s, N={nodes}");
    var ssw = System.Diagnostics.Stopwatch.StartNew();
    GfoldPlanner.SearchResult? best = GfoldPlanner.SearchMinFuel(sp, nodes);
    ssw.Stop();
    if (best == null)
        return Fail("no time of flight reaches the target");
    Console.WriteLine($"optimal tf = {best.TimeOfFlight:F2} s | fuel {best.FuelUsed:F2} kg " +
                      $"(aboard: {sp.FuelMass:F0} kg) | {best.Solves} solves in {ssw.ElapsedMilliseconds} ms");
    Console.WriteLine(best.FuelUsed <= sp.FuelMass
        ? "GO: burn fits the fuel aboard"
        : $"NO-GO: short by {best.FuelUsed - sp.FuelMass:F1} kg");
    File.WriteAllText("gfold_search.csv", best.Trajectory.ToCsv());
    Console.WriteLine("wrote gfold_search.csv");
    return 0;
}

var p = new GfoldParams();
Console.WriteLine($"ECOS {EcosSolver.NativeVersion} | tf={tf}s N={nodes} dt={tf / (nodes - 1):F3}s");
Console.WriteLine($"tf bounds: [{p.TfMin:F1}, {p.TfMax:F1}] s | wet {p.WetMass} kg dry {p.DryMass} kg");
Console.WriteLine();

// --- Problem 3: minimum landing error ---
var sw = System.Diagnostics.Stopwatch.StartNew();
GfoldTrajectory p3 = GfoldPlanner.SolveMinError(p, tf, nodes, verbose);
sw.Stop();
Console.WriteLine($"P3 [{p3.Status}] {sw.ElapsedMilliseconds} ms, {p3.Iterations} iters");
Console.WriteLine($"   landing point: ({p3.LandingPoint[0]:F2}, {p3.LandingPoint[1]:F2}, {p3.LandingPoint[2]:F2})  " +
                  $"error {p3.LandingErrorNorm:F3} m");
Console.WriteLine($"   fuel used: {p3.FuelUsed:F1} kg");
if (p3.Status is not (ConicStatus.Optimal or ConicStatus.OptimalInaccurate))
    return Fail("P3 did not solve");
File.WriteAllText("gfold_p3.csv", p3.ToCsv());

// --- Problem 4: minimum fuel to that landing point ---
sw.Restart();
GfoldTrajectory p4 = GfoldPlanner.SolveMinFuel(p, tf, nodes, p3.LandingPoint, verbose);
sw.Stop();
Console.WriteLine($"P4 [{p4.Status}] {sw.ElapsedMilliseconds} ms, {p4.Iterations} iters");
Console.WriteLine($"   fuel used: {p4.FuelUsed:F1} kg (P3 used {p3.FuelUsed:F1})");
if (p4.Status is not (ConicStatus.Optimal or ConicStatus.OptimalInaccurate))
    return Fail("P4 did not solve");
File.WriteAllText("gfold_p4.csv", p4.ToCsv());

// --- thrust-slew smoothing: SlewReg>0 exercises the new smoothing cone and should
//     cut the total thrust slew (sum of ||u[n+1]-u[n]||) at a small fuel cost. ---
double TotalSlew(GfoldTrajectory t)
{
    double s = 0;
    for (int n = 0; n < t.Nodes - 1; n++)
    {
        double dx = t.AccelCmd[n + 1][0] - t.AccelCmd[n][0];
        double dy = t.AccelCmd[n + 1][1] - t.AccelCmd[n][1];
        double dz = t.AccelCmd[n + 1][2] - t.AccelCmd[n][2];
        s += Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
    return s;
}
Console.WriteLine();
Console.WriteLine($"Thrust-slew smoothing (baseline SlewReg=0 slew {TotalSlew(p4):F2}, fuel {p4.FuelUsed:F1} kg):");
foreach (double w in new[] { 0.02, 0.1, 0.5 })
{
    GfoldTrajectory ps = GfoldPlanner.SolveMinFuel(
        p, tf, nodes, p3.LandingPoint, options: GfoldOptions.Reference with { SlewReg = w });
    Console.WriteLine($"   SlewReg={w,5}: [{ps.Status,-8}] slew {TotalSlew(ps),7:F2}   fuel {ps.FuelUsed,7:F1} kg");
}
Console.WriteLine();

// --- independent physical verification of the P4 trajectory ---
Console.WriteLine();
bool ok = true;
int n1 = p4.Nodes - 1;
double dt = p4.Dt;

// boundary conditions
ok &= Check("initial position", Dist(p4.Position[0], p.R0) < 1e-6);
ok &= Check("final position", Dist(p4.Position[n1], p3.LandingPoint) < 1e-4);
ok &= Check("final velocity", Dist(p4.Velocity[n1], p.Vf) < 1e-6);
ok &= Check("mass >= dry", p4.Mass.All(m => m >= p.DryMass - 1e-6));

// dynamics replay: trapezoidal integration of u + g must reproduce the states
double[] g = [-p.GravityMag, 0, 0];
double maxDynErr = 0;
for (int n = 0; n < n1; n++)
{
    for (int i = 0; i < 3; i++)
    {
        double vNext = p4.Velocity[n][i] + dt / 2 * (p4.AccelCmd[n][i] + p4.AccelCmd[n + 1][i]) + dt * g[i];
        double rNext = p4.Position[n][i] + dt / 2 * (p4.Velocity[n][i] + vNext + p4.Velocity[n + 1][i] - vNext);
        maxDynErr = Math.Max(maxDynErr, Math.Abs(vNext - p4.Velocity[n + 1][i]));
        maxDynErr = Math.Max(maxDynErr, Math.Abs(rNext - p4.Position[n + 1][i]));
    }
}
ok &= Check($"dynamics consistency (max err {maxDynErr:E2})", maxDynErr < 1e-6);

// path constraints (interior nodes; ends are pinned by the formulation)
double vPeak = p4.Velocity.Max(v => Math.Sqrt(v.Sum(x => x * x)));
ok &= Check($"velocity cap (peak {vPeak:F1} <= {p.VMax})", vPeak <= p.VMax + 1e-6);

double minThr = double.MaxValue, maxThr = 0;
for (int n = 1; n < n1 - 1; n++)
{
    double thrust = p4.Sigma[n] * p4.Mass[n];
    minThr = Math.Min(minThr, thrust);
    maxThr = Math.Max(maxThr, thrust);
}
Console.WriteLine($"   thrust range (interior): {minThr:F0} .. {maxThr:F0} N (bounds {p.R1:F0} .. {p.R2:F0})");
ok &= Check("thrust upper bound", maxThr <= p.R2 * 1.01);

double worstGs = 0;
double cot = 1.0 / Math.Tan(p.GlideSlopeDeg * Math.PI / 180.0);
for (int n = 0; n < n1; n++)
{
    double horiz = Math.Sqrt(Math.Pow(p4.Position[n][1] - p.Rf[1], 2) + Math.Pow(p4.Position[n][2] - p.Rf[2], 2));
    worstGs = Math.Max(worstGs, horiz - cot * (p4.Position[n][0] - p.Rf[0]));
}
ok &= Check($"glideslope (worst margin {worstGs:E2})", worstGs < 1e-4);

Console.WriteLine();
Console.WriteLine(ok ? "GFOLD P3/P4 PASS" : "GFOLD P3/P4 FAIL");
Console.WriteLine("wrote gfold_p3.csv, gfold_p4.csv");
return ok ? 0 : 1;

static double Dist(double[] a, double[] b) =>
    Math.Sqrt(a.Zip(b, (x, y) => (x - y) * (x - y)).Sum());

static bool Check(string what, bool pass)
{
    Console.WriteLine($"   {(pass ? "ok  " : "FAIL")} {what}");
    return pass;
}

/// <summary>
/// Checks SparseCcs.VStack against a dense reference, INCLUDING the Build() that
/// follows it, because that is the pair the SCS path actually depends on.
///
/// This exists because the failure it guards against is silent. A vertical stack in a
/// column-major format is an interleave within every column, not a concatenation of two
/// arrays, and getting it wrong produces a well-formed matrix describing a DIFFERENT
/// problem — which SCS will then solve, successfully, to the wrong answer. Comparing
/// backends would show a disagreement and blame the solver.
///
/// The pattern is deliberately awkward: overlapping sparsity, empty columns, an empty
/// trailing row in the top block, and out-of-order Add() calls, since Build() is what
/// sorts rows within a column and that ordering is a hard requirement of both solvers.
/// </summary>
static bool VStackSelfTest()
{
    const int pRows = 3, gRows = 4, cols = 5;
    var top = new SparseCcs(pRows, cols);
    var bottom = new SparseCcs(gRows, cols);

    // (row, col, value) triplets, added out of row order on purpose.
    (int R, int C, double V)[] topT =
        [(2, 0, 1.5), (0, 0, -2.0), (1, 2, 3.25), (0, 4, 7.0), (2, 2, -0.5)];
    (int R, int C, double V)[] botT =
        [(3, 0, 9.0), (0, 0, 4.0), (2, 1, -6.5), (1, 4, 0.125), (3, 4, 2.0), (0, 2, 11.0)];
    foreach ((int r, int c, double v) in topT) top.Add(r, c, v);
    foreach ((int r, int c, double v) in botT) bottom.Add(r, c, v);

    var expected = new double[pRows + gRows, cols];
    foreach ((int r, int c, double v) in topT) expected[r, c] += v;
    foreach ((int r, int c, double v) in botT) expected[r + pRows, c] += v;

    (double[] pr, int[] jc, int[] ir) = SparseCcs.VStack(top, bottom).Build();

    var actual = new double[pRows + gRows, cols];
    for (int j = 0; j < cols; j++)
    {
        int lastRow = -1;
        for (int k = jc[j]; k < jc[j + 1]; k++)
        {
            if (ir[k] <= lastRow)
            {
                Console.WriteLine($"  rows not ascending in column {j}: {lastRow} then {ir[k]}");
                return false;
            }
            lastRow = ir[k];
            actual[ir[k], j] += pr[k];
        }
    }
    if (jc[cols] != pr.Length)
    {
        Console.WriteLine($"  column pointer end {jc[cols]} != nnz {pr.Length}");
        return false;
    }

    for (int i = 0; i < pRows + gRows; i++)
        for (int j = 0; j < cols; j++)
            if (Math.Abs(expected[i, j] - actual[i, j]) > 1e-12)
            {
                Console.WriteLine($"  ({i},{j}): expected {expected[i, j]}, got {actual[i, j]}");
                return false;
            }
    return true;
}

static int Fail(string why)
{
    Console.WriteLine($"FAIL: {why}");
    return 1;
}

static int CheckCsv(string path, double tf, GfoldParams p)
{
    string[][] rows = File.ReadAllLines(path).Skip(1)
        .Select(l => l.Split(',')).ToArray();
    int n = rows.Length;
    // dt from the file's own time column — an externally rounded tf argument
    // shows up as a phantom dynamics violation otherwise.
    double dt = D(rows[1][0]) - D(rows[0][0]);
    _ = tf;
    double[][] r = rows.Select(c => new[] { D(c[1]), D(c[2]), D(c[3]) }).ToArray();
    double[][] v = rows.Select(c => new[] { D(c[4]), D(c[5]), D(c[6]) }).ToArray();
    double[][] u = rows.Select(c => new[] { D(c[7]), D(c[8]), D(c[9]) }).ToArray();
    double[] sig = rows.Select(c => D(c[10])).ToArray();
    double[] mass = rows.Select(c => D(c[11])).ToArray();

    Console.WriteLine($"checking {path}: N={n} dt={dt:F4} fuel={mass[0] - mass[^1]:F2} kg");
    double alpha = p.Alpha;
    double cosP = Math.Cos(p.PointingMaxDeg * Math.PI / 180.0);
    double cot = 1.0 / Math.Tan(p.GlideSlopeDeg * Math.PI / 180.0);
    double worst = 0;
    string worstWhat = "-";
    void W(string what, double violation)
    {
        if (violation > worst) { worst = violation; worstWhat = what; }
    }

    for (int k = 0; k < n - 1; k++)
    {
        for (int i = 0; i < 3; i++)
        {
            double g = i == 0 ? -p.GravityMag : 0;
            W($"dyn v n={k}", Math.Abs(v[k + 1][i] - (v[k][i] + dt / 2 * (u[k][i] + u[k + 1][i]) + dt * g)));
            W($"dyn r n={k}", Math.Abs(r[k + 1][i] - (r[k][i] + dt / 2 * (v[k][i] + v[k + 1][i]))));
        }
        double zk = Math.Log(mass[k]);
        W($"mass n={k}", Math.Abs(Math.Log(mass[k + 1]) - (zk - alpha * dt / 2 * (sig[k] + sig[k + 1]))));
        W($"|u|<=s n={k}", Math.Sqrt(u[k].Sum(x => x * x)) - sig[k]);
        W($"point n={k}", cosP * sig[k] - u[k][0]);
        W($"vmax n={k}", Math.Sqrt(v[k].Sum(x => x * x)) - p.VMax);
        W($"glide n={k}", Math.Sqrt(Math.Pow(r[k][1] - p.Rf[1], 2) + Math.Pow(r[k][2] - p.Rf[2], 2)) - cot * (r[k][0] - p.Rf[0]));
        W($"alt n={k}", -r[k][0]);
        if (k > 0 && k < n - 1)
        {
            double z0T = p.WetMass - alpha * p.R2 * k * dt;
            double z1T = p.WetMass - alpha * p.R1 * k * dt;
            double mu2 = p.R2 / z0T;
            W($"thrustub n={k}", sig[k] - mu2 * (1 - (zk - Math.Log(z0T))));
            W($"zlo n={k}", Math.Log(z0T) - zk);
            W($"zhi n={k}", zk - Math.Log(z1T));
        }
    }
    Console.WriteLine($"worst violation: {worst:E3} ({worstWhat})");
    Console.WriteLine(worst < 1e-5 ? "FEASIBLE under this formulation" : "INFEASIBLE under this formulation");
    return 0;

    static double D(string s) => double.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
}
