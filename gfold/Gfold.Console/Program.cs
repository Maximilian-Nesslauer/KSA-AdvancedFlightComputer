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
if (p3.Status is not (EcosStatus.Optimal or EcosStatus.OptimalInaccurate))
    return Fail("P3 did not solve");
File.WriteAllText("gfold_p3.csv", p3.ToCsv());

// --- Problem 4: minimum fuel to that landing point ---
sw.Restart();
GfoldTrajectory p4 = GfoldPlanner.SolveMinFuel(p, tf, nodes, p3.LandingPoint, verbose);
sw.Stop();
Console.WriteLine($"P4 [{p4.Status}] {sw.ElapsedMilliseconds} ms, {p4.Iterations} iters");
Console.WriteLine($"   fuel used: {p4.FuelUsed:F1} kg (P3 used {p3.FuelUsed:F1})");
if (p4.Status is not (EcosStatus.Optimal or EcosStatus.OptimalInaccurate))
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
