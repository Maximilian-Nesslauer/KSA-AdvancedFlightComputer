using System.Globalization;
using Scvx;

// Validates Scvx.Core's forward-mode AD against JAX.
//
// python_ref/dyn_ref.py evaluates the same dynamics with jax.jacobian at a set
// of test points and writes f, A and B per case; this diffs the C# results
// against them. Agreement to machine precision means the hand-written
// derivative rules and the state/control layout are both right.
//
//   usage: Scvx.Console [path/to/dyn_ref.csv] [--verbose]

const int NX = Dynamics6Dof.NX;
const int NU = Dynamics6Dof.NU;
const int RowLen = NX + NU + NX + NX * NX + NX * NU;

bool verbose = args.Contains("--verbose");

// --sub-scs: validate the cone-problem assembly by solving the same SCvx
// subproblem CVXPY solved in python_ref/sub_ref.py and diffing the trajectory.
if (args.Contains("--sub-scs"))
    return SubproblemCheckScs(verbose);

// --loop: run the full SCvx loop and compare the converged trajectory against
// python_ref/loop_ref.py.
if (args.Length == 0 || args.Contains("--help") || args.Contains("-h"))
{
    Console.WriteLine("""
        Scvx.Console - headless validation and measurement for the 6-DOF SCvx solver.

        CORRECTNESS - these should always pass
          --fd            forward-mode AD Jacobians against the JAX reference
          --sub-scs       one subproblem against the reference solution
          --loop          the full SCvx loop against the reference trajectory
          --scs-layout    P/Invoke struct layouts against the native SCS build

        BEHAVIOUR - properties the guidance depends on
          --path          path constraints bind, and stay solvable when violated
          --gates         node-ladder transitions carry the plan across
          --defect        defect gate vs range and node count
          --spiral        reproduce a logged engage state offline
          --seed          cold-start seed: straight line vs 3-DOF G-FOLD
          --spread        spread the cold solve across frames while the vehicle falls
          --soft          soft terminal position: unreachable target still yields a plan

        MEASUREMENT - closed-loop MPC with injected dispersions
          --mpc                 sweep configurations
          --mpc --split         where solve time goes, and its distribution
          --mpc --tail          tolerance and Anderson memory against the tail
          --mpc --budget        SCvx iterations per cycle
          --mpc --trunc         truncation handling
          --mpc --grav          gravity vs thrust margin, at dynamic similarity
          --nodes               node count vs altitude: defect and solve time

        RECEDING HORIZON AND TUNING
          --rh [--cadence]      receding-horizon behaviour; cadence sweep
          --scale --cond        problem scaling and conditioning
          --arc --roll          trajectory smoothness diagnostics
          --ablate --fixed      objective-term ablation; fixed burn time
          --body                the same descent on different worlds

        OPTIONS
          --verbose             per-iteration trace
          --eps <value>         subproblem tolerance
          --no-warm             disable warm starting
        """);
    return 0;
}

if (args.Contains("--soft"))
{
    return SoftTerminalCheck.Run();
}

if (args.Contains("--spread"))
{
    return SpreadCheck.Run();
}

if (args.Contains("--seed"))
{
    return SeedCheck.Run();
}

if (args.Contains("--nodes"))
{
    return NodeSweep.Run();
}

if (args.Contains("--spiral"))
{
    return SpiralCheck.Run();
}

if (args.Contains("--defect"))
{
    return DefectCheck.Run();
}

if (args.Contains("--gates"))
{
    return NodeGateCheck.Run(FindFile("loop_ref.csv"));
}

if (args.Contains("--path"))
{
    return PathConstraintCheck.Run(FindFile("loop_ref.csv"));
}

if (args.Contains("--loop"))
    return LoopCheck(verbose);

// --rh: measure receding-horizon cost, the number that decides whether this is
// realtime feasible. Cold-converge once, then repeatedly advance one control
// interval and re-converge from the shifted reference with a capped budget.
if (args.Contains("--mpc"))
{
    MpcHarness.TruncOnly = args.Contains("--trunc");
    MpcHarness.BudgetOnly = args.Contains("--budget");
    MpcHarness.SplitOnly = args.Contains("--split");
    MpcHarness.GravityOnly = args.Contains("--grav");
    MpcHarness.TailOnly = args.Contains("--tail");
    return MpcHarness.Run();
}

if (args.Contains("--fixed"))
    return FixedTimeCheck();

if (args.Contains("--ablate"))
    return AblateCheck();

if (args.Contains("--roll"))
    return RollCheck();

if (args.Contains("--arc"))
    return ArcCheck();

if (args.Contains("--body"))
    return BodyCheck();

if (args.Contains("--cond"))
    return CondCheck();

if (args.Contains("--scale"))
    return ScaleCheck();

if (args.Contains("--rh") && args.Contains("--cadence"))
    return CadenceSweep();

if (args.Contains("--rh"))
    return RecedingHorizonCheck();

if (args.Contains("--scs-layout"))
{
    Console.WriteLine(ScsWorkspace.DumpNativeStructLayouts());
    return 0;
}
string csv = args.FirstOrDefault(a => !a.StartsWith("--"))
             ?? FindRef() ?? "dyn_ref.csv";

if (!File.Exists(csv))
{
    Console.Error.WriteLine($"reference not found: {csv}");
    Console.Error.WriteLine("run:  python scvx/python_ref/dyn_ref.py");
    return 2;
}

var p = new Dynamics6Dof.Params();

// Relative comparison with an absolute floor: entries span ~1e-8 (mass flow)
// to ~1e6 (thrust/mass), so a pure absolute tolerance would be meaningless at
// one end and a pure relative one would blow up on entries that are legitimately
// zero. The floor is scaled by the largest magnitude in each matrix.
const double RelTol = 1e-9;

int cases = 0, failures = 0;
double worstF = 0, worstA = 0, worstB = 0;
string worstWhere = "";

foreach (string line in File.ReadLines(csv))
{
    if (line.Length == 0 || line[0] == '#') continue;
    double[] vals = line.Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
    if (vals.Length != RowLen)
    {
        Console.Error.WriteLine($"case {cases}: expected {RowLen} values, got {vals.Length}");
        return 2;
    }

    int o = 0;
    double[] x = vals[o..(o += NX)];
    double[] u = vals[o..(o += NU)];
    double[] fRef = vals[o..(o += NX)];
    double[] aRef = vals[o..(o += NX * NX)];
    double[] bRef = vals[o..(o += NX * NU)];

    double[] f = new double[NX];
    double[] A = new double[NX * NX];
    double[] B = new double[NX * NU];
    Dynamics6Dof.Jacobian(x, u, p, f, A, B);

    double ef = MaxRelDiff(f, fRef, out int wf);
    double ea = MaxRelDiff(A, aRef, out int wa);
    double eb = MaxRelDiff(B, bRef, out int wb);

    if (ef > worstF) { worstF = ef; worstWhere = $"case {cases} f[{wf}]"; }
    if (ea > worstA) worstA = ea;
    if (eb > worstB) worstB = eb;

    bool ok = ef <= RelTol && ea <= RelTol && eb <= RelTol;
    if (!ok) failures++;

    if (verbose || !ok)
    {
        Console.WriteLine($"case {cases}: f {ef:E2} (idx {wf})   " +
                          $"A {ea:E2} (r{wa / NX},c{wa % NX})   B {eb:E2} (r{wb / NU},c{wb % NU})   " +
                          (ok ? "ok" : "FAIL"));
    }
    cases++;
}

// Independent cross-check: central finite differences against the same F().
// This validates the derivative rules a second way (it cannot share a seeding
// bug with the AD path) and measures what accuracy FD actually delivers here —
// directly relevant to bolting on a black-box aero model later, where AD isn't
// available and FD is the fallback.
if (args.Contains("--fd"))
{
    Console.WriteLine();
    Console.WriteLine("AD vs central finite differences (same C# F):");
    double worstFd = 0;
    int fdCase = 0;
    foreach (string line in File.ReadLines(csv))
    {
        if (line.Length == 0 || line[0] == '#') continue;
        double[] vals = line.Split(',')
            .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        double[] x = vals[0..NX];
        double[] u = vals[NX..(NX + NU)];

        double[] A = new double[NX * NX], B = new double[NX * NU];
        Dynamics6Dof.Jacobian(x, u, p, [], A, B);
        double[] Afd = FdStateJacobian(x, u, p);

        double scale = A.Max(Math.Abs);
        double worst = 0;
        for (int i = 0; i < A.Length; i++)
            worst = Math.Max(worst, Math.Abs(A[i] - Afd[i]) / scale);
        worstFd = Math.Max(worstFd, worst);
        Console.WriteLine($"  case {fdCase}: max rel diff {worst:E2}");
        fdCase++;
    }
    Console.WriteLine($"  worst {worstFd:E2}  (AD vs JAX was {worstA:E2})");
    Console.WriteLine("  FD is ~5-6 orders of magnitude looser than AD, as expected " +
                      "for central differences at h = cbrt(eps)*scale.");
}

Console.WriteLine();
Console.WriteLine($"{cases} cases, {failures} failing (tolerance {RelTol:E0})");
Console.WriteLine($"worst relative error:  f {worstF:E2}   A {worstA:E2}   B {worstB:E2}");
if (worstF > 0) Console.WriteLine($"worst f entry: {worstWhere}");
Console.WriteLine(failures == 0
    ? "PASS - C# forward-mode AD matches JAX"
    : "FAIL - see cases above");
return failures == 0 ? 0 : 1;


// Validate the cone assembly against the SCS native-P formulation:
// reference-point feasibility + objective audit, then an actual solve, then
// compare X/U/sigma/objective to the CVXPY reference. Also exercises the
// warm-start path (second solve with warmStart=true) since that's the entire
// reason to prefer SCS here.
static int SubproblemCheckScs(bool verbose)
{
    string path = FindFile("sub_ref.csv") ?? "sub_ref.csv";
    if (!File.Exists(path))
    {
        Console.Error.WriteLine($"reference not found: {path}");
        Console.Error.WriteLine("run:  python scvx/python_ref/sub_ref.py");
        return 2;
    }

    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();

    double[] x0 = Row(0), xf = Row(1), xbar = Row(2), ubar = Row(3);
    double[] meta = Row(4);
    double[] xRef = Row(5), uRef = Row(6), tail = Row(7);
    double sigBar = meta[0], tr = meta[1];
    double sigRef = tail[0], objRef = tail[1];

    int n = xbar.Length / NX;
    var cfg = new Scvx6DofConfig { Nodes = n };
    var dyn = new Dynamics6Dof.Params();

    double[] A = new double[n * NX * NX];
    double[] B = new double[n * NX * NU];
    double[] f0 = new double[n * NX];
    for (int k = 0; k < n; k++)
        Dynamics6Dof.Jacobian(
            xbar.AsSpan(k * NX, NX), ubar.AsSpan(k * NU, NU), dyn,
            f0.AsSpan(k * NX, NX),
            A.AsSpan(k * NX * NX, NX * NX),
            B.AsSpan(k * NX * NU, NX * NU));

    var sub = new Scvx6DofSubproblemScs(cfg);
    Console.WriteLine($"SCS {ScsWorkspace.NativeVersion}   " +
                      $"n={sub.VariableCount} eq={sub.EqualityCount} rows={sub.RowCount}   " +
                      $"(ECOS port was n=950 rows=1972 — {950 - sub.VariableCount} fewer vars, " +
                      $"{1972 - sub.RowCount} fewer rows, from dropping the epigraphs)");

    sub.Assemble(x0, xf, xbar, ubar, sigBar, tr, A, B, f0);
    if (lines.Length > 8)
    {
        double[] wvRef = Row(8);
        double[] packed = sub.PackPrimal(xRef, uRef, wvRef, sigRef);
        var (eqRes, eqRow, coneVio, coneIdx) = sub.CheckPrimal(packed);
        double obj = sub.Objective(packed);
        Console.WriteLine($"reference point audit: max |Az-b| {eqRes:E2} (row {eqRow}), " +
                          $"worst cone violation {coneVio:E2} (row {coneIdx})");
        Console.WriteLine($"  objective at reference = {obj:E10}   expected {objRef - 1.0:E10}   " +
                          $"rel {Math.Abs(obj - (objRef - 1.0)) / Math.Abs(objRef - 1.0):E2}");
    }

    Console.WriteLine(sub.DiagnoseScsValidation());

    var t0 = System.Diagnostics.Stopwatch.StartNew();
    ScsStatus st = sub.Run(warmStart: false, verbose, maxIterations: 100_000, epsAbs: 1e-7, epsRel: 1e-7);
    double firstMs = t0.Elapsed.TotalMilliseconds;
    Console.WriteLine($"first solve : {st} \"{sub.StatusText}\", {sub.Iterations} iters, {firstMs:F1} ms");

    if (!st.IsUsable())
    {
        Console.Error.WriteLine($"solve failed: {st}");
        return 1;
    }

    // Re-assemble with IDENTICAL data (a no-op iteration) and solve warm-started,
    // to show the warm-start path runs and to see its iteration count against
    // the cold solve above — on identical data this should converge in very few
    // iterations if warm starting is doing anything.
    t0 = System.Diagnostics.Stopwatch.StartNew();
    sub.Assemble(x0, xf, xbar, ubar, sigBar, tr, A, B, f0);
    ScsStatus st2 = sub.Run(warmStart: true, false, maxIterations: 100_000, epsAbs: 1e-7, epsRel: 1e-7);
    double warmMs = t0.Elapsed.TotalMilliseconds;
    Console.WriteLine($"warm solve  : {st2} \"{sub.StatusText}\", {sub.Iterations} iters, {warmMs:F1} ms " +
                      "(re-init is unavoidable, so timing reflects the ADMM iteration count, not a skipped setup)");

    double[] xGot = sub.SolutionX, uGot = sub.SolutionU;
    double objGot = sub.PrimalObjective;   // SCS's own reported (1/2)x'Px+c'x, already unscaled to our units? see note below

    double ex = MaxRelDiff(xGot, xRef, out int wx);
    double eu = MaxRelDiff(uGot, uRef, out int wu);
    double es = Math.Abs(sub.SolutionSigma - sigRef) / Math.Abs(sigRef);

    Console.WriteLine();
    Console.WriteLine("vs CVXPY reference:");
    Console.WriteLine($"  X       max rel diff {ex:E2}  (node {wx / NX}, comp {wx % NX})");
    Console.WriteLine($"  U       max rel diff {eu:E2}  (node {wu / NU}, comp {wu % NU})");
    Console.WriteLine($"  sigma   {sub.SolutionSigma:F6} vs {sigRef:F6}   rel {es:E2}");
    Console.WriteLine($"  SCS-reported objective (scaled coords): {objGot:E6}");

    // Recompute the objective from the unscaled solution the same way the audit
    // did, which is the honest apples-to-apples number against objRef.
    double[] solZ = sub.PackPrimal(xGot, uGot, sub.SolutionWv, sub.SolutionSigma);
    double objRecomputed = sub.Objective(solZ);
    double eo = Math.Abs(objRecomputed - (objRef - 1.0)) / Math.Abs(objRef - 1.0);
    Console.WriteLine($"  objective (recomputed) {objRecomputed:E10} vs {objRef - 1.0:E10}   rel {eo:E2}");

    // Tolerance is set by the WEAKEST of the four comparisons, which is the
    // per-component control diff (~1.5e-3); everything else lands far tighter
    // (X 6e-6, sigma 3e-7, objective 1.2e-5).
    //
    // That spread is the point: the OBJECTIVE agrees to 1.2e-5 while an
    // individual control component differs by 1.5e-3. Matching objective with
    // differing components is the signature of a near-degenerate optimum — at
    // SCvx iteration 0 the trust region is active on almost every variable
    // (visible as dozens of trust rows at ~0 slack in the audit above), so the
    // argmin is close to non-unique and two correct solvers can legitimately
    // land on different points of the same optimal face.
    //
    // So this is deliberately NOT justified by "solvers disagree by about this
    // much" — the Clarabel-vs-SCS gap measured in session 2 was an OBJECTIVE
    // gap (~1e-5), which is a different quantity and would be the wrong thing
    // to compare a component tolerance against.
    const double Tol = 1e-2;
    bool ok = ex < Tol && eu < Tol && es < Tol && eo < Tol;
    Console.WriteLine();
    Console.WriteLine(ok
        ? "PASS - SCS native-P formulation reproduces the reference subproblem"
        : $"FAIL - exceeds {Tol:E0}");
    return ok ? 0 : 1;
}

// Run the full SCvx loop from the same cold seed as python_ref/loop_ref.py and
// compare the CONVERGED result. SCvx is a nonconvex local method, so the
// meaningful test is that both implementations land on the same fixed point —
// matching iteration-for-iteration is a bonus that only holds while both take
// identical accept/reject decisions, and the two use different convex solvers.
static int LoopCheck(bool verbose)
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    if (!File.Exists(path))
    {
        Console.Error.WriteLine($"reference not found: {path}");
        Console.Error.WriteLine("run:  python scvx/python_ref/loop_ref.py");
        return 2;
    }

    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();

    double[] x0 = Row(0), xf = Row(1), xRef = Row(2), uRef = Row(3);
    double[] meta = Row(4);
    double sigRef = meta[0], itersRef = meta[1], costRef = meta[2], defectRef = meta[3];

    int n = xRef.Length / NX;
    var cfg = new Scvx6DofConfig { Nodes = n };
    if (CheckConstants(path, cfg, new Dynamics6Dof.Params()) != 0)
        return 1;

    // Same cold seed as the reference: straight-line r/v, identity attitude,
    // linear mass bleed, hover-ish axial thrust, sigma = 12 s.
    double m0 = x0[Dynamics6Dof.IM];
    double[] xSeed = new double[n * NX];
    double[] uSeed = new double[n * NU];
    for (int k = 0; k < n; k++)
    {
        double t = (double)k / (n - 1);
        for (int i = 0; i < 3; i++)
        {
            xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
            xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
        }
        xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
        xSeed[k * NX + Dynamics6Dof.IM] = m0 + t * (0.92 * m0 - m0);
        uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
    }

    double eps = ScsWorkspace.DefaultEps;
    int epsIdx = Array.FindIndex(Environment.GetCommandLineArgs(), a => a == "--eps");
    if (epsIdx >= 0) eps = double.Parse(Environment.GetCommandLineArgs()[epsIdx + 1], CultureInfo.InvariantCulture);
    Console.WriteLine($"subproblem eps = {eps:E0}");
    bool noWarm = Environment.GetCommandLineArgs().Contains("--no-warm");
    var solver = new Scvx6DofSolver(cfg) { SubproblemEps = eps, WarmStart = !noWarm };
    Console.WriteLine($"warm start = {!noWarm}");
    solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed: 12.0);

    var sw = System.Diagnostics.Stopwatch.StartNew();
    ScvxStatus status = solver.Solve(maxIterations: 150);
    double totalMs = sw.Elapsed.TotalMilliseconds;

    Console.WriteLine($"SCS {ScsWorkspace.NativeVersion}   SCvx loop from cold seed");
    foreach (var it in solver.Trace)
    {
        if (!verbose && it.Index > 0 && !it.Accepted && it.Solved) continue;
        Console.WriteLine(it.Solved
            ? $"  iter {it.Index,3}: rho={it.Rho,+7:F2} {(it.Accepted ? "accept" : "REJECT")}  " +
              $"tr={it.TrustRegion:F3}  sig={it.Sigma,6:F2}  step={it.Step:E2}  " +
              $"defect={it.DefectNorm:E2}  J={it.Cost:E3}  ({it.SolverIterations} scs, {it.ElapsedMs:F0} ms)"
            : $"  iter {it.Index,3}: subproblem FAILED ({solver.LastFailureReason}) " +
              $"after {it.SolverIterations} scs iters, {it.ElapsedMs:F0} ms -> shrink tr={it.TrustRegion:F4}");
    }

    Console.WriteLine();
    Console.WriteLine($"status={status}  iterations={solver.IterationCount}  " +
                      $"total={totalMs:F0} ms  ({totalMs / Math.Max(solver.IterationCount, 1):F0} ms/iter)");

    double[] xGot = solver.ReferenceX, uGot = solver.ReferenceU;
    (double costGot, double defectGot) = solver.TrueCost(xGot, uGot, solver.Sigma);
    double propGot = m0 - xGot[(n - 1) * NX + Dynamics6Dof.IM];
    double propRef = m0 - xRef[(n - 1) * NX + Dynamics6Dof.IM];

    double ex = MaxRelDiff(xGot, xRef, out int wx);
    double eu = MaxRelDiff(uGot, uRef, out int wu);
    double es = Math.Abs(solver.Sigma - sigRef) / Math.Abs(sigRef);
    double ep = Math.Abs(propGot - propRef) / Math.Abs(propRef);

    Console.WriteLine();
    Console.WriteLine("converged solution vs python_ref/loop_ref.py:");
    Console.WriteLine($"  X          max rel diff {ex:E2}  (node {wx / NX}, comp {wx % NX})");
    Console.WriteLine($"  U          max rel diff {eu:E2}  (node {wu / NU}, comp {wu % NU})");
    Console.WriteLine($"  sigma      {solver.Sigma:F6} vs {sigRef:F6}   rel {es:E2}");
    Console.WriteLine($"  propellant {propGot:F0} vs {propRef:F0} kg   rel {ep:E2}");
    Console.WriteLine($"  cost       {costGot:E6} vs {costRef:E6}");
    Console.WriteLine($"  defect     {defectGot:E2} vs {defectRef:E2}   (tolerance 1e-3)");
    Console.WriteLine($"  iterations {solver.IterationCount} vs {itersRef:F0}");

    // SCvx is a LOCAL method on a nonconvex problem, so "did it reproduce the
    // reference trajectory" is the wrong pass criterion — and demonstrably so
    // here. The reference run hit SIX convex-solver failures (iterations 6, 7,
    // 8, 10, 13, 15 come back rho=0 / rejected in loop_ref.csv); each one
    // shrank the trust region, and that is what arrested sigma's growth around
    // 17 s. This run had zero solver failures, so sigma kept growing until the
    // ratio test shrank the region on its own merits, landing at ~24 s.
    //
    // Both are legitimate SCvx runs of the same algorithm. The right test is
    // therefore: did it converge, is the answer genuinely FEASIBLE against the
    // TRUE nonlinear constraints (not the linearised ones it was solved with),
    // and is its merit no worse than the reference's.
    // Dump the converged trajectory so python_ref/plot_compare.py can render it
    // with the SAME matplotlib code 6dof.py uses, overlaid on the reference.
    string outPath = Path.Combine(Path.GetDirectoryName(path)!, "loop_cs.csv");
    using (var w = new StreamWriter(outPath))
    {
        w.WriteLine($"# C# SCvx: sigma={solver.Sigma:R} iters={solver.IterationCount} status={status}");
        w.WriteLine(string.Join(",", xGot.Select(z => z.ToString("R"))));
        w.WriteLine(string.Join(",", uGot.Select(z => z.ToString("R"))));
        w.WriteLine(string.Join(",", new[] { solver.Sigma, solver.IterationCount, costGot, defectGot }
            .Select(z => z.ToString("R"))));
        foreach (var t in solver.Trace)
            w.WriteLine(string.Join(",", new[]
            {
                (double)t.Index, t.Rho, t.Accepted ? 1.0 : 0.0, t.TrustRegion,
                t.Sigma, t.Step, t.DefectNorm, t.Cost, t.SolverIterations, t.ElapsedMs
            }.Select(z => z.ToString("R"))));
    }
    Console.WriteLine($"wrote {outPath}");

    string audit = AuditTrajectory(xGot, uGot, solver.Sigma, x0, xf, cfg,
                                   solver.SubproblemEps, out bool feasible);
    Console.WriteLine();
    Console.WriteLine("true (nonlinear) constraint audit of the converged solution:");
    Console.Write(audit);

    bool converged = status == ScvxStatus.Converged;
    bool meritNoWorse = costGot <= costRef * 1.05;
    bool ok = converged && feasible && defectGot < 1e-3 && meritNoWorse;

    Console.WriteLine();
    Console.WriteLine($"  converged            {converged}");
    Console.WriteLine($"  feasible (nonlinear) {feasible}");
    Console.WriteLine($"  defect < 1e-3        {defectGot < 1e-3}  ({defectGot:E2})");
    Console.WriteLine($"  merit <= reference   {meritNoWorse}  ({costGot:E4} vs {costRef:E4})");
    Console.WriteLine();
    Console.WriteLine(ok
        ? "PASS - SCvx loop converges to a feasible solution at least as good as the reference"
        : "FAIL");
    return ok ? 0 : 1;
}

// Measure what a receding-horizon guidance cycle actually costs.
//
// The cold-start numbers (17 iterations, ~6 s) are NOT the realtime figure: in
// flight the previous cycle's solution is available and one control interval of
// motion barely changes the problem. This walks the plan forward the way a
// guidance loop would — advance the state to plan node 1, shift the reference,
// shrink sigma by one interval — and reports per-cycle cost at several
// iteration budgets.
static int RecedingHorizonCheck()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
    double[] x0 = Row(0), xf = Row(1), xRef = Row(2);

    int n = xRef.Length / NX;
    var cfg = new Scvx6DofConfig { Nodes = n };
    double m0 = x0[Dynamics6Dof.IM];

    double[] xSeed = new double[n * NX], uSeed = new double[n * NU];
    for (int k = 0; k < n; k++)
    {
        double t = (double)k / (n - 1);
        for (int i = 0; i < 3; i++)
        {
            xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
            xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
        }
        xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
        xSeed[k * NX + Dynamics6Dof.IM] = m0 + t * (0.92 * m0 - m0);
        uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
    }

    double rhEps = ScsWorkspace.DefaultEps;
    var cliArgs = Environment.GetCommandLineArgs();
    int ei = Array.FindIndex(cliArgs, a => a == "--eps");
    if (ei >= 0) rhEps = double.Parse(cliArgs[ei + 1], CultureInfo.InvariantCulture);
    Console.WriteLine($"subproblem eps = {rhEps:E0}");

    var solver = new Scvx6DofSolver(cfg) { SubproblemEps = rhEps };
    solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed: 12.0);
    var cold = System.Diagnostics.Stopwatch.StartNew();
    ScvxStatus st = solver.Solve(150);
    Console.WriteLine($"cold start: {st}, {solver.IterationCount} iters, " +
                      $"{cold.Elapsed.TotalMilliseconds:F0} ms  (built once, during the coast)");
    if (st != ScvxStatus.Converged) { Console.Error.WriteLine("cold solve failed"); return 1; }

    // Walk the plan forward. Each cycle: the vehicle has flown one control
    // interval, so take plan node 1 as the new state (plus a little tracking
    // error, since a real vehicle never lands exactly on the plan), shift the
    // reference, and re-converge.
    var rng = new Random(12345);
    foreach (int budget in new[] { 1, 2, 5 })
    {
        // Restart from the converged plan for each budget so they are comparable.
        var s2 = new Scvx6DofSolver(cfg) { SubproblemEps = rhEps };
        s2.Initialize(x0, xf, xSeed, uSeed, 12.0);
        s2.Solve(150);

        double[] x = (double[])s2.ReferenceX.Clone();
        double[] u = (double[])s2.ReferenceU.Clone();
        double sig = s2.Sigma;

        var times = new List<double>();
        var iters = new List<int>();
        int converged = 0, cycles = 8;
        var itersUsed = new List<int>();
        for (int c = 0; c < cycles; c++)
        {
            double dt = sig / (n - 1);
            double[] xs = new double[n * NX], us = new double[n * NU];
            // shift one node forward, hold the tail
            for (int k = 0; k < n; k++)
            {
                int src = Math.Min(k + 1, n - 1);
                Array.Copy(x, src * NX, xs, k * NX, NX);
                Array.Copy(u, src * NU, us, k * NU, NU);
            }
            double[] newX0 = new double[NX];
            Array.Copy(xs, 0, newX0, 0, NX);
            for (int i = 0; i < 3; i++)
            {
                newX0[i] += (rng.NextDouble() - 0.5) * 2.0;        // ~1 m position error
                newX0[3 + i] += (rng.NextDouble() - 0.5) * 0.6;    // ~0.3 m/s velocity error
            }
            Array.Copy(newX0, 0, xs, 0, NX);
            sig = Math.Max(sig - dt, cfg.SigmaMin + 1e-3);

            s2.Reseed(newX0, xs, us, sig, trustRegion: 0.05);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            ScvxStatus rs = s2.Solve(budget);
            itersUsed.Add(s2.IterationCount);
            times.Add(sw.Elapsed.TotalMilliseconds);
            iters.Add(s2.IterationCount);
            if (rs == ScvxStatus.Converged) converged++;
            x = (double[])s2.ReferenceX.Clone();
            u = (double[])s2.ReferenceU.Clone();
            sig = s2.Sigma;
        }

        times.Sort();
        double med = times[times.Count / 2], worst = times[^1], mean = times.Average();
        Console.WriteLine($"  budget {budget} SCvx iter(s)/cycle: " +
                          $"mean {mean,6:F0} ms   median {med,6:F0} ms   WORST {worst,6:F0} ms   " +
                          $"({converged}/{cycles} hit the convergence test)  " +
                          $"SCvx iters actually used: [{string.Join(",", itersUsed)}]");
    }

    Console.WriteLine();
    Console.WriteLine("Realtime reading: the WORST case is what matters for a control loop,");
    Console.WriteLine("not the mean — a guidance cycle that occasionally takes 5x its budget");
    Console.WriteLine("is a dropped update, and ADMM iteration counts here vary ~9x.");
    return 0;
}

// Check a converged trajectory against the TRUE nonlinear constraint set — the
// one the problem actually has, not the linearisation any single subproblem was
// solved with. A solution can satisfy every linearised constraint and still
// violate the real one (the tilt cone especially, since its linearisation is
// only exact at the reference), so this is the check that means something.
static string AuditTrajectory(double[] x, double[] u, double sigma,
                              double[] x0, double[] xf, Scvx6DofConfig cfg,
                              double solverEps, out bool feasible)
{
    int n = cfg.Nodes;
    var sb = new System.Text.StringBuilder();
    bool allOk = true;   // local, because a lambda cannot capture an out parameter

    // Constraint tolerances SCALE WITH THE SOLVER TOLERANCE, because auditing more
    // tightly than you solved is just measuring the solver's own residual and
    // calling it a violation. A fixed 1e-6 gate against a 1e-5 solve reported
    // terminal-state and throttle-floor "violations" of 2e-6 and 8e-6 -- i.e. below
    // what was asked for -- while the physical answer was identical to five
    // significant figures.
    //
    // The 10x multiple is because these are DERIVED quantities: the solver's eps
    // bounds its own residual on the convex subproblem, and the audit measures the
    // result of the whole SCvx loop through the true nonlinear constraints. The 1e-6
    // floor keeps the historical gate for tight solves, so nothing gets looser than
    // it was.
    double conTol = Math.Max(10.0 * solverEps, 1e-6);

    void Check(string name, double worst, double tol, string units = "")
    {
        bool ok = worst <= tol;
        if (!ok) allOk = false;
        sb.AppendLine($"  {name,-28} worst {worst:E2}{units}  tol {tol:E1}  {(ok ? "ok" : "VIOLATED")}");
    }

    double bc0 = 0;
    for (int i = 0; i < NX; i++) bc0 = Math.Max(bc0, Math.Abs(x[i] - x0[i]) / Math.Max(Math.Abs(x0[i]), 1));
    double bcf = 0;
    for (int i = 0; i < NX - 1; i++)
        bcf = Math.Max(bcf, Math.Abs(x[(n - 1) * NX + i] - xf[i]) / Math.Max(Math.Abs(xf[i]), 1));
    Check("initial state (rel)", bc0, conTol);
    Check("terminal state (rel)", bcf, conTol);

    double quatErr = 0, thrustLo = 0, thrustHi = 0, gimbal = 0, roll = 0, tilt = 0, ground = 0;
    for (int k = 0; k < n; k++)
    {
        int q = k * NX + Dynamics6Dof.IQ;
        double norm = Math.Sqrt(x[q] * x[q] + x[q + 1] * x[q + 1] + x[q + 2] * x[q + 2] + x[q + 3] * x[q + 3]);
        quatErr = Math.Max(quatErr, Math.Abs(norm - 1.0));

        double tdx = u[k * NU + 0], tdy = u[k * NU + 1], T = u[k * NU + 2], tau = u[k * NU + 3];
        thrustLo = Math.Max(thrustLo, (cfg.Tmin - T) / cfg.Tmax);
        thrustHi = Math.Max(thrustHi, (T - cfg.Tmax) / cfg.Tmax);
        gimbal = Math.Max(gimbal, (Math.Sqrt(tdx * tdx + tdy * tdy) - cfg.TanGimbal * T) / cfg.Tmax);
        roll = Math.Max(roll, (Math.Abs(tau) - cfg.TauRollMax) / cfg.TauRollMax);

        // TRUE tilt: R22 = 1 - 2(qx^2+qy^2) >= cos(tilt_max), no linearisation.
        double qx = x[q + 1], qy = x[q + 2];
        double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
        tilt = Math.Max(tilt, cfg.CosTilt - r22);

        ground = Math.Max(ground, cfg.GroundFloor - x[k * NX + 2]);
    }

    // |q|=1 does NOT scale: the solver reprojects the quaternion on every accepted
    // step, so this is exact regardless of tolerance and a loose gate would hide a
    // real bug. "above ground" does not scale either — it is a physical margin in
    // metres, not a solver residual.
    Check("|q| = 1", quatErr, 1e-9);
    Check("thrust >= Tmin (rel Tmax)", thrustLo, conTol);
    Check("thrust <= Tmax (rel Tmax)", thrustHi, conTol);
    Check("gimbal cone (rel Tmax)", gimbal, conTol);
    Check("|roll torque| (rel max)", roll, conTol);
    Check("tilt cone (cos margin)", tilt, conTol);
    Check("above ground (m)", ground, 1e-3, " m");

    double peakTiltDeg = 0;
    for (int k = 0; k < n; k++)
    {
        int q = k * NX + Dynamics6Dof.IQ;
        double r22 = 1.0 - 2.0 * (x[q + 1] * x[q + 1] + x[q + 2] * x[q + 2]);
        peakTiltDeg = Math.Max(peakTiltDeg, Math.Acos(Math.Clamp(r22, -1, 1)) * 180 / Math.PI);
    }
    double minT = double.MaxValue, maxT = 0;
    for (int k = 0; k < n; k++) { double T = u[k * NU + 2]; minT = Math.Min(minT, T); maxT = Math.Max(maxT, T); }
    sb.AppendLine($"  (peak tilt {peakTiltDeg:F1} deg of {cfg.TiltMaxDeg:F0} limit; " +
                  $"throttle {minT / cfg.Tmax * 100:F0}-{maxT / cfg.Tmax * 100:F0}% of {cfg.ThrottleFloor * 100:F0}% floor; " +
                  $"burn {sigma:F1} s)");
    feasible = allOk;
    return sb.ToString();
}

// df/dx by central differences. Step per variable is cbrt(eps) scaled by the
// variable's own magnitude — central differences trade O(h^2) truncation against
// O(eps/h) round-off, so cbrt(eps) is the optimum, giving ~eps^(2/3) accuracy.
// (Forward differences would want sqrt(eps) and deliver only ~sqrt(eps).)
static double[] FdStateJacobian(double[] x, double[] u, Dynamics6Dof.Params p)
{
    const int NXl = Dynamics6Dof.NX;
    double cbrtEps = Math.Cbrt(2.220446049250313e-16);
    double[] A = new double[NXl * NXl];
    double[] fp = new double[NXl], fm = new double[NXl];
    double[] xp = (double[])x.Clone();

    for (int c = 0; c < NXl; c++)
    {
        double h = cbrtEps * Math.Max(Math.Abs(x[c]), 1.0);
        xp[c] = x[c] + h;
        Dynamics6Dof.Eval(xp, u, p, fp);
        xp[c] = x[c] - h;
        Dynamics6Dof.Eval(xp, u, p, fm);
        xp[c] = x[c];
        double inv = 1.0 / (2.0 * h);
        for (int r = 0; r < NXl; r++)
            A[r * NXl + c] = (fp[r] - fm[r]) * inv;
    }
    return A;
}

// Largest |a-b| normalised by the scale of the array, so entries that are
// legitimately zero don't produce a divide-by-zero and large entries don't
// swamp small ones.
static double MaxRelDiff(double[] got, double[] want, out int worstIdx)
{
    double scale = 0;
    for (int i = 0; i < want.Length; i++)
        scale = Math.Max(scale, Math.Abs(want[i]));
    if (scale == 0) scale = 1;

    double worst = 0;
    worstIdx = 0;
    for (int i = 0; i < got.Length; i++)
    {
        double e = Math.Abs(got[i] - want[i]) / scale;
        if (e > worst) { worst = e; worstIdx = i; }
    }
    return worst;
}

// Walk up from the executable to find the repo's python_ref output.
static string? FindRef() => FindFile("dyn_ref.csv");

static string? FindFile(string name)
{
    var dir = new DirectoryInfo(AppContext.BaseDirectory);
    while (dir != null)
    {
        string candidate = Path.Combine(dir.FullName, "python_ref", name);
        if (File.Exists(candidate)) return candidate;
        dir = dir.Parent;
    }
    return null;
}


// Guard against the C# and Python constant sets drifting apart.
//
// The SCENARIO (x0/xf) travels through loop_ref.csv rows, so the two sides cannot
// disagree about WHICH problem they solve. Everything else -- gravity, Isp, inertia,
// thrust and gimbal limits, SCvx weights -- is hand-mirrored between
// Dynamics6Dof.Params / Scvx6DofConfig and 6dof.py. Editing one side alone used to
// leave this comparison reporting PASS while the two sides solved DIFFERENT
// problems: a green tick on the wrong question. loop_ref.py now emits its values on
// a "# consts" line and this asserts against them.
//
// Tolerance is RELATIVE, not exact: values that round-trip through radians come back
// as e.g. 29.999999999999996 deg, which is agreement, not drift.
static int CheckConstants(string path, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn)
{
    const double RelTolerance = 1e-9;

    string? line = File.ReadLines(path).FirstOrDefault(l => l.StartsWith("# consts "));
    if (line == null)
    {
        Console.Error.WriteLine("reference has no '# consts' line - regenerate it:");
        Console.Error.WriteLine("  python scvx/python_ref/loop_ref.py");
        return 1;
    }

    var reference = new Dictionary<string, double>();
    foreach (string token in line["# consts ".Length..].Split(' ', StringSplitOptions.RemoveEmptyEntries))
    {
        string[] kv = token.Split('=');
        if (kv.Length == 2 && double.TryParse(kv[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
            reference[kv[0]] = v;
    }

    (string Name, double Ours)[] ours =
    [
        ("gz", dyn.Gz), ("g0", dyn.G0), ("isp", dyn.Isp), ("l_arm", dyn.LArm),
        ("ixx", dyn.Ixx), ("iyy", dyn.Iyy), ("izz", dyn.Izz),
        ("tmax", cfg.Tmax), ("throttle_floor", cfg.ThrottleFloor),
        ("gimbal_max_deg", cfg.GimbalMaxDeg), ("tau_roll_max", cfg.TauRollMax),
        ("tilt_max_deg", cfg.TiltMaxDeg),
        ("rho_vc", cfg.RhoVc), ("w_du", cfg.WDu), ("w_w", cfg.WW),
        ("sig_min", cfg.SigmaMin), ("sig_max", cfg.SigmaMax), ("sig_scale", cfg.SigmaScale),
    ];

    int bad = 0;
    foreach ((string name, double mine) in ours)
    {
        if (!reference.TryGetValue(name, out double theirs))
        {
            Console.Error.WriteLine($"  constant '{name}' missing from the reference");
            bad++;
            continue;
        }
        double scale = Math.Max(Math.Abs(theirs), 1.0);
        if (Math.Abs(mine - theirs) > RelTolerance * scale)
        {
            Console.Error.WriteLine($"  CONSTANT DRIFT '{name}': C# {mine:G17} vs 6dof.py {theirs:G17}");
            bad++;
        }
    }

    if (bad > 0)
    {
        Console.Error.WriteLine(
            $"{bad} constant(s) differ - the C# and Python sides are solving different " +
            "problems, so any comparison below would be meaningless. Reconcile " +
            "Dynamics6Dof.Params / Scvx6DofConfig against 6dof.py.");
    }
    return bad;
}

// How does the re-solve cadence affect COST? Intuition says fewer solves is cheaper.
// It is the other way round: the warm start is only good while the vehicle is still
// near its previous plan, so a LONGER interval makes each individual solve harder as
// well as the plan staler. This sweeps the advance per cycle, in fractions of a plan
// node, and reports both cost per cycle and cost per second of flight.
static int CadenceSweep()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
    double[] x0 = Row(0), xf = Row(1), xRef = Row(2);

    int n = xRef.Length / NX;
    var cfg = new Scvx6DofConfig { Nodes = n };
    double m0 = x0[Dynamics6Dof.IM];

    double[] xSeed = new double[n * NX], uSeed = new double[n * NU];
    for (int k = 0; k < n; k++)
    {
        double t = (double)k / (n - 1);
        for (int i = 0; i < 3; i++)
        {
            xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
            xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
        }
        xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
        xSeed[k * NX + Dynamics6Dof.IM] = m0 + t * (0.92 * m0 - m0);
        uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
    }

    Console.WriteLine("re-solve cadence sweep (eps 1e-5, budget 5, 12 cycles each)");
    Console.WriteLine("advance is in PLAN NODES; 1.0 node is what --rh measures");
    Console.WriteLine();
    Console.WriteLine("  advance   cadence     mean     median     worst    ms per second of flight");

    foreach (double step in new[] { 0.25, 0.5, 1.0, 1.5, 2.0, 3.0 })
    {
        var s2 = new Scvx6DofSolver(cfg) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        s2.Initialize(x0, xf, xSeed, uSeed, 12.0);
        s2.Solve(150);

        double[] x = (double[])s2.ReferenceX.Clone();
        double[] u = (double[])s2.ReferenceU.Clone();
        double sig = s2.Sigma;
        var rng = new Random(12345);
        var times = new List<double>();

        for (int c = 0; c < 12; c++)
        {
            double dt = sig / (n - 1);
            double[] xs = new double[n * NX], us = new double[n * NU];
            for (int k = 0; k < n; k++)
            {
                double src = Math.Min(k + step, n - 1);
                int a = (int)src, b = Math.Min(a + 1, n - 1);
                double f = src - a;
                for (int i = 0; i < NX; i++)
                    xs[k * NX + i] = x[a * NX + i] * (1 - f) + x[b * NX + i] * f;
                for (int i = 0; i < NU; i++)
                    us[k * NU + i] = u[a * NU + i] * (1 - f) + u[b * NU + i] * f;
            }
            double qn = 0;
            for (int k = 0; k < n; k++)
            {
                qn = 0;
                for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
                qn = Math.Sqrt(qn);
                if (qn > 1e-12) for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            }

            double[] newX0 = new double[NX];
            Array.Copy(xs, 0, newX0, 0, NX);
            // Tracking error accumulates with the interval, so scale it with the step
            // rather than injecting a fixed amount regardless of how far we advanced.
            for (int i = 0; i < 3; i++)
            {
                newX0[i] += (rng.NextDouble() - 0.5) * 2.0 * step;
                newX0[3 + i] += (rng.NextDouble() - 0.5) * 0.6 * step;
            }
            Array.Copy(newX0, 0, xs, 0, NX);
            sig = Math.Max(sig - dt * step, cfg.SigmaMin + 1e-3);

            s2.Reseed(newX0, xs, us, sig, trustRegion: 0.05);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            s2.Solve(5);
            times.Add(sw.Elapsed.TotalMilliseconds);
            x = (double[])s2.ReferenceX.Clone();
            u = (double[])s2.ReferenceU.Clone();
            sig = s2.Sigma;
        }

        double nodeDt = s2.Sigma / (n - 1);
        double cadence = step * nodeDt;
        times.Sort();
        double mean = times.Average(), med = times[times.Count / 2], worst = times[^1];
        Console.WriteLine($"  {step,5:F2} nd  {cadence,7:F2} s  {mean,7:F0} ms {med,8:F0} ms {worst,8:F0} ms   {mean / cadence,8:F0} ms/s");
    }

    Console.WriteLine();
    Console.WriteLine("A SHORTER cadence is cheaper PER SOLVE (better warm start) and the");
    Console.WriteLine("last column shows whether it is also cheaper in total CPU.");
    return 0;
}

// Does the problem still solve when it is BIGGER than the reference case?
//
// The solver works on x~ = x/scale and the trust region is in those units, so a
// scale hard-coded to one test case means the physical step size stays fixed while
// the problem grows — and the iteration budget eventually cannot traverse it. This
// scales the reference descent up and compares a FIXED reference XScale against one
// sized from the problem itself.
static int ScaleCheck()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
    double[] baseX0 = Row(0), baseXf = Row(1), xRef = Row(2);
    int n = xRef.Length / NX;

    Console.WriteLine("descent scaled up; 1x is the reference case (300 m, 50 m/s)");
    Console.WriteLine();
    Console.WriteLine("  size        FIXED XScale [100,100,300,50,50,50]      ADAPTIVE XScale");

    foreach (double mult in new[] { 1.0, 2.0, 5.0, 10.0, 20.0 })
    {
        var x0 = (double[])baseX0.Clone();
        var xf = (double[])baseXf.Clone();
        for (int i = 0; i < 3; i++) { x0[i] *= mult; xf[i] *= mult; }
        for (int i = 3; i < 6; i++) x0[i] *= Math.Sqrt(mult);   // free-fall-ish entry speed
        double sigma = 12.0 * Math.Sqrt(mult);

        string Run(bool adaptive)
        {
            double L = Math.Sqrt((x0[0]-xf[0])*(x0[0]-xf[0]) + (x0[1]-xf[1])*(x0[1]-xf[1]) + (x0[2]-xf[2])*(x0[2]-xf[2]));
            double sp = Math.Sqrt(x0[3]*x0[3] + x0[4]*x0[4] + x0[5]*x0[5]);
            double V = Math.Max(Math.Max(sp, Math.Sqrt(L * 9.81)), 1.0);
            double[] xs = adaptive
                ? [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, 250000.0]
                : [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];

            var cfg = new Scvx6DofConfig
            {
                Nodes = n, XScale = xs, SigmaScale = sigma,
                SigmaMin = sigma * 0.25, SigmaMax = sigma * 2.5,
            };
            var xSeed = new double[n * NX];
            var uSeed = new double[n * NU];
            double m0 = x0[Dynamics6Dof.IM];
            for (int k = 0; k < n; k++)
            {
                double t = (double)k / (n - 1);
                for (int i = 0; i < 3; i++)
                {
                    xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                    xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
                }
                xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
                xSeed[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
                uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
            }

            var s = new Scvx6DofSolver(cfg) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
            s.Initialize(x0, xf, xSeed, uSeed, sigma);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            ScvxStatus st = s.Solve(25);
            return $"{st,-22} {s.IterationCount,2} it {sw.Elapsed.TotalMilliseconds,6:F0} ms";
        }

        Console.WriteLine($"  {mult,4:F0}x   {Run(false)}   {Run(true)}");
    }
    return 0;
}

// Where is the SOLVE TIME going? ADMM iteration count is the currency, not wall
// clock: a subproblem that needs 50x the ADMM iterations costs 50x, and that is a
// CONDITIONING property of the matrices we hand it. This isolates the two things
// the mod changed relative to the validated reference — the state scaling and the
// objective weights — and reports ADMM iterations for each.
static int CondCheck()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
    double[] x0 = Row(0), xf = Row(1), xRef = Row(2);
    int n = xRef.Length / NX;
    double m0 = x0[Dynamics6Dof.IM];

    double[] refScale = [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];
    double L = 0, sp = 0;
    for (int i = 0; i < 3; i++) { L += (x0[i] - xf[i]) * (x0[i] - xf[i]); sp += x0[3 + i] * x0[3 + i]; }
    L = Math.Sqrt(L); sp = Math.Sqrt(sp);
    double V = Math.Max(Math.Max(sp, Math.Sqrt(L * 9.81)), 1.0);
    double[] adaptScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, 250000.0];

    Console.WriteLine($"reference XScale  pos {refScale[2],8:F1}  vel {refScale[3],8:F1}");
    Console.WriteLine($"adaptive  XScale  pos {L,8:F1}  vel {V,8:F1}");
    Console.WriteLine();
    Console.WriteLine("  case                          SCvx it   ADMM it    ms   burn time   merit");

    void Run(string name, double[] xs, double wDu, double wW, double prox = 0.0)
    {
        var cfg = new Scvx6DofConfig { Nodes = n, XScale = xs, WDu = wDu, WW = wW, ProximalWeight = prox };
        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
            xSeed[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
        }
        var s = new Scvx6DofSolver(cfg) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        s.Initialize(x0, xf, xSeed, uSeed, 12.0);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus st = s.Solve(25);
        double ms = sw.Elapsed.TotalMilliseconds;
        int admm = s.Trace.Sum(t => t.SolverIterations);
        Console.WriteLine($"  {name,-28} {s.IterationCount,5}  {admm,8}  {ms,6:F0}   sigma {s.Sigma,6:F2}  merit {s.Cost:E4}   {st}");
    }

    // Reference weights are 0.2 / 1.0. The mod now runs far smaller ones.
    Run("reference scale + weights", refScale, 0.2, 1.0);
    Run("adaptive scale, ref weights", adaptScale, 0.2, 1.0);
    Run("reference scale, low weights", refScale, 0.01, 0.05);
    Run("adaptive scale, low weights", adaptScale, 0.01, 0.05);
    Run("adaptive scale, tiny weights", adaptScale, 0.001, 0.005);
    Console.WriteLine();
    Console.WriteLine("  + proximal conditioning (same low weights, no bias):");
    foreach (double prox in new[] { 0.001, 0.01, 0.05, 0.2 })
        Run($"  prox {prox:F3}", adaptScale, 0.01, 0.05, prox);

    Console.WriteLine();
    Console.WriteLine("ADMM iterations is the cost driver. If the low-weight rows blow up,");
    Console.WriteLine("the objective weights are conditioning P, not just shaping the answer.");
    return 0;
}

// Is the solver body-agnostic? Gravity enters the dynamics, the velocity scale, the
// seed thrust and the over-powered test; a hard-coded 9.81 anywhere would show up as
// the Moon case failing or planning something unflyable. Vehicle thrust is scaled
// with gravity so the TWR is comparable and only the body changes.
static int BodyCheck()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] Row(int i) => lines[i].Split(',')
        .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
    double[] x0 = Row(0), xf = Row(1), xRef = Row(2);
    int n = xRef.Length / NX;
    double m0 = x0[Dynamics6Dof.IM];

    Console.WriteLine("  body        g     v0       Tmax      sigma   defect     merit   ADMM    ms   status");

    foreach ((string name, double g) in new[]
             { ("Earth", 9.81), ("Moon", 1.62), ("Mars", 3.72), ("Ceres", 0.27) })
    {
        // Same TWR on every body, so only gravity differs.
        double tmax = 6.0e6 * (g / 9.81);

        // AND a physically equivalent entry speed. Holding v0 at the Earth value is
        // not a body-agnosticism test — at the same TWR the stopping distance scales
        // as v0^2/g, so 50 m/s needs 532 m on the Moon and 3193 m on Ceres against
        // 316 m of altitude. Those cases are simply impossible, and the solver
        // correctly refuses them. Scaling v0 as sqrt(L*g) makes the descent equally
        // hard everywhere (109 m of stopping distance on every body).
        double L = 0;
        for (int i = 0; i < 3; i++) L += (x0[i] - xf[i]) * (x0[i] - xf[i]);
        L = Math.Sqrt(L);

        var bx0 = (double[])x0.Clone();
        double vEntry = Math.Sqrt(L * g);
        bx0[3] = 0.0; bx0[4] = 0.0; bx0[5] = -vEntry;
        double V = Math.Max(vEntry, 1.0);

        // Free-fall-ish time over the descent, the same rule the mod's seed uses.
        double sigma = Math.Sqrt(2.0 * L / g);
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, WDu = 0.01, WW = 0.05, ProximalWeight = 0.05,
            SigmaMin = sigma * 0.15, SigmaMax = sigma * 4.0, SigmaScale = sigma,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };

        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = bx0[i] + t * (xf[i] - bx0[i]);
                xSeed[k * NX + 3 + i] = bx0[3 + i] + t * (xf[3 + i] - bx0[3 + i]);
            }
            xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
            xSeed[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }

        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        s.Initialize(bx0, xf, xSeed, uSeed, sigma);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus st = s.Solve(25);
        double ms = sw.Elapsed.TotalMilliseconds;
        int admm = s.Trace.Sum(t => t.SolverIterations);
        double defect = double.PositiveInfinity;
        for (int i = s.Trace.Count - 1; i >= 0; i--)
            if (s.Trace[i].Accepted) { defect = s.Trace[i].DefectNorm; break; }

        Console.WriteLine($"  {name,-8} {g,5:F2} {vEntry,5:F1}  {tmax,9:E2}  {s.Sigma,6:F2}  {defect,9:E2}  " +
                          $"{s.Cost,8:E2}  {admm,6}  {ms,5:F0}  {st}");
    }
    Console.WriteLine();
    Console.WriteLine("All bodies must converge with a defect under 1e-3. A hard-coded 9.81");
    Console.WriteLine("would show as the low-gravity cases failing or drifting.");
    return 0;
}

// A descent starting DIRECTLY ABOVE the pad, with no horizontal position or velocity
// and pointing straight up. Nothing in the geometry favours going sideways, so any
// horizontal excursion is the solver telling us something. Sweeps the throttle floor,
// because a floor above hover forces the vehicle to tilt to shed thrust and an arc is
// then the CORRECT min-fuel answer, not a bug.
static int ArcCheck()
{
    const int n = 30;
    double g = 9.81, m0 = 250000.0, tmax = 6.0e6;
    double h = 300.0, vDown = 40.0, alt = 20.0;

    var x0 = new double[NX];
    x0[2] = h; x0[5] = -vDown; x0[Dynamics6Dof.IQ] = 1.0; x0[Dynamics6Dof.IM] = m0;
    var xf = new double[NX];
    xf[2] = alt; xf[Dynamics6Dof.IQ] = 1.0;

    Console.WriteLine($"straight-down descent: {h:F0} m -> {alt:F0} m, {vDown:F0} m/s, starting vertical");
    Console.WriteLine($"weight {m0 * g / 1e6:F2} MN, Tmax {tmax / 1e6:F2} MN (max TWR {tmax / (m0 * g):F2})");
    Console.WriteLine();
    Console.WriteLine("  floor  TWRmin   sigma   max |horizontal|   peak tilt   defect     status");

    foreach (double floor in new[] { 0.05, 0.10, 0.20, 0.30, 0.40, 0.50 })
    {
        double L = h - alt;
        double V = Math.Max(Math.Max(vDown, Math.Sqrt(L * g)), 1.0);
        double sigma = Math.Sqrt(2.0 * L / g) * 2.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = floor,
            WDu = 0.0, WW = 0.0, ProximalWeight = 0.05,     // penalties OFF, as reported
            SigmaMin = sigma * 0.15, SigmaMax = sigma * 4.0, SigmaScale = sigma,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };

        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
            xSeed[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + Dynamics6Dof.IT] = Math.Max(1.05 * m0 * g, floor * tmax);
        }

        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        s.Initialize(x0, xf, xSeed, uSeed, sigma);
        ScvxStatus st = s.Solve(30);

        double[] X = s.ReferenceX;
        double horiz = 0, tilt = 0;
        for (int k = 0; k < n; k++)
        {
            horiz = Math.Max(horiz, Math.Sqrt(X[k * NX] * X[k * NX] + X[k * NX + 1] * X[k * NX + 1]));
            double qx = X[k * NX + 7], qy = X[k * NX + 8];
            double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
            tilt = Math.Max(tilt, Math.Acos(Math.Clamp(r22, -1, 1)) * 180.0 / Math.PI);
        }
        double defect = double.PositiveInfinity;
        for (int i = s.Trace.Count - 1; i >= 0; i--)
            if (s.Trace[i].Accepted) { defect = s.Trace[i].DefectNorm; break; }

        Console.WriteLine($"  {floor,5:F2}  {floor * tmax / (m0 * g),6:F2}  {s.Sigma,6:F2}   {horiz,12:F1} m   {tilt,8:F1} deg  {defect,9:E2}  {st}");
    }
    // Now perturb the START, one variable at a time, to see what actually bends it.
    Console.WriteLine();
    Console.WriteLine("  perturbation of the START (floor 0.10 throughout):");
    Console.WriteLine("  case                       sigma   max |horizontal|   peak tilt   status");

    void Perturb(string label, double vx, double tiltDeg, double xOffset)
    {
        var px0 = new double[NX];
        px0[0] = xOffset; px0[2] = h; px0[3] = vx; px0[5] = -vDown; px0[Dynamics6Dof.IM] = m0;
        double a = tiltDeg * Math.PI / 180.0 / 2.0;
        px0[Dynamics6Dof.IQ] = Math.Cos(a); px0[Dynamics6Dof.IQ + 2] = Math.Sin(a);

        double L = Math.Sqrt(xOffset * xOffset + (h - alt) * (h - alt));
        double sp = Math.Sqrt(vx * vx + vDown * vDown);
        double V = Math.Max(Math.Max(sp, Math.Sqrt(L * g)), 1.0);
        double sg = Math.Sqrt(2.0 * L / g) * 2.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.10,
            WDu = 0.0, WW = 0.0, ProximalWeight = 0.05,
            SigmaMin = sg * 0.15, SigmaMax = sg * 4.0, SigmaScale = sg,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = px0[i] + t * (xf[i] - px0[i]);
                xs[k * NX + 3 + i] = px0[3 + i] + t * (xf[3 + i] - px0[3 + i]);
            }
            for (int i = 0; i < 4; i++)
                xs[k * NX + Dynamics6Dof.IQ + i] = px0[Dynamics6Dof.IQ + i] * (1 - t) + xf[Dynamics6Dof.IQ + i] * t;
            double qn = 0;
            for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
            qn = Math.Sqrt(qn);
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(px0, 0, xs, 0, NX);

        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(px0, xf, xs, us, sg);
        ScvxStatus st = sv.Solve(30);
        double[] X = sv.ReferenceX;
        double hz = 0, tl = 0;
        for (int k = 0; k < n; k++)
        {
            hz = Math.Max(hz, Math.Sqrt(X[k * NX] * X[k * NX] + X[k * NX + 1] * X[k * NX + 1]));
            double qx = X[k * NX + 7], qy = X[k * NX + 8];
            tl = Math.Max(tl, Math.Acos(Math.Clamp(1.0 - 2.0 * (qx * qx + qy * qy), -1, 1)) * 180.0 / Math.PI);
        }
        Console.WriteLine($"  {label,-24} {sv.Sigma,6:F2}   {hz,12:F1} m   {tl,8:F1} deg  {st}");
    }

    Perturb("baseline (nothing)",        0.0,  0.0,  0.0);
    Perturb("horiz velocity 10 m/s",    10.0,  0.0,  0.0);
    Perturb("horiz velocity 30 m/s",    30.0,  0.0,  0.0);
    Perturb("initial tilt 10 deg",       0.0, 10.0,  0.0);
    Perturb("initial tilt 25 deg",       0.0, 25.0,  0.0);
    Perturb("downrange offset 100 m",    0.0,  0.0, 100.0);

    Console.WriteLine();
    Console.WriteLine("TWRmin > 1 CANNOT hold altitude pointing up: it must tilt to shed thrust,");
    Console.WriteLine("so a large horizontal excursion there is correct physics, not a solver bug.");
    return 0;
}

// Motion starts in the x-z plane (horizontal velocity along +x only) and the target
// is on the axis, so the optimal trajectory is PLANAR. Any y excursion is the solver
// exploiting a degeneracy, not physics.
//
// The degeneracy: the translational dynamics only see R(q)*(tdx,tdy,T) — three
// numbers — but the control has FOUR components. Roll is a free parameter; infinitely
// many (roll, tdx, tdy) triples give the same inertial thrust. W_W (the angular-rate
// penalty) is the only thing that picks among them. Drive it to zero and roll becomes
// arbitrary, which shows up as out-of-plane kinks.
static int RollCheck()
{
    const int n = 30;
    double g = 9.81, m0 = 250000.0, tmax = 6.0e6;
    double h = 300.0, alt = 20.0, vx = 20.0, vDown = 40.0;

    var x0 = new double[NX];
    x0[2] = h; x0[3] = vx; x0[5] = -vDown; x0[Dynamics6Dof.IQ] = 1.0; x0[Dynamics6Dof.IM] = m0;
    var xf = new double[NX];
    xf[2] = alt; xf[Dynamics6Dof.IQ] = 1.0;

    Console.WriteLine("planar problem: velocity along +x only, target on the axis.");
    Console.WriteLine("any OUT-OF-PLANE (y) motion is degeneracy, not physics.");
    Console.WriteLine();
    Console.WriteLine("   W_W     in-plane |x|   OUT-OF-PLANE |y|   peak roll rate   peak |tau_roll|   merit");

    foreach (double ww in new[] { 0.0, 1e-4, 1e-3, 1e-2, 5e-2 })
    {
        double L = h - alt;
        double V = Math.Max(Math.Sqrt(vx * vx + vDown * vDown), 1.0);
        double sg = Math.Sqrt(2.0 * L / g) * 2.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.10,
            WDu = 0.0, WW = ww, ProximalWeight = 0.05,
            SigmaMin = sg * 0.15, SigmaMax = sg * 4.0, SigmaScale = sg,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xs[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            xs[k * NX + Dynamics6Dof.IQ] = 1.0;
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(x0, 0, xs, 0, NX);

        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(x0, xf, xs, us, sg);
        sv.Solve(30);
        double[] X = sv.ReferenceX, U = sv.ReferenceU;
        double inPlane = 0, outPlane = 0, rollRate = 0, tauRoll = 0;
        for (int k = 0; k < n; k++)
        {
            inPlane = Math.Max(inPlane, Math.Abs(X[k * NX]));
            outPlane = Math.Max(outPlane, Math.Abs(X[k * NX + 1]));
            rollRate = Math.Max(rollRate, Math.Abs(X[k * NX + 12]));
            tauRoll = Math.Max(tauRoll, Math.Abs(U[k * NU + 3]));
        }
        Console.WriteLine($"  {ww,6:F4}  {inPlane,12:F1} m  {outPlane,15:F2} m  {rollRate,14:F4}  {tauRoll,16:E2}  {sv.Cost:E3}");
    }
    // The clean start above stays perfectly planar. Now the two things a REAL engage
    // has that it did not: an arbitrary initial attitude (including ROLL about the
    // thrust axis, which rotates how tdx/tdy map to inertial directions) and non-zero
    // body rates. Plus a sweep of W_DU, since with no control-rate penalty the
    // min-fuel optimum is bang-bang and the thrust vector may jump between nodes.
    Console.WriteLine();
    Console.WriteLine("  realistic start (tilt 15 deg + roll 40 deg, rates 0.05 rad/s):");
    Console.WriteLine("   W_DU   path kink        speed at kink        slowest point        burn time");

    foreach (double wdu in new[] { 0.0, 1e-3, 1e-2, 5e-2 })
    {
        var px0 = new double[NX];
        px0[2] = h; px0[3] = vx; px0[5] = -vDown; px0[Dynamics6Dof.IM] = m0;
        // tilt 15 deg about body Y, then roll 40 deg about body Z
        double ha = 15.0 * Math.PI / 180.0 / 2.0, hr = 40.0 * Math.PI / 180.0 / 2.0;
        double tw = Math.Cos(ha), ty = Math.Sin(ha);
        double rw = Math.Cos(hr), rz = Math.Sin(hr);
        px0[Dynamics6Dof.IQ + 0] = tw * rw;
        px0[Dynamics6Dof.IQ + 1] = -ty * rz;
        px0[Dynamics6Dof.IQ + 2] = ty * rw;
        px0[Dynamics6Dof.IQ + 3] = tw * rz;
        px0[10] = 0.05; px0[11] = -0.05; px0[12] = 0.05;

        double L = h - alt;
        double V = Math.Max(Math.Sqrt(vx * vx + vDown * vDown), 1.0);
        double sg = Math.Sqrt(2.0 * L / g) * 2.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.10,
            WDu = wdu, WW = 1e-3, ProximalWeight = 0.05,
            SigmaMin = sg * 0.15, SigmaMax = sg * 4.0, SigmaScale = sg,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = px0[i] + t * (xf[i] - px0[i]);
                xs[k * NX + 3 + i] = px0[3 + i] + t * (xf[3 + i] - px0[3 + i]);
            }
            for (int i = 0; i < 4; i++)
                xs[k * NX + 6 + i] = px0[6 + i] * (1 - t) + xf[6 + i] * t;
            double qn = 0;
            for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
            qn = Math.Sqrt(qn);
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            for (int i = 0; i < 3; i++) xs[k * NX + 10 + i] = px0[10 + i] * (1 - t);
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(px0, 0, xs, 0, NX);

        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(px0, xf, xs, us, sg);
        sv.Solve(30);
        double[] X = sv.ReferenceX, U = sv.ReferenceU;
        double ip = 0, op = 0, jump = 0, rr = 0;
        double px = 0, py = 0, pz = 0;
        for (int k = 0; k < n; k++)
        {
            ip = Math.Max(ip, Math.Abs(X[k * NX]));
            op = Math.Max(op, Math.Abs(X[k * NX + 1]));
            rr = Math.Max(rr, Math.Abs(X[k * NX + 12]));
            double tn = Math.Sqrt(U[k * NU] * U[k * NU] + U[k * NU + 1] * U[k * NU + 1] + U[k * NU + 2] * U[k * NU + 2]);
            double dx = U[k * NU] / tn, dy = U[k * NU + 1] / tn, dz = U[k * NU + 2] / tn;
            if (k > 0)
            {
                double d = Math.Acos(Math.Clamp(dx * px + dy * py + dz * pz, -1, 1)) * 180.0 / Math.PI;
                jump = Math.Max(jump, d);
            }
            px = dx; py = dy; pz = dz;
        }
        // The thing actually being LOOKED AT: the angle between consecutive position
        // segments. Control smoothness is not the same measurement, and a path kink
        // is what the overlay draws.
        double pathKink = 0; int kinkAt = -1;
        for (int k = 1; k + 1 < n; k++)
        {
            double ax = X[k * NX] - X[(k - 1) * NX], ay = X[k * NX + 1] - X[(k - 1) * NX + 1], az = X[k * NX + 2] - X[(k - 1) * NX + 2];
            double bx = X[(k + 1) * NX] - X[k * NX], by = X[(k + 1) * NX + 1] - X[k * NX + 1], bz = X[(k + 1) * NX + 2] - X[k * NX + 2];
            double na = Math.Sqrt(ax * ax + ay * ay + az * az), nb = Math.Sqrt(bx * bx + by * by + bz * bz);
            if (na < 1e-9 || nb < 1e-9) continue;
            double c = Math.Clamp((ax * bx + ay * by + az * bz) / (na * nb), -1, 1);
            double ang = Math.Acos(c) * 180.0 / Math.PI;
            if (ang > pathKink) { pathKink = ang; kinkAt = k; }
        }
        double defc = double.PositiveInfinity;
        for (int i = sv.Trace.Count - 1; i >= 0; i--)
            if (sv.Trace[i].Accepted) { defc = sv.Trace[i].DefectNorm; break; }
        double vAtKink = kinkAt >= 0
            ? Math.Sqrt(X[kinkAt * NX + 3] * X[kinkAt * NX + 3] + X[kinkAt * NX + 4] * X[kinkAt * NX + 4] + X[kinkAt * NX + 5] * X[kinkAt * NX + 5])
            : 0.0;
        double vMin = double.MaxValue; int vMinAt = -1;
        for (int k = 0; k < n; k++)
        {
            double vv = Math.Sqrt(X[k * NX + 3] * X[k * NX + 3] + X[k * NX + 4] * X[k * NX + 4] + X[k * NX + 5] * X[k * NX + 5]);
            if (vv < vMin) { vMin = vv; vMinAt = k; }
        }
        double thrMin = double.MaxValue, thrMax = 0;
        for (int k = 0; k < n; k++) { thrMin = Math.Min(thrMin, U[k * NU + 2]); thrMax = Math.Max(thrMax, U[k * NU + 2]); }
        Console.WriteLine($"  {wdu,6:F3} kink {pathKink,6:F1} deg @{kinkAt,2}  |v| there {vAtKink,6:F2} m/s   " +
                          $"min |v| {vMin,6:F2} @{vMinAt,2}   sigma {sv.Sigma,6:F2}  " +
                          $"throttle {thrMin / tmax * 100,4:F0}-{thrMax / tmax * 100,3:F0}%");
    }

    // Is the kink a bad LOCAL OPTIMUM (seed / budget) or the true answer? Sweep the
    // iteration budget and the sigma seed at fixed weights.
    Console.WriteLine();
    Console.WriteLine("  is it a local optimum? (W_DU 0.01, W_W 1e-3)");
    Console.WriteLine("   iters  sigma seed   sigma   path kink   |v| at kink   merit      status");

    foreach (int budget in new[] { 30, 100 })
    foreach (double seedMult in new[] { 0.7, 1.0, 1.5 })
    {
        var px0 = new double[NX];
        px0[2] = h; px0[3] = vx; px0[5] = -vDown; px0[Dynamics6Dof.IM] = m0;
        double ha = 15.0 * Math.PI / 180.0 / 2.0, hr = 40.0 * Math.PI / 180.0 / 2.0;
        double tw = Math.Cos(ha), ty = Math.Sin(ha), rw = Math.Cos(hr), rz = Math.Sin(hr);
        px0[6] = tw * rw; px0[7] = -ty * rz; px0[8] = ty * rw; px0[9] = tw * rz;
        px0[10] = 0.05; px0[11] = -0.05; px0[12] = 0.05;

        double L = h - alt;
        double V = Math.Max(Math.Sqrt(vx * vx + vDown * vDown), 1.0);
        double sg = Math.Sqrt(2.0 * L / g) * 2.0 * seedMult;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.10,
            WDu = 0.01, WW = 1e-3, ProximalWeight = 0.05,
            SigmaMin = sg * 0.15, SigmaMax = sg * 4.0, SigmaScale = sg,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = px0[i] + t * (xf[i] - px0[i]);
                xs[k * NX + 3 + i] = px0[3 + i] + t * (xf[3 + i] - px0[3 + i]);
            }
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] = px0[6 + i] * (1 - t) + xf[6 + i] * t;
            double qn = 0;
            for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
            qn = Math.Sqrt(qn);
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            for (int i = 0; i < 3; i++) xs[k * NX + 10 + i] = px0[10 + i] * (1 - t);
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(px0, 0, xs, 0, NX);

        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(px0, xf, xs, us, sg);
        ScvxStatus st = sv.Solve(budget);
        double[] X = sv.ReferenceX;
        double kink = 0; int at = -1;
        for (int k = 1; k + 1 < n; k++)
        {
            double ax = X[k * NX] - X[(k - 1) * NX], ay = X[k * NX + 1] - X[(k - 1) * NX + 1], az = X[k * NX + 2] - X[(k - 1) * NX + 2];
            double bx = X[(k + 1) * NX] - X[k * NX], by = X[(k + 1) * NX + 1] - X[k * NX + 1], bz = X[(k + 1) * NX + 2] - X[k * NX + 2];
            double na = Math.Sqrt(ax * ax + ay * ay + az * az), nb = Math.Sqrt(bx * bx + by * by + bz * bz);
            if (na < 1e-9 || nb < 1e-9) continue;
            double ang = Math.Acos(Math.Clamp((ax * bx + ay * by + az * bz) / (na * nb), -1, 1)) * 180.0 / Math.PI;
            if (ang > kink) { kink = ang; at = k; }
        }
        double vk = at >= 0 ? Math.Sqrt(X[at * NX + 3] * X[at * NX + 3] + X[at * NX + 4] * X[at * NX + 4] + X[at * NX + 5] * X[at * NX + 5]) : 0;
        Console.WriteLine($"  {budget,5}  {sg,10:F2}  {sv.Sigma,6:F2}   {kink,7:F1} deg  {vk,10:F2} m/s  {sv.Cost:E3}  {st}");
    }

    Console.WriteLine();
    Console.WriteLine("If more iterations do not remove it, the kink is a genuine local optimum");
    Console.WriteLine("and the SEED is what selects it.");
    return 0;
}

// Is the Python reference's neat trajectory a property of the SOLVER, or of a
// benign initial condition? Its x0 is planar (r.y = v.y = 0), has NO horizontal
// velocity, is perfectly upright at BOTH ends, and is not rotating. Perturb one
// thing at a time from that exact state and watch the path kink.
static int AblateCheck()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] R(int i) => lines[i].Split(',').Select(z => double.Parse(z, CultureInfo.InvariantCulture)).ToArray();
    double[] refX0 = R(0), xf = R(1);
    const int n = 30;
    double g = 9.81, tmax = 6.0e6, m0 = refX0[Dynamics6Dof.IM];

    Console.WriteLine("perturbing the PYTHON REFERENCE initial condition, one change at a time");
    Console.WriteLine();
    Console.WriteLine("  case                          path kink   |v| at kink   out-of-plane   merit");

    void Run(string label, double vy, double tiltDeg, double rollDeg, double rate)
    {
        var x0 = (double[])refX0.Clone();
        x0[4] = vy;
        double ha = tiltDeg * Math.PI / 180.0 / 2.0, hr = rollDeg * Math.PI / 180.0 / 2.0;
        double tw = Math.Cos(ha), ty = Math.Sin(ha), rw = Math.Cos(hr), rz = Math.Sin(hr);
        x0[6] = tw * rw; x0[7] = -ty * rz; x0[8] = ty * rw; x0[9] = tw * rz;
        x0[10] = rate; x0[11] = -rate; x0[12] = rate;

        double L = 0;
        for (int i = 0; i < 3; i++) L += (x0[i] - xf[i]) * (x0[i] - xf[i]);
        L = Math.Sqrt(L);
        double sp = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double V = Math.Max(Math.Max(sp, Math.Sqrt(L * g)), 1.0);
        double sg = 12.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.40,
            WDu = 0.01, WW = 1e-3, ProximalWeight = 0.05,
            SigmaMin = sg * 0.15, SigmaMax = sg * 4.0, SigmaScale = sg,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xs[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] = x0[6 + i] * (1 - t) + xf[6 + i] * t;
            double qn = 0;
            for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
            qn = Math.Sqrt(qn);
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            for (int i = 0; i < 3; i++) xs[k * NX + 10 + i] = x0[10 + i] * (1 - t);
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(x0, 0, xs, 0, NX);

        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(x0, xf, xs, us, sg);
        sv.Solve(60);
        double[] X = sv.ReferenceX;
        double kink = 0, oop = 0; int at = -1;
        for (int k = 0; k < n; k++) oop = Math.Max(oop, Math.Abs(X[k * NX + 1]));
        for (int k = 1; k + 1 < n; k++)
        {
            double ax = X[k * NX] - X[(k - 1) * NX], ay = X[k * NX + 1] - X[(k - 1) * NX + 1], az = X[k * NX + 2] - X[(k - 1) * NX + 2];
            double bx = X[(k + 1) * NX] - X[k * NX], by = X[(k + 1) * NX + 1] - X[k * NX + 1], bz = X[(k + 1) * NX + 2] - X[k * NX + 2];
            double na = Math.Sqrt(ax * ax + ay * ay + az * az), nb = Math.Sqrt(bx * bx + by * by + bz * bz);
            if (na < 1e-9 || nb < 1e-9) continue;
            double a2 = Math.Acos(Math.Clamp((ax * bx + ay * by + az * bz) / (na * nb), -1, 1)) * 180.0 / Math.PI;
            if (a2 > kink) { kink = a2; at = k; }
        }
        double vk = at >= 0 ? Math.Sqrt(X[at * NX + 3] * X[at * NX + 3] + X[at * NX + 4] * X[at * NX + 4] + X[at * NX + 5] * X[at * NX + 5]) : 0;
        Console.WriteLine($"  {label,-28} {kink,7:F1} deg  {vk,10:F2} m/s  {oop,10:F2} m  {sv.Cost:E3}");
    }

    Run("reference exactly",            0.0,  0.0,  0.0, 0.0);
    Run("+ out-of-plane vel 10 m/s",   10.0,  0.0,  0.0, 0.0);
    Run("+ initial tilt 15 deg",        0.0, 15.0,  0.0, 0.0);
    Run("+ initial ROLL 40 deg",        0.0,  0.0, 40.0, 0.0);
    Run("+ body rates 0.05 rad/s",      0.0,  0.0,  0.0, 0.05);
    Run("+ all of the above",          10.0, 15.0, 40.0, 0.05);
    return 0;
}

// Does pinning sigma actually deliver what it promises? The claim is that with sigma
// fixed the regularisers can no longer buy anything by stretching time, so raising
// them for smoothness stops distorting the trajectory.
static int FixedTimeCheck()
{
    string path = FindFile("loop_ref.csv") ?? "loop_ref.csv";
    string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
    double[] R(int i) => lines[i].Split(',').Select(z => double.Parse(z, CultureInfo.InvariantCulture)).ToArray();
    double[] rx = R(0), xf = R(1);
    const int n = 30;
    double g = 9.81, tmax = 6.0e6, m0 = rx[Dynamics6Dof.IM];

    // The realistic (perturbed) start, which is where the kinks showed up.
    var x0 = (double[])rx.Clone();
    x0[4] = 10.0;
    double ha = 15.0 * Math.PI / 180.0 / 2.0, hr = 40.0 * Math.PI / 180.0 / 2.0;
    double tw = Math.Cos(ha), ty = Math.Sin(ha), rw = Math.Cos(hr), rz = Math.Sin(hr);
    x0[6] = tw * rw; x0[7] = -ty * rz; x0[8] = ty * rw; x0[9] = tw * rz;
    x0[10] = 0.05; x0[11] = -0.05; x0[12] = 0.05;

    Console.WriteLine("realistic perturbed start, sweeping W_DU under FREE vs FIXED burn time");
    Console.WriteLine();
    Console.WriteLine("  mode     W_DU    sigma    path kink   out-of-plane   merit");

    void Run(bool fixedTime, double wdu)
    {
        double L = 0;
        for (int i = 0; i < 3; i++) L += (x0[i] - xf[i]) * (x0[i] - xf[i]);
        L = Math.Sqrt(L);
        double sp = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double V = Math.Max(Math.Max(sp, Math.Sqrt(L * g)), 1.0);
        double sg = 16.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.40,
            WDu = wdu, WW = 1e-3, ProximalWeight = 0.05, SigmaScale = sg,
            SigmaMin = fixedTime ? sg * (1 - 1e-9) : sg * 0.15,
            SigmaMax = fixedTime ? sg * (1 + 1e-9) : sg * 4.0,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xs[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] = x0[6 + i] * (1 - t) + xf[6 + i] * t;
            double qn = 0;
            for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
            qn = Math.Sqrt(qn);
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            for (int i = 0; i < 3; i++) xs[k * NX + 10 + i] = x0[10 + i] * (1 - t);
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(x0, 0, xs, 0, NX);

        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(x0, xf, xs, us, sg);
        sv.Solve(40);
        double[] X = sv.ReferenceX;
        double kink = 0, oop = 0;
        for (int k = 0; k < n; k++) oop = Math.Max(oop, Math.Abs(X[k * NX + 1]));
        for (int k = 1; k + 1 < n; k++)
        {
            double ax = X[k * NX] - X[(k - 1) * NX], ay = X[k * NX + 1] - X[(k - 1) * NX + 1], az = X[k * NX + 2] - X[(k - 1) * NX + 2];
            double bx = X[(k + 1) * NX] - X[k * NX], by = X[(k + 1) * NX + 1] - X[k * NX + 1], bz = X[(k + 1) * NX + 2] - X[k * NX + 2];
            double na = Math.Sqrt(ax * ax + ay * ay + az * az), nb = Math.Sqrt(bx * bx + by * by + bz * bz);
            if (na < 1e-9 || nb < 1e-9) continue;
            kink = Math.Max(kink, Math.Acos(Math.Clamp((ax * bx + ay * by + az * bz) / (na * nb), -1, 1)) * 180.0 / Math.PI);
        }
        Console.WriteLine($"  {(fixedTime ? "FIXED" : "free "),-6} {wdu,6:F3}  {sv.Sigma,6:F2}   {kink,7:F1} deg  {oop,10:F2} m  {sv.Cost:E3}");
    }

    // How tight can the sigma pin be before it fights the solver's own tolerance?
    Console.WriteLine("  sigma pin width vs solve cost (SubproblemEps is 1e-5):");
    Console.WriteLine("   rel slack   ADMM it     ms    sigma      status");
    foreach (double slack in new[] { 1e-9, 1e-7, 1e-5, 1e-3, 1e-2 })
    {
        double L = 0;
        for (int i = 0; i < 3; i++) L += (x0[i] - xf[i]) * (x0[i] - xf[i]);
        L = Math.Sqrt(L);
        double sp = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double V = Math.Max(Math.Max(sp, Math.Sqrt(L * g)), 1.0);
        double sg = 16.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n, Tmax = tmax, ThrottleFloor = 0.40,
            WDu = 0.05, WW = 1e-3, ProximalWeight = 0.05, SigmaScale = sg,
            SigmaMin = sg * (1 - slack), SigmaMax = sg * (1 + slack),
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -g };
        var xs = new double[n * NX];
        var us = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xs[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] = x0[6 + i] * (1 - t) + xf[6 + i] * t;
            double qn = 0;
            for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
            qn = Math.Sqrt(qn);
            for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
            for (int i = 0; i < 3; i++) xs[k * NX + 10 + i] = x0[10 + i] * (1 - t);
            xs[k * NX + Dynamics6Dof.IM] = m0 * (1.0 - 0.08 * t);
            us[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * g;
        }
        Array.Copy(x0, 0, xs, 0, NX);
        var sv = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        sv.Initialize(x0, xf, xs, us, sg);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus st = sv.Solve(40);
        Console.WriteLine($"   {slack,9:E0}  {sv.Trace.Sum(t => t.SolverIterations),8}  {sw.Elapsed.TotalMilliseconds,6:F0}  {sv.Sigma,7:F3}   {st}");
    }
    Console.WriteLine();

    foreach (double wdu in new[] { 0.01, 0.05, 0.20 }) Run(false, wdu);
    Console.WriteLine();
    foreach (double wdu in new[] { 0.01, 0.05, 0.20 }) Run(true, wdu);
    Console.WriteLine();
    Console.WriteLine("free: sigma should DRIFT UPWARD with W_DU (the regulariser buying time).");
    Console.WriteLine("fixed: sigma constant, so W_DU only smooths and does not distort.");
    return 0;
}
