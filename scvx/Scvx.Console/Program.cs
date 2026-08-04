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
static string? FindRef()
{
    var dir = new DirectoryInfo(AppContext.BaseDirectory);
    while (dir != null)
    {
        string candidate = Path.Combine(dir.FullName, "python_ref", "dyn_ref.csv");
        if (File.Exists(candidate)) return candidate;
        dir = dir.Parent;
    }
    return null;
}
