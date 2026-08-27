namespace Gfold;

/// <summary>
/// A conic program in the standard split form, independent of which solver runs it:
///
///   minimize    c'x
///   subject to  A x = b
///               G x + s = h,   s in K
///
/// where K = R+^l x SOC(q[0]) x SOC(q[1]) x ... (no exponential cones).
///
/// This is ECOS's native input shape and is kept as the assembly target because it
/// separates equalities from cone rows, which is how <see cref="GfoldPlanner"/> builds
/// them. SCS wants the two stacked into a single matrix with the equalities expressed
/// as a leading ZERO cone; <see cref="ScsSolver"/> does that conversion, which is a
/// concatenation and nothing more — the algebra is identical either way, since a zero
/// cone forces s = 0 and leaves A x = b.
/// </summary>
public sealed class ConicProblem
{
    public required double[] C { get; init; }            // length n
    public required SparseCcs G { get; init; }           // m x n
    public required double[] H { get; init; }            // length m
    public SparseCcs? A { get; init; }                   // p x n, optional
    public double[]? B { get; init; }                    // length p, optional
    public int PositiveOrthantDim { get; init; }         // l
    public int[] SocDims { get; init; } = [];            // q

    public int Vars => G.Cols;
    public int ConeRows => G.Rows;
    public int EqualityRows => A?.Rows ?? 0;
}

/// <summary>
/// Solver outcome, normalised across backends.
///
/// The values are ECOS's exit codes because this started as ECOS's own enum, and
/// keeping them means the ECOS path needs no mapping at all; <see cref="ScsSolver"/>
/// translates SCS's status onto the nearest member. The distinctions that matter
/// downstream are only "usable", "infeasible" and "failed" — see
/// <see cref="ConicResult.IsOptimal"/> and GfoldPlanner.IsUsable.
/// </summary>
public enum ConicStatus
{
    Optimal = 0,
    PrimalInfeasible = 1,
    DualInfeasible = 2,
    OptimalInaccurate = 10,
    PrimalInfeasibleInaccurate = 11,
    DualInfeasibleInaccurate = 12,
    MaxIterations = -1,
    Numerics = -2,
    OutsideCone = -3,
    Interrupted = -4,
    SetupFailed = -5,
    Fatal = -7,
}

/// <summary>
/// One solve's result. <paramref name="Iterations"/> is not comparable across
/// backends — an interior-point method's tens and a first-order method's thousands
/// measure different things — but wall time and <paramref name="PrimalCost"/> are.
/// </summary>
public sealed record ConicResult(ConicStatus Status, double[] X, double PrimalCost, int Iterations)
{
    public bool IsOptimal => Status is ConicStatus.Optimal or ConicStatus.OptimalInaccurate;
}
