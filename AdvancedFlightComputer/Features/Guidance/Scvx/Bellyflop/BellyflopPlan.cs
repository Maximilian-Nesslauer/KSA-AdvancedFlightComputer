namespace AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

/// <summary>
/// A bellyflop plan: states and controls at the nodes over its flight time, from the time it was solved for. Controls are first-order hold between nodes; states are the shot trajectory's.
/// </summary>
public sealed class BellyflopPlan
{
    public required double StartTime { get; init; }
    public required double Sigma { get; init; }
    public required double[,] X { get; init; }
    public required double[,] U { get; init; }
    public required bool Converged { get; init; }
    public required int Iterations { get; init; }
    public required double MaxDefect { get; init; }
    public required double MissM { get; init; }
    public required double SolveMs { get; init; }
    public required bool Cold { get; init; }

    public int Nodes => X.GetLength(0);
    public double EndTime => StartTime + Sigma;

    /// <summary>The controls at a time, held at the ends: alpha, beta, psi, rad.</summary>
    public (double Alpha, double Beta, double Psi) ControlAt(double time)
    {
        int n = Nodes;
        double f = Math.Clamp((time - StartTime) / Sigma, 0.0, 1.0) * (n - 1);
        int k = Math.Min((int)f, n - 2);
        double w = f - k;
        return ((1 - w) * U[k, 0] + w * U[k + 1, 0], (1 - w) * U[k, 1] + w * U[k + 1, 1], (1 - w) * U[k, 2] + w * U[k + 1, 2]);
    }

    /// <summary>The planned state at a time, linear between nodes.</summary>
    public void StateAt(double time, Span<double> x)
    {
        int n = Nodes;
        double f = Math.Clamp((time - StartTime) / Sigma, 0.0, 1.0) * (n - 1);
        int k = Math.Min((int)f, n - 2);
        double w = f - k;
        for (int j = 0; j < BellyflopModel.NX; j++) x[j] = (1 - w) * X[k, j] + w * X[k + 1, j];
    }
}
