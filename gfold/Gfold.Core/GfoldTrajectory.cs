using System.Globalization;
using System.Text;

namespace Gfold;

// A solved descent trajectory at N discrete nodes, spaced Dt apart.
public sealed class GfoldTrajectory
{
    public required ConicStatus Status { get; init; }
    public required double Dt { get; init; }
    public required double[][] Position { get; init; }  // [N][3], x = up
    public required double[][] Velocity { get; init; }  // [N][3]
    public required double[][] AccelCmd { get; init; }  // [N][3], u = Tc/m
    public required double[] Sigma { get; init; }       // [N], thrust accel slack
    public required double[] Mass { get; init; }        // [N], kg
    public required double[] LandingPoint { get; init; }// [3]
    public required double LandingErrorNorm { get; init; }
    public required int Iterations { get; init; }

    /// <summary>
    /// The solver returned something worth flying. "Inaccurate" is included on purpose
    /// — both backends use it for a solution that converged loosely rather than one
    /// that failed, and the search's own tolerances are far wider than the difference.
    /// A TRUNCATED solve is not in here: ScsSolver maps that to MaxIterations
    /// precisely so it lands outside this test.
    /// </summary>
    public bool IsUsable => Status is ConicStatus.Optimal or ConicStatus.OptimalInaccurate;

    public int Nodes => Position.Length;
    public double TimeOfFlight => Dt * (Nodes - 1);
    public double FuelUsed => Mass[0] - Mass[^1];

    public string ToCsv()
    {
        var sb = new StringBuilder();
        sb.AppendLine("t,rx,ry,rz,vx,vy,vz,ux,uy,uz,sigma,mass,thrustN");
        for (int n = 0; n < Nodes; n++)
        {
            double[] r = Position[n], v = Velocity[n], u = AccelCmd[n];
            sb.AppendLine(string.Join(",", new[]
            {
                n * Dt, r[0], r[1], r[2], v[0], v[1], v[2],
                u[0], u[1], u[2], Sigma[n], Mass[n], Sigma[n] * Mass[n],
            }.Select(d => d.ToString("G9", CultureInfo.InvariantCulture))));
        }
        return sb.ToString();
    }
}
