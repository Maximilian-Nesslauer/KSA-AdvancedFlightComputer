#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// A finished trajectory, published as one immutable object.
///  This is the ONLY thing the sim thread reads from a solve. Command needs four things that are written together - the control history, the burn time, the time the plan was anchored, and the node count - and reading them as four separate fields is a torn read waiting to happen: new controls paired with an old anchor time means the vehicle is commanded from the wrong point of the right trajectory, which looks exactly like the guidance ignoring its own plan.
///  Publishing is a single reference assignment, so a reader either sees the whole previous plan or the whole new one and never a mixture. No lock is involved, and none is needed: the arrays are freshly allocated by each solve and never written again once published, so they are immutable in practice even though double[] cannot say so in the type system. Nothing may mutate them in place - that is the one rule this type depends on.
/// </summary>
/// <param name="X">Node states, Nodes * Dynamics6Dof.NX. Never mutated after publish.</param>
/// <param name="U">Node controls, Nodes * Dynamics6Dof.NU. Never mutated after publish.</param>
/// <param name="Sigma">Burn time this plan covers, seconds.</param>
/// <param name="SolveTime">Sim time at which node 0 was the vehicle.</param>
/// <param name="Nodes">Node count, so the reader does not have to infer it from a length.</param>
/// <param name="Coast">The coast before ignition this plan opens with, s: the time from SolveTime to ignition. 0 for a plan that is all burn (Scvx6DofConfig.CoastIntervals).</param>
/// <param name="CoastIntervals">How many of the plan's intervals are the coast; 0 for none.</param>
public sealed record Ksa6DofPlan(double[] X, double[] U, double Sigma, double SolveTime, int Nodes,
                                 double Coast = 0.0, int CoastIntervals = 0)
{
    /// <summary>The whole plan's duration, s: the coast and the burn.</summary>
    public double Duration => Coast + Sigma;

    /// <summary>Whether the plan opens with a coast, however short.</summary>
    public bool HasCoast => CoastIntervals > 0;

    /// <summary>Sim time of the plan's ignition: its burn's start.</summary>
    public double IgnitionTime => SolveTime + Coast;

    /// <summary>Node k's time from SolveTime, s.</summary>
    public double NodeTime(int k) => AdvancedFlightComputer.Guidance.Scvx.SixDof.Scvx6DofSolver.NodeTime(k, Nodes, CoastIntervals, Sigma, Coast);

    /// <summary>Interval k's duration, s: a coast interval's or a burn interval's.</summary>
    public double IntervalDt(int k) => k < CoastIntervals ? Coast / CoastIntervals : Sigma / (Nodes - 1 - CoastIntervals);

    /// <summary>Whether time t from SolveTime is still in the coast: before ignition.</summary>
    public bool InCoast(double t) => CoastIntervals > 0 && t < Coast;

    /// <summary>
    /// Where time t from SolveTime falls on the plan: interval k and the fraction f along it, clamped to the plan. In the coast the interval is a coast one; from ignition on, a burn one, so the two phases are never blended across ignition. <paramref name="burnSide"/> takes the burn's side at ignition itself.
    /// </summary>
    public (int K, double F) Locate(double t, bool burnSide = true)
    {
        if (CoastIntervals > 0 && (t < Coast || (!burnSide && t <= Coast)))
        {
            double s = Coast > 0.0 ? System.Math.Clamp(t / (Coast / CoastIntervals), 0.0, CoastIntervals) : 0.0;
            int k = System.Math.Min((int)s, CoastIntervals - 1);
            return (k, s - k);
        }
        int burn = Nodes - 1 - CoastIntervals;
        double b = Sigma > 0.0 ? System.Math.Clamp((t - Coast) / (Sigma / burn), 0.0, burn - 0.001) : 0.0;
        int kb = (int)b;
        return (CoastIntervals + kb, b - kb);
    }
}
