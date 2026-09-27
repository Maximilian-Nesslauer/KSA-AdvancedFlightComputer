namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Moves a cold-solve reference along with the craft between spread iterations.
/// Position, velocity and mass shift together in the site frame, because replacing only node zero leaves a false jump across the first interval that the solver then has to answer for.
/// </summary>
public static class ColdReferenceShift
{
    public static void Apply(double[] reference, int nodes, double sigma, ReadOnlySpan<double> observed)
    {
        const int stride = Dynamics6Dof.NX;
        if (nodes < 2 || reference.Length != nodes * stride || observed.Length < stride || !(sigma > 0))
            throw new ArgumentException("The cold reference needs a complete trajectory and a positive duration.");

        const int r = Dynamics6Dof.IR, v = Dynamics6Dof.IV;
        double dx = observed[r] - reference[r];
        double dy = observed[r + 1] - reference[r + 1];
        double dz = observed[r + 2] - reference[r + 2];
        double dvx = observed[v] - reference[v];
        double dvy = observed[v + 1] - reference[v + 1];
        double dvz = observed[v + 2] - reference[v + 2];
        double dm = observed[Dynamics6Dof.IM] - reference[Dynamics6Dof.IM];
        double dt = sigma / (nodes - 1);

        // The terminal node moves too, by up to dx + dv * sigma. That is a linearisation point, not a target: the terminal constraints still pull the solution back onto xf.
        for (int k = 0; k < nodes; k++)
        {
            int offset = k * stride;
            double time = k * dt;
            reference[offset + r] += dx + dvx * time;
            reference[offset + r + 1] += dy + dvy * time;
            reference[offset + r + 2] += dz + dvz * time;
            reference[offset + v] += dvx;
            reference[offset + v + 1] += dvy;
            reference[offset + v + 2] += dvz;
            reference[offset + Dynamics6Dof.IM] += dm;
        }
        // Only node zero takes measured attitude and body rates because rotating the full path would also change its thrust directions.
        observed[..stride].CopyTo(reference);
    }
}
