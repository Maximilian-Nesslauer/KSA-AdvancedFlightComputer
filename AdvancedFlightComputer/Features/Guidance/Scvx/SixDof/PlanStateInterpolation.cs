namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Samples plan position and velocity between two nodes with the cubic Hermite segment their positions and velocities define.
/// Straight-line position interpolation cuts the corner of a curved path, which during a hard braking burn puts metres of false drift into every warm seed.
/// </summary>
public static class PlanStateInterpolation
{
    public static void SamplePositionVelocity(ReadOnlySpan<double> states, int left, int right,
                                              double fraction, double intervalSeconds,
                                              Span<double> position, Span<double> velocity)
    {
        int a = left * Dynamics6Dof.NX;
        int b = right * Dynamics6Dof.NX;
        if (left == right)
        {
            states.Slice(a, 3).CopyTo(position);
            states.Slice(a + Dynamics6Dof.IV, 3).CopyTo(velocity);
            return;
        }

        double t = fraction;
        double t2 = t * t;
        double t3 = t2 * t;
        double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        double h10 = t3 - 2.0 * t2 + t;
        double h01 = -2.0 * t3 + 3.0 * t2;
        double h11 = t3 - t2;
        double d00 = (6.0 * t2 - 6.0 * t) / intervalSeconds;
        double d10 = 3.0 * t2 - 4.0 * t + 1.0;
        double d01 = (-6.0 * t2 + 6.0 * t) / intervalSeconds;
        double d11 = 3.0 * t2 - 2.0 * t;

        for (int i = 0; i < 3; i++)
        {
            double p0 = states[a + i], p1 = states[b + i];
            double v0 = states[a + Dynamics6Dof.IV + i];
            double v1 = states[b + Dynamics6Dof.IV + i];
            position[i] = h00 * p0 + h10 * intervalSeconds * v0
                + h01 * p1 + h11 * intervalSeconds * v1;
            velocity[i] = d00 * p0 + d10 * v0 + d01 * p1 + d11 * v1;
        }
    }
}
