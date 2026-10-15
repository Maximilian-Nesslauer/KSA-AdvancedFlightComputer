namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Reading a glide-and-burn plan at a time, and carrying it onto a different grid.
///
/// The receding-horizon seed, the swap to the burn-only problem and the commanded attitude all read the plan between its nodes. Position and velocity come from the cubic Hermite segment their own values define, as the 6-DOF's PlanStateInterpolation does - a straight line cuts the corner of a curved path, and a hard braking burn is a curved path - and mass and controls are linear.
///
/// THE PHASE BOUNDARY. The ignition node belongs to both phases, and a time exactly at it reads the BURN side, so a resample that puts a new ignition node on the old one starts the burn with the burn's throttle. Resampled glide nodes have their throttle cleared and burn nodes are held at the floor: the new problem pins both, and a seed that disagrees with its own constraints only costs the first iteration.
/// </summary>
public static class Resample3Dof
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    /// <summary>The interval holding plan time t, its fraction through it, and its length in seconds. Times off either end clamp to the end node.</summary>
    public static void Locate(int nodes, int glideIntervals, double sigG, double sigB, double t,
                              out int left, out int right, out double fraction, out double intervalSeconds)
    {
        double total = sigG + sigB;
        int burnIntervals = nodes - 1 - glideIntervals;
        if (!(t > 0.0))
        {
            left = right = 0;
            fraction = 0.0;
            intervalSeconds = glideIntervals > 0 ? sigG / glideIntervals : sigB / burnIntervals;
            return;
        }
        if (t >= total)
        {
            left = right = nodes - 1;
            fraction = 0.0;
            intervalSeconds = sigB / burnIntervals;
            return;
        }
        if (glideIntervals > 0 && t < sigG)
        {
            double dt = sigG / glideIntervals;
            double s = t / dt;
            left = Math.Min((int)s, glideIntervals - 1);
            right = left + 1;
            fraction = Math.Clamp(s - left, 0.0, 1.0);
            intervalSeconds = dt;
            return;
        }
        double dtb = sigB / burnIntervals;
        double sb = (t - sigG) / dtb;
        int ib = Math.Min((int)sb, burnIntervals - 1);
        left = glideIntervals + ib;
        right = left + 1;
        fraction = Math.Clamp(sb - ib, 0.0, 1.0);
        intervalSeconds = dtb;
    }

    /// <summary>State and control at plan time t.</summary>
    public static void Sample(double[] x, double[] u, int nodes, int glideIntervals, double sigG, double sigB, double t,
                              Span<double> xs, Span<double> us)
    {
        Locate(nodes, glideIntervals, sigG, sigB, t, out int a, out int b, out double f, out double dt);
        if (a == b)
        {
            x.AsSpan(a * NX, NX).CopyTo(xs);
            u.AsSpan(a * NU, NU).CopyTo(us);
            return;
        }

        double t2 = f * f, t3 = t2 * f;
        double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + f, h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
        double d00 = (6 * t2 - 6 * f) / dt, d10 = 3 * t2 - 4 * f + 1, d01 = (-6 * t2 + 6 * f) / dt, d11 = 3 * t2 - 2 * f;
        int ia = a * NX, ib = b * NX;
        for (int i = 0; i < 3; i++)
        {
            double p0 = x[ia + i], p1 = x[ib + i];
            double v0 = x[ia + PointMass3Dof.IV + i], v1 = x[ib + PointMass3Dof.IV + i];
            xs[i] = h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1;
            xs[PointMass3Dof.IV + i] = d00 * p0 + d10 * v0 + d01 * p1 + d11 * v1;
        }
        xs[PointMass3Dof.IM] = (1 - f) * x[ia + PointMass3Dof.IM] + f * x[ib + PointMass3Dof.IM];

        int ua = a * NU, ub = b * NU;
        for (int j = 0; j < NU; j++)
            us[j] = (1 - f) * u[ua + j] + f * u[ub + j];
        // Inside the last glide interval the right node's throttle is the burn's; the glide flies with the engine off.
        if (a < glideIntervals)
            us[PointMass3Dof.ITH] = 0.0;
        Normalise(us);
    }

    /// <summary>
    /// The plan read onto a new grid whose node 0 sits at old plan time <paramref name="t0"/>.
    /// </summary>
    public static void Resample(double[] x, double[] u, int nodes, int glideIntervals, double sigG, double sigB, double t0,
                                int newNodes, int newGlideIntervals, double newSigG, double newSigB, double throttleFloor,
                                double[] xOut, double[] uOut)
    {
        for (int k = 0; k < newNodes; k++)
        {
            double t = t0 + Scvx3DofSolver.NodeTime(k, newGlideIntervals, newNodes, newSigG, newSigB);
            Sample(x, u, nodes, glideIntervals, sigG, sigB, t, xOut.AsSpan(k * NX, NX), uOut.AsSpan(k * NU, NU));
            ref double th = ref uOut[k * NU + PointMass3Dof.ITH];
            th = k < newGlideIntervals ? 0.0 : Math.Max(th, throttleFloor);
        }
    }

    private static void Normalise(Span<double> u)
    {
        double len = Math.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        if (len < 1e-12) return;
        for (int j = 0; j < 3; j++) u[j] /= len;
    }
}
