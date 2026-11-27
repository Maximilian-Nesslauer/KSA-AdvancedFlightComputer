namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Moves a cold-solve reference along with the craft between spread iterations.
/// Position, velocity and mass shift together in the site frame, because replacing only node zero leaves a false jump across the first interval that the solver then has to answer for.
/// </summary>
public static class ColdReferenceShift
{
    public static void Apply(double[] reference, int nodes, double sigma, ReadOnlySpan<double> observed)
        => Apply(reference, nodes, sigma, observed, 0, 0.0);

    /// <summary>As above for a plan that opens with a coast (Scvx6DofConfig.CoastIntervals), whose nodes are not evenly spaced in time.</summary>
    public static void Apply(double[] reference, int nodes, double sigma, ReadOnlySpan<double> observed,
                             int coastIntervals, double coastSigma)
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

        // The terminal node moves too, by up to dx + dv * sigma. That is a linearisation point, not a target: the terminal constraints still pull the solution back onto xf.
        for (int k = 0; k < nodes; k++)
        {
            int offset = k * stride;
            double time = Scvx6DofSolver.NodeTime(k, nodes, coastIntervals, sigma, coastSigma);
            reference[offset + r] += dx + dvx * time;
            reference[offset + r + 1] += dy + dvy * time;
            reference[offset + r + 2] += dz + dvz * time;
            reference[offset + v] += dvx;
            reference[offset + v + 1] += dvy;
            reference[offset + v + 2] += dvz;
            reference[offset + Dynamics6Dof.IM] += dm;
        }
        // Blend measured attitude and body-rate offsets over the first few nodes before overwriting node 0,
        // so interval 0 does not inherit a step discontinuity while the terminal node remains untouched.
        const int iq = Dynamics6Dof.IQ, iw = Dynamics6Dof.IW;
        double q0w = reference[iq], q0x = reference[iq + 1], q0y = reference[iq + 2], q0z = reference[iq + 3];
        double q0n = Math.Sqrt(q0w * q0w + q0x * q0x + q0y * q0y + q0z * q0z);
        double qow = observed[iq], qox = observed[iq + 1], qoy = observed[iq + 2], qoz = observed[iq + 3];
        double qon = Math.Sqrt(qow * qow + qox * qox + qoy * qoy + qoz * qoz);
        double dqx = 0.0, dqy = 0.0, dqz = 0.0, vNorm = 0.0, halfAngle = 0.0;
        if (q0n > 1e-12 && qon > 1e-12)
        {
            q0w /= q0n; q0x /= q0n; q0y /= q0n; q0z /= q0n;
            qow /= qon; qox /= qon; qoy /= qon; qoz /= qon;
            // Shortest-arc hemisphere alignment so a pure sign flip (q -> -q) produces zero rotation
            // and leaves branch alignment to AlignQuaternionBranch.
            double dot = q0w * qow + q0x * qox + q0y * qoy + q0z * qoz;
            if (dot < 0.0) { qow = -qow; qox = -qox; qoy = -qoy; qoz = -qoz; }
            double dqw = qow * q0w + qox * q0x + qoy * q0y + qoz * q0z;
            dqx = -qow * q0x + qox * q0w - qoy * q0z + qoz * q0y;
            dqy = -qow * q0y + qox * q0z + qoy * q0w - qoz * q0x;
            dqz = -qow * q0z - qox * q0y + qoy * q0x + qoz * q0w;
            vNorm = Math.Sqrt(dqx * dqx + dqy * dqy + dqz * dqz);
            halfAngle = Math.Atan2(vNorm, Math.Clamp(dqw, 0.0, 1.0));
        }

        double dwx = observed[iw] - reference[iw];
        double dwy = observed[iw + 1] - reference[iw + 1];
        double dwz = observed[iw + 2] - reference[iw + 2];

        int blendEnd = Math.Min(5, nodes - 1);
        for (int k = 1; k < blendEnd; k++)
        {
            double alpha = 1.0 - (double)k / blendEnd;
            int offset = k * stride;
            if (vNorm > 1e-12 && halfAngle > 1e-12)
            {
                double aHalf = alpha * halfAngle;
                double rw = Math.Cos(aHalf), rs = Math.Sin(aHalf) / vNorm;
                double rx = dqx * rs, ry = dqy * rs, rz = dqz * rs;
                double kw = reference[offset + iq], kx = reference[offset + iq + 1];
                double ky = reference[offset + iq + 2], kz = reference[offset + iq + 3];
                double nw = rw * kw - rx * kx - ry * ky - rz * kz;
                double nx = rw * kx + rx * kw + ry * kz - rz * ky;
                double ny = rw * ky - rx * kz + ry * kw + rz * kx;
                double nz = rw * kz + rx * ky - ry * kx + rz * kw;
                double nn = Math.Sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
                if (nn > 1e-12)
                {
                    reference[offset + iq] = nw / nn;
                    reference[offset + iq + 1] = nx / nn;
                    reference[offset + iq + 2] = ny / nn;
                    reference[offset + iq + 3] = nz / nn;
                }
            }
            reference[offset + iw] += alpha * dwx;
            reference[offset + iw + 1] += alpha * dwy;
            reference[offset + iw + 2] += alpha * dwz;
        }

        observed[..stride].CopyTo(reference);
    }
}
