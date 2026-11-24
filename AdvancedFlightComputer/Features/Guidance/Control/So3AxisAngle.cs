namespace AdvancedFlightComputer.Features.Guidance;

internal static class So3AxisAngle
{
    // Exact SO(3) axis-angle vector phi * axis from the three columns t0, t1, t2 of the
    // relative rotation matrix R_err = R_body^T * R_target. Preserves exact small-angle sign,
    // avoids sin(phi) fold-back past 90 deg, and extracts a unit axis from R_err + I at 180 deg.
    internal static void FromBasisColumns(
        double t0x, double t0y, double t0z,
        double t1x, double t1y, double t1z,
        double t2x, double t2y, double t2z,
        out double ex, out double ey, out double ez)
    {
        // 0.5 * sum_i (e_i x t_i) = sin(phi) * axis.
        double sx = 0.5 * (t1z - t2y);
        double sy = 0.5 * (t2x - t0z);
        double sz = 0.5 * (t0y - t1x);
        double sinPhi = Math.Sqrt(sx * sx + sy * sy + sz * sz);
        double cosPhi = Math.Clamp(0.5 * (t0x + t1y + t2z - 1.0), -1.0, 1.0);
        double phi = Math.Atan2(sinPhi, cosPhi);

        if (sinPhi > 1e-12)
        {
            double scale = phi / sinPhi;
            ex = sx * scale;
            ey = sy * scale;
            ez = sz * scale;
            return;
        }

        if (cosPhi < 0.0)
        {
            // Near 180 deg, R_err + I = 2 * axis * axis^T; use the largest column for the axis.
            double c0x = t0x + 1.0, c0y = t0y, c0z = t0z;
            double c1x = t1x, c1y = t1y + 1.0, c1z = t1z;
            double c2x = t2x, c2y = t2y, c2z = t2z + 1.0;
            double n0 = c0x * c0x + c0y * c0y + c0z * c0z;
            double n1 = c1x * c1x + c1y * c1y + c1z * c1z;
            double n2 = c2x * c2x + c2y * c2y + c2z * c2z;

            double ax, ay, az, nBest;
            if (n0 >= n1 && n0 >= n2) { ax = c0x; ay = c0y; az = c0z; nBest = n0; }
            else if (n1 >= n2) { ax = c1x; ay = c1y; az = c1z; nBest = n1; }
            else { ax = c2x; ay = c2y; az = c2z; nBest = n2; }

            if (nBest > 1e-18)
            {
                double inv = phi / Math.Sqrt(nBest);
                ex = ax * inv;
                ey = ay * inv;
                ez = az * inv;
                return;
            }
        }

        ex = 0.0;
        ey = 0.0;
        ez = 0.0;
    }
}
