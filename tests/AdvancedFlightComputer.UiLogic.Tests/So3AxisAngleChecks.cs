using AdvancedFlightComputer.Features.Guidance;

internal static partial class Program
{
    private static void So3AxisAngleRules()
    {
        // Zero rotation returns zero vector.
        EvalAxisAngle(1, 0, 0, 0.0, out double ex, out double ey, out double ez);
        Check("SO(3) zero angle returns zero vector", Math.Sqrt(ex * ex + ey * ey + ez * ez) < 1e-12);

        // Monotonic angle recovery across 1..180 deg along an arbitrary unit axis (no sin(phi) fold-back past 90 deg).
        double ax = 0.2672612419124244, ay = 0.5345224838248488, az = 0.8017837257372732;
        double prevDeg = 0.0;
        foreach (double deg in new[] { 1.0, 30.0, 60.0, 90.0, 120.0, 150.0, 175.0, 179.9, 180.0 })
        {
            EvalAxisAngle(ax, ay, az, deg * Math.PI / 180.0, out ex, out ey, out ez);
            double gotDeg = Math.Sqrt(ex * ex + ey * ey + ez * ez) * 180.0 / Math.PI;
            Check($"SO(3) recovers exact angle at {deg:F1} deg",
                Math.Abs(gotDeg - deg) < 1e-9 && gotDeg > prevDeg,
                $"got {gotDeg:F6} deg");
            prevDeg = gotDeg;

            // For phi < 180 deg, direction matches +axis; at 180 deg, +/-axis represent the same rotation.
            double dot = (ex * ax + ey * ay + ez * az) / (deg * Math.PI / 180.0);
            bool dirOk = deg < 180.0 ? Math.Abs(dot - 1.0) < 1e-9 : Math.Abs(Math.Abs(dot) - 1.0) < 1e-9;
            Check($"SO(3) axis aligns at {deg:F1} deg", dirOk, $"dot={dot:F9}");
        }

        // Exact 180 deg about each principal axis stays non-zero with magnitude pi.
        foreach ((double px, double py, double pz, string name) in new[]
        {
            (1.0, 0.0, 0.0, "X"),
            (0.0, 1.0, 0.0, "Y"),
            (0.0, 0.0, 1.0, "Z"),
        })
        {
            EvalAxisAngle(px, py, pz, Math.PI, out ex, out ey, out ez);
            double gotDeg = Math.Sqrt(ex * ex + ey * ey + ez * ez) * 180.0 / Math.PI;
            double dot = Math.Abs((ex * px + ey * py + ez * pz) / Math.PI);
            Check($"SO(3) exact 180 deg flip about {name} is non-zero with magnitude pi",
                Math.Abs(gotDeg - 180.0) < 1e-9 && Math.Abs(dot - 1.0) < 1e-9,
                $"got {gotDeg:F6} deg, |dot|={dot:F9}");
        }
    }

    // Builds the three columns of Rodrigues rotation R(axis, phi) and runs So3AxisAngle.FromBasisColumns.
    private static void EvalAxisAngle(double ux, double uy, double uz, double phi,
                                      out double ex, out double ey, out double ez)
    {
        double c = Math.Cos(phi), s = Math.Sin(phi), k = 1.0 - c;
        double t0x = c + ux * ux * k, t0y = uy * ux * k + uz * s, t0z = uz * ux * k - uy * s;
        double t1x = ux * uy * k - uz * s, t1y = c + uy * uy * k, t1z = uz * uy * k + ux * s;
        double t2x = ux * uz * k + uy * s, t2y = uy * uz * k - ux * s, t2z = c + uz * uz * k;
        So3AxisAngle.FromBasisColumns(t0x, t0y, t0z, t1x, t1y, t1z, t2x, t2y, t2z, out ex, out ey, out ez);
    }
}
