using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// --aero-com-move: the sweep cache's move of aero moments to another centre of mass (AeroMomentMove). Each node's force
/// is applied at a point of its own; the moment table about the old centre of mass, moved, must be the moment taken
/// about the new one directly - for body-axis tables (values and the fitted table between nodes) and for belly-first
/// trim rows on each belly axis.
/// </summary>
internal static class AeroComMoveCheck
{
    public static int Run()
    {
        int fails = 0;
        void Check(bool ok, string what)
        {
            Console.WriteLine((ok ? "  ok   " : "  FAIL ") + what);
            if (!ok) fails++;
        }

        var rng = new Random(7);
        double R(double a) => (rng.NextDouble() * 2.0 - 1.0) * a;
        const double L = 50.8;
        double[] c0 = { -18.0, 0.02, 0.10 }, c1 = { -16.5, -0.05, 0.30 };
        double dx = c0[0] - c1[0], dy = c0[1] - c1[1], dz = c0[2] - c1[2];

        static double[] Cross(double ax, double ay, double az, double bx, double by, double bz)
            => new[] { ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx };

        // ---- body-axis tables ----
        double[] mach = { 0.5, 2.0, 5.0 }, alpha = { -10, 0, 10, 20 }, beta = { -5, 0, 5 };
        int n = mach.Length * alpha.Length * beta.Length;
        var force = new double[3 * n];
        var about0 = new double[3 * n];
        var about1 = new double[3 * n];
        for (int k = 0; k < n; k++)
        {
            double fx = R(1.5), fy = R(0.8), fz = R(0.8);
            double px = R(25.0) - 16.0, py = R(4.5), pz = R(4.5);
            double[] m0 = Cross(px - c0[0], py - c0[1], pz - c0[2], fx, fy, fz);
            double[] m1 = Cross(px - c1[0], py - c1[1], pz - c1[2], fx, fy, fz);
            for (int j = 0; j < 3; j++)
            {
                force[3 * k + j] = j == 0 ? fx : j == 1 ? fy : fz;
                about0[3 * k + j] = m0[j] / L;
                about1[3 * k + j] = m1[j] / L;
            }
        }
        double[]? moved = AeroMomentMove.Moved(force, about0, dx, dy, dz, L);
        double worst = 0.0;
        for (int i = 0; moved != null && i < moved.Length; i++)
            worst = Math.Max(worst, Math.Abs(moved[i] - about1[i]));
        Check(moved != null && worst < 1e-12, $"body table values moved {c0[0] - c1[0]:F1}/{c0[1] - c1[1]:F2}/{c0[2] - c1[2]:F2} m match the moment about the new centre of mass (worst {worst:E1})");

        var forceTable = new BodyAeroTable(mach, alpha, beta, force);
        var momentTable = new BodyAeroTable(mach, alpha, beta, about0);
        var direct = new BodyAeroTable(mach, alpha, beta, about1);
        BodyAeroTable? movedTable = AeroMomentMove.Moved(forceTable, momentTable, dx, dy, dz, L);
        double worstFit = 0.0;
        for (int s = 0; s < 50 && movedTable != null; s++)
        {
            double m = 0.5 + rng.NextDouble() * 4.5, a = (-10 + rng.NextDouble() * 30) * Math.PI / 180, b = (-5 + rng.NextDouble() * 10) * Math.PI / 180;
            movedTable.Coefficients(m, a, b, out double l1, out double m1, out double n1);
            direct.Coefficients(m, a, b, out double l2, out double m2, out double n2);
            worstFit = Math.Max(worstFit, Math.Max(Math.Abs(l1 - l2), Math.Max(Math.Abs(m1 - m2), Math.Abs(n1 - n2))));
        }
        Check(movedTable != null && worstFit < 1e-9, $"the moved table between nodes matches a table fitted about the new centre of mass (worst {worstFit:E1})");
        Check(AeroMomentMove.Moved(forceTable, momentTable, 0, 0, 0, L) is { } same && same.Values.SequenceEqual(momentTable.Values), "no move leaves the values as they were");
        Check(AeroMomentMove.Moved(new double[6], new double[9], dx, dy, dz, L) == null, "tables on different grids are not moved");

        // ---- belly-first trim rows, each belly axis ----
        double[] trimAlpha = { 0, 20, 40, 60, 70, 80, 90 };
        double[][] axes = { new double[] { 0, 0, -1 }, new double[] { 0, 0, 1 }, new double[] { 0, -1, 0 }, new double[] { 0, 1, 0 } };
        foreach (double[] b in axes)
        {
            int cells = mach.Length * trimAlpha.Length;
            var cd = new double[cells];
            var cl = new double[cells];
            var cm0 = new double[cells];
            var cm1 = new double[cells];
            double[] pitch = Cross(b[0], b[1], b[2], 1, 0, 0);
            for (int cell = 0; cell < cells; cell++)
            {
                double a = trimAlpha[cell % trimAlpha.Length] * Math.PI / 180.0;
                cd[cell] = 0.5 + rng.NextDouble();
                cl[cell] = R(0.6);
                double ux = Math.Cos(a) + Math.Sin(a) * b[0], uy = Math.Sin(a) * b[1], uz = Math.Sin(a) * b[2];
                double lx = Math.Sin(a) - Math.Cos(a) * b[0], ly = -Math.Cos(a) * b[1], lz = -Math.Cos(a) * b[2];
                double fx = -cd[cell] * ux + cl[cell] * lx, fy = -cd[cell] * uy + cl[cell] * ly, fz = -cd[cell] * uz + cl[cell] * lz;
                double px = R(25.0) - 16.0, py = R(4.5), pz = R(4.5);
                double[] m0 = Cross(px - c0[0], py - c0[1], pz - c0[2], fx, fy, fz);
                double[] m1 = Cross(px - c1[0], py - c1[1], pz - c1[2], fx, fy, fz);
                cm0[cell] = (m0[0] * pitch[0] + m0[1] * pitch[1] + m0[2] * pitch[2]) / L;
                cm1[cell] = (m1[0] * pitch[0] + m1[1] * pitch[1] + m1[2] * pitch[2]) / L;
            }
            double[]? movedCm = AeroMomentMove.MovedTrimCm(cd, cl, cm0, trimAlpha, b[0], b[1], b[2], dx, dy, dz, L);
            double w = 0.0;
            for (int i = 0; movedCm != null && i < cells; i++)
                w = Math.Max(w, Math.Abs(movedCm[i] - cm1[i]));
            Check(movedCm != null && w < 1e-12, $"trim rows on belly axis ({b[0]:0},{b[1]:0},{b[2]:0}) moved match the pitch moment about the new centre of mass (worst {w:E1})");
        }

        Console.WriteLine(fails == 0 ? "PASS aero-com-move" : $"FAIL aero-com-move ({fails})");
        return fails;
    }
}
