using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The convex ascent with the full aero force (#216): lift and side force from a nose-first body-axis table, in place of drag alone.
///
/// The vehicle is AscentAoaCheck's 2stage_new, with its box-law drag laid out as a body-axis table, and on top of it a normal force across the axis, C_N S = 1000 m^2 per radian of sin(alpha) - as large as the drag 5 deg off the airflow, the size KSA 5901 shows - and, for the roll, a side force the airframe makes at no angle at all.
///
/// Checked:
///  - the force the dynamics apply against the table, tilted in pitch, in yaw and in both, and at a held roll;
///  - a body table holding only the drag plans the same ascent as the drag table it was laid out from;
///  - a plan with the lift converges, inserts and is dynamically feasible, and the lift the drag-only plan leaves out is reported;
///  - the problem file keeps the table and the roll.
/// </summary>
internal static class AscentLiftCheck
{
    private const double NoseFace = 0.3 * 9.6, Flanks = 217.6, Skin = 94.9 - NoseFace;
    private const double NormalPerRad = 1000.0;

    private static readonly double[] Angles = [-20, -15, -10, -6, -4, -2, -1, 0, 1, 2, 4, 6, 10, 15, 20];

    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-66} {detail}");
        if (!ok) _fails++;
    }

    /// <summary>The body-axis coefficients (m^2 of C A over the stage's drag area) at a nose-first air direction h in body axes.</summary>
    private static (double X, double Y, double Z) Law(double hx, double hy, double hz, double normalPerRad, double sideM2)
    {
        double c = Math.Clamp(hx, -1.0, 1.0), s = Math.Sqrt(Math.Max(0.0, 1.0 - c * c));
        double cdA = NoseFace * c + Flanks * s + Skin;
        // Drag against the airflow; the normal force square to the axis, towards where the nose is tilted from the airflow; a side force along body y.
        double fx = -cdA * hx, fy = -cdA * hy - normalPerRad * hy + sideM2, fz = -cdA * hz - normalPerRad * hz;
        double a = AscentAoaCheck.DragArea;
        return (fx / a, fy / a, fz / a);
    }

    internal static BodyAeroTable Table(double normalPerRad, double sideM2 = 0.0)
    {
        double[] mach = AeroTable.DefaultMachBreakpoints;
        int n = Angles.Length;
        var c = new double[mach.Length * n * n * 3];
        for (int i = 0; i < mach.Length; i++)
            for (int ja = 0; ja < n; ja++)
                for (int jb = 0; jb < n; jb++)
                {
                    (double hx, double hy, double hz) = BodyAeroTable.AirDirection(Angles[ja] * Math.PI / 180.0, Angles[jb] * Math.PI / 180.0, noseFirst: true);
                    (double x, double y, double z) = Law(hx, hy, hz, normalPerRad, sideM2);
                    int node = ((i * n + ja) * n + jb) * 3;
                    c[node] = x;
                    c[node + 1] = y;
                    c[node + 2] = z;
                }
        return new BodyAeroTable(mach, Angles, Angles, c, noseFirst: true);
    }

    private static AscentProblem Problem(BodyAeroTable? body, double rollRad = 0.0)
    {
        AscentProblem p = AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true));
        var air = (KsaAscentAtmosphere)p.Atmosphere;
        return new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = p.Stages,
            Atmosphere = new KsaAscentAtmosphere(air.Atmosphere, air.Table, bodyForce: body),
            GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            PlaneNormal = p.PlaneNormal, QMax = p.QMax, QAlphaMax = p.QAlphaMax, RollAngleRad = rollRad,
        };
    }

    private static double[] Cross(double[] a, double[] b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    private static double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    private static double[] Unit(double[] a) { double n = Math.Sqrt(Dot(a, a)); return [a[0] / n, a[1] / n, a[2] / n]; }

    /// <summary>The body axes the flight holds for a thrust axis x (GuidanceWindow.AscentRollRef): y the plane's -n square to x, turned about x by the roll.</summary>
    private static (double[] X, double[] Y, double[] Z) Axes(double[] x, double[] normal, double roll)
    {
        x = Unit(x);
        double along = -Dot(normal, x);
        double[] y0 = Unit([-normal[0] - along * x[0], -normal[1] - along * x[1], -normal[2] - along * x[2]]);
        double[] z0 = Cross(x, y0);
        double c = Math.Cos(roll), s = Math.Sin(roll);
        double[] y = [c * y0[0] + s * z0[0], c * y0[1] + s * z0[1], c * y0[2] + s * z0[2]];
        return (x, y, Cross(x, y));
    }

    /// <summary>
    /// The dynamics' aero acceleration in body axes, and the table's, for the craft at 10 km and 400 m/s through the air, thrust axis 45 deg up the plane, the air at alpha and beta about the nose.
    /// </summary>
    private static (double[] Model, double[] Table) Compare(AscentProblem p, BodyAeroTable table, double alphaDeg, double betaDeg, double roll)
    {
        double[] n = Unit(p.PlaneNormal);
        double[] up = Unit(p.R0);
        double[] east = Unit(Cross(n, up));
        double h = 10000.0, speed = 400.0, mass = 120000.0;
        double[] r = [up[0] * (p.BodyRadius + h), up[1] * (p.BodyRadius + h), up[2] * (p.BodyRadius + h)];
        double[] x = Unit([up[0] + east[0], up[1] + east[1], up[2] + east[2]]);
        (double[] bx, double[] by, double[] bz) = Axes(x, n, roll);
        (double ux, double uy, double uz) = BodyAeroTable.AirDirection(alphaDeg * Math.PI / 180.0, betaDeg * Math.PI / 180.0, noseFirst: true);
        double[] vRel = new double[3];
        for (int i = 0; i < 3; i++) vRel[i] = speed * (ux * bx[i] + uy * by[i] + uz * bz[i]);
        double[] spin = [-p.Omega * r[1], p.Omega * r[0], 0.0];
        double[] v = [vRel[0] + spin[0], vRel[1] + spin[1], vRel[2] + spin[2]];
        double[] a = AscentAeroLaw.Acceleration(p, 0, r, v, mass, [0.9 * x[0], 0.9 * x[1], 0.9 * x[2]]);
        double rho = ((KsaAscentAtmosphere)p.Atmosphere).Density(new Dual(h)).V;
        double qsm = 0.5 * rho * speed * speed * AscentAoaCheck.DragArea / mass;
        table.Coefficients(0.5, alphaDeg * Math.PI / 180.0, betaDeg * Math.PI / 180.0, out double cx, out double cy, out double cz);
        return ([Dot(a, bx), Dot(a, by), Dot(a, bz)], [qsm * cx, qsm * cy, qsm * cz]);
    }

    private static AscentSolution Solve(string what, AscentProblem p)
    {
        AscentSolution s = AscentScvx.Solve(p);
        Console.WriteLine($"  {what}: {s.Message}, {s.Iterations} iterations, {s.SolveSeconds:F1} s"
            + (s.Nodes > 0 ? $"; {s.FinalMass / 1000.0:F3} t to orbit, max q {s.DynamicPressure.Max() / 1000.0:F1} kPa, "
                           + $"defects {s.MaxDefectPosition:F1} m / {s.MaxDefectVelocity:F3} m/s" : ""));
        return s;
    }

    /// <summary>The dV the lift across the airflow gives a plan's burns, which a drag-only plan never saw: the integral of |a_aero across the airflow| over its burns.</summary>
    private static double LiftDv(AscentSolution s, AscentProblem withLift)
    {
        double dv = 0.0;
        var lift = new double[s.Nodes];
        for (int k = 0; k < s.Nodes; k++)
        {
            double[] r = [s.Position[3 * k], s.Position[3 * k + 1], s.Position[3 * k + 2]];
            double[] v = [s.Velocity[3 * k], s.Velocity[3 * k + 1], s.Velocity[3 * k + 2]];
            double[] u = [s.Throttle[3 * k], s.Throttle[3 * k + 1], s.Throttle[3 * k + 2]];
            double[] a = AscentAeroLaw.Acceleration(withLift, s.NodeStage[k], r, v, s.Mass[k], u);
            double[] h = Unit([v[0] + withLift.Omega * r[1], v[1] - withLift.Omega * r[0], v[2]]);
            double along = Dot(a, h);
            double[] across = [a[0] - along * h[0], a[1] - along * h[1], a[2] - along * h[2]];
            lift[k] = Math.Sqrt(Dot(across, across));
        }
        for (int k = 0; k < s.Nodes - 1; k++)
            if (s.NodeStage[k] == s.NodeStage[k + 1])
                dv += 0.5 * (lift[k] + lift[k + 1]) * (s.Time[k + 1] - s.Time[k]);
        return dv;
    }

    private static double MaxAlphaAbove(AscentSolution s, double q)
    {
        double max = 0.0;
        for (int k = 0; k < s.Nodes; k++)
            if (s.DynamicPressure[k] > q) max = Math.Max(max, s.AngleOfAttackDeg[k]);
        return max;
    }

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT SCVX: the full aero force from a nose-first body-axis table (#216), on 2stage_new");

        // 1. The model against the table. The lateral coefficients are read at the true angles, so they match the table at its nodes; the axial one at the rounded tilt (AlphaRounding), so it matches at zero and within a few per cent from 4 deg out.
        BodyAeroTable lift = Table(NormalPerRad, sideM2: 5.0);
        AscentProblem pl = Problem(lift);
        double worstLat = 0.0, worstAx = 0.0;
        string worstAt = "";
        foreach ((double a, double b) in new[] { (0.0, 0.0), (4.0, 0.0), (10.0, 0.0), (-6.0, 0.0), (0.0, 4.0), (0.0, -10.0), (4.0, 4.0), (-6.0, 10.0), (2.0, -1.0) })
            foreach (double roll in new[] { 0.0, 0.7 })
            {
                (double[] m, double[] t) = Compare(Problem(lift, roll), lift, a, b, roll);
                double scale = Math.Sqrt(Dot(t, t));
                double lat = Math.Max(Math.Abs(m[1] - t[1]), Math.Abs(m[2] - t[2])) / scale;
                double ax = Math.Abs(m[0] - t[0]) / Math.Abs(t[0]);
                if (verbose)
                    Console.WriteLine($"    alpha {a,5:F1} beta {b,5:F1} roll {roll:F1}: model ({m[0]:F4}, {m[1]:F4}, {m[2]:F4}) table ({t[0]:F4}, {t[1]:F4}, {t[2]:F4}) m/s^2");
                if (lat > worstLat) { worstLat = lat; worstAt = $"alpha {a} beta {b} roll {roll}"; }
                if (Math.Sqrt(a * a + b * b) >= 4.0 || (a == 0.0 && b == 0.0))
                    worstAx = Math.Max(worstAx, ax);
            }
        Check("the force across the axis is the table's, in pitch, yaw and both", worstLat < 1e-4, $"worst {worstLat:E1} of the force at {worstAt} (the soft angle bound)");
        Check("the axial force is the table's within the rounding", worstAx < 0.03, $"worst {100 * worstAx:F2} % (at 0 deg and from 4 deg out)");

        // 2. The held roll: the side force the airframe makes at no angle is along the body's y, which the roll turns about the thrust axis.
        (double[] m0, _) = Compare(Problem(lift, 0.0), lift, 0.0, 0.0, 0.0);
        (double[] m90, _) = Compare(Problem(lift, Math.PI / 2), lift, 0.0, 0.0, Math.PI / 2);
        Check("the side force turns with the held roll", m0[1] > 0.0 && m90[1] > 0.0 && Math.Abs(m0[1] - m90[1]) < 1e-6 * Math.Abs(m0[1]),
              $"along body y {m0[1]:F4} at roll 0, {m90[1]:F4} at 90 deg");

        // 3. Drag alone, as a body table, plans what the drag table plans.
        AscentSolution dragTable = Solve("drag table (as before)", AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true)));
        AscentSolution dragBody = Solve("body table, drag only", Problem(Table(0.0)));
        AscentSolution withLift = Solve("body table with lift", Problem(Table(NormalPerRad)));
        Check("the drag table's plan converges", dragTable.Converged, dragTable.Message);
        Check("the drag-only body table's plan converges", dragBody.Converged, dragBody.Message);
        Check("the plan with lift converges", withLift.Converged, withLift.Message);
        if (dragTable.Nodes == 0 || dragBody.Nodes == 0 || withLift.Nodes == 0)
            return 1;
        double diff = Math.Abs(dragBody.FinalMass - dragTable.FinalMass) / dragTable.FinalMass;
        Check("drag alone as a body table delivers what the drag table does", diff < 0.01,
              $"{dragBody.FinalMass / 1000.0:F3} t against {dragTable.FinalMass / 1000.0:F3} t");

        // 4. The plan with lift: feasible, inserts, and the lift the drag-only plan ignored.
        AscentProblem liftProblem = Problem(Table(NormalPerRad));
        double ignored = LiftDv(dragTable, liftProblem), used = LiftDv(withLift, liftProblem);
        Console.WriteLine($"  lift across the airflow over the burns: {ignored:F0} m/s on the drag-only plan, which it never planned for; {used:F0} m/s on the plan with lift, which it did");
        Console.WriteLine($"  max alpha where q > 10 kPa: {MaxAlphaAbove(dragTable, 10000.0):F2} deg drag-only, {MaxAlphaAbove(withLift, 10000.0):F2} deg with lift");
        Check("the plan with lift is dynamically feasible", withLift.MaxDefectPosition < 35.0 && withLift.MaxDefectVelocity < 0.8,
              $"{withLift.MaxDefectPosition:F2} m, {withLift.MaxDefectVelocity:F4} m/s");
        Check("and inserts", Math.Abs(withLift.TerminalResidual[0]) < 150.0 && Math.Abs(withLift.TerminalResidual[1]) < 0.2,
              $"{withLift.TerminalResidual[0]:F1} m, {withLift.TerminalResidual[1]:F3} m/s");
        Check("within 2 % of the drag-only plan's mass to orbit", Math.Abs(withLift.FinalMass / dragTable.FinalMass - 1.0) < 0.02,
              $"{withLift.FinalMass / 1000.0:F3} t");

        // 5. The problem file keeps the table and the roll.
        AscentProblem rolled = Problem(lift, 0.7);
        AscentProblem back = AscentProblemFile.FromJson(AscentProblemFile.ToJson(rolled));
        (double[] before, _) = Compare(rolled, lift, 4.0, -6.0, 0.7);
        (double[] after, _) = Compare(back, lift, 4.0, -6.0, 0.7);
        bool same = back.Atmosphere.BodyForce is { NoseFirst: true } && back.RollAngleRad == 0.7
                    && Enumerable.Range(0, 3).All(i => Math.Abs(before[i] - after[i]) < 1e-12);
        Check("the problem file keeps the body table and the roll", same);
        AscentProblem old = AscentProblemFile.FromJson(AscentProblemFile.ToJson(AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true))));
        Check("a file without one reads back as drag alone", old.Atmosphere.BodyForce == null);

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }
}
