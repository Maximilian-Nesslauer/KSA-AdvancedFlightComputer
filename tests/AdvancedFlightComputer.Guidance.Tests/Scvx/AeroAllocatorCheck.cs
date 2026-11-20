using System.Globalization;
using AdvancedFlightComputer.Features.AeroControl;

/// <summary>
/// --aero-alloc: the aero torque allocator (AeroTorqueAllocator) on four fins in a cross, each turning the craft about the axis across its span and rolling it, with a curve that stalls: full slope to 15 deg, half beyond, to 30. Checked against what that layout must give: a pure demand within reach met exactly and symmetrically, the reach of each axis each way, a demand out of reach giving the reachable part, the axis priorities deciding what gives way, and what a set of deflections reports matching its curve.
/// </summary>
internal static class AeroAllocatorCheck
{
    // Per degree up to 15 deg, N m; half that beyond. Roll per degree, N m, from each fin's offset off the axis.
    private const double Slope = 1000.0, RollSlope = 200.0;
    private static readonly double[] Grid = [-30.0, -15.0, 0.0, 15.0, 30.0];

    internal static int Run(bool verbose)
    {
        Console.WriteLine("AERO TORQUE ALLOCATOR: four fins in a cross, stalling past 15 deg");
        AeroTorqueAllocator a = Cross();
        bool ok = true;
        var priority = new double[] { 1.0, 1.0, 1.0 };
        var d = new double[4];

        // Reach: a pitch (x) fin pair gives 2 x (15 + 7.5) x Slope at full; roll needs all four, 4 x 22.5 x RollSlope.
        double pitchMax = 2.0 * 22.5 * Slope, rollMax = 4.0 * 22.5 * RollSlope;
        AeroTorqueAllocator.AxisAuthority px = a.Characterise(0, 1), nx = a.Characterise(0, -1), pz = a.Characterise(2, 1);
        ok &= Check($"pitch reach +/-: {px.MaxTorqueNm:F0}/{nx.MaxTorqueNm:F0} N m against {pitchMax:F0}",
                    Near(px.MaxTorqueNm, pitchMax, 1.0) && Near(nx.MaxTorqueNm, pitchMax, 1.0));
        ok &= Check($"roll reach: {pz.MaxTorqueNm:F0} N m against {rollMax:F0}", Near(pz.MaxTorqueNm, rollMax, 1.0));
        ok &= Check($"pitch comes with its fins' side force: {px.ForcePerNmY:F4} N per N m", Math.Abs(px.ForcePerNmY) > 1e-6);

        // A pitch demand inside the first segment: met by the pitch pair alone, half each (the pair's rolls cancel only so).
        AeroTorqueAllocator.Result r = a.Allocate(12_000.0, 0.0, 0.0, priority, d);
        ok &= Check($"12 kN m of pitch: shortfall {r.ShortfallNm:F3} N m, deflections {Text(d)}",
                    r.Solved && r.ShortfallNm < 1e-6 && Near(d[0], 6.0, 1e-6) && Near(d[2], 6.0, 1e-6) && d[1] == 0.0 && d[3] == 0.0);

        // Into the stall: 36 kN m needs each pitch fin at 15 + (18 - 15) x 2 = 21 deg.
        r = a.Allocate(36_000.0, 0.0, 0.0, priority, d);
        ok &= Check($"36 kN m of pitch: shortfall {r.ShortfallNm:F3} N m, deflections {Text(d)}",
                    r.ShortfallNm < 1e-6 && Near(d[0], 21.0, 1e-6) && Near(d[2], 21.0, 1e-6));

        // Out of reach: the reachable 45 kN m, every pitch fin at its stop.
        r = a.Allocate(60_000.0, 0.0, 0.0, priority, d);
        ok &= Check($"60 kN m of pitch: gives {r.TorqueX:F0}, short {r.ShortfallNm:F0} N m, deflections {Text(d)}",
                    Near(r.TorqueX, pitchMax, 1.0) && Near(r.ShortfallNm, 60_000.0 - pitchMax, 1.0) && Near(d[0], 30.0, 1e-6));

        // Pitch and roll together past what the pitch fins can share: roll last, so pitch is met first.
        var rollLast = new double[] { 1.0, 1.0, 0.1 };
        r = a.Allocate(45_000.0, 0.0, 10_000.0, rollLast, d);
        ok &= Check($"45 kN m pitch + 10 kN m roll, roll last: pitch {r.TorqueX:F0}, roll {r.TorqueZ:F0} N m, deflections {Text(d)}",
                    Near(r.TorqueX, 45_000.0, 1.0) && r.TorqueZ > 0.0);

        // What a set of deflections reports is its curve's.
        double[] set = [10.0, -20.0, 25.0, 0.0];
        AeroTorqueAllocator.Result e = a.Evaluate(set, 0.0, 0.0, 0.0);
        double expect = Curve(10.0) * Slope + Curve(25.0) * Slope;
        ok &= Check($"deflections {Text(set)} give {e.TorqueX:F0} N m of pitch against {expect:F0} off the curves", Near(e.TorqueX, expect, 1e-6));

        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the aero torque allocator" : "FAIL - the aero torque allocator");
        return ok ? 0 : 1;
    }

    /// <summary>The stalling curve in "degrees' worth of the first slope": 1 per degree to 15, 0.5 beyond.</summary>
    private static double Curve(double deg)
    {
        double s = Math.Sign(deg), m = Math.Abs(deg);
        return s * (Math.Min(m, 15.0) + 0.5 * Math.Max(m - 15.0, 0.0));
    }

    /// <summary>
    /// Fins 0 and 2 on the y axis pitch the craft about x; fins 1 and 3 on the z axis yaw it about y. A positive deflection of each rolls it positively about z, and fins 2 and 3 roll it the other way, as a fin on the far side does: a pair deflected together pitches (or yaws) with its rolls cancelling, and deflected against each other rolls alone.
    /// </summary>
    private static AeroTorqueAllocator Cross()
    {
        var a = new AeroTorqueAllocator(4);
        for (int i = 0; i < 4; i++)
        {
            var torque = new double[3 * Grid.Length];
            var force = new double[3 * Grid.Length];
            double rollSign = i < 2 ? 1.0 : -1.0;
            for (int k = 0; k < Grid.Length; k++)
            {
                double c = Curve(Grid[k]);
                int axis = i % 2 == 0 ? 0 : 1;
                torque[3 * k + axis] = c * Slope;
                torque[3 * k + 2] = rollSign * c * RollSlope;
                // Each fin pushes across its own span: the pitch pair along y, the yaw pair along x.
                force[3 * k + (axis == 0 ? 1 : 0)] = -c * 50.0;
            }
            a.SetSurface(i, Grid, torque, force);
        }
        return a;
    }

    private static bool Near(double a, double b, double tol) => Math.Abs(a - b) <= tol;

    private static string Text(ReadOnlySpan<double> d)
    {
        var parts = new string[d.Length];
        for (int i = 0; i < d.Length; i++)
            parts[i] = d[i].ToString("F2", CultureInfo.InvariantCulture);
        return string.Join("/", parts);
    }

    private static bool Check(string what, bool ok)
    {
        Console.WriteLine($"  {what}: {(ok ? "ok" : "FAIL")}");
        return ok;
    }
}
