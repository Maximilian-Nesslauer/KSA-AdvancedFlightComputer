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

        // FROM WHERE THE FINS ARE. Rate-bound: from rest with 2 deg of reach, a big pitch demand gets each pitch fin 2 deg and no more.
        double[] rest = [0.0, 0.0, 0.0, 0.0], reach2 = [2.0, 2.0, 2.0, 2.0];
        r = a.Allocate(36_000.0, 0.0, 0.0, priority, d, rest, reach2, default, default, default);
        ok &= Check($"36 kN m of pitch with 2 deg of reach: gives {r.TorqueX:F0} N m, deflections {Text(d)}",
                    Near(d[0], 2.0, 1e-6) && Near(d[2], 2.0, 1e-6) && Near(r.TorqueX, 4000.0, 1.0));

        // Sticky: fins already giving the demand stay put, even in a split the cheapest answer from rest would not choose (8 and 4 deg, not 6 and 6).
        double[] split = [8.0, 0.0, 4.0, 0.0];
        AeroTorqueAllocator.Result held = a.Evaluate(split, 0.0, 0.0, 0.0);
        r = a.Allocate(held.TorqueX, held.TorqueY, held.TorqueZ, priority, d, split, reach2, default, default, default);
        ok &= Check($"a demand the fins already give moves nothing: deflections {Text(d)}",
                    Near(d[0], 8.0, 1e-6) && Near(d[1], 0.0, 1e-6) && Near(d[2], 4.0, 1e-6) && Near(d[3], 0.0, 1e-6));

        // Home: with nothing asked, every fin heads for rest at its reach.
        double[] out5 = [5.0, 5.0, 5.0, 5.0], reach1 = [1.0, 1.0, 1.0, 1.0];
        r = a.Allocate(0.0, 0.0, 0.0, priority, d, out5, reach1, default, default, default);
        ok &= Check($"no demand takes the fins 1 deg toward rest: deflections {Text(d)}",
                    Near(d[0], 4.0, 1e-6) && Near(d[1], 4.0, 1e-6) && Near(d[2], 4.0, 1e-6) && Near(d[3], 4.0, 1e-6));

        // WITH A BACKUP: two thruster columns, one pure roll (5 kN m), one pure pitch (5 kN m), at 0.02 a newton metre.
        double[] backupTorque = [0.0, 0.0, 5000.0, 5000.0, 0.0, 0.0], backupCost = [100.0, 100.0];
        var duty = new double[2];
        // Within the fins' reach: the fins take it all, the thrusters stay off.
        r = a.Allocate(12_000.0, 0.0, 6_000.0, rollLast, d, rest, default, backupTorque, backupCost, duty);
        ok &= Check($"12 kN m pitch + 6 kN m roll with a backup: fins {r.TorqueX:F0}/{r.TorqueZ:F0} N m, duties {Text(duty)}",
                    Near(r.TorqueX, 12_000.0, 1.0) && Near(r.TorqueZ, 6_000.0, 1.0) && duty[0] < 1e-6 && duty[1] < 1e-6);
        // Past the fins: they give all the pitch they can and the roll goes to the backup, where a fixed order would have left it short.
        r = a.Allocate(45_000.0, 0.0, 10_000.0, rollLast, d, rest, default, backupTorque, backupCost, duty);
        double roll = r.TorqueZ + 5000.0 * duty[0], pitch = r.TorqueX + 5000.0 * duty[1];
        ok &= Check($"45 kN m pitch + 10 kN m roll with a backup: pitch {pitch:F0}, roll {roll:F0} N m (fins {r.TorqueX:F0}/{r.TorqueZ:F0}), duties {Text(duty)}",
                    Near(pitch, 45_000.0, 1.0) && Near(roll, 10_000.0, 1.0) && duty[0] > 0.0);

        // A fin that bites harder further out (500 N m a degree to 15, 1500 beyond): one pass fills the steeper outer piece alone and comes up short; the refinement from its own answer reaches the 15 kN m at 20 deg.
        var steep = new AeroTorqueAllocator(1);
        double[] grid3 = [-30.0, -15.0, 0.0, 15.0, 30.0];
        double Steep(double deg) => Math.Sign(deg) * (500.0 * Math.Min(Math.Abs(deg), 15.0) + 1500.0 * Math.Max(Math.Abs(deg) - 15.0, 0.0));
        var steepTorque = new double[3 * grid3.Length];
        for (int k = 0; k < grid3.Length; k++)
            steepTorque[3 * k] = Steep(grid3[k]);
        steep.SetSurface(0, grid3, steepTorque, new double[3 * grid3.Length]);
        var one = new double[1];
        r = steep.Allocate(15_000.0, 0.0, 0.0, priority, one);
        ok &= Check($"15 kN m from a fin steeper past 15 deg: gives {r.TorqueX:F0} N m at {one[0]:F2} deg",
                    Near(r.TorqueX, 15_000.0, 75.0) && Near(one[0], 20.0, 0.05));

        // 2stage_KSAero's four fins at Mach 0.3, 6 kPa, engine first, +-45 deg (the harness's sweep, 2026-11-20), asmb axes: each fin's roll about as big as its pitch, and the drag it adds growing faster out to 45 deg. Half the pitch reach, met.
        double[][] real =
        [
            [23605.0, 29152.5, -24763.2, 22530.4, 27100.2, -13864.0, 13225.5, 14687.4, -3642.4, 0, 0, 0, -12496.7, -13504.5, -3348.9, -21775.4, -24718.9, -11672.8, -20826.1, -23308.8, -19778.5],
            [27969.5, 29130.5, 35134.8, 27740.1, 17293.1, 34273.6, 20761.9, 6635.1, 25471.8, 0, 0, 0, -17989.1, 4087.2, -20712.7, -25048.3, 13437.2, -29118.3, -23623.0, 22963.2, -26770.6],
            [24529.3, -30434.3, 23550.7, 23572.5, -28805.7, 12752.9, 19968.5, -24349.6, 4191.4, 0, 0, 0, -19173.6, 23158.1, 6300.1, -21048.3, 23814.7, 13132.1, -20839.3, 23491.2, 21625.8],
            [24808.5, -23797.7, -30307.6, 24075.3, -13005.4, -28958.7, 19133.8, -4212.4, -23199.5, 0, 0, 0, -18934.9, -5979.6, 22522.9, -24152.4, -14941.6, 27878.1, -24003.6, -24947.5, 27237.9],
        ];
        double[] grid45 = [-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0];
        var fins = new AeroTorqueAllocator(4);
        for (int i = 0; i < 4; i++)
            fins.SetSurface(i, grid45, real[i], new double[real[i].Length]);
        double pitchReach = Math.Min(fins.Characterise(1, 1).MaxTorqueNm, fins.Characterise(1, -1).MaxTorqueNm);
        var d4 = new double[4];
        r = fins.Allocate(0.0, 0.5 * pitchReach, 0.0, priority, d4);
        ok &= Check($"2stage_KSAero's fins, half the pitch reach of {pitchReach:F0} N m: short {r.ShortfallNm:F0}, gives {r.TorqueX:F0}/{r.TorqueY:F0}/{r.TorqueZ:F0}, deflections {Text(d4)}",
                    r.ShortfallNm < 0.02 * pitchReach);

        // The same craft re-saved with its fins scaled 1.28 (2026-11-20, 20:54), coefficients about twice: a window that could only shrink came 8 % short of half the pitch reach there, and the reach itself, read off the straightened pieces, was twice what the fins give.
        double[][] scaled =
        [
            [67260.8, 65532.6, -70038.3, 66984.8, 65389.0, -40859.2, 49216.0, 46839.0, -14317.6, 0.0, 0.0, 0.0, -51628.6, -47173.1, -12606.9, -70214.1, -62709.4, -38695.6, -70489.1, -61085.2, -68259.2],
            [73419.7, 77324.9, 71826.2, 73833.4, 45978.4, 72851.3, 55061.5, 17155.8, 54776.5, 0.0, 0.0, 0.0, -57499.4, 12997.3, -55335.4, -76138.4, 40983.3, -70596.4, -76592.4, 73196.9, -68910.1],
            [67275.9, -66114.4, 63855.7, 64936.2, -63815.4, 34869.0, 47701.5, -47185.2, 10935.8, 0.0, 0.0, 0.0, -51971.2, 47356.0, 15732.2, -68261.6, 60052.9, 41734.1, -70598.8, 60477.0, 73544.4],
            [72339.2, -68549.1, -70745.7, 71662.7, -38411.6, -70104.8, 54450.7, -12378.8, -53628.9, 0.0, 0.0, 0.0, -54341.1, -16250.4, 50501.1, -74312.3, -45334.2, 67495.1, -75083.1, -78119.8, 66094.0]
        ];
        var big = new AeroTorqueAllocator(4);
        for (int i = 0; i < 4; i++)
            big.SetSurface(i, grid45, scaled[i], new double[scaled[i].Length]);
        double bigReach = Math.Min(big.Characterise(1, 1).MaxTorqueNm, big.Characterise(1, -1).MaxTorqueNm);
        r = big.Allocate(0.0, 0.5 * bigReach, 0.0, priority, d4);
        ok &= Check($"the scaled fins, half the pitch reach of {bigReach:F0} N m: short {r.ShortfallNm:F0}, deflections {Text(d4)}",
                    r.ShortfallNm < 0.02 * bigReach);
        // The reach is one the fins give: asked for it, they come within 2 %.
        r = big.Allocate(0.0, bigReach, 0.0, priority, d4);
        ok &= Check($"the scaled fins, all of the reach: gives {r.TorqueY:F0} of {bigReach:F0} N m, deflections {Text(d4)}",
                    r.TorqueY >= 0.98 * bigReach);

        // Refinement chords crossing 0 deg and current[i] keep exact outward and change cost signs.
        double[] cur4 = [4.0], wide = [30.0];
        r = steep.Allocate(-15_000.0, 0.0, 0.0, priority, one, cur4, wide, default, default, default);
        ok &= Check($"steep fin from +4 deg to -15 kN m across 0 deg: gives {r.TorqueX:F0} N m at {one[0]:F2} deg",
                    r.Solved && Near(r.TorqueX, -15_000.0, 75.0) && Near(one[0], -20.0, 0.05));
        double[] cur20 = [20.0];
        r = steep.Allocate(0.0, 0.0, 0.0, priority, one, cur20, wide, default, default, default);
        ok &= Check($"steep fin from +20 deg to zero demand returns to rest: gives {r.TorqueX:F0} N m at {one[0]:F2} deg",
                    r.Solved && Math.Abs(one[0]) < 0.05 && Math.Abs(r.TorqueX) < 50.0);

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
