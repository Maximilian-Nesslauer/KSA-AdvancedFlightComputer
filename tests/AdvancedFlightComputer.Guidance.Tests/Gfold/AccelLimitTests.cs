using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Gfold;

// GfoldParams.AccelMax, the thrust acceleration bound a structural load limit puts on the descent.
// The default Mars case reaches 8.3 to 9.6 m/s^2 at full thrust, so 7 m/s^2 binds and 100 m/s^2 does not.
// The last thrusting node carries at least twice gravity, 7.42 m/s^2, because the end thrust is pinned to zero.
internal static class AccelLimitTests
{
    private const double Tf = 81.0;
    private const int Nodes = 120;

    internal static int Run()
    {
        int failures = 0;
        void Check(string what, bool pass, string detail = "")
        {
            Console.WriteLine($"   {(pass ? "ok  " : "FAIL")} {what}{(detail.Length > 0 ? "   " + detail : "")}");
            if (!pass) failures++;
        }

        var free = new GfoldParams();
        Check("no bound keeps the reference flight-time floor",
            free.TfMin == free.DryMass * Math.Sqrt(free.V0.Sum(v => v * v)) / free.R2, $"{free.TfMin:F3} s");

        // A bound the plan never reaches must not change it.
        Console.WriteLine("non-binding bound (100 m/s^2):");
        GfoldTrajectory p3 = GfoldPlanner.SolveMinError(free, Tf, Nodes);
        GfoldTrajectory p4 = GfoldPlanner.SolveMinFuel(free, Tf, Nodes, p3.LandingPoint);
        var loose = free with { AccelMax = 100 };
        GfoldTrajectory p3Loose = GfoldPlanner.SolveMinError(loose, Tf, Nodes);
        GfoldTrajectory p4Loose = GfoldPlanner.SolveMinFuel(loose, Tf, Nodes, p3Loose.LandingPoint);
        Check("both solve", p4.IsUsable && p4Loose.IsUsable, $"{p4.Status} / {p4Loose.Status}");
        Check("same fuel", Math.Abs(p4Loose.FuelUsed - p4.FuelUsed) <= 1e-5 * p4.FuelUsed,
            $"{p4Loose.FuelUsed:F6} kg against {p4.FuelUsed:F6} kg");
        Check("same flight-time floor", loose.TfMin == free.TfMin);

        // The bound binds on a free-throttle descent: the plan rides it and pays fuel for it.
        Console.WriteLine("binding bound (7 m/s^2), free-throttle descent search:");
        const double aMax = 7.0;
        var limited = free with { AccelMax = aMax };
        Check("the flight-time floor follows the bound",
            Math.Abs(limited.TfMin - Math.Sqrt(free.V0.Sum(v => v * v)) / aMax) < 1e-12, $"{limited.TfMin:F3} s");
        GfoldPlanner.SearchResult? unlimitedSearch = GfoldPlanner.SearchMinFuel(free, Nodes, options: GfoldOptions.Descent);
        GfoldPlanner.SearchResult? limitedSearch = GfoldPlanner.SearchMinFuel(limited, Nodes, options: GfoldOptions.Descent);
        Check("both searches find a landing", unlimitedSearch != null && limitedSearch != null,
            $"unlimited {(unlimitedSearch == null ? "none" : unlimitedSearch.TimeOfFlight.ToString("F1") + " s")}, limited {(limitedSearch == null ? "none" : limitedSearch.TimeOfFlight.ToString("F1") + " s")}");
        if (unlimitedSearch != null && limitedSearch != null)
        {
            GfoldTrajectory t = limitedSearch.Trajectory;
            double worstAccel = 0, worstThrust = double.NegativeInfinity, worstGap = 0, peak = 0;
            for (int n = 0; n < t.Nodes; n++)
            {
                double u = Norm(t.AccelCmd[n]);
                double bound = Bound(limited, n, t.Nodes);
                worstAccel = Math.Max(worstAccel, t.Sigma[n] - bound);
                worstThrust = Math.Max(worstThrust, t.Sigma[n] * t.Mass[n] - Math.Min(limited.R2, bound * t.Mass[n]));
                worstGap = Math.Max(worstGap, Math.Abs(u - t.Sigma[n]));
                if (n < t.Nodes - 2)
                    peak = Math.Max(peak, t.Sigma[n]);
            }
            Check("thrust acceleration stays under the bound", worstAccel <= 1e-5 * aMax, $"worst excess {worstAccel:E2} m/s^2");
            Check("the bound binds", peak >= aMax * (1 - 1e-3), $"peak {peak:F4} m/s^2");
            Check("thrust stays under what the structure takes", worstThrust <= 1e-5 * limited.R2,
                $"worst excess {worstThrust:E2} N");
            Check("the relaxation stays lossless, ||u|| = sigma", worstGap <= 1e-4 * aMax, $"worst gap {worstGap:E2} m/s^2");
            Check("dynamics replay", DynamicsError(t, limited) < 1e-6, $"{DynamicsError(t, limited):E2}");
            Check("the bound costs fuel, never saves it", limitedSearch.FuelUsed >= unlimitedSearch.FuelUsed - 1e-6,
                $"{limitedSearch.FuelUsed:F2} kg against {unlimitedSearch.FuelUsed:F2} kg, tf {limitedSearch.TimeOfFlight:F1} s against {unlimitedSearch.TimeOfFlight:F1} s");
        }

        // Between gravity and twice gravity only the last thrusting node may exceed the bound, by the minimum the pinned zero end thrust needs.
        var low = free with { AccelMax = 5.0 };
        GfoldPlanner.SearchResult? lowSearch = GfoldPlanner.SearchMinFuel(low, Nodes, options: GfoldOptions.Descent);
        Check("a bound between g and 2g still lands", lowSearch != null);
        if (lowSearch != null)
        {
            GfoldTrajectory t = lowSearch.Trajectory;
            double worst = 0;
            for (int n = 0; n < t.Nodes; n++)
                worst = Math.Max(worst, t.Sigma[n] - Bound(low, n, t.Nodes));
            Check("and holds the bound before the last thrusting node", worst <= 1e-5 * low.AccelMax,
                $"worst excess {worst:E2} m/s^2, last node {t.Sigma[t.Nodes - 2]:F3} m/s^2");
        }

        // A bound under the thrust floor: a lunar descent whose engine floor, 5.2 to 7.5 m/s^2, climbs above the bound as the craft gets lighter.
        // The bound gives way to the floor relief on those nodes, as the engine minimum does when stock holds a load limit.
        // The relief follows the floor at the lightest mass a node admits, so the plan can sit above the floor at its own mass there, and the check only holds it to the relief.
        Console.WriteLine("bound under the thrust floor (5.5 m/s^2), real-time options:");
        var floored = new GfoldParams
        {
            GravityMag = 1.62, DryMass = 1600, FuelMass = 700, ThrottleMin = 0.5, VMax = 200,
            R0 = [2400, 300, 0], V0 = [-120, 0, 0], AccelMax = 5.5,
        };
        GfoldPlanner.SearchResult? flooredSearch = null;
        try
        {
            flooredSearch = GfoldPlanner.SearchMinFuel(floored, Nodes, tfLo: 4, tfHi: 120, options: GfoldOptions.RealTime);
            Check("solves without an exception", true);
        }
        catch (Exception e)
        {
            Check("solves without an exception", false, e.Message);
        }
        Check("finds a landing", flooredSearch != null);
        if (flooredSearch != null)
        {
            GfoldTrajectory t = flooredSearch.Trajectory;
            double worst = 0;
            int floorWins = 0;
            for (int n = 0; n < t.Nodes; n++)
            {
                worst = Math.Max(worst, t.Sigma[n] - Bound(floored, n, t.Nodes, t.Dt, floor: true));
                if (n < t.Nodes - 2 && t.Sigma[n] > floored.AccelMax * (1 + 1e-6))
                    floorWins++;
            }
            Check("the bound holds where the floor allows it", worst <= 1e-5 * floored.AccelMax, $"worst excess {worst:E2} m/s^2");
            Check("the bound gives way to the floor relief", floorWins > 0, $"{floorWins} node(s) above the bound");
        }

        Console.WriteLine("invalid bounds:");
        foreach (double bad in new[] { double.NaN, 0.0, -1.0 })
        {
            bool threw = false;
            try { GfoldPlanner.SolveMinFuel(free with { AccelMax = bad }, Tf, Nodes, p3.LandingPoint); }
            catch (ArgumentException) { threw = true; }
            Check($"AccelMax {bad} is refused", threw);
        }
        Check("a search with a zero bound finds no landing",
            GfoldPlanner.SearchMinFuel(free with { AccelMax = 0 }, Nodes, options: GfoldOptions.Descent) == null);

        Console.WriteLine(failures == 0 ? "ACCEL LIMIT PASS" : $"ACCEL LIMIT FAIL ({failures})");
        return failures == 0 ? 0 : 1;
    }

    // The bound GfoldPlanner places on node n, computed here from the physical parameters.
    private static double Bound(GfoldParams p, int n, int nodes, double dt = 0, bool floor = false)
    {
        double bound = p.AccelMax;
        if (floor && n > 0)
            bound = Math.Max(bound, p.R1 / (p.WetMass - p.Alpha * p.R2 * n * dt));
        if (n == nodes - 2)
            bound = Math.Max(bound, 2 * p.GravityMag);
        return bound;
    }

    private static double Norm(double[] v) => Math.Sqrt(v.Sum(x => x * x));

    // Trapezoidal replay of u + g, independent of the solver's residuals.
    private static double DynamicsError(GfoldTrajectory t, GfoldParams p)
    {
        double worst = 0;
        for (int n = 0; n < t.Nodes - 1; n++)
            for (int i = 0; i < 3; i++)
            {
                double g = i == 0 ? -p.GravityMag : 0;
                double vNext = t.Velocity[n][i] + t.Dt / 2 * (t.AccelCmd[n][i] + t.AccelCmd[n + 1][i]) + t.Dt * g;
                double rNext = t.Position[n][i] + t.Dt / 2 * (t.Velocity[n][i] + t.Velocity[n + 1][i]);
                worst = Math.Max(worst, Math.Abs(vNext - t.Velocity[n + 1][i]));
                worst = Math.Max(worst, Math.Abs(rNext - t.Position[n + 1][i]));
            }
        return worst;
    }
}
