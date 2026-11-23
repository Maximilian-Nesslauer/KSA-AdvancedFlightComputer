using AdvancedFlightComputer.Guidance.Gfold;

namespace AdvancedFlightComputer.Guidance.Tests.Gfold;

internal static class WaveAGfoldTests
{
    internal static int Run()
    {
        int fails = 0;
        void Check(string name, bool ok, string detail)
        {
            Console.WriteLine($"  [{(ok ? "PASS" : "FAIL")}] {name}: {detail}");
            if (!ok) fails++;
        }

        Console.WriteLine("G-FOLD node-0 bounds and free-end terminal thrust:");

        // Trapezoidal z[1] = z[0] - alpha*(dt/2)*(s[0] + s[1]) with z0(1) <= z[1] <= z1(1)
        // bounds only the average 0.5*(s[0] + s[1]), so without explicit node-0 bounds s[0]
        // can spike up to 2*R2/m0 when node 1 coasts or sits at R1.
        var pSpike = new GfoldParams
        {
            DryMass = 1505.0,
            FuelMass = 400.0,
            ThrustMax = 13260.0,
            ThrottleMin = 0.25,
            ThrottleMax = 1.00,
            Isp = 225.0,
            GravityMag = 3.7114,
            R0 = [2400.0, 0.0, 0.0],
            V0 = [-10.0, 22.0, 0.0],
            VMax = 200.0,
            GlideSlopeDeg = 5.0,
            PointingMaxDeg = 10.0,
        };
        double aMax0 = pSpike.R2 / pSpike.WetMass;
        double[] rfZero = [0.0, 0.0, 0.0];

        GfoldOptions unguardedDescent = GfoldOptions.Descent with { EnforceNode0Bounds = false };
        GfoldTrajectory tUnguardedSpike = GfoldPlanner.SolveMinFuel(pSpike, tf: 72.0, nodes: 40, landingPoint: rfZero, options: unguardedDescent);
        GfoldTrajectory tBoundedDescent = GfoldPlanner.SolveMinFuel(pSpike, tf: 72.0, nodes: 40, landingPoint: rfZero, options: GfoldOptions.Descent);

        var pRt = pSpike with { V0 = [-10.0, 14.0, 0.0], ThrottleMin = 0.35 };
        double aMinRt = pRt.R1 / pRt.WetMass;
        GfoldOptions unguardedRt = GfoldOptions.RealTime with { EnforceNode0Bounds = false };
        GfoldTrajectory tUnguardedRt = GfoldPlanner.SolveMinFuel(pRt, tf: 72.0, nodes: 40, landingPoint: rfZero, options: unguardedRt);
        GfoldTrajectory tBoundedRt = GfoldPlanner.SolveMinFuel(pRt, tf: 72.0, nodes: 40, landingPoint: rfZero, options: GfoldOptions.RealTime);

        double u0UnguardedSpike = Norm3(tUnguardedSpike.AccelCmd[0]);
        double u0BoundedDescent = Norm3(tBoundedDescent.AccelCmd[0]);
        double u0UnguardedRt = Norm3(tUnguardedRt.AccelCmd[0]);
        double u0BoundedRt = Norm3(tBoundedRt.AccelCmd[0]);
        Check("Node-0 unguarded (Descent, EnforceNode0Bounds=false) spikes above R2/m0",
            tUnguardedSpike.IsUsable && u0UnguardedSpike > aMax0 * 1.25,
            $"||u[0]||={u0UnguardedSpike:F2} m/s^2 vs R2/m0={aMax0:F2} m/s^2 ({100.0 * u0UnguardedSpike / aMax0:F1}%)");
        Check("Node-0 bounded (Descent, EnforceNode0Bounds=true) obeys ||u[0]|| <= R2/m0",
            tBoundedDescent.IsUsable && u0BoundedDescent <= aMax0 * 1.001,
            $"||u[0]||={u0BoundedDescent:F2} m/s^2 <= {aMax0:F2} m/s^2 ({100.0 * u0BoundedDescent / aMax0:F1}%)");
        Check("Node-0 unguarded (RealTime) spikes above R2/m0 while bounded RealTime obeys [R1/m0, R2/m0]",
            tUnguardedRt.IsUsable && u0UnguardedRt > aMax0 * 1.15 && tBoundedRt.IsUsable && u0BoundedRt >= aMinRt * 0.999 && u0BoundedRt <= aMax0 * 1.001,
            $"unguarded={u0UnguardedRt:F2} ({100.0 * u0UnguardedRt / aMax0:F1}%) vs bounded={u0BoundedRt:F2} m/s^2 in [{aMinRt:F2}, {aMax0:F2}]");

        // Low-TWR (1.4 on Luna) terminal flare with pinned s[N-1] = 0 vs FreeTerminalThrust = true.
        const double gLuna = 1.62;
        const double mLuna = 10000.0;
        var pLowTwr = new GfoldParams
        {
            DryMass = 6000.0,
            FuelMass = 4000.0,
            ThrustMax = 1.4 * mLuna * gLuna,
            ThrottleMin = 0.10,
            ThrottleMax = 1.00,
            Isp = 310.0,
            GravityMag = gLuna,
            R0 = [1500.0, 500.0, 0.0],
            V0 = [-25.0, -10.0, 0.0],
            VMax = 120.0,
            GlideSlopeDeg = 10.0,
            PointingMaxDeg = 70.0,
        };
        GfoldOptions pinnedEnd = GfoldOptions.Descent with { FreeTerminalThrust = false };
        GfoldPlanner.SearchResult? sPinned = GfoldPlanner.SearchMinFuel(pLowTwr, nodes: 40, options: pinnedEnd);
        GfoldPlanner.SearchResult? sFree = GfoldPlanner.SearchMinFuel(pLowTwr, nodes: 40, options: GfoldOptions.Descent);

        bool bothFound = sPinned != null && sFree != null;
        Check("Low-TWR (1.4) Luna search finds feasible trajectories for pinned and free-end",
            bothFound,
            bothFound
                ? $"pinned tf={sPinned!.TimeOfFlight:F2} s fuel={sPinned.FuelUsed:F1} kg | free tf={sFree!.TimeOfFlight:F2} s fuel={sFree.FuelUsed:F1} kg"
                : "missing solution");

        if (bothFound)
        {
            GfoldTrajectory tp = sPinned!.Trajectory;
            GfoldTrajectory tfTraj = sFree!.Trajectory;
            double endUPinned = Norm3(tp.AccelCmd[^1]);
            double endUFree = Norm3(tfTraj.AccelCmd[^1]);
            double maxVxPinned = tp.Velocity.Skip(tp.Velocity.Length / 2).Max(v => v[0]);
            double maxVxFree = tfTraj.Velocity.Skip(tfTraj.Velocity.Length / 2).Max(v => v[0]);
            Check("FreeTerminalThrust removes sigma[N-1]=0 pin and reduces fuel",
                endUPinned < 0.05 && endUFree > 0.5 * gLuna && sFree.FuelUsed <= sPinned.FuelUsed + 1e-3,
                $"endU pinned={endUPinned:F3} vs free={endUFree:F3} m/s^2, fuel saved={sPinned.FuelUsed - sFree.FuelUsed:F1} kg");
            Check("FreeTerminalThrust prevents late vertical climb bounce (max vx_late <= 0.1 m/s)",
                maxVxFree <= 0.1,
                $"maxVx_late pinned={maxVxPinned:F2} m/s, free={maxVxFree:F2} m/s");
        }

        Console.WriteLine(fails == 0 ? "ALL G-FOLD NODE-0 AND FLARE CHECKS PASSED." : $"G-FOLD CHECKS FAILED ({fails} failures).");
        return fails == 0 ? 0 : 1;
    }

    private static double Norm3(double[] v) => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
