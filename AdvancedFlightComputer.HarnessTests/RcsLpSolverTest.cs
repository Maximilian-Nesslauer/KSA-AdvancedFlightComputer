using AdvancedFlightComputer.Features.RcsTranslation;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the LP allocator's simplex on hand-checkable problems: cost
// optimality (the cheap thruster wins), zero-torque constraint
// satisfaction (an off-CoM pair splits evenly), support selection between
// a torque-coupled pair and a clean but expensive thruster, and clean
// infeasibility (unreachable direction, un-nullable torque) returning
// null instead of garbage. Columns are wrenches per second of firing:
// rows 0-2 force, rows 3-5 torque.
public sealed class RcsLpSolverTest : IHarnessTest
{
    private const double Tol = 1e-6;

    public string Name => "afc-rcs-lp-solver";

    public int Run(HeadlessSession session)
    {
        bool ok = true;
        ok &= CheckCheapThrusterWins();
        ok &= CheckTorqueNullingPair();
        ok &= CheckSupportSelection();
        ok &= CheckPricedTorqueSlack();
        ok &= CheckInfeasibleDirection();
        ok &= CheckUnnullableTorque();
        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckCheapThrusterWins()
    {
        // Two identical +X thrusters through the CoM, one twice as thirsty.
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 0 },
            new[] { 10.0, 0, 0, 0, 0, 0 });
        double[]? x = RcsLpSolver.Solve(6, 2, columns, new[] { 1.0, 2.0 },
            new[] { 1.0, 0, 0, 0, 0, 0 });
        bool ok = Check("cheap wins: solved", x != null);
        if (x != null)
        {
            ok &= Near("cheap wins: x0", x[0], 0.1);
            ok &= Near("cheap wins: x1", x[1], 0.0);
        }
        return ok;
    }

    private bool CheckTorqueNullingPair()
    {
        // Two +X thrusters with opposite yaw torque: the only zero-torque
        // combination is the even split.
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 5.0 },
            new[] { 10.0, 0, 0, 0, 0, -5.0 });
        double[]? x = RcsLpSolver.Solve(6, 2, columns, new[] { 1.0, 1.0 },
            new[] { 1.0, 0, 0, 0, 0, 0 });
        bool ok = Check("torque pair: solved", x != null);
        if (x != null)
        {
            ok &= Near("torque pair: x0", x[0], 0.05);
            ok &= Near("torque pair: x1", x[1], 0.05);
            ok &= CheckResiduals("torque pair", columns, 2, x, new[] { 1.0, 0, 0, 0, 0, 0 });
        }
        return ok;
    }

    private bool CheckSupportSelection()
    {
        // Torque-coupled pair (combined cost 0.1 per unit impulse) vs a
        // clean single thruster: the LP must pick whichever is cheaper.
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 5.0 },
            new[] { 10.0, 0, 0, 0, 0, -5.0 },
            new[] { 10.0, 0, 0, 0, 0, 0 });
        double[] rhs = { 1.0, 0, 0, 0, 0, 0 };

        // Clean thruster costs 0.25 per unit impulse: the pair wins.
        double[]? x = RcsLpSolver.Solve(6, 3, columns, new[] { 1.0, 1.0, 2.5 }, rhs);
        bool ok = Check("expensive clean: solved", x != null);
        if (x != null)
        {
            ok &= Near("expensive clean: pair fires", x[0] + x[1], 0.1);
            ok &= Near("expensive clean: clean idle", x[2], 0.0);
        }

        // Clean thruster costs 0.09 per unit impulse: it wins alone.
        x = RcsLpSolver.Solve(6, 3, columns, new[] { 1.0, 1.0, 0.9 }, rhs);
        ok &= Check("cheap clean: solved", x != null);
        if (x != null)
        {
            ok &= Near("cheap clean: clean fires", x[2], 0.1);
            ok &= Near("cheap clean: pair idle", x[0] + x[1], 0.0);
        }
        return ok;
    }

    // Mirrors the executor's torque-slack columns: unit yaw-torque columns
    // with a per-unit price. With cheap slack the LP fires only the lean
    // thruster and hands its torque to the attitude hold; with expensive
    // slack the balanced (but thirstier) pair wins. Cost accounting:
    // lean-only is 0.1*1 + 0.5*price versus the pair's 0.05*1 + 0.05*3.
    private bool CheckPricedTorqueSlack()
    {
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 5.0 },
            new[] { 10.0, 0, 0, 0, 0, -5.0 },
            new[] { 0.0, 0, 0, 0, 0, 1.0 },
            new[] { 0.0, 0, 0, 0, 0, -1.0 });
        double[] rhs = { 1.0, 0, 0, 0, 0, 0 };

        double[]? x = RcsLpSolver.Solve(6, 4, columns, new[] { 1.0, 3.0, 0.02, 0.02 }, rhs);
        bool ok = Check("cheap slack: solved", x != null);
        if (x != null)
        {
            ok &= Near("cheap slack: lean fires", x[0], 0.1);
            ok &= Near("cheap slack: thirsty idle", x[1], 0.0);
            ok &= Near("cheap slack: slack absorbs", x[3], 0.5);
        }

        x = RcsLpSolver.Solve(6, 4, columns, new[] { 1.0, 3.0, 1.0, 1.0 }, rhs);
        ok &= Check("dear slack: solved", x != null);
        if (x != null)
        {
            ok &= Near("dear slack: pair splits a", x[0], 0.05);
            ok &= Near("dear slack: pair splits b", x[1], 0.05);
            ok &= Near("dear slack: slack idle", x[2] + x[3], 0.0);
        }
        return ok;
    }

    private bool CheckInfeasibleDirection()
    {
        double[] columns = BuildColumns(new[] { 10.0, 0, 0, 0, 0, 0 });
        double[]? x = RcsLpSolver.Solve(6, 1, columns, new[] { 1.0 },
            new[] { -1.0, 0, 0, 0, 0, 0 });
        return Check("unreachable direction: null", x == null);
    }

    private bool CheckUnnullableTorque()
    {
        // A single off-CoM thruster cannot produce force without torque.
        double[] columns = BuildColumns(new[] { 10.0, 0, 0, 0, 0, 5.0 });
        double[]? x = RcsLpSolver.Solve(6, 1, columns, new[] { 1.0 },
            new[] { 1.0, 0, 0, 0, 0, 0 });
        return Check("un-nullable torque: null", x == null);
    }

    private bool CheckResiduals(string label, double[] columns, int n, double[] x, double[] rhs)
    {
        bool ok = true;
        for (int r = 0; r < 6; r++)
        {
            double sum = 0.0;
            for (int i = 0; i < n; i++)
                sum += columns[i * 6 + r] * x[i];
            if (Math.Abs(sum - rhs[r]) > Tol)
            {
                HarnessLog.Line($"[{Name}] TEST {label}: row {r} residual {sum - rhs[r]:E3} => FAIL");
                ok = false;
            }
        }
        return ok;
    }

    private static double[] BuildColumns(params double[][] wrenches)
    {
        double[] columns = new double[wrenches.Length * 6];
        for (int i = 0; i < wrenches.Length; i++)
        {
            for (int r = 0; r < 6; r++)
                columns[i * 6 + r] = wrenches[i][r];
        }
        return columns;
    }

    private bool Near(string label, double actual, double expected)
    {
        bool ok = Math.Abs(actual - expected) < 1e-5;
        if (!ok)
            HarnessLog.Line($"[{Name}] TEST {label}: got {actual}, expected {expected} => FAIL");
        return ok;
    }

    private bool Check(string label, bool condition)
    {
        if (!condition)
            HarnessLog.Line($"[{Name}] TEST {label} => FAIL");
        return condition;
    }
}
