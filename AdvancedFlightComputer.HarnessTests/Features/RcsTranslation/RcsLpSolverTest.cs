using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the LP allocator's simplex on hand-checkable problems: cost
// optimality (the cheap thruster wins), zero-torque constraint
// satisfaction (an off-CoM pair splits evenly), support selection between
// a torque-coupled pair and a clean but expensive thruster, and clean
// infeasibility (unreachable direction, un-nullable torque) returning
// null instead of garbage. Columns are wrenches per second of firing:
// rows 0-2 force, rows 3-5 torque.
public sealed class RcsLpSolverTest : AfcTest
{
    private const double Tol = 1e-6;
    private const double SolutionTol = 1e-5;

    public override string Name => "afc-rcs-lp-solver";

    protected override void Execute(TestContext t)
    {
        CheckCheapThrusterWins(t);
        CheckTorqueNullingPair(t);
        CheckSupportSelection(t);
        CheckPricedTorqueSlack(t);
        CheckInfeasibleDirection(t);
        CheckUnnullableTorque(t);
    }

    private static void CheckCheapThrusterWins(TestContext t)
    {
        // Two identical +X thrusters through the CoM, one twice as thirsty.
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 0 },
            new[] { 10.0, 0, 0, 0, 0, 0 });
        double[]? x = RcsLpSolver.Solve(6, 2, columns, new[] { 1.0, 2.0 },
            new[] { 1.0, 0, 0, 0, 0, 0 });
        if (!t.Check("cheap wins: solved", x != null))
            return;
        t.CheckAbs("cheap wins: x0", x![0], 0.1, SolutionTol);
        t.CheckAbs("cheap wins: x1", x[1], 0.0, SolutionTol);
    }

    private static void CheckTorqueNullingPair(TestContext t)
    {
        // Two +X thrusters with opposite yaw torque: the only zero-torque
        // combination is the even split.
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 5.0 },
            new[] { 10.0, 0, 0, 0, 0, -5.0 });
        double[] rhs = { 1.0, 0, 0, 0, 0, 0 };
        double[]? x = RcsLpSolver.Solve(6, 2, columns, new[] { 1.0, 1.0 }, rhs);
        if (!t.Check("torque pair: solved", x != null))
            return;
        t.CheckAbs("torque pair: x0", x![0], 0.05, SolutionTol);
        t.CheckAbs("torque pair: x1", x[1], 0.05, SolutionTol);
        CheckResiduals(t, "torque pair", columns, 2, x, rhs);
    }

    private static void CheckSupportSelection(TestContext t)
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
        if (t.Check("expensive clean: solved", x != null))
        {
            t.CheckAbs("expensive clean: pair fires", x![0] + x[1], 0.1, SolutionTol);
            t.CheckAbs("expensive clean: clean idle", x[2], 0.0, SolutionTol);
        }

        // Clean thruster costs 0.09 per unit impulse: it wins alone.
        x = RcsLpSolver.Solve(6, 3, columns, new[] { 1.0, 1.0, 0.9 }, rhs);
        if (t.Check("cheap clean: solved", x != null))
        {
            t.CheckAbs("cheap clean: clean fires", x![2], 0.1, SolutionTol);
            t.CheckAbs("cheap clean: pair idle", x[0] + x[1], 0.0, SolutionTol);
        }
    }

    // Mirrors the executor's torque-slack columns: unit yaw-torque columns
    // with a per-unit price. With cheap slack the LP fires only the lean
    // thruster and hands its torque to the attitude hold; with expensive
    // slack the balanced (but thirstier) pair wins. Cost accounting:
    // lean-only is 0.1*1 + 0.5*price versus the pair's 0.05*1 + 0.05*3.
    private static void CheckPricedTorqueSlack(TestContext t)
    {
        double[] columns = BuildColumns(
            new[] { 10.0, 0, 0, 0, 0, 5.0 },
            new[] { 10.0, 0, 0, 0, 0, -5.0 },
            new[] { 0.0, 0, 0, 0, 0, 1.0 },
            new[] { 0.0, 0, 0, 0, 0, -1.0 });
        double[] rhs = { 1.0, 0, 0, 0, 0, 0 };

        double[]? x = RcsLpSolver.Solve(6, 4, columns, new[] { 1.0, 3.0, 0.02, 0.02 }, rhs);
        if (t.Check("cheap slack: solved", x != null))
        {
            t.CheckAbs("cheap slack: lean fires", x![0], 0.1, SolutionTol);
            t.CheckAbs("cheap slack: thirsty idle", x[1], 0.0, SolutionTol);
            t.CheckAbs("cheap slack: slack absorbs", x[3], 0.5, SolutionTol);
        }

        x = RcsLpSolver.Solve(6, 4, columns, new[] { 1.0, 3.0, 1.0, 1.0 }, rhs);
        if (t.Check("dear slack: solved", x != null))
        {
            t.CheckAbs("dear slack: pair splits a", x![0], 0.05, SolutionTol);
            t.CheckAbs("dear slack: pair splits b", x[1], 0.05, SolutionTol);
            t.CheckAbs("dear slack: slack idle", x[2] + x[3], 0.0, SolutionTol);
        }
    }

    private static void CheckInfeasibleDirection(TestContext t)
    {
        double[] columns = BuildColumns(new[] { 10.0, 0, 0, 0, 0, 0 });
        double[]? x = RcsLpSolver.Solve(6, 1, columns, new[] { 1.0 },
            new[] { -1.0, 0, 0, 0, 0, 0 });
        t.Check("unreachable direction: null", x == null);
    }

    private static void CheckUnnullableTorque(TestContext t)
    {
        // A single off-CoM thruster cannot produce force without torque.
        double[] columns = BuildColumns(new[] { 10.0, 0, 0, 0, 0, 5.0 });
        double[]? x = RcsLpSolver.Solve(6, 1, columns, new[] { 1.0 },
            new[] { 1.0, 0, 0, 0, 0, 0 });
        t.Check("un-nullable torque: null", x == null);
    }

    // Every wrench row the solution produces must land on the requested right-hand side; reported as
    // the worst row, so one line names which constraint drifted.
    private static void CheckResiduals(
        TestContext t, string label, double[] columns, int n, double[] x, double[] rhs)
    {
        double worst = 0.0;
        int worstRow = 0;
        for (int r = 0; r < 6; r++)
        {
            double sum = 0.0;
            for (int i = 0; i < n; i++)
                sum += columns[i * 6 + r] * x[i];
            double residual = Math.Abs(sum - rhs[r]);
            if (residual > worst)
            {
                worst = residual;
                worstRow = r;
            }
        }
        t.Check($"{label}: residuals", worst <= Tol, $"worst row {worstRow} off by {worst:E3}");
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
}
