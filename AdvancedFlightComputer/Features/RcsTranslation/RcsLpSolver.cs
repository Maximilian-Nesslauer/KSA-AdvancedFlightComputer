namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Dense two-phase simplex for the thruster allocation LP:
///
///   minimize    sum_i cost_i * x_i
///   subject to  A x = b,  x >= 0
///
/// where each column of A is one thruster's wrench per second of firing
/// (3 force rows, 3 torque rows), b is the demanded wrench (unit impulse
/// along the burn direction, zero net torque), and cost_i is the thruster's
/// propellant mass flow. The optimum is a basic solution firing at most as
/// many thrusters as there are independent constraints, which is what makes
/// it fuel-optimal - the linear-programming jet selection of Bergmann and
/// Weiler, "Accommodation of Practical Constraints by a Linear Programming
/// Jet Select" (AIAA 1983-2209). Bland's rule guarantees termination on
/// degenerate problems.
///
/// Problem sizes are tiny (6 rows, tens of columns), so a straightforward
/// dense tableau is fast enough for the main-thread driver cadence.
/// </summary>
internal static class RcsLpSolver
{
    private const double Eps = 1e-9;

    /// <summary>
    /// Solves min cost.x s.t. A x = b, x >= 0. <paramref name="columns"/>
    /// is column-major: columns[i * rows + r] is row r of column i.
    /// Returns null when infeasible (no combination of thrusters can
    /// produce the demanded wrench).
    /// </summary>
    public static double[]? Solve(int rows, int n, double[] columns, double[] cost, double[] b)
    {
        // Row scaling for conditioning: forces are ~1e5 N while unit-impulse
        // components are <= 1, torque rows can be ~1e6 N m. Each row is
        // normalized by its largest coefficient; all-zero rows must have a
        // (near) zero rhs and drop out, otherwise the problem is infeasible.
        double[] rowScale = new double[rows];
        bool[] rowActive = new bool[rows];
        int activeRows = 0;
        for (int r = 0; r < rows; r++)
        {
            double max = Math.Abs(b[r]);
            for (int i = 0; i < n; i++)
                max = Math.Max(max, Math.Abs(columns[i * rows + r]));
            if (max < Eps)
            {
                rowActive[r] = false;
                continue;
            }
            bool anyCoeff = false;
            for (int i = 0; i < n; i++)
            {
                if (Math.Abs(columns[i * rows + r]) > Eps * max)
                {
                    anyCoeff = true;
                    break;
                }
            }
            if (!anyCoeff)
            {
                // No thruster influences this row; feasible only if the
                // demand on it is (relatively) zero.
                if (Math.Abs(b[r]) > 1e-6 * max)
                    return null;
                rowActive[r] = false;
                continue;
            }
            rowScale[r] = 1.0 / max;
            rowActive[r] = true;
            activeRows++;
        }
        if (activeRows == 0)
            return new double[n];

        // Tableau: activeRows x (n + activeRows + 1). Artificial variables
        // occupy columns n..n+activeRows-1, the rhs sits last. Rows are
        // sign-flipped so every rhs is nonnegative.
        int m = activeRows;
        int width = n + m + 1;
        double[] t = new double[m * width];
        int[] basis = new int[m];
        int row = 0;
        for (int r = 0; r < rows; r++)
        {
            if (!rowActive[r])
                continue;
            double sign = b[r] * rowScale[r] >= 0.0 ? 1.0 : -1.0;
            for (int i = 0; i < n; i++)
                t[row * width + i] = sign * columns[i * rows + r] * rowScale[r];
            t[row * width + n + row] = 1.0;
            t[row * width + n + m] = sign * b[r] * rowScale[r];
            basis[row] = n + row;
            row++;
        }

        // Phase 1: minimize the sum of artificials.
        double[] phase1Cost = new double[n + m];
        for (int i = 0; i < m; i++)
            phase1Cost[n + i] = 1.0;
        if (!RunSimplex(t, basis, m, width, phase1Cost, out double phase1Objective))
            return null;
        if (phase1Objective > 1e-7)
            return null;

        // Drive leftover artificials out of the basis; a row where no
        // structural column can pivot is redundant and is zeroed out.
        for (int r2 = 0; r2 < m; r2++)
        {
            if (basis[r2] < n)
                continue;
            int pivotCol = -1;
            for (int i = 0; i < n; i++)
            {
                if (Math.Abs(t[r2 * width + i]) > Eps)
                {
                    pivotCol = i;
                    break;
                }
            }
            if (pivotCol >= 0)
                Pivot(t, basis, m, width, r2, pivotCol);
            else
                for (int c = 0; c < width; c++)
                    t[r2 * width + c] = 0.0;
        }

        // Phase 2: the real cost over structural variables only (artificial
        // columns are barred by an effectively infinite cost).
        double[] phase2Cost = new double[n + m];
        Array.Copy(cost, phase2Cost, n);
        for (int i = 0; i < m; i++)
            phase2Cost[n + i] = double.PositiveInfinity;
        if (!RunSimplex(t, basis, m, width, phase2Cost, out _))
            return null;

        double[] x = new double[n];
        for (int r2 = 0; r2 < m; r2++)
        {
            if (basis[r2] < n)
                x[basis[r2]] = Math.Max(0.0, t[r2 * width + n + m]);
        }
        return x;
    }

    /// <summary>Primal simplex with Bland's anti-cycling rule. False only
    /// when the objective is unbounded, which cannot happen for phase 1
    /// (bounded below by zero) and indicates a malformed problem in
    /// phase 2.</summary>
    private static bool RunSimplex(
        double[] t, int[] basis, int m, int width, double[] varCost, out double objective)
    {
        int rhs = width - 1;
        int numVars = width - 1;
        while (true)
        {
            // Reduced cost per nonbasic column: c_j - c_B . B^-1 A_j, with
            // the tableau already expressing B^-1 A.
            int entering = -1;
            for (int j = 0; j < numVars; j++)
            {
                if (IsBasic(basis, m, j) || double.IsPositiveInfinity(varCost[j]))
                    continue;
                double reduced = varCost[j];
                for (int r = 0; r < m; r++)
                {
                    double cb = varCost[basis[r]];
                    if (double.IsPositiveInfinity(cb))
                        continue;
                    reduced -= cb * t[r * width + j];
                }
                if (reduced < -1e-9)
                {
                    entering = j;
                    break;
                }
            }
            if (entering < 0)
                break;

            int leaving = -1;
            double bestRatio = double.PositiveInfinity;
            for (int r = 0; r < m; r++)
            {
                double a = t[r * width + entering];
                if (a <= Eps)
                    continue;
                double ratio = t[r * width + rhs] / a;
                if (ratio < bestRatio - Eps
                    || (ratio < bestRatio + Eps && (leaving < 0 || basis[r] < basis[leaving])))
                {
                    bestRatio = ratio;
                    leaving = r;
                }
            }
            if (leaving < 0)
            {
                objective = double.NegativeInfinity;
                return false;
            }
            Pivot(t, basis, m, width, leaving, entering);
        }

        objective = 0.0;
        for (int r = 0; r < m; r++)
        {
            double cb = varCost[basis[r]];
            if (!double.IsPositiveInfinity(cb))
                objective += cb * t[r * width + rhs];
        }
        return true;
    }

    private static bool IsBasic(int[] basis, int m, int j)
    {
        for (int r = 0; r < m; r++)
        {
            if (basis[r] == j)
                return true;
        }
        return false;
    }

    private static void Pivot(double[] t, int[] basis, int m, int width, int pivotRow, int pivotCol)
    {
        double pivot = t[pivotRow * width + pivotCol];
        for (int c = 0; c < width; c++)
            t[pivotRow * width + c] /= pivot;
        for (int r = 0; r < m; r++)
        {
            if (r == pivotRow)
                continue;
            double factor = t[r * width + pivotCol];
            if (factor == 0.0)
                continue;
            for (int c = 0; c < width; c++)
                t[r * width + c] -= factor * t[pivotRow * width + c];
        }
        basis[pivotRow] = pivotCol;
    }
}
