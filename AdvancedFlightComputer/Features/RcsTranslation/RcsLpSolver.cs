namespace AdvancedFlightComputer.Features.RcsTranslation;

// Solve min cost.x with A x = b and x >= 0 using two-phase simplex and Bland's rule.
// Columns contain thruster wrenches and costs contain mass flow. See Bergmann and Weiler, "Accommodation of Practical Constraints by a Linear Programming Jet Select" (AIAA 1983-2209).
internal static class RcsLpSolver
{
    private const double Eps = 1e-9;

    // Columns are column-major. Return null for invalid, infeasible, or unbounded inputs.
    public static double[]? Solve(int rows, int n, double[] columns, double[] cost, double[] b)
    {
        if (!HasValidInputs(rows, n, columns, cost, b))
            return null;
        return SolveValid(rows, n, columns, cost, b);
    }

    private static bool HasValidInputs(int rows, int n, double[] columns, double[] cost, double[] b)
    {
        if (rows < 0 || n < 0 || columns == null || cost == null || b == null
            || (long)rows * n != columns.Length || cost.Length != n || b.Length != rows)
            return false;
        foreach (double value in columns)
            if (!double.IsFinite(value)) return false;
        foreach (double value in cost)
            if (!double.IsFinite(value)) return false;
        foreach (double value in b)
            if (!double.IsFinite(value)) return false;
        return true;
    }

    private static double[]? SolveValid(int rows, int n, double[] columns, double[] cost, double[] b)
    {
        if (!ScaleRows(rows, n, columns, b, out double[] rowScale, out bool[] rowActive, out int activeRows))
            return null;
        if (activeRows == 0)
        {
            foreach (double value in cost)
                if (value < 0.0) return null;
            double[] zero = new double[n];
            return SatisfiesConstraints(rows, n, columns, b, zero) ? zero : null;
        }

        BuildTableau(rows, n, columns, b, rowScale, rowActive, activeRows,
            out double[] t, out int[] basis, out int width);
        int m = activeRows;
        if (!FindFeasibleBasis(t, basis, m, n, width))
            return null;

        // Artificial columns cannot enter the basis during the cost phase.
        double[] phase2Cost = new double[n + m];
        Array.Copy(cost, phase2Cost, n);
        for (int i = 0; i < m; i++)
            phase2Cost[n + i] = double.PositiveInfinity;
        if (!RunSimplex(t, basis, m, width, phase2Cost, out _))
            return null;

        double[] x = new double[n];
        for (int r = 0; r < m; r++)
            if (basis[r] < n)
                x[basis[r]] = Math.Max(0.0, t[r * width + width - 1]);
        return SatisfiesConstraints(rows, n, columns, b, x) ? x : null;
    }

    private static bool ScaleRows(int rows, int n, double[] columns, double[] b,
        out double[] rowScale, out bool[] rowActive, out int activeRows)
    {
        // Normalize force and torque rows separately because their magnitudes can differ greatly.
        rowScale = new double[rows];
        rowActive = new bool[rows];
        activeRows = 0;
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
                    return false;
                rowActive[r] = false;
                continue;
            }
            rowScale[r] = 1.0 / max;
            rowActive[r] = true;
            activeRows++;
        }
        return true;
    }

    private static void BuildTableau(int rows, int n, double[] columns, double[] b,
        double[] rowScale, bool[] rowActive, int m,
        out double[] t, out int[] basis, out int width)
    {
        // Artificial variables start in the basis. Flip rows to keep every right-hand side nonnegative.
        width = n + m + 1;
        t = new double[m * width];
        basis = new int[m];
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

    }

    private static bool FindFeasibleBasis(double[] t, int[] basis, int m, int n, int width)
    {
        // A feasible basis has zero artificial-variable cost.
        double[] phase1Cost = new double[n + m];
        for (int i = 0; i < m; i++)
            phase1Cost[n + i] = 1.0;
        if (!RunSimplex(t, basis, m, width, phase1Cost, out double phase1Objective))
            return false;
        if (phase1Objective > 1e-7)
            return false;

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

        return true;
    }

    private static bool SatisfiesConstraints(int rows, int n, double[] columns, double[] b, double[] x)
    {
        foreach (double value in x)
            if (!double.IsFinite(value)) return false;
        // Check the original units because a small scaled residual can hide a large unmet demand.
        for (int r = 0; r < rows; r++)
        {
            double sum = 0.0;
            double magnitude = 0.0;
            for (int i = 0; i < n; i++)
            {
                double term = columns[i * rows + r] * x[i];
                sum += term;
                magnitude += Math.Abs(term);
            }
            double tolerance = 1e-10 + 1e-6 * Math.Max(Math.Abs(b[r]), magnitude);
            if (!double.IsFinite(sum) || !double.IsFinite(magnitude) || Math.Abs(sum - b[r]) > tolerance)
                return false;
        }
        return true;
    }

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
