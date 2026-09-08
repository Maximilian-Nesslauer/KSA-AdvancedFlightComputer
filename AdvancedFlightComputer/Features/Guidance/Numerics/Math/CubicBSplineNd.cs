namespace PoweredGuidance.Numerics;

/// <summary>How the surrogate behaves outside the fitted domain.</summary>
public enum EdgeMode
{
    /// <summary>
    /// First-order Taylor extension from the nearest boundary point. Value and
    /// gradient are both continuous across the boundary, and the surrogate grows
    /// linearly rather than cubically. This is the right choice when an optimiser
    /// may step off the table.
    /// </summary>
    Linear,

    /// <summary>
    /// Hold the boundary value. Cheap, but the gradient drops discontinuously to
    /// zero at the boundary, which a gradient-based solver will feel as a cliff.
    /// </summary>
    Clamp,

    /// <summary>
    /// Let the end polynomial run. Diverges cubically; here for completeness.
    /// </summary>
    Native
}

/// <summary>
/// Interpolating tensor-product cubic B-spline over a rectilinear grid of
/// arbitrary rank, with vector-valued output. No external dependencies.
///
/// Rank N = number of input axes (1 = curve, 2 = surface, 3 = volume, ...).
/// OutputDim M = number of values carried at each grid node.
///
/// End conditions estimate the true end curvature from a local cubic through
/// the four nearest samples, so the fit reproduces any cubic exactly and does
/// not flatten near the domain edges the way a natural spline does. Per axis
/// the collocation system is tridiagonal and solved by the Thomas algorithm,
/// so fitting is O(total nodes); evaluation is O(4^N * M), and a full gradient
/// costs one basis setup plus N+1 contractions.
///
/// Evaluation touches only stack and readonly state, so a fitted instance may
/// be shared across threads.
/// </summary>
public sealed class CubicBSplineNd
{
    private const int Degree = 3;
    private const int Order = Degree + 1;   // 4 nonzero basis functions per span
    private const int MaxDeriv = 2;         // basis orders 0, 1, 2 are retained
    private const int PerAxis = (MaxDeriv + 1) * Order;
    private const int InlineRank = 6;       // ranks up to this avoid heap buffers

    private readonly Axis[] _axes;
    private readonly double[] _coefficients; // row-major over coef grid, M per node
    private readonly int[] _coefStride;      // in nodes, not doubles

    public int Rank => _axes.Length;
    public int OutputDim { get; }

    /// <summary>Fixed at construction: evaluation must not change behaviour mid-flight.</summary>
    public EdgeMode EdgeMode { get; }

    // ---------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------

    /// <param name="grids">One array per axis, each strictly increasing, length >= 2.</param>
    /// <param name="values">
    /// Flattened row-major over the grid shape (axis 0 slowest), with
    /// <paramref name="outputDim"/> contiguous components per node.
    /// </param>
    public static CubicBSplineNd Fit(
        double[][] grids, double[] values, int outputDim = 1,
        EdgeMode edgeMode = EdgeMode.Linear)
    {
        if (grids == null || grids.Length == 0)
            throw new ArgumentException("At least one axis is required.", nameof(grids));
        if (values == null) throw new ArgumentNullException(nameof(values));
        if (outputDim < 1)
            throw new ArgumentOutOfRangeException(nameof(outputDim), "Must be >= 1.");

        int rank = grids.Length;
        long expected = outputDim;
        for (int a = 0; a < rank; a++)
        {
            double[] g = grids[a];
            if (g == null || g.Length < 2)
                throw new ArgumentException($"Axis {a} needs at least 2 samples.", nameof(grids));
            for (int i = 1; i < g.Length; i++)
                if (!(g[i] > g[i - 1]))
                    throw new ArgumentException(
                        $"Axis {a} must be strictly increasing (index {i}).", nameof(grids));
            expected *= g.Length;
        }
        if (values.LongLength != expected)
            throw new ArgumentException(
                $"Expected {expected} values, got {values.LongLength}.", nameof(values));

        var axes = new Axis[rank];
        for (int a = 0; a < rank; a++) axes[a] = new Axis(grids[a]);

        // Successively replace sample values along each axis by B-spline
        // coefficients. After axis a is processed that axis has grown from
        // n to n + 2. Because the spline is a tensor product, doing this one
        // axis at a time is exact -- there is no N-dimensional system.
        var shape = new int[rank];
        for (int a = 0; a < rank; a++) shape[a] = grids[a].Length;

        double[] data = values;
        for (int a = 0; a < rank; a++)
        {
            data = TransformAxis(data, shape, a, axes[a], outputDim);
            shape[a] = axes[a].CoefCount;
        }

        var strides = new int[rank];
        int acc = 1;
        for (int a = rank - 1; a >= 0; a--) { strides[a] = acc; acc *= shape[a]; }

        return new CubicBSplineNd(axes, data, strides, outputDim, edgeMode);
    }

    /// <summary>Convenience overload for a scalar-valued 1D curve.</summary>
    public static CubicBSplineNd Fit(double[] x, double[] y, EdgeMode edgeMode = EdgeMode.Linear) =>
        Fit(new[] { x }, y, 1, edgeMode);

    private CubicBSplineNd(Axis[] axes, double[] coefficients, int[] coefStride,
                           int outputDim, EdgeMode edgeMode)
    {
        _axes = axes;
        _coefficients = coefficients;
        _coefStride = coefStride;
        OutputDim = outputDim;
        EdgeMode = edgeMode;
    }

    private static double[] TransformAxis(
        double[] src, int[] shape, int axis, Axis solver, int outputDim)
    {
        int rank = shape.Length;
        int n = shape[axis];
        int nc = solver.CoefCount;

        // Elements (not nodes) between consecutive entries along this axis.
        int inner = outputDim;
        for (int a = axis + 1; a < rank; a++) inner *= shape[a];
        int outer = 1;
        for (int a = 0; a < axis; a++) outer *= shape[a];

        long dstLen = (long)outer * nc * inner;
        if (dstLen > int.MaxValue) throw new InvalidOperationException("Grid too large.");
        var dst = new double[dstLen];

        var scratch = new double[nc];
        int srcBlock = n * inner;
        int dstBlock = nc * inner;

        for (int o = 0; o < outer; o++)
        {
            int srcBase = o * srcBlock;
            int dstBase = o * dstBlock;
            for (int i = 0; i < inner; i++)
                solver.SolveLine(src, srcBase + i, inner, dst, dstBase + i, inner, scratch);
        }
        return dst;
    }

    // ---------------------------------------------------------------
    // Evaluation
    // ---------------------------------------------------------------

    public double[] Evaluate(params double[] point)
    {
        var result = new double[OutputDim];
        Evaluate(point, result);
        return result;
    }

    /// <summary>Allocation-free evaluation into a caller-supplied buffer.</summary>
    public void Evaluate(ReadOnlySpan<double> point, Span<double> result)
    {
        CheckPoint(point, result);
        int rank = Rank;

        Span<int> spans = rank <= InlineRank ? stackalloc int[InlineRank] : new int[rank];
        Span<double> w = rank <= InlineRank
            ? stackalloc double[InlineRank * PerAxis] : new double[rank * PerAxis];
        Span<double> delta = rank <= InlineRank
            ? stackalloc double[InlineRank] : new double[rank];
        Span<int> order = rank <= InlineRank ? stackalloc int[InlineRank] : new int[rank];
        order.Clear();

        // Only pay for derivative bases if the query actually left the domain.
        int outside = PrepareEdges(point, delta);
        PrepareBasis(point, delta, spans, w, outside > 0 ? 1 : 0);
        Contract(spans, w, order, result);

        if (outside == 0) return;

        // f_ext(u) = f(t) + sum_a delta_a * df/dx_a(t)
        Span<double> term = OutputDim <= 8
            ? stackalloc double[8] : new double[OutputDim];
        for (int a = 0; a < rank; a++)
        {
            if (delta[a] == 0.0) continue;
            order[a] = 1;
            Contract(spans, w, order, term);
            order[a] = 0;
            for (int c = 0; c < OutputDim; c++) result[c] += delta[a] * term[c];
        }
    }

    /// <summary>
    /// Value and full gradient in one pass. <paramref name="gradient"/> is laid out
    /// axis-major: gradient[a * OutputDim + c] is d(result[c]) / d(point[a]).
    /// The gradient is the exact derivative of what <see cref="Evaluate"/> returns,
    /// including the edge-mode extension, so value and slope stay consistent.
    /// </summary>
    public void EvaluateWithGradient(
        ReadOnlySpan<double> point, Span<double> value, Span<double> gradient)
    {
        CheckPoint(point, value);
        int rank = Rank;
        if (gradient.Length < rank * OutputDim)
            throw new ArgumentException("Gradient buffer too small.", nameof(gradient));

        Span<int> spans = rank <= InlineRank ? stackalloc int[InlineRank] : new int[rank];
        Span<double> w = rank <= InlineRank
            ? stackalloc double[InlineRank * PerAxis] : new double[rank * PerAxis];
        Span<double> delta = rank <= InlineRank
            ? stackalloc double[InlineRank] : new double[rank];
        Span<int> order = rank <= InlineRank ? stackalloc int[InlineRank] : new int[rank];
        order.Clear();

        // Second derivatives are only needed for the off-domain mixed terms.
        int outside = PrepareEdges(point, delta);
        PrepareBasis(point, delta, spans, w, outside > 0 ? 2 : 1);
        Contract(spans, w, order, value);

        for (int a = 0; a < rank; a++)
        {
            order[a] = 1;
            Contract(spans, w, order, gradient.Slice(a * OutputDim, OutputDim));
            order[a] = 0;
        }

        if (outside == 0) return;

        // Value picks up the linear terms.
        for (int a = 0; a < rank; a++)
        {
            if (delta[a] == 0.0) continue;
            var g = gradient.Slice(a * OutputDim, OutputDim);
            for (int c = 0; c < OutputDim; c++) value[c] += delta[a] * g[c];
        }

        // For an axis b that is OUTSIDE, d f_ext / d u_b is exactly df/dx_b(t):
        // the boundary point does not move, so the slope is constant out there
        // and the gradient already holds the right number.
        //
        // For an axis b that is INSIDE, moving u_b moves the boundary point that
        // the outside axes extrapolate from, which brings in one mixed second
        // derivative per outside axis. No diagonal second derivatives are needed.
        Span<double> mixed = OutputDim <= 8 ? stackalloc double[8] : new double[OutputDim];
        for (int b = 0; b < rank; b++)
        {
            if (delta[b] != 0.0) continue;              // outside: already correct
            var gb = gradient.Slice(b * OutputDim, OutputDim);
            for (int a = 0; a < rank; a++)
            {
                if (delta[a] == 0.0) continue;
                order[a] = 1; order[b] = 1;             // a != b here, so no clash
                Contract(spans, w, order, mixed);
                order[a] = 0; order[b] = 0;
                for (int c = 0; c < OutputDim; c++) gb[c] += delta[a] * mixed[c];
            }
        }
    }

    /// <summary>Convenience gradient accessor for scalar-valued splines.</summary>
    public double Derivative(ReadOnlySpan<double> point, int axis)
    {
        if (OutputDim != 1)
            throw new InvalidOperationException("Use EvaluateWithGradient for vector output.");
        if ((uint)axis >= (uint)Rank) throw new ArgumentOutOfRangeException(nameof(axis));
        Span<double> v = stackalloc double[1];
        Span<double> g = Rank <= InlineRank ? stackalloc double[InlineRank] : new double[Rank];
        EvaluateWithGradient(point, v, g);
        return g[axis];
    }

    private void CheckPoint(ReadOnlySpan<double> point, Span<double> result)
    {
        if (point.Length != Rank)
            throw new ArgumentException($"Expected {Rank} coordinates.", nameof(point));
        if (result.Length < OutputDim)
            throw new ArgumentException("Result buffer too small.", nameof(result));
    }

    /// <summary>
    /// Records how far outside the domain the query fell on each axis, and returns
    /// how many axes that was. Zero is the common case and lets the caller skip the
    /// derivative bases entirely.
    /// </summary>
    private int PrepareEdges(ReadOnlySpan<double> point, Span<double> delta)
    {
        if (EdgeMode != EdgeMode.Linear)
        {
            delta.Slice(0, Rank).Clear();
            return 0;
        }
        int outside = 0;
        for (int a = 0; a < Rank; a++)
        {
            double u = point[a];
            Axis ax = _axes[a];
            double d = u < ax.Min ? u - ax.Min : (u > ax.Max ? u - ax.Max : 0.0);
            delta[a] = d;
            if (d != 0.0) outside++;
        }
        return outside;
    }

    /// <summary>
    /// Locates the span on each axis and evaluates the basis functions there, up to
    /// <paramref name="maxOrder"/> derivatives. Order 0 takes the lean Cox-de Boor
    /// path, which is what the hot value-only query uses.
    /// </summary>
    private void PrepareBasis(
        ReadOnlySpan<double> point, ReadOnlySpan<double> delta,
        Span<int> spans, Span<double> w, int maxOrder)
    {
        for (int a = 0; a < Rank; a++)
        {
            Axis ax = _axes[a];
            double t = point[a] - delta[a];      // the clamped point
            if (EdgeMode == EdgeMode.Clamp)
            {
                if (t < ax.Min) t = ax.Min;
                else if (t > ax.Max) t = ax.Max;
            }
            int span = ax.FindSpan(t);
            spans[a] = span;
            var slot = w.Slice(a * PerAxis, PerAxis);
            if (maxOrder == 0) ax.BasisValues(span, t, slot);
            else ax.BasisDerivatives(span, t, slot, maxOrder);
        }
    }

    /// <summary>
    /// Contracts the 4^rank active coefficients against the per-axis weights,
    /// taking the derivative order given by <paramref name="order"/> on each axis.
    /// All-zero order gives the value; a single 1 gives a first partial; two 1s
    /// give a mixed second partial.
    /// </summary>
    private void Contract(
        ReadOnlySpan<int> spans, ReadOnlySpan<double> w, ReadOnlySpan<int> order,
        Span<double> result)
    {
        int rank = Rank;
        int m = OutputDim;
        for (int c = 0; c < m; c++) result[c] = 0.0;

        int last = rank - 1;
        int lastStride = _coefStride[last];
        int lastW = last * PerAxis + order[last] * Order;
        int lastBase = spans[last] - Degree;

        Span<int> digits = rank <= InlineRank ? stackalloc int[InlineRank] : new int[rank];
        digits.Clear();

        int outerCombos = 1;
        for (int a = 0; a < last; a++) outerCombos *= Order;

        for (int k = 0; k < outerCombos; k++)
        {
            // Weight product and node offset for every axis but the last.
            double wo = 1.0;
            int node = 0;
            for (int a = 0; a < last; a++)
            {
                wo *= w[a * PerAxis + order[a] * Order + digits[a]];
                node += (spans[a] - Degree + digits[a]) * _coefStride[a];
            }

            if (wo != 0.0)
            {
                int baseOff = node + lastBase * lastStride;
                for (int j = 0; j < Order; j++)
                {
                    double ww = wo * w[lastW + j];
                    int off = (baseOff + j * lastStride) * m;
                    for (int c = 0; c < m; c++) result[c] += ww * _coefficients[off + c];
                }
            }

            for (int a = last - 1; a >= 0; a--)
            {
                if (++digits[a] < Order) break;
                digits[a] = 0;
            }
        }
    }

    // ---------------------------------------------------------------
    // Per-axis machinery
    // ---------------------------------------------------------------

    /// <summary>
    /// Knot vector, span search, basis evaluation, and a prefactored tridiagonal
    /// system for one axis. The factorization depends only on the sample positions,
    /// so it is computed once and reused for every line parallel to this axis.
    /// </summary>
    private sealed class Axis
    {
        private readonly double[] _knots;
        private readonly int _n;

        // Prefactored Thomas system of size n.
        private readonly double[] _mult;  // elimination multipliers
        private readonly double[] _diag;  // post-elimination diagonal
        private readonly double[] _super;
        private readonly double _bcStart; // basis 2nd-deriv weight on c[0]
        private readonly double _bcEnd;   // basis 2nd-deriv weight on c[nc-1]

        // End-curvature estimator: f''(x[0]) ~ sum_j _w0[j]*y[j], and
        // f''(x[n-1]) ~ sum_j _w1[j]*y[n-_endM+j]. Depends only on the grid.
        private readonly double[] _w0 = new double[Order];
        private readonly double[] _w1 = new double[Order];
        private readonly int _endM;

        public int CoefCount => _n + 2;
        public double Min { get; }
        public double Max { get; }

        public Axis(double[] x)
        {
            _n = x.Length;
            Min = x[0];
            Max = x[_n - 1];

            // Clamped knot vector with knots at the interior data sites:
            // [x0 x0 x0 x0, x1 .. x(n-2), x(n-1) x(n-1) x(n-1) x(n-1)]
            _knots = new double[_n + 6];
            for (int i = 0; i < 4; i++) _knots[i] = x[0];
            for (int i = 1; i < _n - 1; i++) _knots[i + 3] = x[i];
            for (int i = 0; i < 4; i++) _knots[_n + 2 + i] = x[_n - 1];

            // End curvature from a local cubic (or quadratic if only 3 samples).
            // With 2 samples there is no curvature information and the weights
            // stay zero, which reproduces the natural condition and the straight
            // line -- the right answer for two points.
            _endM = Math.Min(Order, _n);
            if (_endM >= 3)
            {
                EndCurvatureWeights(x, 0, _endM, x[0], _w0);
                EndCurvatureWeights(x, _n - _endM, _endM, x[_n - 1], _w1);
            }

            int nc = CoefCount;
            var sub = new double[_n];
            var dia = new double[_n];
            var sup = new double[_n];
            var row = new double[nc];
            Span<double> ders = stackalloc double[PerAxis];

            // The two endpoint interpolation conditions collapse to
            // c[0] = y[0] and c[nc-1] = y[n-1] because the knot vector is
            // clamped, which is what leaves a genuinely tridiagonal system
            // in the remaining n unknowns c[1..n].

            // Row 0: S''(x[0]) = estimated end curvature.
            int span = FindSpan(x[0]);
            BasisDerivatives(span, x[0], ders, 2);
            Array.Clear(row, 0, nc);
            for (int j = 0; j < Order; j++) row[span - Degree + j] = ders[2 * Order + j];
            _bcStart = row[0];
            dia[0] = row[1];
            sup[0] = row[2];

            // Rows 1..n-2: interpolation at the interior data sites.
            for (int i = 1; i < _n - 1; i++)
            {
                span = FindSpan(x[i]);
                BasisDerivatives(span, x[i], ders, 2);
                Array.Clear(row, 0, nc);
                for (int j = 0; j < Order; j++) row[span - Degree + j] = ders[j];
                sub[i] = row[i];
                dia[i] = row[i + 1];
                sup[i] = row[i + 2];
            }

            // Row n-1: S''(x[n-1]) = estimated end curvature.
            span = FindSpan(x[_n - 1]);
            BasisDerivatives(span, x[_n - 1], ders, 2);
            Array.Clear(row, 0, nc);
            for (int j = 0; j < Order; j++) row[span - Degree + j] = ders[2 * Order + j];
            sub[_n - 1] = row[_n - 1];
            dia[_n - 1] = row[_n];
            _bcEnd = row[_n + 1];

            // Forward elimination, storing multipliers for reuse. The interior
            // rows are B-spline collocation rows and the two end rows are
            // second-derivative rows; the assembled system is diagonally
            // dominant, so elimination without pivoting is safe here.
            _mult = new double[_n];
            for (int i = 1; i < _n; i++)
            {
                if (Math.Abs(dia[i - 1]) < 1e-300)
                    throw new InvalidOperationException(
                        "Degenerate spline system; check for duplicate sample positions.");
                _mult[i] = sub[i] / dia[i - 1];
                dia[i] -= _mult[i] * sup[i - 1];
            }
            _diag = dia;
            _super = sup;
        }

        /// <summary>
        /// Weights w[0..m-1] with sum_j w[j]*y[offset+j] = p''(evalAt), where p is
        /// the Newton interpolating polynomial through those m samples. Computed by
        /// running the divided-difference recurrence on each unit vector in turn,
        /// which is exact and avoids hand-expanding the algebra.
        /// </summary>
        private static void EndCurvatureWeights(
            double[] x, int offset, int m, double evalAt, double[] w)
        {
            Span<double> dd = stackalloc double[Order];
            for (int e = 0; e < m; e++)
            {
                for (int j = 0; j < m; j++) dd[j] = j == e ? 1.0 : 0.0;
                for (int k = 1; k < m; k++)
                    for (int j = m - 1; j >= k; j--)
                        dd[j] = (dd[j] - dd[j - 1]) / (x[offset + j] - x[offset + j - k]);

                // p(t) = c0 + c1(t-x0) + c2(t-x0)(t-x1) + c3(t-x0)(t-x1)(t-x2)
                // p''(t) = 2*c2 + 2*c3*(3t - x0 - x1 - x2)
                double v = 2.0 * dd[2];
                if (m == 4)
                    v += 2.0 * dd[3] *
                         (3.0 * evalAt - x[offset] - x[offset + 1] - x[offset + 2]);
                w[e] = v;
            }
            for (int e = m; e < Order; e++) w[e] = 0.0;
        }

        /// <summary>
        /// Solves one line: reads n values from src at the given stride and
        /// writes CoefCount coefficients to dst at the given stride.
        /// </summary>
        public void SolveLine(double[] src, int srcOffset, int srcStride,
                              double[] dst, int dstOffset, int dstStride,
                              double[] scratch)
        {
            int n = _n;
            double[] r = scratch;
            double first = src[srcOffset];
            double last = src[srcOffset + (n - 1) * srcStride];

            double d2Start = 0.0, d2End = 0.0;
            for (int j = 0; j < _endM; j++)
            {
                d2Start += _w0[j] * src[srcOffset + j * srcStride];
                d2End += _w1[j] * src[srcOffset + (n - _endM + j) * srcStride];
            }

            r[0] = d2Start - _bcStart * first;
            for (int i = 1; i < n - 1; i++) r[i] = src[srcOffset + i * srcStride];
            r[n - 1] = d2End - _bcEnd * last;

            for (int i = 1; i < n; i++) r[i] -= _mult[i] * r[i - 1];

            double prev = r[n - 1] / _diag[n - 1];
            dst[dstOffset + n * dstStride] = prev;
            for (int i = n - 2; i >= 0; i--)
            {
                prev = (r[i] - _super[i] * prev) / _diag[i];
                dst[dstOffset + (i + 1) * dstStride] = prev;
            }

            dst[dstOffset] = first;
            dst[dstOffset + (n + 1) * dstStride] = last;
        }

        /// <summary>Largest i with knots[i] &lt;= u, restricted to valid spans.</summary>
        public int FindSpan(double u)
        {
            int nc = CoefCount;
            if (u >= _knots[nc]) return nc - 1;
            if (u <= _knots[Degree]) return Degree;
            int lo = Degree, hi = nc;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) >> 1;
                if (u < _knots[mid]) hi = mid; else lo = mid;
            }
            return lo;
        }

        /// <summary>Cox-de Boor: the 4 nonzero cubic basis values at u.</summary>
        public void BasisValues(int span, double u, Span<double> result)
        {
            Span<double> left = stackalloc double[Order];
            Span<double> right = stackalloc double[Order];
            result[0] = 1.0;
            for (int j = 1; j <= Degree; j++)
            {
                left[j] = u - _knots[span + 1 - j];
                right[j] = _knots[span + j] - u;
                double saved = 0.0;
                for (int r = 0; r < j; r++)
                {
                    double temp = result[r] / (right[r + 1] + left[j - r]);
                    result[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                result[j] = saved;
            }
        }

        /// <summary>
        /// The 4 nonzero cubic basis functions at u together with their derivatives up
        /// to <paramref name="maxOrder"/>, laid out as ders[order * 4 + j]. This is the
        /// standard derivative recurrence (Piegl and Tiller, algorithm A2.3), which
        /// shares one triangular table between the values and the derivatives: the
        /// lower triangle holds knot differences, the upper triangle the lower-degree
        /// bases.
        /// </summary>
        public void BasisDerivatives(int span, double u, Span<double> ders, int maxOrder)
        {
            Span<double> ndu = stackalloc double[Order * Order];
            Span<double> left = stackalloc double[Order];
            Span<double> right = stackalloc double[Order];
            Span<double> a = stackalloc double[2 * Order];

            ndu[0] = 1.0;
            for (int j = 1; j <= Degree; j++)
            {
                left[j] = u - _knots[span + 1 - j];
                right[j] = _knots[span + j] - u;
                double saved = 0.0;
                for (int r = 0; r < j; r++)
                {
                    ndu[j * Order + r] = right[r + 1] + left[j - r];
                    double temp = ndu[r * Order + (j - 1)] / ndu[j * Order + r];
                    ndu[r * Order + j] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                ndu[j * Order + j] = saved;
            }

            for (int j = 0; j <= Degree; j++) ders[j] = ndu[j * Order + Degree];

            for (int r = 0; r <= Degree; r++)
            {
                int s1 = 0, s2 = 1;
                a[0] = 1.0;
                for (int k = 1; k <= maxOrder; k++)
                {
                    double d = 0.0;
                    int rk = r - k, pk = Degree - k;
                    if (r >= k)
                    {
                        a[s2 * Order] = a[s1 * Order] / ndu[(pk + 1) * Order + rk];
                        d = a[s2 * Order] * ndu[rk * Order + pk];
                    }
                    int j1 = rk >= -1 ? 1 : -rk;
                    int j2 = r - 1 <= pk ? k - 1 : Degree - r;
                    for (int j = j1; j <= j2; j++)
                    {
                        a[s2 * Order + j] =
                            (a[s1 * Order + j] - a[s1 * Order + j - 1])
                            / ndu[(pk + 1) * Order + rk + j];
                        d += a[s2 * Order + j] * ndu[(rk + j) * Order + pk];
                    }
                    if (r <= pk)
                    {
                        a[s2 * Order + k] = -a[s1 * Order + k - 1] / ndu[(pk + 1) * Order + r];
                        d += a[s2 * Order + k] * ndu[r * Order + pk];
                    }
                    ders[k * Order + r] = d;
                    int t = s1; s1 = s2; s2 = t;
                }
            }

            // The recurrence produces the derivatives up to the factor p!/(p-k)!.
            int f = Degree;
            for (int k = 1; k <= maxOrder; k++)
            {
                for (int j = 0; j <= Degree; j++) ders[k * Order + j] *= f;
                f *= Degree - k;
            }
        }
    }
}
