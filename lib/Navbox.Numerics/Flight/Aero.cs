using Navbox.Numerics;

namespace Navbox.Flight;

/// <summary>
/// Tabulated axial drag coefficient Cd(Mach, alpha), fitted once to a
/// tensor-product cubic B-spline and queried thereafter for both the value and
/// its slopes.
///
/// The point of the spline rather than bilinear lookup is the slopes. SCvx
/// linearises the dynamics at every node on every iteration, so an aero model
/// with a discontinuous or piecewise-constant gradient shows up as trust-region
/// chatter, not as a small modelling error. The spline is C2 in both inputs and
/// its derivatives are analytic, so <see cref="Cd(Dual, Dual)"/> hands the AD
/// sweep an exact slope at the same cost as the value.
///
/// The numbers below are a plausible slender-booster set, NOT measured data:
/// a subsonic plateau near 0.315, a transonic rise peaking near M = 1.25, a
/// supersonic decay, and a cross-flow increment that grows roughly with
/// sin^2(alpha). Replace wholesale when real data arrives; nothing outside this
/// file depends on the particular values.
/// </summary>
public sealed class AeroTable
{
    // Mach breakpoints. Spacing is deliberately fine through 0.85 - 1.30: a cubic
    // fit through a coarsely sampled drag rise overshoots the peak, and the
    // overshoot is larger in dCd/dM than in Cd. Real aero tables are sampled this
    // way through the transonic region for the same reason.
    private static readonly double[] DefaultMachGrid =
    {
        0.00, 0.20, 0.40, 0.60, 0.70, 0.80, 0.85, 0.90,
        0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.30, 1.40,
        1.60, 1.80, 2.00, 2.50, 3.00, 3.50, 4.00, 5.00
    };

    private static readonly double[] DefaultAlphaGridDeg =
    {
        0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0
    };

    private static readonly double[] DefaultCdTable =
    {
        // M = 0.00
        0.3150, 0.3172, 0.3238, 0.3347, 0.3499, 0.3693, 0.3929, 0.4357, 0.5258, 0.6368, 0.7655,
        // M = 0.20
        0.3150, 0.3172, 0.3238, 0.3347, 0.3500, 0.3695, 0.3931, 0.4360, 0.5263, 0.6376, 0.7665,
        // M = 0.40
        0.3151, 0.3173, 0.3240, 0.3350, 0.3504, 0.3700, 0.3938, 0.4370, 0.5280, 0.6402, 0.7701,
        // M = 0.60
        0.3168, 0.3191, 0.3259, 0.3372, 0.3529, 0.3730, 0.3973, 0.4416, 0.5347, 0.6495, 0.7824,
        // M = 0.70
        0.3224, 0.3247, 0.3317, 0.3432, 0.3593, 0.3799, 0.4048, 0.4501, 0.5454, 0.6629, 0.7990,
        // M = 0.80
        0.3437, 0.3461, 0.3533, 0.3652, 0.3819, 0.4032, 0.4290, 0.4759, 0.5745, 0.6962, 0.8371,
        // M = 0.85
        0.3695, 0.3720, 0.3794, 0.3916, 0.4086, 0.4304, 0.4567, 0.5046, 0.6055, 0.7298, 0.8737,
        // M = 0.90
        0.4139, 0.4164, 0.4240, 0.4365, 0.4539, 0.4762, 0.5032, 0.5523, 0.6556, 0.7829, 0.9303,
        // M = 0.95
        0.4809, 0.4835, 0.4912, 0.5041, 0.5220, 0.5448, 0.5725, 0.6229, 0.7288, 0.8594, 1.0107,
        // M = 1.00
        0.5650, 0.5676, 0.5756, 0.5888, 0.6071, 0.6306, 0.6590, 0.7107, 0.8194, 0.9535, 1.1087,
        // M = 1.05
        0.6491, 0.6518, 0.6599, 0.6735, 0.6923, 0.7164, 0.7455, 0.7985, 0.9101, 1.0475, 1.2068,
        // M = 1.10
        0.7161, 0.7189, 0.7272, 0.7411, 0.7603, 0.7850, 0.8148, 0.8691, 0.9833, 1.1241, 1.2872,
        // M = 1.15
        0.7605, 0.7633, 0.7718, 0.7859, 0.8056, 0.8308, 0.8613, 0.9167, 1.0334, 1.1772, 1.3438,
        // M = 1.20
        0.7863, 0.7892, 0.7979, 0.8123, 0.8324, 0.8580, 0.8891, 0.9455, 1.0643, 1.2108, 1.3804,
        // M = 1.30
        0.7874, 0.7903, 0.7992, 0.8141, 0.8347, 0.8610, 0.8930, 0.9510, 1.0732, 1.2238, 1.3983,
        // M = 1.40
        0.7730, 0.7761, 0.7851, 0.8002, 0.8212, 0.8480, 0.8806, 0.9397, 1.0640, 1.2173, 1.3949,
        // M = 1.60
        0.7376, 0.7407, 0.7499, 0.7652, 0.7866, 0.8139, 0.8469, 0.9071, 1.0335, 1.1895, 1.3701,
        // M = 1.80
        0.7036, 0.7067, 0.7160, 0.7314, 0.7529, 0.7803, 0.8136, 0.8740, 1.0012, 1.1580, 1.3396,
        // M = 2.00
        0.6723, 0.6754, 0.6847, 0.7002, 0.7217, 0.7491, 0.7825, 0.8430, 0.9704, 1.1274, 1.3093,
        // M = 2.50
        0.6046, 0.6077, 0.6170, 0.6325, 0.6540, 0.6815, 0.7149, 0.7754, 0.9029, 1.0601, 1.2421,
        // M = 3.00
        0.5498, 0.5529, 0.5622, 0.5776, 0.5992, 0.6267, 0.6600, 0.7206, 0.8481, 1.0052, 1.1873,
        // M = 3.50
        0.5053, 0.5084, 0.5177, 0.5332, 0.5547, 0.5822, 0.6155, 0.6761, 0.8036, 0.9607, 1.1428,
        // M = 4.00
        0.4693, 0.4724, 0.4817, 0.4971, 0.5186, 0.5461, 0.5795, 0.6401, 0.7675, 0.9247, 1.1068,
        // M = 5.00
        0.4164, 0.4195, 0.4288, 0.4442, 0.4657, 0.4932, 0.5266, 0.5872, 0.7146, 0.8718, 1.0539,
    };

    private readonly CubicBSplineNd _cd;
    private readonly double[] _machGrid;

    public double MachMin => _machGrid[0];
    public double MachMax => _machGrid[^1];
    public double AlphaMaxRad { get; }

    /// <summary>
    /// The built-in placeholder table. Useful headless and as a fallback, but a
    /// flight build should be constructing this from data sampled off the game -
    /// see the data-driven constructor.
    /// </summary>
    public AeroTable() : this(DefaultMachGrid, DefaultAlphaGridDeg, DefaultCdTable) { }

    /// <summary>
    /// Fit from supplied data. This is the constructor the mod uses: it samples
    /// KSA's own aero onto a grid ONCE, on the main thread, and hands the numbers
    /// across as plain arrays.
    ///
    /// The arrays are copied, and nothing here retains a reference to anything the
    /// game owns. That is the whole point - a solve must read nothing that can
    /// change once it has started, and a live callback into the game could return
    /// different answers to different nodes of the same linearisation. Sampling to
    /// a fixed grid at setup is what makes the surrogate safe to hand to a worker
    /// thread; see Ksa6DofInputs for the same argument applied to bias and inertia.
    /// </summary>
    /// <param name="machGrid">Strictly increasing Mach breakpoints. Sample finely
    /// through the transonic rise - a coarse grid there overshoots the peak, and
    /// the overshoot is worse in dCd/dM than in Cd.</param>
    /// <param name="alphaGridDeg">Strictly increasing angle-of-attack breakpoints,
    /// in DEGREES; converted here so no caller has to think about units.</param>
    /// <param name="cdTable">Row-major, Mach slowest:
    /// cdTable[i * alphaGridDeg.Length + j] = Cd(machGrid[i], alphaGridDeg[j]).</param>
    public AeroTable(double[] machGrid, double[] alphaGridDeg, double[] cdTable)
    {
        if (machGrid is null) throw new ArgumentNullException(nameof(machGrid));
        if (alphaGridDeg is null) throw new ArgumentNullException(nameof(alphaGridDeg));
        if (cdTable is null) throw new ArgumentNullException(nameof(cdTable));
        if (cdTable.Length != machGrid.Length * alphaGridDeg.Length)
            throw new ArgumentException(
                $"Expected {machGrid.Length} x {alphaGridDeg.Length} = "
                + $"{machGrid.Length * alphaGridDeg.Length} values, got {cdTable.Length}.",
                nameof(cdTable));

        // A single NaN here would propagate through Cd into the Jacobian and on into
        // SCS, which is native and does not validate its input. Catch it at the
        // boundary, where the message can still say which value was bad.
        for (int i = 0; i < cdTable.Length; i++)
            if (!double.IsFinite(cdTable[i]))
                throw new ArgumentException($"Cd[{i}] is not finite ({cdTable[i]}).", nameof(cdTable));

        _machGrid = (double[])machGrid.Clone();
        var alphaRad = new double[alphaGridDeg.Length];
        for (int j = 0; j < alphaGridDeg.Length; j++)
            alphaRad[j] = alphaGridDeg[j] * Math.PI / 180.0;
        AlphaMaxRad = alphaRad[^1];

        // EdgeMode.Linear matters more than it looks. The optimiser's iterate
        // wanders off the table routinely - past the last Mach breakpoint early in a
        // descent, or to an alpha the table never covered on a bad iteration - and
        // clamping would drop the gradient discontinuously to zero exactly there.
        // Linear extension keeps the slope continuous across the boundary, so a step
        // off the table costs accuracy rather than breaking the linearisation.
        _cd = CubicBSplineNd.Fit(new[] { _machGrid, alphaRad }, (double[])cdTable.Clone(),
                                 1, EdgeMode.Linear);
    }

    /// <summary>Cd at a Mach number and angle of attack (radians). Value only.</summary>
    public double Cd(double mach, double alphaRad)
    {
        Span<double> point = stackalloc double[2] { mach, alphaRad };
        Span<double> result = stackalloc double[1];
        _cd.Evaluate(point, result);
        return result[0];
    }

    /// <summary>Cd together with dCd/dMach and dCd/dAlpha, all analytic.</summary>
    public double Cd(double mach, double alphaRad, out double dCdMach, out double dCdAlpha)
    {
        Span<double> point = stackalloc double[2] { mach, alphaRad };
        Span<double> value = stackalloc double[1];
        Span<double> grad = stackalloc double[2];
        _cd.EvaluateWithGradient(point, value, grad);
        dCdMach = grad[0];
        dCdAlpha = grad[1];
        return value[0];
    }

    /// <summary>
    /// The autodiff bridge: Cd as a <see cref="Dual"/>, so aero can be written
    /// inline in dynamics code alongside everything else.
    ///
    /// This does NOT differentiate through the spline evaluation, and should not.
    /// Sweeping Duals through de Boor's recurrence would work - it is smooth in
    /// the query point - but it would cost several times more and buy nothing,
    /// because the analytic gradient is already exact. The span search it also
    /// contains is a discrete branch with no meaningful derivative at all.
    ///
    /// Instead this is a custom derivative rule: take the value and the exact
    /// gradient from the spline, then apply the chain rule to whatever the caller
    /// seeded. That is the standard way to embed a tabulated function in a
    /// forward-AD framework, and it makes the table a first-class differentiable
    /// primitive rather than something the sweep has to be routed around.
    /// </summary>
    public Dual Cd(Dual mach, Dual alpha)
    {
        double value = Cd(mach.V, alpha.V, out double dM, out double dA);
        return new Dual(value, dM * mach.D + dA * alpha.D);
    }

    /// <summary>
    /// Angle of attack: the angle between the body +z axis (the vehicle's long
    /// axis, matching the thrust direction in <see cref="Dynamics6Dof"/>) and the
    /// relative wind, given the velocity already rotated into body axes.
    ///
    /// atan2 of cross-flow against axial speed rather than acos of a normalised
    /// dot product: acos loses precision exactly where a booster spends its time,
    /// near alpha = 0, because its derivative is unbounded as the argument
    /// approaches 1.
    /// </summary>
    public static Dual AngleOfAttack(Dual vbx, Dual vby, Dual vbz)
    {
        // Floor the cross-flow term so the Sqrt derivative stays finite when the
        // velocity is exactly on the body axis. At 1e-9 m/s the floor is far below
        // anything physical and alpha is 0 to every digit that matters.
        Dual cross = Dual.Sqrt(vbx * vbx + vby * vby + 1e-18);
        return Dual.Atan2(cross, vbz);
    }
}
