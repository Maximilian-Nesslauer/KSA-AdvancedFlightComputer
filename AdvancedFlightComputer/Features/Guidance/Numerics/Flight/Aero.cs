using PoweredGuidance.Numerics;

namespace PoweredGuidance.Flight;

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
/// ALPHA IS RETROGRADE-FIRST throughout: alpha = 0 is flying tail-first with the
/// engine into the wind, alpha = 180 is nose-first. See
/// <see cref="AngleOfAttack"/> for why, and note that it makes the table's
/// interesting end its FIRST row rather than its last.
///
/// The numbers below are a plausible slender-booster set, NOT measured data. They
/// come from a closed-form generator, written down here so the shape can be judged
/// rather than trusted:
///
///   Cd = [Cd_tail*w + Cd_nose*(1-w)] * f(M)  +  1.55*sin^2(alpha) * (1 + f(M))/2
///   w  = (1 + cos alpha) / 2          1 at alpha = 0, 0 at alpha = 180
///   f  = transonic rise, 1 subsonic, peaking 2.47 at M = 1.25, settling to 1.32
///
/// with Cd_tail = 1.05 and Cd_nose = 0.315 - blunt base into the wind costs more
/// than the nose does, and broadside costs most of all. A flight build should not
/// be using any of this: the mod samples the game's own aerodynamics onto a grid
/// and calls the data-driven constructor. This exists so headless checks have
/// something with the right shape, and nothing outside this file depends on the
/// particular values.
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
        0.0, 2.0, 5.0, 8.0, 12.0, 16.0, 20.0, 25.0, 30.0, 40.0,
        50.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0, 150.0, 165.0, 180.0
    };

    private static readonly double[] DefaultCdTable =
    {
        // M = 0.00
        1.0500, 1.0517, 1.0604, 1.0765, 1.1090, 1.1535, 1.2092, 1.2924, 1.3883, 1.6045, 1.8283, 2.0288, 2.2238, 2.2325, 2.0336, 1.6613, 1.1976, 0.7517, 0.4314, 0.3150,
        // M = 0.20
        1.0501, 1.0517, 1.0604, 1.0765, 1.1090, 1.1536, 1.2092, 1.2925, 1.3883, 1.6045, 1.8284, 2.0288, 2.2239, 2.2326, 2.0336, 1.6613, 1.1977, 0.7518, 0.4314, 0.3150,
        // M = 0.40
        1.0510, 1.0527, 1.0614, 1.0775, 1.1100, 1.1546, 1.2102, 1.2935, 1.3894, 1.6057, 1.8296, 2.0302, 2.2253, 2.2339, 2.0348, 1.6623, 1.1984, 0.7523, 0.4317, 0.3153,
        // M = 0.60
        1.0647, 1.0663, 1.0751, 1.0913, 1.1240, 1.1688, 1.2248, 1.3085, 1.4050, 1.6224, 1.8475, 2.0490, 2.2448, 2.2529, 2.0519, 1.6763, 1.2090, 0.7595, 0.4367, 0.3194,
        // M = 0.70
        1.1043, 1.1060, 1.1149, 1.1313, 1.1645, 1.2101, 1.2669, 1.3520, 1.4500, 1.6708, 1.8993, 2.1035, 2.3013, 2.3078, 2.1013, 1.7171, 1.2395, 0.7806, 0.4510, 0.3313,
        // M = 0.80
        1.2377, 1.2395, 1.2489, 1.2662, 1.3013, 1.3493, 1.4091, 1.4987, 1.6018, 1.8341, 2.0739, 2.2876, 2.4921, 2.4931, 2.2679, 1.8544, 1.3425, 0.8515, 0.4992, 0.3713,
        // M = 0.85
        1.3786, 1.3804, 1.3903, 1.4086, 1.4455, 1.4961, 1.5591, 1.6535, 1.7620, 2.0063, 2.2581, 2.4817, 2.6934, 2.6886, 2.4436, 1.9992, 1.4511, 0.9263, 0.5501, 0.4136,
        // M = 0.90
        1.5843, 1.5863, 1.5970, 1.6166, 1.6562, 1.7106, 1.7783, 1.8796, 1.9961, 2.2579, 2.5272, 2.7653, 2.9874, 2.9742, 2.7004, 2.2108, 1.6099, 1.0357, 0.6244, 0.4753,
        // M = 0.95
        1.8375, 1.8397, 1.8512, 1.8725, 1.9156, 1.9745, 2.0480, 2.1579, 2.2842, 2.5676, 2.8584, 3.1144, 3.3493, 3.3256, 3.0164, 2.4713, 1.8052, 1.1702, 0.7159, 0.5513,
        // M = 1.00
        2.0907, 2.0931, 2.1055, 2.1285, 2.1749, 2.2385, 2.3177, 2.4362, 2.5722, 2.8773, 3.1896, 3.4634, 3.7112, 3.6771, 3.3324, 2.7317, 2.0006, 1.3048, 0.8074, 0.6272,
        // M = 1.05
        2.2964, 2.2990, 2.3121, 2.3365, 2.3856, 2.4530, 2.5369, 2.6623, 2.8063, 3.1289, 3.4588, 3.7471, 4.0052, 3.9627, 3.5892, 2.9433, 2.1593, 1.4141, 0.8818, 0.6889,
        // M = 1.10
        2.4373, 2.4399, 2.4536, 2.4788, 2.5299, 2.5998, 2.6869, 2.8171, 2.9665, 3.3012, 3.6430, 3.9412, 4.2065, 4.1581, 3.7649, 3.0881, 2.2680, 1.4889, 0.9327, 0.7312,
        // M = 1.15
        2.5227, 2.5253, 2.5393, 2.5652, 2.6174, 2.6888, 2.7779, 2.9109, 3.0636, 3.4056, 3.7547, 4.0589, 4.3286, 4.2767, 3.8715, 3.1760, 2.3339, 1.5343, 0.9635, 0.7568,
        // M = 1.20
        2.5707, 2.5735, 2.5876, 2.6138, 2.6666, 2.7389, 2.8291, 2.9638, 3.1183, 3.4644, 3.8176, 4.1252, 4.3973, 4.3434, 3.9315, 3.2254, 2.3710, 1.5599, 0.9809, 0.7712,
        // M = 1.30
        2.5958, 2.5986, 2.6128, 2.6391, 2.6923, 2.7651, 2.8558, 2.9913, 3.1469, 3.4951, 3.8504, 4.1598, 4.4332, 4.3783, 3.9629, 3.2513, 2.3903, 1.5732, 0.9900, 0.7788,
        // M = 1.40
        2.5891, 2.5919, 2.6061, 2.6324, 2.6855, 2.7581, 2.8487, 2.9840, 3.1393, 3.4869, 3.8417, 4.1506, 4.4236, 4.3690, 3.9545, 3.2444, 2.3852, 1.5697, 0.9876, 0.7767,
        // M = 1.60
        2.5563, 2.5590, 2.5731, 2.5991, 2.6518, 2.7239, 2.8137, 2.9479, 3.1019, 3.4467, 3.7987, 4.1053, 4.3766, 4.3234, 3.9135, 3.2106, 2.3598, 1.5522, 0.9757, 0.7669,
        // M = 1.80
        2.4994, 2.5020, 2.5159, 2.5416, 2.5935, 2.6645, 2.7531, 2.8853, 3.0371, 3.3771, 3.7242, 4.0268, 4.2953, 4.2443, 3.8424, 3.1520, 2.3159, 1.5219, 0.9551, 0.7498,
        // M = 2.00
        2.4220, 2.4246, 2.4382, 2.4634, 2.5142, 2.5839, 2.6706, 2.8003, 2.9491, 3.2825, 3.6230, 3.9201, 4.1847, 4.1370, 3.7459, 3.0724, 2.2562, 1.4808, 0.9271, 0.7266,
        // M = 2.50
        2.1713, 2.1738, 2.1865, 2.2100, 2.2575, 2.3225, 2.4036, 2.5248, 2.6639, 2.9759, 3.2951, 3.5746, 3.8264, 3.7890, 3.4330, 2.8146, 2.0628, 1.3476, 0.8366, 0.6514,
        // M = 3.00
        1.9043, 1.9066, 1.9183, 1.9401, 1.9840, 2.0442, 2.1192, 2.2313, 2.3602, 2.6493, 2.9458, 3.2065, 3.4448, 3.4184, 3.0998, 2.5400, 1.8568, 1.2057, 0.7401, 0.5713,
        // M = 3.50
        1.6838, 1.6859, 1.6969, 1.7172, 1.7582, 1.8143, 1.8843, 1.9890, 2.1093, 2.3797, 2.6574, 2.9026, 3.1297, 3.1123, 2.8246, 2.3132, 1.6867, 1.0886, 0.6604, 0.5052,
        // M = 4.00
        1.5350, 1.5370, 1.5475, 1.5667, 1.6058, 1.6592, 1.7258, 1.8255, 1.9400, 2.1977, 2.4628, 2.6974, 2.9170, 2.9057, 2.6389, 2.1601, 1.5719, 1.0095, 0.6066, 0.4605,
        // M = 5.00
        1.4106, 1.4125, 1.4225, 1.4410, 1.4783, 1.5295, 1.5933, 1.6887, 1.7985, 2.0455, 2.3000, 2.5259, 2.7392, 2.7331, 2.4836, 2.0322, 1.4759, 0.9434, 0.5617, 0.4232,
    };

    private readonly CubicBSplineNd _cd;
    private readonly double[] _machGrid;

    public double MachMin => _machGrid[0];
    public double MachMax => _machGrid[^1];
    public double AlphaMaxRad { get; }

    /// <summary>
    /// The default Mach breakpoints, for a caller building its own table.
    ///
    /// Worth reusing rather than inventing: the spacing is deliberately fine through
    /// 0.85 - 1.30 so a cubic fit does not overshoot a transonic drag rise, and a
    /// caller sampling a source that is currently FLAT in Mach - which KSA's is,
    /// since the game models no compressibility at all - still wants the grid shaped
    /// for the day it stops being flat. A copy, so the table's own grid cannot be
    /// mutated from outside.
    /// </summary>
    public static double[] DefaultMachBreakpoints => (double[])DefaultMachGrid.Clone();

    /// <summary>
    /// The default angle-of-attack breakpoints in DEGREES, retrograde-first, spanning
    /// the full 0 - 180. Sampled finely near 0 because that is where a boosting-back
    /// or entering booster actually sits, and coarsely through the broadside region
    /// it only passes through.
    /// </summary>
    public static double[] DefaultAlphaBreakpointsDeg => (double[])DefaultAlphaGridDeg.Clone();

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
        // UNSEEDED FAST PATH. If neither input carries a derivative then the chain
        // rule below produces exactly zero whatever the gradient is, so computing the
        // gradient is pure waste - and it is not cheap: EvaluateWithGradient costs
        // roughly 2.2x Evaluate (see --aero). This is not an approximation; the two
        // branches return bit-identical results.
        //
        // It matters because the same integrator serves both jobs. A Jacobian sweep
        // seeds one input and needs the slope; an impact prediction for the overlay
        // seeds nothing and needs only the value, and that path calls this four times
        // per RK4 step for hundreds of steps, several times a second.
        if (mach.D == 0.0 && alpha.D == 0.0)
            return new Dual(Cd(mach.V, alpha.V), 0.0);

        double value = Cd(mach.V, alpha.V, out double dM, out double dA);
        return new Dual(value, dM * mach.D + dA * alpha.D);
    }

    /// <summary>
    /// Angle of attack, measured RETROGRADE-FIRST: the angle between the body
    /// -z axis - the tail, where the engine is - and the relative wind, given the
    /// velocity already rotated into body axes.
    ///
    ///   alpha = 0    flying tail-first, engine into the wind. Boostback and entry.
    ///   alpha = 90   broadside.
    ///   alpha = 180  flying nose-first. Ascent.
    ///
    /// THE CONVENTION IS THE RETROGRADE ONE ON PURPOSE, and it is worth the sign
    /// that carries it. Every phase this surrogate exists to serve - boostback, the
    /// entry burn, the descent - is flown with the engine pointed into the airflow,
    /// so alpha = 0 is the attitude the vehicle actually holds and the small angles
    /// are the ones it actually flies. Under the prograde convention all of that
    /// happens at alpha near 180, where a table has to be sampled finely at its far
    /// edge and every deviation reads as a number near pi rather than near zero.
    ///
    /// There is exactly one convention. If a prograde angle is ever wanted, it is
    /// pi minus this, computed at the call site and named there - a second function
    /// here would be a coin flip about which one a caller got.
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
        // Negated: alpha is measured from -z, so velocity along -z gives alpha = 0.
        return Dual.Atan2(cross, -vbz);
    }
}
