namespace AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// The aerodynamic force on a craft held at a fixed roll, as three body-axis coefficients over Mach, alpha and beta: C = F_body / (q S), with no averaging over roll.
///
/// WHY. A booster with grid fins or strakes is not axisymmetric: the force for a tilt in the fins' plane is not the force for a tilt across it, a tilt can push sideways as well as up, and the craft can push sideways at no angle at all. A table over the total angle of attack, averaged over roll, keeps none of that - on the 2026-11-18 booster it overstated the lift by about 1 m/s^2 at every angle. Flown at a fixed roll, the body axes are known for any pointing, so the force can be read for the actual tilt.
///
/// THE ANGLES, missile-style (skid to turn), retrograde-first as everywhere else: with u the craft's velocity through the air in body axes, unit, and the engine end leading (u_x &lt; 0),
///   alpha = atan2(u_z, -u_x)   the tilt in the body x-z plane (pitch, when the craft is held wings level: see KsaPointMassModel.RollRef)
///   beta  = atan2(u_y, -u_x)   the tilt in the body x-y plane (yaw)
/// Both are zero flying engine first, and neither has a singularity there, unlike a total angle and the direction of its tilt.
///
/// A NOSE-FIRST TABLE (<see cref="NoseFirst"/>, the convex ascent's, #216) takes its angles about the other end: with the nose leading (u_x &gt; 0), alpha = atan2(u_z, u_x) and beta = atan2(u_y, u_x), both zero flying nose first. The same mirror image, so the code that reads one reads the other through <see cref="Lead"/>.
///
/// The same class holds the moment table (KsaAeroSweep.Result.BodyMoment): three body-axis values per node again, there C_l, C_m, C_n = torque / (q S L) about the centre of mass the sweep saw.
///
/// One cubic B-spline fits all three coefficients over (Mach, alpha, beta), so the value and its exact gradient come in one pass; <see cref="Coefficients(Dual, Dual, Dual, out Dual, out Dual, out Dual)"/> feeds them to the AD sweep by the chain rule, as AeroTable does.
/// </summary>
public sealed class BodyAeroTable
{
    private readonly CubicBSplineNd _c;

    public double[] MachGrid { get; }
    public double[] AlphaGridDeg { get; }
    public double[] BetaGridDeg { get; }

    /// <summary>Largest |alpha| and |beta| sampled, radians. Past it the fit extends linearly; a caller far beyond it should use a whole-sphere table instead.</summary>
    public double AlphaMaxRad { get; }
    public double BetaMaxRad { get; }

    public double MachMin => MachGrid[0];
    public double MachMax => MachGrid[^1];

    /// <summary>The angles are taken about the nose (+x leading) rather than the engine end; see the type summary.</summary>
    public bool NoseFirst { get; }

    /// <summary>The leading end's sign along body x: +1 nose first, -1 engine first.</summary>
    public double Lead => NoseFirst ? 1.0 : -1.0;

    /// <summary>The coefficients as given, row-major over (Mach, alpha, beta): for a problem file to write out.</summary>
    public double[] Values { get; }

    /// <param name="coefficients">Row-major over (Mach, alpha, beta), Mach slowest, three values per node: C_x, C_y, C_z.</param>
    /// <param name="noseFirst">The angles are taken about the nose; see the type summary.</param>
    public BodyAeroTable(double[] machGrid, double[] alphaGridDeg, double[] betaGridDeg, double[] coefficients, bool noseFirst = false)
    {
        if (machGrid is null || alphaGridDeg is null || betaGridDeg is null || coefficients is null)
            throw new ArgumentNullException(nameof(coefficients));
        int n = machGrid.Length * alphaGridDeg.Length * betaGridDeg.Length;
        if (coefficients.Length != 3 * n)
            throw new ArgumentException($"Expected {3 * n} values, got {coefficients.Length}.", nameof(coefficients));
        for (int i = 0; i < coefficients.Length; i++)
            if (!double.IsFinite(coefficients[i]))
                throw new ArgumentException($"Coefficient {i} is not finite ({coefficients[i]}).", nameof(coefficients));

        NoseFirst = noseFirst;
        Values = (double[])coefficients.Clone();
        MachGrid = (double[])machGrid.Clone();
        AlphaGridDeg = (double[])alphaGridDeg.Clone();
        BetaGridDeg = (double[])betaGridDeg.Clone();
        double[] alphaRad = AlphaGridDeg.Select(a => a * Math.PI / 180.0).ToArray();
        double[] betaRad = BetaGridDeg.Select(b => b * Math.PI / 180.0).ToArray();
        AlphaMaxRad = Math.Max(Math.Abs(alphaRad[0]), Math.Abs(alphaRad[^1]));
        BetaMaxRad = Math.Max(Math.Abs(betaRad[0]), Math.Abs(betaRad[^1]));
        _c = CubicBSplineNd.Fit(new[] { MachGrid, alphaRad, betaRad }, (double[])coefficients.Clone(), 3, EdgeMode.Linear);
    }

    /// <summary>The table's alpha and beta (radians) of an air direction in body axes (the craft's velocity through the air, any length).</summary>
    public void Angles(double ux, double uy, double uz, out double alphaRad, out double betaRad)
    {
        double lead = Lead * ux;
        alphaRad = Math.Atan2(uz, lead);
        betaRad = Math.Atan2(uy, lead);
    }

    /// <summary>The three coefficients at a Mach number, alpha and beta (radians).</summary>
    public void Coefficients(double mach, double alphaRad, double betaRad, out double cx, out double cy, out double cz)
    {
        Span<double> point = stackalloc double[3] { mach, alphaRad, betaRad };
        Span<double> value = stackalloc double[3];
        _c.Evaluate(point, value);
        cx = value[0];
        cy = value[1];
        cz = value[2];
    }

    /// <summary>The same as <see cref="Dual"/>s: the spline's value and exact gradient, chained onto whatever the caller seeded.</summary>
    public void Coefficients(Dual mach, Dual alpha, Dual beta, out Dual cx, out Dual cy, out Dual cz)
    {
        if (mach.D == 0.0 && alpha.D == 0.0 && beta.D == 0.0)
        {
            Coefficients(mach.V, alpha.V, beta.V, out double x, out double y, out double z);
            cx = new Dual(x, 0.0);
            cy = new Dual(y, 0.0);
            cz = new Dual(z, 0.0);
            return;
        }
        Span<double> point = stackalloc double[3] { mach.V, alpha.V, beta.V };
        Span<double> value = stackalloc double[3];
        Span<double> grad = stackalloc double[9];
        _c.EvaluateWithGradient(point, value, grad);
        // grad[a * 3 + c] = d(value[c]) / d(point[a]).
        cx = new Dual(value[0], grad[0] * mach.D + grad[3] * alpha.D + grad[6] * beta.D);
        cy = new Dual(value[1], grad[1] * mach.D + grad[4] * alpha.D + grad[7] * beta.D);
        cz = new Dual(value[2], grad[2] * mach.D + grad[5] * alpha.D + grad[8] * beta.D);
    }

    /// <summary>
    /// The coefficients as a vector in the caller's own frame, for a craft held at a fixed roll: body x along the axis b (normalised here), y along w's part square to b (w is the wing direction the roll is held to, GuidanceWindow.ReturnRollRefCci), z = x x y; alpha and beta from the air direction h (unit: the craft's velocity through the air) in those axes; Mach held inside the table. The one place the held roll turns a pointing into body axes, so the entry model, the landing-point prediction and the glide steering cannot disagree about it.
    ///  False where the craft is not within <paramref name="maxTiltRad"/> of engine first, or the axes are degenerate (b along w, flat and square to its own ground track, which a returning booster never flies): the caller then uses its whole-sphere tables.
    /// </summary>
    public bool WorldCoefficients(Dual mach, Dual bx, Dual by, Dual bz, Dual hx, Dual hy, Dual hz,
                                  double wx, double wy, double wz, double maxTiltRad,
                                  out Dual cx, out Dual cy, out Dual cz)
    {
        cx = cy = cz = default;
        Dual bn = Dual.Sqrt(bx * bx + by * by + bz * bz);
        if (!(bn.V > 1e-9))
            return false;
        Dual xx = bx / bn, xy = by / bn, xz = bz / bn;
        Dual ux = hx * xx + hy * xy + hz * xz;
        if (!(Lead * ux.V > Math.Cos(maxTiltRad)))
            return false;
        // y: w less its part along x. Not x x w, whose direction flips as x passes w or -w: with w 45 deg below the horizon towards the site, an engine-first glide at 45 deg had no roll reference, and opposite ones either side (2026-11-20).
        Dual along = xx * wx + xy * wy + xz * wz;
        Dual yx = wx - along * xx, yy = wy - along * xy, yz = wz - along * xz;
        Dual yn = Dual.Sqrt(yx * yx + yy * yy + yz * yz);
        if (!(yn.V > 1e-6))
            return false;
        yx /= yn;
        yy /= yn;
        yz /= yn;
        Dual zx = xy * yz - xz * yy, zy = xz * yx - xx * yz, zz = xx * yy - xy * yx;
        Dual uy = hx * yx + hy * yy + hz * yz;
        Dual uz = hx * zx + hy * zy + hz * zz;
        Dual alpha = Dual.Atan2(uz, Lead * ux);
        Dual beta = Dual.Atan2(uy, Lead * ux);
        if (mach.V < MachMin) mach = new Dual(MachMin);
        else if (mach.V > MachMax) mach = new Dual(MachMax);
        Coefficients(mach, alpha, beta, out Dual c0, out Dual c1, out Dual c2);
        cx = c0 * xx + c1 * yx + c2 * zx;
        cy = c0 * xy + c1 * yy + c2 * zy;
        cz = c0 * xz + c1 * yz + c2 * zz;
        return true;
    }

    /// <summary>The body-axis air velocity direction, unit, for an alpha and beta: the inverse of the definition in the type summary, about the nose for a nose-first table.</summary>
    public static (double ux, double uy, double uz) AirDirection(double alphaRad, double betaRad, bool noseFirst = false)
    {
        double x = noseFirst ? 1.0 : -1.0, y = Math.Tan(betaRad), z = Math.Tan(alphaRad);
        double n = Math.Sqrt(x * x + y * y + z * z);
        return (x / n, y / n, z / n);
    }
}
