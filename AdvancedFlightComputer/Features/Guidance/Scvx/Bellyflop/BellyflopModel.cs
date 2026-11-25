namespace AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

/// <summary>
/// The belly-first air force on the craft, in its belly axes: nose, span and belly normal (out of the belly, the way the craft falls at zero angles), each a coefficient of q S. <paramref name="alphaRad"/> is the airflow's angle towards the nose, atan2(v.nose, v.belly); <paramref name="betaRad"/> towards the span, atan2(v.span, v.belly): the belly sweep's own table angles (KsaAeroSweep BellyBodyAero).
/// </summary>
public delegate void BellyCoefficients(double mach, double alphaRad, double betaRad, out double cNose, out double cSpan, out double cBelly);

/// <summary>
/// Bellyflop's physics: a point mass falling belly-first through the air of a rotating body, in the site frame (origin at the site, z up, rotating with the body; the body's centre at (0, 0, -R)).
///
/// THE ATTITUDE, from the controls u = (alpha, beta, psi). The reference frame at a state is built from its own velocity: belly normal along v (the belly leads), reference nose the part of (bearing + up) across v, span belly x nose. Yaw psi turns the nose about v. The body is then the reference turned by the minimal rotation that puts the airflow at (alpha, beta) in it: the airflow in body axes is (tan alpha, tan beta, 1) normalised, so the tables are read at exactly the angles the solver chose. The reference nose stands up the vertical plane through the bearing in shallow flight and lies along the bearing falling vertically; it is singular only for velocity along bearing + up, climbing at 45 deg.
///
///   r' = v ;  v' = g(r) - 2 w x v - w x (w x (r - c)) + F_air / m
/// </summary>
public sealed record BellyflopModel
{
    public required double Mu { get; init; }
    public required double BodyRadius { get; init; }
    /// <summary>The body's rotation in the site frame, rad/s.</summary>
    public double OmegaX { get; init; }
    public double OmegaY { get; init; }
    public double OmegaZ { get; init; }
    /// <summary>Air density against altitude above the site's datum (the body's mean radius), kg/m^3.</summary>
    public required Func<double, double> Density { get; init; }
    /// <summary>The speed of sound Mach is reckoned against, m/s (the sweep's: sea level).</summary>
    public required double SpeedOfSound { get; init; }
    public required double Mass { get; init; }
    public required double ReferenceArea { get; init; }
    public required BellyCoefficients Coefficients { get; init; }
    /// <summary>The horizontal bearing the yaw is referenced to (site frame, unit, z = 0): to the site when bellyflop took over.</summary>
    public required double BearingX { get; init; }
    public required double BearingY { get; init; }
    /// <summary>Corrections from flight: density and air force scales.</summary>
    public double DensityScale { get; init; } = 1.0;
    public double ForceScale { get; init; } = 1.0;

    public const int NX = 6, NU = 3;
    public const int IA = 0, IB = 1, IPsi = 2;
    const double MinSpeed = 1.0;

    public double Altitude(ReadOnlySpan<double> x)
    {
        double px = x[0], py = x[1], pz = x[2] + BodyRadius;
        return Math.Sqrt(px * px + py * py + pz * pz) - BodyRadius;
    }

    public double Mach(ReadOnlySpan<double> x) => Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]) / SpeedOfSound;

    /// <summary>The state's rate of change under the controls.</summary>
    public void Derivative(ReadOnlySpan<double> x, double alpha, double beta, double psi, Span<double> xdot)
    {
        double px = x[0], py = x[1], pz = x[2] + BodyRadius;
        double rr = Math.Sqrt(px * px + py * py + pz * pz);
        double gk = -Mu / (rr * rr * rr);
        double ax = gk * px, ay = gk * py, az = gk * pz;
        double vx = x[3], vy = x[4], vz = x[5];
        // Coriolis and centrifugal, about the body's centre.
        double wx = OmegaX, wy = OmegaY, wz = OmegaZ;
        ax -= 2 * (wy * vz - wz * vy); ay -= 2 * (wz * vx - wx * vz); az -= 2 * (wx * vy - wy * vx);
        double cx = wy * pz - wz * py, cy = wz * px - wx * pz, cz = wx * py - wy * px;
        ax -= wy * cz - wz * cy; ay -= wz * cx - wx * cz; az -= wx * cy - wy * cx;
        if (AirForce(x, alpha, beta, psi, out double fx, out double fy, out double fz))
        {
            ax += fx / Mass; ay += fy / Mass; az += fz / Mass;
        }
        xdot[0] = vx; xdot[1] = vy; xdot[2] = vz;
        xdot[3] = ax; xdot[4] = ay; xdot[5] = az;
    }

    /// <summary>The air force, N, site frame. False with too little speed to read an airflow.</summary>
    public bool AirForce(ReadOnlySpan<double> x, double alpha, double beta, double psi, out double fx, out double fy, out double fz)
    {
        fx = fy = fz = 0;
        if (!Axes(x, alpha, beta, psi, out var n, out var s, out var b, out double speed))
            return false;
        double rho = Density(Altitude(x)) * DensityScale;
        double q = 0.5 * rho * speed * speed;
        Coefficients(speed / SpeedOfSound, alpha, beta, out double cn, out double cs, out double cb);
        double k = q * ReferenceArea * ForceScale;
        fx = k * (cn * n.X + cs * s.X + cb * b.X);
        fy = k * (cn * n.Y + cs * s.Y + cb * b.Y);
        fz = k * (cn * n.Z + cs * s.Z + cb * b.Z);
        return true;
    }

    /// <summary>The craft's nose, span and belly normal in the site frame under the controls at a state. False with too little speed.</summary>
    public bool Axes(ReadOnlySpan<double> x, double alpha, double beta, double psi, out V3 nose, out V3 span, out V3 belly, out double speed)
    {
        nose = span = belly = default;
        var v = new V3(x[3], x[4], x[5]);
        speed = v.Length;
        if (!(speed > MinSpeed))
            return false;
        V3 f = v / speed;
        // The reference nose: (bearing + up) across the velocity.
        var up = new V3(x[0], x[1], x[2] + BodyRadius).Unit;
        V3 lean = new V3(BearingX, BearingY, 0) + up;
        V3 n0 = (lean - f * V3.Dot(lean, f)).Unit;
        if (n0.Length < 0.5)
            n0 = (up - f * V3.Dot(up, f)).Unit;
        // Yaw about the velocity.
        double cpsi = Math.Cos(psi), spsi = Math.Sin(psi);
        V3 n1 = n0 * cpsi + V3.Cross(f, n0) * spsi;
        V3 s1 = V3.Cross(f, n1);
        // The airflow in body axes, and the minimal rotation from it to the belly normal: the body axes in the reference frame's coordinates.
        double ta = Math.Tan(alpha), tb = Math.Tan(beta);
        double inv = 1.0 / Math.Sqrt(ta * ta + tb * tb + 1);
        var u = new V3(ta * inv, tb * inv, inv);
        V3 en = new(1, 0, 0), es = new(0, 1, 0), eb = new(0, 0, 1);
        double sinAng = Math.Sqrt(u.X * u.X + u.Y * u.Y);
        if (sinAng > 1e-12)
        {
            // Axis u x z = (u.Y, -u.X, 0) / sin; angle acos(u.Z): Rodrigues on each basis vector.
            var k = new V3(u.Y / sinAng, -u.X / sinAng, 0);
            double cosAng = u.Z;
            en = Rotate(en, k, cosAng, sinAng);
            es = Rotate(es, k, cosAng, sinAng);
            eb = Rotate(eb, k, cosAng, sinAng);
        }
        nose = n1 * en.X + s1 * en.Y + f * en.Z;
        span = n1 * es.X + s1 * es.Y + f * es.Z;
        belly = n1 * eb.X + s1 * eb.Y + f * eb.Z;
        return true;
    }

    static V3 Rotate(V3 v, V3 k, double c, double s) => v * c + V3.Cross(k, v) * s + k * (V3.Dot(k, v) * (1 - c));

    /// <summary>The airflow's angles on the belly for a given belly-axes attitude at a state: the inverse of Axes, for the controller's attitude at take-over.</summary>
    public static (double Alpha, double Beta) AnglesOf(V3 velocity, V3 nose, V3 span, V3 belly)
    {
        double vb = V3.Dot(velocity, belly);
        return (Math.Atan2(V3.Dot(velocity, nose), vb), Math.Atan2(V3.Dot(velocity, span), vb));
    }
}

/// <summary>A small 3-vector for the model's geometry.</summary>
public readonly record struct V3(double X, double Y, double Z)
{
    public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
    public V3 Unit { get { double l = Length; return l > 0 ? this / l : this; } }
    public static V3 operator +(V3 a, V3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static V3 operator -(V3 a, V3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static V3 operator *(V3 a, double k) => new(a.X * k, a.Y * k, a.Z * k);
    public static V3 operator /(V3 a, double k) => new(a.X / k, a.Y / k, a.Z / k);
    public static double Dot(V3 a, V3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    public static V3 Cross(V3 a, V3 b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
}
