namespace Navbox.Numerics;

/// <summary>
/// Forward-mode automatic differentiation number: a value paired with its
/// derivative with respect to one seeded input.
///
/// Scalar (single-partial) rather than vector mode is deliberate. The dynamics
/// are tiny — order a hundred flops — so sweeping the function once per input
/// column (18 times: 14 states + 4 controls) costs less than the allocation and
/// indirection a double[] of partials per intermediate would incur. This struct
/// is two doubles, lives entirely on the stack, and allocates nothing, so a full
/// Jacobian sweep across every node produces zero GC pressure.
///
/// Seeding: set D = 1 on the input being differentiated and D = 0 on all others;
/// each output's D is then that column of the Jacobian. The value part is
/// independent of the seed, so f itself comes free from any sweep.
///
/// Only the operations the dynamics actually use are defined. Adding an
/// operation means adding its derivative rule — deliberately explicit, so an
/// unsupported function is a compile error rather than a silently wrong slope.
/// </summary>
public readonly struct Dual
{
    public readonly double V;   // value
    public readonly double D;   // d(value)/d(seeded input)

    public Dual(double v, double d = 0.0)
    {
        V = v;
        D = d;
    }

    /// <summary>A constant: value with zero derivative.</summary>
    public static implicit operator Dual(double v) => new(v, 0.0);

    /// <summary>The seeded (independent) variable: derivative 1 with respect to itself.</summary>
    public static Dual Seed(double v) => new(v, 1.0);

    public static Dual operator +(Dual a, Dual b) => new(a.V + b.V, a.D + b.D);
    public static Dual operator -(Dual a, Dual b) => new(a.V - b.V, a.D - b.D);
    public static Dual operator -(Dual a) => new(-a.V, -a.D);

    // product rule
    public static Dual operator *(Dual a, Dual b) => new(a.V * b.V, a.D * b.V + a.V * b.D);

    // quotient rule
    public static Dual operator /(Dual a, Dual b)
    {
        double inv = 1.0 / b.V;
        return new Dual(a.V * inv, (a.D * b.V - a.V * b.D) * inv * inv);
    }

    public static Dual Sqrt(Dual a)
    {
        double s = Math.Sqrt(a.V);
        return new Dual(s, a.D / (2.0 * s));
    }

    /// <summary>
    /// Exponential. Added for the atmosphere: density is rho0 * exp(-h/H), and a
    /// solver that plans through the air needs d(rho)/d(altitude) to come out of
    /// the same sweep as every other slope rather than being special-cased.
    ///
    ///   d/dt exp(x) = exp(x) * x'
    /// </summary>
    public static Dual Exp(Dual a)
    {
        double e = Math.Exp(a.V);
        return new Dual(e, e * a.D);
    }

    /// <summary>
    /// Four-quadrant arctangent. Added for angle of attack, which is the angle
    /// between the body axis and the relative wind: atan2 of the cross-flow speed
    /// against the axial speed is well conditioned everywhere except at zero
    /// speed, where acos of a normalised dot product would already have lost
    /// precision near alpha = 0.
    ///
    ///   d/dt atan2(y, x) = (x*y' - y*x') / (x^2 + y^2)
    /// </summary>
    public static Dual Atan2(Dual y, Dual x)
    {
        double denom = x.V * x.V + y.V * y.V;
        return new Dual(Math.Atan2(y.V, x.V), (x.V * y.D - y.V * x.D) / denom);
    }

    public override string ToString() => $"{V:G6} (d={D:G6})";
}
