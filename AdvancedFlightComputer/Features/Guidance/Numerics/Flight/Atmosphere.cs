using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// The air a solver flies through: density and pressure by altitude, with slopes, a top above which both are exactly zero, and a speed of sound for the Mach axis of the drag tables.
///
/// Altitude is geometric, above the MEAN radius, and is clamped to zero below it, as KSA's PhysicsEnvironment.RecomputePositionalValues does: a site below mean radius sees sea level air. The solvers must not reference the game, so the mod builds a <see cref="SampledAtmosphere"/> from the game's own pressure and density functions, and the offline checks use the analytic <see cref="ExponentialAtmosphere"/>.
///
/// The game has no Mach number in its aerodynamics. The drag surrogate is parameterised on Mach, so a Mach axis needs a speed of sound even while the table is flat along it. <see cref="SpeedOfSound"/> is one reference value per atmosphere, not a function of altitude, so that the Mach axis stays a fixed scaling of airspeed.
/// </summary>
public abstract class AtmosphereModel
{
    /// <summary>Density floor that defines the top of the atmosphere. KSA's PhysicalAtmosphere.MIN_DENSITY.</summary>
    public const double MinDensity = 1e-9;

    /// <summary>Pressure floor that defines the top of the atmosphere. KSA's PhysicalAtmosphere.MIN_PRESSURE.</summary>
    public const double MinPressure = 1e-4;

    /// <summary>Sea level density, kg/m^3.</summary>
    public abstract double SeaLevelDensity { get; }

    /// <summary>Sea level pressure, Pa.</summary>
    public abstract double SeaLevelPressure { get; }

    /// <summary>
    /// Altitude above mean radius at which the game stops modelling air at all, m. Above it both density and pressure are exactly zero, not merely small.
    /// </summary>
    public abstract double TopAltitude { get; }

    /// <summary>Speed of sound that scales airspeed onto the Mach axis of the drag tables, m/s.</summary>
    public abstract double SpeedOfSound { get; }

    /// <summary>Density at a geometric altitude above mean radius, kg/m^3.</summary>
    public abstract double Density(double altitude);

    /// <summary>Pressure at a geometric altitude above mean radius, Pa.</summary>
    public abstract double Pressure(double altitude);

    /// <summary>
    /// Density as a <see cref="Dual"/>, so dynamics code can compute d(rho)/d(altitude) in the same sweep as every other slope.
    ///
    /// Every model has two kinks that solver dynamics must accept: at h = 0 the clamp flattens rho below mean radius, and at h = Top rho steps to exactly zero. The step at the top is harmless, because rho there is about 1e-9 kg/m^3 by construction.
    /// </summary>
    public abstract Dual Density(Dual altitude);

    /// <summary>Pressure as a <see cref="Dual"/>, with the same two kinks as <see cref="Density(Dual)"/>. The convex ascent reads it for the back pressure of the engines.</summary>
    public abstract Dual Pressure(Dual altitude);

    /// <summary>Free-stream Mach number for a speed in m/s.</summary>
    public double Mach(double speed) => speed / SpeedOfSound;

    /// <summary>Free-stream Mach number, differentiably.</summary>
    public Dual Mach(Dual speed) => speed / SpeedOfSound;

    /// <summary>Dynamic pressure, 1/2 rho V^2, at an altitude and speed. Convenience for readouts; the dynamics compose the pieces themselves.</summary>
    public double DynamicPressure(double altitude, double speed)
        => 0.5 * Density(altitude) * speed * speed;

    /// <summary>
    /// The local density scale height at an altitude, m: rho / (-d rho / dh), from the slope of <see cref="Density(Dual)"/>. A layered atmosphere has no single scale height, so a caller that needs one asks for it at the altitude it cares about. Infinite where the density does not fall, which is below mean radius and above the top.
    /// </summary>
    public double ScaleHeightAt(double altitude)
    {
        Dual rho = Density(Dual.Seed(altitude));
        return rho.D < 0.0 && rho.V > 0.0 ? -rho.V / rho.D : double.PositiveInfinity;
    }

    /// <summary>
    /// The scale height just above sea level, m. Density is clamped at sea level, so its slope there is zero and <see cref="ScaleHeightAt"/> at exactly zero is infinite.
    /// </summary>
    public double SeaLevelScaleHeight => ScaleHeightAt(Math.BitIncrement(0.0));

    /// <summary>
    /// The lowest altitude at which the density has fallen to the given value, m, by bisection on the monotonic density. Zero when sea level is already thinner than that, and the top when the value is below the density floor.
    /// </summary>
    public virtual double AltitudeAtDensity(double density)
    {
        if (!(density < SeaLevelDensity))
            return 0.0;
        double lo = 0.0, hi = TopAltitude;
        if (!(Density(Math.BitDecrement(hi)) < density))
            return hi;
        for (int i = 0; i < 64 && hi - lo > 1e-3; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (Density(mid) > density)
                lo = mid;
            else
                hi = mid;
        }
        return hi;
    }
}

/// <summary>
/// A single-layer isothermal exponential atmosphere:
///
///   rho(h) = rho0 * exp(-h / H)
///   P(h)   = P0   * exp(-h / H)
///
/// cut off where the thinner of the two hits its floor. This is not the game's air, which is layered, but an analytic model with closed-form slopes. The offline checks and the reference problems use it, because their expected numbers come from the same formula.
///
/// An isothermal atmosphere has constant P/rho, so a = sqrt(gamma * P0 / rho0) is the same at every altitude. Gamma is a constructor argument.
/// </summary>
public sealed class ExponentialAtmosphere : AtmosphereModel
{
    private readonly double _seaLevelDensity;
    private readonly double _seaLevelPressure;
    private readonly double _topAltitude;
    private readonly double _speedOfSound;

    /// <summary>Sea level density, kg/m^3.</summary>
    public override double SeaLevelDensity => _seaLevelDensity;

    /// <summary>Sea level pressure, Pa.</summary>
    public override double SeaLevelPressure => _seaLevelPressure;

    /// <summary>Scale height, m. One number for the whole atmosphere, because it is isothermal.</summary>
    public double ScaleHeight { get; }

    /// <summary>Ratio of specific heats, used only for the speed of sound.</summary>
    public double Gamma { get; }

    /// <summary>The higher of the two floor crossings, so neither quantity stays non-zero above it.</summary>
    public override double TopAltitude => _topAltitude;

    /// <summary>Speed of sound, m/s, the same at every altitude because the model is isothermal.</summary>
    public override double SpeedOfSound => _speedOfSound;

    /// <summary>A standard reference: 1.225 kg/m^3, 1 atm, 8 km scale height.</summary>
    public static ExponentialAtmosphere Earth => new(1.225, 101325.0, 8000.0);

    /// <param name="seaLevelDensity">rho0, kg/m^3.</param>
    /// <param name="seaLevelPressure">P0, Pa.</param>
    /// <param name="scaleHeight">H, metres.</param>
    /// <param name="gamma">Ratio of specific heats for the speed of sound. The default is diatomic air.</param>
    public ExponentialAtmosphere(double seaLevelDensity, double seaLevelPressure,
                                 double scaleHeight, double gamma = 1.4)
    {
        if (!(seaLevelDensity > 0.0) || !double.IsFinite(seaLevelDensity))
            throw new ArgumentOutOfRangeException(nameof(seaLevelDensity), seaLevelDensity,
                "Sea-level density must be finite and positive.");
        if (!(seaLevelPressure > 0.0) || !double.IsFinite(seaLevelPressure))
            throw new ArgumentOutOfRangeException(nameof(seaLevelPressure), seaLevelPressure,
                "Sea-level pressure must be finite and positive.");
        if (!(scaleHeight > 0.0) || !double.IsFinite(scaleHeight))
            throw new ArgumentOutOfRangeException(nameof(scaleHeight), scaleHeight,
                "Scale height must be finite and positive.");
        if (!(gamma > 0.0) || !double.IsFinite(gamma))
            throw new ArgumentOutOfRangeException(nameof(gamma), gamma,
                "Gamma must be finite and positive.");

        _seaLevelDensity = seaLevelDensity;
        _seaLevelPressure = seaLevelPressure;
        ScaleHeight = scaleHeight;
        Gamma = gamma;

        _topAltitude = Math.Max(-scaleHeight * Math.Log(MinDensity / seaLevelDensity),
                                -scaleHeight * Math.Log(MinPressure / seaLevelPressure));

        _speedOfSound = Math.Sqrt(gamma * seaLevelPressure / seaLevelDensity);
    }

    public override double Density(double altitude)
    {
        if (altitude >= _topAltitude)
            return 0.0;
        return _seaLevelDensity * Math.Exp(-Math.Max(altitude, 0.0) / ScaleHeight);
    }

    public override double Pressure(double altitude)
    {
        if (altitude >= _topAltitude)
            return 0.0;
        return _seaLevelPressure * Math.Exp(-Math.Max(altitude, 0.0) / ScaleHeight);
    }

    public override Dual Density(Dual altitude)
    {
        if (altitude.V >= _topAltitude)
            return new Dual(0.0, 0.0);
        if (altitude.V <= 0.0)
            return new Dual(_seaLevelDensity, 0.0);

        // Build the exponent by division so this overload uses the same arithmetic as Density(double) and agrees with it in the last bit.
        Dual e = Dual.Exp(new Dual(-altitude.V / ScaleHeight, -altitude.D / ScaleHeight));
        return _seaLevelDensity * e;
    }

    public override Dual Pressure(Dual altitude)
    {
        if (altitude.V >= _topAltitude)
            return new Dual(0.0, 0.0);
        if (altitude.V <= 0.0)
            return new Dual(_seaLevelPressure, 0.0);

        Dual e = Dual.Exp(new Dual(-altitude.V / ScaleHeight, -altitude.D / ScaleHeight));
        return _seaLevelPressure * e;
    }

    /// <summary>Closed form, H * ln(rho0 / rho), capped at the top.</summary>
    public override double AltitudeAtDensity(double density)
    {
        if (!(density < _seaLevelDensity))
            return 0.0;
        if (!(density > 0.0))
            return _topAltitude;
        return Math.Min(ScaleHeight * Math.Log(_seaLevelDensity / density), _topAltitude);
    }

    public override string ToString()
        => $"rho0={SeaLevelDensity:G4} kg/m^3, P0={SeaLevelPressure:G6} Pa, "
         + $"H={ScaleHeight / 1000.0:F2} km, top={TopAltitude / 1000.0:F1} km, "
         + $"a={SpeedOfSound:F1} m/s";
}
