using PoweredGuidance.Numerics;

namespace PoweredGuidance.Flight;

/// <summary>
/// A single-layer isothermal exponential atmosphere, written to MIRROR the one the
/// game actually integrates against rather than to be a good atmosphere.
///
/// KSA's PhysicalAtmosphereReference is exactly this and nothing more:
///
///   rho(h) = rho0 * exp(-h / H)
///   P(h)   = P0   * exp(-h / H)
///   h      = max(|r_ccf| - MeanRadius, 0)          geometric, above MEAN radius
///
/// cut off hard at a boundary height where the thinner of the two hits its floor,
/// above which both are exactly zero. No layers, no lapse rate, no thermosphere, no
/// wind. Reproducing that here - rather than approximating it, or reaching into the
/// game for it - is what lets a solve run on a worker thread and still agree with
/// what the vehicle will experience.
///
/// WHY THIS LIVES HERE AND NOT IN THE MOD. The solver must not reference the game;
/// that is a project-level rule so it is a compile error rather than a convention
/// (see Scvx.Core's csproj). But the solver still has to know about the air. So the
/// mod reads the three numbers off KSA's AtmosphereReference and constructs one of
/// these; everything downstream sees plain doubles. The mirror is checkable in one
/// line against KSA's own GetAtmosphericDensityAtAltitude, which is what the
/// Boostback tab does on every resample.
///
/// SPEED OF SOUND IS DERIVED, NOT INVENTED. An isothermal atmosphere has constant
/// P/rho, so a = sqrt(gamma * P0 / rho0) is constant at every altitude - which is
/// self-consistent with the model rather than an extra assumption bolted onto it.
/// For KSA's Earth that is sqrt(1.4 * 101325 / 1.225) = 340.3 m/s, the right answer
/// for the right reason. Gamma is the one genuinely free parameter and it is a
/// constructor argument.
///
/// The game has no Mach number anywhere in its aerodynamics, so nothing in KSA
/// consumes this today. It exists because the surrogate is parameterised on Mach -
/// see <see cref="AeroTable"/> - and a Mach axis needs a speed of sound even while
/// the table is flat along it.
/// </summary>
public sealed class ExponentialAtmosphere
{
    /// <summary>Density floor that defines the top of the atmosphere. KSA's MIN_DENSITY.</summary>
    public const double MinDensity = 1e-9;

    /// <summary>Pressure floor that defines the top of the atmosphere. KSA's MIN_PRESSURE.</summary>
    public const double MinPressure = 1e-4;

    /// <summary>Sea-level density, kg/m^3.</summary>
    public double SeaLevelDensity { get; }

    /// <summary>Sea-level pressure, Pa.</summary>
    public double SeaLevelPressure { get; }

    /// <summary>Scale height, m. One number for the whole atmosphere - it is isothermal.</summary>
    public double ScaleHeight { get; }

    /// <summary>Ratio of specific heats, used only for the speed of sound.</summary>
    public double Gamma { get; }

    /// <summary>
    /// Altitude above mean radius at which the game stops modelling air at all, m.
    /// Above this both density and pressure are exactly zero, not merely small.
    ///
    /// KSA derives it as the higher of the two floors' crossings and then uses
    /// MeanRadius + this + 1 as the physics-bubble radius, so it is also the altitude
    /// below which a coasting vehicle is never put on rails.
    /// </summary>
    public double TopAltitude { get; }

    /// <summary>
    /// Speed of sound, m/s - the same at every altitude, because the model is
    /// isothermal. See the type summary: this is derived from P0/rho0, not assumed.
    /// </summary>
    public double SpeedOfSound { get; }

    /// <summary>KSA's Earth: 1.225 kg/m^3, 1 atm, 8 km scale height.</summary>
    public static ExponentialAtmosphere Earth => new(1.225, 101325.0, 8000.0);

    /// <param name="seaLevelDensity">rho0, kg/m^3.</param>
    /// <param name="seaLevelPressure">P0, Pa.</param>
    /// <param name="scaleHeight">H, metres.</param>
    /// <param name="gamma">Ratio of specific heats for the speed of sound. The
    /// default is diatomic air; the game models no composition at all, so this is
    /// the caller's choice and not something that can be read off a body.</param>
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

        SeaLevelDensity = seaLevelDensity;
        SeaLevelPressure = seaLevelPressure;
        ScaleHeight = scaleHeight;
        Gamma = gamma;

        // KSA's CalculateBoundaryHeight, verbatim: whichever of the two floors is
        // reached LAST sets the top, so neither quantity is ever left non-zero above
        // the boundary.
        TopAltitude = Math.Max(-scaleHeight * Math.Log(MinDensity / seaLevelDensity),
                               -scaleHeight * Math.Log(MinPressure / seaLevelPressure));

        SpeedOfSound = Math.Sqrt(gamma * seaLevelPressure / seaLevelDensity);
    }

    /// <summary>
    /// Density at a geometric altitude above MEAN radius, kg/m^3.
    ///
    /// Altitude is above the mean radius, not above the terrain: KSA computes it as
    /// |r_ccf| - MeanRadius and never consults the height map, so a vehicle sitting on
    /// a 5 km plateau is at 5 km of altitude for aerodynamic purposes even though it
    /// is on the ground. Feeding this a terrain-relative altitude would be wrong by
    /// however much the local terrain deviates.
    /// </summary>
    public double Density(double altitude)
    {
        if (altitude >= TopAltitude)
            return 0.0;
        return SeaLevelDensity * Math.Exp(-Math.Max(altitude, 0.0) / ScaleHeight);
    }

    /// <summary>Pressure at a geometric altitude above mean radius, Pa.</summary>
    public double Pressure(double altitude)
    {
        if (altitude >= TopAltitude)
            return 0.0;
        return SeaLevelPressure * Math.Exp(-Math.Max(altitude, 0.0) / ScaleHeight);
    }

    /// <summary>Free-stream Mach number for a speed in m/s.</summary>
    public double Mach(double speed) => speed / SpeedOfSound;

    /// <summary>Free-stream Mach number, differentiably.</summary>
    public Dual Mach(Dual speed) => speed / SpeedOfSound;

    /// <summary>
    /// Density as a <see cref="Dual"/>, so an atmosphere can be written inline in
    /// dynamics code and d(rho)/d(altitude) falls out of the same sweep as every
    /// other slope.
    ///
    /// TWO KINKS ARE INHERITED FROM THE GAME DELIBERATELY, and whoever wires this
    /// into a solver's dynamics should decide about them there rather than discover
    /// them:
    ///
    ///   at h = 0    the max() clamp flattens rho below mean radius, so the slope
    ///               steps from -rho0/H to 0. This is a real altitude - a landing
    ///               site below mean radius sits on it - and it is the sharper of
    ///               the two.
    ///   at h = Top  rho steps to exactly zero. Harmless in practice: rho there is
    ///               1e-9 kg/m^3 by construction, so the jump is nine orders below
    ///               anything that moves a booster.
    ///
    /// Smoothing either one would make this stop mirroring the game, which is the
    /// whole point of the type. If the trust region ever chatters on the h = 0 corner
    /// the fix is to drop the clamp HERE, in one place, with the divergence written
    /// down - not to paper over it at the call site.
    /// </summary>
    public Dual Density(Dual altitude)
    {
        if (altitude.V >= TopAltitude)
            return new Dual(0.0, 0.0);
        if (altitude.V <= 0.0)
            return new Dual(SeaLevelDensity, 0.0);

        // The exponent is built by DIVIDING, not by multiplying by the reciprocal,
        // which is what Dual's operator/ would do. That is a one-ULP difference and it
        // would be invisible - except that it makes this overload disagree with
        // Density(double) in the last bit, and "the differentiable path returns
        // something very slightly different" is exactly the kind of divergence that is
        // impossible to find later. Identical arithmetic, checked by --aero.
        Dual e = Dual.Exp(new Dual(-altitude.V / ScaleHeight, -altitude.D / ScaleHeight));
        return SeaLevelDensity * e;
    }

    /// <summary>
    /// Dynamic pressure, 1/2 rho V^2, at an altitude and speed. Convenience for
    /// readouts; the dynamics compose the pieces themselves.
    /// </summary>
    public double DynamicPressure(double altitude, double speed)
        => 0.5 * Density(altitude) * speed * speed;

    public override string ToString()
        => $"rho0={SeaLevelDensity:G4} kg/m^3, P0={SeaLevelPressure:G6} Pa, "
         + $"H={ScaleHeight / 1000.0:F2} km, top={TopAltitude / 1000.0:F1} km, "
         + $"a={SpeedOfSound:F1} m/s";
}
