namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// Everything the orbital entry planner knows about the body and the vehicle, in SI units.
///
/// The vehicle flies an angle-of-attack schedule against Mach, so its drag coefficient and lift-to-drag ratio are functions of Mach alone: <see cref="DragCoefficient"/> and <see cref="LiftToDrag"/> are read along that schedule. The planner never chooses the angle of attack.
///
/// The delegates must be pure and thread-safe, because the planner runs on a worker thread. The game adapter fills them from tables it built on the main thread.
///
/// <see cref="DensityScale"/> and <see cref="LiftToDragScale"/> are the estimator's corrections, measured over model, applied to every evaluation so each re-solve plans for the air the vehicle is actually in.
/// </summary>
public sealed class EntryModel
{
    /// <summary>Body radius, m. The planner's unit of length.</summary>
    public required double BodyRadius { get; init; }

    /// <summary>Gravitational parameter, m^3/s^2.</summary>
    public required double Mu { get; init; }

    /// <summary>Air density at an altitude above the body radius, kg/m^3.</summary>
    public required Func<double, double> Density { get; init; }

    /// <summary>Speed of sound at an altitude, m/s.</summary>
    public required Func<double, double> SpeedOfSound { get; init; }

    /// <summary>Top of the atmosphere, m: above it the density is taken as zero.</summary>
    public required double AtmosphereTop { get; init; }

    /// <summary>Drag coefficient on <see cref="ReferenceArea"/> at a Mach number, along the angle-of-attack schedule.</summary>
    public required Func<double, double> DragCoefficient { get; init; }

    /// <summary>Lift over drag at a Mach number, along the angle-of-attack schedule.</summary>
    public required Func<double, double> LiftToDrag { get; init; }

    public required double Mass { get; init; }
    public required double ReferenceArea { get; init; }

    /// <summary>Nose radius for the stagnation heating estimate, m.</summary>
    public double NoseRadius { get; init; } = 1.0;

    /// <summary>Sutton-Graves constant, SI (W/m^2 with rho in kg/m^3, v in m/s, nose radius in m). 1.7415e-4 for Earth's air.</summary>
    public double SuttonGraves { get; init; } = 1.7415e-4;

    public double DensityScale { get; init; } = 1.0;
    public double LiftToDragScale { get; init; } = 1.0;

    /// <summary>
    /// The body's rotation, rad/s, and the latitude and heading (from north, clockwise) the plan takes as fixed, rad. With them the flight-path equation carries the Coriolis and centrifugal terms of Lu's equations of motion; the speed is the speed over the ground. On a shallow coast they are not small: an eastward pass at 7.3 km/s feels a lift from them of about 70 % of the net pull of gravity at that speed, and without them (2026-11-21) the predicted coast came down too soon, the plan waited at 0.5 m/s^2 for air that was not there, and the craft reached it high.
    /// </summary>
    public double Omega { get; init; }
    public double Latitude { get; init; }
    public double Heading { get; init; }

    /// <summary>Gravity at the body radius, m/s^2: the planner's unit of acceleration.</summary>
    public double G0 => Mu / (BodyRadius * BodyRadius);

    /// <summary>Circular speed at the body radius, m/s: the planner's unit of speed.</summary>
    public double SpeedUnit => Math.Sqrt(Mu / BodyRadius);

    /// <summary>The planner's unit of time, s.</summary>
    public double TimeUnit => Math.Sqrt(BodyRadius / G0);

    public double Rho(double altitudeM)
        => altitudeM >= AtmosphereTop ? 0.0 : DensityScale * Density(Math.Max(altitudeM, 0.0));

    public double Mach(double altitudeM, double speedMs)
    {
        double a = SpeedOfSound(Math.Clamp(altitudeM, 0.0, AtmosphereTop));
        return a > 1e-6 ? speedMs / a : 0.0;
    }

    public double Ld(double mach) => LiftToDragScale * LiftToDrag(mach);

    /// <summary>Drag acceleration, m/s^2.</summary>
    public double DragAccel(double altitudeM, double speedMs)
    {
        double rho = Rho(altitudeM);
        double cd = DragCoefficient(Mach(altitudeM, speedMs));
        return rho * speedMs * speedMs * ReferenceArea * cd / (2.0 * Mass);
    }

    /// <summary>Stagnation-point heating rate, W/m^2 (Sutton-Graves).</summary>
    public double HeatingRate(double altitudeM, double speedMs)
        => SuttonGraves * Math.Sqrt(Rho(altitudeM) / NoseRadius) * speedMs * speedMs * speedMs;

    /// <summary>Dynamic pressure, Pa.</summary>
    public double DynamicPressure(double altitudeM, double speedMs)
        => 0.5 * Rho(altitudeM) * speedMs * speedMs;

    /// <summary>Aerodynamic load factor in Earth g, from drag and the lift that comes with it.</summary>
    public double LoadFactor(double altitudeM, double speedMs)
    {
        double d = DragAccel(altitudeM, speedMs);
        double ld = Ld(Mach(altitudeM, speedMs));
        return d * Math.Sqrt(1.0 + ld * ld) / StandardGravity;
    }

    public const double StandardGravity = 9.80665;

    /// <summary>A copy with new estimator corrections.</summary>
    public EntryModel WithScales(double densityScale, double liftToDragScale) => new()
    {
        BodyRadius = BodyRadius, Mu = Mu, Density = Density, SpeedOfSound = SpeedOfSound,
        AtmosphereTop = AtmosphereTop, DragCoefficient = DragCoefficient, LiftToDrag = LiftToDrag,
        Mass = Mass, ReferenceArea = ReferenceArea, NoseRadius = NoseRadius, SuttonGraves = SuttonGraves,
        DensityScale = densityScale, LiftToDragScale = liftToDragScale,
        Omega = Omega, Latitude = Latitude, Heading = Heading,
    };

    /// <summary>A copy at another latitude and heading.</summary>
    public EntryModel WithRotation(double omega, double latitude, double heading) => new()
    {
        BodyRadius = BodyRadius, Mu = Mu, Density = Density, SpeedOfSound = SpeedOfSound,
        AtmosphereTop = AtmosphereTop, DragCoefficient = DragCoefficient, LiftToDrag = LiftToDrag,
        Mass = Mass, ReferenceArea = ReferenceArea, NoseRadius = NoseRadius, SuttonGraves = SuttonGraves,
        DensityScale = DensityScale, LiftToDragScale = LiftToDragScale,
        Omega = omega, Latitude = latitude, Heading = heading,
    };

    /// <summary>A copy at another mass.</summary>
    public EntryModel WithMass(double massKg) => new()
    {
        BodyRadius = BodyRadius, Mu = Mu, Density = Density, SpeedOfSound = SpeedOfSound,
        AtmosphereTop = AtmosphereTop, DragCoefficient = DragCoefficient, LiftToDrag = LiftToDrag,
        Mass = massKg, ReferenceArea = ReferenceArea, NoseRadius = NoseRadius, SuttonGraves = SuttonGraves,
        DensityScale = DensityScale, LiftToDragScale = LiftToDragScale,
        Omega = Omega, Latitude = Latitude, Heading = Heading,
    };

    /// <summary>
    /// The rotating body's terms in the rates of speed and flight-path angle, normalised (Lu 2008, Eqs. 4 and 5): what V-dot gains and what V gamma-dot gains, at radius r, speed v and flight-path angle g.
    /// </summary>
    public void RotationTerms(double r, double v, double g, out double vDot, out double vGammaDot)
    {
        vDot = vGammaDot = 0.0;
        if (Omega == 0.0)
            return;
        double om = Omega * TimeUnit;
        double cphi = Math.Cos(Latitude), sphi = Math.Sin(Latitude);
        double cpsi = Math.Cos(Heading), spsi = Math.Sin(Heading);
        double cg = Math.Cos(g), sg = Math.Sin(g);
        vDot = om * om * r * cphi * (sg * cphi - cg * sphi * cpsi);
        vGammaDot = 2.0 * om * v * cphi * spsi + om * om * r * cphi * (cg * cphi + sg * cpsi * sphi);
    }
}
