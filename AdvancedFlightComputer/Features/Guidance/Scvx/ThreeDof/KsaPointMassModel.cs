using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// The point-mass model the guidance flies with, built entirely from plain numbers and immutable tables so a solve can run on a worker thread.
///
///   rdot = v
///   vdot = g(r) - 2 w x v - w x (w x r_c) + (T b + D + L) / m
///   mdot = -mdot(throttle)
///
/// THE FRAME ROTATES WITH THE BODY. It is the landing-site frame KsaFrameBridge builds - origin at the site, +z radially up - which the guidance rebuilds every step from the site's current position, so it turns with the planet. In it the site never moves, which keeps the terminal condition fixed whatever the landing time, and the velocity is the airspeed, because KSA's atmosphere co-rotates. The price is the Coriolis and centrifugal terms, which flat-earth 3dof.py did not need: from tens of kilometres out the curvature drop and a Coriolis push of order 0.1 m/s^2 are not negligible. Gravity is central, from the body's centre r_c = r - c. This is DragCoastSystem's physics in different coordinates, so the plan and the impact prediction agree.
///
/// AERODYNAMICS FROM THE GAME'S OWN FORCES. Drag and lift come from KsaAeroSweep's tables, alpha measured retrograde-first: the angle between the tail (-b) and the airflow, zero when flying engine first.
///   D = -q S Cd(M, alpha) v_hat
///   L = q S k(M, alpha) (-p),      p = b - (b . v_hat) v_hat,   |p| = sin(alpha),   k = C_L / sin(alpha)
/// The table's positive C_L is lift towards the side the engine end is tilted, and the engine end's tilt off the airflow is -p, so the lift is along -p; 3dof.py's retrograde sign s is the same thing. Written through k rather than C_L times a unit vector, the lift has no direction singularity at alpha = 0 and keeps its slope there: a seed flown at exactly alpha 0 still sees what tilting would buy.
/// NOT THE TABLE READ DIRECTLY. C_L times a rounded unit vector, -p / sqrt(|p|^2 + e^2), read at a rounded alpha, was tried (2026-11-18): the two roundings make the lift fall off as |p|^3 below e, so its slope at alpha 0 is zero, and a glide seeded at alpha 0 (GLIDE PLAN AoA 0) could not find what lift would buy. Offline (--3dof-mpc) nine of ten cold solves gave up, at either rounding. k keeps that slope, and with the sweep's breakpoint at 1 deg the value it holds below it is the 1 deg one.
///
/// THE CORNER AT alpha = 0. KSA's drag grows with |sin(alpha)|, which has a corner exactly where a booster flies, and a linearisation at the corner promises gains the true drag never pays. As the convex ascent does, the tables are read at an alpha whose |p| is rounded to |p|^2 / sqrt(|p|^2 + e^2): exact at zero, smooth, and about e^2 / 2|p| low elsewhere. Only the LOOKUP argument is rounded; the lift's direction comes from p itself.
///
/// THRUST FOLLOWS THE NOZZLE. Throttle sets the chamber, not the thrust, and mass flow follows the throttle: 3dof.py's mdot proportional to T is wrong under back pressure. In flight both come from <see cref="Engine"/>, the game's own engines sampled over throttle and pressure, because the nozzle's loss to the air falls with the chamber pressure and no constant deficit fits. Without one - the offline checks - the law is T = throttle * F_vac - Pa(h) A_e and mdot = throttle * mdot_max, which is exact at full throttle and only there.
///
/// Correction factors on drag and lift are set per solve by the guidance from what the vehicle is measured to experience - see <see cref="DragScale"/>.
/// </summary>
public sealed record KsaPointMassModel : IPointMassModel
{
    /// <summary>Gravitational parameter, m^3/s^2.</summary>
    public double Mu { get; init; }

    /// <summary>The body's mean radius, m - the atmosphere's altitude datum.</summary>
    public double MeanRadius { get; init; }

    /// <summary>The body's centre in the site frame, m. (0, 0, -R_site) for KsaFrameBridge's frame.</summary>
    public double CentreX { get; init; }
    public double CentreY { get; init; }
    public double CentreZ { get; init; }

    /// <summary>The body's angular velocity in the site frame, rad/s. Constant there, since the frame turns with the body.</summary>
    public double OmegaX { get; init; }
    public double OmegaY { get; init; }
    public double OmegaZ { get; init; }

    /// <summary>The air, or null for an airless body.</summary>
    public AtmosphereModel? Atmosphere { get; init; }

    /// <summary>Cd(Mach, alpha), retrograde-first. Null with no air.</summary>
    public AeroTable? Drag { get; init; }

    /// <summary>k(Mach, alpha) = C_L / sin(alpha), built by <see cref="LiftSlopeTable"/>. Null for no lift.</summary>
    public AeroTable? LiftSlope { get; init; }

    /// <summary>The area both tables are referenced to, m^2.</summary>
    public double ReferenceArea { get; init; }

    /// <summary>
    /// Multipliers on the tabulated drag and lift. The guidance estimates them from the acceleration the vehicle is measured to feel in the glide, where there is no thrust and the non-gravitational acceleration IS the aerodynamic force: the 6-DOF's additive acceleration bias would be wrong over a horizon where q changes tenfold.
    /// </summary>
    public double DragScale { get; init; } = 1.0;
    public double LiftScale { get; init; } = 1.0;

    /// <summary>
    /// A force coefficient across the airflow that the tables do not have, site frame: the model adds q S (C - (C . v_hat) v_hat). The guidance learns it in the glide (AeroScaleEstimator), where the measured lift fell short of the table's by about the same 1 m/s^2 at 1 deg and at 7 deg - an offset, not a ratio, which a lift scale could only follow by swinging with the angle of attack (0.5 to 1.1 within seconds, 2026-11-18) and moving the plan with it. As a coefficient it scales with the dynamic pressure like any aerodynamic force, so it fades as the craft slows and the last metres of a burn do not carry it.
    /// </summary>
    public double CrossBiasX { get; init; }
    public double CrossBiasY { get; init; }
    public double CrossBiasZ { get; init; }

    /// <summary>
    /// The body-axis force over Mach, alpha and beta at a fixed roll (KsaAeroSweep's BodyAero), read in place of Drag and LiftSlope wherever the craft flies within <see cref="BodyAeroMaxTiltRad"/> of engine first and <see cref="RollRef"/> is set. Null for none.
    /// </summary>
    public BodyAeroTable? BodyAero { get; init; }

    /// <summary>
    /// THE ROLL THE CRAFT IS HELD AT, site frame: the wing direction w - up x d, horizontal and square to the track to the site; fixed in the model, which entry guidance gives the one the flight holds as it moves (GuidanceWindow.ReturnRollRefCci) - with the body's +y along w's part square to b. The guidance commands that same roll (GuidanceWindow.ReturnRollRefCci), so the model knows the body axes for any pointing b and can read the body-axis table for the actual tilt: pitch in the body x-z plane, yaw towards +-y. In the glide it is wings level along the ground track, navball roll 0, and it stays defined as the axis passes vertical and through a boostback burn. Zero for none.
    /// </summary>
    public double RollRefX { get; init; }
    public double RollRefY { get; init; }
    public double RollRefZ { get; init; }

    /// <summary>Total tilt off engine first within which the body-axis table is read, radians: past it, the whole-sphere tables. The body table spans 20 deg each way.</summary>
    public double BodyAeroMaxTiltRad { get; init; } = 30.0 * Math.PI / 180.0;

    /// <summary>Rounding e of |p| in the table lookups, so the drag's |sin(alpha)| corner does not stall the linearisation. See the type summary.</summary>
    public double AlphaRounding { get; init; } = 0.02;

    /// <summary>Full-throttle vacuum thrust of the landing engines, N.</summary>
    public double VacuumThrust { get; init; }

    /// <summary>Nozzle exit area of the landing engines, m^2: the back-pressure loss is Pa * ExitArea.</summary>
    public double ExitArea { get; init; }

    /// <summary>Full-throttle mass flow, kg/s.</summary>
    public double MassFlow { get; init; }

    /// <summary>The engines as the game runs them, thrust over throttle and pressure and flow over throttle; null for the constant-loss law above. See <see cref="EngineThrustTable"/>.</summary>
    public EngineThrustTable? Engine { get; init; }

    /// <summary>Multiplier on the thrust: what the vehicle is measured to deliver against the law (Guidance3Dof's estimator). The flow is the engines' own and is not scaled.</summary>
    public double ThrustScale { get; init; } = 1.0;

    private const double VEps2 = PointMass3Dof.VEps2;

    public void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, Span<Dual> xdot)
    {
        Dual rx = x[PointMass3Dof.IR + 0], ry = x[PointMass3Dof.IR + 1], rz = x[PointMass3Dof.IR + 2];
        Dual vx = x[PointMass3Dof.IV + 0], vy = x[PointMass3Dof.IV + 1], vz = x[PointMass3Dof.IV + 2];
        Dual m = x[PointMass3Dof.IM];
        Dual bx = u[PointMass3Dof.IB + 0], by = u[PointMass3Dof.IB + 1], bz = u[PointMass3Dof.IB + 2];
        Dual throttle = u[PointMass3Dof.ITH];

        // Central gravity, from the body's centre.
        Dual cx = rx - CentreX, cy = ry - CentreY, cz = rz - CentreZ;
        Dual r2 = cx * cx + cy * cy + cz * cz + 1e-12;
        Dual rlen = Dual.Sqrt(r2);
        Dual gk = -Mu / (r2 * rlen);
        Dual ax = gk * cx, ay = gk * cy, az = gk * cz;

        // Coriolis, -2 w x v, and centrifugal, -w x (w x r_c).
        double wx = OmegaX, wy = OmegaY, wz = OmegaZ;
        ax -= 2.0 * (wy * vz - wz * vy);
        ay -= 2.0 * (wz * vx - wx * vz);
        az -= 2.0 * (wx * vy - wy * vx);
        Dual wr = wx * cx + wy * cy + wz * cz;
        double w2 = wx * wx + wy * wy + wz * wz;
        ax += w2 * cx - wr * wx;
        ay += w2 * cy - wr * wy;
        az += w2 * cz - wr * wz;

        Dual altitude = rlen - MeanRadius;
        Dual fx = default, fy = default, fz = default;

        // Thrust along the body axis, less the nozzle's back-pressure loss - only while the engine runs. The glide evaluates at throttle 0, where an unlit nozzle has no loss to take; the burn never goes below the floor, so the switch is never crossed inside a phase.
        if (throttle.V > EngineOnThrottle)
        {
            Dual pressure = Atmosphere != null ? Atmosphere.Pressure(altitude) : new Dual(0.0);
            Dual thrust = ThrustScale * (Engine != null ? Engine.Thrust(throttle, pressure) : throttle * VacuumThrust - pressure * ExitArea);
            fx += thrust * bx;
            fy += thrust * by;
            fz += thrust * bz;
        }

        AeroForce(altitude, vx, vy, vz, bx, by, bz, out Dual dx, out Dual dy, out Dual dz);
        fx += dx;
        fy += dy;
        fz += dz;

        xdot[PointMass3Dof.IR + 0] = vx;
        xdot[PointMass3Dof.IR + 1] = vy;
        xdot[PointMass3Dof.IR + 2] = vz;
        xdot[PointMass3Dof.IV + 0] = ax + fx / m;
        xdot[PointMass3Dof.IV + 1] = ay + fy / m;
        xdot[PointMass3Dof.IV + 2] = az + fz / m;
        // Zero with the engine off either way: the glide holds the throttle at 0, and the table's extension below the minimum throttle is not an unlit engine.
        xdot[PointMass3Dof.IM] = Engine == null ? -(throttle * MassFlow)
            : throttle.V > EngineOnThrottle ? -Engine.MassFlow(throttle) : new Dual(0.0);
    }

    /// <summary>
    /// The aerodynamic force, N, on a body with axis b (unit, the thrust direction) moving at v through the air at an altitude above the mean radius: drag along -v_hat and lift along -p, from the tables. Zero with no air or no table.
    /// The model's own dynamics use it, and so does the 6-DOF's (Dynamics6Dof.Params.Aero), so both planners fly the same aerodynamics: before, the 6-DOF had none, and a constant acceleration estimated at the hand-over's speed stood in for drag and lift that fall away as the booster slows.
    /// </summary>
    public void AeroForce(Dual altitude, Dual vx, Dual vy, Dual vz, Dual bx, Dual by, Dual bz,
                          out Dual fx, out Dual fy, out Dual fz)
    {
        fx = default;
        fy = default;
        fz = default;
        if (Atmosphere == null || Drag == null)
            return;
        Dual rho = Atmosphere.Density(altitude);
        if (!(rho.V > 0.0))
            return;

        Dual speed2 = vx * vx + vy * vy + vz * vz;
        Dual speed = Dual.Sqrt(speed2 + VEps2);
        Dual hx = vx / speed, hy = vy / speed, hz = vz / speed;
        Dual qs = 0.5 * rho * speed2 * ReferenceArea;

        // alpha from the tail: cos = -b . v_hat, sin = |p|, with |p| rounded for the lookup only.
        Dual bv = bx * hx + by * hy + bz * hz;
        Dual px = bx - bv * hx, py = by - bv * hy, pz = bz - bv * hz;
        Dual pp = px * px + py * py + pz * pz;
        Dual pRound = pp / Dual.Sqrt(pp + AlphaRounding * AlphaRounding);
        Dual alpha = Dual.Atan2(pRound, -bv);
        Dual machLocal = Atmosphere.Mach(speed, altitude);
        Dual mach = TableMach(machLocal, Drag);

        if (BodyAero != null && (RollRefX != 0.0 || RollRefY != 0.0 || RollRefZ != 0.0)
            && TryBodyAero(bx, by, bz, hx, hy, hz, qs, machLocal, out Dual bfx, out Dual bfy, out Dual bfz))
        {
            fx += bfx;
            fy += bfy;
            fz += bfz;
            AddCrossBias(qs, hx, hy, hz, ref fx, ref fy, ref fz);
            return;
        }

        Dual drag = DragScale * qs * Drag.Cd(mach, alpha);
        fx -= drag * hx;
        fy -= drag * hy;
        fz -= drag * hz;

        AddCrossBias(qs, hx, hy, hz, ref fx, ref fy, ref fz);

        if (LiftSlope != null)
        {
            Dual lift = LiftScale * qs * LiftSlope.Cd(TableMach(machLocal, LiftSlope), alpha);
            fx -= lift * px;
            fy -= lift * py;
            fz -= lift * pz;
        }
    }

    private void AddCrossBias(Dual qs, Dual hx, Dual hy, Dual hz, ref Dual fx, ref Dual fy, ref Dual fz)
    {
        if (CrossBiasX == 0.0 && CrossBiasY == 0.0 && CrossBiasZ == 0.0)
            return;
        Dual along = CrossBiasX * hx + CrossBiasY * hy + CrossBiasZ * hz;
        fx += qs * (CrossBiasX - along * hx);
        fy += qs * (CrossBiasY - along * hy);
        fz += qs * (CrossBiasZ - along * hz);
    }

    /// <summary>
    /// The body-axis table's force, N, for a body axis b and air direction h, with the roll held at RollRef: body x along b, y along w's part square to b, z = x x y; alpha and beta from h in those axes (BodyAeroTable); the force q S (C_x x + C_y y + C_z z), its part along the airflow times DragScale and the rest times LiftScale. False where the craft is not within BodyAeroMaxTiltRad of engine first, or the axes are degenerate (b along w, which a returning booster never flies).
    /// </summary>
    private bool TryBodyAero(Dual bx, Dual by, Dual bz, Dual hx, Dual hy, Dual hz, Dual qs, Dual mach,
                             out Dual fx, out Dual fy, out Dual fz)
    {
        fx = fy = fz = default;
        if (!BodyAero!.WorldCoefficients(mach, bx, by, bz, hx, hy, hz, RollRefX, RollRefY, RollRefZ,
                                         BodyAeroMaxTiltRad, out Dual cx, out Dual cy, out Dual cz))
            return false;
        Dual Fx = qs * cx;
        Dual Fy = qs * cy;
        Dual Fz = qs * cz;
        Dual along = Fx * hx + Fy * hy + Fz * hz;
        fx = LiftScale * (Fx - along * hx) + DragScale * along * hx;
        fy = LiftScale * (Fy - along * hy) + DragScale * along * hy;
        fz = LiftScale * (Fz - along * hz) + DragScale * along * hz;
        return true;
    }

    public double DynamicPressure(ReadOnlySpan<double> x)
    {
        if (Atmosphere == null) return 0.0;
        double v2 = x[PointMass3Dof.IV] * x[PointMass3Dof.IV] + x[PointMass3Dof.IV + 1] * x[PointMass3Dof.IV + 1]
                  + x[PointMass3Dof.IV + 2] * x[PointMass3Dof.IV + 2];
        return 0.5 * Atmosphere.Density(Altitude(x)) * v2;
    }

    /// <summary>Altitude above the mean radius, m - what the atmosphere is read at.</summary>
    public double Altitude(ReadOnlySpan<double> x)
    {
        double cx = x[0] - CentreX, cy = x[1] - CentreY, cz = x[2] - CentreZ;
        return Math.Sqrt(cx * cx + cy * cy + cz * cz) - MeanRadius;
    }

    /// <summary>Whether there is a lit engine to plan with.</summary>
    public bool HasThrust => VacuumThrust > 0.0;

    /// <summary>Thrust at a throttle and altitude, N, by the model's own law; zero with the engine off.</summary>
    public double Thrust(double throttle, double altitude)
        => throttle > EngineOnThrottle ? ThrustLit(throttle, altitude, out _, out _) : 0.0;

    /// <summary>Thrust of a lit engine with its slopes in throttle (N) and in altitude (N/m) - what the acceleration limit linearises.</summary>
    public double ThrustLit(double throttle, double altitude, out double dThrottle, out double dAltitude)
    {
        Dual pressure = Atmosphere != null ? Atmosphere.Pressure(new Dual(altitude, 1.0)) : new Dual(0.0);
        double thrust;
        if (Engine != null)
        {
            thrust = Engine.Thrust(throttle, pressure.V, out double dU, out double dP);
            dThrottle = dU;
            dAltitude = dP * pressure.D;
        }
        else
        {
            thrust = throttle * VacuumThrust - pressure.V * ExitArea;
            dThrottle = VacuumThrust;
            dAltitude = -pressure.D * ExitArea;
        }
        dThrottle *= ThrustScale;
        dAltitude *= ThrustScale;
        return ThrustScale * thrust;
    }

    /// <summary>The throttle that gives a thrust at an altitude by the same law, unclamped: <see cref="Thrust"/>'s inverse for a lit engine.</summary>
    public double ThrottleFor(double thrust, double altitude)
    {
        if (!HasThrust || !(ThrustScale > 0.0))
            return 0.0;
        double pressure = Atmosphere?.Pressure(altitude) ?? 0.0;
        return Engine != null
            ? Engine.ThrottleFor(thrust / ThrustScale, pressure)
            : (thrust / ThrustScale + pressure * ExitArea) / VacuumThrust;
    }

    /// <summary>Throttle below which the engine counts as off.</summary>
    public const double EngineOnThrottle = 1e-6;

    /// <summary>
    /// Mach held inside the table's grid: beyond its last breakpoint the fit extends along its end slope, which on a measured lift curve has been seen to change the lift's sign by Mach 8. Coefficients level off at high Mach, so holding the edge value is the better guess, and the slope there is zero rather than invented.
    /// </summary>
    private static Dual TableMach(Dual mach, AeroTable table)
    {
        if (mach.V < table.MachMin) return new Dual(table.MachMin);
        if (mach.V > table.MachMax) return new Dual(table.MachMax);
        return mach;
    }

    /// <summary>
    /// k(Mach, alpha) = C_L / sin(alpha) from a measured C_L grid (row-major, Mach slowest - KsaAeroSweep's LiftTable), on the breakpoints up to 90 degrees, which is far past any angle a booster is flown at.
    ///
    /// At alpha = 0 the ratio is 0/0, and its limit is the lift slope. The first non-zero breakpoint's ratio stands in for it - 1 deg on KsaAeroSweep's grid, which adds 1, 3 and 4 deg to the default 0, 2, 5: a slender body's crossflow lift grows faster than sin(alpha), so the ratio rises from zero, and extrapolating that rise back to alpha 0 could go negative and flip the sign of the lift at small angles. Holding it flat keeps the small-angle lift the right way round and slightly generous, where it matters least.
    /// </summary>
    public static AeroTable LiftSlopeTable(double[] machGrid, double[] alphaGridDeg, double[] clTable)
    {
        int nm = machGrid.Length, na = alphaGridDeg.Length;
        int keep = 0;
        while (keep < na && alphaGridDeg[keep] <= 90.0 + 1e-9) keep++;
        if (keep < 3) throw new ArgumentException("the alpha grid needs at least three breakpoints up to 90 degrees");

        var alpha = alphaGridDeg[..keep];
        var k = new double[nm * keep];
        for (int i = 0; i < nm; i++)
        {
            for (int j = 0; j < keep; j++)
            {
                double s = Math.Sin(alpha[j] * Math.PI / 180.0);
                k[i * keep + j] = s > 1e-6 ? clTable[i * na + j] / s : double.NaN;
            }
            for (int j = 0; j < keep; j++)
                if (double.IsNaN(k[i * keep + j]))
                    k[i * keep + j] = j + 1 < keep ? k[i * keep + j + 1] : 0.0;
        }
        return new AeroTable(machGrid, alpha, k);
    }
}
