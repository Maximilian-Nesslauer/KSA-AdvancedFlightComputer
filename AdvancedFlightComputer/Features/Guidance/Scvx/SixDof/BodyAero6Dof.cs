using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Touchdown guidance's aerodynamics from the body-axis tables (KsaAeroSweep.Result.BodyAero and BodyMoment): the force AND the moment the game applies for the air the craft meets in its own body axes, read off the plan's full attitude.
///
/// WHY. The 6-DOF plan's gimbal torque turns the craft against nothing but its inertia unless it knows the air's moment. A booster that is not axisymmetric, or whose centre of pressure is off its centre of mass, feels a steady moment through the landing burn; without it in the model the plan spends none of its gimbal on it, the tracker finds the craft turning away from the plan, and the gimbal it then spends pushes the thrust sideways. On 2026-11-18 a booster that entry guidance handed over on target drifted 5.4 m off it and touched down at 9.6 m/s. With the moment in the model, the plan holds the gimbal against it from the start, and the torque it sends the allocator is the one the craft needs.
///
/// NO HELD ROLL NEEDED. Unlike the 3-DOF point mass, the 6-DOF knows its roll: the air velocity is turned into the model's body axes by the plan's own quaternion, and into the tables' axes - the game's body axes - by the fixed rotation <see cref="ModelToTable"/>. Alpha and beta follow (BodyAeroTable's convention), and the force and moment come back the same way.
///
/// FADED OUT where the tables mean nothing: below <see cref="FadeSpeedLowMs"/> - near the ground the airflow's direction is set by a few metres a second of drift - and past <see cref="FadeTiltEndRad"/> off engine first, where the tables, sampled to 20 deg each way, end. A hard switch there would put a jump in the dynamics that the solver cannot converge across; the fade is a smoothstep in both, so the force and its slopes stay continuous. The caller makes up the faded share of the force from the whole-sphere tables (Dynamics6Dof.Params.Aero); the moment fades to none.
/// </summary>
public sealed class BodyAero6Dof
{
    /// <summary>The air, sampled from the game, as the 3-DOF model reads it.</summary>
    public required AtmosphereModel Atmosphere { get; init; }

    /// <summary>The body-axis force coefficients, C = F / (q S), in the game's body axes.</summary>
    public required BodyAeroTable Force { get; init; }

    /// <summary>The body-axis moment coefficients, C = M / (q S L), about <see cref="MomentComShift"/>'s origin, in the game's body axes; null for none.</summary>
    public BodyAeroTable? Moment { get; init; }

    /// <summary>Reference area and length the coefficients are scaled by, m^2 and m.</summary>
    public required double ReferenceArea { get; init; }
    public double ReferenceLength { get; init; }

    /// <summary>
    /// The model's body axes in the tables' axes, row-major 3x3: column j is model axis j (KsaFrameBridge.BodyAxes's mx, my, mz), so a table-axes vector is M times a model-axes one. Identity by default.
    /// </summary>
    public double[] ModelToTable { get; init; } = [1, 0, 0, 0, 1, 0, 0, 0, 1];

    /// <summary>
    /// The centre of mass the moment table was measured about, less the one the craft has now, in the tables' axes, m: the moment about the present centre of mass is the table's plus this cross the force.
    /// </summary>
    public double ComShiftX { get; init; }
    public double ComShiftY { get; init; }
    public double ComShiftZ { get; init; }

    /// <summary>The airspeed the tables fade in over, m/s: none below the low end, all above the high.</summary>
    /// <summary>
    /// THE GAME'S MOMENT LIMITER, per pascal of dynamic pressure: the moment is divided by 1 + MomentLimiterPerPa q. 0 for none.
    ///  KSA's integrator does not apply the air's moment as the tables measure it. ComputeAerodynamicLoads divides it by 1 + invInertiaBound dt q (0.5 dt stiffnessBound + dampingBound / V), a semi-implicit guard worked for the physics substep dt (0.02 s at 1x) and the largest inverse inertia - on a booster the small roll inertia's, so it bites on every axis. The sweep reads the loads at dt 0, without it. On the 2026-11-20 booster at 6-7 kPa it divided the moment by 3.7 on all three axes: the plan saw 40 kN m of roll the RCS could not hold, the booster felt 11 (headless, entry log's roll balance). Set from flight, measured at the game's own substep (KsaAeroSweep.TryMomentLimiter); the speed term is folded into it at the speed it was measured at.
    /// </summary>
    public double MomentLimiterPerPa { get; set; }

    public double FadeSpeedLowMs { get; init; } = 5.0;
    public double FadeSpeedHighMs { get; init; } = 15.0;

    /// <summary>The tilt off engine first the tables fade out over, radians: all of them to the start, none past the end. The tables are sampled to 20 deg each way.</summary>
    public double FadeTiltStartRad { get; init; } = 20.0 * Math.PI / 180.0;
    public double FadeTiltEndRad { get; init; } = 28.0 * Math.PI / 180.0;

    /// <summary>
    /// The aerodynamic force, site frame, N, and moment, model body axes, N m, for an altitude, the air velocity in the site frame and the body-to-site rotation R(q) by its nine entries; both already scaled by <paramref name="weight"/>, the share the tables carry (0 to 1).
    /// </summary>
    public void Wrench(Dual altitude, Dual vx, Dual vy, Dual vz,
                       Dual r00, Dual r01, Dual r02, Dual r10, Dual r11, Dual r12, Dual r20, Dual r21, Dual r22,
                       out Dual fx, out Dual fy, out Dual fz, out Dual tx, out Dual ty, out Dual tz, out Dual weight)
    {
        fx = fy = fz = tx = ty = tz = weight = default;
        Dual speed2 = vx * vx + vy * vy + vz * vz;
        if (!(speed2.V > FadeSpeedLowMs * FadeSpeedLowMs))
            return;
        Dual speed = Dual.Sqrt(speed2);

        // The air velocity in model body axes (R transposed), then in the tables' axes.
        Dual bx = r00 * vx + r10 * vy + r20 * vz;
        Dual by = r01 * vx + r11 * vy + r21 * vz;
        Dual bz = r02 * vx + r12 * vy + r22 * vz;
        double[] m = ModelToTable;
        Dual ux = (m[0] * bx + m[1] * by + m[2] * bz) / speed;
        Dual uy = (m[3] * bx + m[4] * by + m[5] * bz) / speed;
        Dual uz = (m[6] * bx + m[7] * by + m[8] * bz) / speed;

        // Engine first is u along -x; the tilt off it fades the tables out.
        Dual cosTilt = -ux;
        double cosEnd = Math.Cos(FadeTiltEndRad), cosStart = Math.Cos(FadeTiltStartRad);
        if (!(cosTilt.V > cosEnd))
            return;
        Dual wTilt = SmoothStep((cosTilt - cosEnd) / (cosStart - cosEnd));
        Dual wSpeed = SmoothStep((speed - FadeSpeedLowMs) / (FadeSpeedHighMs - FadeSpeedLowMs));
        weight = wTilt * wSpeed;

        Dual alpha = Dual.Atan2(uz, -ux);
        Dual beta = Dual.Atan2(uy, -ux);
        Dual mach = Atmosphere.Mach(speed);
        Dual qs = 0.5 * Atmosphere.Density(altitude) * speed2 * ReferenceArea * weight;

        Dual fm = Clamp(mach, Force.MachMin, Force.MachMax);
        Force.Coefficients(fm, alpha, beta, out Dual cx, out Dual cy, out Dual cz);
        Dual Fx = qs * cx, Fy = qs * cy, Fz = qs * cz;

        // Back to model axes (M transposed), and to the site frame (R).
        Dual mfx = m[0] * Fx + m[3] * Fy + m[6] * Fz;
        Dual mfy = m[1] * Fx + m[4] * Fy + m[7] * Fz;
        Dual mfz = m[2] * Fx + m[5] * Fy + m[8] * Fz;
        fx = r00 * mfx + r01 * mfy + r02 * mfz;
        fy = r10 * mfx + r11 * mfy + r12 * mfz;
        fz = r20 * mfx + r21 * mfy + r22 * mfz;

        // The moment about the present centre of mass: the table's, plus the shift cross the force.
        Dual Mx = ComShiftY * Fz - ComShiftZ * Fy;
        Dual My = ComShiftZ * Fx - ComShiftX * Fz;
        Dual Mz = ComShiftX * Fy - ComShiftY * Fx;
        if (Moment != null && ReferenceLength > 0.0)
        {
            Dual mm = Clamp(mach, Moment.MachMin, Moment.MachMax);
            Moment.Coefficients(mm, alpha, beta, out Dual cl, out Dual cm, out Dual cn);
            Dual qsl = qs * ReferenceLength;
            Mx += qsl * cl;
            My += qsl * cm;
            Mz += qsl * cn;
        }
        // As the game's integrator applies it (MomentLimiterPerPa): the whole moment about the centre of mass, divided by one factor.
        if (MomentLimiterPerPa > 0.0)
        {
            Dual limiter = 1.0 + MomentLimiterPerPa * (0.5 * Atmosphere.Density(altitude) * speed2);
            Mx /= limiter;
            My /= limiter;
            Mz /= limiter;
        }
        tx = m[0] * Mx + m[3] * My + m[6] * Mz;
        ty = m[1] * Mx + m[4] * My + m[7] * Mz;
        tz = m[2] * Mx + m[5] * My + m[8] * Mz;
    }

    private static Dual SmoothStep(Dual s)
    {
        if (!(s.V > 0.0)) return new Dual(0.0);
        if (!(s.V < 1.0)) return new Dual(1.0);
        return s * s * (3.0 - 2.0 * s);
    }

    private static Dual Clamp(Dual v, double lo, double hi)
        => v.V < lo ? new Dual(lo) : v.V > hi ? new Dual(hi) : v;
}
