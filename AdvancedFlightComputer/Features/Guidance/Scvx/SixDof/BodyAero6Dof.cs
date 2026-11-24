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
/// FADED OUT where the tables mean nothing: below <see cref="FadeSpeedLowMs"/> - near the ground the airflow's direction is set by a few metres a second of drift - and past <see cref="FadeTiltEndRad"/> off engine first, where the tables, sampled to 20 deg each way, end. A hard switch there would put a jump in the dynamics that the solver cannot converge across; the fade is a smoothstep in both, so the force and its slopes stay continuous.
///
/// PAST THE FADE, THE WHOLE SPHERE (<see cref="SphereForce"/>, KsaAeroSweep.Result.SphereBodyAero), where the sweep has one: force, moment and fins over every air direction, so a belly-flop's flip from belly first is flown on the air it meets, with the moment its gimbals turn it against. The fine table keeps the tilts it covers and the sphere takes the share it fades out of. Without a sphere the caller makes up that share from the roll-averaged tables (Dynamics6Dof.Params.Aero), and the moment fades to none.
/// </summary>
public sealed class BodyAero6Dof
{
    /// <summary>The air, sampled from the game, as the 3-DOF model reads it.</summary>
    public required AtmosphereModel Atmosphere { get; init; }

    /// <summary>The body-axis force coefficients, C = F / (q S), in the game's body axes.</summary>
    public required BodyAeroTable Force { get; init; }

    /// <summary>The body-axis moment coefficients, C = M / (q S L), about <see cref="MomentComShift"/>'s origin, in the game's body axes; null for none.</summary>
    public BodyAeroTable? Moment { get; init; }

    /// <summary>The same over every air direction (BodyAeroTable.Sphere), in the same axes and about the same centre of mass, or null for none: past the fine tables' fade they carry the force and moment.</summary>
    public BodyAeroTable? SphereForce { get; init; }
    public BodyAeroTable? SphereMoment { get; init; }

    /// <summary>
    /// A BELLY-FLOPPER'S FINER TABLES, between the engine-first ones and the whole sphere, or null for none: the belly-first tables bellyflop flies (KsaAeroSweep.Result.BellyBodyAero: alpha -20 to 80 deg towards the nose, beta 45 either way, about the belly) and the flip band between them and engine first (Result.BandBodyAero: the whole sphere's frame, its longitude from 15 past engine first to 10 past belly first, 45 deg of latitude, every 5 by 7.5 deg). Read in that order after the engine-first tables, each faded out over its edges, and the whole sphere takes what they leave: the sphere's 15 deg body and 30 deg flap grid alone flew a Starship's flip with its flaps credited three times what the game gave (2026-11-24).
    /// </summary>
    public BodyAeroTable? BellyForce { get; init; }
    public BodyAeroTable? BellyMoment { get; init; }
    public BodyAeroTable? BandForce { get; init; }
    public BodyAeroTable? BandMoment { get; init; }

    /// <summary>
    /// The measured error of the model's air moment (the game's less the model's at the craft's state, SixDof's moment-error integrator), per pascal of dynamic pressure, model body axes, N m/Pa: added to the moment at every state at its own dynamic pressure, as AccelBias corrects the force. 0 for none.
    /// </summary>
    public double MomentBiasXPerPa { get; set; }
    public double MomentBiasYPerPa { get; set; }
    public double MomentBiasZPerPa { get; set; }

    /// <summary>Which of a fin's table sets to read (FinCoefficients).</summary>
    public enum FinSet { Engine, Belly, Band, Sphere }

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

    /// <summary>
    /// The craft's control surfaces (KsaAeroSweep.SurfaceTable), in the order of the plan's fin controls (Dynamics6Dof.IFIN on), as many as it has: each one's change to the force and moment coefficients at each swept deflection, read at the flight condition and added to the body's, faded and limited with them. Empty for none.
    /// </summary>
    public FinTable[] Fins { get; init; } = [];

    /// <summary>One control surface: its swept deflections, degrees, ascending with 0 among them, and at each the change it makes to the body-axis force (per q S) and moment (per q S L) coefficients, zero at rest; its range and rate as the plan may use them.</summary>
    public sealed class FinTable
    {
        public required double[] DeflectionDeg { get; init; }
        public required BodyAeroTable[] Force { get; init; }
        public required BodyAeroTable[] Moment { get; init; }
        /// <summary>The same over every air direction (the body's sphere frame), per deflection; empty for none.</summary>
        public BodyAeroTable[] SphereForce { get; init; } = [];
        public BodyAeroTable[] SphereMoment { get; init; } = [];
        /// <summary>The same belly first and on the flip band (BodyAero6Dof.BellyForce, BandForce), per deflection; empty for none.</summary>
        public BodyAeroTable[] BellyForce { get; init; } = [];
        public BodyAeroTable[] BellyMoment { get; init; } = [];
        public BodyAeroTable[] BandForce { get; init; } = [];
        public BodyAeroTable[] BandMoment { get; init; } = [];
        public double MinDeg { get; init; }
        public double MaxDeg { get; init; }
        public double RateDegS { get; init; }
    }

    /// <summary>
    /// A control surface's coefficients at a deflection: straight between its swept deflections, held at its ends. On a swept deflection itself the slope is the mean of the two either side, so a plan linearised there - at rest, most of all, where the drag a fin adds bends the curve whichever way it turns - is told the curve's middle, not one side of it. A fin at rest that is not being differentiated adds nothing and reads no table.
    /// </summary>
    public static void FinCoefficients(FinTable fin, bool moment, Dual deg, Dual mach, Dual alpha, Dual beta,
                                       out Dual c0, out Dual c1, out Dual c2, FinSet set = FinSet.Engine)
    {
        c0 = c1 = c2 = default;
        double[] d = fin.DeflectionDeg;
        BodyAeroTable[] t = set switch
        {
            FinSet.Belly => moment ? fin.BellyMoment : fin.BellyForce,
            FinSet.Band => moment ? fin.BandMoment : fin.BandForce,
            FinSet.Sphere => moment ? fin.SphereMoment : fin.SphereForce,
            _ => moment ? fin.Moment : fin.Force,
        };
        int n = d.Length;
        if (n < 2 || t.Length != n || (deg.V == 0.0 && deg.D == 0.0))
            return;
        double v = Math.Clamp(deg.V, d[0], d[n - 1]);
        int k = 0;
        while (k + 2 < n && v > d[k + 1])
            k++;
        // On a swept deflection inside the range: the value there, the mean slope either side.
        int at = -1;
        for (int i = 1; i + 1 < n; i++)
            if (Math.Abs(v - d[i]) <= 1e-9)
                at = i;
        Dual m0 = Clamp(mach, t[0].MachMin, t[0].MachMax);
        if (at >= 0)
        {
            t[at - 1].Coefficients(m0, alpha, beta, out Dual a0, out Dual a1, out Dual a2);
            t[at].Coefficients(m0, alpha, beta, out Dual b0, out Dual b1, out Dual b2);
            t[at + 1].Coefficients(m0, alpha, beta, out Dual e0, out Dual e1, out Dual e2);
            double wl = d[at] - d[at - 1], wr = d[at + 1] - d[at];
            Dual step = deg - d[at];
            c0 = b0 + step * 0.5 * ((b0 - a0) / wl + (e0 - b0) / wr);
            c1 = b1 + step * 0.5 * ((b1 - a1) / wl + (e1 - b1) / wr);
            c2 = b2 + step * 0.5 * ((b2 - a2) / wl + (e2 - b2) / wr);
            return;
        }
        t[k].Coefficients(m0, alpha, beta, out Dual l0, out Dual l1, out Dual l2);
        t[k + 1].Coefficients(m0, alpha, beta, out Dual h0, out Dual h1, out Dual h2);
        // Held at the ends: past them the deflection moves nothing.
        Dual w = deg.V <= d[0] ? new Dual(0.0) : deg.V >= d[n - 1] ? new Dual(1.0) : (deg - d[k]) / (d[k + 1] - d[k]);
        c0 = l0 + w * (h0 - l0);
        c1 = l1 + w * (h1 - l1);
        c2 = l2 + w * (h2 - l2);
    }

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
                       out Dual fx, out Dual fy, out Dual fz, out Dual tx, out Dual ty, out Dual tz, out Dual weight,
                       ReadOnlySpan<Dual> finDeg = default)
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

        // Engine first is u along -x; the tilt off it fades the fine tables out. What they leave goes to the flip band, then the belly-first tables, each by how far inside its own edges the air is, and the whole sphere, where there is one, takes the rest.
        Dual cosTilt = -ux;
        double cosEnd = Math.Cos(FadeTiltEndRad), cosStart = Math.Cos(FadeTiltStartRad);
        bool sphere = SphereForce != null;
        Dual wTilt = cosTilt.V > cosEnd ? SmoothStep((cosTilt - cosEnd) / (cosStart - cosEnd)) : new Dual(0.0);
        Dual rest = 1.0 - wTilt;
        Dual wBand = default, wBelly = default, lamB = default, muB = default, alB = default, beB = default;
        if (BandForce != null && rest.V > 0.0)
        {
            BodyAeroTable.SphereAngles(BandForce.Frame, ux, uy, uz, out lamB, out muB);
            Dual f = BandWeight(lamB, muB);
            wBand = rest * f;
            rest *= 1.0 - f;
        }
        if (BellyForce != null && rest.V > 0.0 && BellyAngles(BellyForce.Frame, ux, uy, uz, out alB, out beB))
        {
            Dual f = BellyWeight(alB, beB);
            wBelly = rest * f;
            rest *= 1.0 - f;
        }
        Dual wSphere = sphere ? rest : new Dual(0.0);
        if (!(wTilt.V > 0.0) && !(wBand.V > 0.0) && !(wBelly.V > 0.0) && !sphere)
            return;
        Dual wSpeed = SmoothStep((speed - FadeSpeedLowMs) / (FadeSpeedHighMs - FadeSpeedLowMs));
        weight = (wTilt + wBand + wBelly + wSphere) * wSpeed;

        Dual mach = Atmosphere.Mach(speed);
        Dual qs = 0.5 * Atmosphere.Density(altitude) * speed2 * ReferenceArea * wSpeed;

        // The coefficients, force and moment with the control surfaces', as a blend of the two tables by their shares.
        Dual cx = default, cy = default, cz = default, fcl = default, fcm = default, fcn = default;
        int fins = Math.Min(Fins.Length, finDeg.Length);
        if (wTilt.V > 0.0)
        {
            Dual alpha = Dual.Atan2(uz, -ux);
            Dual beta = Dual.Atan2(uy, -ux);
            AddCoefficients(Force, Moment, FinSet.Engine, wTilt, mach, alpha, beta, fins, finDeg, ref cx, ref cy, ref cz, ref fcl, ref fcm, ref fcn);
        }
        if (wBand.V > 0.0)
            AddCoefficients(BandForce!, BandMoment, FinSet.Band, wBand, mach, lamB, muB, fins, finDeg, ref cx, ref cy, ref cz, ref fcl, ref fcm, ref fcn);
        if (wBelly.V > 0.0)
            AddCoefficients(BellyForce!, BellyMoment, FinSet.Belly, wBelly, mach, alB, beB, fins, finDeg, ref cx, ref cy, ref cz, ref fcl, ref fcm, ref fcn);
        if (wSphere.V > 0.0)
        {
            BodyAeroTable.SphereAngles(SphereForce!.Frame, ux, uy, uz, out Dual lambda, out Dual mu);
            AddCoefficients(SphereForce, SphereMoment, FinSet.Sphere, wSphere, mach, lambda, mu, fins, finDeg, ref cx, ref cy, ref cz, ref fcl, ref fcm, ref fcn);
        }
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
        if (ReferenceLength > 0.0)
        {
            Dual qsl = qs * ReferenceLength;
            Mx += qsl * fcl;
            My += qsl * fcm;
            Mz += qsl * fcn;
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
        // The measured error of the moment (MomentBias*PerPa), at this state's dynamic pressure, model body axes, as the game applies it: after the limiter.
        if (MomentBiasXPerPa != 0.0 || MomentBiasYPerPa != 0.0 || MomentBiasZPerPa != 0.0)
        {
            Dual qPa = 0.5 * Atmosphere.Density(altitude) * speed2 * wSpeed;
            tx += MomentBiasXPerPa * qPa;
            ty += MomentBiasYPerPa * qPa;
            tz += MomentBiasZPerPa * qPa;
        }
    }

    /// <summary>The belly-first table's alpha and beta (radians) of an air direction (BodyAeroTable.Angles, as Duals); false where the belly does not lead.</summary>
    private static bool BellyAngles(in BodyAeroFrame frame, Dual ux, Dual uy, Dual uz, out Dual alpha, out Dual beta)
    {
        Dual lead = ux * frame.LX + uy * frame.LY + uz * frame.LZ;
        alpha = beta = default;
        if (!(lead.V > 1e-6))
            return false;
        alpha = Dual.Atan2(ux * frame.AX + uy * frame.AY + uz * frame.AZ, lead);
        beta = Dual.Atan2(ux * frame.BX + uy * frame.BY + uz * frame.BZ, lead);
        return true;
    }

    private const double Deg = Math.PI / 180.0;

    /// <summary>How much of the air the belly-first tables take where they reach: all of it from 12 deg towards the engine to 70 towards the nose and inside 35 of beta, none past their -20, 80 and 45.</summary>
    private static Dual BellyWeight(Dual alpha, Dual beta)
    {
        Dual absB = beta.V < 0.0 ? -beta : beta;
        return SmoothStep((alpha + 20.0 * Deg) / (8.0 * Deg)) * SmoothStep((80.0 * Deg - alpha) / (10.0 * Deg))
               * SmoothStep((45.0 * Deg - absB) / (10.0 * Deg));
    }

    /// <summary>How much of what the engine-first tables leave the flip band takes: all of it from 5 past engine first to 85 (5 short of belly first) and inside 35 of latitude, none past its -15, 100 and 45.</summary>
    private static Dual BandWeight(Dual lambda, Dual mu)
    {
        Dual absM = mu.V < 0.0 ? -mu : mu;
        return SmoothStep((lambda + 15.0 * Deg) / (10.0 * Deg)) * SmoothStep((100.0 * Deg - lambda) / (15.0 * Deg))
               * SmoothStep((45.0 * Deg - absM) / (10.0 * Deg));
    }

    /// <summary>One table set's force and moment coefficients, the body's and its fins' at their deflections, at its angles, added in at a share.</summary>
    private void AddCoefficients(BodyAeroTable force, BodyAeroTable? moment, FinSet set, Dual share, Dual mach, Dual a, Dual b,
                                 int fins, ReadOnlySpan<Dual> finDeg,
                                 ref Dual cx, ref Dual cy, ref Dual cz, ref Dual cl, ref Dual cm, ref Dual cn)
    {
        force.Coefficients(Clamp(mach, force.MachMin, force.MachMax), a, b, out Dual x, out Dual y, out Dual z);
        Dual l = default, m = default, n = default;
        for (int i = 0; i < fins; i++)
        {
            FinCoefficients(Fins[i], false, finDeg[i], mach, a, b, out Dual dx, out Dual dy, out Dual dz, set);
            x += dx; y += dy; z += dz;
            FinCoefficients(Fins[i], true, finDeg[i], mach, a, b, out Dual dl, out Dual dm, out Dual dn, set);
            l += dl; m += dm; n += dn;
        }
        if (moment != null && ReferenceLength > 0.0)
        {
            moment.Coefficients(Clamp(mach, moment.MachMin, moment.MachMax), a, b, out Dual bl, out Dual bm, out Dual bn);
            l += bl; m += bm; n += bn;
        }
        cx += share * x; cy += share * y; cz += share * z;
        cl += share * l; cm += share * m; cn += share * n;
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
