using Navbox.Numerics;

namespace Navbox.Flight;

/// <summary>
/// A boostback burn described by five numbers, flown with a linear tangent steering
/// law and followed by a retrograde coast to the ground.
///
/// The angles are measured in a frame built from the state at ignition: local vertical
/// for pitch, the orbit normal for yaw, and RETROGRADE-HORIZONTAL as the zero. So
/// (0, 0) is "point backwards along the horizon", which is roughly where a boostback
/// wants to be, and the numbers stay small and readable.
/// </summary>
public struct BurnParameters
{
    /// <summary>Pitch of the thrust direction at ignition, degrees above the horizon.</summary>
    public double PitchDeg;

    /// <summary>Yaw of the thrust direction at ignition, degrees out of the orbit plane.</summary>
    public double YawDeg;

    /// <summary>Rate the pitch sweeps at, degrees per second.</summary>
    public double PitchRateDegS;

    /// <summary>Rate the yaw sweeps at, degrees per second.</summary>
    public double YawRateDegS;

    /// <summary>Burn duration, s. At fixed throttle this IS the propellant, so it is
    /// both a variable and the objective.</summary>
    public double Duration;
}

/// <summary>
/// Powered flight under a linear tangent steering law, for the shooting solve.
///
/// Seven states: position, velocity and MASS - the mass matters because thrust
/// acceleration is T/m and m falls through the burn, which is a large part of why an
/// impulsive model gets a long burn wrong.
///
/// THE STEERING LAW is i_F(tau) = unit(lambda + lambdadot * tau). That is the general
/// (bilinear) tangent form: for constant gravity the optimal thrust direction is the
/// primer vector, the primer satisfies pddot = 0, so p = a + b*tau exactly and the
/// direction is its normalisation. It is not an approximation of the optimum in that
/// setting, it IS the optimum. With drag and a non-state terminal condition it stops
/// being provably optimal and becomes a parameterisation - a small, well-shaped family
/// of controls to search within, which is all a direct shooting method needs.
///
/// ALPHA IS COMPUTED, NOT ASSUMED. During the burn the vehicle points along the thrust,
/// so its angle of attack is the angle between the thrust direction and pure retrograde
/// - which is exactly zero only if the burn is aimed straight back. A lofted burn flies
/// at a real angle of attack and the drag is different for it. This is the one place
/// the coast's alpha = 0 assumption does NOT hold, so it is worked out rather than
/// inherited.
/// </summary>
public struct PoweredBurnSystem : IOdeSystem
{
    /// <summary>State layout: position, velocity, mass.</summary>
    public const int N = 7;

    public double Mu;
    public double OmegaZ;
    public double MeanRadius;

    /// <summary>Reference area the Cd table is written against, m^2.</summary>
    public double ReferenceArea;

    /// <summary>Thrust, N. Constant - boostback runs at full throttle.</summary>
    public double Thrust;

    /// <summary>Propellant mass flow, kg/s.</summary>
    public double MassFlow;

    public AeroTable Table;
    public ExponentialAtmosphere Atmosphere;

    /// <summary>Steering: the direction at ignition. Dual because the shoot
    /// differentiates with respect to the angles that built it.</summary>
    public Dual Lx, Ly, Lz;

    /// <summary>Steering: the turning vector, perpendicular to lambda.</summary>
    public Dual Dx, Dy, Dz;

    public readonly bool HasDrag => Table != null && Atmosphere != null;

    public readonly void Derivative(Dual t, ReadOnlySpan<Dual> x, Span<Dual> dx)
    {
        Dual rx = x[0], ry = x[1], rz = x[2];
        Dual vx = x[3], vy = x[4], vz = x[5];
        Dual m = x[6];

        dx[0] = vx; dx[1] = vy; dx[2] = vz;

        // Gravity.
        Dual r2 = rx * rx + ry * ry + rz * rz + 1e-12;
        Dual rlen = Dual.Sqrt(r2);
        Dual gk = -Mu / (r2 * rlen);
        dx[3] = gk * rx; dx[4] = gk * ry; dx[5] = gk * rz;

        // Steering: unit(lambda + lambdadot * t). t is elapsed burn time.
        Dual sx = Lx + Dx * t, sy = Ly + Dy * t, sz = Lz + Dz * t;
        Dual slen = Dual.Sqrt(sx * sx + sy * sy + sz * sz + 1e-18);
        Dual ix = sx / slen, iy = sy / slen, iz = sz / slen;

        // Thrust acceleration.
        Dual acc = Thrust / m;
        dx[3] += acc * ix; dx[4] += acc * iy; dx[5] += acc * iz;

        dx[6] = new Dual(-MassFlow);

        if (!HasDrag)
            return;

        // Air-relative velocity: the atmosphere co-rotates.
        Dual ax = vx + OmegaZ * ry;
        Dual ay = vy - OmegaZ * rx;
        Dual az = vz;
        Dual sp2 = ax * ax + ay * ay + az * az + 1e-18;
        Dual sp = Dual.Sqrt(sp2);

        Dual rho = Atmosphere.Density(rlen - MeanRadius);
        if (rho.V == 0.0)
            return;

        // Angle of attack, retrograde-first: the angle between the TAIL (-thrust) and
        // the relative wind. Zero when the burn is aimed straight back.
        Dual wx = ax / sp, wy = ay / sp, wz = az / sp;
        Dual dot = -(ix * wx + iy * wy + iz * wz);
        Dual cx = -(iy * wz - iz * wy);
        Dual cy = -(iz * wx - ix * wz);
        Dual cz = -(ix * wy - iy * wx);
        Dual cross = Dual.Sqrt(cx * cx + cy * cy + cz * cz + 1e-18);
        Dual alpha = Dual.Atan2(cross, dot);

        Dual mach = sp / Atmosphere.SpeedOfSound;
        Dual cd = Table.Cd(mach, alpha);
        Dual k = 0.5 * rho * sp * cd * ReferenceArea / m;
        dx[3] -= k * ax; dx[4] -= k * ay; dx[5] -= k * az;
    }
}

/// <summary>What one shot produced.</summary>
public struct ShotResult
{
    /// <summary>The burn reached the ground and everything below is meaningful.</summary>
    public bool Valid;

    /// <summary>Miss against the target in the co-rotating ground frame, m. Two of its
    /// three components are steerable; the radial one is not - see ImpactSteering.</summary>
    public Dual MissX, MissY, MissZ;

    /// <summary>Downrange and crossrange miss, m - the two numbers the inner solve
    /// drives to zero.</summary>
    public Dual MissDownrange, MissCrossrange;

    /// <summary>Ground impact point, co-rotating frame.</summary>
    public Dual Fx, Fy, Fz;

    /// <summary>State at cutoff, for diagnosis.</summary>
    public Dual CutoffAltitude, CutoffSpeed, PropellantUsed;

    /// <summary>Coast time from cutoff to the ground, s.</summary>
    public double CoastTime;

    /// <summary>
    /// The shot is physically impossible, not numerically awkward. Worth separating,
    /// because a search that reads infeasibility as solver trouble will keep hammering
    /// at a region no amount of iteration can reach - and because a whole infeasible
    /// region is itself a finding about the vehicle.
    ///
    /// Three ways to get here: the tanks run dry, the burn flies the vehicle into the
    /// ground, or the coast afterwards never comes back down.
    /// </summary>
    public bool Infeasible;

    /// <summary>The specific case where the tanks ran dry, for the readout.</summary>
    public bool OutOfPropellant;

    /// <summary>The specific case where the burn drove the vehicle below the surface.</summary>
    public bool FlewIntoGround;
}

/// <summary>
/// Direct shooting on a linear tangent boostback burn.
///
/// THE SHAPE OF THE PROBLEM. Five knobs describe a burn: pitch, yaw, pitch rate, yaw
/// rate and duration. Two things must come out right - the downrange and crossrange
/// miss. That leaves three knobs spare, and the objective (propellant, which at fixed
/// throttle is just the duration) is minimised over them.
///
/// WHICH KNOB DOES WHICH JOB is not arbitrary, and the split is the whole design:
///
///   inner, driven to hit the site   duration -> downrange, yaw -> crossrange
///   outer, free to be optimised     pitch, pitch rate, yaw rate
///
/// Duration and yaw are chosen because each is the strongest and most INDEPENDENT
/// lever on one component of the miss: burning longer kills more velocity and lands
/// shorter, yawing lands sideways, and neither much affects the other. Pitch would be
/// a poor inner knob - it acts on downrange too, but weakly, because it changes the
/// direction of the dv rather than its size.
///
/// AND THE INNER LOOP USES EXACTLY TWO. Given three it would be underdetermined and
/// would need a minimum-norm choice, which is precisely how the greedy law ends up
/// pointing at the ground: the cheapest way to pull an impact point back is to thrust
/// down. Restricting the inner solve to two knobs keeps that decision where it belongs
/// - in the outer loop, where it is optimised rather than stumbled into.
///
/// Note that duration is an inner knob AND the objective. That is not circular: the
/// inner loop answers "how long a burn do I need to hit the site, at this pitch and
/// turn rate", and the outer loop minimises that answer.
/// </summary>
public static class BoostbackShooter
{
    private const double Deg = System.Math.PI / 180.0;

    /// <summary>
    /// Integration nodes across the powered arc. Fixed, so that varying the duration
    /// changes the step SIZE rather than the step COUNT - a changing count puts small
    /// discontinuities in the objective exactly where a gradient-based search would
    /// trip over them.
    ///
    /// SIXTEEN IS AMPLE, and that was measured rather than guessed: the burn is a
    /// smooth arc of under a minute, and --shoot finds EIGHT nodes landing in the
    /// identical place as 240. This is double what the measurement said is needed, and
    /// it was 60 until that measurement was taken - which made every shot four times
    /// dearer than it had to be, and the in-flight solve unaffordable.
    /// </summary>
    public const int BurnNodes = 16;

    /// <summary>Scratch this needs, in Duals.</summary>
    public const int ScratchLength = (Rk4.ScratchPerState + 3) * PoweredBurnSystem.N
                                   + ImpactPredictor.ScratchLength;

    /// <summary>
    /// The frame the steering angles are measured in, built from the state at ignition:
    /// Up is local vertical, Back is retrograde-horizontal (the zero for pitch and
    /// yaw), and Side completes it out of the orbit plane.
    /// </summary>
    public struct Frame
    {
        public double Ux, Uy, Uz;      // local vertical
        public double Bx, By, Bz;      // retrograde horizontal - the zero
        public double Sx, Sy, Sz;      // out of plane

        public static Frame FromState(ReadOnlySpan<double> x)
        {
            double rl = System.Math.Sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
            double ux = x[0] / rl, uy = x[1] / rl, uz = x[2] / rl;

            // Orbit normal, r x v.
            double hx = x[1] * x[5] - x[2] * x[4];
            double hy = x[2] * x[3] - x[0] * x[5];
            double hz = x[0] * x[4] - x[1] * x[3];
            double hl = System.Math.Sqrt(hx * hx + hy * hy + hz * hz);
            if (hl <= 0.0) { hx = 0; hy = 0; hz = 1; hl = 1; }
            hx /= hl; hy /= hl; hz /= hl;

            // Forward horizontal = h x up; retrograde horizontal is its negative.
            double fx = hy * uz - hz * uy;
            double fy = hz * ux - hx * uz;
            double fz = hx * uy - hy * ux;

            return new Frame
            {
                Ux = ux, Uy = uy, Uz = uz,
                Bx = -fx, By = -fy, Bz = -fz,
                Sx = hx, Sy = hy, Sz = hz,
            };
        }

        /// <summary>Unit direction at a pitch and yaw, in radians.</summary>
        public readonly void Direction(Dual pitch, Dual yaw,
                                       out Dual dx, out Dual dy, out Dual dz)
        {
            Dual cp = Dual.Cos(pitch), sp = Dual.Sin(pitch);
            Dual cy = Dual.Cos(yaw), sy = Dual.Sin(yaw);
            Dual b = cp * cy, u = sp, s = cp * sy;
            dx = b * Bx + u * Ux + s * Sx;
            dy = b * By + u * Uy + s * Sy;
            dz = b * Bz + u * Uz + s * Sz;
        }
    }

    /// <summary>
    /// The steering law's two vectors: lambda, the direction at ignition, and lambdadot,
    /// the turning vector perpendicular to it. The law is unit(lambda + lambdadot * tau).
    ///
    /// SINGLE SOURCE, deliberately. The shot integrates this law, the flight computer is
    /// pointed along it, and any check that replays a plan has to reproduce it; if the
    /// three constructions ever drifted apart, the vehicle would fly something other
    /// than what was optimised and nothing would say so.
    ///
    /// The perpendicular basis is finite-differenced rather than differentiated: it only
    /// has to be perpendicular to lambda to first order, and 1e-4 rad is far below any
    /// angle that matters while staying far above rounding.
    /// </summary>
    public static void SteeringVectors(in Frame frame, Dual pitch, Dual yaw,
                                       Dual pitchRate, Dual yawRate,
                                       out Dual lx, out Dual ly, out Dual lz,
                                       out Dual dx, out Dual dy, out Dual dz)
    {
        frame.Direction(pitch, yaw, out lx, out ly, out lz);
        frame.Direction(pitch + 1e-4, yaw, out Dual px, out Dual py, out Dual pz);
        frame.Direction(pitch, yaw + 1e-4, out Dual qx, out Dual qy, out Dual qz);

        Dual ux = (px - lx) * 1e4, uy = (py - ly) * 1e4, uz = (pz - lz) * 1e4;
        Dual wx = (qx - lx) * 1e4, wy = (qy - ly) * 1e4, wz = (qz - lz) * 1e4;
        Dual wl = Dual.Sqrt(wx * wx + wy * wy + wz * wz + 1e-18);
        wx /= wl; wy /= wl; wz /= wl;

        dx = ux * pitchRate + wx * yawRate;
        dy = uy * pitchRate + wy * yawRate;
        dz = uz * pitchRate + wz * yawRate;
    }

    /// <summary>
    /// The plan's commanded thrust direction, tau seconds after ignition.
    ///
    /// EVALUATES the law rather than sampling a stored vector. It is a function of time,
    /// and a flight computer handed a stationary target has to chase it instead of
    /// tracking it - the same reason the boostback publishes a target rate alongside
    /// the target.
    /// </summary>
    public static void SteeringAt(in Frame frame, in BurnParameters bp, double tau,
                                  out double dx, out double dy, out double dz)
    {
        SteeringVectors(in frame, new Dual(bp.PitchDeg * Deg), new Dual(bp.YawDeg * Deg),
                        new Dual(bp.PitchRateDegS * Deg), new Dual(bp.YawRateDegS * Deg),
                        out Dual lx, out Dual ly, out Dual lz,
                        out Dual tx, out Dual ty, out Dual tz);

        double sx = lx.V + tx.V * tau, sy = ly.V + ty.V * tau, sz = lz.V + tz.V * tau;
        double sl = System.Math.Sqrt(sx * sx + sy * sy + sz * sz);
        if (!(sl > 1e-12)) { dx = lx.V; dy = ly.V; dz = lz.V; return; }
        dx = sx / sl; dy = sy / sl; dz = sz / sl;
    }

    /// <summary>
    /// A copy of the system set up to fly this plan - the template's physics with the
    /// plan's steering law installed. What the shot integrates, and what a replay of a
    /// flown plan has to integrate for the two to mean the same thing.
    /// </summary>
    public static PoweredBurnSystem WithSteering(in PoweredBurnSystem template,
                                                 in Frame frame, in BurnParameters bp)
    {
        SteeringVectors(in frame, new Dual(bp.PitchDeg * Deg), new Dual(bp.YawDeg * Deg),
                        new Dual(bp.PitchRateDegS * Deg), new Dual(bp.YawRateDegS * Deg),
                        out Dual lx, out Dual ly, out Dual lz,
                        out Dual dx, out Dual dy, out Dual dz);
        var sys = template;
        sys.Lx = lx; sys.Ly = ly; sys.Lz = lz;
        sys.Dx = dx; sys.Dy = dy; sys.Dz = dz;
        return sys;
    }

    /// <summary>
    /// Fly one burn and coast, and report where it lands.
    ///
    /// Any of the five parameters may be seeded; the answer carries the derivative
    /// with respect to it, through the burn, the cutoff and the coast alike.
    /// </summary>
    /// <param name="x0">Initial [r(3), v(3)], inertial. Plain values - the burn's
    /// sensitivity is to the PARAMETERS, not the state.</param>
    /// <param name="mass0">Mass at ignition, kg.</param>
    /// <param name="target">Target point in the co-rotating ground frame, 3 values.</param>
    public static ShotResult Shoot(in PoweredBurnSystem template, in Frame frame,
                                   ReadOnlySpan<double> x0, double mass0,
                                   Dual pitch, Dual yaw, Dual pitchRate, Dual yawRate,
                                   Dual duration,
                                   ReadOnlySpan<double> target,
                                   in ImpactOptions coastOpt, Span<Dual> scratch)
        => ShootAt(in template, in frame, x0, mass0, pitch, yaw, pitchRate, yawRate,
                   duration, target, in coastOpt, scratch, BurnNodes);

    /// <summary>As <see cref="Shoot"/>, with the burn node count given explicitly.
    /// Exposed so the discretisation can be priced rather than assumed.</summary>
    public static ShotResult ShootAt(in PoweredBurnSystem template, in Frame frame,
                                     ReadOnlySpan<double> x0, double mass0,
                                     Dual pitch, Dual yaw, Dual pitchRate, Dual yawRate,
                                     Dual duration,
                                     ReadOnlySpan<double> target,
                                     in ImpactOptions coastOpt, Span<Dual> scratch,
                                     int nodes)
    {
        var result = new ShotResult();
        if (duration.V < 0.0 || !double.IsFinite(duration.V))
            return result;

        // The law, from the one place that builds it - so the arc integrated here and
        // the direction the vehicle is pointed cannot drift apart.
        SteeringVectors(in frame, pitch, yaw, pitchRate, yawRate,
                        out Dual lx, out Dual ly, out Dual lz,
                        out Dual tdx, out Dual tdy, out Dual tdz);

        var sys = template;
        sys.Lx = lx; sys.Ly = ly; sys.Lz = lz;
        sys.Dx = tdx; sys.Dy = tdy; sys.Dz = tdz;

        // --- the powered arc ---
        const int N = PoweredBurnSystem.N;
        Span<Dual> work = scratch.Slice(0, Rk4.ScratchPerState * N);
        Span<Dual> a = scratch.Slice(Rk4.ScratchPerState * N, N);
        Span<Dual> b = scratch.Slice((Rk4.ScratchPerState + 1) * N, N);
        Span<Dual> coastScratch = scratch.Slice((Rk4.ScratchPerState + 3) * N,
                                                ImpactPredictor.ScratchLength);

        for (int i = 0; i < 6; i++) a[i] = new Dual(x0[i]);
        a[6] = new Dual(mass0);

        Dual h = duration / nodes;
        Dual t = new Dual(0.0);
        bool inA = true;
        for (int step = 0; step < nodes; step++)
        {
            if (inA) Rk4.Step(in sys, t, a, h, b, work);
            else Rk4.Step(in sys, t, b, h, a, work);
            inA = !inA;
            t += h;
        }
        Span<Dual> end = inA ? a : b;

        for (int i = 0; i < N; i++)
            if (!double.IsFinite(end[i].V))
                return result;
        if (end[6].V <= 0.0)
        {
            // Not a numerical failure: the vehicle has not the propellant for this
            // burn. Steeply nose-down burns need far more of it, so this is how the
            // infeasible end of the pitch range announces itself.
            result.OutOfPropellant = true;
            result.Infeasible = true;
            return result;
        }

        // Did the burn itself fly the vehicle into the ground? A steeply nose-down
        // burn will, and it is the other way the nose-down end of the pitch range
        // announces that it is out of reach.
        {
            Dual rEnd = Dual.Sqrt(end[0] * end[0] + end[1] * end[1] + end[2] * end[2]);
            if (rEnd.V <= sys.MeanRadius)
            {
                result.FlewIntoGround = true;
                result.Infeasible = true;
                return result;
            }
        }

        // --- the coast ---
        // The vehicle flips to retrograde at cutoff, taken as instantaneous, so the
        // coast is the alpha = 0 model the predictor already assumes.
        var coastSys = new DragCoastSystem
        {
            Mu = sys.Mu,
            OmegaZ = sys.OmegaZ,
            MeanRadius = sys.MeanRadius,
            AreaOverMass = sys.ReferenceArea / end[6].V,
            Alpha = 0.0,
            Table = sys.Table,
            Atmosphere = sys.Atmosphere,
        };

        Span<Dual> cs = stackalloc Dual[ImpactPredictor.N];
        for (int i = 0; i < 6; i++) cs[i] = end[i];
        ImpactPrediction p = ImpactPredictor.Predict(in coastSys, cs, in coastOpt,
                                                     coastScratch, default);
        if (!p.Hit)
        {
            // The powered arc was fine, so this is a statement about the trajectory it
            // produced - it never comes back down, or it started underground. Physical
            // either way.
            result.Infeasible = true;
            return result;
        }

        result.Valid = true;
        result.Fx = p.Fx; result.Fy = p.Fy; result.Fz = p.Fz;
        result.MissX = p.Fx - target[0];
        result.MissY = p.Fy - target[1];
        result.MissZ = p.Fz - target[2];
        result.CoastTime = p.TimeOfFlight.V;
        result.PropellantUsed = new Dual(mass0) - end[6];

        Dual rl = Dual.Sqrt(end[0] * end[0] + end[1] * end[1] + end[2] * end[2]);
        result.CutoffAltitude = rl - sys.MeanRadius;
        result.CutoffSpeed = Dual.Sqrt(end[3] * end[3] + end[4] * end[4] + end[5] * end[5]);

        // Resolve the miss into the two steerable directions at the TARGET: along the
        // ground track and across it. The third (radial) component is unreachable by
        // construction and is deliberately not part of the inner solve.
        double tl = System.Math.Sqrt(target[0] * target[0] + target[1] * target[1]
                                   + target[2] * target[2]);
        if (tl <= 0.0)
            return result;
        double nx = target[0] / tl, ny = target[1] / tl, nz = target[2] / tl;

        // Along-track at the target: the component of the site-to-vehicle direction
        // perpendicular to the radial. Built from the frame's retrograde axis, which
        // is in the orbit plane, so "downrange" means along the ground track.
        double ax2 = frame.Bx, ay2 = frame.By, az2 = frame.Bz;
        double d = ax2 * nx + ay2 * ny + az2 * nz;
        ax2 -= d * nx; ay2 -= d * ny; az2 -= d * nz;
        double al = System.Math.Sqrt(ax2 * ax2 + ay2 * ay2 + az2 * az2);
        if (al <= 1e-9)
            return result;
        ax2 /= al; ay2 /= al; az2 /= al;

        double cx2 = ny * az2 - nz * ay2;
        double cy2 = nz * ax2 - nx * az2;
        double cz2 = nx * ay2 - ny * ax2;

        result.MissDownrange = result.MissX * ax2 + result.MissY * ay2 + result.MissZ * az2;
        result.MissCrossrange = result.MissX * cx2 + result.MissY * cy2 + result.MissZ * cz2;
        return result;
    }

    /// <summary>Outcome of the inner solve.</summary>
    public struct InnerResult
    {
        public bool Converged;

        /// <summary>The solve stopped because the burn is impossible, not because it
        /// failed to converge. See ShotResult.Infeasible.</summary>
        public bool Infeasible;

        /// <summary>Infeasible specifically because the tanks ran dry.</summary>
        public bool OutOfPropellant;

        /// <summary>Infeasible specifically because the burn hit the ground.</summary>
        public bool FlewIntoGround;

        /// <summary>Infeasible because the solve wanted a longer burn than the vehicle
        /// has propellant for. This is the usual way a nose-down pitch fails: pointing
        /// down is a poor way to kill downrange velocity, so the burn it needs is
        /// enormous and the tanks run out first.</summary>
        public bool PropellantLimited;

        /// <summary>The iterate stopped moving without reaching the target. Reported
        /// separately from the physical failures because it is the one that means the
        /// SOLVER is at fault rather than the vehicle.</summary>
        public bool Stalled;

        public double YawDeg;
        public double Duration;
        public double MissM;
        public int Iterations;
        public ShotResult Shot;
    }

    /// <summary>
    /// INNER LOOP: with pitch and the turn rates held, find the yaw and duration that
    /// put the burn on the site.
    ///
    /// Two knobs, two residuals, so it is a plain 2x2 Newton - no pseudo-inverse and no
    /// minimum-norm choice, which is the point. Each iteration costs two seeded shots
    /// for the Jacobian columns plus the one that measured the residual.
    /// </summary>
    public static InnerResult SolveBurn(in PoweredBurnSystem template, in Frame frame,
                                        ReadOnlySpan<double> x0, double mass0,
                                        double pitchDeg, double pitchRateDegS,
                                        double yawRateDegS,
                                        double yawGuessDeg, double durationGuess,
                                        ReadOnlySpan<double> target,
                                        in ImpactOptions coastOpt, Span<Dual> scratch,
                                        double maxDuration = double.PositiveInfinity,
                                        double tolM = 25.0, int maxIter = 12)
    {
        var res = new InnerResult
        {
            YawDeg = yawGuessDeg,
            Duration = System.Math.Min(durationGuess, maxDuration),
        };
        double prevMiss = double.PositiveInfinity;
        int stalledFor = 0;

        Dual P = new Dual(pitchDeg * Deg);
        Dual PR = new Dual(pitchRateDegS * Deg);
        Dual YR = new Dual(yawRateDegS * Deg);

        for (int it = 0; it < maxIter; it++)
        {
            res.Iterations = it + 1;

            ShotResult f = Shoot(in template, in frame, x0, mass0,
                                 P, new Dual(res.YawDeg * Deg), PR, YR,
                                 new Dual(res.Duration), target, in coastOpt, scratch);
            if (!f.Valid)
            {
                res.Infeasible = f.Infeasible;
                res.OutOfPropellant = f.OutOfPropellant;
                res.FlewIntoGround = f.FlewIntoGround;
                return res;
            }

            res.Shot = f;
            double dr = f.MissDownrange.V, cr = f.MissCrossrange.V;
            res.MissM = System.Math.Sqrt(dr * dr + cr * cr);
            if (res.MissM < tolM)
            {
                res.Converged = true;
                return res;
            }

            // Not improving. Either the iterate is at the propellant bound and cannot
            // buy any more, or it has found a local minimum of the miss that is not
            // zero - both mean there is no solution here, and grinding out the
            // remaining iterations would only disguise that.
            if (res.MissM > 0.99 * prevMiss)
            {
                if (++stalledFor >= 3)
                {
                    bool atBound = res.Duration >= maxDuration - 1e-6;
                    res.PropellantLimited = atBound;

                    // A stall FAR from the target is a local minimum of the miss -
                    // there is no solution at this pitch, and no amount of iterating
                    // will find one. Only a stall CLOSE to the target is a convergence
                    // problem, and that is the one worth calling a solver fault.
                    bool nearTarget = res.MissM < 20.0 * tolM;
                    res.Stalled = nearTarget && !atBound;
                    res.Infeasible = atBound || !nearTarget;
                    return res;
                }
            }
            else stalledFor = 0;
            prevMiss = res.MissM;

            // Jacobian columns: seed yaw, then duration.
            ShotResult fy = Shoot(in template, in frame, x0, mass0,
                                  P, Dual.Seed(res.YawDeg * Deg), PR, YR,
                                  new Dual(res.Duration), target, in coastOpt, scratch);
            ShotResult ft = Shoot(in template, in frame, x0, mass0,
                                  P, new Dual(res.YawDeg * Deg), PR, YR,
                                  Dual.Seed(res.Duration), target, in coastOpt, scratch);
            if (!fy.Valid || !ft.Valid)
                return res;

            // [ d(down)/d(yaw)  d(down)/d(T) ] [ dyaw ]     [ down ]
            // [ d(cross)/d(yaw) d(cross)/d(T)] [ dT   ]  = -[ cross]
            double a11 = fy.MissDownrange.D, a12 = ft.MissDownrange.D;
            double a21 = fy.MissCrossrange.D, a22 = ft.MissCrossrange.D;
            double det = a11 * a22 - a12 * a21;
            if (!(System.Math.Abs(det) > 1e-12))
                return res;

            double dyaw = (-dr * a22 + cr * a12) / det;
            double dT = (-cr * a11 + dr * a21) / det;

            // Damped, because the first step from a poor guess can be enormous and the
            // linearisation is only local. Duration is also floored - a negative burn
            // is not a solution, it is a diverged iterate.
            double scale = 1.0;
            double yawStepDeg = dyaw / Deg;
            if (System.Math.Abs(yawStepDeg) > 20.0) scale = System.Math.Min(scale, 20.0 / System.Math.Abs(yawStepDeg));
            if (System.Math.Abs(dT) > 0.5 * System.Math.Max(res.Duration, 1.0))
                scale = System.Math.Min(scale, 0.5 * System.Math.Max(res.Duration, 1.0) / System.Math.Abs(dT));

            res.YawDeg += scale * yawStepDeg;
            res.Duration = System.Math.Clamp(res.Duration + scale * dT, 0.1, maxDuration);
        }
        return res;
    }

    /// <summary>Outcome of the whole solve.</summary>
    public struct SolveResult
    {
        public bool Converged;
        public BurnParameters Parameters;
        public double MissM;
        public double PropellantKg;
        public int Shots;
        public ShotResult Shot;
    }

    /// <summary>
    /// OUTER LOOP: choose the pitch and turn rates that need the SHORTEST burn.
    ///
    /// Every candidate is run through <see cref="SolveBurn"/> first, so the search only
    /// ever compares burns that already hit the site - it is choosing between right
    /// answers rather than looking for one. The objective is the duration the inner
    /// loop needed, which at fixed throttle is the propellant.
    ///
    /// A pattern search rather than a gradient method: the outer objective is the
    /// OUTPUT of an iterative solve, so its exact gradient needs the implicit function
    /// theorem applied across the inner loop, and that is worth doing only once the
    /// shape of the landscape is known. Coordinate steps with shrinking size are slow
    /// and completely robust, which is the right trade for the first version.
    ///
    /// <paramref name="minPitchDeg"/> is the flight-path-angle floor, and it enters
    /// here as a plain bound rather than as an override applied afterwards. If the
    /// unconstrained optimum already clears it, it never binds and costs nothing.
    /// </summary>
    public static SolveResult Solve(in PoweredBurnSystem template,
                                    ReadOnlySpan<double> x0, double mass0,
                                    ReadOnlySpan<double> target,
                                    in BurnParameters guess,
                                    in ImpactOptions coastOpt, Span<Dual> scratch,
                                    double minPitchDeg = -90.0,
                                    double maxDuration = double.PositiveInfinity,
                                    bool searchRates = true,
                                    int maxSweeps = 12,
                                    double initialPitchStepDeg = 8.0)
    {
        Frame frame = Frame.FromState(x0);
        var res = new SolveResult { Parameters = guess };

        double pitch = System.Math.Max(guess.PitchDeg, minPitchDeg);
        double pRate = guess.PitchRateDegS;
        double yRate = guess.YawRateDegS;
        double yawSeed = guess.YawDeg, durSeed = guess.Duration;
        int shots = 0;

        double best = Evaluate(in template, in frame, x0, mass0, pitch, pRate, yRate,
                               target, in coastOpt, scratch, maxDuration,
                               ref yawSeed, ref durSeed, ref shots, out InnerResult bestInner);
        if (!bestInner.Converged)
            return res;

        // The pattern search's first step should match how far the answer is expected
        // to have moved. Cold, that is the whole plausible range; warm - re-solving a
        // couple of seconds later from the last plan - it is a degree or two, and
        // starting at 8 degrees there wastes most of the search overshooting.
        double stepP = System.Math.Max(initialPitchStepDeg, 0.25), stepR = 0.25;
        for (int sweep = 0; sweep < maxSweeps; sweep++)
        {
            bool improved = false;

            for (int k = 0; k < 2; k++)
            {
                double sign = k == 0 ? 1.0 : -1.0;
                double trial = System.Math.Max(pitch + sign * stepP, minPitchDeg);
                if (trial == pitch) continue;
                double v = Evaluate(in template, in frame, x0, mass0, trial, pRate, yRate,
                                    target, in coastOpt, scratch, maxDuration,
                                    ref yawSeed, ref durSeed, ref shots, out InnerResult inner);
                if (v < best) { best = v; pitch = trial; bestInner = inner; improved = true; }
            }

            if (searchRates)
            {
                for (int k = 0; k < 2; k++)
                {
                    double sign = k == 0 ? 1.0 : -1.0;
                    double v = Evaluate(in template, in frame, x0, mass0, pitch,
                                        pRate + sign * stepR, yRate,
                                        target, in coastOpt, scratch, maxDuration,
                                        ref yawSeed, ref durSeed, ref shots, out InnerResult inner);
                    if (v < best) { best = v; pRate += sign * stepR; bestInner = inner; improved = true; }
                }
                for (int k = 0; k < 2; k++)
                {
                    double sign = k == 0 ? 1.0 : -1.0;
                    double v = Evaluate(in template, in frame, x0, mass0, pitch, pRate,
                                        yRate + sign * stepR,
                                        target, in coastOpt, scratch, maxDuration,
                                        ref yawSeed, ref durSeed, ref shots, out InnerResult inner);
                    if (v < best) { best = v; yRate += sign * stepR; bestInner = inner; improved = true; }
                }
            }

            if (!improved)
            {
                stepP *= 0.5; stepR *= 0.5;
                if (stepP < 0.25) break;
            }
        }

        res.Converged = bestInner.Converged;
        res.Parameters = new BurnParameters
        {
            PitchDeg = pitch,
            YawDeg = bestInner.YawDeg,
            PitchRateDegS = pRate,
            YawRateDegS = yRate,
            Duration = bestInner.Duration,
        };
        res.MissM = bestInner.MissM;
        res.PropellantKg = bestInner.Shot.PropellantUsed.V;
        res.Shot = bestInner.Shot;
        res.Shots = shots;
        return res;
    }

    /// <summary>One outer-loop evaluation: run the inner solve and report the burn time
    /// it needed. Not a local function because it has to take spans and `in` structs,
    /// which C# will not let a closure capture.</summary>
    private static double Evaluate(in PoweredBurnSystem template, in Frame frame,
                                   ReadOnlySpan<double> x0, double mass0,
                                   double pitchDeg, double pitchRate, double yawRate,
                                   ReadOnlySpan<double> target,
                                   in ImpactOptions coastOpt, Span<Dual> scratch,
                                   double maxDuration,
                                   ref double yawSeed, ref double durSeed, ref int shots,
                                   out InnerResult inner)
    {
        inner = SolveBurn(in template, in frame, x0, mass0, pitchDeg, pitchRate, yawRate,
                          yawSeed, durSeed, target, in coastOpt, scratch, maxDuration);
        shots += inner.Iterations * 3;
        if (!inner.Converged)
            return double.PositiveInfinity;

        // Warm-start the next inner solve from this one: neighbouring candidates have
        // near-identical answers, and this roughly halves the iteration count.
        yawSeed = inner.YawDeg;
        durSeed = inner.Duration;
        return inner.Duration;
    }
}