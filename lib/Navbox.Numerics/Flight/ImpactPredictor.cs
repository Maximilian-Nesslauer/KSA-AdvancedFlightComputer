using Navbox.Numerics;

namespace Navbox.Flight;

/// <summary>Why an impact prediction stopped.</summary>
public enum ImpactStatus
{
    /// <summary>Hit the target radius. Every field of the result is meaningful.</summary>
    Impact,

    /// <summary>Still above the surface when the time horizon ran out - in orbit, or
    /// on a trajectory that will take longer than the caller asked about.</summary>
    NoImpactWithinHorizon,

    /// <summary>Ran out of steps before either happening. Raise MaxSteps or the step
    /// sizes; the partial trajectory is still in the path buffer.</summary>
    StepLimit,

    /// <summary>Started at or below the target radius: there is nothing to predict.</summary>
    StartedBelowSurface,

    /// <summary>The state or the parameters were not finite.</summary>
    Invalid,
}

/// <summary>
/// Unpowered flight under gravity and drag, in a body-centred INERTIAL frame whose
/// +z is the body's spin axis. Six states: position then velocity.
///
///   rdot = v
///   vdot = -mu r / |r|^3  -  1/2 rho |v_air| Cd(M, alpha) (A/m) v_air
///   v_air = v - omega x r,        omega = (0, 0, OmegaZ)
///
/// THE ATMOSPHERE CO-ROTATES, which is why omega appears in a frame that is
/// otherwise inertial. KSA subtracts exactly this term before computing drag (see
/// PhysicsStates.ComputeDerivatives), and near the ground it is worth hundreds of
/// metres per second - an equatorial launch site moves at 465 m/s - so a predictor
/// that used inertial velocity would put the impact point tens of kilometres wrong.
///
/// ALPHA IS AN ASSUMPTION, NOT A STATE. This carries no attitude: it takes a fixed
/// angle of attack and holds it for the whole coast. <see cref="Alpha"/> defaults to
/// zero, which in the retrograde-first convention means engine into the wind - the
/// attitude a booster actually holds through boostback and entry. That is the same
/// simplification CALLISTO makes for its drag landing point, and it is what makes
/// the impact point a function of (r, v) alone rather than of the whole attitude
/// history. It is a real modelling choice and it is the first thing to revisit if a
/// prediction disagrees with what the vehicle does.
/// </summary>
public struct DragCoastSystem : IOdeSystem
{
    /// <summary>Gravitational parameter, m^3/s^2.</summary>
    public double Mu;

    /// <summary>Body rotation rate about the frame's +z, rad/s. Zero for a
    /// non-rotating body, which also removes the co-rotating-atmosphere term.</summary>
    public double OmegaZ;

    /// <summary>Body mean radius, m. The atmosphere's altitude datum - KSA measures
    /// altitude from the mean radius and never consults terrain, so this must be the
    /// mean radius even when the impact target radius is not.</summary>
    public double MeanRadius;

    /// <summary>Reference area over mass, m^2/kg. The area must be the one the Cd
    /// table is referenced to; KsaAeroSweep uses KSA's own nose face.</summary>
    public double AreaOverMass;

    /// <summary>Angle of attack held through the coast, radians, retrograde-first.
    /// Zero is engine-first. See the type summary.</summary>
    public double Alpha;

    /// <summary>Drag coefficient table. Null for a drag-free (Keplerian) coast.</summary>
    public AeroTable Table;

    /// <summary>Atmosphere. Null for an airless body; drag is then zero everywhere.</summary>
    public ExponentialAtmosphere Atmosphere;

    /// <summary>True if drag is modelled at all.</summary>
    public readonly bool HasDrag => Table != null && Atmosphere != null;

    public readonly void Derivative(Dual t, ReadOnlySpan<Dual> x, Span<Dual> dx)
    {
        Dual rx = x[0], ry = x[1], rz = x[2];
        Dual vx = x[3], vy = x[4], vz = x[5];

        // rdot = v
        dx[0] = vx;
        dx[1] = vy;
        dx[2] = vz;

        // Newtonian gravity. The floor keeps Sqrt's derivative finite at the origin,
        // which the integrator should never visit but a diverged iterate might.
        Dual r2 = rx * rx + ry * ry + rz * rz + 1e-12;
        Dual rlen = Dual.Sqrt(r2);
        Dual gk = -Mu / (r2 * rlen);

        dx[3] = gk * rx;
        dx[4] = gk * ry;
        dx[5] = gk * rz;

        if (!HasDrag)
            return;

        // Air-relative velocity: v - omega x r, with omega = (0, 0, w) so
        // omega x r = (-w*ry, w*rx, 0).
        Dual ax = vx + OmegaZ * ry;
        Dual ay = vy - OmegaZ * rx;
        Dual az = vz;

        Dual speed2 = ax * ax + ay * ay + az * az + 1e-18;
        Dual speed = Dual.Sqrt(speed2);

        Dual rho = Atmosphere.Density(rlen - MeanRadius);
        // Above the atmosphere Density returns a hard zero with a zero derivative, so
        // this short-circuits to no drag AND no drag sensitivity - which is correct,
        // not merely convenient: there is no air there to be sensitive to.
        if (rho.V == 0.0)
            return;

        Dual mach = speed / Atmosphere.SpeedOfSound;
        Dual cd = Table.Cd(mach, new Dual(Alpha));

        // a = -(1/2) rho |v| Cd (A/m) * v_air   -- one power of speed is in the vector
        Dual k = 0.5 * rho * speed * cd * AreaOverMass;
        dx[3] -= k * ax;
        dx[4] -= k * ay;
        dx[5] -= k * az;
    }
}

/// <summary>How far and how finely to integrate.</summary>
public struct ImpactOptions
{
    /// <summary>Radius counted as impact, m. Mean radius plus local terrain height if
    /// the caller knows it; mean radius otherwise.</summary>
    public double TargetRadius;

    /// <summary>Step size where drag is negligible, s. The motion there is smooth
    /// Keplerian and tolerates a coarse step; this is what keeps a prediction from
    /// apogee affordable.</summary>
    public double StepVacuum;

    /// <summary>Step size in the thick air low down, s. Drag varies over a scale
    /// height, so this wants to be small enough that a step crosses well under one.</summary>
    public double StepAir;

    /// <summary>Give up after this much flight time, s.</summary>
    public double MaxTime;

    /// <summary>Give up after this many steps, whatever the clock says.</summary>
    public int MaxSteps;

    /// <summary>Record one path point every this many steps. 0 records none.</summary>
    public int PathStride;

    /// <summary>Sensible defaults for a booster coasting to the ground.</summary>
    public static ImpactOptions Default(double targetRadius) => new()
    {
        TargetRadius = targetRadius,
        StepVacuum = 8.0,
        StepAir = 1.0,
        MaxTime = 3600.0,
        MaxSteps = 20000,
        PathStride = 4,
    };
}

/// <summary>Where and when the vehicle reaches the target radius.</summary>
public struct ImpactPrediction
{
    public ImpactStatus Status;

    /// <summary>Seconds from the initial state to impact.</summary>
    public Dual TimeOfFlight;

    /// <summary>Impact position in the same inertial frame the initial state was in.</summary>
    public Dual Rx, Ry, Rz;

    /// <summary>
    /// Impact position in the frame that CO-ROTATES with the body - the inertial
    /// point carried back by omega * (time of flight), so it is fixed to the ground.
    ///
    /// THIS IS THE ONE TO DIFFERENTIATE FOR TARGETING, and it is not just a rotation
    /// of the inertial answer. The rotation angle contains the time of flight, which
    /// is itself a function of the initial state, so
    ///
    ///   d(ground)/dv = R(-w t*) d(r_cci)/dv  +  dR/dt* (-w) r_cci dt*/dv
    ///
    /// and that second term is large: w |r| is 465 m/s at Earth's equator, and
    /// dt*/dv is order 0.1 s per m/s, so it is tens of metres of impact movement per
    /// m/s of velocity change - comparable to the first term and in a different
    /// direction. Computing the rotation OUTSIDE the Dual chain, on the .V parts,
    /// silently drops it. Hence these fields rather than a helper at the call site.
    /// </summary>
    public Dual Fx, Fy, Fz;

    /// <summary>Inertial velocity at impact.</summary>
    public Dual Vx, Vy, Vz;

    /// <summary>Integration steps taken.</summary>
    public int Steps;

    /// <summary>Path samples written to the caller's buffer, four doubles each.</summary>
    public int PathPoints;

    /// <summary>Lowest altitude above <see cref="ImpactOptions.TargetRadius"/> reached.
    /// Meaningful when the status is not Impact: it says how close it came.</summary>
    public double MinAltitude;

    public readonly bool Hit => Status == ImpactStatus.Impact;
}

/// <summary>
/// Integrates <see cref="DragCoastSystem"/> forward to the surface and reports where
/// it lands, carrying <see cref="Dual"/>s throughout so the answer can be
/// differentiated with respect to the initial state.
///
/// GETTING A JACOBIAN. Seed one component of x0 with <see cref="Dual.Seed"/>, leave
/// the other five as plain values, and call. Every field of the result then carries
/// its derivative with respect to that component in its <c>D</c> part - so six calls
/// give the full 7x6 sensitivity of (impact position, impact velocity, time of
/// flight) to the initial state. Nothing else has to change, because the integrator,
/// the atmosphere and the Cd table are all already Dual-valued end to end.
///
/// THAT INCLUDES THE STOPPING CONDITION, which is the part that usually is not
/// differentiable and the reason the crossing is refined the way it is below. The
/// impact point depends on the initial state both through the trajectory and through
/// WHEN it reaches the ground; a predictor that integrated to a fixed time and then
/// projected would silently drop the second term. The regula-falsi refinement is
/// built out of ordinary RK4 steps of Dual length, so d(time of flight)/d(x0) falls
/// out of the same sweep as everything else.
///
/// The step COUNT is a discrete quantity and has no derivative. That is the standard
/// situation for an event-terminated integration and it is harmless: the count is
/// locally constant in x0, so the derivative is exact everywhere except on the
/// measure-zero set where a step boundary coincides with the event.
/// </summary>
public static class ImpactPredictor
{
    /// <summary>State dimension: position and velocity.</summary>
    public const int N = 6;

    /// <summary>Newton iterations used to land exactly on the target radius. Newton
    /// is quadratic, so from a linearly interpolated start three is well past
    /// convergence in both the value and its derivative.</summary>
    private const int RefineIterations = 3;

    /// <summary>Scale heights above which the coarse step is used. See the note at
    /// the use site: the atmosphere's nominal TOP is far too high to switch on.</summary>
    private const double FineStepScaleHeights = 8.0;

    /// <summary>Scratch length <see cref="Predict"/> requires.</summary>
    public const int ScratchLength = (Rk4.ScratchPerState + 3) * N;

    /// <summary>
    /// Integrate to impact.
    /// </summary>
    /// <param name="sys">Dynamics, including the aero model and the atmosphere.</param>
    /// <param name="x0">Initial [r(3), v(3)], inertial. Seed a component to differentiate.</param>
    /// <param name="opt">Step sizes and limits.</param>
    /// <param name="scratch">At least <see cref="ScratchLength"/> Duals, caller-owned
    /// so that a per-frame prediction allocates nothing.</param>
    /// <param name="path">Optional buffer for the flown path, four doubles per
    /// sample: x, y, z and the TIME at which the vehicle is there. The time is not
    /// decoration - an impact overlay has to draw the path relative to the rotating
    /// ground it is going to hit, and that needs the age of each point. Pass an empty
    /// span to skip recording. Truncated silently if it fills.</param>
    public static ImpactPrediction Predict(in DragCoastSystem sys, ReadOnlySpan<Dual> x0,
                                           in ImpactOptions opt, Span<Dual> scratch,
                                           Span<double> path)
    {
        var result = new ImpactPrediction { Status = ImpactStatus.Invalid };

        if (x0.Length != N)
            throw new ArgumentException($"Expected {N} states, got {x0.Length}.", nameof(x0));
        if (scratch.Length < ScratchLength)
            throw new ArgumentException($"scratch must be at least {ScratchLength} long.",
                                        nameof(scratch));
        if (!(opt.TargetRadius > 0.0) || !(opt.StepVacuum > 0.0) || !(opt.StepAir > 0.0))
            return result;

        for (int i = 0; i < N; i++)
            if (!double.IsFinite(x0[i].V))
                return result;

        Span<Dual> work = scratch.Slice(0, Rk4.ScratchPerState * N);
        Span<Dual> x = scratch.Slice(Rk4.ScratchPerState * N, N);
        Span<Dual> xNext = scratch.Slice((Rk4.ScratchPerState + 1) * N, N);
        Span<Dual> xTrial = scratch.Slice((Rk4.ScratchPerState + 2) * N, N);

        x0.CopyTo(x);

        double r0 = Radius(x);
        if (r0 <= opt.TargetRadius)
        {
            result.Status = ImpactStatus.StartedBelowSurface;
            return result;
        }

        // WHERE THE FINE STEP STARTS. Not the atmosphere's top: KSA puts that where
        // density reaches 1e-9 kg/m^3, which is 167 km on Earth, so switching there
        // would put essentially every suborbital trajectory on the fine step for its
        // entire flight - which is exactly what it used to do, and what made a
        // prediction cost milliseconds it did not need to.
        //
        // Eight scale heights is where density is down by e^-8, about a three
        // thousandth of sea level. Above that the drag acceleration on a booster is
        // small AND slowly varying, which is the condition a coarse RK4 step wants;
        // below it, drag is the fastest thing in the dynamics.
        double fineAltitude = sys.Atmosphere != null
            ? FineStepScaleHeights * sys.Atmosphere.ScaleHeight
            : 0.0;
        double minAlt = r0 - opt.TargetRadius;
        int pathCapacity = path.Length / PathStrideDoubles;
        int pathCount = 0;

        if (pathCapacity > 0)
            WritePath(path, pathCount++, x, 0.0);

        Dual t = new Dual(0.0);
        int step = 0;

        while (true)
        {
            if (step >= opt.MaxSteps)
            {
                result.Status = ImpactStatus.StepLimit;
                break;
            }
            if (t.V >= opt.MaxTime)
            {
                result.Status = ImpactStatus.NoImpactWithinHorizon;
                break;
            }

            // Coarse above the air, fine inside it. The switch is on the CURRENT
            // altitude, so a step that enters the atmosphere is the last coarse one -
            // which is why the boundary is checked against the atmosphere top rather
            // than the target radius, giving a full step of margin.
            double alt = Radius(x) - sys.MeanRadius;
            double h = alt > fineAltitude ? opt.StepVacuum : opt.StepAir;
            if (t.V + h > opt.MaxTime)
                h = opt.MaxTime - t.V;

            Rk4.Step(in sys, t, x, new Dual(h), xNext, work);
            step++;

            if (!double.IsFinite(xNext[0].V) || !double.IsFinite(xNext[3].V))
            {
                result.Status = ImpactStatus.Invalid;
                break;
            }

            double rNext = Radius(xNext);
            minAlt = System.Math.Min(minAlt, rNext - opt.TargetRadius);

            if (rNext <= opt.TargetRadius)
            {
                // The step crossed the surface. Solve g(dt) = |x(t + dt)| - R = 0 for
                // the partial step that lands exactly on it, by NEWTON'S METHOD in
                // Dual arithmetic:
                //
                //     dt <- dt - g(dt) / (d|r|/dt),    d|r|/dt = (r . v) / |r|
                //
                // Newton rather than the bisection or regula falsi this obviously
                // wants, for a reason that is entirely about the derivative. A bracket
                // method's iterate is a SECANT estimate of the root, so differentiating
                // it gives a secant estimate of the root's sensitivity - converging,
                // but a good deal slower than the value, and silently short by a few
                // percent at any iteration count you would actually use.
                //
                // Newton in Duals converges the derivative EXACTLY, and does it in one
                // step from a converged value, whatever derivative the starting guess
                // carried. Differentiating dt' = dt - g/g_dot and using g -> 0 leaves
                // dt' = -(dg/dx0)/g_dot, which is the implicit function theorem - the
                // exact d(time of flight)/d(initial state). That is the term an
                // event-terminated integration usually loses, and losing it makes the
                // impact point's Jacobian wrong by more than a factor of two here.
                //
                // The bracket is still computed, purely as a guard: a grazing pass
                // makes r . v small and Newton's step unbounded, and there the
                // interpolated value is the safe answer.
                double gLo = Radius(x) - opt.TargetRadius;      // > 0
                double gHi = rNext - opt.TargetRadius;          // <= 0
                double frac = gLo / (gLo - gHi);
                if (!double.IsFinite(frac) || frac <= 0.0 || frac >= 1.0)
                    frac = 0.5;

                Dual hHit = new Dual(h * frac);

                for (int it = 0; it < RefineIterations; it++)
                {
                    Rk4.Step(in sys, t, x, hHit, xTrial, work);

                    Dual rx = xTrial[0], ry = xTrial[1], rz = xTrial[2];
                    Dual rlen = Dual.Sqrt(rx * rx + ry * ry + rz * rz + 1e-12);
                    Dual g = rlen - opt.TargetRadius;

                    // d|r|/dt along the trajectory. Negative on the way down, and the
                    // steeper the descent the better conditioned this is.
                    Dual rdot = (rx * xTrial[3] + ry * xTrial[4] + rz * xTrial[5]) / rlen;
                    if (System.Math.Abs(rdot.V) < 1e-6)
                        break;                      // grazing: keep the bracketed value

                    Dual next = hHit - g / rdot;
                    if (!double.IsFinite(next.V) || next.V < 0.0 || next.V > h)
                        break;                      // left the step: same fallback
                    hHit = next;
                }

                // Re-step so the returned state is the one the final dt produced.
                Rk4.Step(in sys, t, x, hHit, xTrial, work);

                result.Status = ImpactStatus.Impact;
                result.TimeOfFlight = t + hHit;
                result.Rx = xTrial[0]; result.Ry = xTrial[1]; result.Rz = xTrial[2];
                result.Vx = xTrial[3]; result.Vy = xTrial[4]; result.Vz = xTrial[5];

                // Into the co-rotating frame, IN DUAL, so the time-of-flight term in
                // the rotation angle survives. See ImpactPrediction.Fx.
                Dual ang = -sys.OmegaZ * result.TimeOfFlight;
                Dual ca = Dual.Cos(ang), sa = Dual.Sin(ang);
                result.Fx = result.Rx * ca - result.Ry * sa;
                result.Fy = result.Rx * sa + result.Ry * ca;
                result.Fz = result.Rz;
                if (pathCount < pathCapacity)
                    WritePath(path, pathCount++, xTrial, result.TimeOfFlight.V);
                break;
            }

            xNext.CopyTo(x);
            t += h;

            if (opt.PathStride > 0 && step % opt.PathStride == 0 && pathCount < pathCapacity)
                WritePath(path, pathCount++, x, t.V);
        }

        result.Steps = step;
        result.PathPoints = pathCount;
        result.MinAltitude = minAlt;
        return result;
    }

    /// <summary>
    /// The impact point's sensitivity to the INITIAL VELOCITY: three seeded sweeps,
    /// one per velocity component.
    ///
    /// This is the object a boostback steering law is built on. The law asks "which
    /// way should I push the velocity to move the landing point toward the target",
    /// and the answer is the inverse (or the pseudo-inverse, on the two surface
    /// directions that matter) of the 3x3 below. Jo and Ahn's closed-form IIP guidance
    /// calls these the d vectors and gets them analytically from a KEPLERIAN impact
    /// point; these are the same quantities for the DRAG-integrated one, which is the
    /// whole substitution the drag-aware version needs.
    ///
    /// Velocity only, not the full six columns, because that is what steering can
    /// change: thrust moves v, and r comes along for the ride. Three sweeps instead of
    /// six, so it costs half of a full Jacobian.
    ///
    /// PREFER dGroundDv over dInertialDv unless you specifically want the inertial
    /// answer. The target is a place on the ground, and the ground rotates during the
    /// flight - see <see cref="ImpactPrediction.Fx"/> for why the two differ by more
    /// than a rotation.
    /// </summary>
    /// <param name="x0">Nominal initial state, plain doubles - this seeds them itself.</param>
    /// <param name="dGroundDv">9 entries, row-major 3x3: [i*3+j] = d(ground_i)/d(v_j),
    /// m per m/s. Pass an empty span to skip.</param>
    /// <param name="dInertialDv">9 entries, same layout, for the inertial impact
    /// point. Pass an empty span to skip.</param>
    /// <param name="dTofDv">3 entries: d(time of flight)/d(v_j), s per m/s. Pass an
    /// empty span to skip.</param>
    /// <returns>The nominal prediction. If it did not hit, the Jacobians are not
    /// written and the status says why.</returns>
    public static ImpactPrediction VelocityJacobian(in DragCoastSystem sys,
                                                    ReadOnlySpan<double> x0,
                                                    in ImpactOptions opt, Span<Dual> scratch,
                                                    Span<double> dGroundDv,
                                                    Span<double> dInertialDv,
                                                    Span<double> dTofDv)
    {
        if (x0.Length != N)
            throw new ArgumentException($"Expected {N} states, got {x0.Length}.", nameof(x0));

        Span<Dual> x = stackalloc Dual[N];
        var nominal = default(ImpactPrediction);

        for (int j = 0; j < 3; j++)
        {
            // Seed one velocity component; everything else, including the whole
            // position, is a constant of this sweep.
            for (int i = 0; i < N; i++)
                x[i] = i == 3 + j ? Dual.Seed(x0[i]) : new Dual(x0[i]);

            ImpactPrediction p = Predict(in sys, x, in opt, scratch, default);
            if (j == 0)
                nominal = p;
            if (!p.Hit)
                return p;

            if (dGroundDv.Length >= 9)
            {
                dGroundDv[0 * 3 + j] = p.Fx.D;
                dGroundDv[1 * 3 + j] = p.Fy.D;
                dGroundDv[2 * 3 + j] = p.Fz.D;
            }
            if (dInertialDv.Length >= 9)
            {
                dInertialDv[0 * 3 + j] = p.Rx.D;
                dInertialDv[1 * 3 + j] = p.Ry.D;
                dInertialDv[2 * 3 + j] = p.Rz.D;
            }
            if (dTofDv.Length >= 3)
                dTofDv[j] = p.TimeOfFlight.D;
        }

        return nominal;
    }

    private static double Radius(ReadOnlySpan<Dual> x)
        => System.Math.Sqrt(x[0].V * x[0].V + x[1].V * x[1].V + x[2].V * x[2].V);

    /// <summary>Doubles per recorded path sample: position then time.</summary>
    public const int PathStrideDoubles = 4;

    private static void WritePath(Span<double> path, int index, ReadOnlySpan<Dual> x, double t)
    {
        path[index * PathStrideDoubles + 0] = x[0].V;
        path[index * PathStrideDoubles + 1] = x[1].V;
        path[index * PathStrideDoubles + 2] = x[2].V;
        path[index * PathStrideDoubles + 3] = t;
    }
}
