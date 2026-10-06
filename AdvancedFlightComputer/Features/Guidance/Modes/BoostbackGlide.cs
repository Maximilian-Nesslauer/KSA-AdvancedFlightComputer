#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Numerics;

// The glide: from boostback cutoff to the ground, a PID on the coast impact point steers the angle of attack.
//
// WHAT IS STEERED ON. The prediction the overlay draws - where the vehicle lands if it coasts engine-first into the wind (alpha 0) from here, the 0% throttle impact point - and its miss from the site, a vector on the ground.
// Flying a non-zero alpha moves the vehicle off the path that prediction assumes, which is the point: the lift walks the trajectory, the next prediction sees where it has walked to, and the loop closes on that. As the miss goes to zero the command goes back to alpha 0 and the prediction comes true.
//
// WHAT THE PID ASKS FOR IS A RATE: the impact should move back towards the site at (Kp miss + Ki integral + Kd rate) / tgo, with tgo the prediction's own time to impact. Which lift does that is not assumed but MEASURED: one velocity Jacobian of the coast prediction says how far the impact moves per m/s of push along each lift axis, and a damped two-by-two least squares picks the lift that moves it at the asked-for rate. The Jacobian grows like the time to go, so the lift asked for is Kp miss / tgo^2 - the zero-effort-miss guidance law, Kp 3 its textbook gain - and one set of gains is right from cutoff to the ground.
//  Assuming instead that lift up stretches the impact and lift left moves it left is right for a vehicle that is descending, and wrong for one still climbing after its boostback, where "up" off a steep climb points mostly backwards and pulls the impact in. That version pushed the impact a kilometre the wrong way for as long as the climb lasted, and then had to spend the glide's range winning it back.
//
// THE ACCELERATION BECOMES AN ANGLE THROUGH THE MEASURED LIFT CURVE. Lift coefficient = a m / (q A), and the AoA that makes it is read off KsaAeroSweep's C_L(Mach, alpha) at the current Mach. That curve is not linear - crossflow lift grows with the square of the angle, and the tilted axial force works against it - and commanding degrees straight from a PID left the miss stalled in exactly the thick air where the authority is; inverting the curve gives the loop the lift it asked for, whatever shape the active model's curve has. Should a model have a dead band near alpha 0, tiny requests ramp the angle up from zero rather than jumping to its edge, so the command stays continuous as the miss crosses zero.
//
// AND WHICH WAY TO TILT FOR IT IS MEASURED TOO. A slender body leading with its engine lifts towards the side the engine end is tilted, from attached flow and crossflow along its flanks; a capsule leading with its heat shield lifts the other way, from its tilted axial force; stock's box makes none at all. The sign comes off the same table every update, so the loop points the right way under whatever aero is active - and holds alpha 0 under a model with no lift, rather than wagging the vehicle for drag it cannot aim. It is a property of the shape and the Mach number, never of the air's density, so thin air does not read as "no lift".
//
// STEERING STARTS AT CUTOFF, in any air at all. A dispersion is cheapest to fix early, while there is the most time for a small push to grow; up high the lift asked for just exceeds what the thin air gives, so the command sits at the AoA limit and does what it can until the air thickens.
//
// The two lift axes are perpendicular to the airflow, one in the vertical plane and one across it, built from the ground track - carried through a near-vertical fall from the last heading that had one, so the basis never degenerates. Any basis would do; the Jacobian decides what each one is worth.
//
// AND THE VEHICLE FLIES WHAT IS ASKED. The airflow turns as the trajectory bends, and an attitude loop lagging a turning target flies a steady degree or two of angle of attack nobody commanded - lift that carried the impact past the site before the guidance noticed. A trim integrates the commanded tilt against the tilt actually flown, measured off the thrust axis every step; see GlideTrackGain.
//
// THE INTEGRAL DEFAULTS OFF. A glide is minutes long and the miss starts kilometres out, so the integral of an ordinary, shrinking miss grows into a large stale bias that then drives the vehicle past the site. Kd bought nothing either - the overshoot it was meant to damp turned out to be the attitude lag, which the trim removes. Both are left on the panel to tune against the real thing.
//
// Closed-loop flat-earth runs through KSAero's aero, from descending and from still-climbing cutoffs between 40 and 90 km, boosters of fineness 6 to 13 and a capsule, with up to 5 s of attitude lag: every case within 2 m of the site, at most 11 m of wrong-way motion and 4 m of overshoot.
public static partial class GuidanceWindow
{
    /// <summary>Floor on the time to go the gains are divided by, s, so the last seconds ask for the airframe's limit rather than for infinity.</summary>
    private const double GlideMinTgo = 5.0;

    /// <summary>Low-pass on the miss rate the derivative term uses, s. The prediction steps at its own recompute rate, so the raw difference is a spike train.</summary>
    private const double GlideRateTau = 1.0;

    /// <summary>Least lift-to-drag at the maximum AoA that counts as lift worth steering with. Low on purpose: a little lift early is worth more than a lot of it late.</summary>
    private const double GlideMinLiftToDrag = 0.02;

    /// <summary>Requests below this fraction of the maximum lift ramp the angle linearly from zero, keeping the command continuous through the dead band near alpha 0.</summary>
    private const double GlideLiftRamp = 0.02;

    /// <summary>Horizontal share of the airflow below which the ground track is carried over from the last step that had one.</summary>
    private const double GlideHeadingMinHoriz = 0.02;

    /// <summary>Resolution of the lift-curve inversion, degrees.</summary>
    private const double GlideAoaStepDeg = 0.25;

    /// <summary>How often the glide re-solves, ms - one velocity Jacobian of the coast prediction each time, about 3.6 ms, the same cadence the burn steers at.</summary>
    private const long GlideIntervalMs = 100;

    /// <summary>Tikhonov damping of the lift solve, as a fraction of the lift axes' mean squared authority. Keeps a direction the lift can barely move the impact along from asking for unbounded lift.</summary>
    private const double GlideSolveDamping = 1e-3;

    /// <summary>
    /// Integral gain of the attitude trim, 1/s: how fast the difference between the tilt asked for and the tilt flown is wound into the command.
    ///  The airflow turns as the trajectory bends - half a degree a second through the upper atmosphere - and an attitude loop that lags a turning target by a second or two flies a steady degree or two off it. That is real angle of attack nobody asked for, and its lift carried the impact past the site before the guidance could take it back. Trimming on the MEASURED tilt makes the vehicle fly what the guidance asked for, whatever the lag. 0.3 to 1 all worked in the closed-loop runs; with a 5 s lag the overshoot went from 570 m to 4 m.
    /// </summary>
    private const double GlideTrackGain = 0.5;

    /// <summary>Sim seconds between glide lines in the game log.</summary>
    private const double GlideLogIntervalS = 2.0;

    /// <summary>The trim only integrates while the flown tilt is within this of the asked one, degrees: a steady lag of a degree or two, not a manoeuvre still in progress.</summary>
    private const double GlideTrimMaxErrorDeg = 5.0;

    private static void ResetGlide()
    {
        _s.GlideIntCcf = _s.GlideRateCcf = _s.GlidePrevMissCcf = double3.Zero;
        _s.GlideMissDownM = _s.GlideMissCrossM = 0.0;
        _s.GlidePrevValid = false;
        _s.GlideTiltUpDeg = _s.GlideTiltLeftDeg = _s.GlideAoaDeg = 0.0;
        _s.GlideTrimUpDeg = _s.GlideTrimLeftDeg = _s.GlideFlownAoaDeg = 0.0;
        _s.GlideLiftSign = 0;
        _s.GlideQ = _s.GlideAccelCmd = 0.0;
        _s.GlideTgo = double.NaN;
        _s.GlideHeadingValid = false;
        _s.GlideLastUpdate = double.NaN;
        _s.GlideLogTime = double.NaN;
        _s.GlideTick = 0;
        _s.GlideStatus = "";
    }

    /// <summary>
    /// Where the thrust axis (body +x) points in the glide, CCI: surface retrograde, tilted by the commanded angle of attack.
    /// </summary>
    private static double3 GlideDirection(Vehicle vehicle, Orbit orbit, IParentBody parent, double now, double dt)
    {
        double3 retro = SurfaceRetrogradeCci(orbit, parent);
        double3 r = orbit.StateVectors.PositionCci;
        double3 vAir = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        double speed = vAir.Length();
        if (!(speed > 1.0))
            return retro;
        double3 vHat = vAir / speed;
        double3 rHat = r.NormalizeOrZero();

        // The ground track, held over from the last step that had one when the vehicle is falling (near) straight down.
        double3 horiz = vHat - double3.Dot(vHat, rHat) * rHat;
        if (horiz.Length() > GlideHeadingMinHoriz)
        {
            _s.GlideHeadingCci = double3.Normalize(horiz);
            _s.GlideHeadingValid = true;
        }
        if (!_s.GlideHeadingValid)
        {
            _s.GlideStatus = "No ground track yet - holding alpha 0.";
            return retro;
        }
        double3 heading = (_s.GlideHeadingCci - double3.Dot(_s.GlideHeadingCci, rHat) * rHat).NormalizeOrZero();
        double3 left = double3.Cross(rHat, heading).NormalizeOrZero();
        double3 up = double3.Cross(vHat, left).NormalizeOrZero();
        if (left.Length() < 0.5 || up.Length() < 0.5)
            return retro;

        // Re-solve on its own cadence. Between solves the command is held, and the slew smooths the steps.
        long tick = Environment.TickCount64;
        if (tick - _s.GlideTick >= GlideIntervalMs)
        {
            _s.GlideTick = tick;
            UpdateGlidePid(vehicle, orbit, parent, heading, up, left, speed, now);
        }

        // THE ATTITUDE TRIM, every step: the tilt actually flown, measured off the thrust axis, against the tilt asked for, and the difference wound into the command. See GlideTrackGain.
        const double RadToDeg = 180.0 / Math.PI;
        double3 engine = -ThrustAxisCci(vehicle);
        double3 flownAcross = engine - double3.Dot(engine, vHat) * vHat;
        double flownUp = double3.Dot(flownAcross, up) * RadToDeg;
        double flownLeft = double3.Dot(flownAcross, left) * RadToDeg;
        _s.GlideFlownAoaDeg = Math.Acos(Math.Clamp(double3.Dot(engine.NormalizeOrZero(), vHat), -1.0, 1.0)) * RadToDeg;

        double maxAoa = Math.Clamp(_s.GlideMaxAoaDeg, 0.0, 60.0);
        double sendUp = _s.GlideTiltUpDeg + _s.GlideTrimUpDeg;
        double sendLeft = _s.GlideTiltLeftDeg + _s.GlideTrimLeftDeg;
        double sendSize = Math.Sqrt(sendUp * sendUp + sendLeft * sendLeft);
        bool sendLimited = sendSize > maxAoa;
        if (sendLimited)
        {
            sendUp *= maxAoa / sendSize;
            sendLeft *= maxAoa / sendSize;
            sendSize = maxAoa;
        }

        // Anti-windup as in the PID: a trim that is already being clipped only ever unwinds.
        // Only on a steady lag, never through a manoeuvre: during the turn from the burn attitude after cutoff the vehicle is tens of degrees off the command for seconds, and integrating that wound the trim to its limit in under a second and threw the attitude past retrograde when the turn ended.
        double errUp = _s.GlideTiltUpDeg - flownUp;
        double errLeft = _s.GlideTiltLeftDeg - flownLeft;
        bool steady = Math.Sqrt(errUp * errUp + errLeft * errLeft) <= GlideTrimMaxErrorDeg;
        if (steady && (!sendLimited || errUp * _s.GlideTrimUpDeg < 0.0))
            _s.GlideTrimUpDeg = Math.Clamp(_s.GlideTrimUpDeg + GlideTrackGain * errUp * dt, -maxAoa, maxAoa);
        if (steady && (!sendLimited || errLeft * _s.GlideTrimLeftDeg < 0.0))
            _s.GlideTrimLeftDeg = Math.Clamp(_s.GlideTrimLeftDeg + GlideTrackGain * errLeft * dt, -maxAoa, maxAoa);

        // THE FLIGHT LOG, every GlideLogIntervalS of sim time: what was asked against what was flown, so a glide that misbehaves in the game can be read back afterwards. The two tilts are in the same lift-up / lift-left axes; a flown tilt that does not follow the command is the attitude loop, not the guidance.
        if (!double.IsFinite(_s.GlideLogTime) || now - _s.GlideLogTime >= GlideLogIntervalS)
        {
            _s.GlideLogTime = now;
            KsaAeroSweep.Result logAero = _s.Aero;
            double logMach = logAero?.Atmosphere?.Mach(speed) ?? double.NaN;
            double tableMach = logAero?.Table != null ? GlideTableMach(logAero, speed) : double.NaN;
            GuidanceLog.Info(vehicle, string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "glide: alt {0:F1} km, q {1:F2} kPa, M {2:F2}, tgo {3:F0} s, miss {4:F0} m long / {5:F0} m left; "
                + "tilt up/left asked {6:F1}/{7:F1}, flown {8:F1}/{9:F1}, trim {10:F1}/{11:F1} deg; lift sign {12}, "
                + "table Cd(a0) {13:F3} CL(a{14:F0}) {15:F3}; {16}",
                (r.Length() - parent.MeanRadius) / 1000.0, _s.GlideQ / 1000.0, logMach, _s.GlideTgo,
                _s.GlideMissDownM, _s.GlideMissCrossM,
                _s.GlideTiltUpDeg, _s.GlideTiltLeftDeg, flownUp, flownLeft, _s.GlideTrimUpDeg, _s.GlideTrimLeftDeg,
                _s.GlideLiftSign,
                logAero?.Table?.Cd(tableMach, 0.0) ?? double.NaN, maxAoa, logAero?.LiftAt(tableMach, maxAoa / RadToDeg) ?? double.NaN,
                _s.GlideStatus.Length > 0 ? _s.GlideStatus : "ok"));
        }

        double aoa = sendSize / RadToDeg;
        double3 tilt = (sendUp * up + sendLeft * left).NormalizeOrZero();
        if (!(aoa > 0.0) || tilt.Length() < 0.5)
            return retro;

        // The engine end leads into the airflow, tilted off it; the thrust axis points the other way down the body.
        return -(Math.Cos(aoa) * vHat + Math.Sin(aoa) * tilt);
    }

    private static void UpdateGlidePid(Vehicle vehicle, Orbit orbit, IParentBody parent, double3 headingCci,
                                       double3 liftUpCci, double3 liftLeftCci, double speed, double now)
    {
        double dt = double.IsFinite(_s.GlideLastUpdate) ? Math.Clamp(now - _s.GlideLastUpdate, 0.0, 1.0) : 0.0;
        _s.GlideLastUpdate = now;

        // WHICH WAY THIS AIRFRAME LIFTS, read off the measured table at the AoA the glide may reach for. A property of the shape and the Mach number, not of how thick the air is, so it is read whatever q is.
        KsaAeroSweep.Result aero = _s.Aero;
        double mass = vehicle.TotalMass;
        if (aero?.Table == null || !(mass > 0.0) || !(aero.ReferenceArea > 0.0))
        {
            HoldGlide("No aero surrogate - holding alpha 0.");
            return;
        }
        double maxAoa = Math.Clamp(_s.GlideMaxAoaDeg, 0.0, 60.0);
        double maxAoaRad = maxAoa * Math.PI / 180.0;
        double mach = GlideTableMach(aero, speed);
        double clMaxAoa = aero.LiftAt(mach, maxAoaRad);
        double cdMaxAoa = aero.Table.Cd(mach, maxAoaRad);
        _s.GlideLiftSign = cdMaxAoa > 0.0 && Math.Abs(clMaxAoa / cdMaxAoa) >= GlideMinLiftToDrag ? Math.Sign(clMaxAoa) : 0;
        if (_s.GlideLiftSign == 0 || !(maxAoa > 0.0))
        {
            _s.GlideIntCcf = double3.Zero;
            HoldGlide("The active aero model makes no lift to steer with - holding alpha 0.");
            return;
        }

        // STEER IN ANY AIR AT ALL. A dispersion is cheapest to fix early, while there is the most time for a small push to grow into a large shift; up high the lift asked for simply exceeds what the thin air gives, so the command sits at the AoA limit and does what it can. Only above the atmosphere, where no angle buys anything, is there nothing to do.
        double q = 0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * speed * speed;
        _s.GlideQ = q;
        if (!(q > 0.0))
        {
            HoldGlide("Above the atmosphere - holding alpha 0.");
            return;
        }

        // THE MISS, AND WHAT LIFT DOES TO IT, from one velocity Jacobian of the coast prediction - the same object the burn steers on. Aimed at the site's own radius, as UpdateSteering does, so the miss is distance along the ground and not a difference of terrain heights.
        double siteRadius = parent.MeanRadius + SiteTerrainHeight(parent);
        var sys = new DragCoastSystem
        {
            Mu = parent.Mu,
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            AreaOverMass = aero.ReferenceArea / mass,
            Alpha = 0.0,
            Table = aero.Table,
            Atmosphere = aero.Atmosphere,
        };
        var opt = ImpactOptions.Default(parent.MeanRadius);
        opt.MaxTime = ImpactHorizonMinutes * 60.0;
        opt.PathStride = 0;
        opt.TargetRadius = siteRadius;

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;
        Span<double> x0 = stackalloc double[ImpactPredictor.N] { r0.X, r0.Y, r0.Z, v0.X, v0.Y, v0.Z };
        _s.ImpactScratch ??= new Dual[ImpactPredictor.ScratchLength];
        Span<double> dG = stackalloc double[9];
        ImpactPrediction nom = ImpactPredictor.VelocityJacobian(sys, x0, opt, _s.ImpactScratch, dG, default, default);
        if (!nom.Hit)
        {
            HoldGlide("No impact within the prediction horizon - holding alpha 0.");
            return;
        }

        // Everything in the body-fixed frame, on the ground plane at the site. The Jacobian's rows are in the co-rotating frame - the current body-fixed frame turned back into CCI axes - so the current CCI-to-CCF rotation carries them, and the predicted hit, into it.
        doubleQuat toCcf = parent.GetCci2Ccf();
        double3 siteUp = SiteDirCcf();
        double3 hitCcf = new double3(nom.Fx.V, nom.Fy.V, nom.Fz.V).Transform(toCcf);
        double3 miss = OnGround(hitCcf - siteUp * siteRadius, siteUp);

        // HOW FAR THE IMPACT MOVES PER m/s OF PUSH ALONG EACH LIFT AXIS. Measured, not assumed: lift "up" stretches the impact while descending, but climbing steeply after a boostback it points mostly backwards and can pull the impact in, and the planet turning under a long coast couples the two axes. Steering on the geometric assumption pushed the impact the wrong way for as long as the vehicle was still climbing.
        double3 perUp = OnGround(ApplyJacobian(dG, liftUpCci).Transform(toCcf), siteUp);
        double3 perLeft = OnGround(ApplyJacobian(dG, liftLeftCci).Transform(toCcf), siteUp);

        // The miss split along and across the ground track, for the readout only.
        double3 headingCcf = OnGround(headingCci.Transform(toCcf), siteUp).NormalizeOrZero();
        _s.GlideMissDownM = double3.Dot(miss, headingCcf);
        _s.GlideMissCrossM = double3.Dot(miss, double3.Cross(siteUp, headingCcf));

        if (_s.GlidePrevValid && dt > 1e-6)
            _s.GlideRateCcf += ((miss - _s.GlidePrevMissCcf) / dt - _s.GlideRateCcf) * Blend(GlideRateTau, dt);
        _s.GlidePrevMissCcf = miss;
        _s.GlidePrevValid = true;

        // THE PID, as the rate the impact should move at: (Kp miss + Ki integral + Kd rate) / tgo, back towards the site. With the Jacobian scaling like the time to go, the lift this asks for is the zero-effort-miss law's Kp miss / tgo^2.
        double tgo = Math.Max(nom.TimeOfFlight.V, GlideMinTgo);
        _s.GlideTgo = tgo;
        double3 pid = _s.GlideKp * miss + _s.GlideKi * _s.GlideIntCcf + _s.GlideKd * _s.GlideRateCcf;
        SolveGlideLift(perUp, perLeft, -pid / tgo, out double aUp, out double aLeft);
        double accel = Math.Sqrt(aUp * aUp + aLeft * aLeft);
        _s.GlideAccelCmd = accel;

        double clWanted = accel * mass / (q * aero.ReferenceArea);
        double alphaDeg = GlideAoaForLift(aero, mach, _s.GlideLiftSign, clWanted, maxAoa, out bool saturated);

        // Anti-windup: integrate only while the command has room, or where the integral is unwinding.
        if (!saturated || double3.Dot(miss, _s.GlideIntCcf) < 0.0)
            _s.GlideIntCcf += miss * dt;

        // The engine end tilts towards the lift for a body that lifts that way, away from it for one that lifts the other.
        double nUp = accel > 0.0 ? aUp / accel : 0.0;
        double nLeft = accel > 0.0 ? aLeft / accel : 0.0;
        _s.GlideTiltUpDeg = _s.GlideLiftSign * nUp * alphaDeg;
        _s.GlideTiltLeftDeg = _s.GlideLiftSign * nLeft * alphaDeg;
        _s.GlideAoaDeg = alphaDeg;
        _s.GlideStatus = saturated ? $"AoA limited to {maxAoa:F0} deg - asking for more lift than it has." : "";
    }

    /// <summary>
    /// The smallest AoA at which the measured lift curve gives the lift coefficient asked for, at this Mach, in degrees. Saturated when even the maximum AoA does not.
    /// </summary>
    private static double GlideAoaForLift(KsaAeroSweep.Result aero, double mach, int sign, double clWanted,
                                          double maxAoaDeg, out bool saturated)
    {
        const double DegToRad = Math.PI / 180.0;
        double clMax = sign * aero.LiftAt(mach, maxAoaDeg * DegToRad);
        double floor = GlideLiftRamp * clMax;
        double target = Math.Max(clWanted, floor);
        saturated = true;
        if (target >= clMax)
            return maxAoaDeg;

        double prevAoa = 0.0, prevCl = 0.0;
        for (double aoa = GlideAoaStepDeg; aoa <= maxAoaDeg + 1e-9; aoa += GlideAoaStepDeg)
        {
            double cl = sign * aero.LiftAt(mach, aoa * DegToRad);
            if (cl >= target)
            {
                saturated = false;
                double t = cl > prevCl ? Math.Clamp((target - prevCl) / (cl - prevCl), 0.0, 1.0) : 1.0;
                double found = prevAoa + t * (aoa - prevAoa);
                return clWanted < floor ? found * Math.Max(clWanted, 0.0) / floor : found;
            }
            prevAoa = aoa;
            prevCl = cl;
        }
        return maxAoaDeg;
    }

    /// <summary>
    /// The Mach number to read the measured tables at: the flight's own, held inside the range that was measured. Past the last breakpoint the fit extrapolates along its end slope, which on a lift curve can run all the way through zero and flip the sign the glide steers by; coefficients level off at high Mach, so the last measured value is the honest stand-in.
    /// </summary>
    private static double GlideTableMach(KsaAeroSweep.Result aero, double speed)
    {
        double mach = aero.Atmosphere?.Mach(speed) ?? 0.0;
        return Math.Clamp(mach, aero.Table.MachMin, aero.Table.MachMax);
    }

    /// <summary>The impact's movement for a velocity change: dG is row-major 3x3, [i*3+j] = d(ground_i)/d(v_j).</summary>
    private static double3 ApplyJacobian(ReadOnlySpan<double> dG, double3 dv)
        => new double3(dG[0] * dv.X + dG[1] * dv.Y + dG[2] * dv.Z,
                       dG[3] * dv.X + dG[4] * dv.Y + dG[5] * dv.Z,
                       dG[6] * dv.X + dG[7] * dv.Y + dG[8] * dv.Z);

    /// <summary>A vector with its component along the site's up removed: a distance along the ground.</summary>
    private static double3 OnGround(double3 v, double3 siteUp) => v - double3.Dot(v, siteUp) * siteUp;

    /// <summary>
    /// The lift, as accelerations along the two lift axes, that moves the impact at the wanted rate: [perUp perLeft] a = want in least squares, damped.
    /// </summary>
    private static void SolveGlideLift(double3 perUp, double3 perLeft, double3 want, out double aUp, out double aLeft)
    {
        double m11 = double3.Dot(perUp, perUp);
        double m12 = double3.Dot(perUp, perLeft);
        double m22 = double3.Dot(perLeft, perLeft);
        double damp = GlideSolveDamping * (m11 + m22);
        m11 += damp;
        m22 += damp;
        double r1 = double3.Dot(perUp, want);
        double r2 = double3.Dot(perLeft, want);
        double det = m11 * m22 - m12 * m12;
        if (!(det > 0.0))
        {
            aUp = aLeft = 0.0;
            return;
        }
        aUp = (m22 * r1 - m12 * r2) / det;
        aLeft = (m11 * r2 - m12 * r1) / det;
    }

    private static void HoldGlide(string why)
    {
        _s.GlideTiltUpDeg = _s.GlideTiltLeftDeg = _s.GlideAoaDeg = 0.0;
        _s.GlideAccelCmd = 0.0;
        _s.GlideStatus = why;
    }
}
