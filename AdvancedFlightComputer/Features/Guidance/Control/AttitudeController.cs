#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// AFC's own attitude controller: holds a target attitude and rate with a torque that knows the air, and spends it on the craft's control surfaces first and its RCS for the rest, in place of the stock flight computer's attitude hold.
///
/// WHY. Entry guidance pointed the craft through the stock flight computer, which knows nothing of the air's moment: it nulls the error the air keeps making, so on a booster gliding at a few degrees off the airflow it fires its RCS every way, all the time (2026-11-20), and holds a standing error that entry's trim then winds out.
///
/// THE LAW, in KSA body axes:
///     tau = I kd (w_capture + (w_target - w)_filtered)  +  I ki wn^2 integral(theta)  +  w x I w  -  M_air,   kd = 2 zeta wn
///  - w_capture = (wn / 2 zeta) theta, capped at CaptureRateDegS: unsaturated it is the PD's kp theta; a large error (the glide's first step, 73 deg off in roll on 2026-11-20) is closed at a set rate rather than at the RCS's full thrust.
///  - (w_target - w)_filtered: the rate error through a first-order lag of RateFilterS. On the flown booster the rate error read 0.5 deg/s of wobble, about 10 kN m of demand jittering every step through its inertia, which the fins chased (2026-11-20).
///  - integral(theta): the error integrated while within IntegralWithinDeg, capped at IntegralMaxDegS, at ki = IntegralPerS: it learns what the feedforward from the tables misses (17 % of the air's moment on that flight), so the error settles inside the deadband instead of standing at a degree. It keeps acting in the deadband.
///  - theta: the small rotation that takes the body onto the target, from their axes; w and w_target the body's rate and the target's turning rate.
///  - M_air: the air's moment on the craft as it flies now, from the sweep's body moment table at the Mach, alpha and beta flown, moved to the present centre of mass and divided by the game's moment limiter (BodyAero6Dof.MomentLimiterPerPa). Cancelled before any error builds: the feedforward. None outside the table (20 deg off engine first).
///  - I: the inertia about the model's axes (Ksa6DofSetup.Inertia), and w x I w its gyroscopic part.
///
/// THE SPLIT, in one programme. The control surfaces and the RCS thrusters share the demand (KsaAeroTorqueControl.Command with the RCS as its backup): a degree of fin costs next to nothing, a thruster its propellant, so the fins take every axis they can and the RCS only what they cannot - roll included, where a fixed "fins first, roll last" order had left roll to the RCS. Each fin is asked only for what its rate reaches by the next step, from where it is, and pays to move, so it does not chatter. The RCS is then fired for what the fins will give short of the demand (KsaRcsTorqueControl, pitch and yaw before roll). The engine is not used: this flies a craft whose engine is off.
///
/// THE DEADBAND (Settings.DeadbandDeg, off by default). Inside it, turning at under RateDeadbandDegS from the target's rate, the feedback stands down and only the feedforward and the integral fly; it comes back past the deadband and stands down again under half of it. Off since the integral and the filtered rate went in: each return of the feedback kicked, and in the harness a 0.5 deg deadband cost 50 kg of RCS holding a tilt where none cost 46, and 1 deg cost 234 (2026-11-20).
///
/// Main thread: it reads the craft and commands its actuators.
/// </summary>
internal static class AttitudeController
{
    internal readonly record struct Settings(double BandwidthRadS, double Damping, double DeadbandDeg, double RateDeadbandDegS,
                                             double IntegralPerS = 0.1, double CaptureRateDegS = 5.0, double RateFilterS = 0.2,
                                             double IntegralWithinDeg = 3.0, double IntegralMaxDegS = 8.0)
    {
        public static readonly Settings Default = new(1.0, 0.8, 0.0, 0.5);
    }

    /// <summary>
    /// One step: the attitude and rate error (deg, deg/s), whether the feedback stood down (InDeadband), and in the model's axes (Ksa6DofSetup.Inertia's: x, y across, z along the thrust axis), N m: the torque asked for, the air's moment it was asked against (the feedforward cancels it), what the surfaces gave, what the RCS was asked for and gave, and the RCS's propellant flow, kg/s.
    /// </summary>
    internal readonly record struct Report(
        double ErrorDeg, double RateErrorDegS, bool InDeadband,
        double3 DemandNm, double3 AirNm, double3 SurfacesNm, double3 RcsAskedNm, double3 RcsNm, double RcsFlowKgS, bool AirKnown,
        double3 IntegralNm = default, double[] FinCommandDeg = null, double[] FinNowDeg = null);

    private sealed class Slot
    {
        public bool Feedback;
        public Report Last;
        public double LastT = double.NaN;
        public bool RateKnown;
        public double3 RateErrorFiltered;
        public double3 Integral;
    }

    private static readonly ConditionalWeakTable<Vehicle, Slot> Slots = new();

    /// <summary>Pitch and yaw before roll when the RCS cannot give the whole torque: model z is the roll axis.</summary>
    private static readonly double[] RcsPriority = [1.0, 1.0, 0.3];

    /// <summary>
    /// Holds the target attitude (body to CCI, as SteerBody2Cci builds it) and turning rate (CCI, rad/s) for one step. <paramref name="aero"/> is the craft's sweep (its body moment table and its surfaces), or null to fly without the air.
    /// </summary>
    internal static Report Step(Vehicle vehicle, doubleQuat targetBody2Cci, double3 targetRateCci, KsaAeroSweep.Result aero,
                                double momentLimiterPerPa, in Settings settings)
    {
        Slot slot = Slots.GetValue(vehicle, static _ => new Slot());
        double now = Universe.GetElapsedSeconds();
        double dt = double.IsFinite(slot.LastT) ? Math.Clamp(now - slot.LastT, 0.0, 0.2) : 0.0;
        slot.LastT = now;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz);
        doubleQuat body2Cci = KsaFrameBridge.BodyToCci(vehicle);
        doubleQuat cci2Body = doubleQuat.Inverse(body2Cci);

        // The rotation onto the target, in body axes: from the target's axes seen in the body, half the sum of e_i x t_i (exact for small errors, saturating for large ones).
        double3 theta = default;
        foreach (double3 e in new[] { new double3(1, 0, 0), new double3(0, 1, 0), new double3(0, 0, 1) })
            theta += 0.5 * double3.Cross(e, e.Transform(targetBody2Cci).Transform(cci2Body));
        double3 w = vehicle.BodyRates;
        double3 rawRateError = targetRateCci.Transform(cci2Body) - w;
        if (!slot.RateKnown || !(settings.RateFilterS > 0.0))
            slot.RateErrorFiltered = rawRateError;
        else if (dt > 0.0)
            slot.RateErrorFiltered += dt / (settings.RateFilterS + dt) * (rawRateError - slot.RateErrorFiltered);
        slot.RateKnown = true;
        double3 rateError = slot.RateErrorFiltered;
        double errorDeg = Math.Asin(Math.Min(theta.Length(), 1.0)) * 180.0 / Math.PI;

        // Pointing and roll apart (model z, mz, is the roll axis): a craft that starts 90 deg off in roll must not have its pointing starved by it. On the harness booster (2026-11-20) one cap on the whole error vector let a 5 deg pointing error close at 0.3 deg/s while the roll took 18 s.
        double3 thetaRoll = double3.Dot(theta, mz) * mz, thetaPoint = theta - thetaRoll;
        double withinRad = settings.IntegralWithinDeg * Math.PI / 180.0;

        // The integral, near the target only, each part on its own error, and capped.
        if (dt > 0.0)
        {
            if (thetaPoint.Length() < withinRad)
                slot.Integral += dt * thetaPoint;
            if (thetaRoll.Length() < withinRad)
                slot.Integral += dt * thetaRoll;
            double cap = settings.IntegralMaxDegS * Math.PI / 180.0;
            if (slot.Integral.Length() > cap)
                slot.Integral *= cap / slot.Integral.Length();
        }
        double rateErrorDegS = rateError.Length() * 180.0 / Math.PI;

        // The deadband, with hysteresis: out past it, back under half of it.
        if (slot.Feedback)
            slot.Feedback = !(errorDeg < 0.5 * settings.DeadbandDeg && rateErrorDegS < settings.RateDeadbandDegS);
        else
            slot.Feedback = errorDeg > settings.DeadbandDeg || rateErrorDegS > 2.0 * settings.RateDeadbandDegS;

        double kp = settings.BandwidthRadS * settings.BandwidthRadS, kd = 2.0 * settings.Damping * settings.BandwidthRadS;
        // The capture rate: kp/kd theta, which is the PD's own, capped - pointing and roll each on its own.
        double captureMax = settings.CaptureRateDegS * Math.PI / 180.0;
        double3 capture = Capped(kd > 0.0 ? kp / kd * thetaPoint : default, captureMax)
                        + Capped(kd > 0.0 ? kp / kd * thetaRoll : default, captureMax);
        double3 feedback = slot.Feedback ? Inertia(kd * (capture + rateError), mx, my, mz, ixx, iyy, izz) : default;
        double3 integral = Inertia(settings.IntegralPerS * kp * slot.Integral, mx, my, mz, ixx, iyy, izz);
        double3 gyro = double3.Cross(w, Inertia(w, mx, my, mz, ixx, iyy, izz));
        bool airKnown = TryAirMoment(vehicle, aero, momentLimiterPerPa, out double3 air);
        double3 demand = feedback + integral + gyro - air;

        // One programme for fins and RCS (the RCS as the fins' backup): the fins are commanded, the RCS fired for what they will give short.
        RcsTorqueAllocator rcsColumns = KsaRcsTorqueControl.Measure(vehicle, mx, my, mz, out _);
        KsaAeroTorqueControl.Report surfaces = KsaAeroTorqueControl.Command(vehicle, aero, demand, mx, my, mz, RcsPriority, momentLimiterPerPa,
                                                                           dt > 0.0 ? dt : 0.05, rcsColumns);
        double3 surfacesBody = surfaces.GivenNm.X * mx + surfaces.GivenNm.Y * my + surfaces.GivenNm.Z * mz;
        double3 rcsAsked = demand - surfacesBody;
        KsaRcsTorqueControl.Report rcs = KsaRcsTorqueControl.Command(vehicle, rcsAsked, mx, my, mz, RcsPriority);

        slot.Last = new Report(errorDeg, rateErrorDegS, !slot.Feedback,
            Model(demand, mx, my, mz), Model(air, mx, my, mz), surfaces.GivenNm, Model(rcsAsked, mx, my, mz),
            rcs.GivenNm, rcs.FlowKgS, airKnown, Model(integral, mx, my, mz), surfaces.CommandDeg, surfaces.NowDeg);
        return slot.Last;
    }

    /// <summary>Stops the controller's actuators for this craft: surfaces home, RCS released, the feedback reset.</summary>
    internal static void Release(Vehicle vehicle)
    {
        if (vehicle == null)
            return;
        KsaAeroTorqueControl.Release(vehicle);
        KsaRcsTorqueControl.Release(vehicle);
        if (Slots.TryGetValue(vehicle, out Slot slot))
        {
            slot.Feedback = false;
            slot.Last = default;
            slot.LastT = double.NaN;
            slot.RateKnown = false;
            slot.Integral = default;
        }
    }

    /// <summary>The last step's report for this craft.</summary>
    internal static Report Last(Vehicle vehicle) => vehicle != null && Slots.TryGetValue(vehicle, out Slot slot) ? slot.Last : default;

    /// <summary>
    /// The air's moment on the craft as it flies now, body axes, N m (see the class): false, and none, without a body moment table, below 50 Pa or outside the table.
    /// </summary>
    internal static bool TryAirMoment(Vehicle vehicle, KsaAeroSweep.Result aero, double momentLimiterPerPa, out double3 moment)
    {
        moment = default;
        if (aero?.BodyMoment == null || aero.BodyAero == null || aero.Atmosphere == null
            || !(aero.ReferenceArea > 0.0) || !(aero.ReferenceLength > 0.0))
            return false;
        float3 airF = PhysicsStates.ComputeAirVelocityBody(vehicle.BubbleOrigin, vehicle.KinematicStates, vehicle.PhysicsEnvironment);
        double3 airVelocity = double3.Unpack(in airF);
        double speed = airVelocity.Length();
        double q = 0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * speed * speed;
        if (!(speed > 1.0) || !(q > 50.0))
            return false;
        double3 u = airVelocity / speed;
        double alpha = Math.Atan2(u.Z, -u.X), beta = Math.Atan2(u.Y, -u.X);
        double limit = KsaAeroSweep.BodyAngleMaxDeg * Math.PI / 180.0;
        if (!(-u.X > 0.0) || Math.Abs(alpha) > limit || Math.Abs(beta) > limit)
            return false;
        double mach = aero.Atmosphere.Mach(speed);
        aero.BodyAero.Coefficients(mach, alpha, beta, out double cx, out double cy, out double cz);
        aero.BodyMoment.Coefficients(mach, alpha, beta, out double cl, out double cm, out double cn);
        double qs = q * aero.ReferenceArea;
        var force = new double3(qs * cx, qs * cy, qs * cz);
        float3 comNow = vehicle.CenterOfMassAsmbF;
        double3 shift = aero.BodyMomentComAsmb - new double3(comNow.X, comNow.Y, comNow.Z);
        double divisor = 1.0 + (double.IsFinite(momentLimiterPerPa) && momentLimiterPerPa > 0.0 ? momentLimiterPerPa * q : 0.0);
        moment = (qs * aero.ReferenceLength * new double3(cl, cm, cn) + double3.Cross(shift, force)) / divisor;
        return double.IsFinite(moment.X) && double.IsFinite(moment.Y) && double.IsFinite(moment.Z);
    }

    private static double3 Capped(double3 v, double max)
        => max > 0.0 && v.Length() > max ? v * (max / v.Length()) : v;

    /// <summary>Roll steps searched round the axis, degrees.</summary>
    private const double RollSearchStepDeg = 5.0;

    /// <summary>
    /// The roll to turn by, rad (right-handed about body +x, the engine-first axis), to reach the nearest roll at which the air's rolling moment about that axis is zero and restoring (a roll one way brings a moment the other), from the body moment table at the air the craft meets in its present axes, <paramref name="airVelocityBody"/> (m/s, body axes; rolling the body turns that air about +x). Where no roll trims, the one with the least moment. Also the moment there now and at the roll chosen, N m, with the game's moment limiter. The tables' centre of mass is used: a centre of mass off the axis adds a little the search ignores. False without a body moment table, below 50 Pa, or outside the tables.
    /// </summary>
    internal static bool TryLeastRollMoment(KsaAeroSweep.Result aero, double3 airVelocityBody, double rho, double momentLimiterPerPa,
                                            out double deltaRad, out double momentNowNm, out double momentAtNm)
    {
        deltaRad = 0.0;
        momentNowNm = momentAtNm = double.NaN;
        if (aero?.BodyMoment == null || aero.Atmosphere == null || !(aero.ReferenceArea > 0.0) || !(aero.ReferenceLength > 0.0))
            return false;
        double speed = airVelocityBody.Length();
        double q = 0.5 * rho * speed * speed;
        if (!(speed > 1.0) || !(q > 50.0))
            return false;
        double3 u = airVelocityBody / speed;
        if (!(-u.X > 0.0))
            return false;
        double mach = aero.Atmosphere.Mach(speed);
        double limit = KsaAeroSweep.BodyAngleMaxDeg * Math.PI / 180.0;
        double divisor = 1.0 + (double.IsFinite(momentLimiterPerPa) && momentLimiterPerPa > 0.0 ? momentLimiterPerPa * q : 0.0);
        double scale = q * aero.ReferenceArea * aero.ReferenceLength / divisor;

        int n = (int)Math.Round(360.0 / RollSearchStepDeg);
        var moment = new double[n];
        var deltas = new double[n];
        for (int k = 0; k < n; k++)
        {
            double d = (-180.0 + k * RollSearchStepDeg) * Math.PI / 180.0;
            deltas[k] = d;
            // The body rolled by d about +x: its y and z turn by d, so the air in them turns by -d.
            double c = Math.Cos(d), s = Math.Sin(d);
            double uy = u.Y * c + u.Z * s, uz = -u.Y * s + u.Z * c;
            double alpha = Math.Atan2(uz, -u.X), beta = Math.Atan2(uy, -u.X);
            if (Math.Abs(alpha) > limit || Math.Abs(beta) > limit)
                return false;
            aero.BodyMoment.Coefficients(mach, alpha, beta, out double cl, out _, out _);
            moment[k] = scale * cl;
        }
        int zero = n / 2;
        momentNowNm = moment[zero];

        // Trims: the moment falling through zero as the roll grows, a restoring one. The nearest to the present roll.
        double best = double.NaN;
        for (int k = 0; k < n; k++)
        {
            int k1 = (k + 1) % n;
            if (!(moment[k] > 0.0 && moment[k1] <= 0.0))
                continue;
            double frac = moment[k] / (moment[k] - moment[k1]);
            double d = Math.IEEERemainder(deltas[k] + frac * RollSearchStepDeg * Math.PI / 180.0, 2.0 * Math.PI);
            if (double.IsNaN(best) || Math.Abs(d) < Math.Abs(best))
                best = d;
        }
        if (double.IsNaN(best))
        {
            // None: the least moment, the nearer of equals.
            int at = zero;
            for (int k = 0; k < n; k++)
                if (Math.Abs(moment[k]) < Math.Abs(moment[at]) - 1e-9
                    || (Math.Abs(moment[k]) <= Math.Abs(moment[at]) + 1e-9 && Math.Abs(deltas[k]) < Math.Abs(deltas[at])))
                    at = k;
            best = deltas[at];
            momentAtNm = moment[at];
        }
        else
            momentAtNm = 0.0;
        deltaRad = best;
        return true;
    }

    /// <summary>I v for a body-axis vector, the inertia diagonal in the model's axes (mx, my, mz).</summary>
    private static double3 Inertia(double3 v, double3 mx, double3 my, double3 mz, double ixx, double iyy, double izz)
        => ixx * double3.Dot(v, mx) * mx + iyy * double3.Dot(v, my) * my + izz * double3.Dot(v, mz) * mz;

    private static double3 Model(double3 v, double3 mx, double3 my, double3 mz)
        => new(double3.Dot(v, mx), double3.Dot(v, my), double3.Dot(v, mz));
}
