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
///     tau = I (wn^2 theta + 2 zeta wn (w_target - w))  +  w x I w  -  M_air
///  - theta: the small rotation that takes the body onto the target, from their axes; w and w_target the body's rate and the target's turning rate.
///  - M_air: the air's moment on the craft as it flies now, from the sweep's body moment table at the Mach, alpha and beta flown, moved to the present centre of mass and divided by the game's moment limiter (BodyAero6Dof.MomentLimiterPerPa). Cancelled before any error builds: the feedforward. None outside the table (20 deg off engine first).
///  - I: the inertia about the model's axes (Ksa6DofSetup.Inertia), and w x I w its gyroscopic part.
///
/// THE SPLIT, cheapest first. The control surfaces take what they can (KsaAeroTorqueControl: free), then the RCS the rest (KsaRcsTorqueControl, pitch and yaw before roll). The engine is not used: this flies a craft whose engine is off.
///
/// THE DEADBAND. Inside RcsDeadbandDeg of the target, turning at under RcsRateDeadbandDegS from its rate, the feedback stands down and only the feedforward flies: the air's moment is held with a steady duty instead of a correction each way of it. It comes back once the error is past the deadband, and stands down again under half of it.
///
/// Main thread: it reads the craft and commands its actuators.
/// </summary>
internal static class AttitudeController
{
    internal readonly record struct Settings(double BandwidthRadS, double Damping, double DeadbandDeg, double RateDeadbandDegS)
    {
        public static readonly Settings Default = new(1.0, 0.8, 0.5, 0.5);
    }

    /// <summary>
    /// One step: the attitude and rate error (deg, deg/s), whether the feedback stood down (InDeadband), and in the model's axes (Ksa6DofSetup.Inertia's: x, y across, z along the thrust axis), N m: the torque asked for, the air's moment it was asked against (the feedforward cancels it), what the surfaces gave, what the RCS was asked for and gave, and the RCS's propellant flow, kg/s.
    /// </summary>
    internal readonly record struct Report(
        double ErrorDeg, double RateErrorDegS, bool InDeadband,
        double3 DemandNm, double3 AirNm, double3 SurfacesNm, double3 RcsAskedNm, double3 RcsNm, double RcsFlowKgS, bool AirKnown);

    private sealed class Slot
    {
        public bool Feedback;
        public Report Last;
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
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz);
        doubleQuat body2Cci = KsaFrameBridge.BodyToCci(vehicle);
        doubleQuat cci2Body = doubleQuat.Inverse(body2Cci);

        // The rotation onto the target, in body axes: from the target's axes seen in the body, half the sum of e_i x t_i (exact for small errors, saturating for large ones).
        double3 theta = default;
        foreach (double3 e in new[] { new double3(1, 0, 0), new double3(0, 1, 0), new double3(0, 0, 1) })
            theta += 0.5 * double3.Cross(e, e.Transform(targetBody2Cci).Transform(cci2Body));
        double3 w = vehicle.BodyRates;
        double3 rateError = targetRateCci.Transform(cci2Body) - w;
        double errorDeg = Math.Asin(Math.Min(theta.Length(), 1.0)) * 180.0 / Math.PI;
        double rateErrorDegS = rateError.Length() * 180.0 / Math.PI;

        // The deadband, with hysteresis: out past it, back under half of it.
        if (slot.Feedback)
            slot.Feedback = !(errorDeg < 0.5 * settings.DeadbandDeg && rateErrorDegS < settings.RateDeadbandDegS);
        else
            slot.Feedback = errorDeg > settings.DeadbandDeg || rateErrorDegS > 2.0 * settings.RateDeadbandDegS;

        double kp = settings.BandwidthRadS * settings.BandwidthRadS, kd = 2.0 * settings.Damping * settings.BandwidthRadS;
        double3 feedback = slot.Feedback ? Inertia(kp * theta + kd * rateError, mx, my, mz, ixx, iyy, izz) : default;
        double3 gyro = double3.Cross(w, Inertia(w, mx, my, mz, ixx, iyy, izz));
        bool airKnown = TryAirMoment(vehicle, aero, momentLimiterPerPa, out double3 air);
        double3 demand = feedback + gyro - air;

        // Cheapest first: the surfaces, then the RCS for what they leave.
        KsaAeroTorqueControl.Report surfaces = KsaAeroTorqueControl.Command(vehicle, aero, demand, mx, my, mz, RcsPriority, momentLimiterPerPa);
        double3 surfacesBody = surfaces.GivenNm.X * mx + surfaces.GivenNm.Y * my + surfaces.GivenNm.Z * mz;
        double3 rcsAsked = demand - surfacesBody;
        KsaRcsTorqueControl.Report rcs = KsaRcsTorqueControl.Command(vehicle, rcsAsked, mx, my, mz, RcsPriority);

        slot.Last = new Report(errorDeg, rateErrorDegS, !slot.Feedback,
            Model(demand, mx, my, mz), Model(air, mx, my, mz), surfaces.GivenNm, Model(rcsAsked, mx, my, mz),
            rcs.GivenNm, rcs.FlowKgS, airKnown);
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

    /// <summary>I v for a body-axis vector, the inertia diagonal in the model's axes (mx, my, mz).</summary>
    private static double3 Inertia(double3 v, double3 mx, double3 my, double3 mz, double ixx, double iyy, double izz)
        => ixx * double3.Dot(v, mx) * mx + iyy * double3.Dot(v, my) * my + izz * double3.Dot(v, mz) * mz;

    private static double3 Model(double3 v, double3 mx, double3 my, double3 mz)
        => new(double3.Dot(v, mx), double3.Dot(v, my), double3.Dot(v, mz));
}
