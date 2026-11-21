#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Features.RcsTorque;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// AFC's own attitude controller: holds a target attitude and rate with a torque that knows the air, and spends it on the craft's control surfaces first and its RCS for the rest, in place of the stock flight computer's attitude hold.
///
/// WHY. Entry guidance pointed the craft through the stock flight computer, which knows nothing of the air's moment: it nulls the error the air keeps making, so on a booster gliding at a few degrees off the airflow it fires its RCS every way, all the time (2026-11-20), and holds a standing error that entry's trim then winds out.
///
/// THE LAW, in KSA body axes:
///     tau = I kd (w_capture + (w_target - w)_filtered)  +  I ki wn^2 integral(theta)  +  w x I w  -  M_air,   kd = 2 zeta wn
///  - w_capture = (wn / 2 zeta) theta, capped at CaptureRateDegS: unsaturated it is the PD's kp theta; a large error (the glide's first step, 73 deg off in roll on 2026-11-20) is closed at a set rate rather than at the RCS's full thrust. The roll has its own cap where RollCaptureRateDegS is set: an ascent's roll program turns up to 180 deg while it climbs straight up, and at 5 deg/s it was still 62 deg off when the roll program gave up after 10 s, and only finished 13 s into the pitch-over (harness, 2026-11-21; the stock hold rolled at 17-34 deg/s).
///    That cap is also held to what the craft can stop in the roll left, sqrt(2 a theta) at half the roll acceleration its lit gimbals and RCS can give (RollAuthority), and the roll may have its own, softer gain (RollBandwidthRadS): at 30 deg/s with neither, the solids stack rolled through its target and never settled.
///  - theta's roll part is the true roll angle about the roll axis, to 180 deg either way: the sum below gives the sine of the angle, which falls back to nothing as a roll error nears 180 deg, the heads-down roll from a pad facing the other way.
///  - (w_target - w)_filtered: the rate error through a first-order lag of RateFilterS. On the flown booster the rate error read 0.5 deg/s of wobble, about 10 kN m of demand jittering every step through its inertia, which the fins chased (2026-11-20).
///  - integral(theta): the error integrated while within IntegralWithinDeg, capped at IntegralMaxDegS, at ki = IntegralPerS: it learns what the feedforward from the tables misses (17 % of the air's moment on that flight), so the error settles inside the deadband instead of standing at a degree. It keeps acting in the deadband.
///  - theta: the small rotation that takes the body onto the target, from their axes; w and w_target the body's rate and the target's turning rate.
///  - M_air: the air's moment on the craft as it flies now, from the sweep's body moment table at the Mach, alpha and beta flown, moved to the present centre of mass and divided by the game's moment limiter (BodyAero6Dof.MomentLimiterPerPa). Cancelled before any error builds: the feedforward. None outside the table (20 deg off engine first).
///  - I: the inertia about the model's axes (Ksa6DofSetup.Inertia), and w x I w its gyroscopic part.
///
/// THE GIMBALS, on a burn (Settings.UseGimbals: an ascent). The lit engines' allocation (KsaGimbalControl.SetLsq, the one touchdown guidance flies) is given the whole demand first; the torque it will give (KsaGimbalControl.PredictLsq) comes off what the surfaces and RCS are asked for, so they take only the roll and whatever the gimbals' travel cannot reach.
///
/// THE ROLL, held or free (Settings.RollFree). Free, as an ascent flies until its forced roll, no roll angle is held - only the roll rate damped - as the stock flight computer's decoupled roll held none.
///
/// THE SPLIT, in one programme. The control surfaces and the RCS thrusters share the demand (KsaAeroTorqueControl.Command with the RCS as its backup): a degree of fin costs next to nothing, a thruster its propellant, so the fins take every axis they can and the RCS only what they cannot - roll included, where a fixed "fins first, roll last" order had left roll to the RCS. Each fin is asked only for what its rate reaches by the next step, from where it is, and pays to move, so it does not chatter. The RCS is then fired for what the fins will give short of the demand (KsaRcsTorqueControl, pitch and yaw before roll). The engine's gimbals come in only where the settings use them (see THE GIMBALS).
///
/// AUTHORITY-AWARE CAPTURE (Settings.AuthorityCapture; an orbital entry). The capture rate becomes the square-root law of a time-optimal stop: k theta near the target, and past theta = a / k^2, sqrt(2 a (theta - a / 2k^2)), the fastest the craft can still stop from in the angle left, joined to the linear part without a kink (ArduPilot's sqrt controller has the same shape). a is the angular acceleration the craft can spare about the axis now: what its surfaces and RCS can give each way (their Characterise) less the trim they hold against the air, over the inertia, at AuthorityShare of it: the surfaces' reach measured every AuthorityEveryS, as it follows the air, and the RCS's once, and again only when the mass or the centre of mass moves past RcsRemeasureShare. Near the target nothing changes; far off, or with little authority, it asks only for rates it can stop from, so the demand stays within reach and the surfaces do not slam from stop to stop (2026-11-21: 1.5-4 MN m asked against 0.1-0.6 given, 34 full flap reversals a minute). The integral stands still while the actuators fall short of the demand (conditional integration, against wind-up). The report carries the acceleration the craft can spare about the airflow, which entry guidance holds its bank's slew to.
///
/// SURFACE SMOOTHING (Settings.SurfaceSmoothS; an orbital entry, where RCS backs the surfaces). The surfaces' deflections follow the allocator's through a first-order lag of that time, and the RCS takes what they then give short: dynamic control allocation (Harkegard), a price on the surfaces' rate of movement and none on their deflection, so slow demand goes to the surfaces and fast jitter to the RCS. In thin air the RCS's flutter about a held attitude swung the flaps 10 deg every half second for next to no torque (2026-11-21).
///
/// THE DEADBAND (Settings.DeadbandDeg, off by default). Inside it, turning at under RateDeadbandDegS from the target's rate, the feedback stands down and only the feedforward and the integral fly; it comes back past the deadband and stands down again under half of it. Off since the integral and the filtered rate went in: each return of the feedback kicked, and in the harness a 0.5 deg deadband cost 50 kg of RCS holding a tilt where none cost 46, and 1 deg cost 234 (2026-11-20).
///
/// Main thread: it reads the craft and commands its actuators.
/// </summary>
internal static class AttitudeController
{
    internal readonly record struct Settings(double BandwidthRadS, double Damping, double DeadbandDeg, double RateDeadbandDegS,
                                             double IntegralPerS = 0.1, double CaptureRateDegS = 5.0, double RateFilterS = 0.2,
                                             double IntegralWithinDeg = 3.0, double IntegralMaxDegS = 8.0,
                                             bool UseSurfaces = true, bool UseGimbals = false, bool RollFree = false,
                                             double RollCaptureRateDegS = double.NaN, double RollBandwidthRadS = double.NaN,
                                             bool AuthorityCapture = false, double SurfaceSmoothS = 0.0)
    {
        public static readonly Settings Default = new(1.0, 0.8, 0.0, 0.5);
    }

    /// <summary>
    /// One step: the attitude and rate error (deg, deg/s), whether the feedback stood down (InDeadband), and in the model's axes (Ksa6DofSetup.Inertia's: x, y across, z along the thrust axis), N m: the torque asked for, the air's moment it was asked against (the feedforward cancels it), what the surfaces gave, what the RCS was asked for and gave, and the RCS's propellant flow, kg/s.
    /// </summary>
    internal readonly record struct Report(
        double ErrorDeg, double RateErrorDegS, bool InDeadband,
        double3 DemandNm, double3 AirNm, double3 SurfacesNm, double3 RcsAskedNm, double3 RcsNm, double RcsFlowKgS, bool AirKnown,
        double3 IntegralNm = default, double[] FinCommandDeg = null, double[] FinNowDeg = null, double3 GimbalsNm = default,
        double3 SpareAccelRadS2 = default, double AirAxisAccelRadS2 = double.NaN, bool Saturated = false);

    private sealed class Slot
    {
        public bool Feedback;
        public Report Last;
        public double LastT = double.NaN;
        public bool RateKnown;
        public double3 RateErrorFiltered;
        public double3 Integral;
        /// <summary>The gimbals were given a torque (KsaGimbalControl.SetLsq), so they are handed back when the controller lets go or stops using them.</summary>
        public bool GimbalsOwned;
        /// <summary>The roll acceleration the lit gimbals and the RCS can give, rad/s^2 (RollAuthority), and when it was measured.</summary>
        public double RollAccel = double.NaN;
        public double RollAccelAt = double.NaN;
        /// <summary>The most torque the surfaces can give about each model axis, each way, N m (SpareAccel), and when measured.</summary>
        public double3 FinPlusNm, FinMinusNm;
        public double MaxAt = double.NaN;
        /// <summary>The same for the RCS, measured once and again only when the mass or the centre of mass has moved past RcsRemeasureShare (SpareAccel): the mass and centre of mass it was measured at, and whether it has been.</summary>
        public double3 RcsPlusNm, RcsMinusNm;
        public double RcsMassKg = double.NaN;
        public double3 RcsComAsmb;
        public bool RcsKnown;
        public readonly List<ControlSurface> Scratch = new();
        /// <summary>Last step the actuators fell short of the demand: the integral stands still.</summary>
        public bool Saturated;
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
        // Roll free (Settings.RollFree): no roll angle is held, only the roll rate damped.
        if (settings.RollFree)
            theta -= double3.Dot(theta, mz) * mz;
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
        double3 thetaPoint = theta - double3.Dot(theta, mz) * mz;
        double3 thetaRoll = settings.RollFree ? default : RollAngle(targetBody2Cci, cci2Body, mz) * mz;
        double withinRad = settings.IntegralWithinDeg * Math.PI / 180.0;

        // The integral, near the target only, each part on its own error, and capped; standing still while the actuators fell short last step (AuthorityCapture).
        if (dt > 0.0 && !(settings.AuthorityCapture && slot.Saturated))
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

        bool airKnown = TryAirMoment(vehicle, aero, momentLimiterPerPa, out double3 air);
        double kp = settings.BandwidthRadS * settings.BandwidthRadS, kd = 2.0 * settings.Damping * settings.BandwidthRadS;
        // The roll's own gains where RollBandwidthRadS is set (an ascent: see Settings).
        double rollBandwidth = double.IsFinite(settings.RollBandwidthRadS) ? settings.RollBandwidthRadS : settings.BandwidthRadS;
        double kpRoll = rollBandwidth * rollBandwidth, kdRoll = 2.0 * settings.Damping * rollBandwidth;
        // The capture rate: kp/kd theta, which is the PD's own, capped - pointing and roll each on its own.
        double captureMax = settings.CaptureRateDegS * Math.PI / 180.0;
        double rollCaptureMax = captureMax;
        if (double.IsFinite(settings.RollCaptureRateDegS))
        {
            // A fast roll is one the craft can stop: no faster than half its roll acceleration brings to rest in the roll left (RollAuthority). At 30 deg/s with nothing braking it, the solids stack rolled through its target and swung 30-128 deg either side of it for the rest of the solid burn (harness, 2026-11-21).
            rollCaptureMax = settings.RollCaptureRateDegS * Math.PI / 180.0;
            double accel = RollAuthority(vehicle, slot, now, mx, my, mz, izz);
            if (double.IsFinite(accel))
                rollCaptureMax = Math.Min(rollCaptureMax, Math.Max(Math.Sqrt(2.0 * RollBrakingShare * accel * thetaRoll.Length()), captureMax));
        }
        double3 rateRoll = double3.Dot(rateError, mz) * mz, ratePoint = rateError - rateRoll;
        double3 integralRoll = double3.Dot(slot.Integral, mz) * mz, integralPoint = slot.Integral - integralRoll;
        double3 capturePoint = Capped(kd > 0.0 ? kp / kd * thetaPoint : default, captureMax);
        double3 captureRoll = Capped(kdRoll > 0.0 ? kpRoll / kdRoll * thetaRoll : default, rollCaptureMax);
        double3 spare = default;
        double airAxisAccel = double.NaN;
        if (settings.AuthorityCapture)
        {
            // The acceleration the craft can spare about each model axis, after the trim it holds: the square-root law's a.
            spare = SpareAccel(vehicle, slot, now, aero, momentLimiterPerPa, settings.UseSurfaces, mx, my, mz, ixx, iyy, izz,
                               airKnown ? -Model(air, mx, my, mz) : default);
            double rollCap = double.IsFinite(settings.RollCaptureRateDegS) ? settings.RollCaptureRateDegS * Math.PI / 180.0 : captureMax;
            capturePoint = SqrtCapture(thetaPoint, kd > 0.0 ? kp / kd : 0.0, captureMax, Math.Min(spare.X, spare.Y));
            captureRoll = SqrtCapture(thetaRoll, kdRoll > 0.0 ? kpRoll / kdRoll : 0.0, rollCap, spare.Z);
            airAxisAccel = AirAxisAccel(vehicle, mx, my, mz, spare);
        }
        double3 capture = capturePoint + captureRoll;
        double3 feedback = slot.Feedback ? Inertia(kd * (capturePoint + ratePoint) + kdRoll * (captureRoll + rateRoll), mx, my, mz, ixx, iyy, izz) : default;
        double3 integral = Inertia(settings.IntegralPerS * (kp * integralPoint + kpRoll * integralRoll), mx, my, mz, ixx, iyy, izz);
        double3 gyro = double3.Cross(w, Inertia(w, mx, my, mz, ixx, iyy, izz));
        double3 demand = feedback + integral + gyro - air;

        // THE GIMBALS FIRST, where the settings use them (an ascent): the whole demand goes to the lit engines' allocation (KsaGimbalControl.SetLsq), and what it will give - pitch and yaw, and the roll the travel left allows - comes off what the rest are asked for. With no engine lit they give nothing and the rest take it all.
        double3 remaining = demand, gimbalsBody = default;
        if (settings.UseGimbals)
        {
            gimbalsBody = KsaGimbalControl.PredictLsq(vehicle, demand, out _);
            KsaGimbalControl.SetLsq(vehicle, demand);
            slot.GimbalsOwned = true;
            remaining -= gimbalsBody;
        }
        else if (slot.GimbalsOwned)
        {
            KsaGimbalControl.Disengage(vehicle);
            slot.GimbalsOwned = false;
        }

        // One programme for fins and RCS (the RCS as the fins' backup): the fins are commanded, the RCS fired for what they will give short. Fins off (Settings.UseSurfaces) are held at rest and the RCS takes it all.
        RcsTorqueAllocator rcsColumns = KsaRcsTorqueControl.Measure(vehicle, mx, my, mz, out _);
        KsaAeroTorqueControl.Report surfaces = default;
        if (settings.UseSurfaces)
            surfaces = KsaAeroTorqueControl.Command(vehicle, aero, remaining, mx, my, mz, RcsPriority, momentLimiterPerPa,
                                                    dt > 0.0 ? dt : 0.05, rcsColumns, settings.SurfaceSmoothS);
        else
            KsaAeroTorqueControl.Release(vehicle);
        double3 surfacesBody = surfaces.GivenNm.X * mx + surfaces.GivenNm.Y * my + surfaces.GivenNm.Z * mz;
        double3 rcsAsked = remaining - surfacesBody;
        KsaRcsTorqueControl.Report rcs = KsaRcsTorqueControl.Command(vehicle, rcsAsked, mx, my, mz, RcsPriority);
        // Short of the demand by more than SaturatedShare of it: the actuators are at their limits.
        double3 givenBody = gimbalsBody + surfacesBody + rcs.GivenNm.X * mx + rcs.GivenNm.Y * my + rcs.GivenNm.Z * mz;
        slot.Saturated = demand.Length() > 0.0 && (demand - givenBody).Length() > SaturatedShare * demand.Length();

        slot.Last = new Report(errorDeg, rateErrorDegS, !slot.Feedback,
            Model(demand, mx, my, mz), Model(air, mx, my, mz), surfaces.GivenNm, Model(rcsAsked, mx, my, mz),
            rcs.GivenNm, rcs.FlowKgS, airKnown, Model(integral, mx, my, mz), surfaces.CommandDeg, surfaces.NowDeg,
            Model(gimbalsBody, mx, my, mz), spare, airAxisAccel, slot.Saturated);
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
            if (slot.GimbalsOwned)
                KsaGimbalControl.Disengage(vehicle);
            slot.GimbalsOwned = false;
            slot.Feedback = false;
            slot.Last = default;
            slot.LastT = double.NaN;
            slot.RateKnown = false;
            slot.Integral = default;
            slot.Saturated = false;
            slot.MaxAt = double.NaN;
            slot.RcsKnown = false;
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
        // Engine first, nose first (an ascent) or belly first (an orbital entry): whichever of the sweep's tables the air is in (KsaAeroSweep.TryPickTables).
        if (!KsaAeroSweep.TryPickTables(aero, u, surfacesToo: false, out KsaAeroSweep.TableSet set, out double alpha, out double beta))
            return false;
        BodyAeroTable forceTable = aero.BodyForceFor(set), momentTable = aero.BodyMomentFor(set);
        if (forceTable == null || momentTable == null)
            return false;
        double mach = Math.Clamp(aero.Atmosphere.Mach(speed), forceTable.MachMin, forceTable.MachMax);
        forceTable.Coefficients(mach, alpha, beta, out double cx, out double cy, out double cz);
        momentTable.Coefficients(mach, alpha, beta, out double cl, out double cm, out double cn);
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

    /// <summary>The share of the spare acceleration the square-root capture counts on to stop (AuthorityCapture): the estimate of the surfaces' reach can be off, and the same actuators also hold the other axes.</summary>
    private const double AuthorityShare = 0.5;

    /// <summary>The least acceleration the square-root capture assumes, rad/s^2, so a craft whose trim takes all it has still turns, slowly.</summary>
    private const double AuthorityFloorRadS2 = 0.2 * Math.PI / 180.0;

    /// <summary>How often the surfaces' and RCS's reach is measured again, s (six small LPs each).</summary>
    private const double AuthorityEveryS = 0.5;

    /// <summary>
    /// How far the mass may change, as a share of it, and the centre of mass move, as a share of the craft's largest radius of gyration (about the size of the thrusters' levers), before the RCS's reach is measured again: a lever off by that share puts the torque off by about as much. Measuring it took six linear programmes on the main thread every half second of game time, more often under warp (2026-11-21).
    /// </summary>
    private const double RcsRemeasureShare = 0.02;

    /// <summary>The share of the demand the actuators may fall short by before the step counts as saturated.</summary>
    private const double SaturatedShare = 0.15;

    /// <summary>
    /// The capture rate along <paramref name="theta"/>: k theta near the target, and beyond theta = a / k^2 the square root of a time-optimal stop, sqrt(2 a (theta - a / 2k^2)), with a = AuthorityShare of <paramref name="accel"/> (floored at AuthorityFloorRadS2); capped at <paramref name="cap"/>. Unknown acceleration: the linear law, capped.
    /// </summary>
    private static double3 SqrtCapture(double3 theta, double k, double cap, double accel)
    {
        double m = theta.Length();
        if (!(m > 0.0) || !(k > 0.0))
            return default;
        double rate = k * m;
        if (double.IsFinite(accel))
        {
            double a = Math.Max(AuthorityShare * accel, AuthorityFloorRadS2);
            double linear = a / (k * k);
            if (m > linear)
                rate = Math.Sqrt(2.0 * a * (m - 0.5 * linear));
        }
        if (cap > 0.0)
            rate = Math.Min(rate, cap);
        return theta * (rate / m);
    }

    /// <summary>
    /// The angular acceleration the craft can spare about each model axis, rad/s^2: the most its surfaces and RCS give that way and the other (measured every AuthorityEveryS), less the trim torque <paramref name="trimModel"/> they hold against the air (model axes, N m), the smaller way round, over the inertia. NaN on an axis with no inertia.
    /// </summary>
    private static double3 SpareAccel(Vehicle vehicle, Slot slot, double now, KsaAeroSweep.Result aero, double momentLimiterPerPa, bool surfaces,
                                      double3 mx, double3 my, double3 mz, double ixx, double iyy, double izz, double3 trimModel)
    {
        // The surfaces' reach follows the air, so it is measured again every AuthorityEveryS.
        if (!(double.IsFinite(slot.MaxAt) && now - slot.MaxAt < AuthorityEveryS && now >= slot.MaxAt))
        {
            slot.MaxAt = now;
            double[] plus = new double[3], minus = new double[3];
            AeroTorqueAllocator fins = surfaces ? KsaAeroTorqueControl.Measure(vehicle, aero, mx, my, mz, momentLimiterPerPa, slot.Scratch, out _) : null;
            if (fins != null)
                for (int axis = 0; axis < 3; axis++)
                {
                    plus[axis] = Math.Max(fins.Characterise(axis, 1).MaxTorqueNm, 0.0);
                    minus[axis] = Math.Max(fins.Characterise(axis, -1).MaxTorqueNm, 0.0);
                }
            slot.FinPlusNm = new double3(plus[0], plus[1], plus[2]);
            slot.FinMinusNm = new double3(minus[0], minus[1], minus[2]);
        }
        // The RCS's reach does not: its thrusters' levers change only as the mass and centre of mass do, so it is measured once and again only past RcsRemeasureShare of either.
        double mass = vehicle.TotalMass;
        float3 comF = vehicle.CenterOfMassAsmbF;
        var com = new double3(comF.X, comF.Y, comF.Z);
        double gyration = mass > 0.0 ? Math.Sqrt(Math.Max(Math.Max(ixx, iyy), izz) / mass) : double.NaN;
        bool moved = !slot.RcsKnown
                     || !(Math.Abs(mass - slot.RcsMassKg) <= RcsRemeasureShare * slot.RcsMassKg)
                     || (double.IsFinite(gyration) && !((com - slot.RcsComAsmb).Length() <= RcsRemeasureShare * gyration));
        if (moved)
        {
            double[] plus = new double[3], minus = new double[3];
            RcsTorqueAllocator rcs = KsaRcsTorqueControl.Measure(vehicle, mx, my, mz, out _);
            if (rcs != null)
                for (int axis = 0; axis < 3; axis++)
                {
                    plus[axis] = Math.Max(rcs.Characterise(axis, 1).MaxTorqueNm, 0.0);
                    minus[axis] = Math.Max(rcs.Characterise(axis, -1).MaxTorqueNm, 0.0);
                }
            slot.RcsPlusNm = new double3(plus[0], plus[1], plus[2]);
            slot.RcsMinusNm = new double3(minus[0], minus[1], minus[2]);
            slot.RcsMassKg = mass;
            slot.RcsComAsmb = com;
            slot.RcsKnown = true;
        }
        double3 up = slot.FinPlusNm + slot.RcsPlusNm, down = slot.FinMinusNm + slot.RcsMinusNm;
        static double Spare(double up, double down, double trim, double inertia)
            => inertia > 0.0 ? Math.Max(Math.Min(up - trim, down + trim), 0.0) / inertia : double.NaN;
        return new double3(Spare(up.X, down.X, trimModel.X, ixx),
                           Spare(up.Y, down.Y, trimModel.Y, iyy),
                           Spare(up.Z, down.Z, trimModel.Z, izz));
    }

    /// <summary>
    /// The angular acceleration the craft can spare about the airflow, rad/s^2 - the axis a bank turns about: the rate about a unit axis u (model components u_i) can grow at a only while a |u_i| stays within each axis's spare, so a = min a_i / |u_i|. NaN without air.
    /// </summary>
    private static double AirAxisAccel(Vehicle vehicle, double3 mx, double3 my, double3 mz, double3 spare)
    {
        float3 airF = PhysicsStates.ComputeAirVelocityBody(vehicle.BubbleOrigin, vehicle.KinematicStates, vehicle.PhysicsEnvironment);
        double3 airBody = double3.Unpack(in airF);
        if (!(airBody.Length() > 1.0))
            return double.NaN;
        double3 u = airBody / airBody.Length();
        double best = double.PositiveInfinity;
        double[] comp = { double3.Dot(u, mx), double3.Dot(u, my), double3.Dot(u, mz) };
        double[] acc = { spare.X, spare.Y, spare.Z };
        for (int i = 0; i < 3; i++)
            if (Math.Abs(comp[i]) > 1e-3 && double.IsFinite(acc[i]))
                best = Math.Min(best, acc[i] / Math.Abs(comp[i]));
        return double.IsFinite(best) ? best : double.NaN;
    }

    /// <summary>The share of the measured roll acceleration a fast roll counts on to stop (RollAuthority): the gimbals slew, and they also hold the pointing.</summary>
    private const double RollBrakingShare = 0.5;

    /// <summary>How often the roll authority is measured again, s.</summary>
    private const double RollAuthorityEveryS = 0.5;

    /// <summary>
    /// The roll acceleration the craft can give now, rad/s^2: the roll torque the lit engines' gimbals deliver asked for far more than they have (KsaGimbalControl.PredictLsq), and the RCS's smaller way round (RcsTorqueAllocator.Characterise), over the inertia about the roll axis. Measured every RollAuthorityEveryS. NaN where there is no inertia.
    /// </summary>
    private static double RollAuthority(Vehicle vehicle, Slot slot, double now, double3 mx, double3 my, double3 mz, double izz)
    {
        if (double.IsFinite(slot.RollAccelAt) && now - slot.RollAccelAt < RollAuthorityEveryS && now >= slot.RollAccelAt)
            return slot.RollAccel;
        slot.RollAccelAt = now;
        double torque = 0.0;
        double3 ask = mz * Math.Max(vehicle.TotalMass * 9.81 * 1000.0, 1.0);
        torque += Math.Max(Math.Min(double3.Dot(KsaGimbalControl.PredictLsq(vehicle, ask, out _), mz),
                                    -double3.Dot(KsaGimbalControl.PredictLsq(vehicle, -ask, out _), mz)), 0.0);
        RcsTorqueAllocator rcs = KsaRcsTorqueControl.Measure(vehicle, mx, my, mz, out _);
        if (rcs != null)
            torque += Math.Max(Math.Min(rcs.Characterise(2, 1).MaxTorqueNm, rcs.Characterise(2, -1).MaxTorqueNm), 0.0);
        slot.RollAccel = izz > 0.0 && double.IsFinite(torque) ? torque / izz : double.NaN;
        return slot.RollAccel;
    }

    /// <summary>
    /// The roll onto the target about the roll axis <paramref name="mz"/> (body coordinates), radians, -pi to pi, signed as theta is: a direction square to the axis against the target's same direction, both taken square to it. Exact where the pointing is on; the sum in Step gives only its sine.
    /// </summary>
    private static double RollAngle(doubleQuat targetBody2Cci, doubleQuat cci2Body, double3 mz)
    {
        double3 a = Math.Abs(mz.X) < 0.6 ? new double3(1, 0, 0) : new double3(0, 1, 0);
        double3 p = double3.Normalize(double3.Cross(mz, a));
        double3 t = p.Transform(targetBody2Cci).Transform(cci2Body);
        t -= double3.Dot(t, mz) * mz;
        if (!(t.Length() > 1e-9))
            return 0.0;
        return Math.Atan2(double3.Dot(double3.Cross(p, t), mz), double3.Dot(p, t));
    }

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
