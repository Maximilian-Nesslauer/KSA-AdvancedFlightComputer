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
///  - theta's roll part is the true roll angle about the roll axis, to 180 deg either way (RollAngle): the heads-down roll from a pad facing the other way. The old error, half the sum of e_i x t_i, gave the sine of the angle, which fell back to nothing near 180 deg.
///  - (w_target - w)_filtered: the rate error, unfiltered by default (RateFilterS = 0 s) because KSA's Vehicle.BodyRates are noiseless physics values; callers may set a positive RateFilterS to apply a first-order lag.
///  - integral(theta): the error integrated per model axis while within IntegralWithinDeg, capped at IntegralMaxDegS, at ki = IntegralPerS, with conditional anti-windup freezing any axis whose downstream actuators fall short in the same direction as theta while still allowing unwinding when theta reverses sign.
///  - theta: the rotation that takes the body onto the target, its exact axis and angle (So3AxisAngle); w and w_target the body's rate and the target's turning rate.
///  - M_air: the air's moment on the craft as it flies now, from the sweep's body moment table at the Mach, alpha and beta flown, moved to the present centre of mass and divided by the game's moment limiter (BodyAero6Dof.MomentLimiterPerPa). Cancelled before any error builds: the feedforward. None outside the table (20 deg off engine first).
///  - I: the inertia about the model's axes (Ksa6DofSetup.Inertia), and w x I w its gyroscopic part.
///
/// THE GIMBALS, on a burn (Settings.UseGimbals: an ascent). The lit engines' allocation (KsaGimbalControl.SetLsq) receives the full demand, while the 1-step rate-limited torque reachable under KSA's GimbalAxis.UpdateState (KsaGimbalControl.PredictLsqReachable) comes off what the surfaces and RCS are asked for, handing transient gimbal slew deficits to fins and RCS.
///
/// THE ROLL, held or free (Settings.RollFree). Free, as an ascent flies until its forced roll, no roll angle is held - only the roll rate damped - as the stock flight computer's decoupled roll held none.
///
/// THE SPLIT, in one programme. The control surfaces and the RCS thrusters share the demand (KsaAeroTorqueControl.Command with the RCS as its backup): a degree of fin costs next to nothing, a thruster its propellant, so the fins take every axis they can and the RCS only what they cannot - roll included, where a fixed "fins first, roll last" order had left roll to the RCS. Each fin is asked only for what its rate reaches by the next step, from where it is, and pays to move, so it does not chatter. The RCS is then fired for what the fins will give short of the demand (KsaRcsTorqueControl, pitch and yaw before roll). The engine's gimbals come in only where the settings use them (see THE GIMBALS).
///
/// AUTHORITY-AWARE CAPTURE (Settings.AuthorityCapture; an orbital entry). The capture rate becomes the square-root law of a time-optimal stop: k theta near the target, and past theta = a / k^2, sqrt(2 a (theta - a / 2k^2)), the fastest the craft can still stop from in the angle left, joined to the linear part without a kink (ArduPilot's sqrt controller has the same shape). a is the angular acceleration the craft can spare about the axis now: what its surfaces and RCS can give each way (their Characterise) less the trim they hold against the air, over the inertia, at AuthorityShare of it: the surfaces' reach measured when Mach or the airflow's direction moves (FinRemeasure*) and scaled by q in between, and the RCS's once, and again only when the mass or the centre of mass moves past RcsRemeasureShare. Near the target nothing changes; far off, or with little authority, it asks only for rates it can stop from, so the demand stays within reach and the surfaces do not slam from stop to stop (2026-11-21: 1.5-4 MN m asked against 0.1-0.6 given, 34 full flap reversals a minute). The integral stands still while the actuators fall short of the demand (conditional integration, against wind-up). The report carries the acceleration the craft can spare about the airflow, which entry guidance holds its bank's slew to.
///
/// SURFACE SMOOTHING (Settings.SurfaceSmoothS; off by default, and no flight uses it since 2026-11-22, when orbital entry went back to the one programme with an RCS dead band: the split spent the RCS on the jitter, at 2.4 times the propellant, where the flaps alone held the attitude). The torque demand is lagged over that time, and the RCS is given the fast part only as far as half its measured reach goes (SpareAccel's RCS reach); the surfaces are asked for the rest within one step's reach, and the RCS then takes whatever they leave: frequency-split allocation by reach, so with RCS to spare the jitter goes to it and the surfaces take the slow torque at their full rate, and with little RCS the surfaces take nearly all of it (KsaAeroTorqueControl.Command's smoothS). Needs AuthorityCapture, which measures the RCS's reach; without it the RCS takes none of the fast part. In thin air the RCS's flutter about a held attitude swung the flaps 10 deg every half second for next to no torque (2026-11-21).
///
/// PHASE-PLANE RCS (Settings.PhasePlaneRcs; an orbital entry). Each model axis has one owner, as the stock flight computer gives each to its gimbals or its RCS: the surfaces once their spare torque after trim is what this law needs to hold a 1 deg band (inertia x bandwidth^2 x 1 deg, OwnershipBandDeg, whatever the RCS's own band) or matches the RCS's, the RCS below that (thin air, a craft without surfaces). The surfaces fly their own axes on this law, alone, and hold only the trim on the RCS's. The RCS fires by the stock flight computer's phase plane (PhasePlaneRcs): it coasts inside switching lines drawn from its own braking authority, and on crossing one fires the one impulse that puts the craft on a slow drift back across the band - on its own axes with RcsAngleBandDeg, on the surfaces' only as a backup, past three times the band and while the error there still grows, so it never fires against the surfaces' own moves. A dead band on this law instead (2026-11-22) banged across it in thin air at 1.3 kg/s, against 0.6-0.75 continuous, and fired through every turn the flaps were flying; offline, the phase plane holds a calm axis on a twentieth of the dead band's impulse.
///
/// THE DEADBAND (Settings.DeadbandDeg, off by default). Inside it, turning at under RateDeadbandDegS from the target's rate, the feedback stands down and only the feedforward and the integral fly; it comes back past the deadband and stands down again under half of it. Off since the integral and the filtered rate went in: each return of the feedback kicked, and in the harness a 0.5 deg deadband cost 50 kg of RCS holding a tilt where none cost 46, and 1 deg cost 234 (2026-11-20).
///
/// Main thread: it reads the craft and commands its actuators.
/// </summary>
internal static class AttitudeController
{
    internal readonly record struct Settings(double BandwidthRadS, double Damping, double DeadbandDeg, double RateDeadbandDegS,
                                             double IntegralPerS = 0.1, double CaptureRateDegS = 5.0, double RateFilterS = 0.0,
                                             double IntegralWithinDeg = 3.0, double IntegralMaxDegS = 8.0,
                                             bool UseSurfaces = true, bool UseGimbals = false, bool RollFree = false,
                                             double RollCaptureRateDegS = double.NaN, double RollBandwidthRadS = double.NaN,
                                             bool AuthorityCapture = false, double SurfaceSmoothS = 0.0, bool UseRcs = true,
                                             bool PhasePlaneRcs = false, double RcsAngleBandDeg = 1.0, double RcsRateLimitDegS = 5.0,
                                             double RollPriority = 0.3)
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
        double3 SpareAccelRadS2 = default, double AirAxisAccelRadS2 = double.NaN, bool Saturated = false, int RcsAxes = 0, int RcsFiring = 0,
        double3 SpareTotalRadS2 = default);

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
        /// <summary>The most torque the surfaces can give about each model axis, each way, N m, at the flight condition they were measured at (SpareAccel): its q over the moment limiter, Mach, the airflow's direction in body axes, and the wall-clock tick.</summary>
        public double3 FinPlusNm, FinMinusNm;
        public bool FinKnown;
        public double FinQEff, FinMach;
        public double3 FinAirDir;
        public long FinAtTicks;
        /// <summary>The same for the RCS, measured once and again only when the mass or the centre of mass has moved past RcsRemeasureShare (SpareAccel): the mass and centre of mass it was measured at, and whether it has been.</summary>
        public double3 RcsPlusNm, RcsMinusNm;
        public double RcsMassKg = double.NaN;
        public double3 RcsComAsmb;
        public bool RcsKnown;
        public readonly List<ControlSurface> Scratch = new();
        /// <summary>Last step the actuators fell short of the demand: the integral stands still.</summary>
        public bool Saturated;
        /// <summary>Last step's demand and shortfall in model axes, N m, for per-axis conditional anti-windup.</summary>
        public double3 LastDemandModel;
        public double3 LastShortfallModel;
        /// <summary>PHASE-PLANE RCS: each model axis's phase-plane state, whether the RCS owns it (else the surfaces), the thrusters' shortest pulse, s, and the surfaces' spare torque each axis after trim, N m (SpareAccel).</summary>
        public readonly PhasePlaneRcs.State[] RcsState = new PhasePlaneRcs.State[3];
        public readonly bool[] RcsOwns = new bool[3];
        /// <summary>The RCS's steady work moved onto the surfaces, model axes (x, y, z), N m: on each axis the RCS owns, its torque integrated over RcsTrimShiftS, asked of the surfaces with the trim. Zero on the surfaces' own axes.</summary>
        public double[] RcsTrimNm = new double[3];
        public double RcsMinPulseS;
        /// <summary>The smallest pulse the RCS can fire about each model axis, N m s: one thruster's torque on the axis times its shortest pulse, the least of those that turn the axis at all (stock's MinRotationalImpulse).</summary>
        public double3 RcsMinImpulseNms;
        public double3 AeroSpareNm;
    }

    private static readonly ConditionalWeakTable<Vehicle, Slot> Slots = new();

    /// <summary>Pitch and yaw before roll when the RCS cannot give the whole torque: model z is the roll axis. The roll's weight is Settings.RollPriority (0.3 by default).</summary>
    private static readonly double[] RcsPriority = [1.0, 1.0, 0.3];

    /// <summary>
    /// Holds the target attitude (body to CCI, as SteerBody2Cci builds it) and turning rate (CCI, rad/s) for one step. <paramref name="aero"/> is the craft's sweep (its body moment table and its surfaces), or null to fly without the air.
    /// </summary>
    internal static Report Step(Vehicle vehicle, doubleQuat targetBody2Cci, double3 targetRateCci, KsaAeroSweep.Result aero,
                                double momentLimiterPerPa, in Settings settings)
    {
        using var _perf = new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.Step");
        Slot slot = Slots.GetValue(vehicle, static _ => new Slot());
        double now = Universe.GetElapsedSeconds();
        double dt = double.IsFinite(slot.LastT) ? Math.Clamp(now - slot.LastT, 0.0, 0.2) : 0.0;
        // The game time this command is held for until the next, unclamped: a warped frame holds it for warp times a frame, which the RCS fires through period after period.
        double holdS = double.IsFinite(slot.LastT) ? Math.Max(now - slot.LastT, 0.0) : 0.0;
        slot.LastT = now;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz);
        doubleQuat body2Cci = KsaFrameBridge.BodyToCci(vehicle);
        doubleQuat cci2Body = doubleQuat.Inverse(body2Cci);

        // Exact SO(3) axis-angle error in body axes from the relative rotation matrix columns t_i = R_err e_i.
        // Avoids sin(phi) fold-back past 90 deg and remains non-zero at 180 deg.
        double3 t0 = new double3(1, 0, 0).Transform(targetBody2Cci).Transform(cci2Body);
        double3 t1 = new double3(0, 1, 0).Transform(targetBody2Cci).Transform(cci2Body);
        double3 t2 = new double3(0, 0, 1).Transform(targetBody2Cci).Transform(cci2Body);
        So3AxisAngle.FromBasisColumns(t0.X, t0.Y, t0.Z, t1.X, t1.Y, t1.Z, t2.X, t2.Y, t2.Z,
            out double ex, out double ey, out double ez);
        var theta = new double3(ex, ey, ez);
        // Roll free (Settings.RollFree): no roll angle is held, only the roll rate damped.
        if (settings.RollFree)
            theta -= double3.Dot(theta, mz) * mz;
        double3 w = vehicle.BodyRates;
        double3 rawRateError = targetRateCci.Transform(cci2Body) - w;
        AttitudeControlMath.FilterRateError(
            rawRateError.X, rawRateError.Y, rawRateError.Z,
            slot.RateErrorFiltered.X, slot.RateErrorFiltered.Y, slot.RateErrorFiltered.Z,
            slot.RateKnown, settings.RateFilterS, dt,
            out double rfX, out double rfY, out double rfZ);
        slot.RateErrorFiltered = new double3(rfX, rfY, rfZ);
        slot.RateKnown = true;
        double3 rateError = slot.RateErrorFiltered;
        double errorDeg = theta.Length() * 180.0 / Math.PI;

        // Pointing and roll apart (model z, mz, is the roll axis) so a large roll error does not starve pointing capture.
        double3 thetaPoint = theta - double3.Dot(theta, mz) * mz;
        double3 thetaRoll = settings.RollFree ? default : RollAngle(targetBody2Cci, cci2Body, mz) * mz;
        double withinRad = settings.IntegralWithinDeg * Math.PI / 180.0;

        // Per-axis conditional anti-windup: freeze integration on any axis whose downstream actuators fell short
        // in the same direction as the attitude error, while still allowing unwinding when the error reverses sign.
        if (dt > 0.0)
        {
            double3 intModel = Model(slot.Integral, mx, my, mz);
            double3 pointModel = Model(thetaPoint, mx, my, mz);
            double rollZ = double3.Dot(thetaRoll, mz);
            double cap = settings.IntegralMaxDegS * Math.PI / 180.0;
            AttitudeControlMath.UpdateIntegral(
                intModel.X, intModel.Y, intModel.Z,
                pointModel.X, pointModel.Y, rollZ,
                settings.RollFree, withinRad, cap, dt,
                slot.LastDemandModel.X, slot.LastDemandModel.Y, slot.LastDemandModel.Z,
                slot.LastShortfallModel.X, slot.LastShortfallModel.Y, slot.LastShortfallModel.Z,
                SaturatedShare,
                out double nextIntX, out double nextIntY, out double nextIntZ);
            slot.Integral = nextIntX * mx + nextIntY * my + nextIntZ * mz;
        }
        double rateErrorDegS = rateError.Length() * 180.0 / Math.PI;

        bool rcsNow = settings.UseRcs;
        bool phasePlane = settings.PhasePlaneRcs && settings.UseRcs;

        // The deadband, with hysteresis: out past it, back under half of it.
        if (slot.Feedback)
            slot.Feedback = !(errorDeg < 0.5 * settings.DeadbandDeg && rateErrorDegS < settings.RateDeadbandDegS);
        else
            slot.Feedback = errorDeg > settings.DeadbandDeg || rateErrorDegS > 2.0 * settings.RateDeadbandDegS;

        bool airKnown;
        double3 air;
        using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.AirMoment"))
            airKnown = TryAirMoment(vehicle, aero, momentLimiterPerPa, out air);
        double kp = settings.BandwidthRadS * settings.BandwidthRadS, kd = 2.0 * settings.Damping * settings.BandwidthRadS;
        // The roll's own gains where RollBandwidthRadS is set (an ascent: see Settings).
        double rollBandwidth = double.IsFinite(settings.RollBandwidthRadS) ? settings.RollBandwidthRadS : settings.BandwidthRadS;
        double kpRoll = rollBandwidth * rollBandwidth, kdRoll = 2.0 * settings.Damping * rollBandwidth;
        // The capture rate: kp/kd theta, which is the PD's own, capped - pointing and roll each on its own.
        double captureMax = settings.CaptureRateDegS * Math.PI / 180.0;
        double rollCaptureMax = captureMax;
        // Not under AuthorityCapture, whose square-root capture holds the roll to the authority itself: measuring the roll's here too (four solves every half second of game time) was thrown away (2026-11-21).
        if (double.IsFinite(settings.RollCaptureRateDegS) && !settings.AuthorityCapture)
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
        // The surfaces and the RCS together, before the phase plane takes each axis's owner alone below: what hand flying's rates are shares of (HandFlying).
        double3 spareTotal = default;
        double airAxisAccel = double.NaN;
        if (settings.AuthorityCapture || phasePlane)
        {
            // The acceleration the craft can spare about each model axis, after the trim it holds: the square-root law's a, and the surfaces' and the RCS's reach the axes' owners are picked by.
            using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.SpareAccel"))
                spare = SpareAccel(vehicle, slot, now, aero, momentLimiterPerPa, settings.UseSurfaces, settings.UseRcs, mx, my, mz, ixx, iyy, izz,
                                   airKnown ? -Model(air, mx, my, mz) : default);
            spareTotal = spare;
        }
        int rcsAxes = 0;
        if (phasePlane)
        {
            // THE OWNER OF EACH AXIS, one at a time as the stock flight computer gives an axis to its gimbals or its RCS. The surfaces take it once their spare torque after trim is what this law needs to hold the band, inertia x bandwidth^2 x band (a 1 rad/s loop pushing back a 1 deg error asks 1 deg/s^2), or once they match the RCS, whichever comes first; they give it back under RcsOwnerLeaveShare of that. Flown on the flaps alone (2026-11-22), the belly flop held 0.13 deg (90th percentile) from 1 kPa, with about 1 deg/s^2 spare, and 5.7 deg below it with 0.6: a tenth of the RCS, stock's share for gimbals, handed the flaps the axes at 0.5 kPa, where they waggled. No RCS on an axis: the surfaces keep it.
            double3 aeroSpare = settings.UseSurfaces ? slot.AeroSpareNm : default;
            // The need is taken on a fixed band, not the RCS's: a wider RCS band is propellant saved, and must not hand the RCS more axes. On the RCS's band, widening it from 1 to 4 deg quadrupled the need, and the RCS kept a Starship's pitch and yaw to 1.1 kPa, holding the air's steady 100 kN m on pulses (2026-11-24).
            double bandRad = OwnershipBandDeg * Math.PI / 180.0;
            for (int i = 0; i < 3; i++)
            {
                double rcsReach = Math.Min(Axis(slot.RcsPlusNm, i), Axis(slot.RcsMinusNm, i));
                double aeroReach = Axis(aeroSpare, i);
                double inertia = i == 0 ? ixx : i == 1 ? iyy : izz;
                double need = Math.Min(inertia * settings.BandwidthRadS * settings.BandwidthRadS * bandRad, rcsReach);
                bool owned = slot.RcsOwns[i];
                if (!(rcsReach > 0.0))
                    slot.RcsOwns[i] = false;
                else
                    slot.RcsOwns[i] = slot.RcsOwns[i] ? !(aeroReach >= need) : aeroReach < RcsOwnerLeaveShare * need;
                // A change of owner starts the RCS's phase plane afresh: its own and the surfaces' backup are different laws.
                if (slot.RcsOwns[i] != owned)
                {
                    slot.RcsState[i] = PhasePlaneRcs.State.Inside;
                    slot.RcsTrimNm[i] = 0.0;
                }
                if (slot.RcsOwns[i])
                {
                    rcsAxes |= 1 << i;
                    // The surfaces' integral has nothing to wind on an axis they do not steer, as stock clears it on the axes its gimbals leave.
                    double3 m = i == 0 ? mx : i == 1 ? my : mz;
                    slot.Integral -= double3.Dot(slot.Integral, m) * m;
                }
            }
            integralRoll = double3.Dot(slot.Integral, mz) * mz;
            integralPoint = slot.Integral - integralRoll;

            // THE OWNER'S AUTHORITY ALONE, axis by axis, for the capture and the turns planned on it (AirAxisAccel, the entry's bank slew): the surfaces' spare on their axes, the RCS's reach on its own. Summed, a turn was planned at a rate only both together could fly; the surfaces could not follow it alone, and the RCS's backup fired through every turn to make up the difference (offline, 90-114 pulses a minute against 8-17).
            double3 own = default;
            for (int i = 0; i < 3; i++)
            {
                double inertia = i == 0 ? ixx : i == 1 ? iyy : izz;
                double reach = slot.RcsOwns[i] ? Math.Min(Axis(slot.RcsPlusNm, i), Axis(slot.RcsMinusNm, i)) : Axis(aeroSpare, i);
                double accel = inertia > 0.0 ? Math.Max(reach, 0.0) / inertia : double.NaN;
                own = i == 0 ? new double3(accel, own.Y, own.Z) : i == 1 ? new double3(own.X, accel, own.Z) : new double3(own.X, own.Y, accel);
            }
            spare = own;
        }
        if (settings.AuthorityCapture)
        {
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

        // Command the full demand to lit gimbals and pass the 1-step rate-limited slew residual to surfaces and RCS.
        double stepS = dt > 0.0 ? dt : (vehicle.KinematicMeasurements.DeltaTime > 0.0 && vehicle.KinematicMeasurements.DeltaTime <= 0.2 ? vehicle.KinematicMeasurements.DeltaTime : 0.05);
        double3 remaining = demand, gimbalsBody = default;
        if (settings.UseGimbals)
        {
            using var _gimbals = new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.Gimbals");
            gimbalsBody = KsaGimbalControl.PredictLsqReachable(vehicle, demand, stepS, out _);
            KsaGimbalControl.SetLsq(vehicle, demand);
            slot.GimbalsOwned = true;
            remaining -= gimbalsBody;
        }
        else if (slot.GimbalsOwned)
        {
            KsaGimbalControl.Disengage(vehicle);
            slot.GimbalsOwned = false;
        }

        // PHASE-PLANE RCS: the surfaces are asked for the demand on the axes they own and only the trim (the air's moment and the gyroscopic torque) on the RCS's, alone; the RCS fires by the phase plane below. On the RCS's axes the trim carries the RCS's own steady work too (RcsTrimNm): the air's moment the tables miss, which the RCS would otherwise hold on pulses all the way down.
        if (phasePlane)
        {
            double3 trim = gyro - air;
            double3 ask = default;
            for (int i = 0; i < 3; i++)
            {
                double3 m = i == 0 ? mx : i == 1 ? my : mz;
                ask += slot.RcsOwns[i] ? (double3.Dot(trim, m) + slot.RcsTrimNm[i]) * m : double3.Dot(remaining, m) * m;
            }
            remaining = ask;
        }
        // One programme for fins and RCS (the RCS as the fins' backup): the fins are commanded and the joint RCS duties are consumed directly when solved together.
        RcsTorqueAllocator rcsColumns = null;
        ThrusterController[] rcsModules = null;
        if (rcsNow && !phasePlane)
            using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.RcsColumns"))
                rcsColumns = KsaRcsTorqueControl.Measure(vehicle, mx, my, mz, out rcsModules);
        KsaAeroTorqueControl.Report surfaces = default;
        Span<double> priority = stackalloc double[3];
        RcsPriority.CopyTo(priority);
        priority[2] = Math.Max(settings.RollPriority, 0.0);
        if (settings.UseSurfaces)
            surfaces = KsaAeroTorqueControl.Command(vehicle, aero, remaining, mx, my, mz, priority, momentLimiterPerPa,
                                                    stepS, rcsColumns, settings.SurfaceSmoothS,
                                                    slot.RcsKnown ? slot.RcsPlusNm : default, slot.RcsKnown ? slot.RcsMinusNm : default);
        else
            KsaAeroTorqueControl.Release(vehicle);
        double3 surfacesBody = surfaces.GivenNm.X * mx + surfaces.GivenNm.Y * my + surfaces.GivenNm.Z * mz;
        double3 rcsAsked = remaining - surfacesBody;
        int rcsFiring = 0;
        Span<double> rcsPriority = stackalloc double[3];
        priority.CopyTo(rcsPriority);
        if (phasePlane)
        {
            rcsAsked = PhasePlaneTorque(slot, thetaPoint + thetaRoll, -rawRateError, mx, my, mz, ixx, iyy, izz, dt, holdS, settings, out rcsFiring);
            for (int i = 0; i < 3; i++)
                if ((rcsFiring & (1 << i)) == 0)
                    rcsPriority[i] = RcsSpillPriority;
        }
        KsaRcsTorqueControl.Report rcs = default;
        if (rcsNow && (!phasePlane || rcsFiring != 0))
        {
            if (!phasePlane && rcsColumns != null && surfaces.BackupDuty != null && surfaces.BackupDuty.Length == rcsColumns.Count)
                rcs = KsaRcsTorqueControl.CommandDuties(vehicle, rcsAsked, mx, my, mz, rcsColumns, rcsModules, surfaces.BackupDuty);
            else
                rcs = rcsColumns != null
                    ? KsaRcsTorqueControl.Command(vehicle, rcsAsked, mx, my, mz, rcsPriority, rcsColumns, rcsModules)
                    : KsaRcsTorqueControl.Command(vehicle, rcsAsked, mx, my, mz, rcsPriority, jetMapFallback: phasePlane);
        }
        else
            KsaRcsTorqueControl.Release(vehicle);
        if (phasePlane && settings.UseSurfaces && dt > 0.0)
        {
            double k = Math.Min(Math.Max(dt, holdS) / RcsTrimShiftS, 1.0);
            for (int i = 0; i < 3; i++)
            {
                if (!slot.RcsOwns[i])
                    continue;
                double reach = Math.Max(Math.Abs(Axis(slot.AeroSpareNm, i)) + Math.Abs(slot.RcsTrimNm[i]), 0.0);
                slot.RcsTrimNm[i] = Math.Clamp(slot.RcsTrimNm[i] + k * Axis(rcs.GivenNm, i), -reach, reach);
            }
        }
        // Short of the demand by more than SaturatedShare of it: the actuators are at their limits. With the phase plane, only the surfaces' own axes count: the RCS's are held in a band, not to the demand.
        double3 givenBody = gimbalsBody + surfacesBody + rcs.GivenNm.X * mx + rcs.GivenNm.Y * my + rcs.GivenNm.Z * mz;
        if (phasePlane)
        {
            slot.LastDemandModel = Model(remaining, mx, my, mz);
            slot.LastShortfallModel = slot.LastDemandModel - surfaces.GivenNm;
            slot.Saturated = remaining.Length() > 0.0 && (remaining - surfacesBody).Length() > SaturatedShare * remaining.Length();
        }
        else
        {
            slot.LastDemandModel = Model(demand, mx, my, mz);
            slot.LastShortfallModel = slot.LastDemandModel - Model(givenBody, mx, my, mz);
            slot.Saturated = demand.Length() > 0.0 && (demand - givenBody).Length() > SaturatedShare * demand.Length();
        }

        slot.Last = new Report(errorDeg, rateErrorDegS, !slot.Feedback,
            Model(demand, mx, my, mz), Model(air, mx, my, mz), surfaces.GivenNm, Model(rcsAsked, mx, my, mz),
            rcs.GivenNm, rcs.FlowKgS, airKnown, Model(integral, mx, my, mz), surfaces.CommandDeg, surfaces.NowDeg,
            Model(gimbalsBody, mx, my, mz), spare, airAxisAccel, slot.Saturated, rcsAxes, rcsFiring, spareTotal);
        return slot.Last;
    }

    /// <summary>The time the RCS's steady work on its own axes is moved onto the surfaces over, s (Slot.RcsTrimNm).</summary>
    private const double RcsTrimShiftS = 10.0;

    /// <summary>The band the ownership rule's need is reckoned on, deg: the 1 deg it was tuned at (2026-11-22), whatever the RCS's own band.</summary>
    private const double OwnershipBandDeg = 1.0;

    /// <summary>The surfaces give an axis back to the RCS once their spare torque falls under this share of what took it (the ownership rule's need).</summary>
    private const double RcsOwnerLeaveShare = 0.7;

    /// <summary>On the surfaces' axes the RCS's backup fires only outside a band this many times wider than its own, and only while the error there still grows (PhasePlaneRcs.Backup).</summary>
    private const double RcsBackupBandScale = 3.0;

    /// <summary>The RCS's rate band is never narrower than its angle band crossed in this time, s: a band held to the rate bit alone (0.007 deg/s on the belly flop, the edge 0.57 deg out) was crossed by every air and flap disturbance, and the RCS pulsed constantly (2026-11-22). At 1 deg the band coasts at 0.05 deg/s, its edges about 1 deg out.</summary>
    private const double RcsDriftCrossS = 20.0;

    /// <summary>The shortest pulse the rate bit is worked from, s, where the thrusters name none shorter.</summary>
    private const double RcsMinPulseFloorS = 0.02;

    /// <summary>A thruster counts towards an axis's smallest pulse when it turns the axis by at least this share of the strongest one: a thruster that grazes it is not one the axis would fire.</summary>
    private const double RcsBitShare = 0.1;

    /// <summary>The weight of a coasting axis's torque in the RCS's programme while another axis fires, against 1 for the firing ones: small, so a firing axis's pulse is not refused for what it spills.</summary>
    private const double RcsSpillPriority = 0.05;

    /// <summary>
    /// The RCS's torque this step, axis by axis in the model's axes: on the RCS's own axes by the phase plane (PhasePlaneRcs.Step) with the settings' band and its rate band floored at the band over RcsDriftCrossS; on the surfaces' only as a backup (PhasePlaneRcs.Backup), past RcsBackupBandScale times the band and while the error still grows. Each axis's rate change is asked for as a torque over the time it is held: one of the thrusters' firing periods (KsaRcsTorqueControl.PeriodSec), which the worker fires a request over, or the whole step where that is longer (<paramref name="holdS"/>, a warped frame, fired period after period), and held to the RCS's reach. <paramref name="theta"/> is the rotation onto the target (body axes), <paramref name="rateFromTarget"/> the body's rate less the target's; the phase plane's error is the craft less the target, so the angle is -theta. The target's own turning rate is left out of the rate band, as the error's rate is already the target's: in an entry it would widen the band to tens of degrees through every bank slew. The axes that fire, as bits (x 1, y 2, z 4), into <paramref name="firing"/>.
    /// </summary>
    private static double3 PhasePlaneTorque(Slot slot, double3 theta, double3 rateFromTarget, double3 mx, double3 my, double3 mz,
                                            double ixx, double iyy, double izz, double dt, double holdS, in Settings settings, out int firing)
    {
        firing = 0;
        double3 torque = default;
        double period = KsaRcsTorqueControl.PeriodSec, pulse = Math.Max(slot.RcsMinPulseS, RcsMinPulseFloorS);
        double hold = Math.Max(holdS, period);
        for (int i = 0; i < 3; i++)
        {
            double3 m = i == 0 ? mx : i == 1 ? my : mz;
            double inertia = i == 0 ? ixx : i == 1 ? iyy : izz;
            double plus = Axis(slot.RcsPlusNm, i), minus = Axis(slot.RcsMinusNm, i);
            double a = inertia > 0.0 ? Math.Min(plus, minus) / inertia : 0.0;
            double band = settings.RcsAngleBandDeg * Math.PI / 180.0;
            // The rate bit is one thruster's shortest pulse, as stock's: the whole axis's authority times it made the rate bit, and with it the turnaround (at least 10 s of drift at it), several times too wide (2026-11-22: a band of several degrees each way).
            double impulse = Axis(slot.RcsMinImpulseNms, i);
            double rateBit = inertia > 0.0 && impulse > 0.0 && double.IsFinite(impulse) ? impulse / inertia : a * pulse;
            double angle = -double3.Dot(theta, m), rate = double3.Dot(rateFromTarget, m);
            double dw;
            if (slot.RcsOwns[i])
            {
                var p = new PhasePlaneRcs.Axis(a, rateBit, 0.0, band, settings.RcsRateLimitDegS * Math.PI / 180.0, hold, band / RcsDriftCrossS);
                dw = PhasePlaneRcs.Step(p, angle, rate, ref slot.RcsState[i]);
            }
            else
                dw = a > 0.0 ? PhasePlaneRcs.Backup(0.5 * RcsBackupBandScale * band, rateBit, angle, rate, ref slot.RcsState[i]) : 0.0;
            if (dw == 0.0)
                continue;
            firing |= 1 << i;
            torque += Math.Clamp(inertia * dw / hold, -minus, plus) * m;
        }
        return torque;
    }

    private static double Axis(double3 v, int i) => i == 0 ? v.X : i == 1 ? v.Y : v.Z;

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
            Array.Clear(slot.RcsState);
            Array.Clear(slot.RcsOwns);
            slot.Last = default;
            slot.LastT = double.NaN;
            slot.RateKnown = false;
            slot.Integral = default;
            slot.Saturated = false;
            slot.LastDemandModel = default;
            slot.LastShortfallModel = default;
            slot.FinKnown = false;
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
        double liveA = vehicle.PhysicsEnvironment.SpeedOfSound;
        double localMach = liveA > 1.0 ? speed / liveA : aero.Atmosphere.Mach(speed);
        double mach = Math.Clamp(localMach, forceTable.MachMin, forceTable.MachMax);
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

    /// <summary>
    /// When the surfaces' reach is measured again in full (six solves on the main thread): Mach moved by FinRemeasureMachShare of itself (at least FinRemeasureMachMin), the airflow turned by FinRemeasureAirDeg in body axes, or FinRemeasureS of real time passed. In between it is scaled by q over the moment limiter. Measuring every half second of game time, more often under warp, slowed the game (2026-11-21).
    /// </summary>
    private const double FinRemeasureMachShare = 0.05, FinRemeasureMachMin = 0.05, FinRemeasureAirDeg = 2.0, FinRemeasureS = 5.0;

    /// <summary>The least real time between two measurements of the surfaces' reach, s, however the condition moves: an angle of attack that swings back and forth must not have them measured every frame.</summary>
    private const double FinRemeasureMinS = 0.5;

    /// <summary>The dynamic pressure under which the surfaces' reach is not measured, Pa: KsaAeroTorqueControl counts them as giving nothing there.</summary>
    private const double FinMeasureMinQPa = 50.0;

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
    /// The angular acceleration the craft can spare about each model axis, rad/s^2: the most its surfaces give that way and the other (measured as the flight condition moves, scaled by q between) and its RCS (measured as the mass and centre of mass move), less the trim torque <paramref name="trimModel"/> they hold against the air (model axes, N m), the smaller way round, over the inertia. NaN on an axis with no inertia.
    /// </summary>
    private static double3 SpareAccel(Vehicle vehicle, Slot slot, double now, KsaAeroSweep.Result aero, double momentLimiterPerPa, bool surfaces, bool useRcs,
                                      double3 mx, double3 my, double3 mz, double ixx, double iyy, double izz, double3 trimModel)
    {
        // The surfaces' reach is q times coefficients that follow Mach and the airflow's direction, over the game's moment limiter: measured in full only when those move (or every FinRemeasureS of real time), and scaled by q / (1 + k q) in between.
        float3 airF = PhysicsStates.ComputeAirVelocityBody(vehicle.BubbleOrigin, vehicle.KinematicStates, vehicle.PhysicsEnvironment);
        double3 airBody = double3.Unpack(in airF);
        double speed = airBody.Length();
        double q = 0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * speed * speed;
        double qEff = q / (1.0 + (double.IsFinite(momentLimiterPerPa) && momentLimiterPerPa > 0.0 ? momentLimiterPerPa * q : 0.0));
        double liveA = vehicle.PhysicsEnvironment.SpeedOfSound;
        double mach = speed > 0.0
            ? (liveA > 1.0 ? speed / liveA : aero?.Atmosphere != null ? aero.Atmosphere.Mach(speed) : double.NaN)
            : double.NaN;
        double3 airDir = speed > 1.0 ? airBody / speed : default;
        // Under FinMeasureMinQPa the surfaces give nothing (KsaAeroTorqueControl's own floor), so in thin air or none there is nothing to measure.
        bool finAir = surfaces && q > FinMeasureMinQPa;
        if (finAir)
        {
            long ticks = System.Diagnostics.Stopwatch.GetTimestamp();
            bool machMoved = double.IsFinite(mach) && double.IsFinite(slot.FinMach)
                ? Math.Abs(mach - slot.FinMach) > Math.Max(FinRemeasureMachShare * slot.FinMach, FinRemeasureMachMin)
                : double.IsFinite(mach) != double.IsFinite(slot.FinMach);
            double sinceS = (ticks - slot.FinAtTicks) / (double)System.Diagnostics.Stopwatch.Frequency;
            bool stale = !slot.FinKnown
                         || sinceS > FinRemeasureS
                         || (sinceS > FinRemeasureMinS
                             && (machMoved || double3.Dot(airDir, slot.FinAirDir) < Math.Cos(FinRemeasureAirDeg * Math.PI / 180.0)));
            if (stale)
            {
                using var _fin = new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.FinReach");
                double[] plus = new double[3], minus = new double[3];
                AeroTorqueAllocator fins = KsaAeroTorqueControl.Measure(vehicle, aero, mx, my, mz, momentLimiterPerPa, slot.Scratch, out _);
                if (fins != null)
                    for (int axis = 0; axis < 3; axis++)
                    {
                        plus[axis] = Math.Max(fins.Characterise(axis, 1).MaxTorqueNm, 0.0);
                        minus[axis] = Math.Max(fins.Characterise(axis, -1).MaxTorqueNm, 0.0);
                    }
                slot.FinPlusNm = new double3(plus[0], plus[1], plus[2]);
                slot.FinMinusNm = new double3(minus[0], minus[1], minus[2]);
                slot.FinQEff = fins != null ? qEff : 0.0;
                slot.FinMach = mach;
                slot.FinAirDir = airDir;
                slot.FinAtTicks = ticks;
                slot.FinKnown = true;
            }
        }
        double finScale = finAir && slot.FinKnown && slot.FinQEff > 0.0 ? qEff / slot.FinQEff : 0.0;
        // The RCS's reach does not: its thrusters' levers change only as the mass and centre of mass do, so it is measured once and again only past RcsRemeasureShare of either.
        double mass = vehicle.TotalMass;
        float3 comF = vehicle.CenterOfMassAsmbF;
        var com = new double3(comF.X, comF.Y, comF.Z);
        double gyration = mass > 0.0 ? Math.Sqrt(Math.Max(Math.Max(ixx, iyy), izz) / mass) : double.NaN;
        bool moved = useRcs && (!slot.RcsKnown
                     || !(Math.Abs(mass - slot.RcsMassKg) <= RcsRemeasureShare * slot.RcsMassKg)
                     || (double.IsFinite(gyration) && !((com - slot.RcsComAsmb).Length() <= RcsRemeasureShare * gyration)));
        if (moved)
        {
            using var _rcs = new AdvancedFlightComputer.Core.PerfTracker.Scope("Attitude.RcsReach");
            double[] plus = new double[3], minus = new double[3];
            RcsTorqueAllocator rcs = KsaRcsTorqueControl.Measure(vehicle, mx, my, mz, out ThrusterController[] modules);
            if (rcs != null)
                for (int axis = 0; axis < 3; axis++)
                {
                    plus[axis] = Math.Max(rcs.Characterise(axis, 1).MaxTorqueNm, 0.0);
                    minus[axis] = Math.Max(rcs.Characterise(axis, -1).MaxTorqueNm, 0.0);
                }
            double shortest = double.PositiveInfinity;
            if (modules != null)
                foreach (ThrusterController module in modules)
                    if (module != null && module.MinimumPulseTime > 0f)
                        shortest = Math.Min(shortest, module.MinimumPulseTime);
            slot.RcsMinPulseS = double.IsFinite(shortest) ? shortest : 0.0;
            // The smallest single pulse each axis can be given: a thruster that turns the axis by at least RcsBitShare of the strongest one does, fired for its shortest pulse.
            double[] bit = { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };
            if (rcs != null && modules != null && modules.Length == rcs.Count)
                for (int axis = 0; axis < 3; axis++)
                {
                    double strongest = 0.0;
                    for (int i = 0; i < rcs.Count; i++)
                        strongest = Math.Max(strongest, Math.Abs(rcs.Torque[3 * i + axis]));
                    for (int i = 0; i < rcs.Count; i++)
                    {
                        double t = Math.Abs(rcs.Torque[3 * i + axis]);
                        if (!(t >= RcsBitShare * strongest) || !(t > 0.0) || !(rcs.Flow[i] > 0.0))
                            continue;
                        double pulse = Math.Max(modules[i] != null ? modules[i].MinimumPulseTime : 0.0, RcsMinPulseFloorS);
                        bit[axis] = Math.Min(bit[axis], t * pulse);
                    }
                }
            slot.RcsMinImpulseNms = new double3(bit[0], bit[1], bit[2]);
            if (GuidanceLog.Enabled)
            {
                static string AxisText(double plus, double minus, double inertia, double bitNms)
                {
                    double acc = inertia > 0.0 ? Math.Min(plus, minus) / inertia : 0.0, rb = inertia > 0.0 && double.IsFinite(bitNms) ? bitNms / inertia : double.NaN;
                    double turn = acc > 0.0 && double.IsFinite(rb) ? Math.Max(rb * rb / acc, 10.0 * rb) : double.NaN;
                    return FormattableString.Invariant($"{plus / 1000:F0}/{minus / 1000:F0} kN m, {acc * 180 / Math.PI:F2} deg/s2, rate bit {rb * 180 / Math.PI:F3} deg/s, turnaround {turn * 180 / Math.PI:F2} deg");
                }
                GuidanceLog.Debug(vehicle, "RCS reach for the phase plane: x " + AxisText(plus[0], minus[0], ixx, bit[0]) + "; y " + AxisText(plus[1], minus[1], iyy, bit[1])
                    + "; z " + AxisText(plus[2], minus[2], izz, bit[2]) + FormattableString.Invariant($"; shortest pulse {slot.RcsMinPulseS:F3} s."));
            }
            slot.RcsPlusNm = new double3(plus[0], plus[1], plus[2]);
            slot.RcsMinusNm = new double3(minus[0], minus[1], minus[2]);
            slot.RcsMassKg = mass;
            slot.RcsComAsmb = com;
            slot.RcsKnown = true;
        }
        double3 aeroUp = finScale * slot.FinPlusNm, aeroDown = finScale * slot.FinMinusNm;
        static double SpareNm(double up, double down, double trim) => Math.Max(Math.Min(up - trim, down + trim), 0.0);
        slot.AeroSpareNm = new double3(SpareNm(aeroUp.X, aeroDown.X, trimModel.X), SpareNm(aeroUp.Y, aeroDown.Y, trimModel.Y), SpareNm(aeroUp.Z, aeroDown.Z, trimModel.Z));
        double3 rcsUp = useRcs ? slot.RcsPlusNm : default, rcsDown = useRcs ? slot.RcsMinusNm : default;
        double3 up = finScale * slot.FinPlusNm + rcsUp, down = finScale * slot.FinMinusNm + rcsDown;
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
        double cx = Math.Abs(double3.Dot(u, mx));
        if (cx > 1e-3 && double.IsFinite(spare.X)) best = Math.Min(best, spare.X / cx);
        double cy = Math.Abs(double3.Dot(u, my));
        if (cy > 1e-3 && double.IsFinite(spare.Y)) best = Math.Min(best, spare.Y / cy);
        double cz = Math.Abs(double3.Dot(u, mz));
        if (cz > 1e-3 && double.IsFinite(spare.Z)) best = Math.Min(best, spare.Z / cz);
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
                                            out double deltaRad, out double momentNowNm, out double momentAtNm, double altitude = 0.0)
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
        double mach = aero.Atmosphere.Mach(speed, altitude);
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
