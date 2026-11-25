#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

/// <summary>The PILOT strip's buttons (Content/HandFlyingPanel.xml): who flies the pitch, yaw and roll keys, and how hard AFC turns for them.</summary>
public enum AfcPilot { Stock, Afc, Rate30, Rate50, Rate80 }

/// <summary>
/// HAND FLYING THROUGH AFC'S ATTITUDE CONTROLLER - AFC MANUAL INTERVENTION (2026-11-25, the user's ask): with PILOT on AFC MANUAL INTERVENTION the pitch, yaw and roll keys are turning-rate demands, flown by AFC's own controller (AttitudeController, the one orbital entry flies) - rate command, attitude hold, the way a fly-by-wire airliner flies: a key held turns the craft about that axis, let go it holds the attitude it has.
///
/// THE KEYS are read where the game keeps them (Vehicle._manualControlInputs' ThrusterCommandFlags, set on a key press and cleared on its release) and hidden from the stock flight computer for the step - cleared in the PrepareWorker prefix the guidance step runs in, put back in its postfix - so stock's RCS and gimbals do not fire for them as well. They are taken in stock's own control frame (FlightComputer.Ctrl2Asmb: x roll, y pitch, z yaw; RollRight, PitchUp, YawRight positive, as stock's ManualAxisOverride), so they turn the craft the way they always have, the engine's frame while it thrusts.
///
/// THE RATES (RATE 30 / 50 / 80 %) are shares of what the craft can turn with now, axis by axis: the angular acceleration its flaps (by q) and RCS together can spare after trim (AttitudeController's SpareTotalRadS2, measured under AuthorityCapture; not each axis's owner alone, which the controller's own capture uses and which left the keys sluggish: a Starship spared 1.7-1.9 deg/s^2 in pitch and yaw, 2.7 deg/s at 50 %). A key spins the axis up at that share and its release stops it at the same share; the rate itself is held to what that share can stop from in <see cref="StopS"/>. Before the controller has measured the craft, <see cref="FallbackAccelDegS2"/> stands in.
///
/// FLOWN AS BELLYFLOP FLIES ITS PLAN (2026-11-25, the user's ask: "exactly like the flop attitude controller, but with rates coming from me"): the keys turn a COMMANDED attitude at their rate, at once on a press and to zero at once on a release; that command goes through bellyflop's critically damped second-order filter (<see cref="CommandOmega"/>, read 2/omega ahead, BellyflopAttitude), and the filtered attitude and its rate are flown with orbital entry's and bellyflop's own controller settings (FlyOwnAttitude's: capture at ROLL RATE or 10 deg/s, authority-aware, phase-plane RCS on ENTRY RCS DEAD BAND, ENTRY RCS). The flop's planner never asks for more than the craft can do, and the keys might, so two guards of its own stay (below).
///
/// THE TARGET is the attitude the keys have turned to: it moves at the demanded rate, fed forward to the controller as the target's own rate, and stops where the keys are let go. ITS RATE FOLLOWS THE CRAFT: on each axis it is held within <see cref="FollowS"/> of the chosen effort of the rate the craft actually turns at, so it never runs away from a craft that cannot keep up, and a released key stops it as fast as the craft stops. Past <see cref="SoftLeadDeg"/> of lead it slows, to a stop at <see cref="MaxLeadDeg"/>. (2026-11-25: a Starship's measured roll authority, 12-18 deg/s^2 over its small roll inertia, asked for 18-25 deg/s of roll; the target ran 15 deg ahead, stopped dead, and jumped on again as the craft caught up - a stop-start the pilot read as sluggish and oscillating.)
///
/// THE LOG (navbox-logs, -hand-flying.csv): what was asked and what was flown, axis by axis, every <see cref="LogEveryS"/> of game time while it flies.
///
/// IT OVERRIDES EVERYTHING while it is on (the user's ask): the stock flight computer is held at MANUAL, a running AFC programme is stopped as a takeover stops it, its engine left as it is (GuidanceWindow.StopForManualIntervention), and so is one engaged while it stays on; a running RCS burn is asked to cancel. It stands down, and the keys go to stock untouched, only while PILOT is STOCK, on a craft the player is not flying, and on one another mod flies (VehicleControlOwnership.KfsOwns).
/// </summary>
internal static class HandFlying
{
    /// <summary>Seconds the turn rate can be stopped in at the chosen share of the authority: the rate cap. Own choice: quick enough to feel direct, slow enough that a released key does not overshoot far.</summary>
    internal const double StopS = 3.0;

    /// <summary>The angular acceleration assumed on an axis before the controller has measured the craft, deg/s^2.</summary>
    internal const double FallbackAccelDegS2 = 1.0;

    /// <summary>The fastest any axis is turned, deg/s, whatever the authority.</summary>
    internal const double MaxRateDegS = 30.0;

    /// <summary>How far the target may lead the craft, deg: it slows past <see cref="SoftLeadDeg"/> and stands still here.</summary>
    internal const double MaxLeadDeg = 15.0;
    internal const double SoftLeadDeg = 7.5;

    /// <summary>Bellyflop's command filter's natural frequency, rad/s (Bellyflop.BellyflopCommandOmega): a step settles in about 2 s, and the command is read 2/omega ahead.</summary>
    internal const double CommandOmega = 2.0;

    /// <summary>How far the target's rate may run from the craft's own, as the seconds of the chosen effort it takes to close it. Own choice: a second keeps the target with the craft and still lets a key build the rate at the full effort.</summary>
    internal const double FollowS = 1.0;

    /// <summary>The log's row interval, s of game time.</summary>
    internal const double LogEveryS = 0.25;

    private const ThrusterMapFlags RotationKeys = ThrusterMapFlags.RollRight | ThrusterMapFlags.RollLeft | ThrusterMapFlags.PitchUp
                                                | ThrusterMapFlags.PitchDown | ThrusterMapFlags.YawRight | ThrusterMapFlags.YawLeft;

    private sealed class Slot
    {
        public bool Active;
        /// <summary>The filtered attitude flown (body to CCI), and the commanded one the keys turn, which the filter follows.</summary>
        public doubleQuat Target, Raw;
        /// <summary>The filtered attitude's turning rate, CCI, rad/s: the controller's feedforward.</summary>
        public double3 FiltRateCci;
        /// <summary>The target's turning rate, model axes (KsaFrameBridge.BodyAxes), rad/s.</summary>
        public double3 RateModel;
        public double LastT = double.NaN;
        /// <summary>The rotation keys hidden from stock this step, put back after it (<see cref="AfterPrepare"/>).</summary>
        public ThrusterMapFlags Hidden;
        /// <summary>The log of this stretch of hand flying, and when its last row was taken.</summary>
        public HandFlyingLog Log;
        public double LogAt = double.NegativeInfinity;
    }

    private static readonly ConditionalWeakTable<Vehicle, Slot> Slots = new();

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.Field(typeof(Vehicle), "_manualControlInputs") is { } f ? AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>(f) : null;

    internal static bool Available => Inputs != null;

    // ---- the PILOT buttons ----

    internal static double ShareOf(AfcPilot rate) => rate switch { AfcPilot.Rate30 => 0.30, AfcPilot.Rate80 => 0.80, _ => 0.50 };

    internal static void Press(Vehicle vehicle, AfcPilot button)
    {
        VehicleAutopilotState s = VehicleAutopilotState.For(vehicle);
        switch (button)
        {
            case AfcPilot.Stock:
                if (s.HandFlyAfc)
                    StandDown(vehicle, release: !s.ControlAcquired);
                s.HandFlyAfc = false;
                break;
            case AfcPilot.Afc:
                s.HandFlyAfc = true;
                break;
            default:
                s.HandFlyShare = ShareOf(button);
                break;
        }
    }

    internal static bool IsLit(Vehicle vehicle, AfcPilot button)
    {
        if (!VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState s))
            return button is AfcPilot.Stock or AfcPilot.Rate50;
        return button switch
        {
            AfcPilot.Stock => !s.HandFlyAfc,
            AfcPilot.Afc => s.HandFlyAfc,
            _ => Math.Abs(s.HandFlyShare - ShareOf(button)) < 1e-9,
        };
    }

    // ---- the step ----

    /// <summary>One step of hand flying, from the guidance step in Vehicle.PrepareWorker's prefix, after the guidance modes. Main thread.</summary>
    internal static void Step(Vehicle vehicle)
    {
        if (Inputs == null || vehicle == null)
            return;
        Slot slot = Slots.GetValue(vehicle, static _ => new Slot());
        slot.Hidden = ThrusterMapFlags.None;
        if (!VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState s) || !s.HandFlyAfc)
        {
            if (slot.Active)
                StandDown(vehicle, release: s == null || !s.ControlAcquired);
            return;
        }
        FlightComputer fc = vehicle.FlightComputer;
        bool flies = ReferenceEquals(vehicle, Program.ControlledVehicle)
                     && vehicle.IsControllable && !VehicleControlOwnership.KfsOwns(vehicle)
                     && vehicle.Orbit?.Parent != null;
        if (!flies)
        {
            if (slot.Active)
                StandDown(vehicle, release: !s.ControlAcquired);
            return;
        }

        // MANUAL INTERVENTION OVERRIDES: a running AFC programme stops (its engine left as it is), an RCS burn is asked to cancel and flies nothing more meanwhile, and the stock flight computer goes to MANUAL, out of any burn or docking frame.
        if (s.ControlAcquired)
            GuidanceWindow.StopForManualIntervention(vehicle);
        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution exec) && exec.IsActive)
        {
            RcsExecutor.RequestCancel(exec, "AFC MANUAL INTERVENTION");
            return;
        }
        if (fc.AttitudeMode != FlightComputerAttitudeMode.Manual)
            fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
        if (fc.AttitudeFrame is VehicleReferenceFrame.BurnBody or VehicleReferenceFrame.Dock)
            fc.SetInertialNullRot();

        double now = Universe.GetElapsedSeconds();
        double dt = double.IsFinite(slot.LastT) ? Math.Clamp(now - slot.LastT, 0.0, 0.2) : 0.0;
        slot.LastT = now;
        doubleQuat body2Cci = KsaFrameBridge.BodyToCci(vehicle);
        if (!slot.Active)
        {
            // Taken where the craft is, the filter turning as the craft turns now, as bellyflop's filter starts from the angles flown at its take-over: a craft taken turning is brought to rest smoothly onto where it was taken.
            slot.Active = true;
            slot.Target = slot.Raw = body2Cci;
            slot.FiltRateCci = vehicle.BodyRates.Transform(body2Cci);
            slot.RateModel = default;
        }

        // THE KEYS, in stock's control frame, then hidden from stock for this step.
        ref ManualControlInputs inputs = ref Inputs(vehicle);
        ThrusterMapFlags keys = inputs.ThrusterCommandFlags & RotationKeys;
        slot.Hidden = keys;
        inputs.ThrusterCommandFlags &= ~RotationKeys;
        static double Axis(ThrusterMapFlags k, ThrusterMapFlags plus, ThrusterMapFlags minus)
            => ((k & plus) != 0 ? 1.0 : 0.0) - ((k & minus) != 0 ? 1.0 : 0.0);
        var ctrl = new double3(Axis(keys, ThrusterMapFlags.RollRight, ThrusterMapFlags.RollLeft),
                               Axis(keys, ThrusterMapFlags.PitchUp, ThrusterMapFlags.PitchDown),
                               Axis(keys, ThrusterMapFlags.YawRight, ThrusterMapFlags.YawLeft));
        // Stock's ctrl axes into the craft's (assembly) axes, then into the model axes the authority is measured on.
        double3 askAsmb = ctrl.Transform(fc.Ctrl2Asmb);
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);

        // THE AUTHORITY, model axes: what the controller measured the craft can spare, at the chosen share.
        double share = Math.Clamp(s.HandFlyShare, 0.05, 1.0);
        AttitudeController.Report last = AttitudeController.Last(vehicle);
        double3 spare = last.SpareTotalRadS2 is { } total && (total.X > 0.0 || total.Y > 0.0 || total.Z > 0.0) ? total : last.SpareAccelRadS2;
        double fallback = FallbackAccelDegS2 * Math.PI / 180.0;
        double Accel(double a) => share * (double.IsFinite(a) && a > 1e-6 ? a : fallback);
        var accel = new double3(Accel(spare.X), Accel(spare.Y), Accel(spare.Z));
        double maxRate = MaxRateDegS * Math.PI / 180.0;
        var cap = new double3(Math.Min(accel.X * StopS, maxRate), Math.Min(accel.Y * StopS, maxRate), Math.Min(accel.Z * StopS, maxRate));

        // THE RATE, each model axis on its own: while a key turns it, the keys' rate at once, held near the rate the craft actually turns at (see the class summary: ITS RATE FOLLOWS THE CRAFT); with no key on it, zero at once. The filter below shapes both, as it shapes bellyflop's plans.
        var want = new double3(double3.Dot(askAsmb, mx), double3.Dot(askAsmb, my), double3.Dot(askAsmb, mz));
        double3 w = vehicle.BodyRates;
        var flown = new double3(double3.Dot(w, mx), double3.Dot(w, my), double3.Dot(w, mz));
        static double AxisRate(double wantAxis, double capAxis, double accelAxis, double craft)
            => Math.Abs(wantAxis) < 1e-3 ? 0.0 : Math.Clamp(wantAxis * capAxis, craft - accelAxis * FollowS, craft + accelAxis * FollowS);
        slot.RateModel = new double3(AxisRate(want.X, cap.X, accel.X, flown.X),
                                     AxisRate(want.Y, cap.Y, accel.Y, flown.Y),
                                     AxisRate(want.Z, cap.Z, accel.Z, flown.Z));

        // A TARGET THE CRAFT CANNOT REACH comes to it: never more than MaxLeadDeg away, so a craft that drifted on past where the keys stopped (one turning when it was taken, or one the air holds) is not left saturated against an attitude it will never reach (2026-11-25: taken rolling at 14 deg/s, it settled 28 deg from the target, which then stood still, and the keys did nothing until AFC was pressed again).
        if (LeadDeg(slot.Target, body2Cci) > MaxLeadDeg)
            slot.Target = Toward(body2Cci, slot.Target, MaxLeadDeg);
        if (LeadDeg(slot.Raw, body2Cci) > MaxLeadDeg)
            slot.Raw = Toward(body2Cci, slot.Raw, MaxLeadDeg);

        // THE COMMANDED ATTITUDE turns at that rate, about its own axes, slowing as it leads the craft past SoftLeadDeg.
        double lead = LeadDeg(slot.Raw, body2Cci);
        double ease = Math.Clamp((MaxLeadDeg - lead) / (MaxLeadDeg - SoftLeadDeg), 0.0, 1.0);
        double3 rawRateCci = (ease * (slot.RateModel.X * mx + slot.RateModel.Y * my + slot.RateModel.Z * mz)).Transform(slot.Raw);
        slot.Raw = Turned(slot.Raw, rawRateCci, dt);

        // BELLYFLOP'S FILTER (BellyflopAttitude), on the attitude: critically damped, second order, semi-implicit in 20 ms steps, towards the command read 2/omega ahead (the filter's lag on a ramp), so the flown attitude, its rate and its acceleration are all continuous.
        doubleQuat ahead = Turned(slot.Raw, rawRateCci, 2.0 / CommandOmega);
        double wn = CommandOmega;
        for (double done = 0.0; done < dt - 1e-9;)
        {
            double h = Math.Min(0.02, dt - done);
            done += h;
            slot.FiltRateCci += h * (wn * wn * RotationVector(slot.Target, ahead) - 2.0 * wn * slot.FiltRateCci);
            slot.Target = Turned(slot.Target, slot.FiltRateCci, h);
        }
        double3 rateCci = slot.FiltRateCci;

        // THE CONTROLLER, on every actuator the craft has: flaps in the air (with its sweep's tables, kept current while it flies), RCS when the player has it on, gimbals while an engine burns.
        IParentBody parent = vehicle.Orbit.Parent;
        if (parent.GetPhysicalAtmosphere() is { Height: > 0.0 } air && vehicle.Orbit.StateVectors.PositionCci.Length() - parent.MeanRadius < air.Height)
            ViewDemand.Mark(vehicle, ViewDemand.Kind.Aero);
        KsaAeroSweep.Result aero = s.OrbitalEntry?.Aero ?? s.Aero;
        // ORBITAL ENTRY'S AND BELLYFLOP'S SETTINGS, as FlyOwnAttitude gives them while orbital entry flies: its gains, capture at ROLL RATE or 10 deg/s (pointing and roll alike), authority-aware, phase-plane RCS on ENTRY RCS DEAD BAND, ENTRY RCS - and the stock RCS switch, which the player flies with. Gimbals only while an engine burns.
        bool rcs = fc.RCSMode == FlightComputerRCSMode.Enabled && s.OrbitalEntryUseRcs;
        double capture = Math.Max(10.0, s.OrbitalEntryRollRateDegS);
        var settings = new AttitudeController.Settings(s.AttitudeBandwidthRadS, AttitudeController.Settings.Default.Damping,
                                                       s.AttitudeDeadbandDeg, AttitudeController.Settings.Default.RateDeadbandDegS)
        {
            UseSurfaces = true,
            UseGimbals = KsaEnginePerf.LiveEngineThrust(vehicle) > 0.0,
            CaptureRateDegS = capture,
            RollCaptureRateDegS = capture,
            AuthorityCapture = true,
            UseRcs = rcs,
            PhasePlaneRcs = true,
            RcsAngleBandDeg = s.OrbitalEntryRcsDeadbandDeg,
        };
        AttitudeController.Report report = AttitudeController.Step(vehicle, slot.Target, rateCci, aero, s.MomentLimiterPerPa, settings);

        if (now - slot.LogAt >= LogEveryS)
        {
            slot.LogAt = now;
            slot.Log ??= HandFlyingLog.Start(vehicle, share);
            double3 cmdModel = slot.FiltRateCci.Transform(doubleQuat.Inverse(body2Cci));
            slot.Log?.Row(now, ctrl, share, spare, new double3(double3.Dot(cmdModel, mx), double3.Dot(cmdModel, my), double3.Dot(cmdModel, mz)), flown, LeadDeg(slot.Target, body2Cci), report);
        }
    }

    /// <summary>Vehicle.PrepareWorker's postfix: the rotation keys hidden for the step are put back, so a key held stays held. Main thread.</summary>
    internal static void AfterPrepare(Vehicle vehicle)
    {
        if (Inputs == null || vehicle == null || !Slots.TryGetValue(vehicle, out Slot slot) || slot.Hidden == ThrusterMapFlags.None)
            return;
        Inputs(vehicle).ThrusterCommandFlags |= slot.Hidden;
        slot.Hidden = ThrusterMapFlags.None;
    }

    /// <summary>Hand flying lets go: the controller's actuators are handed back unless a guidance mode has taken the craft, which flies them now.</summary>
    private static void StandDown(Vehicle vehicle, bool release)
    {
        if (Slots.TryGetValue(vehicle, out Slot slot))
        {
            slot.Active = false;
            slot.RateModel = default;
            slot.LastT = double.NaN;
            slot.Log?.Close();
            slot.Log = null;
            slot.LogAt = double.NegativeInfinity;
        }
        if (release)
            AttitudeController.Release(vehicle);
    }

    /// <summary>An attitude (body to CCI) turned for <paramref name="seconds"/> at <paramref name="rateCci"/>.</summary>
    private static doubleQuat Turned(doubleQuat q, double3 rateCci, double seconds)
    {
        double angle = rateCci.Length() * seconds;
        return angle > 1e-12 ? doubleQuat.Concatenate(q, QuaternionEx.CreateFromAxisAngle(rateCci / rateCci.Length(), angle)) : q;
    }

    /// <summary>The rotation from <paramref name="from"/> to <paramref name="to"/> as a vector in CCI: its axis times its angle, rad, the short way.</summary>
    private static double3 RotationVector(doubleQuat from, doubleQuat to)
    {
        doubleQuat r = doubleQuat.Concatenate(doubleQuat.Inverse(from), to);
        double w = Math.Clamp(r.W, -1.0, 1.0);
        double3 v = new(r.X, r.Y, r.Z);
        if (w < 0.0) { w = -w; v = -v; }
        double sinHalf = v.Length();
        if (sinHalf < 1e-12)
            return default;
        return v / sinHalf * (2.0 * Math.Acos(w));
    }

    /// <summary>The attitude <paramref name="maxDeg"/> from <paramref name="from"/> towards <paramref name="to"/> (the rotation between them, about its own axis).</summary>
    private static doubleQuat Toward(doubleQuat from, doubleQuat to, double maxDeg)
    {
        // The rotation in CCI that takes from to to: to = from, then r.
        doubleQuat r = doubleQuat.Concatenate(doubleQuat.Inverse(from), to);
        double w = Math.Clamp(r.W, -1.0, 1.0);
        double3 v = new(r.X, r.Y, r.Z);
        if (w < 0.0) { w = -w; v = -v; }
        double angle = 2.0 * Math.Acos(w);
        double sinHalf = v.Length();
        if (angle < 1e-9 || sinHalf < 1e-12)
            return to;
        double keep = Math.Min(maxDeg * Math.PI / 180.0, angle);
        return doubleQuat.Concatenate(from, QuaternionEx.CreateFromAxisAngle(v / sinHalf, keep));
    }

    private static double LeadDeg(doubleQuat a, doubleQuat b)
    {
        double dot = Math.Abs(a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W);
        return 2.0 * Math.Acos(Math.Min(dot, 1.0)) * 180.0 / Math.PI;
    }

    // ---- the gauge buttons ----

    private static bool _enumInjected;

    /// <summary>Before the game reads the gauges (Mod.OnBeforeMain): the PILOT buttons' enum goes into the list the gauge buttons bind their Action by name from.</summary>
    internal static void InjectGaugeEnum()
    {
        if (GameReflection.GaugeButtonFlightComputer_EnumTypes?.GetValue(null) is not System.Collections.Generic.List<EnumTypeOption> list)
        {
            DefaultCategory.Log.Warning("[AFC] GaugeButtonFlightComputer.EnumTypes is not a List<EnumTypeOption>; the PILOT gauge buttons are not registered.");
            return;
        }
        if (!list.Exists(o => o.Type == typeof(AfcPilot)))
            list.Add(new EnumTypeOption(typeof(AfcPilot)));
        _enumInjected = true;
    }

    internal static void ApplyPatches(Harmony harmony)
    {
        if (!_enumInjected)
            InjectGaugeEnum();
        harmony.CreateClassProcessor(typeof(TogglePatch)).Patch();
        harmony.CreateClassProcessor(typeof(ButtonIsDisabledPatch)).Patch();
        harmony.CreateClassProcessor(typeof(ButtonPackDataPatch)).Patch();
    }

    private static bool IsPilotButton(GaugeButtonFlightComputer button, out AfcPilot value)
    {
        value = default;
        if (GameReflection.GaugeButtonFlightComputer_enumValueRef?.Invoke(button) is AfcPilot p)
        {
            value = p;
            return true;
        }
        return false;
    }

    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.ToggleEnum), new[] { typeof(Enum) })]
    internal static class TogglePatch
    {
        static bool Prefix(Vehicle __instance, Enum enumValue)
        {
            if (enumValue is not AfcPilot button)
                return true;
            Press(__instance, button);
            return false;
        }
    }

    [HarmonyPatch(typeof(GaugeButtonFlightComputer), nameof(GaugeButtonFlightComputer.IsDisabled), new Type[0])]
    internal static class ButtonIsDisabledPatch
    {
        static void Postfix(GaugeButtonFlightComputer __instance, ref bool __result)
        {
            if (IsPilotButton(__instance, out _))
                __result = Program.ControlledVehicle is null or KittenEva || !Available;
        }
    }

    [HarmonyPatch(typeof(GaugeButtonFlightComputer), nameof(GaugeButtonFlightComputer.PackData), new Type[0])]
    internal static class ButtonPackDataPatch
    {
        // Bit 0 clicked, bit 1 lit, bit 2 disabled, bit 3 enabled, as stock packs them.
        static void Postfix(GaugeButtonFlightComputer __instance, ref uint2 __result)
        {
            if (!IsPilotButton(__instance, out AfcPilot button) || Program.ControlledVehicle is not { } vehicle || vehicle is KittenEva)
                return;
            uint bits = (__result.X & 0b0001u) | (Available ? 0b1000u : 0b0100u);
            if (IsLit(vehicle, button))
                bits |= 0b0010u;
            __result = new uint2(bits, __result.Y);
        }
    }
}
