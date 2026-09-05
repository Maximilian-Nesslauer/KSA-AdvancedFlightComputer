using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

// THE TARGET'S OWN TURNING RATE, handed to KSA's flight computer.
//
// WHY THIS EXISTS. FlightComputer.AttitudeTarget carries two things — Target2Cci,
// where to point, and RatesCci, how fast that target is itself rotating — and the
// tracking error is computed against BOTH:
//
//     ErrorRates = ctrlRates - AttitudeTarget.RatesCci.Transform(ctrl2Cci^-1)
//
// So RatesCci is a genuine feedforward: tell the FC the target turns at omega and it
// drives the body rate TO omega instead of to zero.
//
// But look at what the Custom branch of UpdateAttitudeTarget puts there:
//
//     AttitudeTarget.Target2Cci = Concatenate(EulerToQuat(CustomAttitudeTarget), frame2Cci);
//     AttitudeTarget.RatesCci   = AttitudeFrame.GetFrameRatesCci(in nav);
//
// We steer by writing CustomAttitudeTarget in the EclBody frame, which is inertial —
// so its frame rate is nil and the FC is told, every single step, that the target is
// STATIONARY. Guidance then hands it a sequence of stationary targets and the
// controller nulls each one in turn: chase, settle, chase. The steering law has a
// turning rate the whole time (UPFG's own document: "during active guidance calls a
// turning rate may be implied. Logic will be included to incorporate this"), and
// throwing it away is what turns tracking into chasing.
//
// HOW IT ATTACHES. A Harmony POSTFIX on FlightComputer.UpdateAttitudeTarget, which
// ComputeControl calls immediately before UpdateAttitudeError. That is the one window
// where the target has been built and nothing has read it yet, so overwriting
// RatesCci there is the last word without touching how Target2Cci is derived — the
// existing CustomAttitudeTarget path still decides where to point.
//
// THREADING AND IDENTITY. ComputeControl runs on a VehicleSolvers job thread against
// a COPY of the FlightComputer, so the instance handed to the postfix is never the
// live one and reference-comparing it would match nothing. VehicleConfigInfo is the
// usable identity — FlightComputer.CopyFrom assigns it by reference rather than
// cloning — which is the same key KsaGimbalControl is built on, for the same reason.
// The published value is a double3 written on the sim thread and read on the job
// thread; it is republished every step, so a torn read costs one step of a slightly
// wrong rate and never a wrong ANGLE.
public static class KsaAttitudeRate
{
    private sealed class Slot
    {
        public double3 RateCci;
        public bool Engaged;
    }

    private static readonly ConditionalWeakTable<FlightComputer.VehicleConfigInfo, Slot> Slots = new();

    /// <summary>
    /// Publish the rate the commanded attitude is turning at, rad/s in CCI. Called
    /// from the same place the attitude itself is commanded, so the pair can never
    /// describe different instants.
    /// </summary>
    public static void Set(Vehicle vehicle, double3 rateCci)
    {
        Slot s = SlotFor(vehicle);
        if (s == null)
            return;
        s.RateCci = rateCci;
        s.Engaged = true;
    }

    /// <summary>
    /// Stop supplying a rate. MUST be called when guidance releases the vehicle: a
    /// stale feedforward is a standing instruction to keep rotating, and the FC would
    /// go on driving the body rate to it against a target that is no longer moving.
    /// </summary>
    public static void Clear(Vehicle vehicle)
    {
        FlightComputer.VehicleConfigInfo cfg = vehicle?.FlightComputer?.VehicleConfig;
        if (cfg != null && Slots.TryGetValue(cfg, out Slot s))
        {
            s.RateCci = default;
            s.Engaged = false;
        }
    }

    private static Slot SlotFor(Vehicle vehicle)
    {
        FlightComputer.VehicleConfigInfo cfg = vehicle?.FlightComputer?.VehicleConfig;
        return cfg == null ? null : Slots.GetOrCreateValue(cfg);
    }

    // Harmony postfix body. Kept beside the state it drives.
    internal static void OnUpdateAttitudeTarget(FlightComputer flightComputer)
    {
        FlightComputer.VehicleConfigInfo cfg = flightComputer.VehicleConfig;
        if (cfg == null || !Slots.TryGetValue(cfg, out Slot s) || !s.Engaged)
            return;

        // ADDED to whatever the frame contributes, not substituted for it. For the
        // EclBody frame we steer in that term is nil, but the field is the target's
        // total rate in CCI and a frame that does rotate would still have to be
        // carried — overwriting it outright would silently subtract the frame.
        flightComputer.AttitudeTarget.RatesCci += s.RateCci;
    }
}
