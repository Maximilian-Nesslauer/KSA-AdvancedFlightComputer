#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using KSA;

public enum GimbalOverrideMode
{
    Off = 0,

    /// <summary>Write the same normalized deflection to every gimbal. Plumbing check only.</summary>
    Direct = 1,

    /// <summary>Physical body torque in N*m through a least-squares allocation. The interface guidance should use.</summary>
    Lsq = 3,
}

// Thrust-vector-control override.
//  WHY THIS EXISTS. The 6-DOF SCvx solver plans in (tdx, tdy, T, tau_roll). Until now the mod could only write CustomAttitudeTarget and let KSA's flight computer decide how to get there, throwing away most of the plan.
//  HOW IT ATTACHES. FlightComputer.ComputeControl calls ComputeTvcControl (which allocates, or ZeroizeTvcs which clears) and then ComputeRcsControl. AFC's VehicleCommandSink runs this writer after ComputeControl, so it lands after every stock writer of CommandY/Z and the override is the last word regardless of attitude mode. Actuator writes are reported through the sink's receipt, which is what keeps a commanded craft in full physics.
//  UNITS. CommandY/CommandZ are NORMALIZED, not angles: Gimbal.GetCommand clamps each to [-1,1] and multiplies by that axis's MaxAngle (radians).
//  THREADING. The demand is published from the Vehicle.PrepareWorker prefix on the main thread, before the step's worker job is queued, and read on a VehicleSolvers job thread inside that job, so a publish is ordered before the read that consumes it. Only Disengage is also called from the draw. One immutable Command record per publish keeps a read from pairing a new mode with an old torque.
public static class KsaGimbalControl
{
    /// <summary>
    /// One vehicle's gimbal demand, published whole.
    ///  A record because the apply side runs on a VehicleSolvers JOB THREAD while the demand is written on the sim thread: reading six loose fields there could pair a new mode with an old torque. One reference assignment cannot.
    /// </summary>
    public sealed record Command(
        GimbalOverrideMode Mode,
        float CommandY, float CommandZ,                         // Direct
        double TorqueXNm, double TorqueYNm, double TorqueZNm)   // Lsq (N.m)
    {
        public static readonly Command Off =
            new(GimbalOverrideMode.Off, 0, 0, 0, 0, 0);
    }

    /// <summary>
    /// Everything the override holds for ONE vehicle: its demand, its scratch buffers and its diagnostics.
    ///  THE SCRATCH HAS TO BE PER-VEHICLE, not merely the demand. ApplyLsq runs on a job thread, and with two guided vehicles two of those run at once - sharing one commands array between them would interleave two allocations into the same buffer and then fly the result.
    /// </summary>
    public sealed class Slot
    {
        public volatile Command Cmd = Command.Off;

        // Scratch, reused so the allocation stays allocation-free on the hot path.
        // Touched only by the job thread servicing THIS vehicle.
        internal double[] Commands = [];
        internal double[] Thrusts = [];
        internal GimbalController[] Gimbals = [];

        /// <summary>Gimbals actually written on the last worker pass - 0 means the override is not reaching anything.</summary>
        public int AppliedCount;

        /// <summary>Diagnostics from the last Lsq allocation, for the UI readout.</summary>
        public TvcAllocationResult LastAllocation;
    }

    /// <summary>
    /// Per-vehicle state, keyed on the identity the apply side can actually see.
    ///  The FlightComputer handed to the postfix is NOT the live one - the worker runs on a copy - so reference-comparing the FlightComputer would never match.
    /// VehicleConfigInfo is the usable identity because FlightComputer.CopyFrom assigns it by REFERENCE rather than cloning.
    ///  This replaces a single _target field, and the gain is not only that two vehicles can be driven at once. With one target, a second vehicle engaging silently stole the first one's gimbals, and whether the first still flew depended on the order KSA happened to interleave prepare and compute across vehicles. A lookup cannot be pointed at the wrong vehicle.
    /// </summary>
    private static readonly ConditionalWeakTable<FlightComputer.VehicleConfigInfo, Slot> Slots = new();

    private static Slot SlotFor(Vehicle vehicle)
    {
        FlightComputer.VehicleConfigInfo cfg = vehicle?.FlightComputer?.VehicleConfig;
        return cfg == null ? null : Slots.GetOrCreateValue(cfg);
    }

    /// <summary>This vehicle's diagnostics, or null if it has never been driven.</summary>
    public static Slot Diagnostics(Vehicle vehicle)
    {
        FlightComputer.VehicleConfigInfo cfg = vehicle?.FlightComputer?.VehicleConfig;
        return cfg != null && Slots.TryGetValue(cfg, out Slot s) ? s : null;
    }

    /// <summary>Command a body torque in N.m, allocated across this vehicle's gimbals.</summary>
    public static void SetLsq(Vehicle vehicle, double3 torqueNm)
    {
        Slot s = SlotFor(vehicle);
        if (s != null)
            s.Cmd = new Command(GimbalOverrideMode.Lsq, 0, 0,
                                torqueNm.X, torqueNm.Y, torqueNm.Z);
    }

    /// <summary>Command raw deflections on every gimbal. A plumbing check, used by the harness.</summary>
    public static void SetDirect(Vehicle vehicle, float y, float z)
    {
        Slot s = SlotFor(vehicle);
        if (s != null)
            s.Cmd = new Command(GimbalOverrideMode.Direct, y, z, 0, 0, 0);
    }

    /// <summary>Hand this vehicle's gimbals back to the game.</summary>
    public static void Disengage(Vehicle vehicle)
    {
        Slot s = Diagnostics(vehicle);
        if (s == null) return;
        s.Cmd = Command.Off;
        s.AppliedCount = 0;
    }

    // Runs on the vehicle worker from VehicleCommandSink.Run. Switching guidance off stops the override on the next control step, one step before the per-vehicle release disengages it.
    internal static void OnComputeControl(FlightComputer flightComputer, ref FlightComputerOutput outputs,
                                          ref VehicleCommandSink.Receipt receipt)
    {
        if (!GuidanceWindow.ModActive)
            return;

        FlightComputer.VehicleConfigInfo cfg = flightComputer.VehicleConfig;
        if (cfg == null || !Slots.TryGetValue(cfg, out Slot st))
            return;

        // ONE read of the demand, into a local. Re-reading the field would let a sim thread publish land between two reads and mix two demands together.
        Command cmd = st.Cmd;
        GimbalOverrideMode mode = cmd.Mode;
        if (mode == GimbalOverrideMode.Off)
            return;

        if (mode == GimbalOverrideMode.Lsq)
        {
            ApplyLsq(st, cmd, cfg, flightComputer.CenterOfMassAsmb, ref outputs, ref receipt);
            return;
        }

        // Direct: the same deflection on every gimbal.
        int applied = 0;

        foreach (GimbalController gimbal in cfg.Gimbals)
        {
            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(gimbal);

            // Empty slots carry a null Module and a null State ref - writing through that would be an access violation, not an exception.
            if (slot.Module == null)
                continue;

            // Deliberately NOT gated on TotalThrust > 0, unlike the game's own ComputeTvcControl. An unlit engine still swings its nozzle visually, which is what makes this testable on the pad before committing to a burn.
            slot.State.CommandY = cmd.CommandY;
            slot.State.CommandZ = cmd.CommandZ;
            applied++;
        }

        if (applied > 0)
            receipt.Command();

        st.AppliedCount = applied;
    }

    // Physical allocation: solve for the deflections delivering the commanded N*m.
    //  Two passes over the gimbals because the solve needs every gimbal's thrust before it can produce any command. Thrust falls back to the nameplate maximum when the engine is unlit, so the allocation can be inspected on the pad - the resulting commands are then what WOULD be flown at full thrust.
    private static void ApplyLsq(Slot st, Command cmd, FlightComputer.VehicleConfigInfo cfg,
                                 float3 com, ref FlightComputerOutput outputs,
                                 ref VehicleCommandSink.Receipt receipt)
    {
        int n = cfg.Gimbals.Count;
        if (n == 0)
        {
            st.AppliedCount = 0;
            return;
        }

        if (st.Gimbals.Length < n)
        {
            st.Gimbals = new GimbalController[n];
            st.Thrusts = new double[n];
            st.Commands = new double[2 * n];
        }

        for (int i = 0; i < n; i++)
        {
            GimbalController gc = cfg.Gimbals[i];
            st.Gimbals[i] = gc;

            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(gc);
            double thrust = slot.Module != null ? slot.State.TotalThrust : 0.0;
            st.Thrusts[i] = thrust > 0.0 ? thrust : gc.Data.MaximumThrust;
        }

        st.LastAllocation = KsaTvcAllocator.Solve(
            st.Gimbals.AsSpan(0, n), st.Thrusts.AsSpan(0, n), com,
            new double3(cmd.TorqueXNm, cmd.TorqueYNm, cmd.TorqueZNm), st.Commands);

        int applied = 0;
        for (int i = 0; i < n; i++)
        {
            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(cfg.Gimbals[i]);
            if (slot.Module == null)
                continue;

            slot.State.CommandY = (float)st.Commands[2 * i];
            slot.State.CommandZ = (float)st.Commands[2 * i + 1];
            applied++;
        }

        if (applied > 0)
            receipt.Command();
        st.AppliedCount = applied;
    }

}
