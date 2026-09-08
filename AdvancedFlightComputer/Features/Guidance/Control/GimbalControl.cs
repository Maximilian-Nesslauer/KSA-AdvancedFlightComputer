using System;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

public enum GimbalOverrideMode
{
    Off = 0,

    /// <summary>Write the same normalized deflection to every gimbal. Plumbing check only.</summary>
    Direct = 1,

    /// <summary>Normalized body-frame demand through a replica of KSA's own per-gimbal heuristic. For comparison.</summary>
    Torque = 2,

    /// <summary>Physical body torque in N·m through a least-squares allocation. The interface guidance should use.</summary>
    Lsq = 3,
}

// Thrust-vector-control override.
//
// WHY THIS EXISTS. The 6-DOF SCvx solver plans in (tdx, tdy, T, tau_roll). Until
// now the mod could only write CustomAttitudeTarget and let KSA's flight computer
// decide how to get there, throwing away most of the plan.
//
// HOW IT ATTACHES. FlightComputer.ComputeControl calls ComputeTvcControl (which
// allocates, or ZeroizeTvcs which clears) and then ComputeRcsControl. A Harmony
// POSTFIX on ComputeControl therefore lands after every writer of CommandY/Z, so
// the override is the last word regardless of attitude mode.
//
// UNITS. CommandY/CommandZ are NORMALIZED, not angles: Gimbal.GetCommand clamps
// each to [-1,1] and multiplies by that axis's MaxAngle (radians).
//
// THREADING. ComputeControl runs on a VehicleSolvers job thread, not the main
// thread, so these fields are written by the UI draw and read by the worker. They
// are bool/float/enum — 32-bit aligned, reads cannot tear — and a command landing
// one frame late is harmless for a manual test. Do NOT extend this with anything
// needing a consistent multi-field snapshot without real synchronisation.
public static class KsaGimbalControl
{
    /// <summary>
    /// One vehicle's gimbal demand, published whole.
    ///
    /// A record because the apply side runs on a VehicleSolvers JOB THREAD while the
    /// demand is written on the sim thread: reading nine loose fields there could pair
    /// a new mode with an old torque. One reference assignment cannot.
    /// </summary>
    public sealed record Command(
        GimbalOverrideMode Mode,
        float CommandY, float CommandZ,                         // Direct
        float TorqueRoll, float TorquePitch, float TorqueYaw,   // Torque (normalised)
        double TorqueXNm, double TorqueYNm, double TorqueZNm)   // Lsq (N.m)
    {
        public static readonly Command Off =
            new(GimbalOverrideMode.Off, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    /// <summary>
    /// Everything the override holds for ONE vehicle: its demand, its scratch buffers
    /// and its diagnostics.
    ///
    /// THE SCRATCH HAS TO BE PER-VEHICLE, not merely the demand. ApplyLsq runs on a
    /// job thread, and with two guided vehicles two of those run at once - sharing one
    /// commands array between them would interleave two allocations into the same
    /// buffer and then fly the result.
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

        /// <summary>
        /// Commands from the last Lsq allocation, 2 per gimbal (Y then Z). Written by
        /// the worker, read by the UI for display only - an occasional torn read just
        /// means one frame of a stale number in a readout.
        /// </summary>
        public ReadOnlySpan<double> LastCommands => Commands;
    }

    /// <summary>
    /// Per-vehicle state, keyed on the identity the apply side can actually see.
    ///
    /// The FlightComputer handed to the postfix is NOT the live one - the worker runs
    /// on a copy - so reference-comparing the FlightComputer would never match.
    /// VehicleConfigInfo is the usable identity because FlightComputer.CopyFrom assigns
    /// it by REFERENCE rather than cloning.
    ///
    /// This replaces a single _target field, and the gain is not only that two vehicles
    /// can be driven at once. With one target, a second vehicle engaging silently stole
    /// the first one's gimbals, and whether the first still flew depended on the order
    /// KSA happened to interleave prepare and compute across vehicles. A lookup cannot
    /// be pointed at the wrong vehicle.
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
            s.Cmd = new Command(GimbalOverrideMode.Lsq, 0, 0, 0, 0, 0,
                                torqueNm.X, torqueNm.Y, torqueNm.Z);
    }

    /// <summary>Command raw deflections on every gimbal. Manual probe only.</summary>
    public static void SetDirect(Vehicle vehicle, float y, float z)
    {
        Slot s = SlotFor(vehicle);
        if (s != null)
            s.Cmd = new Command(GimbalOverrideMode.Direct, y, z, 0, 0, 0, 0, 0, 0);
    }

    /// <summary>Command a normalised body-frame torque, KSA's own convention.</summary>
    public static void SetTorque(Vehicle vehicle, float roll, float pitch, float yaw)
    {
        Slot s = SlotFor(vehicle);
        if (s != null)
            s.Cmd = new Command(GimbalOverrideMode.Torque, 0, 0, roll, pitch, yaw, 0, 0, 0);
    }

    /// <summary>Hand this vehicle's gimbals back to the game.</summary>
    public static void Disengage(Vehicle vehicle)
    {
        Slot s = Diagnostics(vehicle);
        if (s == null) return;
        s.Cmd = Command.Off;
        s.AppliedCount = 0;
    }

    // Harmony postfix body. Kept beside the state it drives.
    internal static void OnComputeControl(FlightComputer flightComputer, ref FlightComputerOutput outputs)
    {
        FlightComputer.VehicleConfigInfo cfg = flightComputer.VehicleConfig;
        if (cfg == null || !Slots.TryGetValue(cfg, out Slot st))
            return;

        // ONE read of the demand, into a local. Re-reading the field would let a sim
        // thread publish land between two reads and mix two demands together.
        Command cmd = st.Cmd;
        GimbalOverrideMode mode = cmd.Mode;
        if (mode == GimbalOverrideMode.Off)
            return;

        float3 com = flightComputer.CenterOfMassAsmb;

        if (mode == GimbalOverrideMode.Lsq)
        {
            ApplyLsq(st, cmd, cfg, com, ref outputs);
            return;
        }

        float directY = cmd.CommandY, directZ = cmd.CommandZ;
        var demand = new double3(cmd.TorqueRoll, cmd.TorquePitch, cmd.TorqueYaw);
        int applied = 0;

        foreach (GimbalController gimbal in cfg.Gimbals)
        {
            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(gimbal);

            // Empty slots carry a null Module and a null State ref — writing through
            // that would be an access violation, not an exception.
            if (slot.Module == null)
                continue;

            float y, z;
            if (mode == GimbalOverrideMode.Direct)
            {
                y = directY;
                z = directZ;
            }
            else
            {
                Allocate(gimbal, com, demand, out y, out z);
            }

            // Deliberately NOT gated on TotalThrust > 0, unlike the game's own
            // ComputeTvcControl. An unlit engine still swings its nozzle visually,
            // which is what makes this testable on the pad before committing to a burn.
            slot.State.CommandY = y;
            slot.State.CommandZ = z;
            applied++;
        }

        if (applied > 0)
            outputs.AnyActuatorCommanded = true;

        st.AppliedCount = applied;
    }

    // Physical allocation: solve for the deflections delivering the commanded N·m.
    //
    // Two passes over the gimbals because the solve needs every gimbal's thrust
    // before it can produce any command. Thrust falls back to the nameplate maximum
    // when the engine is unlit, so the allocation can be inspected on the pad — the
    // resulting commands are then what WOULD be flown at full thrust.
    private static void ApplyLsq(Slot st, Command cmd, FlightComputer.VehicleConfigInfo cfg,
                                 float3 com, ref FlightComputerOutput outputs)
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
            outputs.AnyActuatorCommanded = true;
        st.AppliedCount = applied;
    }

    /// <summary>
    /// Distribute one body-frame torque demand onto a single gimbal — a faithful
    /// replica of the per-gimbal block inside FlightComputer.ComputeTvcControl.
    ///
    /// This is the layer worth commanding. KSA does NOT solve a control-allocation
    /// matrix; each gimbal independently works out, from pure geometry, which way
    /// to push to make torque in the demanded direction about ITS OWN moment arm:
    ///
    ///   arm     = thrust-weighted nozzle position - centre of mass
    ///   dir     = cross(normalize(cross(demand, armHat)), armHat)
    ///
    /// i.e. the lateral thrust direction perpendicular to the arm whose moment lies
    /// along the demand. That direction is rotated into the gimbal's own frame and
    /// its Y/Z components become the deflection command.
    ///
    /// The consequence — and the reason this is engine-config-agnostic — is that
    /// each gimbal is silently excluded from any axis it has no leverage over: an
    /// arm with no component in a plane cannot torque about the perpendicular axis,
    /// so that component of the demand is ZEROED FOR THIS GIMBAL ONLY. A centreline
    /// main engine therefore contributes pitch and yaw but no roll, while an
    /// off-axis vernier picks the roll up. Commanding torque rather than deflection
    /// means we inherit that split for free on any vehicle layout.
    ///
    /// Public so the UI can display exactly what the worker will command.
    /// </summary>
    public static void Allocate(GimbalController gimbal, float3 comAsmb, double3 demand,
                                out float commandY, out float commandZ)
    {
        commandY = 0f;
        commandZ = 0f;

        float3 armF = gimbal.Data.ThrustPosVehicleAsmb - comAsmb;
        var arm = new double3(armF.X, armF.Y, armF.Z);
        double3 armHat = double3.NormalizeOrZero(arm);

        // Per-axis leverage test, matching KSA's tolerance. Written out on components
        // rather than via the double2 swizzles so the intent is legible: an arm lying
        // along one axis has no moment about it.
        double3 d = demand;
        if (Math.Sqrt(armHat.X * armHat.X + armHat.Y * armHat.Y) < 1e-3) d.Z = 0.0;
        if (Math.Sqrt(armHat.X * armHat.X + armHat.Z * armHat.Z) < 1e-3) d.Y = 0.0;
        if (Math.Sqrt(armHat.Y * armHat.Y + armHat.Z * armHat.Z) < 1e-3) d.X = 0.0;

        double3 axis = double3.NormalizeOrZero(double3.Cross(d, armHat));
        double3 dir = double3.Cross(axis, armHat);
        if (dir.Length() < 1e-3)
            return;

        dir = double3.NormalizeOrZero(dir);
        double3 inGimbal = dir.Transform(doubleQuat.Unpack(in gimbal.Data.VehicleAsmb2Gimbal));
        double3 scaled = inGimbal * d.Length();

        commandY = (float)scaled.Y;
        commandZ = (float)scaled.Z;
    }

    /// <summary>Which body axes this gimbal has any leverage over, for the UI readout.</summary>
    public static void Leverage(GimbalController gimbal, float3 comAsmb,
                                out bool roll, out bool pitch, out bool yaw)
    {
        float3 armF = gimbal.Data.ThrustPosVehicleAsmb - comAsmb;
        double3 armHat = double3.NormalizeOrZero(new double3(armF.X, armF.Y, armF.Z));
        yaw = Math.Sqrt(armHat.X * armHat.X + armHat.Y * armHat.Y) >= 1e-3;
        pitch = Math.Sqrt(armHat.X * armHat.X + armHat.Z * armHat.Z) >= 1e-3;
        roll = Math.Sqrt(armHat.Y * armHat.Y + armHat.Z * armHat.Z) >= 1e-3;
    }
}
