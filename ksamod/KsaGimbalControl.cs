using System;
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
    public static GimbalOverrideMode Mode;

    // Direct mode.
    public static float CommandY;
    public static float CommandZ;

    // Torque mode: normalized body-frame demand, X = roll, Y = pitch, Z = yaw,
    // matching the axis convention of the torque vector KSA builds internally
    // from its RollRight/PitchUp/YawRight inputs.
    public static float TorqueRoll;
    public static float TorquePitch;
    public static float TorqueYaw;

    // Lsq mode: desired body torque in N·m.
    public static double TorqueXNm;
    public static double TorqueYNm;
    public static double TorqueZNm;

    /// <summary>Gimbals actually written on the last worker pass — 0 means the override isn't reaching anything.</summary>
    public static int AppliedCount;

    /// <summary>Diagnostics from the last Lsq allocation, for the UI readout.</summary>
    public static TvcAllocationResult LastAllocation;

    /// <summary>
    /// Commands from the last Lsq allocation, 2 per gimbal (Y then Z). Written by the
    /// worker, read by the UI for display only — an occasional torn read just means
    /// one frame of a stale number in a readout.
    /// </summary>
    public static ReadOnlySpan<double> LastCommands => _commands;

    // Scratch reused across worker passes so the allocation stays allocation-free on
    // the hot path. Only ever touched from the worker thread.
    private static double[] _commands = [];
    private static double[] _thrusts = [];
    private static GimbalController[] _gimbals = [];

    // Which vehicle we are allowed to drive.
    //
    // The FlightComputer handed to the postfix is NOT the live one: the worker runs
    // on VehicleUpdateData.NewFlightComputer, a copy, so reference-comparing the
    // FlightComputer itself would never match. VehicleConfigInfo is the usable
    // identity because FlightComputer.CopyFrom assigns it by REFERENCE
    // (`VehicleConfig = existing.VehicleConfig`) rather than cloning. Without this
    // check the override would apply to every vehicle being stepped.
    private static FlightComputer.VehicleConfigInfo _target;

    public static void SetTarget(Vehicle vehicle)
    {
        _target = vehicle?.FlightComputer?.VehicleConfig;
    }

    public static void Disengage()
    {
        Mode = GimbalOverrideMode.Off;
        CommandY = CommandZ = 0f;
        TorqueRoll = TorquePitch = TorqueYaw = 0f;
        AppliedCount = 0;
        _target = null;
    }

    // Harmony postfix body. Kept beside the state it drives.
    internal static void OnComputeControl(FlightComputer flightComputer, ref FlightComputerOutput outputs)
    {
        GimbalOverrideMode mode = Mode;
        if (mode == GimbalOverrideMode.Off)
            return;

        FlightComputer.VehicleConfigInfo cfg = flightComputer.VehicleConfig;
        if (cfg == null || !ReferenceEquals(cfg, _target))
            return;

        float3 com = flightComputer.CenterOfMassAsmb;

        if (mode == GimbalOverrideMode.Lsq)
        {
            ApplyLsq(cfg, com, ref outputs);
            return;
        }

        float directY = CommandY, directZ = CommandZ;
        var demand = new double3(TorqueRoll, TorquePitch, TorqueYaw);
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

        AppliedCount = applied;
    }

    // Physical allocation: solve for the deflections delivering the commanded N·m.
    //
    // Two passes over the gimbals because the solve needs every gimbal's thrust
    // before it can produce any command. Thrust falls back to the nameplate maximum
    // when the engine is unlit, so the allocation can be inspected on the pad — the
    // resulting commands are then what WOULD be flown at full thrust.
    private static void ApplyLsq(FlightComputer.VehicleConfigInfo cfg, float3 com,
                                 ref FlightComputerOutput outputs)
    {
        int n = cfg.Gimbals.Count;
        if (n == 0)
        {
            AppliedCount = 0;
            return;
        }

        if (_gimbals.Length < n)
        {
            _gimbals = new GimbalController[n];
            _thrusts = new double[n];
            _commands = new double[2 * n];
        }

        for (int i = 0; i < n; i++)
        {
            GimbalController gc = cfg.Gimbals[i];
            _gimbals[i] = gc;

            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(gc);
            double thrust = slot.Module != null ? slot.State.TotalThrust : 0.0;
            _thrusts[i] = thrust > 0.0 ? thrust : gc.Data.MaximumThrust;
        }

        LastAllocation = KsaTvcAllocator.Solve(
            _gimbals.AsSpan(0, n), _thrusts.AsSpan(0, n), com,
            new double3(TorqueXNm, TorqueYNm, TorqueZNm), _commands);

        int applied = 0;
        for (int i = 0; i < n; i++)
        {
            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(cfg.Gimbals[i]);
            if (slot.Module == null)
                continue;

            slot.State.CommandY = (float)_commands[2 * i];
            slot.State.CommandZ = (float)_commands[2 * i + 1];
            applied++;
        }

        if (applied > 0)
            outputs.AnyActuatorCommanded = true;
        AppliedCount = applied;
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
