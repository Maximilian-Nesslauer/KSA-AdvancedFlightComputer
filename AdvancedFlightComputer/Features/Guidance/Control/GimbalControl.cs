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
        double TorqueXNm, double TorqueYNm, double TorqueZNm,   // Lsq (N.m)
        double CoupledXNm = 0, double CoupledYNm = 0, double CoupledZNm = 0)   // Lsq: the part of the torque that comes with the strong axes' (KsaTvcAllocator.Solve)
    {
        public static readonly Command Off =
            new(GimbalOverrideMode.Off, 0, 0, 0, 0, 0);
    }

    /// <summary>
    /// Everything the override holds for ONE vehicle: its demand, its scratch buffers and its diagnostics.
    ///  THE SCRATCH HAS TO BE PER-VEHICLE, not merely the demand. ApplyLsq runs on a job thread, and with two guided vehicles two of those run at once - sharing one commands array between them would interleave two allocations into the same buffer and then fly the result.
    /// </summary>
    /// <summary>
    /// One gimbal's command and state on a control pass, in FRACTIONS OF ITS RANGE (Gimbal.GetCommand's units): the command just written, the angle and rate the game's state had before this pass (per second for the rate), its range in degrees and its slew cap in ranges per second, the thrust through it, and how many gimbals the allocation drove. An angle that trails its command by the slew cap is the gimbal's rate limit; one on its command with the torque still short is the allocation.
    /// </summary>
    public sealed record GimbalTrace(double CmdY, double CmdZ, double AngleY, double AngleZ, double RateY, double RateZ,
                                     double MaxAngleDeg, double SlewPerS, double ThrustN, int Count);

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

        /// <summary>The highest-thrust gimbal on the last Lsq pass: what it was told against where it is. Replaced whole by the worker, read by the sim thread for the log.</summary>
        public volatile GimbalTrace Trace;

        /// <summary>The vehicle the demand is for. The worker reads Vehicle.IsControllable through it, which Vehicle.UpdateControl writes on the main thread while no worker runs.</summary>
        internal volatile Vehicle Owner;

        /// <summary>Where the gimbals of the engines a landing burn leaves off are parked (KsaIdleEngineParking), or null outside one. Published whole by the sim thread like the demand, and independent of it: entry guidance's burn steers through stock TVC with the override off.</summary>
        internal volatile KsaIdleEngineParking.Plan Parking;
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
        if (cfg == null)
            return null;
        Slot slot = Slots.GetOrCreateValue(cfg);
        slot.Owner = vehicle;
        return slot;
    }

    /// <summary>This vehicle's diagnostics, or null if it has never been driven.</summary>
    public static Slot Diagnostics(Vehicle vehicle)
    {
        FlightComputer.VehicleConfigInfo cfg = vehicle?.FlightComputer?.VehicleConfig;
        return cfg != null && Slots.TryGetValue(cfg, out Slot s) ? s : null;
    }

    /// <summary>Command a body torque in N.m, allocated across this vehicle's gimbals. <paramref name="coupledNm"/> is the part of it that comes with the pitch and yaw torque on its own (KsaTvcAllocator.Solve): the roll an off-centre engine's deflection brings.</summary>
    public static void SetLsq(Vehicle vehicle, double3 torqueNm, double3 coupledNm = default)
    {
        Slot s = SlotFor(vehicle);
        if (s != null)
            s.Cmd = new Command(GimbalOverrideMode.Lsq, 0, 0,
                                torqueNm.X, torqueNm.Y, torqueNm.Z, coupledNm.X, coupledNm.Y, coupledNm.Z);
    }

    /// <summary>Command raw deflections on every gimbal. A plumbing check, used by the harness.</summary>
    public static void SetDirect(Vehicle vehicle, float y, float z)
    {
        Slot s = SlotFor(vehicle);
        if (s != null)
            s.Cmd = new Command(GimbalOverrideMode.Direct, y, z, 0, 0, 0);
    }

    /// <summary>Park the gimbals of this vehicle's idle engines as this plan says, or with null slew any it parked back to rest. Only a gimbal whose engine is not activated is ever written, so the live engines and their allocation are untouched.</summary>
    internal static void SetIdleParking(Vehicle vehicle, KsaIdleEngineParking.Plan plan)
    {
        Slot s = plan != null ? SlotFor(vehicle) : Diagnostics(vehicle);
        if (s != null)
            s.Parking = plan;
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
        Slot st = null;
        if (cfg != null)
            Slots.TryGetValue(cfg, out st);

        // A craft out of control flies only what stock commands, as with the player's inputs, which ControlsLockout blocks.
        if (st?.Owner != null && (!st.Owner.IsControllable || VehicleControlOwnership.KfsOwns(st.Owner)))
            return;

        if (st != null)
            ApplyCommand(st, flightComputer, cfg, ref outputs, ref receipt);

        // LAST, after stock TVC and the allocation, so nothing writes the idle gimbals after it. It runs with no slot too: a flight computer reset (an engine shut off, a split) makes a new VehicleConfigInfo, and a gimbal parked before it still has to come back to rest.
        KsaIdleEngineParking.Apply(st?.Parking, ref outputs, ref receipt);
    }

    private static void ApplyCommand(Slot st, FlightComputer flightComputer, FlightComputer.VehicleConfigInfo cfg,
                                     ref FlightComputerOutput outputs, ref VehicleCommandSink.Receipt receipt)
    {
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

            // Deliberately NOT gated on thrust, unlike the game's own ComputeTvcControl, which gives a zero command to an unlit gimbal while another gimbal is lit. An unlit engine still swings its nozzle visually, which is what makes this testable on the pad before committing to a burn. FlightComputer.VehicleConfig.Gimbals holds only the gimbals whose engine is activated, so that check needs the engine staged.
            slot.State.CommandY = cmd.CommandY;
            slot.State.CommandZ = cmd.CommandZ;
            // Gimbal.UpdateModules follows the command at the lower of Gimbal.NaturalFrequency and this limit, and stock TVC in Auto leaves a finite limit in the state until it writes again, so the override sets the gimbal's own response.
            slot.State.NaturalFrequencyLimit = float.PositiveInfinity;
            applied++;
        }

        if (applied > 0)
            receipt.Command();

        st.AppliedCount = applied;
    }

    /// <summary>
    /// Whether any of these gimbals has an activated engine, by GimbalController.HasActiveEngine. That is the test FlightComputer.ReadUpdatedVehicleConfiguration uses to fill FlightComputer.VehicleConfig.Gimbals, while Vehicle.Parts.Modules also holds the gimbals of upper stages that are not staged yet and can produce no torque.
    /// Planning counts the gimbals of activated engines, or every gimbal when none is activated, so that a check on the pad still has gimbals to measure; see <see cref="CountsForPlanning"/>.
    /// </summary>
    public static bool AnyEngineActive(ReadOnlySpan<GimbalController> gimbals)
    {
        foreach (GimbalController gc in gimbals)
            if (gc.HasActiveEngine())
                return true;
        return false;
    }

    public static bool CountsForPlanning(GimbalController gc, bool anyEngineActive)
        => !anyEngineActive || gc.HasActiveEngine();

    /// <summary>The gimbals planning counts, as an array for the allocator's setup-time measurements.</summary>
    public static GimbalController[] PlanningGimbals(Vehicle vehicle)
    {
        Span<GimbalController> all = vehicle.Parts.Modules.Get<GimbalController>();
        bool anyActive = AnyEngineActive(all);
        int count = 0;
        foreach (GimbalController gc in all)
            if (CountsForPlanning(gc, anyActive))
                count++;

        var result = new GimbalController[count];
        int next = 0;
        foreach (GimbalController gc in all)
            if (CountsForPlanning(gc, anyActive))
                result[next++] = gc;
        return result;
    }

    /// <summary>
    /// The thrust now flowing through the engines that sit on a gimbal with a range, N, from the gimbal states of the last step (GimbalControllerState.TotalThrust). Above zero, the gimbals can steer the craft.
    /// </summary>
    public static double LitGimbalThrust(Vehicle vehicle)
    {
        if (vehicle?.Parts?.States == null
            || !ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var gimbals))
            return 0.0;
        double thrust = 0.0;
        foreach (var gimbal in gimbals.ModulesAndStates)
        {
            Gimbal g = gimbal.Module.Gimbal;
            if (g != null && Math.Max(g.AxisY.MaxAngle, g.AxisZ.MaxAngle) > 0f)
                thrust += gimbal.State.TotalThrust;
        }
        return thrust;
    }

    // Physical allocation: solve for the deflections delivering the commanded N*m.
    //  Two passes over the gimbals because the solve needs every gimbal's thrust before it can produce any command.
    //  FlightComputer.VehicleConfig.Gimbals holds only the gimbals whose engine is activated, so a stage that is not staged yet never enters the solve.
    //  An unlit gimbal enters the solve with zero thrust while another gimbal is lit, as in stock FlightComputer.UpdateTvcThrusts, and stock ComputeTvcControl gives it a zero command. With nameplate thrust in its columns the minimum-norm solve would give it a share of the demand by its moment arm, and that share would produce no torque. Only when no gimbal has thrust does the nameplate maximum stand in, so the allocation can still be inspected on the pad.
    //  A gimbal whose engine is not activated is not in the solve at all. In a landing burn KsaIdleEngineParking parks it, after this writer, and never touches a gimbal the solve drives.
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

        bool anyLit = false;
        for (int i = 0; i < n; i++)
        {
            GimbalController gc = cfg.Gimbals[i];
            st.Gimbals[i] = gc;

            // Read the thrust from the state before this control step, as stock FlightComputer.UpdateTvcThrusts does.
            double thrust = outputs.Gimbals.GetModuleAndReadOnlyState(gc).State.TotalThrust;
            st.Thrusts[i] = thrust;
            anyLit |= thrust > 0.0;
        }

        // The preview keeps to the gimbals PlanningGimbals chooses. FlightComputer.VehicleConfig.Gimbals holds the gimbals with an activated engine as of the last flight computer reset, so this filter only acts on an engine deactivated after that reset. Such a gimbal gets no share here while another engine in the list is still activated.
        if (!anyLit)
        {
            bool anyActive = AnyEngineActive(st.Gimbals.AsSpan(0, n));
            for (int i = 0; i < n; i++)
                st.Thrusts[i] = CountsForPlanning(st.Gimbals[i], anyActive)
                    ? st.Gimbals[i].Data.MaximumThrust
                    : 0.0;
        }

        st.LastAllocation = KsaTvcAllocator.Solve(
            st.Gimbals.AsSpan(0, n), st.Thrusts.AsSpan(0, n), com,
            new double3(cmd.TorqueXNm, cmd.TorqueYNm, cmd.TorqueZNm), st.Commands,
            coupled: new double3(cmd.CoupledXNm, cmd.CoupledYNm, cmd.CoupledZNm));

        // The log's gimbal: the one with the most thrust through it.
        int lead = 0;
        for (int i = 1; i < n; i++)
            if (st.Thrusts[i] > st.Thrusts[lead])
                lead = i;
        {
            GimbalController gc = st.Gimbals[lead];
            Gimbal g = gc.Gimbal;
            GimbalControllerState was = outputs.Gimbals.GetModuleAndReadOnlyState(gc).State;
            double maxY = g != null ? g.AxisY.MaxAngle : 0.0, maxZ = g != null ? g.AxisZ.MaxAngle : 0.0;
            st.Trace = new GimbalTrace(
                st.Commands[2 * lead], st.Commands[2 * lead + 1],
                maxY > 0.0 ? was.Angles.X / maxY : 0.0, maxZ > 0.0 ? was.Angles.Y / maxZ : 0.0,
                maxY > 0.0 ? was.AngleRates.X / maxY : 0.0, maxZ > 0.0 ? was.AngleRates.Y / maxZ : 0.0,
                Math.Max(maxY, maxZ) * 180.0 / Math.PI,
                g != null ? g.NaturalFrequency * Gimbal.RATE_CAP_PER_NATURAL_FREQUENCY : 0.0,
                st.Thrusts[lead], n);
        }

        int applied = 0;
        for (int i = 0; i < n; i++)
        {
            ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>
                .StateUpdater.ModuleAndNewStateRef slot = outputs.Gimbals.GetModuleAndNewState(cfg.Gimbals[i]);
            if (slot.Module == null)
                continue;

            slot.State.CommandY = (float)st.Commands[2 * i];
            slot.State.CommandZ = (float)st.Commands[2 * i + 1];
            slot.State.NaturalFrequencyLimit = float.PositiveInfinity;
            applied++;
        }

        if (applied > 0)
            receipt.Command();
        st.AppliedCount = applied;
    }

}
