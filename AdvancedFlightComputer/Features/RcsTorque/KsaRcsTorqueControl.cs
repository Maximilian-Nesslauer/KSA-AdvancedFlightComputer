using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// Gives a craft a torque from its RCS thrusters, the cheapest way its layout allows, for any mode that wants one. The caller asks for a torque in the vehicle's assembly axes; this solves <see cref="RcsTorqueAllocator"/> for each thruster's duty on the main thread, and the vehicle worker turns the duties into pulses.
///
/// SEPARATE FROM THE STOCK FLIGHT COMPUTER. Nothing here reads its RCS mode, its thruster configuration, its authority figures or its attitude modes: the thrusters come from the craft's parts, their thrust and torque from their own nozzles (RcsWrenchTable.ComputeLive), and the state is keyed on the thruster modules. The one point of contact is where the pulses are delivered. Each thruster's pulse lives in its module state (ThrusterControllerState.CommandPulseTime), which stock rewrites for every thruster on every control step, zero when it fires nothing, so the pulses are written after it: from VehicleCommandSink, after FlightComputer.ComputeControl, taking the longer of stock's pulse and this one.
///
/// WHY NOT STOCK'S RCS. Stock steers one axis at a time: every thruster tagged to an axis fires for the same time, t = impulse / that axis's authority, and only the tagged component of each thruster's torque counts (FlightComputer.SelectJetsToFire, ThrusterController.ComputeIntendedAction). It cannot be asked for a torque, it ignores what a thruster does to the other axes and the side force it brings, and it fires every tagged thruster whether or not a cheaper set would do.
///
/// PULSES. A thruster is on or off, so a duty d over the control period <see cref="PeriodSec"/> is a pulse of d x period. A pulse shorter than the thruster's minimum is carried into the next period instead of fired, so a small duty comes out as an occasional pulse of the right average rather than a minimum pulse every period (or none).
/// </summary>
internal static class KsaRcsTorqueControl
{
    /// <summary>The control period, s: each thruster's duty is a pulse of that fraction of it.</summary>
    internal const double PeriodSec = 0.1;

    /// <summary>What the last command asked for and what its duties give, all in the caller's frame: torque N m, force N, propellant kg/s.</summary>
    internal readonly record struct Report(
        bool Active,
        double3 DemandNm, double3 GivenNm, double3 ForceN,
        double FlowKgS, double ShortfallNm, int Firing, int Thrusters);

    /// <summary>One published set of duties, never edited after it is published, so the worker reads it whole.</summary>
    private sealed class Pulses
    {
        public readonly ThrusterController[] Modules;
        public readonly double[] Duty;
        public readonly Dictionary<ThrusterController, int> Index;

        public Pulses(ThrusterController[] modules, double[] duty)
        {
            Modules = modules;
            Duty = duty;
            Index = new Dictionary<ThrusterController, int>(modules.Length, ReferenceEqualityComparer.Instance);
            for (int i = 0; i < modules.Length; i++)
                Index[modules[i]] = i;
        }
    }

    private sealed class Slot
    {
        internal volatile Pulses? Cmd;
        internal volatile Vehicle? Owner;
        internal Report Last;

        /// <summary>Thrusters the worker matched to the published duties on its last pass: 0 with a command standing means the pulses are not reaching anything.</summary>
        internal volatile int Reached;

        // The worker's own: the pulse each thruster fires in the current period, s after its start, and the time carried into the next.
        internal double[] FireUntil = [];
        internal double[] Carry = [];
        internal UniverseTime PeriodStart;
        internal bool PeriodValid;
    }

    /// <summary>Each craft's slot, and the same slot under each of its thruster modules, which is what the worker can see: the module states it is handed.</summary>
    private static readonly ConditionalWeakTable<Vehicle, Slot> ByVehicle = new();
    private static readonly ConditionalWeakTable<ThrusterController, Slot> ByThruster = new();

    /// <summary>How many crafts have a command standing, so the worker looks for none when there are none.</summary>
    private static int _live;

    private static void Publish(Slot slot, Pulses? cmd)
    {
        Pulses? was = slot.Cmd;
        slot.Cmd = cmd;
        if (was == null && cmd != null)
            Interlocked.Increment(ref _live);
        else if (was != null && cmd == null)
            Interlocked.Decrement(ref _live);
    }

    /// <summary>The craft's staged thrusters, from its parts.</summary>
    private static ThrusterController[] Thrusters(Vehicle vehicle)
    {
        Span<ThrusterController> all = vehicle.Parts.Modules.Get<ThrusterController>();
        int count = 0;
        foreach (ThrusterController t in all)
            if (t.IsActive)
                count++;
        var list = new ThrusterController[count];
        int i = 0;
        foreach (ThrusterController t in all)
            if (t.IsActive)
                list[i++] = t;
        return list;
    }

    /// <summary>
    /// The craft's staged RCS thrusters as an allocator in the frame (<paramref name="ax"/>, <paramref name="ay"/>, <paramref name="az"/>), unit vectors in vehicle assembly axes: each thruster's full-thrust force and torque about the live centre of mass, from its own nozzles' performance (RcsWrenchTable.ComputeLive). Null when the craft has none that can fire.
    /// </summary>
    internal static RcsTorqueAllocator? Measure(Vehicle vehicle, double3 ax, double3 ay, double3 az,
                                                out ThrusterController[] modules)
    {
        modules = [];
        if (vehicle?.Parts == null)
            return null;
        modules = Thrusters(vehicle);
        if (modules.Length == 0)
            return null;

        ReadOnlySpan<RocketState> rocketStates = RocketStates.Of(vehicle.Parts);
        float3 com = vehicle.TotalMassPropsAsmb.Offset;
        float ambient = vehicle.PhysicsEnvironment.AtmosphericPressure;
        var assembly = new RcsCtrlFrame(floatQuat.Identity);

        int n = modules.Length;
        var alloc = new RcsTorqueAllocator(n);
        int usable = 0;
        for (int i = 0; i < n; i++)
        {
            RcsWrenchTable.ComputeLive(modules[i], rocketStates, com, ambient, in assembly,
                out float3 force, out float3 torque, out float flow);
            var f = new double3(force.X, force.Y, force.Z);
            var t = new double3(torque.X, torque.Y, torque.Z);
            alloc.Force[3 * i] = double3.Dot(f, ax);
            alloc.Force[3 * i + 1] = double3.Dot(f, ay);
            alloc.Force[3 * i + 2] = double3.Dot(f, az);
            alloc.Torque[3 * i] = double3.Dot(t, ax);
            alloc.Torque[3 * i + 1] = double3.Dot(t, ay);
            alloc.Torque[3 * i + 2] = double3.Dot(t, az);
            bool live = flow > 0f && !force.IsExactlyZero() && float.IsFinite(flow);
            alloc.Flow[i] = live ? flow : 0.0;
            if (live)
                usable++;
        }
        return usable > 0 ? alloc : null;
    }

    /// <summary>
    /// The ROLL BALANCE's RCS terms (EntryLog's roll monitor), about <paramref name="axisAsmb"/> in assembly axes, N m: the torque the thrusters fire this step - each one's full-thrust torque times the share of <paramref name="stepS"/> its commanded pulse covers, whoever commanded it - and what all of them can give each way, every thruster that turns the craft that way at full thrust, as the stock flight computer counts its authority (ThrusterAuthority.TotalTorque). Main thread.
    /// </summary>
    internal static bool FiredTorqueAlong(Vehicle vehicle, double3 axisAsmb, double stepS,
                                          out double firedNm, out double maxPosNm, out double maxNegNm)
    {
        firedNm = maxPosNm = maxNegNm = 0.0;
        if (vehicle?.Parts == null)
            return false;
        ThrusterController[] modules = Thrusters(vehicle);
        if (modules.Length == 0
            || !ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var states))
            return false;
        ReadOnlySpan<RocketState> rocketStates = RocketStates.Of(vehicle.Parts);
        float3 com = vehicle.TotalMassPropsAsmb.Offset;
        float ambient = vehicle.PhysicsEnvironment.AtmosphericPressure;
        var assembly = new RcsCtrlFrame(floatQuat.Identity);
        double step = Math.Max(stepS, 1e-3);
        foreach (ThrusterController module in modules)
        {
            RcsWrenchTable.ComputeLive(module, rocketStates, com, ambient, in assembly,
                out float3 force, out float3 torque, out float flow);
            double along = double3.Dot(new double3(torque.X, torque.Y, torque.Z), axisAsmb);
            if (!double.IsFinite(along))
                continue;
            if (along > 0.0) maxPosNm += along;
            else maxNegNm -= along;
            ThrusterControllerState st = states.States[module.StatesIdx];
            double share = Math.Clamp(st.CommandPulseTime / step, 0.0, 1.0);
            if (st.IsPropellantAvailable && share > 0.0)
                firedNm += share * along;
        }
        return true;
    }

    /// <summary>
    /// What the craft's RCS can do about each axis of the frame each way, into <paramref name="six"/> in the order +x, -x, +y, -y, +z, -z (RcsTorqueAllocator.Characterise). False when it has no usable thruster.
    /// </summary>
    internal static bool TryCharacterise(Vehicle vehicle, double3 ax, double3 ay, double3 az,
                                         Span<RcsTorqueAllocator.AxisAuthority> six)
    {
        six.Clear();
        RcsTorqueAllocator? alloc = Measure(vehicle, ax, ay, az, out _);
        if (alloc == null)
            return false;
        bool any = false;
        for (int axis = 0; axis < 3; axis++)
            for (int s = 0; s < 2; s++)
            {
                RcsTorqueAllocator.AxisAuthority a = alloc.Characterise(axis, s == 0 ? 1 : -1);
                six[2 * axis + s] = a;
                any |= a.MaxTorqueNm > 0.0;
            }
        return any;
    }

    /// <summary>
    /// Fire this craft's RCS for a torque, N m, in vehicle assembly axes, from now until the next command or <see cref="Release"/>. The frame (<paramref name="ax"/>, <paramref name="ay"/>, <paramref name="az"/>) is the one <paramref name="priority"/> weights the axes in: where the demand is out of reach, the axis with the smallest weight gives way first. Main thread; call it every step the torque is wanted.
    /// </summary>
    internal static Report Command(Vehicle vehicle, double3 torqueAsmb,
                                   double3 ax, double3 ay, double3 az, ReadOnlySpan<double> priority)
    {
        if (vehicle == null)
            return default;
        Slot slot = ByVehicle.GetValue(vehicle, _ => new Slot());
        slot.Owner = vehicle;
        RcsTorqueAllocator? alloc = Measure(vehicle, ax, ay, az, out ThrusterController[] modules);
        if (alloc == null)
        {
            Publish(slot, null);
            slot.Last = default;
            return default;
        }

        double dx = double3.Dot(torqueAsmb, ax), dy = double3.Dot(torqueAsmb, ay), dz = double3.Dot(torqueAsmb, az);
        var duty = new double[alloc.Count];
        RcsTorqueAllocator.Result r = alloc.Allocate(dx, dy, dz, priority, duty);
        if (!r.Solved)
        {
            Publish(slot, null);
            slot.Last = default;
            return default;
        }

        // The worker finds the slot through any of the craft's thrusters.
        foreach (ThrusterController m in modules)
            ByThruster.AddOrUpdate(m, slot);
        Publish(slot, new Pulses(modules, duty));
        slot.Last = new Report(true, new double3(dx, dy, dz),
            new double3(r.TorqueX, r.TorqueY, r.TorqueZ), new double3(r.ForceX, r.ForceY, r.ForceZ),
            r.FlowKgS, r.ShortfallNm, r.Firing, alloc.Count);
        return slot.Last;
    }

    /// <summary>Stop firing for this craft. The worker writes nothing more, so stock's own command (none, unless the player fires RCS) stands from the next step.</summary>
    internal static void Release(Vehicle vehicle)
    {
        if (vehicle != null && ByVehicle.TryGetValue(vehicle, out Slot? slot))
        {
            Publish(slot, null);
            slot.Last = default;
            slot.Reached = 0;
        }
    }

    /// <summary>The last command's report for this craft, for the log and the panel.</summary>
    internal static Report Last(Vehicle vehicle)
        => vehicle != null && ByVehicle.TryGetValue(vehicle, out Slot? slot) ? slot.Last : default;

    /// <summary>How many thrusters the worker's last pass matched to this craft's duties (see Slot.Reached).</summary>
    internal static int Reached(Vehicle vehicle)
        => vehicle != null && ByVehicle.TryGetValue(vehicle, out Slot? slot) ? slot.Reached : 0;

    /// <summary>
    /// Runs on the vehicle worker from VehicleCommandSink.Run, after stock has written this step's thruster commands: writes each thruster's pulse for the current period into its module state. Reads nothing from the flight computer; the thruster states it is handed say which craft this is.
    /// </summary>
    internal static void OnComputeControl(in FlightComputerNavigation nav,
                                          ref FlightComputerOutput outputs, ref VehicleCommandSink.Receipt receipt)
    {
        if (Volatile.Read(ref _live) <= 0)
            return;

        // Which craft: the slot any of its thrusters was registered under, found through the read-only states so the search marks nothing changed.
        Slot? st = null;
        var find = outputs.Thrusters.ModulesAndReadOnlyStates.GetEnumerator();
        while (find.MoveNext())
        {
            ThrusterController? module = find.Current.Module;
            if (module != null && ByThruster.TryGetValue(module, out st))
                break;
        }
        if (st == null)
            return;

        // ONE read of the published duties.
        Pulses? cmd = st.Cmd;
        if (cmd == null)
        {
            st.PeriodValid = false;
            return;
        }
        // A craft out of control fires nothing, as with the player's inputs.
        Vehicle? owner = st.Owner;
        if (owner != null && (!owner.IsControllable || VehicleControlOwnership.KfsOwns(owner)))
            return;

        int n = cmd.Modules.Length;
        if (st.FireUntil.Length != n)
        {
            st.FireUntil = new double[n];
            st.Carry = new double[n];
            st.PeriodValid = false;
        }

        double elapsed = st.PeriodValid ? (nav.Time - st.PeriodStart).Seconds() : double.PositiveInfinity;
        if (!(elapsed >= 0.0) || elapsed >= 0.999 * PeriodSec)
        {
            st.PeriodStart = nav.Time;
            st.PeriodValid = true;
            elapsed = 0.0;
            for (int i = 0; i < n; i++)
            {
                double want = Math.Clamp(cmd.Duty[i], 0.0, 1.0) * PeriodSec + st.Carry[i];
                double minPulse = Math.Max(cmd.Modules[i].MinimumPulseTime, 0f);
                if (want > 0.0 && want >= minPulse)
                {
                    double on = Math.Min(want, PeriodSec);
                    st.FireUntil[i] = on;
                    st.Carry[i] = Math.Min(want - on, PeriodSec);
                }
                else
                {
                    st.FireUntil[i] = 0.0;
                    st.Carry[i] = want;
                }
            }
        }

        double nextWake = PeriodSec - elapsed;
        int reached = 0;
        var enumerator = outputs.Thrusters.GetModulesAndNewStates().GetEnumerator();
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            // Duties solved for another set of thrusters (staging, a thruster run dry) find no match here and fire nothing until the next command.
            if (current.Module == null || !cmd.Index.TryGetValue(current.Module, out int i))
                continue;
            ref ThrusterControllerState state = ref current.State;
            reached++;
            if (!state.IsPropellantAvailable)
                continue;
            double left = st.FireUntil[i] - elapsed;
            if (left <= 1e-4)
                continue;
            if (left > state.CommandPulseTime)
                state.CommandPulseTime = (float)left;
            state.CommandTime = nav.Time;
            receipt.Command();
        }
        st.Reached = reached;
        // A control step at the start of the next period only. A pulse ends on its own; asking for a step at each pulse's end, as the first version did (down to 1 ms apart), made the game take several extra physics steps a period, and stutter.
        receipt.Wake(Math.Max(nextWake, 0.01));
    }
}
