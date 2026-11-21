using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using KSA;
using V3 = AdvancedFlightComputer.Features.Guidance.IdleEngineParking.V3;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// The KSA side of <see cref="IdleEngineParking"/>: the engines of the craft as nozzle exits in the vehicle frame, the plan of the idle gimbals' commands, and the writer that slews those gimbals there on the vehicle worker.
///  WHAT IT WRITES. Only the gimbals whose engine is not activated (GimbalController.HasActiveEngine). Those are outside FlightComputer.VehicleConfig.Gimbals, so neither stock TVC nor the allocation (KsaGimbalControl.ApplyLsq) writes them, and Gimbal.UpdateModules still moves every gimbal to its command, which is what the game draws (the part model follows GimbalState.Gimbal2Asmb). An engine that is not activated makes no thrust, so its deflection takes nothing from the live engines.
///  THE SLEW. A parked gimbal's NaturalFrequencyLimit is <see cref="ParkFrequency"/>, which caps its rate at <see cref="SlewRangePerS"/> of its range a second, so it swings there and back instead of snapping. The value also marks the gimbal as parked by AFC: one that holds it with no plan any more is slewed back to rest and then given back its own response.
/// </summary>
internal static class KsaIdleEngineParking
{
    /// <summary>How fast a parked gimbal swings, in shares of its range a second: half its range in a second, 10 deg in two seconds on a 10 deg gimbal.</summary>
    internal const float SlewRangePerS = 0.5f;

    /// <summary>The NaturalFrequencyLimit of a parked gimbal: Gimbal.UpdateModules caps the rate at this times Gimbal.RATE_CAP_PER_NATURAL_FREQUENCY times the range.</summary>
    internal const float ParkFrequency = SlewRangePerS / Gimbal.RATE_CAP_PER_NATURAL_FREQUENCY;

    private const float SettledRad = 1e-4f;

    /// <summary>The commands of the idle gimbals, in shares of their range, and how many of them move.</summary>
    internal sealed record Plan(GimbalController[] Gimbals, float[] CommandY, float[] CommandZ, int Parked, double LargestDeg);

    /// <summary>Changes when an engine is activated or shut off, or the craft's gimbals change, so a plan is rebuilt only then.</summary>
    internal static int Signature(Vehicle vehicle)
    {
        var hash = new HashCode();
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
            hash.Add(System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(gc));
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            hash.Add(System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(engine));
            hash.Add(engine.IsActive);
        }
        return hash.ToHashCode();
    }

    private sealed class Entry
    {
        internal GimbalController? Gimbal;
        internal bool Live;
        internal double3 Exit;
        internal double3 Direction;
        internal double RadiusM;
    }

    /// <summary>
    /// The plan for the craft as it is now, or null when it has no live engine or no idle gimbal to park. Engines more than LandingEngineLayout.AftPlaneDepthM forward of the aft-most live exit are left out, as in the landing engine picker, so a stack's upper stage engine is neither parked nor in the way.
    /// </summary>
    internal static Plan? Build(Vehicle vehicle)
    {
        var entries = new List<Entry>();
        var onGimbal = new HashSet<RocketNozzle>();
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
        {
            if (gc.Data.Nozzles.Count == 0 || gc.Gimbal == null)
                continue;
            Entry entry = new() { Gimbal = gc, Live = gc.HasActiveEngine() };
            int count = 0;
            foreach (RocketNozzle nozzle in gc.Data.Nozzles)
            {
                onGimbal.Add(nozzle);
                if (Add(entry, nozzle))
                    count++;
            }
            if (count > 0)
            {
                entry.Exit /= count;
                entries.Add(entry);
            }
        }
        foreach (RocketNozzle nozzle in vehicle.Parts.Modules.Get<RocketNozzle>())
        {
            if (onGimbal.Contains(nozzle) || nozzle.Rocket == null)
                continue;
            Entry entry = new() { Live = nozzle.Rocket.Controller?.IsActive == true };
            if (Add(entry, nozzle))
                entries.Add(entry);
        }

        double3 axis = double3.Zero;
        foreach (Entry e in entries)
            if (e.Live)
                axis += e.Direction;
        if (axis.LengthSquared() < 1e-12)
            return null;
        axis = axis.Normalized();
        double aft = double.PositiveInfinity;
        foreach (Entry e in entries)
            if (e.Live)
                aft = Math.Min(aft, double3.Dot(e.Exit, axis));
        entries.RemoveAll(e => double3.Dot(e.Exit, axis) - aft > LandingEngineLayout.AftPlaneDepthM);

        var engines = new IdleEngineParking.Engine[entries.Count];
        int idle = 0;
        for (int i = 0; i < entries.Count; i++)
        {
            Entry e = entries[i];
            if (e.Gimbal is not { } gc)
            {
                engines[i] = new IdleEngineParking.Engine(e.Live, e.RadiusM, [ToV3(e.Exit)], [0.0], [0.0]);
                continue;
            }
            (double Y, double Z)[] commands = IdleEngineParking.Commands(gc.Gimbal.ConstrainToCircle);
            var exits = new V3[commands.Length];
            for (int c = 0; c < commands.Length; c++)
                exits[c] = ToV3(Swing(gc, e.Exit, commands[c].Y, commands[c].Z));
            if (e.Live)
            {
                double reach = 0.0;
                foreach (V3 exit in exits)
                    reach = Math.Max(reach, V3.Distance(exit, exits[0]));
                engines[i] = new IdleEngineParking.Engine(true, e.RadiusM, [exits[0]], [0.0], [0.0], reach);
                continue;
            }
            engines[i] = new IdleEngineParking.Engine(false, e.RadiusM, exits,
                commands.Select(c => c.Y).ToArray(), commands.Select(c => c.Z).ToArray());
            idle++;
        }
        if (idle == 0)
            return null;

        int[] choice = IdleEngineParking.Solve(engines);
        var gimbals = new GimbalController[idle];
        var commandY = new float[idle];
        var commandZ = new float[idle];
        int next = 0, parked = 0;
        double largest = 0.0;
        for (int i = 0; i < entries.Count; i++)
        {
            if (entries[i].Gimbal is not { } gc || engines[i].Live)
                continue;
            double y = engines[i].CommandY[choice[i]], z = engines[i].CommandZ[choice[i]];
            gimbals[next] = gc;
            commandY[next] = (float)y;
            commandZ[next] = (float)z;
            next++;
            if (choice[i] != 0)
            {
                parked++;
                double degY = y * gc.Gimbal.AxisY.MaxAngle, degZ = z * gc.Gimbal.AxisZ.MaxAngle;
                largest = Math.Max(largest, Math.Sqrt(degY * degY + degZ * degZ) * 180.0 / Math.PI);
            }
        }
        return new Plan(gimbals, commandY, commandZ, parked, largest);
    }

    /// <summary>Adds one nozzle's exit (the plume's origin, RocketNozzle.FxLocationAsmb), thrust direction and exit radius to an engine, in the vehicle frame.</summary>
    private static bool Add(Entry entry, RocketNozzle nozzle)
    {
        if (nozzle.Parent is not { } part)
            return false;
        float3 at = nozzle.FxLocationAsmb.Transform(float4x4.Pack(part.MatrixAsmb2VehicleAsmb));
        float3 thrust = (-nozzle.ExhaustDirectionAsmb).Transform(floatQuat.Pack(part.Asmb2VehicleAsmb));
        entry.Exit += new double3(at.X, at.Y, at.Z);
        entry.Direction += new double3(thrust.X, thrust.Y, thrust.Z);
        entry.RadiusM = Math.Max(entry.RadiusM, nozzle.FxExitRadius);
        return true;
    }

    /// <summary>A point of the gimbal's nozzle at this command, vehicle frame: swung about the pivot in the gimbal's own frame, as RocketNozzle.UpdateState swings the plume and KsaTvcAllocator.ThrustLocation the thrust.</summary>
    internal static double3 Swing(GimbalController gc, double3 restVehicle, double commandY, double commandZ)
    {
        Gimbal g = gc.Gimbal;
        doubleQuat gimbalToVehicle = doubleQuat.Unpack(in gc.Data.Gimbal2VehicleAsmb);
        double3 pivot = KsaTvcAllocator.GimbalPivot(gc);
        double3 offset = (restVehicle - pivot).Transform(doubleQuat.Inverse(gimbalToVehicle));
        double angleY = Math.Clamp(commandY, -1.0, 1.0) * g.AxisY.MaxAngle;
        double angleZ = Math.Clamp(commandZ, -1.0, 1.0) * g.AxisZ.MaxAngle;
        return pivot + offset.Transform(Gimbal.ComputeGimbal2Asmb(angleY, angleZ)).Transform(gimbalToVehicle);
    }

    private static V3 ToV3(double3 v) => new(v.X, v.Y, v.Z);

    /// <summary>
    /// Runs on the vehicle worker after every other gimbal writer (KsaGimbalControl.OnComputeControl): holds each idle gimbal of the plan on its command, and slews a gimbal this parked with no plan any more back to rest. A gimbal whose engine is activated is never written here, whatever the plan says, so the allocation and stock TVC keep every live engine.
    /// </summary>
    internal static void Apply(Plan? plan, ref FlightComputerOutput outputs, ref VehicleCommandSink.Receipt receipt)
    {
        bool wrote = false;
        var enumerator = outputs.Gimbals.ModulesAndReadOnlyStates.GetEnumerator();
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            GimbalController gc = current.Module;
            if (gc?.Gimbal == null)
                continue;
            GimbalControllerState was = current.State;
            int k = plan != null ? Array.IndexOf(plan.Gimbals, gc) : -1;
            bool ours = was.NaturalFrequencyLimit == ParkFrequency;
            if ((k < 0 && !ours) || gc.HasActiveEngine())
                continue;

            float y = k >= 0 ? plan!.CommandY[k] : 0f;
            float z = k >= 0 ? plan!.CommandZ[k] : 0f;
            Gimbal g = gc.Gimbal;
            bool settled = Math.Abs(was.Angles.X - y * g.AxisY.MaxAngle) < SettledRad
                           && Math.Abs(was.Angles.Y - z * g.AxisZ.MaxAngle) < SettledRad
                           && Math.Abs(was.AngleRates.X) < SettledRad && Math.Abs(was.AngleRates.Y) < SettledRad;

            var slot = outputs.Gimbals.GetModuleAndNewState(gc);
            if (slot.Module == null)
                continue;
            slot.State.CommandY = y;
            slot.State.CommandZ = z;
            // Back at rest with no plan, the gimbal gets its own response back, which also ends the mark.
            slot.State.NaturalFrequencyLimit = k < 0 && settled ? float.PositiveInfinity : ParkFrequency;
            wrote = true;
        }
        if (wrote)
            receipt.Command();
    }
}
