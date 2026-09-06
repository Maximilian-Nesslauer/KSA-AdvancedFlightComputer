using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>The driver publishes one command payload for the worker each tick while execution is active. The payload is immutable. Only its consumption receipt can change.</summary>
internal sealed class RcsWorkerCommand
{
    private int _consumed;
    internal bool WasConsumed => Volatile.Read(ref _consumed) != 0;
    internal void MarkConsumed() => Volatile.Write(ref _consumed, 1);

    /// <summary>The driver has taken over and translation pulses should fire once the ignition time and attitude gate allow.</summary>
    public required bool Active { get; init; }

    /// <summary>Construct the ignition time on the main thread because UniverseTime rejects NaN. A conversion failure inside the worker would interrupt the physics step.</summary>
    public UniverseTime IgnitionTime { get; init; }

    /// <summary>Align requires pitch and yaw errors inside the firing gate. Roll about the aligned axis does not change the thrust direction. Hold does not apply an attitude gate.</summary>
    public bool RequireAttitude { get; init; }

    /// <summary>Limit each pulse in seconds to the stock burn control period.</summary>
    public float MaxPulseSec { get; init; }

    /// <summary>Pass separate forces and minimum impulses for each signed axis. Stock authority uses the smaller value of both signs and can report zero for a vehicle that has thrusters in only one direction.</summary>
    public float3 AxisForcePos { get; init; }
    public float3 AxisForceNeg { get; init; }
    public float3 AxisMinImpulsePos { get; init; }
    public float3 AxisMinImpulseNeg { get; init; }

    /// <summary>Store seconds of firing per N s of net impulse along LpDirCtrl. Indices match VehicleConfig.Thrusters. The worker checks the length because staging can replace that list between driver ticks. Null selects axis groups.</summary>
    public float[]? LpSecondsPerImpulse { get; init; }
    public float3 LpDirCtrl { get; init; }

    /// <summary>Impulse ceiling per control period so no single pulse in the LP pattern exceeds <see cref="MaxPulseSec"/>.</summary>
    public float LpImpulseCapNs { get; init; }
}

/// <summary>FlightComputer.CopyFrom shares BurnPlan between the vehicle and worker flight computers. Use that shared reference as the command key because the flight computer instances differ.</summary>
internal static class RcsCommandChannel
{
    private static readonly ConditionalWeakTable<BurnPlan, RcsWorkerCommand> _commands = new();

    public static void Publish(BurnPlan plan, RcsWorkerCommand command)
        => _commands.AddOrUpdate(plan, command);

    public static void Clear(BurnPlan plan)
    {
        // Avoid taking the table write lock when this vehicle has no published command.
        if (_commands.TryGetValue(plan, out _))
            _commands.Remove(plan);
    }

    public static bool TryGet(BurnPlan plan, out RcsWorkerCommand command)
        => _commands.TryGetValue(plan, out command!);

    public static void Reset() => _commands.Clear();
}
