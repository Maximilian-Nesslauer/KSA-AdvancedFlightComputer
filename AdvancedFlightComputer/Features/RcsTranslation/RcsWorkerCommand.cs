using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Immutable per-vehicle command the main-thread driver publishes for the
/// worker-side ComputeControl postfix, only while an execution is running.
/// Replaced wholesale each driver tick; the worker only ever reads one
/// reference, so no locking is needed.
/// </summary>
internal sealed class RcsWorkerCommand
{
    /// <summary>The driver has taken over and translation pulses should fire
    /// once the ignition time and attitude gate allow.</summary>
    public required bool Active { get; init; }

    /// <summary>Built on the main thread rather than from a raw seconds value
    /// the worker converts: UniverseTime throws on a NaN construction, and the
    /// worker runs inside the physics step where that would take the frame
    /// down rather than one burn.</summary>
    public UniverseTime IgnitionTime { get; init; }

    /// <summary>Fire only when the pitch/yaw error angles are inside the
    /// align gate (Align strategy; see RcsExecutor.OutsideAlignGate). Roll
    /// about the aligned axis does not move the thrust vector, the same
    /// reason stock's burn gate ignores it. Hold fires through any
    /// attitude.</summary>
    public bool RequireAttitude { get; init; }

    /// <summary>Upper bound on a single commanded pulse, seconds. Keeps the
    /// loop closed at the stock burn-control cadence.</summary>
    public float MaxPulseSec { get; init; }

    /// <summary>Per-signed-axis group force and minimum impulse, probed on
    /// the main thread. Passed here because the game's cached authority is
    /// the conservative min of both signs, which reads zero for one-sided
    /// layouts (a forward-only probe would otherwise never fire).</summary>
    public float3 AxisForcePos { get; init; }
    public float3 AxisForceNeg { get; init; }
    public float3 AxisMinImpulsePos { get; init; }
    public float3 AxisMinImpulseNeg { get; init; }

    /// <summary>LP allocation: seconds of firing per newton-second of net
    /// impulse along <see cref="LpDirCtrl"/>, index-aligned with
    /// VehicleConfig.Thrusters. Null runs the axis-group path. The worker
    /// verifies the length against the live thruster list because staging
    /// can swap VehicleConfig between driver ticks.</summary>
    public float[]? LpSecondsPerImpulse { get; init; }
    public float3 LpDirCtrl { get; init; }

    /// <summary>Impulse ceiling per control period so no single pulse in
    /// the LP pattern exceeds <see cref="MaxPulseSec"/>.</summary>
    public float LpImpulseCapNs { get; init; }
}

/// <summary>
/// Bridge between the main-thread driver and the worker-side FlightComputer
/// copy. Keyed by BurnPlan because FlightComputer.CopyFrom shares the
/// BurnPlan reference between the vehicle instance and the worker copy,
/// while the FlightComputer instances themselves differ.
/// </summary>
internal static class RcsCommandChannel
{
    private static readonly ConditionalWeakTable<BurnPlan, RcsWorkerCommand> _commands = new();

    public static void Publish(BurnPlan plan, RcsWorkerCommand command)
        => _commands.AddOrUpdate(plan, command);

    public static void Clear(BurnPlan plan)
    {
        // Runs every tick for every burn-carrying vehicle that does not
        // resolve to RCS; skip the table's write lock when nothing is there.
        if (_commands.TryGetValue(plan, out _))
            _commands.Remove(plan);
    }

    public static bool TryGet(BurnPlan plan, out RcsWorkerCommand command)
        => _commands.TryGetValue(plan, out command!);

    public static void Reset() => _commands.Clear();
}
