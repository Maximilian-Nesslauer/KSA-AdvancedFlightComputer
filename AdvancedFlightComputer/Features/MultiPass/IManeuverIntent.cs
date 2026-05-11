using System.IO;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// What the multi-pass plan is trying to achieve. RecomputePass is
/// called per-pass against the vehicle's current state, so execution
/// drift from earlier passes is corrected on each commit.
///
/// Concrete implementations are self-serializable: WriteToToml plus a
/// static FromToml factory, dispatched on <see cref="Kind"/>.
/// </summary>
internal interface IManeuverIntent
{
    /// <summary>Discriminator for TOML serialization (e.g. "set-ap").
    /// Stable across versions or saved games fail to deserialize.</summary>
    string Kind { get; }

    PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode);

    void WriteToToml(TextWriter w);
}
