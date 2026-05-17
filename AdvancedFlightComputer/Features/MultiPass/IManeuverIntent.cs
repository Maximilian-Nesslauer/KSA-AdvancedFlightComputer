using System.IO;
using AdvancedFlightComputer.Features.ManeuverTools;
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

    /// <summary>Maneuver from <paramref name="vehicle"/>'s current orbit
    /// toward this intent's locked goal. Independent of any live UI
    /// input. Null when the goal is unreachable from the current state.</summary>
    OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle);

    /// <summary>True when the goal is already met. Distinguishes
    /// "rotated to within tolerance" from "cannot make progress" so the
    /// postfix can complete the execution instead of warning-cancelling
    /// on RecomputePass failures during the converging tail of a
    /// multi-pass plan (common when the splitter over-allocates dV per
    /// pass relative to the asymptotically-shrinking remaining angle).</summary>
    bool IsSatisfied(Vehicle vehicle);

    PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode);

    void WriteToToml(TextWriter w);
}
