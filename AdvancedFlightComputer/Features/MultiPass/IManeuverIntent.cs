using System.IO;
using AdvancedFlightComputer.Features.ManeuverTools;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Each intent keeps its goal while RecomputePass adjusts the next burn to the current vehicle state.
internal interface IManeuverIntent
{
    // Keep Kind stable across releases so saved intents can still be loaded.
    string Kind { get; }

    // TypeKey prevents the UI from showing a preview for a different maneuver while this intent is active.
    string TypeKey { get; }

    // Compute from the saved goal rather than current UI input.
    // Return null when the goal is unreachable.
    OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle);

    // Distinguish a completed goal from an unreachable one so execution can finish without reporting a failure.
    bool IsSatisfied(Vehicle vehicle);

    PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode);

    void WriteToToml(TextWriter w);
}

internal static class IntentPlanning
{
    // A failed preview can still supply the first valid pass.
    // Execution checks the next pass again before scheduling it.
    public static PassPlanResult FirstPass(PassPreviewResult result) => result.Passes.Length == 0
        ? PassPlanResult.Failure(result.FailureReason ?? "planner produced no passes")
        : PassPlanResult.Success(result.Passes[0]);
}
