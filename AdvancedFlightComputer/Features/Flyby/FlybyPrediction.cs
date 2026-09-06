using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

internal enum FlybyPredictionStatus
{
    NoDeparture,
    NoCoveringPatch,
    PropagationFailed,
    NoEncounter,
    InvalidPeriapsis,
    Available,
}

internal readonly record struct FlybyPrediction(
    FlybyPredictionStatus Status, double? PeriapsisRadius = null)
{
    internal string Reason => Status switch
    {
        FlybyPredictionStatus.NoCoveringPatch => "No prediction: no flight-plan patch covers the departure time.",
        FlybyPredictionStatus.PropagationFailed => "No prediction: trajectory propagation failed.",
        FlybyPredictionStatus.NoEncounter => "No prediction: the preview did not resolve a target encounter.",
        FlybyPredictionStatus.InvalidPeriapsis => "No prediction: the target encounter has no finite periapsis.",
        FlybyPredictionStatus.NoDeparture => "No prediction: no valid flyby departure is available.",
        _ => string.Empty,
    };

    internal static FlybyPrediction FromPlan(FlightPlan plan, IParentBody target)
    {
        foreach (PatchedConic patch in plan.Patches)
        {
            if (patch.Orbit.Parent?.Id != target.Id) continue;
            double radius = patch.Orbit.Periapsis;
            return double.IsFinite(radius)
                ? new(FlybyPredictionStatus.Available, radius)
                : new(FlybyPredictionStatus.InvalidPeriapsis);
        }
        return new(FlybyPredictionStatus.NoEncounter);
    }
}
