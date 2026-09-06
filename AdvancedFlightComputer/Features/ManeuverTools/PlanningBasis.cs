using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// A chained maneuver uses the trajectory after the preceding burn and cannot start before it.
internal readonly record struct PlanningBasis(
    Orbit Orbit, PatchedConic? Patch, UniverseTime Earliest, bool IsChained, FlightPlan? Plan)
{
    // Keep the plan with its patch so burn creation and preview use the same trajectory.
    public static PlanningBasis For(Vehicle source)
    {
        UniverseTime now = Universe.GetElapsedTime();

        Burn? finalBurn = source.FlightComputer.BurnPlan.FindFinalNontrivialBurn();
        if (finalBurn == null)
            return new PlanningBasis(source.Orbit, source.FlightPlan.TryFindPatch(now), now, false, null);

        FlightPlan plan = finalBurn.FlightPlan;
        if (plan.Patches.Count == 0)
            return new PlanningBasis(source.Orbit, source.FlightPlan.TryFindPatch(now), now, false, null);

        // Patch zero describes the orbit after the burn, while later patches describe SOI transitions.
        PatchedConic patch = plan.Patches[0];
        return new PlanningBasis(patch.Orbit, patch, finalBurn.Time, true, plan);
    }
}
