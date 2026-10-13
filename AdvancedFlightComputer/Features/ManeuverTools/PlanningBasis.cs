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

        // The last finite burn piece carries the conic after cutoff, and later patches describe SOI transitions.
        // A plan without a finite burn piece starts with the orbit after an impulse at the burn's impulse time.
        PatchedConic patch = plan.Patches[0];
        for (int i = plan.Patches.Count - 1; i >= 0; i--)
        {
            if (plan.Patches[i].BurnTrajectory.HasValue)
            {
                patch = plan.Patches[i];
                break;
            }
        }

        // A finite burn piece starts at ignition, and its conic is valid only from the cutoff, so a chained maneuver may not start before the cutoff.
        UniverseTime earliest = finalBurn.ImpulseTime;
        UniverseTime conicStart = patch.ConicStartTime;
        if (conicStart > earliest)
            earliest = conicStart;
        return new PlanningBasis(patch.Orbit, patch, earliest, true, plan);
    }
}
