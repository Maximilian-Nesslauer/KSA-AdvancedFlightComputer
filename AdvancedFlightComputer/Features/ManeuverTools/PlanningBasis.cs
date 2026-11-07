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
        if (ConicAfterBurn(plan) is not PatchedConic patch)
            return new PlanningBasis(source.Orbit, source.FlightPlan.TryFindPatch(now), now, false, null);

        // A finite burn piece starts at ignition, and its conic is valid only from the cutoff, so a chained maneuver may not start before the cutoff.
        UniverseTime earliest = finalBurn.ImpulseTime;
        UniverseTime conicStart = patch.ConicStartTime;
        if (conicStart > earliest)
            earliest = conicStart;
        return new PlanningBasis(patch.Orbit, patch, earliest, true, plan);
    }

    /// <summary>The patch of the orbit after the burn of <paramref name="plan"/>, or null for a plan with no patches. The last finite burn piece carries the conic after cutoff; later patches are SOI transitions.</summary>
    public static PatchedConic? ConicAfterBurn(FlightPlan plan)
    {
        if (plan.Patches.Count == 0)
            return null;
        for (int i = plan.Patches.Count - 1; i >= 0; i--)
        {
            if (plan.Patches[i].BurnTrajectory.HasValue)
                return plan.Patches[i];
        }
        return plan.Patches[0];
    }
}
