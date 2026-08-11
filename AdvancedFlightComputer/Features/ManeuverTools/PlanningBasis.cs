using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// The trajectory a maneuver is planned against, and the earliest time a new burn
/// on it may be placed.
///
/// Without this every tool planned from <see cref="Vehicle.Orbit"/>, the orbit the
/// vehicle is on right now. A pending burn does not change that orbit, so a second
/// maneuver chained onto the first read the pre-burn apsides: raising apoapsis and
/// then trying to raise periapsis was rejected against the apoapsis the first burn
/// was about to replace.
/// </summary>
internal readonly record struct PlanningBasis(
    Orbit Orbit, PatchedConic? Patch, UniverseTime Earliest, bool IsChained, FlightPlan? Plan)
{
    /// <summary>Resolves what to plan against: the trajectory after the last burn
    /// that actually changes it, or the live orbit when the plan is empty. Mirrors
    /// stock's own rule that a chained burn may only be created on the final
    /// post-burn trajectory (see BurnContextMenu.TryPlaceBurn). <see cref="Plan"/>
    /// carries the chained flight plan so the created burn can anchor on the same
    /// trajectory the numbers were computed from; null when planning off the live
    /// orbit.</summary>
    public static PlanningBasis For(Vehicle source)
    {
        UniverseTime now = Universe.GetElapsedTime();

        Burn? finalBurn = source.FlightComputer.BurnPlan.FindFinalNontrivialBurn();
        if (finalBurn == null)
            return new PlanningBasis(source.Orbit, source.FlightPlan.TryFindPatch(now), now, false, null);

        FlightPlan plan = source.FindFinalFlightPlan();
        if (plan.Patches.Count == 0)
            return new PlanningBasis(source.Orbit, source.FlightPlan.TryFindPatch(now), now, false, null);

        // Patch 0 is the orbit the burn puts the vehicle on; later patches are the
        // SOI transitions that follow it, which are not what an apse or inclination
        // maneuver around the current parent operates on.
        PatchedConic patch = plan.Patches[0];
        return new PlanningBasis(patch.Orbit, patch, finalBurn.Time, true, plan);
    }
}
