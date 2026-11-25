#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using KSA;

/// <summary>
/// The parachutes of a craft, as KSA models them: a Parachute module per canopy, stowed until armed. Arm is what the part menu's ARM does (Parachute.ShowContextMenu): the chute then deploys itself once the air is safe for it and the craft is under its deploy altitude (ParachutePhysics.ShouldAutoDeploy, in Parachute.UpdateModules), so AFC never picks the moment, the part's own DEPLOY ALTITUDE does. A chute that is not fitted or not enabled does not arm.
/// </summary>
internal static class KsaParachutes
{
    /// <summary>The fitted, enabled parachutes of <paramref name="vehicle"/>.</summary>
    public static int Count(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 0;
        int count = 0;
        foreach (Parachute chute in tree.Modules.Get<Parachute>())
            if (chute.Fitted && chute.IsEnabled)
                count++;
        return count;
    }

    /// <summary>Arms every stowed, fitted, enabled parachute of <paramref name="vehicle"/> and returns how many it armed.</summary>
    public static int ArmAll(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 0;
        int armed = 0;
        foreach (Parachute chute in tree.Modules.Get<Parachute>())
        {
            if (!chute.Fitted || !chute.IsEnabled || chute.State != ChuteState.Stowed)
                continue;
            chute.Arm();
            if (chute.State == ChuteState.Armed)
                armed++;
        }
        return armed;
    }

    /// <summary>Whether any parachute of <paramref name="vehicle"/> is open now: line stretch to full. A chute cut away or torn before the entry does not count, or the entry would let go at its hand-over.</summary>
    public static bool AnyOpen(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return false;
        foreach (Parachute chute in tree.Modules.Get<Parachute>())
            if (chute.State is ChuteState.LineStretch or ChuteState.Reefed or ChuteState.Disreefing or ChuteState.Full)
                return true;
        return false;
    }
}
