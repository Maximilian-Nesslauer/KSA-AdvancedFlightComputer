using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Estimates the propellant mass currently reachable by the vehicle's
/// active RCS thrusters. Reads each Combustor core's resource-manager
/// ConsumptionOrder (the flow-rule-selected reachable tank levels the
/// game's own consumption path walks), sums reactant masses over the
/// distinct reachable tanks, and bounds the usable total by the reactant
/// mix's mass fractions.
///
/// Approximations, acceptable for a warning: all thrusters are assumed to
/// share one reactant mix (the first one found), and reachability ignores
/// depletion order, so this is an upper bound on what the thrusters can
/// actually burn.
/// </summary>
internal static class RcsPropellant
{
    public static double AvailableKg(Vehicle vehicle)
    {
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        HashSet<Tank> tanks = new();
        ReactantMix? mix = null;

        foreach (ThrusterController thruster in vehicle.Parts.Modules.Get<ThrusterController>())
        {
            if (!thruster.IsActive)
                continue;
            foreach (RocketCore core in thruster.Cores)
            {
                // Only a liquid core (Combustor) carries a reactant mix and a
                // resource-manager tank graph; solid motors carry neither, so
                // any non-Combustor core is skipped. RCS thrusters are liquid.
                if (core is not Combustor combustor)
                    continue;
                mix ??= combustor.DesiredMix;
                CollectReachableTanks(combustor.ResourceManager, tanks);
            }
        }
        if (mix == null || tanks.Count == 0)
            return 0.0;

        double usable = double.PositiveInfinity;
        foreach (Reactant reactant in mix.Reactants)
        {
            double reactantMass = 0.0;
            foreach (Tank tank in tanks)
            {
                if (!tank.PropellantUseEnabled)
                    continue;
                if (tank.TryGetMoleAndState(reactant.SubstancePhase, moleStates, out var mole))
                    reactantMass += mole.State.Mass;
            }
            if (reactant.MassFraction > 0f)
                usable = Math.Min(usable, reactantMass / reactant.MassFraction);
        }
        return double.IsFinite(usable) ? usable : 0.0;
    }

    private static void CollectReachableTanks(ResourceManager? rm, HashSet<Tank> tanks)
    {
        // ConsumptionOrder selects the flow-rule's reachable tank levels the
        // same way the game's own consumption path does (ResourceManagerBase),
        // so it stays correct if the flow-rule set changes; null before the
        // manager's graph is built.
        Tank[][]? groups = rm?.ConsumptionOrder;
        if (groups == null)
            return;
        foreach (Tank[]? group in groups)
        {
            if (group == null)
                continue;
            foreach (Tank? tank in group)
            {
                if (tank != null)
                    tanks.Add(tank);
            }
        }
    }
}
