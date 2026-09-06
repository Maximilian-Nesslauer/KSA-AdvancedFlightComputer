using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Estimate reachable liquid RCS propellant from ResourceManagerBase.ConsumptionOrder. Assume one reactant mix and ignore depletion order, so this is an upper bound used for a warning.</summary>
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
                // This estimate covers liquid reactants. Solid cores have no tank graph.
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
        // ConsumptionOrder is null until the resource graph exists.
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
