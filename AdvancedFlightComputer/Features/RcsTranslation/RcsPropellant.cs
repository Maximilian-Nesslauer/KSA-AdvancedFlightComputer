using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Estimate reachable liquid RCS propellant from ResourceManagerBase.ConsumptionOrder. Partition reachable tanks by reactant mix and ignore depletion order, so this is an upper bound used for a warning.</summary>
internal static class RcsPropellant
{
    public static double AvailableKg(Vehicle vehicle)
    {
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        List<(ReactantMix Mix, HashSet<Tank> Tanks)> mixGroups = new();

        foreach (ThrusterController thruster in vehicle.Parts.Modules.Get<ThrusterController>())
        {
            if (!thruster.IsActive)
                continue;
            foreach (Rocket rocket in thruster.Rockets)
            foreach (RocketCore core in rocket.Cores)
            {
                // This estimate covers tank-fed cores. Solid cores have no tank graph.
                if (core is not PlumbedCore plumbed || plumbed.DesiredMix == null)
                    continue;
                HashSet<Tank> tanks = GetOrAddMixTanks(mixGroups, plumbed.DesiredMix);
                CollectReachableTanks(plumbed.ResourceManager, tanks);
            }
        }
        if (mixGroups.Count == 0)
            return 0.0;

        double totalUsable = 0.0;
        for (int g = 0; g < mixGroups.Count; g++)
        {
            (ReactantMix mix, HashSet<Tank> tanks) = mixGroups[g];
            if (tanks.Count == 0)
                continue;
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
            if (double.IsFinite(usable) && usable > 0.0)
                totalUsable += usable;
        }
        return totalUsable;
    }

    private static HashSet<Tank> GetOrAddMixTanks(
        List<(ReactantMix Mix, HashSet<Tank> Tanks)> groups, ReactantMix mix)
    {
        for (int i = 0; i < groups.Count; i++)
        {
            if (ReferenceEquals(groups[i].Mix, mix) || groups[i].Mix.SharesReactantSet(mix))
                return groups[i].Tanks;
        }
        HashSet<Tank> tanks = new();
        groups.Add((mix, tanks));
        return tanks;
    }

    private static void CollectReachableTanks(ResourceManager? rm, HashSet<Tank> tanks)
    {
        if (rm == null)
            return;
        // The order has no levels until the resource graph exists.
        FlowOrder<Tank> levels = rm.ConsumptionOrder;
        for (int i = 0; i < levels.LevelCount; i++)
        {
            foreach (Tank? tank in levels[i])
            {
                if (tank != null)
                    tanks.Add(tank);
            }
        }
    }
}
