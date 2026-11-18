using KSA;

namespace AdvancedFlightComputer.Core;

internal static class ProbePower
{
    /// <summary>Seconds until the last usable core runs out at current generation. NaN means there is no forecast.</summary>
    internal static double RemainingSeconds(Vehicle vehicle)
    {
        if (vehicle.IsDisposed || vehicle.Parts is not { } tree)
            return double.NaN;
        Span<Control> controls = tree.Modules.Get<Control>();
        if (controls.IsEmpty)
            return double.NaN;

        tree.EnsureHabitatZones();
        UniverseTime now = Universe.GetElapsedTime();
        var circuits = new Dictionary<int, (Part Representative, bool HasActiveCore, double StandbyWatts)>();
        var seen = new HashSet<PowerConsumer>();
        foreach (Control control in controls)
        {
            Part part = control.Parent;
            if (HabitatZone.FindZone(tree, part) is { } zone)
            {
                foreach (IVASeat seat in zone.Seats)
                    if (seat.AssignedKittenHash != KeyHash.Zero
                        && Universe.KittenRoster.Find(seat.AssignedKittenHash) is { Kia: false } kitten
                        && (!Universe.Rules.LifeSupport || !kitten.Condition.IsIncapacitated(now)))
                        return double.PositiveInfinity;
                continue;
            }

            if (Universe.Rules.Radiation && part.Modules.Get<Avionics>() is { IsEmpty: false } avionics
                && (avionics[0].ComputeHealth() == AvionicsHealth.Failed || avionics[0].IsRebooting(now.Seconds())))
                continue;
            PowerConsumer? consumer = null;
            foreach (PowerConsumer candidate in part.Modules.Get<PowerConsumer>())
            {
                if (!candidate.PowerSwitch)
                    continue;
                consumer = candidate;
                break;
            }
            if (consumer == null)
                return double.PositiveInfinity;
            if (consumer.Mode == PowerSwitchMode.Off || Universe.Rules.Radiation
                && consumer.Hold is PowerHold.Failed or PowerHold.Rebooting or PowerHold.StormStandby)
                continue;
            if (!seen.Add(consumer))
                continue;

            int index = tree.ElectricalCircuits.IndexFor(part.FullPart);
            bool wanted = consumer.IsWanted(tree.PowerConsumers.States);
            circuits.TryGetValue(index, out var prior);
            circuits[index] = (part, prior.HasActiveCore || wanted, prior.StandbyWatts + (wanted ? 0.0 : consumer.Consumed.Value()));
        }

        double remaining = double.NaN;
        foreach (var circuit in circuits.Values)
        {
            PowerForecast estimate = PowerForecast.Compute(tree, circuit.Representative);
            double seconds = estimate.Remaining.IsEndOfTime() ? double.PositiveInfinity : estimate.Remaining.Seconds();
            // When control is lost on the primary circuit, stock wakes its eligible backups together.
            if (!circuit.HasActiveCore && estimate.Drain != PowerDrain.NoBattery)
            {
                double drain = (estimate.Draw - estimate.Supply).Value() + circuit.StandbyWatts;
                seconds = drain > 0.0 ? estimate.Charge.Value() / drain : double.PositiveInfinity;
            }
            remaining = double.IsNaN(remaining) ? seconds : Math.Max(remaining, seconds);
        }
        return remaining;
    }

    internal static double UntilBurn(Vehicle vehicle, Burn node)
    {
        FlightComputer fc = vehicle.FlightComputer;
        UniverseTime time = ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), node)
            && fc.Burn is { } target && !target.IgnitionTime.IsEndOfTime() ? target.IgnitionTime : node.Time;
        return (time - Universe.GetElapsedTime()).Seconds();
    }
}
