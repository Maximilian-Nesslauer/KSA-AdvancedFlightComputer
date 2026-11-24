using KSA;

namespace AdvancedFlightComputer.Core;

internal static class ProbePower
{
    [ThreadStatic] private static Dictionary<int, (Part Representative, bool HasActiveCore, double StandbyWatts)>? _circuits;
    [ThreadStatic] private static HashSet<PowerConsumer>? _seen;
    [ThreadStatic] private static Dictionary<int, PowerForecast>? _estimates;

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
        var circuits = _circuits ??= new Dictionary<int, (Part Representative, bool HasActiveCore, double StandbyWatts)>();
        var seen = _seen ??= new HashSet<PowerConsumer>();
        var estimates = _estimates ??= new Dictionary<int, PowerForecast>();
        circuits.Clear();
        seen.Clear();
        estimates.Clear();
        try
        {
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

            if (circuits.Count == 0)
                return double.NaN;

            double handover = 0.0;
            foreach (var (index, circuit) in circuits)
            {
                PowerForecast estimate = PowerForecast.Compute(tree, circuit.Representative);
                estimates.Add(index, estimate);
                if (!circuit.HasActiveCore)
                    continue;
                if (estimate.Remaining.IsEndOfTime())
                    return double.PositiveInfinity;
                handover = Math.Max(handover, estimate.Remaining.Seconds());
            }

            // When the last powered primary runs out, stock wakes the eligible Auto cores together.
            double remaining = handover;
            foreach (var (index, circuit) in circuits)
            {
                PowerForecast estimate = estimates[index];
                if (!(circuit.StandbyWatts > 0.0) || estimate.Drain == PowerDrain.NoBattery)
                    continue;
                double capacity = 0.0;
                foreach (Battery battery in tree.ElectricalCircuits.Batteries(index))
                    capacity += battery.MaximumCapacity.Value();
                double backgroundDrain = (estimate.Draw - estimate.Supply).Value();
                double charge = Math.Clamp(estimate.Charge.Value() - backgroundDrain * handover, 0.0, capacity);
                double activeDrain = backgroundDrain + circuit.StandbyWatts;
                if (activeDrain <= 0.0)
                    return double.PositiveInfinity;
                remaining = Math.Max(remaining, handover + charge / activeDrain);
            }

            return remaining;
        }
        finally
        {
            circuits.Clear();
            seen.Clear();
            estimates.Clear();
        }
    }

    internal static double UntilBurn(Vehicle vehicle, Burn node)
    {
        FlightComputer fc = vehicle.FlightComputer;
        UniverseTime time = ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), node)
            && fc.Burn is { } target && !target.IgnitionTime.IsEndOfTime() ? target.IgnitionTime : node.Time;
        return (time - Universe.GetElapsedTime()).Seconds();
    }
}
