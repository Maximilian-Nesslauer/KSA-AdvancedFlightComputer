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
        double remaining = double.NaN;
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

            PowerForecast estimate = PowerForecast.Compute(tree, part);
            double seconds = estimate.Remaining.IsEndOfTime() ? double.PositiveInfinity : estimate.Remaining.Seconds();
            // Stock excludes an Auto core on standby from demand. It needs its own draw when it takes over.
            if (estimate.Drain != PowerDrain.NoBattery && !consumer.IsWanted(tree.PowerConsumers.States))
            {
                double drain = (estimate.Draw + consumer.Consumed - estimate.Supply).Value();
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
