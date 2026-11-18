using KSA;

namespace AdvancedFlightComputer.Core;

internal static class StageControl
{
    // A separation removes whole branches of the attachment tree. A zone or circuit path between
    // two selected parts stays within the selection, so crew and batteries can be filtered in place.
    private readonly record struct Selection(IReadOnlySet<Part>? Parts, bool Exclude)
    {
        public bool Contains(Part part) => Parts == null || Parts.Contains(part.FullPart) != Exclude;

        public Part StructuralPart(Part part)
        {
            Part full = part.FullPart;
            while (full.IsAttachedInternal && full.TreeParent is { } parent && Contains(parent))
                full = parent.FullPart;
            return full;
        }
    }

    /// <summary>Stock control for the selected parts. Null means there is no Control module.</summary>
    internal static VehicleControl? Evaluate(Vehicle vehicle, IReadOnlySet<Part>? parts = null, bool exclude = false)
    {
        PartTree tree = vehicle.Parts;
        Span<Control> controls = tree.Modules.Get<Control>();
        if (controls.IsEmpty)
            return null;

        tree.EnsureHabitatZones();
        var selection = new Selection(parts, exclude);
        UniverseTime now = Universe.GetElapsedTime();
        bool found = false;
        CommandPodCrew crew = CommandPodCrew.NoCommandPod;
        ProbeCorePower probe = ProbeCorePower.NoProbeCore;
        int? stormLevel = null;
        foreach (Control control in controls)
        {
            Part part = control.Parent;
            if (!selection.Contains(part))
                continue;
            found = true;

            HabitatZone? zone = HabitatZone.FindZone(tree, part);
            if (zone != null && HasSelectedHabitat(zone, part, selection))
            {
                CommandPodCrew localCrew = FindCrew(zone, selection, now);
                if (localCrew == CommandPodCrew.Able)
                    return Vehicle.ComputeControl(localCrew, probe);
                if (localCrew > crew)
                    crew = localCrew;
                continue;
            }

            ProbeCorePower localProbe = FindProbePower(vehicle, part, selection, now, ref stormLevel);
            if (localProbe == ProbeCorePower.Powered)
                return Vehicle.ComputeControl(crew, localProbe);
            if (localProbe > probe)
                probe = localProbe;
        }

        return found ? Vehicle.ComputeControl(crew, probe) : null;
    }

    private static bool HasSelectedHabitat(HabitatZone zone, Part part, Selection selection)
    {
        if (selection.Parts == null)
            return true;
        Part structural = selection.StructuralPart(part);
        foreach (Habitat habitat in zone.Habitats)
            if (selection.Contains(habitat.FullPart) && selection.StructuralPart(habitat.FullPart) == structural)
                return true;
        return false;
    }

    private static CommandPodCrew FindCrew(HabitatZone zone, Selection selection, UniverseTime now)
    {
        CommandPodCrew result = CommandPodCrew.Empty;
        foreach (IVASeat seat in zone.Seats)
        {
            if (seat.AssignedKittenHash == KeyHash.Zero || !selection.Contains(seat.Parent)
                || !HasSelectedHabitat(zone, seat.Parent, selection))
                continue;
            KittenRosterEntryData? kitten = Universe.KittenRoster.Find(seat.AssignedKittenHash);
            if (kitten == null || kitten.Kia)
                continue;
            if (!Universe.Rules.LifeSupport || !kitten.Condition.IsIncapacitated(now))
                return CommandPodCrew.Able;
            result = CommandPodCrew.Incapacitated;
        }
        return result;
    }

    private static ProbeCorePower FindProbePower(Vehicle vehicle, Part part, Selection selection,
        UniverseTime now, ref int? stormLevel)
    {
        PowerConsumer? consumer = null;
        foreach (PowerConsumer candidate in part.Modules.Get<PowerConsumer>())
        {
            if (!candidate.PowerSwitch)
                continue;
            consumer = candidate;
            break;
        }

        bool rebooting = false;
        if (Universe.Rules.Radiation)
        {
            Span<Avionics> avionics = part.Modules.Get<Avionics>();
            if (!avionics.IsEmpty && avionics[0].ComputeHealth() == AvionicsHealth.Failed)
                return ProbeCorePower.Failed;
            if (consumer is { StandbyInStorms: true, Mode: not PowerSwitchMode.Off })
            {
                stormLevel ??= SpaceWeatherAlerts.GetSLevel(
                    SpaceWeatherAlerts.ComputeStormFlux(vehicle.GetPositionEcl(), now.Seconds()));
                if (stormLevel > 0)
                    return ProbeCorePower.StormStandby;
            }
            rebooting = !avionics.IsEmpty && avionics[0].IsRebooting(now.Seconds());
        }
        if (consumer == null)
            return rebooting ? ProbeCorePower.Rebooting : ProbeCorePower.Powered;
        if (consumer.Mode == PowerSwitchMode.Off)
            return ProbeCorePower.SwitchedOff;

        // An Auto probe can wake when the split leaves it alone. Its inherited Active state
        // cannot prove that a battery remains on the same side of the joint.
        bool powered = selection.Parts == null
            ? consumer.HasPower(vehicle.Parts)
            : HasSelectedPower(vehicle.Parts, consumer, selection);
        if (!powered)
            return ProbeCorePower.Unpowered;
        return rebooting ? ProbeCorePower.Rebooting : ProbeCorePower.Powered;
    }

    private static bool HasSelectedPower(PartTree tree, PowerConsumer consumer, Selection selection)
    {
        ElectricalCircuits circuits = tree.ElectricalCircuits;
        int circuit = circuits.IndexFor(consumer.Parent);
        bool hasBattery = false;
        foreach (Battery battery in circuits.Batteries(circuit))
        {
            if (!selection.Contains(battery.Parent))
                continue;
            hasBattery = true;
            if (tree.Batteries.States[battery.StatesIdx].Charge > Joules.Zero)
                return true;
        }
        if (!hasBattery || !consumer.IsSwitchedOn())
            return false;

        // KSA runs generation before consumers. A supplied consumer can leave the batteries empty.
        // Count live generation from this side and debit earlier loads in stock update order.
        Watts available = Watts.Zero;
        if (Generator.TryGetFrom(tree.States, out var generators)
            && EngineController.TryGetFrom(tree.States, out var engines))
            foreach (var item in generators.ModulesAndStates)
                if (InSelectedCircuit(item.Module.Parent)
                    && engines.TryGetModuleAndStateById(item.Module.EngineInstanceId, out var engine)
                    && engine.Module.IsActive && engine.State.CommandBurnTime > 0.0
                    && InSelectedCircuit(engine.Module.Parent))
                    available += item.Module.Produced * engine.State.CommandThrottle;
        if (SolarPanel.TryGetFrom(tree.States, out var panels))
            foreach (var item in panels.ModulesAndStates)
                if (item.State.Active && !item.State.IsOccluded && InSelectedCircuit(item.Module.Parent))
                    available += item.State.Produced;
        if (!(available > Watts.Zero))
            return false;

        foreach (var item in tree.PowerConsumers.ModulesAndStates)
        {
            if (ReferenceEquals(item.Module, consumer))
                return available > Watts.Zero && available >= consumer.Consumed;
            if (item.State.Active && InSelectedCircuit(item.Module.Parent))
                available -= item.State.Consumed;
        }
        return false;

        bool InSelectedCircuit(Part part) => selection.Contains(part) && circuits.IndexFor(part) == circuit;
    }
}
