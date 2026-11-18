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
            : HasSelectedCharge(vehicle.Parts, part, selection);
        if (!powered)
            return ProbeCorePower.Unpowered;
        return rebooting ? ProbeCorePower.Rebooting : ProbeCorePower.Powered;
    }

    private static bool HasSelectedCharge(PartTree tree, Part part, Selection selection)
    {
        ElectricalCircuits circuits = tree.ElectricalCircuits;
        foreach (Battery battery in circuits.Batteries(circuits.IndexFor(part)))
            if (selection.Contains(battery.Parent) && tree.Batteries.States[battery.StatesIdx].Charge > Joules.Zero)
                return true;
        return false;
    }
}
