using System.Collections.Generic;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// Keep selections by id because LookupCollection.Deregister can move indices between frames.
internal static class TargetSelection
{
    public static void BuildList(Vehicle source, List<TransferObject> list)
    {
        list.Clear();
        IParentBody? parent = source.Parent;
        if (parent == null || Universe.CurrentSystem == null)
            return;

        // Match the parent filter in TransferPlanner.PopulateWithVehiclesAsTargets.
        foreach (Vehicle v in Program.VehiclesInFrame)
        {
            if (v == source) continue;
            if (v.Parent?.Id == parent.Id)
                list.Add(new TransferObject(v));
        }

        foreach (Astronomical astro in Universe.CurrentSystem.All.AsSpan())
        {
            if (astro is not Celestial celestial) continue;
            if (celestial.Orbit == null) continue;
            if (celestial.Parent?.Id == parent.Id)
                list.Add(new TransferObject(astro));
        }
    }

    // Reject targets in a different CCI frame because Orbit.GetRelativeInclination does not check their parents.
    public static IOrbiter? Resolve(string? targetId, string? parentId)
    {
        if (targetId == null) return null;
        CelestialSystem? system = Universe.CurrentSystem;
        if (system == null) return null;
        if (!system.All.TryGet(targetId, out Astronomical? body)) return null;
        if (body is not IOrbiter orbiter) return null;
        return orbiter.Parent?.Id == parentId ? orbiter : null;
    }

    // An empty list clears the selection so readouts and Create cannot use an unavailable target.
    public static TransferObject? Reconcile(List<TransferObject> list, ref string? targetId)
    {
        if (list.Count == 0)
        {
            targetId = null;
            return null;
        }

        TransferObject picked = list[0];
        if (targetId != null)
        {
            for (int i = 0; i < list.Count; i++)
            {
                if (list[i].GetKey() == targetId)
                {
                    picked = list[i];
                    break;
                }
            }
        }
        targetId = picked.GetKey();
        return picked;
    }
}
