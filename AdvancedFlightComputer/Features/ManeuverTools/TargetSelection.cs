using System.Collections.Generic;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Which bodies a maneuver can target from a given source, and how a selection
/// survives from one frame to the next.
///
/// Kept out of the ImGui code because the identity rule is the load-bearing part.
/// A <see cref="TransferObject"/> stores only a LookupIndex, and
/// <c>LookupCollection.Deregister</c> swap-removes, so an index held across frames
/// resolves to a different body once any vehicle is deregistered, or makes
/// <c>CelestialSystem.GetIndex</c> throw once it is past the end of the
/// collection. A selection is therefore held as an id and re-resolved through the
/// lookup's hash map, and the per-frame list is rebuilt rather than cached.
/// </summary>
internal static class TargetSelection
{
    /// <summary>Bodies orbiting the same parent as <paramref name="source"/>.</summary>
    public static void BuildList(Vehicle source, List<TransferObject> list)
    {
        list.Clear();
        IParentBody? parent = source.Parent;
        if (parent == null || Universe.CurrentSystem == null)
            return;

        // Vehicles: the same parent filter stock's PopulateWithVehiclesAsTargets
        // applies, over the same set (Program.RefreshVehiclesInFrame fills
        // VehiclesInFrame from every Astronomical of type Vehicle in the current
        // system, with no further filtering).
        foreach (Vehicle v in Program.VehiclesInFrame)
        {
            if (v == source) continue;
            if (v.Parent?.Id == parent.Id)
                list.Add(new TransferObject(v));
        }

        // Celestials: full registry, since they don't move in/out of frame.
        foreach (Astronomical astro in Universe.CurrentSystem.All.AsSpan())
        {
            if (astro is not Celestial celestial) continue;
            if (celestial.Orbit == null) continue;
            if (celestial.Parent?.Id == parent.Id)
                list.Add(new TransferObject(astro));
        }
    }

    /// <summary>Resolves a stored target id, or null when it no longer names a body
    /// orbiting <paramref name="parentId"/>.
    ///
    /// The parent check is part of the contract, not a nicety:
    /// <see cref="Orbit.GetRelativeInclination"/> compares orbit normals without a
    /// parent check of its own, so a target that has left the source's SOI - which
    /// drops it from <see cref="BuildList"/> while leaving its LookupIndex
    /// perfectly valid - would otherwise yield a relative inclination, AN/DN and
    /// burn computed across two different CCI frames.</summary>
    public static IOrbiter? Resolve(string? targetId, string? parentId)
    {
        if (targetId == null) return null;
        CelestialSystem? system = Universe.CurrentSystem;
        if (system == null) return null;
        if (!system.All.TryGet(targetId, out Astronomical? body)) return null;
        if (body is not IOrbiter orbiter) return null;
        return orbiter.Parent?.Id == parentId ? orbiter : null;
    }

    /// <summary>Reconciles a stored selection against a freshly built list.
    /// Returns the entry matching <paramref name="targetId"/>, the first entry when
    /// the id no longer resolves (destroyed, renamed, or gone from the SOI), or
    /// null when the list is empty. Writes the outcome back to
    /// <paramref name="targetId"/>, so a fallback becomes the selection and an
    /// empty list drops it.
    ///
    /// Dropping it matters: a selection kept past an empty list leaves the
    /// relative-inclination readout, the AN/DN times and an enabled Create button
    /// live for a body the UI has just said is unavailable. Stock avoids the same
    /// trap by assigning <c>new TransferObject(-1)</c> when its own target list
    /// comes back empty, which makes its Destination resolve to null.</summary>
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
