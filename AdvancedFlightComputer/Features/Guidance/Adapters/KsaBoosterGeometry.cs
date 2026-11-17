#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using BepuPhysics.Collidables;
using BoundingBox = BepuUtilities.BoundingBox;
using Brutal.Numerics;
using KSA;

/// <summary>
/// A booster still attached to its stack, as the vehicle it becomes when it separates: the properties the game's air forces read, worked out the way the game works them out for a vehicle of its own.
///
/// SPLIT, NOT SCALED. A booster's return was priced with the whole stack's aero: on a heavy, three cores wide against one core's mass, which put the core's terminal velocity at about 50 m/s where it fell at 98 and left it short of its landing burn (2026-11-17). The game measures a vehicle's air forces from its own geometry, so the booster's are measured from its own:
///   - THE BOX. Vehicle.Split re-roots the separated parts at their root part, posed with no rotation, so the new vehicle's assembly axes are the root part's. Vehicle.UpdateCollisionGeometry bounds the vehicle's collider compound in those axes; here each of the booster's own colliders is bounded at its live pose along the same axes (as KsaVehicleHeight bounds them), and the union taken.
///   - THE AREAS. VehicleProperties.RecomputeAerodynamicProperties derives the face drag areas, surface area and volume from that box, as it does for any vehicle.
///   - THE FORCE. The game's own force function (PhysicsStates.ComputeDerivatives) is then asked with these properties (KsaAeroSweep.TryBuildBooster), so whatever aero model is in - stock's box, KSAero's body of revolution - measures the booster as it will measure it once separated.
///
/// A model that reads more than the vehicle's properties, per-part panels say, needs the booster's parts handed to it in the same way: the parts are here (<see cref="Subtree"/>), the rest follows that model.
/// </summary>
internal static class KsaBoosterGeometry
{
    /// <summary>The parts that leave with <paramref name="root"/>: its subtree, each tree part with its subparts.</summary>
    internal static HashSet<Part> Subtree(Part root)
    {
        var parts = new HashSet<Part>();
        var stack = new Stack<Part>();
        stack.Push(root);
        while (stack.Count > 0)
        {
            Part p = stack.Pop();
            if (!parts.Add(p))
                continue;
            foreach (Part child in p.TreeChildren)
                stack.Push(child);
        }
        return parts;
    }

    /// <summary>
    /// The booster rooted at <paramref name="root"/>'s vehicle properties as the separated vehicle's, and its box along its own assembly axes (x the long one), m. Copied from <paramref name="vehicle"/>'s, with the box, its centre and the face areas the booster's own; the rest only matters to torques and the integrator's limiter, which a force sweep does not read.
    /// </summary>
    internal static bool TryProperties(Vehicle vehicle, Part root, out VehicleProperties props, out double3 extents, out string error)
        => TryProperties(vehicle, root, Subtree(root), out props, out extents, out error);

    /// <summary>
    /// The same for the parts in <paramref name="parts"/>, rooted at <paramref name="root"/>: the booster's subtree short of whatever leaves before it or beside it (GuidanceWindow.PartsLeavingWith).
    ///
    /// NOT THE WHOLE SUBTREE. Side boosters hang off the core in the part tree, so the core's subtree on the pad holds them too: measured whole, a heavy's core came out three cores wide again, the stack's own 100 m^2 (2026-11-17).
    /// </summary>
    internal static bool TryProperties(Vehicle vehicle, Part root, HashSet<Part> parts, out VehicleProperties props, out double3 extents, out string error)
    {
        props = default;
        extents = default;
        error = "";
        if (vehicle?.Parts == null || root == null)
        {
            error = "no booster to measure";
            return false;
        }
        if (!TryBox(vehicle, root, parts, out double3 min, out double3 max, out int colliders))
        {
            error = colliders == 0 ? "the booster has no colliders to bound" : "the booster's bounds are not finite";
            return false;
        }

        extents = max - min;
        props = vehicle.Props;
        props.BoundingBoxAsmb = new Box((float)extents.X, (float)extents.Y, (float)extents.Z);
        props.GeometricCenterAsmb = props.CenterOfMassAsmb;
        props.RecomputeAerodynamicProperties();
        return true;
    }

    /// <summary>The union of the booster's collider bounds along its root part's assembly axes, each collider at its live pose: the box Vehicle.UpdateCollisionGeometry would take for it once it is a vehicle of its own.</summary>
    private static bool TryBox(Vehicle vehicle, Part root, HashSet<Part> parts, out double3 min, out double3 max, out int colliders)
    {
        min = new double3(double.PositiveInfinity);
        max = new double3(double.NegativeInfinity);
        colliders = 0;

        // The separated vehicle's assembly axes, in the stack's: the root part's own.
        doubleQuat rootRot = root.Asmb2VehicleAsmb;
        double3 mx = new double3(1, 0, 0).Transform(rootRot);
        double3 my = new double3(0, 1, 0).Transform(rootRot);
        double3 mz = new double3(0, 0, 1).Transform(rootRot);

        Shapes shapes = ConstraintSim.ReadOnlyShapes;
        foreach (ColliderModule c in vehicle.Parts.Modules.Get<ColliderModule>())
        {
            if (c?.Parent == null || !c.Handle.Exists || !parts.Contains(c.Parent.FullPart))
                continue;

            // The collider's axes in the booster's frame, by transforming basis vectors with KSA's own Transform (the KsaFrameBridge rule), and Bepu bounds the shape exactly at that orientation, as KsaVehicleHeight does.
            doubleQuat rot = c.Collider2VehicleAsmb;
            double3 ax = new double3(1, 0, 0).Transform(rot);
            double3 ay = new double3(0, 1, 0).Transform(rot);
            double3 az = new double3(0, 0, 1).Transform(rot);
            // System.Numerics is row-vector: row i is the image of basis vector i.
            var basis = new System.Numerics.Matrix4x4(
                (float)double3.Dot(ax, mx), (float)double3.Dot(ax, my), (float)double3.Dot(ax, mz), 0f,
                (float)double3.Dot(ay, mx), (float)double3.Dot(ay, my), (float)double3.Dot(ay, mz), 0f,
                (float)double3.Dot(az, mx), (float)double3.Dot(az, my), (float)double3.Dot(az, mz), 0f,
                0f, 0f, 0f, 1f);
            System.Numerics.Quaternion q = System.Numerics.Quaternion.Normalize(System.Numerics.Quaternion.CreateFromRotationMatrix(basis));
            shapes.UpdateBounds(System.Numerics.Vector3.Zero, q, c.Handle, out BoundingBox bounds);

            double3 pos = c.PositionVehicleAsmb;
            var at = new double3(double3.Dot(pos, mx), double3.Dot(pos, my), double3.Dot(pos, mz));
            var lo = new double3(at.X + bounds.Min.X, at.Y + bounds.Min.Y, at.Z + bounds.Min.Z);
            var hi = new double3(at.X + bounds.Max.X, at.Y + bounds.Max.Y, at.Z + bounds.Max.Z);
            if (!double.IsFinite(lo.X + lo.Y + lo.Z + hi.X + hi.Y + hi.Z))
                continue;
            min = new double3(Math.Min(min.X, lo.X), Math.Min(min.Y, lo.Y), Math.Min(min.Z, lo.Z));
            max = new double3(Math.Max(max.X, hi.X), Math.Max(max.Y, hi.Y), Math.Max(max.Z, hi.Z));
            colliders++;
        }
        double3 size = max - min;
        return colliders > 0 && size.X > 0.0 && size.Y > 0.0 && size.Z > 0.0 && double.IsFinite(size.X + size.Y + size.Z);
    }
}
