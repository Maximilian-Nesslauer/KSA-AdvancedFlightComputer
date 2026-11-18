namespace AdvancedFlightComputer.Features.Guidance;

using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using KSA.Aero;

internal static class KsaBoosterAeroMesh
{
    /// <summary>
    /// Builds the panel mesh for an ordinary decoupler split. Call between vehicle solver steps
    /// while the selected parts and their poses are stable. The key identifies those inputs
    /// within the current process. The caller must dispose the holder.
    /// </summary>
    internal static bool TryBuild(Part root, IReadOnlySet<Part> parts,
        [NotNullWhen(true)] out PartTree? holder, out ulong geometryHash, out string error)
    {
        holder = null;
        geometryHash = 0;
        error = "";
        if (root.IsSubPart || !parts.Contains(root))
        {
            error = "the booster selection needs a full root part";
            return false;
        }

        PartTree? candidate = null;
        try
        {
            var shapes = new AeroShapeSetBuilder();
            var partNames = new List<string>(parts.Count);
            double3 origin = root.PositionParentAsmb;
            doubleQuat stackToBooster = root.Asmb2ParentAsmb.Inverse();
            ulong key = Mix(Mix(14695981039346656037UL, root.InstanceId), (uint)parts.Count);

            // TransferPartSubtree visits the root first, then its descendants in this order.
            var descendants = new PartTreeChildrenIterator(root);
            for (Part? part = root; part != null; part = descendants.GetNextNode())
            {
                if (part.IsSubPart || !parts.Contains(part))
                    continue;

                int index = partNames.Count;
                partNames.Add(part.Template.Id);
                Span<ColliderModule> colliders = part.SubtreeModules.Get<ColliderModule>();
                key = Mix(Mix(key, part.InstanceId), (uint)colliders.Length);
                foreach (ColliderModule collider in colliders)
                {
                    ColliderTemplate template = collider.ShapeTemplate;
                    float scale = (float)collider.Scale;
                    GetColliderPose(collider, origin, stackToBooster, out Quaternion rotation, out Vector3 position);
                    if (!(scale > 0f) || !float.IsFinite(scale)
                        || !float.IsFinite(rotation.LengthSquared()) || !float.IsFinite(position.LengthSquared()))
                    {
                        error = "the booster has an invalid collider pose or scale";
                        return false;
                    }

                    if (shapes.TryAdd(template, scale, rotation, position, index))
                    {
                        key = Mix(key, unchecked((uint)RuntimeHelpers.GetHashCode(template)));
                        key = MixFloat(key, scale);
                        key = MixFloat(key, rotation.X);
                        key = MixFloat(key, rotation.Y);
                        key = MixFloat(key, rotation.Z);
                        key = MixFloat(key, rotation.W);
                        key = MixFloat(key, position.X);
                        key = MixFloat(key, position.Y);
                        key = MixFloat(key, position.Z);
                    }
                }
            }

            AeroShapeSet shapeSet = shapes.Build();
            if (partNames.Count != parts.Count || shapeSet.Count == 0)
            {
                error = partNames.Count != parts.Count
                    ? "the booster selection contains parts outside its full-part subtree"
                    : "the booster has no supported aero colliders";
                return false;
            }

            // TryAdd uses shared hulls from AeroHulls. The source has no hull pool to clear.
            ulong hash = key == 0 ? 1UL : key;
            var source = new AeroMeshSource(shapeSet, partNames.ToArray(), hash);
            using var builder = new AerodynamicsBuilder();
            candidate = new PartTree();
            builder.Start(source, previous: null, shell: null, pool: null);
            if (!builder.TryTakeFinished(out Aerodynamics? aero))
            {
                error = "the game could not build the booster aero mesh";
                return false;
            }

            // The empty holder owns the finished product and clears its mesh and flow pool on disposal.
            candidate.ExchangeAerodynamics(aero);
            if (aero.Mesh.Surface.FaceCount == 0)
            {
                error = "the booster aero mesh has no faces";
                return false;
            }

            holder = candidate;
            candidate = null;
            geometryHash = hash;
            return true;
        }
        catch (Exception ex)
        {
            error = $"could not build the booster aero mesh ({ex.Message})";
            return false;
        }
        finally
        {
            candidate?.Dispose();
        }
    }

    private static void GetColliderPose(ColliderModule collider, double3 origin, doubleQuat stackToBooster,
        out Quaternion rotation, out Vector3 position)
    {
        double4x4 parent = collider.Parent.ComputeMatrixAsmb2VehicleAsmb();

        // SetRootPose keeps the detached root at the same position and removes its rotation.
        // Gather uses unit rows for the parent's rotation because the collider has its own scale.
        Vector3 x = Vector3.Normalize(new double3(parent.M11, parent.M12, parent.M13).Transform(stackToBooster).ToBepu());
        Vector3 y = Vector3.Normalize(new double3(parent.M21, parent.M22, parent.M23).Transform(stackToBooster).ToBepu());
        Vector3 z = Vector3.Normalize(new double3(parent.M31, parent.M32, parent.M33).Transform(stackToBooster).ToBepu());
        var basis = new Matrix4x4(x.X, x.Y, x.Z, 0f, y.X, y.Y, y.Z, 0f, z.X, z.Y, z.Z, 0f, 0f, 0f, 0f, 1f);
        Quaternion parentRotation = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(basis));
        var parentPosition = new double3(parent.M41, parent.M42, parent.M43);
        rotation = Quaternion.Concatenate(collider.Collider2PartAsmb.ToBepu(), parentRotation);
        position = (origin + (parentPosition - origin).Transform(stackToBooster)).ToBepu()
            + Vector3.Transform(collider.PositionPartAsmb.ToBepu(), parentRotation);
    }

    private static ulong Mix(ulong hash, uint value) => unchecked((hash ^ value) * 1099511628211UL);

    private static ulong MixFloat(ulong hash, float value) => Mix(hash, BitConverter.SingleToUInt32Bits(value));
}
