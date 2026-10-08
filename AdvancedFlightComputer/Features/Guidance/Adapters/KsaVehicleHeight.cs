#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using BepuPhysics.Collidables;
using BoundingBox = BepuUtilities.BoundingBox;
using Brutal.Numerics;
using KSA;

/// <summary>
/// Measures how far the vehicle's lowest touchdown point sits below its centre of mass, along the thrust axis. That is the height the centre of mass has above the ground when the craft stands upright on its legs, which is what every landing mode's "vehicle height" means.
///  THIS IS A MEASUREMENT OF THE COLLIDERS, not of the meshes: the colliders are what the physics step lands on (the terrain-contact flag fires on a Bepu contact), so they are what "on the ground" means. Each collider's own Bepu shape is bounded at its live pose, so a sphere foot, a capsule strut and a box base each give their exact lowest point.
///  LANDING LEGS ARE MEASURED DEPLOYED, whatever they are doing now. KSA only ever poses a leg's subparts (and so its colliders) at the animation's CURRENT time, so a stowed leg reports its stowed geometry, and a height measured from it would be the engine bell's. Every deploy/retract animation is re-posed here at its deployed end - the same keyframe evaluation KeyframeAnimationModule.ApplyAnimationTransforms uses, without writing anything back to the part. Animations that only rotate sideways (solar panels and the like) move nothing downward, so they cannot change the result.
///  The centre of mass is read live: it climbs the vehicle as the propellant drains, and the height with it.
/// </summary>
public static class KsaVehicleHeight
{
    public readonly struct Result(double deployedM, double currentM, int colliders, int deployables, int stowed)
    {
        /// <summary>CoM to lowest point with every deployable at its deployed end, m. The touchdown height.</summary>
        public readonly double DeployedM = deployedM;

        /// <summary>CoM to lowest point with every part where it is right now, m.</summary>
        public readonly double CurrentM = currentM;

        public readonly int Colliders = colliders;

        /// <summary>Deploy/retract animations found (legs among them).</summary>
        public readonly int Deployables = deployables;

        /// <summary>Of those, how many are not at their deployed end and not on their way there.</summary>
        public readonly int Stowed = stowed;
    }

    private readonly struct Pose(double3 position, doubleQuat rotation, double4x4 matrix)
    {
        public readonly double3 Position = position;
        public readonly doubleQuat Rotation = rotation;
        public readonly double4x4 Matrix = matrix;
    }

    private readonly struct LocalPose(double3 position, doubleQuat rotation, double3 scale)
    {
        public readonly double3 Position = position;
        public readonly doubleQuat Rotation = rotation;
        public readonly double3 Scale = scale;
    }

    /// <summary>False when the vehicle has no collider to measure, or the result is not finite.</summary>
    public static bool TryMeasure(Vehicle vehicle, out Result result)
    {
        result = default;
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return false;

        Span<ColliderModule> colliders = tree.Modules.Get<ColliderModule>();
        if (colliders.Length == 0)
            return false;

        // Model +Z: the thrust axis, pointing up the vehicle. The same axis the 6-DOF plan's terminal attitude stands upright on, so "height" here is height in the attitude it lands in.
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        float3 comF = vehicle.CenterOfMassAsmbF;
        double comUp = double3.Dot(new double3(comF.X, comF.Y, comF.Z), mz);

        var deployed = new Dictionary<Part, LocalPose>();
        int deployables = 0, stowed = 0;
        foreach (KeyframeAnimationModule anim in tree.Modules.Get<KeyframeAnimationModule>())
        {
            // ShowDeployRetract is KSA's own flag for a two-state deploy/retract part. A slider-driven actuator has no "deployed" end to assume, and a sun-tracking extension adds a rotation of its own - both stay where they are.
            if (!anim.ShowDeployRetract || anim.Extension != null || anim.Shared == null || anim.Shared.Duration <= 0f)
                continue;
            deployables++;
            if (anim.TimeGoal < anim.Shared.Duration)
                stowed++;

            ReadOnlySpan<Part> subParts = anim.Parent.SubParts;
            for (int i = 0; i < subParts.Length; i++)
            {
                Part sub = subParts[i];
                if (!anim.Shared.PartLookup.TryGetValue(sub.Id, out KeyframeAnimationData.AnimatedPart animated))
                    continue;
                float4x4 m = anim.Shared.EvaluateWorldMatrix(animated, anim.Shared.Duration);
                float4x4.Decompose(m, out float3 scale, out floatQuat rotation, out float3 translation);
                deployed[sub] = new LocalPose(double3.Unpack(in translation), doubleQuat.Unpack(in rotation), double3.Unpack(in scale));
            }
        }

        var poses = new Dictionary<Part, Pose>();
        Shapes shapes = ConstraintSim.ReadOnlyShapes;
        double lowestNow = double.PositiveInfinity, lowestDeployed = double.PositiveInfinity;
        int measured = 0;
        foreach (ColliderModule c in colliders)
        {
            if (c?.Parent == null || !c.Handle.Exists)
                continue;

            // Current pose: exactly what the game hands Bepu (Vehicle.UpdateColliderCompoundLeaves).
            double lowNow = ColliderLowest(shapes, c, c.PositionVehicleAsmb, c.Collider2VehicleAsmb, mx, my, mz);

            // Deployed pose: the same composition, through the re-posed subparts.
            Pose p = PoseOf(c.Parent, deployed, poses);
            double3 pos = c.PositionPartAsmb.Transform(p.Rotation) + p.Position;
            doubleQuat rot = c.Collider2PartAsmb.Concatenate(p.Rotation);
            double lowDep = ColliderLowest(shapes, c, pos, rot, mx, my, mz);

            if (!double.IsFinite(lowNow) || !double.IsFinite(lowDep))
                continue;
            lowestNow = Math.Min(lowestNow, lowNow);
            lowestDeployed = Math.Min(lowestDeployed, lowDep);
            measured++;
        }

        if (measured == 0)
            return false;
        result = new Result(comUp - lowestDeployed, comUp - lowestNow, measured, deployables, stowed);
        return double.IsFinite(result.DeployedM) && result.DeployedM > 0.0;
    }

    // Lowest point of one collider along +mz, in the vehicle assembly frame's units. The collider's axes are carried into the model frame by transforming basis vectors with KSA's own Transform - the KsaFrameBridge rule, so no quaternion convention is assumed - and Bepu then bounds the shape exactly at that orientation; the Z of its box is the support along the thrust axis.
    private static double ColliderLowest(Shapes shapes, ColliderModule c, double3 positionAsmb, doubleQuat collider2Asmb,
                                         double3 mx, double3 my, double3 mz)
    {
        double3 ax = new double3(1, 0, 0).Transform(collider2Asmb);
        double3 ay = new double3(0, 1, 0).Transform(collider2Asmb);
        double3 az = new double3(0, 0, 1).Transform(collider2Asmb);
        // System.Numerics is row-vector: row i is the image of basis vector i.
        var basis = new System.Numerics.Matrix4x4(
            (float)double3.Dot(ax, mx), (float)double3.Dot(ax, my), (float)double3.Dot(ax, mz), 0f,
            (float)double3.Dot(ay, mx), (float)double3.Dot(ay, my), (float)double3.Dot(ay, mz), 0f,
            (float)double3.Dot(az, mx), (float)double3.Dot(az, my), (float)double3.Dot(az, mz), 0f,
            0f, 0f, 0f, 1f);
        System.Numerics.Quaternion q = System.Numerics.Quaternion.Normalize(System.Numerics.Quaternion.CreateFromRotationMatrix(basis));
        shapes.UpdateBounds(System.Numerics.Vector3.Zero, q, c.Handle, out BoundingBox bounds);
        return double3.Dot(positionAsmb, mz) + bounds.Min.Z;
    }

    // A part's pose in the vehicle assembly frame, with any re-posed animated subpart substituted for its live local transform. Mirrors Part.PositionVehicleAsmb / Asmb2VehicleAsmb / MatrixAsmb2VehicleAsmb, so a part with nothing re-posed above it comes out exactly as the game has it.
    private static Pose PoseOf(Part part, Dictionary<Part, LocalPose> deployed, Dictionary<Part, Pose> cache)
    {
        if (cache.TryGetValue(part, out Pose cached))
            return cached;

        Pose pose;
        if (!part.IsSubPart)
        {
            pose = new Pose(part.PositionVehicleAsmb, part.Asmb2VehicleAsmb, part.MatrixAsmb2VehicleAsmb);
        }
        else
        {
            Pose parent = PoseOf(part.PartParent, deployed, cache);
            double3 position, scale;
            doubleQuat rotation;
            if (deployed.TryGetValue(part, out LocalPose local))
            {
                position = local.Position;
                rotation = local.Rotation;
                scale = local.Scale;
            }
            else
            {
                position = part.PositionParentAsmb;
                rotation = part.Asmb2ParentAsmb;
                scale = part.Scale;
            }
            double4x4 localMatrix = double4x4.CreateScale(scale) * double4x4.CreateFromQuaternion(rotation) * double4x4.CreateTranslation(position);
            pose = new Pose(position.Transform(parent.Matrix), doubleQuat.Concatenate(rotation, parent.Rotation), localMatrix * parent.Matrix);
        }
        cache[part] = pose;
        return pose;
    }
}
