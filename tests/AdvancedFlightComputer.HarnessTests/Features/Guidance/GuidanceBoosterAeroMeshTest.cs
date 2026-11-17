using System.Numerics;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;
using KSA.Aero;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceBoosterAeroMeshTest : AfcTest
{
    public override string Name => "afc-guidance-booster-aero-mesh";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        PartInstance? design = DefaultVehicleSaves.FindSave("Rocket")?.VehicleSaveData.RootPartInstance;
        AeroAtmosphere sea = AeroProbe.Atmosphere(home, 0.0);
        if (!t.Check("the stock Rocket and a sea-level atmosphere are available", design != null && sea.HasAtmosphere))
            return;

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool forceOffRails = PhysicsBubble._forceOffRails;
        PartTree? holder = null;
        try
        {
            PhysicsBubble._forceOffRails = true;
            Vehicle stack = VehicleFixtures.SpawnDesign(t.System, home, design!,
                "AfcBoosterAeroMesh", OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            Part stackRoot = stack.Parts.Root;
            doubleQuat rotation = doubleQuat.CreateFromAxisAngle(double3.Normalize(new double3(1, 2, 3)), 0.61);
            stack.Parts.SetRootPose(stackRoot.PositionParentAsmb, rotation);
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(0.02, 2);

            var (decoupler, root, parts) = FindSplit(stack);
            if (decoupler == null || root == null || parts == null)
            {
                t.Fail("a proper split subset", "the stock Rocket has no multi-part decoupler branch with subpart colliders");
                return;
            }

            AeroMeshSource? liveSource = stack.Parts.AwaitAerodynamics()?.Source;
            doubleQuat rootToStack = root.Asmb2ParentAsmb;
            if (!t.Check("the attached stack has a mesh and the split root is tilted",
                    liveSource != null && (double3.UnitX.Transform(rootToStack) - double3.UnitX).Length() > 0.1))
                return;

            if (!t.Check("the booster mesh builds", KsaBoosterAeroMesh.TryBuild(root, parts,
                    out holder, out ulong hash, out string error), error))
                return;
            PartTree privateTree = holder!;
            Aerodynamics product = privateTree.AwaitAerodynamics()!;
            AeroMeshSource source = product.Source;
            t.Check("the holder contains only the private aero product",
                privateTree.Parts.IsEmpty && privateTree.OwningVehicle == null && !ReferenceEquals(source, liveSource));
            t.Check("the source has only the selected parts and fewer colliders than the stack",
                source.PartNames.Length == parts.Count && source.Shapes.Count < liveSource!.Shapes.Count);
            t.Check("the result reports its own geometry key", hash != 0 && source.Hash == hash);

            bool builtAgain = KsaBoosterAeroMesh.TryBuild(root, parts, out PartTree? repeat, out ulong repeatHash, out error);
            t.Check("the same selection has a stable key and independent mesh",
                builtAgain && repeatHash == hash && !ReferenceEquals(repeat!.Aerodynamics, product), error);
            repeat?.Dispose();

            bool builtTrimmed = KsaBoosterAeroMesh.TryBuild(root, new HashSet<Part> { root },
                out PartTree? trimmed, out ulong trimmedHash, out error);
            t.Check("excluding descendants changes the key and removes their colliders",
                builtTrimmed && trimmedHash != hash && trimmed!.Aerodynamics!.Source.Shapes.Count < source.Shapes.Count, error);
            trimmed?.Dispose();

            bool builtEmpty = KsaBoosterAeroMesh.TryBuild(root, new HashSet<Part>(),
                out PartTree? rejected, out ulong rejectedHash, out error);
            t.Check("a rejected selection returns no holder or key",
                !builtEmpty && rejected == null && rejectedHash == 0 && error.Length > 0);
            rejected?.Dispose();

            t.Check("private builds leave the live mesh and membership in place",
                ReferenceEquals(stack.Parts.Aerodynamics?.Source, liveSource)
                && root.Tree == stack.Parts && decoupler.Connector.Connection != null);
            double3 offAxis = new double3(-0.55, 0.73, 0.4).NormalizeOrZero() * (2.1 * sea.SpeedOfSound);
            if (!t.Check("the attached booster sweep succeeds", KsaAeroSweep.TryBuildBooster(stack, root, parts,
                home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result predicted, out string sweepError), sweepError))
                return;
            t.Check("the sweep uses the selected mesh key", predicted.MeshHash == hash);

            decoupler.SetIsEnabled(true);
            decoupler.SetIsActive(stack, true);
            driver.Step(0.02, 2);
            Vehicle? detached = root.Tree?.OwningVehicle;
            if (detached == null || detached == stack || detached.Parts.Root != root)
            {
                t.Fail("the game detached the predicted subtree", "the selected decoupler did not produce the expected vehicle");
                return;
            }

            Aerodynamics? detachedAero = detached.Parts.AwaitAerodynamics();
            if (detachedAero == null || detachedAero.Mesh.Surface.FaceCount == 0)
            {
                t.Fail("the detached craft has a fresh stock mesh", "the detached vehicle has no panel loads");
                return;
            }

            t.Info($"panel widths {product.Mesh.PanelWidth:G5} / {detachedAero.Mesh.PanelWidth:G5} m, " +
                $"faces {product.Mesh.Surface.FaceCount} / {detachedAero.Mesh.Surface.FaceCount}");
            t.Check("the subset source matches the detached collider poses", SameColliders(source, detachedAero.Source));
            double3 subsetForce = CheckForce(t, "supersonic off-axis", detached, privateTree, sea, offAxis);
            CheckForce(t, "subsonic nose-first", detached, privateTree, sea, double3.UnitX * (0.3 * sea.SpeedOfSound));
            CheckForce(t, "hypersonic broadside", detached, privateTree, sea, double3.UnitY * (5.0 * sea.SpeedOfSound));
            if (t.Check("the detached whole-craft sweep succeeds", KsaAeroSweep.TryBuild(detached, home,
                Universe.GetElapsedSeconds(), out KsaAeroSweep.Result measured, out sweepError), sweepError))
                CheckSweepParity(t, predicted, measured);

            privateTree.Dispose();
            holder = null;
            t.Check("disposing the holder clears its aero product",
                privateTree.IsDisposed && privateTree.Aerodynamics == null);
            double3 detachedForce = ForceBody(detached, detached.Parts, sea, offAxis);
            t.Check("the detached stock mesh still produces force after holder disposal",
                ReferenceEquals(detached.Parts.Aerodynamics, detachedAero)
                && (detachedForce - subsetForce).Length() <= 0.02 * subsetForce.Length());
        }
        finally
        {
            holder?.Dispose();
            PhysicsBubble._forceOffRails = forceOffRails;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void CheckSweepParity(TestContext t, KsaAeroSweep.Result predicted, KsaAeroSweep.Result measured)
    {
        if (!t.Check("attached and detached sweep grids match",
            predicted.MachGrid.SequenceEqual(measured.MachGrid) && predicted.AlphaGridDeg.SequenceEqual(measured.AlphaGridDeg)))
            return;
        double dragError = 0.0, liftError = 0.0;
        for (int i = 0; i < predicted.CdTable.Length; i++)
        {
            double predictedDrag = predicted.CdTable[i] * predicted.ReferenceArea;
            double measuredDrag = measured.CdTable[i] * measured.ReferenceArea;
            double predictedLift = predicted.LiftTable[i] * predicted.ReferenceArea;
            double measuredLift = measured.LiftTable[i] * measured.ReferenceArea;
            dragError = Math.Max(dragError, Math.Abs(predictedDrag - measuredDrag) / Math.Max(1.0, Math.Abs(measuredDrag)));
            liftError = Math.Max(liftError, Math.Abs(predictedLift - measuredLift) / Math.Max(1.0, Math.Abs(measuredLift)));
        }
        t.Check("the complete booster drag-area table matches after separation", dragError <= 0.03,
            FormattableString.Invariant($"maximum scaled error {dragError:G6}"));
        t.Check("the complete signed lift-area table matches after separation", liftError <= 0.03,
            FormattableString.Invariant($"maximum scaled error {liftError:G6}"));
    }

    private static (Decoupler?, Part?, HashSet<Part>?) FindSplit(Vehicle stack)
    {
        foreach (Decoupler decoupler in stack.Parts.Modules.Get<Decoupler>())
        {
            Part near = decoupler.Connector.ConnectionPart;
            Part? far = decoupler.Connector.Connection?.OtherPart(near);
            if (far == null)
                continue;
            Part? root = far.TreeParent == near ? far : near.TreeParent == far ? near : null;
            if (root == null || root.IsSubPart || root.SubtreeModules.Get<ColliderModule>().IsEmpty)
                continue;

            var parts = new HashSet<Part> { root };
            var descendants = new PartTreeChildrenIterator(root);
            for (Part? child = descendants.GetNextNode(); child != null; child = descendants.GetNextNode())
                parts.Add(child);
            if (parts.Count > 1 && parts.Any(HasSubpartCollider))
                return (decoupler, root, parts);
        }
        return (null, null, null);
    }

    private static bool HasSubpartCollider(Part part)
    {
        foreach (ColliderModule collider in part.SubtreeModules.Get<ColliderModule>())
            if (collider.Parent.IsSubPart)
                return true;
        return false;
    }

    private static bool SameColliders(AeroMeshSource subset, AeroMeshSource stock)
    {
        if (!subset.PartNames.SequenceEqual(stock.PartNames) || subset.Shapes.Count != stock.Shapes.Count)
            return false;
        for (int i = 0; i < subset.Shapes.Count; i++)
        {
            AeroShape lhs = subset.Shapes.Shapes[i];
            AeroShape rhs = stock.Shapes.Shapes[i];
            if (lhs.Kind != rhs.Kind || lhs.Parameters != rhs.Parameters || lhs.HullIndex != rhs.HullIndex
                || subset.Shapes.SourcePartIndices[i] != stock.Shapes.SourcePartIndices[i]
                || Vector3.Distance(lhs.Position, rhs.Position) > 0.0001f
                || Math.Abs(Quaternion.Dot(lhs.Orientation, rhs.Orientation)) < 0.99999f)
                return false;
        }
        return true;
    }

    private static double3 CheckForce(TestContext t, string label, Vehicle detached, PartTree holder,
        in AeroAtmosphere sea, double3 airVelocity)
    {
        double3 stock = ForceBody(detached, detached.Parts, sea, airVelocity);
        double3 subset = ForceBody(detached, holder, sea, airVelocity);
        double miss = (stock - subset).Length();
        t.Check($"{label} force agrees with the detached craft",
            stock.Length() > 1.0 && miss <= 0.02 * stock.Length(),
            FormattableString.Invariant($"difference {miss:G5} N on {stock.Length():G5} N"));
        return subset;
    }

    private static double3 ForceBody(Vehicle vehicle, PartTree tree, in AeroAtmosphere sea, double3 airVelocity)
    {
        BubbleOrigin origin = vehicle.BubbleOrigin;
        KinematicStates kinematic = vehicle.KinematicStates;
        VehicleProperties props = vehicle.Props;
        PhysicsEnvironment environment = vehicle.PhysicsEnvironment;
        environment.InPhysicsRadius = true;
        environment.AtmosphericDensity = (float)sea.Density;
        environment.AtmosphericPressure = (float)sea.Pressure;
        environment.AtmosphericTemperature = (float)sea.Temperature;
        environment.SpeedOfSound = (float)sea.SpeedOfSound;
        environment.OceanVolume = 0;
        environment.OceanSurfaceArea = 0;
        environment.GravitationBub = double3.Zero;
        kinematic.AngularVelocityPhys = origin.BubFrame == BubbleFrame.Ccf
            ? -environment.ClosestParent.GetAngularVelocityCci() : double3.Zero;
        kinematic.VelocityPhys = double3.Zero;
        float3 frameAir = PhysicsStates.ComputeAirVelocityBody(in origin, in kinematic, in environment);
        kinematic.VelocityPhys = (airVelocity - double3.Unpack(in frameAir)).Transform(kinematic.Body2Phys);
        return PhysicsStates.ComputeDerivatives(in origin, in kinematic, in props, in environment,
            0.0, 0.0, origin.PositionBub, origin.VelocityBub, default, default, tree).ForceBody;
    }
}
