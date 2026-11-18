using System.Collections;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;
using KSA.Kfs;

namespace AdvancedFlightComputer.HarnessTests;

// Uses real separation, then calls adoption directly for each detached vehicle.
public sealed class GuidanceBoosterHandoverTest : AfcTest
{
    public override string Name => "afc-guidance-booster-handover";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    private const double WindowS = 30.0;
    private const double DecouplerDelayS = 35.0;
    private const int MaxRowsBeforeSeparation = 4;

    private static double _now;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        // The test replaces the shared clock and clears records, so it needs an idle guidance session.
        if (Records.Count != 0 || VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state or a pending hand-over.");
            return;
        }

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        bool active = GuidanceWindow.ModActive;
        bool guidanceEnabled = SharedVehicleHooks.GuidanceEnabled;
        bool kfsEnabled = KfsPrototype.Enabled;
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.handover");
        using var staging = AutoStageTestPatches.Apply();
        try
        {
            GuidanceWindow.SetModActive(true);
            SharedVehicleHooks.GuidanceEnabled = true;
            StagingHooks.DecouplerActivating += GuidanceWindow.OnDecouplerActivating;
            harmony.Patch(Method("SimNow"),
                prefix: new HarmonyMethod(typeof(GuidanceBoosterHandoverTest), nameof(Clock)));
            harmony.Patch(AccessTools.Method(typeof(Vehicle), nameof(Vehicle.UpdateAfterPartTreeModification)),
                postfix: new HarmonyMethod(typeof(GuidanceFeature), "OnPartTreeModified"));
            _now = 100.0;
            if (saves.Count > 0)
            {
                EverySeparatedVehicleIsAdopted(t, home, saves[0], ambient);
                TwoPendingSeparationsCoexist(t, home, saves[0]);
                AnUnclaimedRecordExpires(t, home, saves[0]);
            }
            else
                t.Skip("no saved vehicle for the stock staging and multiple handover cases.");
            CheckDecouplerActivation(t, home, cancel: false);
            CheckDecouplerActivation(t, home, cancel: true);
            CheckKfsStage(t, home);
            CheckExactRow(t, home);
            CheckRestoredRoot(t, home, rcsOwns: false);
            CheckRestoredRoot(t, home, rcsOwns: true);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            StagingHooks.DecouplerActivating -= GuidanceWindow.OnDecouplerActivating;
            GuidanceWindow.SetModActive(active);
            SharedVehicleHooks.GuidanceEnabled = guidanceEnabled;
            KfsPrototype.Enabled = kfsEnabled;
            ambient.SetValue(null, previousAmbient);
            Records.Clear();
            _siteLat.Clear();
            foreach (KeyValuePair<Vehicle, VehicleAutopilotState> entry in VehicleAutopilotState.Snapshot())
                VehicleAutopilotState.Remove(entry.Key);
        }
    }

    private static void EverySeparatedVehicleIsAdopted(TestContext t, IParentBody home, string save,
                                                       FieldInfo ambient)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        try
        {
            Vehicle parent;
            try
            {
                parent = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandover",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{save}: {e.Message}");
                return;
            }

            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 20);

            if (TargetFirstSeparation(parent, driver) == null)
            {
                t.Skip("the next sequence of the save separates nothing.");
                return;
            }
            VehicleAutopilotState state = VehicleAutopilotState.For(parent);
            state.StageModelDirty = false;
            state.ReserveArmed = true;
            state.ReserveTanks.Add(parent.Parts.Modules.Get<Tank>()[0]);

            // Activation queues the separation, and SimDriver.Step drains the input buffer.
            int before = Records.Count;
            parent.Parts.SequenceList.ActivateNextSequence(parent);
            int armed = Records.Count - before;
            t.Check("stock staging arms the targeted decoupler", armed > 0);
            t.Check("queuing a separation keeps the live reserve until the split",
                !state.StageModelDirty && state.ReserveArmed);
            driver.Step(0.05, 20);
            t.Check("the actual split invalidates the reserve and stage model",
                state.StageModelDirty && !state.ReserveArmed && state.ReserveTanks.Count == 0);

            List<Vehicle> children = new();
            foreach (Astronomical body in t.System.All.AsSpan())
            {
                if (body is Vehicle child && child is not KittenEva
                    && !preexisting.Contains(child.Id) && !ReferenceEquals(child, parent))
                    children.Add(child);
            }
            if (children.Count == 0)
            {
                t.Fail("separation", "the activation produced no separated vehicle");
                return;
            }
            t.Info($"the separation produced {children.Count} vehicle(s)");

            int adopted = 0;
            foreach (Vehicle child in children)
            {
                if (Adopt(child))
                    adopted++;
            }
            t.Check("every separated vehicle adopts a record", adopted == children.Count,
                $"{adopted} of {children.Count}");
            t.Check("each adoption consumes exactly one record", Records.Count == armed - adopted);

            int withState = 0;
            foreach (Vehicle child in children)
            {
                if (VehicleAutopilotState.TryGet(child, out _))
                    withState++;
            }
            t.Check("an adopted vehicle has guidance state of its own", withState == adopted);

            // Each craft flies to the site set for its own root, not the first root's (#130).
            int ownSite = 0;
            foreach (Vehicle child in children)
            {
                if (VehicleAutopilotState.TryGet(child, out VehicleAutopilotState childState)
                    && childState.SiteLatDeg == ArmedSiteLat(child))
                    ownSite++;
            }
            t.Check("each adopted vehicle is sent to its own root's site", ownSite == adopted, $"{ownSite} of {adopted}");

            Span<Tank> migrated = children[0].Parts.Modules.Get<Tank>();
            if (t.Check("the separated stage has a migrated tank", !migrated.IsEmpty))
            {
                ambient.SetValue(null, state);
                state.ReserveArmed = true;
                state.StageModelDirty = false;
                state.ReserveTanks.Add(migrated[0]);
                double mass = (double)Method("ReserveTankMass").Invoke(null, [parent])!;
                t.Check("a migrated tank invalidates the cache without reading the parent's moles",
                    double.IsNaN(mass) && state.StageModelDirty && !state.ReserveArmed && state.ReserveTanks.Count == 0);
            }

            if (children.Count < 2)
                t.Skip("the save separates one vehicle, so two records from one sequence are untested.");
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // Pending handovers from different vehicles must survive arming and adoption elsewhere.
    private static void TwoPendingSeparationsCoexist(TestContext t, IParentBody home, string save)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        try
        {
            Vehicle first, second;
            try
            {
                first = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandoverFirst",
                    OrbitFixtures.CircularAt(home, 700_000.0, driver.Elapsed));
                second = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandoverSecond",
                    OrbitFixtures.CircularAt(home, 800_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{save}: {e.Message}");
                return;
            }

            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 2);
            if (TargetFirstSeparation(first, driver) == null || TargetFirstSeparation(second, driver) == null)
            {
                t.Skip("the next sequence of the save separates nothing.");
                return;
            }
            int before = Records.Count;
            first.Parts.SequenceList.ActivateNextSequence(first);
            int afterFirst = Records.Count;
            second.Parts.SequenceList.ActivateNextSequence(second);
            t.Check("the first separation arms a record", afterFirst > before);
            t.Check("arming a second separation keeps the records of the first",
                Records.Count > afterFirst, $"{afterFirst} then {Records.Count}");

            int pending = Records.Count;
            t.Check("a booster still on its source cannot adopt its record", !Adopt(second) && Records.Count == pending);
            driver.Step(0.05, 2);
            VehicleAutopilotState secondState = VehicleAutopilotState.For(second);
            Vehicle? separated = FindTargetedChild(t, second, secondState);
            t.Check("adopting from one separation leaves the other pending",
                separated != null && Adopt(separated) && Records.Count == pending - 1 && Records.Count >= afterFirst);
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void AnUnclaimedRecordExpires(TestContext t, IParentBody home, string save)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandoverExpiry",
                    OrbitFixtures.CircularAt(home, 600_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{save}: {e.Message}");
                return;
            }

            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 2);
            if (TargetFirstSeparation(vehicle, driver) == null)
                return;
            int before = Records.Count;
            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            if (!t.Check("a separation leaves a record behind", Records.Count > before))
                return;

            driver.Step(0.05, 2);
            Vehicle? child = FindTargetedChild(t, vehicle, VehicleAutopilotState.For(vehicle));
            _now += WindowS + 1.0;
            t.Check("a record nobody claimed expires",
                child != null && !Adopt(child) && Records.Count == 0);
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static IList Records =>
        (IList)typeof(GuidanceWindow).GetField("_handovers", PrivateStatic)!.GetValue(null)!;

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static bool Clock(ref double __result)
    {
        __result = _now;
        return false;
    }

    // The site each separated root was given, by root id: every root gets one of its own, as a player setting each booster's site would.
    private static readonly Dictionary<uint, double> _siteLat = new();

    // The site this separated vehicle was armed with, or NaN when none of its parts is a root that was given one.
    private static double ArmedSiteLat(Vehicle child)
    {
        ReadOnlySpan<Part> parts = child.Parts.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            if (_siteLat.TryGetValue(parts[i].InstanceId, out double lat))
                return lat;
        }
        return double.NaN;
    }

    // Fire ignition-only rows before assigning a distinct landing site to each root of the first separation.
    private static Sequence? TargetFirstSeparation(Vehicle vehicle, SimDriver driver)
    {
        var roots = new List<Part>();
        for (int row = 0; row < MaxRowsBeforeSeparation; row++)
        {
            Sequence? next = StagingHooks.NextRowToFire(vehicle.Parts.SequenceList);
            if (next == null)
                return null;
            Method("SequenceRoots").Invoke(null, [next, roots]);
            if (roots.Count > 0)
            {
                VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
                foreach (Part root in roots)
                {
                    double lat = 10.0 + _siteLat.Count;
                    _siteLat[root.InstanceId] = lat;
                    state.StageTargets[root.InstanceId] = new Brutal.Numerics.double2(lat, 20.0);
                }
                return next;
            }
            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            driver.Step(0.05, 2);
        }
        return null;
    }

    private static Vehicle? FindTargetedChild(TestContext t, Vehicle source, VehicleAutopilotState state)
    {
        foreach (Astronomical body in t.System.All.AsSpan())
            if (body is Vehicle child && !ReferenceEquals(child, source)
                && state.StageTargets.ContainsKey(child.Parts.Root.InstanceId))
                return child;
        return null;
    }

    private static Vehicle SpawnRocket(TestContext t, IParentBody home, SimDriver driver, string id)
    {
        PartInstance design = DefaultVehicleSaves.FindSave("Rocket")?.VehicleSaveData.RootPartInstance
            ?? throw new InvalidOperationException("Rocket is required for the staging fixture.");
        Vehicle vehicle = VehicleFixtures.SpawnDesign(t.System, home, design, id,
            OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
        driver.Step(0.05, 2);
        return vehicle;
    }

    private static void CheckDecouplerActivation(TestContext t, IParentBody home, bool cancel)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        var delays = new Dictionary<string, double>(StagingConfig.DecouplerDelays);
        try
        {
            PhysicsBubble._forceOffRails = true;
            Vehicle vehicle = SpawnRocket(t, home, driver, "AfcDelayedBooster" + cancel);
            Sequence? row = TargetFirstSeparation(vehicle, driver);
            if (!t.Check("the stock design has a row to delay", row != null))
                return;
            foreach (Part part in row!.Parts)
                foreach (ISequenced module in part.InSequence(row.Number))
                    if (module is Decoupler)
                        StagingConfig.DecouplerDelays[SequencedModules.DelayKey(module)] = DecouplerDelayS;

            int before = Records.Count;
            StagingDetector.Arm(vehicle, true);
            StagingDetector.RequestStaging(vehicle);
            StagingDetector.Sample();
            StagingDetector.Evaluate();
            StagingState state = StagingDetector.StateOf(vehicle);
            PendingStaging? pending = state.Pending;
            if (!t.Check("the row holds a decoupler for longer than the handover window",
                pending?.DecouplerDelay == DecouplerDelayS && pending.DecouplersPending))
                return;
            t.Check("enqueuing the delayed row arms no handover", Records.Count == before);
            int partsBefore = vehicle.Parts.Count;

            if (cancel)
                KfsFixtures.LoseControl(vehicle, driver);
            _now += DecouplerDelayS;
            MethodInfo tick = AccessTools.Method(typeof(StagingDetector), "TickPendingStaging");
            tick.Invoke(null, [vehicle, state, pending!.DecouplerDeadline]);
            driver.Step(0.05, 2);
            if (cancel)
            {
                t.Check("loss cancels the delayed separation and its handover",
                    !vehicle.IsControllable && state.Pending == null && Records.Count == before
                    && vehicle.Parts.Count == partsBefore);
            }
            else
            {
                Vehicle? child = FindTargetedChild(t, vehicle, VehicleAutopilotState.For(vehicle));
                t.Check("the delayed decoupler gives its booster a fresh handover",
                    child != null && Records.Count > before && Adopt(child));
            }
        }
        finally
        {
            StagingConfig.DecouplerDelays.Clear();
            foreach ((string key, double delay) in delays)
                StagingConfig.DecouplerDelays.Add(key, delay);
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void CheckKfsStage(TestContext t, IParentBody home)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        bool kfsEnabled = KfsPrototype.Enabled;
        Vehicle? vehicle = null;
        try
        {
            PhysicsBubble._forceOffRails = true;
            vehicle = SpawnRocket(t, home, driver, "AfcKfsBooster");
            Sequence? row = TargetFirstSeparation(vehicle, driver);
            if (!t.Check("the kFS stage will separate a targeted booster", row != null))
                return;

            KfsPrototype.Enabled = true;
            const string program = "#include <kfs.h>\n#pragma kfs tick_hz 10\n#pragma kfs task step rate_hz=10\nvoid step(void) { kfs_stage(); }\n";
            var host = KfsFixtures.QueuePilot(vehicle, program);
            int before = Records.Count;
            for (int i = 0; i < 6 && !host.Computer.AwaitingStage; i++)
                driver.Step(0.05);
            t.Check("the running pilot queued a kFS stage service",
                host.Computer.UnderKfsControl && host.Computer.AwaitingStage && !row!.Activated);
            host.Services.MainThread();
            t.Check("stock applies the kFS row without an AFC handover",
                row!.Activated && !host.Computer.AwaitingStage && Records.Count == before);
            host.Computer.Stop();
            driver.Step(0.05, 2);
            Vehicle? child = FindTargetedChild(t, vehicle, VehicleAutopilotState.For(vehicle));
            t.Check("the kFS row physically separates without AFC adopting the child",
                child != null && !Adopt(child) && !VehicleAutopilotState.TryGet(child, out _));
        }
        finally
        {
            vehicle?.KfsHost?.Computer.Stop();
            if (vehicle != null)
                vehicle.KfsHost = null;
            KfsPrototype.Enabled = kfsEnabled;
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void CheckExactRow(TestContext t, IParentBody home)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        try
        {
            Vehicle vehicle = SpawnRocket(t, home, driver, "AfcMultiModuleRow");
            Decoupler? decoupler = null;
            foreach (Decoupler candidate in vehicle.Parts.Modules.Get<Decoupler>())
                if (StagingHooks.CanActivate(candidate))
                {
                    decoupler = candidate;
                    break;
                }
            if (!t.Check("the stock part has a connected decoupler", decoupler != null))
                return;

            // The same full part appears on both rows because it also carries an engine.
            decoupler!.Sequence = 21;
            Part part = decoupler.Parent;
            part.Modules.Add(new EngineController("AfcTestEngine")
            {
                Parent = part, Rockets = [], MinimumThrottle = 0f, Sequence = 20,
            });
            Sequence ignition = new(vehicle.Parts, 20);
            Sequence separation = new(vehicle.Parts, 21);
            var roots = new List<Part>();
            Method("SequenceRoots").Invoke(null, [ignition, roots]);
            t.Check("the engine's row contains the part without inheriting its decoupler",
                ignition.Parts.Contains(part.FullPart) && roots.Count == 0);
            Method("SequenceRoots").Invoke(null, [separation, roots]);
            t.Check("the decoupler's own row finds the departing root", roots.Count == 1);
            decoupler.SetIsEnabled(false);
            decoupler.Sequence = 21;
            separation.RecacheParts();
            Method("SequenceRoots").Invoke(null, [separation, roots]);
            t.Check("a disabled decoupler on the correct row cannot separate", roots.Count == 0);
        }
        finally
        {
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void CheckRestoredRoot(TestContext t, IParentBody home, bool rcsOwns)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? child = null;
        try
        {
            PhysicsBubble._forceOffRails = true;
            Vehicle source = SpawnRocket(t, home, driver, "AfcRestoredBooster" + rcsOwns);
            Sequence? row = TargetFirstSeparation(source, driver);
            if (!t.Check("the fixture has a targeted separation", row != null))
                return;

            if (FindPreDockFixture(row!) is not { } found)
            {
                t.Fail("pre-dock fixture", "the detached subtree needs a connector and another possible root");
                return;
            }
            (Part attachment, Part.Connector connector, Part restoredRoot) = found;
            // Give the stock split the same pre-dock record it saves when docking reroots the child.
            DockingPort port = (DockingPort)AccessTools.Constructor(typeof(DockingPort), [typeof(string)])
                .Invoke(["AfcTestDock"]);
            port.Parent = attachment;
            port.Connector = connector;
            port.PreDock = new DockingPort.PreDockRecord
            {
                RootPart = restoredRoot,
                RootPositionParentAsmb = restoredRoot.PositionParentAsmb,
                RootAsmb2ParentAsmb = restoredRoot.Asmb2ParentAsmb,
            };
            attachment.Modules.Add(port);

            var site = VehicleAutopilotState.For(source).StageTargets[attachment.InstanceId];
            source.Parts.SequenceList.ActivateNextSequence(source);
            driver.Step(0.05, 2);
            child = attachment.Tree?.OwningVehicle;
            if (!t.Check("stock split consumes the pre-dock record and restores a different root",
                child != null && !ReferenceEquals(child, source) && port.PreDock == null
                && ReferenceEquals(child.Parts.Root, restoredRoot) && !ReferenceEquals(restoredRoot, attachment)))
                return;

            TestSupport.SetManualControlInputs(child!, 0.37f, engineOn: true);
            var inputs = AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");
            if (rcsOwns)
            {
                if (!t.Check("RCS owns the newly separated craft",
                    VehicleControlOwnership.TryClaim(child!, ControlClaimant.RcsTranslation, out _)))
                    return;
                t.Check("handover leaves a craft held by RCS untouched",
                    !Adopt(child!) && inputs(child!).EngineOn && inputs(child!).EngineThrottle == 0.37f
                    && VehicleControlOwnership.Holds(child!, ControlClaimant.RcsTranslation)
                    && !VehicleAutopilotState.TryGet(child!, out _));
                VehicleControlOwnership.Release(child!, ControlClaimant.RcsTranslation);
                t.Check("the refused handover cannot start after RCS releases", !Adopt(child!));
                return;
            }

            t.Check("rerooted child adopts the attachment's site and cuts its inherited engine",
                Adopt(child!) && !inputs(child!).EngineOn && VehicleAutopilotState.TryGet(child!, out var state)
                && state.SiteLatDeg == site.X && state.SiteLonDeg == site.Y
                && state.HandoverPendingUntil > _now && VehicleControlOwnership.HolderOf(child!) == ControlClaimant.None);
        }
        finally
        {
            if (child != null)
                VehicleControlOwnership.Release(child, ControlClaimant.RcsTranslation);
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static (Part Attachment, Part.Connector Connector, Part RestoredRoot)? FindPreDockFixture(Sequence row)
    {
        MethodInfo detachedRoot = Method("DetachedRoot");
        foreach (Part part in row.Parts)
            foreach (ISequenced module in part.InSequence(row.Number))
            {
                if (module is not Decoupler decoupler || !StagingHooks.CanActivate(decoupler)
                    || detachedRoot.Invoke(null, [decoupler]) is not Part attachment
                    || attachment.TreeChildren.FirstOrDefault() is not Part restoredRoot)
                    continue;
                Part.Connection connection = decoupler.Connector.Connection!;
                var endpoint = ReferenceEquals(connection.Connectors[0].ConnectionPart, attachment)
                    ? connection.Connectors[0] : connection.Connectors[1];
                if (endpoint is Part.Connector connector)
                    return (attachment, connector, restoredRoot);
            }
        return null;
    }

    private static bool Adopt(Vehicle vehicle) =>
        (bool)Method("TryAdoptBooster").Invoke(null, [vehicle])!;
}
