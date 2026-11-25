using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// UncompressedSave.Load returns without UncompressedSave.ApplyLoaded when the editor refuses the load or the universe file cannot be read, and SaveSystemSwap.RequestLoad defers ApplyLoaded for a save of another star system or skips it for one that is not installed.
// The binding check covers all of those paths. Run the real Load on a save that cannot be read to verify that a load that did not happen leaves the current save state unchanged, then drive the load patch for an applied load.
public sealed class RefusedLoadTest : AfcTest
{
    private const double MarkerAltitudeM = 654_321.0;

    public override string Name => "afc-refused-load";

    protected override void Execute(TestContext t)
    {
        FieldInfo saveIdField = StaticField(typeof(SaveLoadObserver), "<CurrentSaveId>k__BackingField");
        FieldInfo loadedEvent = StaticField(typeof(SaveLoadObserver), "SaveLoaded");
        FieldInfo configPath = StaticField(typeof(MultiPassRegistry), "_configPath");
        FieldInfo modDir = StaticField(typeof(MultiPassRegistry), "_modDir");
        object? oldSaveId = saveIdField.GetValue(null);
        object? oldEvent = loadedEvent.GetValue(null);
        object? oldPath = configPath.GetValue(null);
        object? oldDir = modDir.GetValue(null);
        var oldEntries = new List<MultiPassExecution>(MultiPassRegistry.Snapshot.Values);
        double oldAltitude = OrbitToolForm.TargetAltitude;

        Harmony harmony = new("com.maxi.afc.harnesstests.refused-load");
        string temp = Path.Combine(Path.GetTempPath(), "afc-refused-load-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(temp);
        try
        {
            SaveLoadObserver.ApplyPatches(harmony);
            CheckBinding(t, harmony.Id);

            string file = Path.Combine(temp, "multipass.toml");
            using (var writer = new StreamWriter(file))
                MultiPassRegistry.WriteToml(writer, [Make("accepted", "disk-vehicle")]);
            configPath.SetValue(null, file);
            modDir.SetValue(null, temp);
            loadedEvent.SetValue(null, null);
            int loadedCount = 0;
            SaveLoadObserver.SaveLoaded += () => loadedCount++;

            saveIdField.SetValue(null, "world");
            MultiPassRegistry.Reset();
            MultiPassRegistry.Add(Make("world", "live-vehicle"));
            OrbitToolForm.TargetAltitude = MarkerAltitudeM;

            string empty = Path.Combine(temp, "unreadable-save");
            Directory.CreateDirectory(empty);
            DriveUnreadableLoad("phantom", empty);
            t.Check("unloaded save keeps the save scope", SaveLoadObserver.CurrentSaveId == "world");
            t.Check("unloaded save raises no SaveLoaded", loadedCount == 0);
            t.Check("unloaded save keeps the live registry entry", MultiPassRegistry.Has("live-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("accepted", "disk-vehicle")));
            t.Check("unloaded save keeps save-scoped state", OrbitToolForm.TargetAltitude == MarkerAltitudeM);

            DriveAppliedLoad("accepted");
            t.Check("accepted load moves the save scope", SaveLoadObserver.CurrentSaveId == "accepted");
            t.Check("accepted load raises SaveLoaded once", loadedCount == 1);
            t.Check("accepted load restores the registry from disk", MultiPassRegistry.Has("disk-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("world", "live-vehicle")));
            t.Check("accepted load resets save-scoped state", OrbitToolForm.TargetAltitude == 0.0);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            OrbitToolForm.TargetAltitude = oldAltitude;
            configPath.SetValue(null, oldPath);
            modDir.SetValue(null, oldDir);
            loadedEvent.SetValue(null, oldEvent);
            saveIdField.SetValue(null, oldSaveId);
            MultiPassRegistry.Reset();
            foreach (MultiPassExecution entry in oldEntries) MultiPassRegistry.Add(entry);
            if (Directory.Exists(temp)) Directory.Delete(temp, recursive: true);
        }
    }

    // Both constructors touch the save folder, so the save is built without them, with only the ID, the folder and an empty UniverseData.
    private static UncompressedSave NewSave(string saveId, DirectoryInfo? directory)
    {
        var save = (UncompressedSave)RuntimeHelpers.GetUninitializedObject(typeof(UncompressedSave));
        FieldInfo idField = typeof(GameSave).GetField("<Id>k__BackingField",
                BindingFlags.Instance | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(typeof(GameSave).FullName, "<Id>k__BackingField");
        idField.SetValue(save, saveId);
        typeof(UncompressedSave).GetField(nameof(UncompressedSave.Directory))!.SetValue(save, directory);
        save.UniverseData = new UniverseData();
        return save;
    }

    // UniverseData.LoadFrom throws for a folder with no universe file, and UncompressedSave.Load logs that and returns before SaveSystemSwap.RequestLoad.
    private static void DriveUnreadableLoad(string saveId, string emptyFolder)
        => NewSave(saveId, new DirectoryInfo(emptyFolder)).Load();

    // The real ApplyLoaded would deserialize the save into the running universe, so only its postfix runs here.
    private static void DriveAppliedLoad(string saveId)
        => LoadPatchMethod("Postfix").Invoke(null, [NewSave(saveId, null)]);

    private static MethodInfo LoadPatchMethod(string name)
        => AccessTools.Method(AccessTools.Inner(typeof(SaveLoadObserver), "LoadPatch"), name)
           ?? throw new MissingMethodException(typeof(SaveLoadObserver).FullName, "LoadPatch." + name);

    private static void CheckBinding(TestContext t, string owner)
    {
        MethodInfo apply = AccessTools.Method(typeof(UncompressedSave), "ApplyLoaded", Type.EmptyTypes)
            ?? throw new MissingMethodException(typeof(UncompressedSave).FullName, "ApplyLoaded");
        Patches? patches = Harmony.GetPatchInfo(apply);
        t.Check("one postfix bound on the apply of a load", patches != null
            && patches.Prefixes.Count(p => p.owner == owner) == 0
            && patches.Postfixes.Count(p => p.owner == owner) == 1);
        MethodInfo load = AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Load), Type.EmptyTypes)
            ?? throw new MissingMethodException(typeof(UncompressedSave).FullName, nameof(UncompressedSave.Load));
        Patches? loadPatches = Harmony.GetPatchInfo(load);
        t.Check("no patch bound on the read of a load", loadPatches == null
            || loadPatches.Prefixes.Concat(loadPatches.Postfixes).All(p => p.owner != owner));
    }

    private static MultiPassExecution Make(string save, string vehicle)
        => new()
        {
            SaveId = save,
            VehicleId = vehicle,
            Intent = new ApseIntent
            {
                ParentId = "Earth",
                IsSetApoapsis = true,
                TargetRadiusMeters = 7_200_000.0,
            },
            Mode = SplitMode.EqualBurnTime,
            PassCountTotal = 2,
        };

    private static FieldInfo StaticField(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.NonPublic)
           ?? throw new MissingFieldException(type.FullName, name);
}
