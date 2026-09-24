using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// UncompressedSave.Load returns normally when the editor refuses the load or the universe file cannot be read, so its postfix still runs.
// Drive the load patch directly to verify that a load that did not happen leaves the current save state unchanged.
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
        double oldAltitude = ManeuverToolsWindow.TargetAltitude;

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
            ManeuverToolsWindow.TargetAltitude = MarkerAltitudeM;

            DriveLoad("phantom", completes: false);
            t.Check("unloaded save keeps the save scope", SaveLoadObserver.CurrentSaveId == "world");
            t.Check("unloaded save raises no SaveLoaded", loadedCount == 0);
            t.Check("unloaded save keeps the live registry entry", MultiPassRegistry.Has("live-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("accepted", "disk-vehicle")));
            t.Check("unloaded save keeps save-scoped state", ManeuverToolsWindow.TargetAltitude == MarkerAltitudeM);

            DriveLoad("accepted", completes: true);
            t.Check("accepted load moves the save scope", SaveLoadObserver.CurrentSaveId == "accepted");
            t.Check("accepted load raises SaveLoaded once", loadedCount == 1);
            t.Check("accepted load restores the registry from disk", MultiPassRegistry.Has("disk-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("world", "live-vehicle")));
            t.Check("accepted load resets save-scoped state", ManeuverToolsWindow.TargetAltitude == 0.0);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            ManeuverToolsWindow.TargetAltitude = oldAltitude;
            configPath.SetValue(null, oldPath);
            modDir.SetValue(null, oldDir);
            loadedEvent.SetValue(null, oldEvent);
            saveIdField.SetValue(null, oldSaveId);
            MultiPassRegistry.Reset();
            foreach (MultiPassExecution entry in oldEntries) MultiPassRegistry.Add(entry);
            if (Directory.Exists(temp)) Directory.Delete(temp, recursive: true);
        }
    }

    // Both constructors touch the save folder, but the patch only needs the save ID and its UniverseData.
    // A completed load replaces UniverseData, and a refused or unreadable one leaves it as the prefix saw it.
    private static void DriveLoad(string saveId, bool completes)
    {
        var save = (UncompressedSave)RuntimeHelpers.GetUninitializedObject(typeof(UncompressedSave));
        FieldInfo idField = typeof(GameSave).GetField("<Id>k__BackingField",
                BindingFlags.Instance | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(typeof(GameSave).FullName, "<Id>k__BackingField");
        idField.SetValue(save, saveId);
        save.UniverseData = new UniverseData();

        object?[] prefixArgs = [save, null];
        LoadPatchMethod("Prefix").Invoke(null, prefixArgs);
        if (completes)
            save.UniverseData = new UniverseData();
        LoadPatchMethod("Postfix").Invoke(null, [save, prefixArgs[1]]);
    }

    private static MethodInfo LoadPatchMethod(string name)
        => AccessTools.Method(AccessTools.Inner(typeof(SaveLoadObserver), "LoadPatch"), name)
           ?? throw new MissingMethodException(typeof(SaveLoadObserver).FullName, "LoadPatch." + name);

    private static void CheckBinding(TestContext t, string owner)
    {
        MethodInfo target = AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Load), Type.EmptyTypes)
            ?? throw new MissingMethodException(typeof(UncompressedSave).FullName, nameof(UncompressedSave.Load));
        Patches? patches = Harmony.GetPatchInfo(target);
        t.Check("one load prefix and one load postfix bound", patches != null
            && patches.Prefixes.Count(p => p.owner == owner) == 1
            && patches.Postfixes.Count(p => p.owner == owner) == 1);
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
