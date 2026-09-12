using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A refused UncompressedSave.Load still runs its postfix. Drive the postfix directly to verify that
// the current save state stays unchanged without adding the stock refusal alert to the test log.
public sealed class RefusedLoadTest : AfcTest
{
    private const double MarkerAltitudeM = 654_321.0;

    public override string Name => "afc-refused-load";

    protected override void Execute(TestContext t)
    {
        if (Program.IsEditorOpen)
        {
            t.Skip("the vehicle editor is open, so the accepted case cannot run");
            return;
        }

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
        bool oldEditorFlag = Program.EditorFlag;

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

            // Program.OnFrameEditor changes the live world when EditorFlag is set, so do not drive a frame here.
            Program.EditorFlag = true;
            DrivePostfix("phantom");
            t.Check("refused load keeps the save scope", SaveLoadObserver.CurrentSaveId == "world");
            t.Check("refused load raises no SaveLoaded", loadedCount == 0);
            t.Check("refused load keeps the live registry entry", MultiPassRegistry.Has("live-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("accepted", "disk-vehicle")));
            t.Check("refused load keeps save-scoped state", ManeuverToolsWindow.TargetAltitude == MarkerAltitudeM);

            Program.EditorFlag = false;
            DrivePostfix("accepted");
            t.Check("accepted load moves the save scope", SaveLoadObserver.CurrentSaveId == "accepted");
            t.Check("accepted load raises SaveLoaded once", loadedCount == 1);
            t.Check("accepted load restores the registry from disk", MultiPassRegistry.Has("disk-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("world", "live-vehicle")));
            t.Check("accepted load resets save-scoped state", ManeuverToolsWindow.TargetAltitude == 0.0);
        }
        finally
        {
            Program.EditorFlag = oldEditorFlag;
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

    // Both constructors touch the save folder, but the postfix only needs the save ID.
    private static void DrivePostfix(string saveId)
    {
        var save = (UncompressedSave)RuntimeHelpers.GetUninitializedObject(typeof(UncompressedSave));
        FieldInfo idField = typeof(GameSave).GetField("<Id>k__BackingField",
                BindingFlags.Instance | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(typeof(GameSave).FullName, "<Id>k__BackingField");
        idField.SetValue(save, saveId);

        MethodInfo postfix = AccessTools.Method(
                AccessTools.Inner(typeof(SaveLoadObserver), "LoadPatch"), "Postfix")
            ?? throw new MissingMethodException(typeof(SaveLoadObserver).FullName, "LoadPatch.Postfix");
        postfix.Invoke(null, [save]);
    }

    private static void CheckBinding(TestContext t, string owner)
    {
        MethodInfo target = AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Load), Type.EmptyTypes)
            ?? throw new MissingMethodException(typeof(UncompressedSave).FullName, nameof(UncompressedSave.Load));
        Patches? patches = Harmony.GetPatchInfo(target);
        t.Check("one load postfix bound", patches != null
            && patches.Postfixes.Count(p => p.owner == owner) == 1
            && !patches.Prefixes.Any(p => p.owner == owner));
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
