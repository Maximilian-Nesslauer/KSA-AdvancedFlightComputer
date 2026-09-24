using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// UncompressedSave.Write returns false when it cannot write the save, and its postfix still runs.
// Drive the write patch directly to verify that a failed write leaves the save scope and the registry on the previous save.
public sealed class FailedWriteTest : AfcTest
{
    public override string Name => "afc-failed-write";

    protected override void Execute(TestContext t)
    {
        FieldInfo saveIdField = StaticField(typeof(SaveLoadObserver), "<CurrentSaveId>k__BackingField");
        FieldInfo writtenEvent = StaticField(typeof(SaveLoadObserver), "SaveWritten");
        FieldInfo configPath = StaticField(typeof(MultiPassRegistry), "_configPath");
        FieldInfo modDir = StaticField(typeof(MultiPassRegistry), "_modDir");
        object? oldSaveId = saveIdField.GetValue(null);
        object? oldEvent = writtenEvent.GetValue(null);
        object? oldPath = configPath.GetValue(null);
        object? oldDir = modDir.GetValue(null);
        var oldEntries = new List<MultiPassExecution>(MultiPassRegistry.Snapshot.Values);

        Harmony harmony = new("com.maxi.afc.harnesstests.failed-write");
        string temp = Path.Combine(Path.GetTempPath(), "afc-failed-write-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(temp);
        try
        {
            SaveLoadObserver.ApplyPatches(harmony);
            CheckBinding(t, harmony.Id);

            string file = Path.Combine(temp, "multipass.toml");
            configPath.SetValue(null, file);
            modDir.SetValue(null, temp);
            writtenEvent.SetValue(null, null);
            var events = new List<(string Old, string New)>();
            SaveLoadObserver.SaveWritten += (oldSave, newSave) => events.Add((oldSave, newSave));

            saveIdField.SetValue(null, "world");
            MultiPassRegistry.Reset();
            MultiPassRegistry.Add(Make("world", "live-vehicle"));

            DriveWrite("new-save", succeeded: false);
            t.Check("failed write keeps the save scope", SaveLoadObserver.CurrentSaveId == "world");
            t.Check("failed write keeps the registry scope", MultiPassRegistry.Snapshot.ContainsKey(("world", "live-vehicle"))
                && !MultiPassRegistry.Snapshot.ContainsKey(("new-save", "live-vehicle")));
            t.Check("failed write raises no SaveWritten", events.Count == 0);
            t.Check("failed write persists no registry", !File.Exists(file));

            DriveWrite("new-save", succeeded: true);
            t.Check("completed write moves the save scope", SaveLoadObserver.CurrentSaveId == "new-save");
            t.Check("completed write moves the registry scope", MultiPassRegistry.Snapshot.ContainsKey(("new-save", "live-vehicle"))
                && !MultiPassRegistry.Snapshot.ContainsKey(("world", "live-vehicle")));
            t.Check("completed write raises SaveWritten once", events.Count == 1 && events[0] == ("world", "new-save"));
            t.Check("completed write persists the registry", File.Exists(file));
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            configPath.SetValue(null, oldPath);
            modDir.SetValue(null, oldDir);
            writtenEvent.SetValue(null, oldEvent);
            saveIdField.SetValue(null, oldSaveId);
            MultiPassRegistry.Reset();
            foreach (MultiPassExecution entry in oldEntries) MultiPassRegistry.Add(entry);
            if (Directory.Exists(temp)) Directory.Delete(temp, recursive: true);
        }
    }

    // Both constructors touch the save folder, but the patch only needs the save ID and the result of the write.
    private static void DriveWrite(string saveId, bool succeeded)
    {
        var save = (UncompressedSave)RuntimeHelpers.GetUninitializedObject(typeof(UncompressedSave));
        FieldInfo idField = typeof(GameSave).GetField("<Id>k__BackingField",
                BindingFlags.Instance | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(typeof(GameSave).FullName, "<Id>k__BackingField");
        idField.SetValue(save, saveId);

        MethodInfo postfix = AccessTools.Method(
                AccessTools.Inner(typeof(SaveLoadObserver), "WritePatch"), "Postfix")
            ?? throw new MissingMethodException(typeof(SaveLoadObserver).FullName, "WritePatch.Postfix");
        postfix.Invoke(null, [save, succeeded]);
    }

    private static void CheckBinding(TestContext t, string owner)
    {
        MethodInfo target = AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Write), Type.EmptyTypes)
            ?? throw new MissingMethodException(typeof(UncompressedSave).FullName, nameof(UncompressedSave.Write));
        Patches? patches = Harmony.GetPatchInfo(target);
        t.Check("one write postfix bound", patches != null
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
