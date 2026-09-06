using System.Globalization;
using System.Reflection;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class MultiPassRegistryTest : AfcTest
{
    public override string Name => "afc-multipass-registry";

    protected override void Execute(TestContext t)
    {
        CheckRoundTrip(t);
        CheckSaveScope(t);
    }

    private static MultiPassExecution Make(string save, string vehicle, IManeuverIntent? intent = null)
        => new()
        {
            SaveId = save,
            VehicleId = vehicle,
            Intent = intent ?? new ApseIntent
            {
                ParentId = "Earth",
                IsSetApoapsis = true,
                TargetRadiusMeters = 7200000.25,
            },
            Mode = SplitMode.EqualBurnTime,
            PassCountTotal = 4,
            PassIndex = 1,
            CurrentBurnTimeSec = 12345.125,
            CurrentBurnDvMagnitudeMs = 123.456789012345,
        };

    private static void CheckRoundTrip(TestContext t)
    {
        var hohmann = new HohmannTransferIntent
        {
            ParentId = "Earth",
            TargetId = "Mars",
            TFinalSec = 76543.125,
            DFinalVlf = new double3(1.25, -2.5, 3200.75),
            IsCrossParent = true,
            VInfMs = 2900.125,
            ApoTargetRadiusMeters = 0,
            ParkingPeriodSec = 5400.25,
        };
        MultiPassExecution apse = Make("save\\one", "ship \"A\"#1");
        apse.BurnAutoEngagedThisPass = true;
        apse.AwaitingMaterialization = true;
        apse.ReengageAutoOnNextBurn = true;
        using var writer = new StringWriter(CultureInfo.GetCultureInfo("de-DE"));
        int written = MultiPassRegistry.WriteToml(writer,
            [apse, Make("other", "ship", hohmann), Make("", "transient")]);
        var parsed = new Dictionary<(string SaveId, string VehicleId), MultiPassExecution>();
        MultiPassRegistry.ParseLines(writer.ToString().Split('\n'), "round-trip", parsed);
        t.Check("transient execution is not persisted", written == 2 && parsed.Count == 2);
        if (t.Check("escaped identity survives", parsed.TryGetValue((apse.SaveId, apse.VehicleId), out var back)))
        {
            t.Check("pass state survives", back!.PassIndex == 1 && back.PassCountTotal == 4
                && back.Mode == SplitMode.EqualBurnTime);
            t.Check("burn fingerprint is exact", back.CurrentBurnTimeSec == apse.CurrentBurnTimeSec
                && back.CurrentBurnDvMagnitudeMs == apse.CurrentBurnDvMagnitudeMs);
            t.Check("transient control state resets", back.CurrentBurn == null
                && !back.AwaitingMaterialization && !back.BurnAutoEngagedThisPass
                && !back.ReengageAutoOnNextBurn);
            t.Check("apse intent survives", back.Intent is ApseIntent a
                && a.IsSetApoapsis && a.ParentId == "Earth" && a.TargetRadiusMeters == 7200000.25);
        }
        t.Check("Hohmann intent survives", parsed.TryGetValue(("other", "ship"), out var h)
            && h.Intent is HohmannTransferIntent hi && hi.ParentId == hohmann.ParentId
            && hi.TargetId == hohmann.TargetId && hi.TFinalSec == hohmann.TFinalSec
            && hi.DFinalVlf == hohmann.DFinalVlf && hi.IsCrossParent == hohmann.IsCrossParent
            && hi.VInfMs == hohmann.VInfMs && hi.ApoTargetRadiusMeters == hohmann.ApoTargetRadiusMeters
            && hi.ParkingPeriodSec == hohmann.ParkingPeriodSec);
    }

    private static void CheckSaveScope(TestContext t)
    {
        FieldInfo configPath = Field(typeof(MultiPassRegistry), "_configPath");
        FieldInfo modDir = Field(typeof(MultiPassRegistry), "_modDir");
        FieldInfo saveId = Field(typeof(SaveLoadObserver), "<CurrentSaveId>k__BackingField");
        FieldInfo writtenEvent = Field(typeof(SaveLoadObserver), "SaveWritten");
        object? oldPath = configPath.GetValue(null);
        object? oldDir = modDir.GetValue(null);
        object? oldId = saveId.GetValue(null);
        object? oldEvent = writtenEvent.GetValue(null);
        var oldEntries = new List<MultiPassExecution>(MultiPassRegistry.Snapshot.Values);
        string temp = Path.Combine(Path.GetTempPath(), "afc-multipass-" + Guid.NewGuid().ToString("N"));
        string file = Path.Combine(temp, "multipass.toml");
        try
        {
            configPath.SetValue(null, file);
            modDir.SetValue(null, temp);
            saveId.SetValue(null, "");
            writtenEvent.SetValue(null, null);
            MultiPassRegistry.Reset();
            MultiPassExecution active = Make("", "shared-vehicle");
            MultiPassRegistry.Add(active);
            MultiPassRegistry.Add(Make("first", "stale-vehicle"));
            MultiPassRegistry.Add(Make("unrelated", "shared-vehicle"));
            var events = new List<(string Old, string New, string Current)>();
            SaveLoadObserver.SaveWritten += (oldSave, newSave) =>
                events.Add((oldSave, newSave, SaveLoadObserver.CurrentSaveId));

            SaveLoadObserver.OnSaveWritten("first");
            t.Check("first save promotes transient execution", ReferenceEquals(Get("shared-vehicle"), active)
                && active.SaveId == "first" && !MultiPassRegistry.Has("stale-vehicle"));
            MultiPassRegistry.Add(Make("second", "overwritten-vehicle"));
            SaveLoadObserver.OnSaveWritten("second");
            t.Check("Save-As moves a non-empty scope", ReferenceEquals(Get("shared-vehicle"), active)
                && active.SaveId == "second" && !MultiPassRegistry.Has("overwritten-vehicle")
                && !MultiPassRegistry.Snapshot.ContainsKey(("first", "shared-vehicle")));
            t.Check("unrelated save remains", MultiPassRegistry.Snapshot.ContainsKey(("unrelated", "shared-vehicle")));
            t.Check("write event receives both IDs after scope change", events.Count == 2
                && events[0] == ("", "first", "first") && events[1] == ("first", "second", "second"));
            SaveLoadObserver.OnSaveWritten("second");
            t.Check("same-scope write keeps execution", ReferenceEquals(Get("shared-vehicle"), active));
            MultiPassRegistry.Load();
            t.Check("disk round-trip restores active scope", Get("shared-vehicle") is { PassIndex: 1, PassCountTotal: 4 }
                && MultiPassRegistry.Count == 2);
            t.Check("save leaves no temporary file", Directory.GetFiles(temp).Length == 1 && File.Exists(file));
        }
        finally
        {
            configPath.SetValue(null, oldPath);
            modDir.SetValue(null, oldDir);
            saveId.SetValue(null, oldId);
            writtenEvent.SetValue(null, oldEvent);
            MultiPassRegistry.Reset();
            foreach (MultiPassExecution entry in oldEntries) MultiPassRegistry.Add(entry);
            if (Directory.Exists(temp)) Directory.Delete(temp, recursive: true);
        }
    }

    private static MultiPassExecution? Get(string vehicle)
        => MultiPassRegistry.TryGet(vehicle, out var exec) ? exec : null;

    private static FieldInfo Field(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.NonPublic)
           ?? throw new MissingFieldException(type.FullName, name);
}
