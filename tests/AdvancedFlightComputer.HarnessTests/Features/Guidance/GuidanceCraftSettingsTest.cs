using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The guidance settings of a craft go into the folder of the save that is written and come back onto the craft of the same id when that save loads.
// A state that is removed and made again stands for the new vehicle objects a real load creates.
public sealed class GuidanceCraftSettingsTest : AfcTest
{
    public override string Name => "afc-guidance-craft-settings";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        string? prismaFixture = Environment.GetEnvironmentVariable("AFC_LANDING_SETTINGS_PRISMA_SAVE");
        VehicleSave? design = prismaFixture == null ? DefaultVehicleSaves.FindSave("Rocket")
            : VehicleSaves.AsSpan().ToArray().FirstOrDefault(save => save.Id == prismaFixture);
        if (design?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("vehicle fixture", $"{prismaFixture ?? "Rocket"} is not available");
            return;
        }

        string firstId = "CraftSettings_" + Guid.NewGuid().ToString("N");
        string otherId = "CraftSettings_" + Guid.NewGuid().ToString("N");
        string renamedId = "CraftSettings_" + Guid.NewGuid().ToString("N");
        string temp = Path.Combine(Path.GetTempPath(), "afc-craft-settings-" + Guid.NewGuid().ToString("N"));
        Vehicle? craft = null;
        Vehicle? other = null;
        try
        {
            craft = VehicleFixtures.SpawnDesign(t.System, home, design.VehicleSaveData.RootPartInstance, firstId,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            other = VehicleFixtures.SpawnDesign(t.System, home, design.VehicleSaveData.RootPartInstance, otherId,
                OrbitFixtures.CircularAt(home, 600_000, Universe.GetElapsedTime()));

            if (prismaFixture != null)
            {
                CheckEngineGroupRoundTrip(t, craft, other, Save(temp, "prisma-engine-groups"), prisma: true);
                return;
            }

            CheckFreshCraftHasNoRecord(t);
            CheckSaveAndLoad(t, craft, other, Save(temp, "first"));
            CheckEngineGroupRoundTrip(t, craft, other, Save(temp, "engine-groups"));
            CheckBodyDefaults(t, craft, home, Save(temp, "luna"));
            CheckSecondSave(t, craft, Save(temp, "first"), Save(temp, "second"), Save(temp, "empty"));
            CheckRename(t, craft, renamedId, Save(temp, "renamed"));
            CheckOldRecords(t, craft, temp);
            CheckSaveEvents(t, craft, Save(temp, "events"));
            CheckOverlayPreferences(t);
        }
        finally
        {
            if (craft != null)
            {
                VehicleAutopilotState.Remove(craft);
                VehicleSpawner.Despawn(craft);
            }
            if (other != null)
            {
                VehicleAutopilotState.Remove(other);
                VehicleSpawner.Despawn(other);
            }
            if (Directory.Exists(temp))
                Directory.Delete(temp, recursive: true);
        }
    }

    private static DirectoryInfo Save(string root, string name)
    {
        var dir = new DirectoryInfo(Path.Combine(root, name));
        dir.Create();
        return dir;
    }

    private static string FilePath(DirectoryInfo save) => Path.Combine(save.FullName, CraftSettings.FileName);

    // The defaults are the initializers, so a craft nobody touched writes nothing.
    private static void CheckFreshCraftHasNoRecord(TestContext t)
    {
        List<(string Key, string Value)> record = CraftSettings.Record(new VehicleAutopilotState());
        t.Check("a fresh craft has no record", record.Count == 0, string.Join(", ", record.Select(r => r.Key)));
    }

    private static void SetPlayerValues(VehicleAutopilotState s)
    {
        s.SiteLatDeg = -12.5;
        s.SiteLonDeg = 47.25;
        s.LegsDeployBufferS = 2.5;
        s.LandingGimbalSteers = false;
        s.GfoldPointingDeg = 45.0;
        s.SixDofTiltDeg = 60.0;
        s.SixDofNodes = 30;
        s.PeKm = 250.0;
        s.AscentMethod = GuidanceWindow.AscentMethod.GravityTurn;
        s.MoonTransferId = "Luna";
        s.MoonTransferCapture = true;
        // Seeded from the launch site, not kept.
        s.LanDeg = 33.0;
        // Measured from the craft while the measurement is on, so not the player's value.
        s.VehicleHeightM = 23.0;
    }

    private static void CheckSaveAndLoad(TestContext t, Vehicle craft, Vehicle other, DirectoryInfo save)
    {
        VehicleAutopilotState.Remove(craft);
        VehicleAutopilotState.Remove(other);
        SetPlayerValues(VehicleAutopilotState.For(craft));
        // A craft whose console was drawn but whose values are the defaults.
        VehicleAutopilotState.For(other);

        int written = CraftSettings.WriteTo(save, Universe.CurrentSystem);
        string text = File.Exists(FilePath(save)) ? File.ReadAllText(FilePath(save)) : "";
        t.Check("one craft is written", written == 1, $"{written}");
        t.Check("the record names the craft", text.Contains("\"" + craft.Id + "\""), text);
        t.Check("a craft with defaults has no record", !text.Contains(other.Id), text);
        t.Check("the LAN is not kept", !text.Contains(nameof(VehicleAutopilotState.LanDeg)), text);
        t.Check("a measured value is not kept", !text.Contains(nameof(VehicleAutopilotState.VehicleHeightM) + " "), text);
        t.Check("the file is ASCII", text.All(c => c < 128));

        // A load makes new vehicle objects, so their state starts fresh.
        VehicleAutopilotState.Remove(craft);
        VehicleAutopilotState.Remove(other);
        int restored = CraftSettings.RestoreFrom(save, Universe.CurrentSystem);
        t.Check("one craft is restored", restored == 1, $"{restored}");
        if (!t.Check("the craft has a state after the load", VehicleAutopilotState.TryGet(craft, out VehicleAutopilotState s)))
            return;
        t.CheckAbs("site latitude", s.SiteLatDeg, -12.5, 1e-12);
        t.CheckAbs("site longitude", s.SiteLonDeg, 47.25, 1e-12);
        t.CheckAbs("legs buffer", s.LegsDeployBufferS, 2.5, 1e-12);
        t.Check("steer burn with gimbal", !s.LandingGimbalSteers);
        t.CheckAbs("G-FOLD max tilt", s.GfoldPointingDeg, 45.0, 1e-12);
        t.CheckAbs("touchdown max tilt", s.SixDofTiltDeg, 60.0, 1e-12);
        t.Check("solver points", s.SixDofNodes == 30, $"{s.SixDofNodes}");
        t.CheckAbs("periapsis", s.PeKm, 250.0, 1e-12);
        t.Check("ascent method", s.AscentMethod == GuidanceWindow.AscentMethod.GravityTurn);
        t.Check("THEN", s.MoonTransferId == "Luna" && s.MoonTransferCapture);
        t.CheckAbs("the LAN keeps its default", s.LanDeg, GuidanceSettingSpecs.LanDeg.Default, 1e-12);
        t.CheckAbs("the measured height keeps its default", s.VehicleHeightM, GuidanceSettingSpecs.VehicleHeightM.Default, 1e-12);
        t.Check("a load starts no mode", !s.Active && !s.EngagePending && !s.Running && !s.LaunchArmed && !s.ControlAcquired
            && s.LandingPhase == GuidanceWindow.LandingPhase.Idle && s.MoonChain == MoonChainStage.Idle);
        t.Check("a new craft keeps its defaults", !VehicleAutopilotState.TryGet(other, out _));
    }

    // The landing engine picture of the nine-engine bay: the center and one ring of eight on it, the upper stage engine off it, and balanced choices with the center alone recommended at an empty booster's mass.
    private static void CheckEnginePicture(TestContext t, Vehicle craft, Dictionary<string, EngineController?> candidates)
    {
        var layout = LandingEngineLayout.Build(craft, candidates, 101325.0, 9.81, 7000.0, double.NaN);
        LandingEngineDot[] plane = layout.Dots.Where(d => d.AftPlane).ToArray();
        float reach = plane.Length == 0 ? 0f : plane.Max(d => MathF.Sqrt(d.X * d.X + d.Y * d.Y));
        int center = plane.Count(d => MathF.Sqrt(d.X * d.X + d.Y * d.Y) < 0.15f * reach);
        t.Check("the engine picture holds the nine bay engines and not the upper stage engine",
            plane.Length == 9 && center == 1 && plane.All(d => d.Engine.Parent.FullPart.Template.Id == "AltairEngineA_Prefab_EngineBay"),
            $"{plane.Length} on the picture, {center} at the center, {layout.Dots.Length} in all");
        LandingEngineOption[] options = layout.Suggestions();
        string names = string.Join(", ", options.Select(o => $"{o.Name} {o.Stats.TwrMinDry:F2}-{o.Stats.TwrMaxLoaded:F2} {o.Stats.Verdict}{(o.Recommended ? " best" : "")}"));
        t.Check("the choices are the center, center and pair, and all nine, all balanced",
            options.Any(o => o.Name == "CENTER" && o.Stats.Engines == 1)
            && options.Any(o => o.Name == "CENTER + PAIR" && o.Stats.Engines == 3)
            && options.Any(o => o.Name == "ALL 9" && o.Stats.Engines == 9)
            && options.All(o => o.Stats.OffsetM <= LandingEngineLayout.BalancedOffsetM), names);
        t.Check("the center alone is recommended for an empty booster of 7 t", options.SingleOrDefault(o => o.Recommended)?.Name == "CENTER", names);
        string[] handPicked = plane.Where(d => d.Engine.TemplateId is "AltairCenter" or "AltairRing1" or "AltairRing5").Select(d => d.Key).ToArray();
        string[] lopsided = plane.Where(d => d.Engine.TemplateId is "AltairCenter" or "AltairRing1" or "AltairRing2").Select(d => d.Key).ToArray();
        t.Check("any opposite pair with the center is the CENTER + PAIR choice, a lopsided three is not",
            layout.Matching(options, handPicked)?.Name == "CENTER + PAIR" && layout.Matching(options, lopsided) == null,
            $"{layout.Matching(options, handPicked)?.Name ?? "none"}, {layout.Matching(options, lopsided)?.Name ?? "none"}");
    }

    private static void CheckEngineGroupRoundTrip(TestContext t, Vehicle craft, Vehicle rebuilt, DirectoryInfo save, bool prisma = false)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        var original = LandingEngineGroups.Controllers(craft, craft.Parts.Root);
        string[] keys;
        if (prisma)
        {
            var bay = original.Where(item => item.Value?.Parent.FullPart.Template.Id == "AltairEngineA_Prefab_EngineBay").ToArray();
            string[] expected = ["AltairCenter", .. Enumerable.Range(1, 8).Select(index => $"AltairRing{index}")];
            string[] names = bay.Select(item => item.Value!.TemplateId).ToArray();
            t.Check("nine independent Altair Bay addresses", bay.Length == 9 && names.ToHashSet(StringComparer.Ordinal).SetEquals(expected)
                && bay.All(item => item.Value!.Rockets.Length == 1), string.Join(", ", names));
            keys = bay.Where(item => item.Value?.TemplateId is "AltairCenter" or "AltairRing1")
                .OrderBy(item => item.Value!.TemplateId).Select(item => item.Key).ToArray();
            CheckEnginePicture(t, craft, original);
        }
        else
            keys = original.Where(item => item.Value != null).Select(item => item.Key).Take(2).ToArray();
        if (!t.Check("at least two distinct controller addresses", keys.Length == 2))
            return;

        var group = new LandingEngineGroup("Landing pair", keys);
        var single = new LandingEngineGroup("Center", [keys[0]]);
        state.EngineGroups[LandingEngineGroups.CraftScope] = new LandingEngineGroups([single, group], group.Name);
        t.Check("engine groups write with the vehicle", CraftSettings.WriteTo(save, Universe.CurrentSystem) == 1);
        VehicleAutopilotState.Remove(craft);
        t.Check("engine group vehicle restored", CraftSettings.RestoreFrom(save, Universe.CurrentSystem) == 1);
        LandingEngineGroups restored = VehicleAutopilotState.For(craft).GetEngineGroups(LandingEngineGroups.CraftScope);
        t.Check("shared membership and selected group round trip", restored.Groups.Length == 2
            && restored.Chosen is { } chosen && chosen.Name == group.Name && chosen.Engines.SequenceEqual(keys));

        var rebuiltControllers = LandingEngineGroups.Controllers(rebuilt, rebuilt.Parts.Root);
        EngineController[] members = [];
        if (t.Check("saved addresses resolve on a newly deserialized vehicle with new runtime IDs",
            restored.Chosen is { } selection && LandingEngineGroups.Resolve(selection, rebuiltControllers, out members)
            && members.Length == 2 && members[0].InstanceId != original[keys[0]]!.InstanceId))
        {
            LandingEngineGroups.Activate(rebuilt, members);
            InputEvents.ApplyInputEvents();
            t.Check("reload and apply the saved group activates exactly its two controllers",
                rebuilt.Parts.Modules.Get<EngineController>().ToArray().Where(engine => engine.IsActive).ToHashSet().SetEquals(members));
        }
        rebuiltControllers.Remove(keys[1]);
        t.Check("a missing member never silently shrinks a group",
            restored.Chosen is { } incomplete && !LandingEngineGroups.Resolve(incomplete, rebuiltControllers, out _));

        EngineController first = original[keys[0]]!, second = original[keys[1]]!;
        LandingEngineGroups.Activate(craft, [first]);
        InputEvents.ApplyInputEvents();
        t.Check("apply group leaves only the selected controller active",
            craft.Parts.Modules.Get<EngineController>().ToArray().Where(engine => engine.IsActive).SequenceEqual([first]));
        LandingEngineGroups.Activate(craft, [second], enableSingle: true);
        InputEvents.ApplyInputEvents();
        t.Check("manual enable keeps the other controller active", first.IsActive && second.IsActive);
        LandingEngineGroups.Activate(craft, [first], enableSingle: false);
        InputEvents.ApplyInputEvents();
        t.Check("manual disable affects only the selected controller", !first.IsActive && second.IsActive);
    }

    // A craft follows the defaults of the body it is at: a value still at the old body's default moves with it, one the player set stays, and a save measures its record against that body.
    private static void CheckBodyDefaults(TestContext t, Vehicle craft, IParentBody home, DirectoryInfo save)
    {
        const string Handover = nameof(VehicleAutopilotState.ConvexHandoverAltKm);
        BodyDefaults.Reset();
        VehicleAutopilotState.Remove(craft);
        VehicleAutopilotState s = VehicleAutopilotState.For(craft);
        s.ConvexQMaxKpa = 150.0;
        int moved = BodySettings.Follow(s, "Luna");
        t.Check("Luna's UPFG FROM and orbit are taken", moved == 3 && s.ConvexHandoverAltKm == 10.0 && s.PeKm == 30.0 && s.ApKm == 50.0 && s.DefaultsBody == "Luna", $"moved {moved}, {s.ConvexHandoverAltKm} km, {s.PeKm} x {s.ApKm} km, body '{s.DefaultsBody}'");
        t.CheckAbs("a value the player set stays", s.ConvexQMaxKpa, 150.0, 1e-12);
        t.Check("a value at Luna's default is not counted as changed", SettingAccess.CountChanged(s, GuidanceSettingSpecs.TuningAscent) == 0);
        List<(string Key, string Value)> record = CraftSettings.Record(s);
        string keys = string.Join(", ", record.Select(r => r.Key + "=" + r.Value));
        t.Check("a value at the body's default is not kept", !record.Any(r => r.Key == Handover), keys);
        t.Check("the record names its body", record.Any(r => r.Key == nameof(VehicleAutopilotState.DefaultsBody) && r.Value == "\"Luna\""), keys);

        // 80 km on Luna is the player's value, though it is the default of every other body.
        s.ConvexHandoverAltKm = 80.0;
        t.Check("80 km on Luna counts as changed", SettingAccess.CountChanged(s, GuidanceSettingSpecs.TuningAscent) == 1);
        CraftSettings.WriteTo(save, Universe.CurrentSystem);
        VehicleAutopilotState.Remove(craft);
        CraftSettings.RestoreFrom(save, Universe.CurrentSystem);
        if (!t.Check("the craft has a state after the load", VehicleAutopilotState.TryGet(craft, out s)))
            return;
        t.Check("the restored craft follows Luna", s.DefaultsBody == "Luna", s.DefaultsBody);
        t.CheckAbs("80 km comes back as the player's", s.ConvexHandoverAltKm, 80.0, 1e-12);
        BodySettings.Follow(s, craft);
        t.Check("the craft follows the body it is at", s.DefaultsBody == home.Id, s.DefaultsBody);
        t.CheckAbs("the player's 80 km stays at home", s.ConvexHandoverAltKm, 80.0, 1e-12);

        // A value at Luna's default goes back to the home default, and a reset on Luna writes Luna's.
        VehicleAutopilotState.Remove(craft);
        s = VehicleAutopilotState.For(craft);
        BodySettings.Follow(s, "Luna");
        BodySettings.Follow(s, craft);
        t.CheckAbs("Luna's default goes back to the home default", s.ConvexHandoverAltKm, GuidanceSettingSpecs.ConvexHandoverAltKm.Default, 1e-12);
        BodySettings.Follow(s, "Luna");
        s.ConvexHandoverAltKm = 33.0;
        SettingAccess.ResetToDefaults(s, GuidanceSettingSpecs.TuningAscent);
        t.CheckAbs("a reset on Luna writes Luna's default", s.ConvexHandoverAltKm, 10.0, 1e-12);
        VehicleAutopilotState.Remove(craft);
    }

    private static void CheckSecondSave(TestContext t, Vehicle craft, DirectoryInfo first, DirectoryInfo second, DirectoryInfo empty)
    {
        VehicleAutopilotState.Remove(craft);
        VehicleAutopilotState.For(craft).LegsDeployBufferS = 0.5;
        CraftSettings.WriteTo(first, Universe.CurrentSystem);
        VehicleAutopilotState.For(craft).LegsDeployBufferS = 3.0;
        CraftSettings.WriteTo(second, Universe.CurrentSystem);

        VehicleAutopilotState.Remove(craft);
        CraftSettings.RestoreFrom(first, Universe.CurrentSystem);
        t.CheckAbs("the first save keeps its value", VehicleAutopilotState.For(craft).LegsDeployBufferS, 0.5, 1e-12);

        VehicleAutopilotState.Remove(craft);
        CraftSettings.RestoreFrom(second, Universe.CurrentSystem);
        t.CheckAbs("the second save keeps its value", VehicleAutopilotState.For(craft).LegsDeployBufferS, 3.0, 1e-12);

        VehicleAutopilotState.Remove(craft);
        int restored = CraftSettings.RestoreFrom(empty, Universe.CurrentSystem);
        t.Check("a save without a file restores nothing", restored == 0 && !VehicleAutopilotState.TryGet(craft, out _));

        // Settings back to the defaults leave no file behind on the next write of the same save.
        VehicleAutopilotState.Remove(craft);
        VehicleAutopilotState.For(craft);
        int written = CraftSettings.WriteTo(second, Universe.CurrentSystem);
        t.Check("a save of defaults has no file", written == 0 && !File.Exists(FilePath(second)));
    }

    private static void CheckRename(TestContext t, Vehicle craft, string renamedId, DirectoryInfo save)
    {
        string oldId = craft.Id;
        VehicleAutopilotState.Remove(craft);
        VehicleAutopilotState.For(craft).GfoldPointingDeg = 35.0;
        if (!t.Check("the system accepts the new name", t.System.Rename(craft, renamedId)))
            return;
        CraftSettings.WriteTo(save, Universe.CurrentSystem);
        string text = File.ReadAllText(FilePath(save));
        t.Check("the record has the new name", text.Contains("\"" + renamedId + "\"") && !text.Contains("\"" + oldId + "\""), text);

        VehicleAutopilotState.Remove(craft);
        CraftSettings.RestoreFrom(save, Universe.CurrentSystem);
        t.CheckAbs("the renamed craft gets its value", VehicleAutopilotState.For(craft).GfoldPointingDeg, 35.0, 1e-12);
    }

    // A file of another version: unknown keys, values out of range or unreadable, a table AFC does not know, a craft without an id, and other formats.
    private static void CheckOldRecords(TestContext t, Vehicle craft, string root)
    {
        DirectoryInfo save = Save(root, "old");
        string id = craft.Id;
        File.WriteAllLines(FilePath(save), new[]
        {
            "# written by another version",
            "format = 1",
            "",
            "[[craft]]",
            "id = \"" + id + "\"",
            "SiteLonDeg = 10.5 # a comment",
            "OldSetting = 5",
            "GfoldPointingDeg = 900",
            "LegsAutoDeploy = maybe",
            "AscentMethod = \"Hopper\"",
            "MoonTransferCapture = true",
            "LegsDeployBufferS",
            "LegsDeployHeightM = 500",
            "",
            "[[future]]",
            "SiteLatDeg = 5",
            "",
            "[[craft]]",
            "SiteLatDeg = 7",
        });
        VehicleAutopilotState.Remove(craft);
        int restored = -1;
        Exception? error = null;
        try
        {
            restored = CraftSettings.RestoreFrom(save, Universe.CurrentSystem);
        }
        catch (Exception ex)
        {
            error = ex;
        }
        t.Check("an old record loads without an error", error == null && restored == 1, error?.ToString() ?? $"{restored}");
        VehicleAutopilotState s = VehicleAutopilotState.For(craft);
        t.CheckAbs("a known value is taken", s.SiteLonDeg, 10.5, 1e-12);
        t.CheckAbs("a value of another table is not taken", s.SiteLatDeg, GuidanceSettingSpecs.SiteLatDeg.Default, 1e-12);
        t.CheckAbs("a value out of range is skipped", s.GfoldPointingDeg, GuidanceSettingSpecs.GfoldPointingDeg.Default, 1e-12);
        t.Check("an unreadable switch is skipped", s.LegsAutoDeploy == GuidanceSettingSpecs.LegsAutoDeploy.Default);
        t.CheckAbs("a legs height of an older version leaves the default buffer", s.LegsDeployBufferS, GuidanceSettingSpecs.LegsDeployBufferS.Default, 1e-12);
        t.Check("an unknown ascent method is skipped", s.AscentMethod == GuidanceWindow.AscentMethod.CatsUpfg);
        t.Check("a capture without a moon is skipped", !s.MoonTransferCapture && s.MoonTransferId.Length == 0);

        // A record that names its body and still holds the legs height of an older version takes the body and the default buffer.
        BodyDefaults.Reset();
        File.WriteAllLines(FilePath(save), new[]
        {
            "format = 1",
            "[[craft]]",
            "id = \"" + id + "\"",
            "DefaultsBody = \"Luna\"",
            "LegsDeployHeightM = 500",
            "SiteLonDeg = 12.5",
        });
        VehicleAutopilotState.Remove(craft);
        error = null;
        try
        {
            restored = CraftSettings.RestoreFrom(save, Universe.CurrentSystem);
        }
        catch (Exception ex)
        {
            error = ex;
        }
        t.Check("a record with a body and a legs height loads without an error", error == null && restored == 1, error?.ToString() ?? $"{restored}");
        s = VehicleAutopilotState.For(craft);
        t.Check("the record with the legs height follows its body", s.DefaultsBody == "Luna" && s.ConvexHandoverAltKm == 10.0, $"body '{s.DefaultsBody}', {s.ConvexHandoverAltKm} km");
        t.CheckAbs("the record with the legs height keeps its other values", s.SiteLonDeg, 12.5, 1e-12);
        t.CheckAbs("the record with the legs height gets the default buffer", s.LegsDeployBufferS, GuidanceSettingSpecs.LegsDeployBufferS.Default, 1e-12);

        foreach (string format in new[] { "", "format = 0", "format = 2" })
        {
            var lines = new List<string>();
            if (format.Length > 0)
                lines.Add(format);
            lines.AddRange(new[] { "[[craft]]", "id = \"" + id + "\"", "SiteLonDeg = 11.5" });
            File.WriteAllLines(FilePath(save), lines);
            VehicleAutopilotState.Remove(craft);
            error = null;
            try
            {
                restored = CraftSettings.RestoreFrom(save, Universe.CurrentSystem);
            }
            catch (Exception ex)
            {
                error = ex;
            }
            string label = format.Length > 0 ? format : "no format";
            t.Check("a file with " + label + " restores nothing and raises no error",
                error == null && restored == 0 && !VehicleAutopilotState.TryGet(craft, out _), error?.ToString() ?? $"{restored}");
        }
    }

    // The save patches hand the folder of the save to their subscribers: the write postfix after a completed write, the load postfix after a completed load.
    private static void CheckSaveEvents(TestContext t, Vehicle craft, DirectoryInfo save)
    {
        FieldInfo saveIdField = StaticField(typeof(SaveLoadObserver), "<CurrentSaveId>k__BackingField");
        FieldInfo saveDirField = StaticField(typeof(SaveLoadObserver), "<CurrentSaveDirectory>k__BackingField");
        FieldInfo loadedEvent = StaticField(typeof(SaveLoadObserver), "SaveLoaded");
        FieldInfo writtenEvent = StaticField(typeof(SaveLoadObserver), "SaveWritten");
        FieldInfo configPath = StaticField(typeof(MultiPassRegistry), "_configPath");
        FieldInfo modDir = StaticField(typeof(MultiPassRegistry), "_modDir");
        object? oldSaveId = saveIdField.GetValue(null);
        object? oldSaveDir = saveDirField.GetValue(null);
        object? oldLoaded = loadedEvent.GetValue(null);
        object? oldWritten = writtenEvent.GetValue(null);
        object? oldPath = configPath.GetValue(null);
        object? oldDir = modDir.GetValue(null);
        var oldEntries = new List<MultiPassExecution>(MultiPassRegistry.Snapshot.Values);
        try
        {
            configPath.SetValue(null, Path.Combine(save.FullName, "multipass.toml"));
            modDir.SetValue(null, save.FullName);
            loadedEvent.SetValue(null, null);
            writtenEvent.SetValue(null, null);
            SaveLoadObserver.SaveWritten += CraftSettings.OnSaveWritten;
            SaveLoadObserver.SaveLoaded += CraftSettings.OnSaveLoaded;
            // The write postfix rekeys the live multi-pass entries in place, so the registry is emptied first and its old entries stay on their own save.
            MultiPassRegistry.Reset();

            VehicleAutopilotState.Remove(craft);
            VehicleAutopilotState.For(craft).LegsDeployBufferS = 1.5;
            UncompressedSave written = MakeSave("events", save);
            InvokePatch("WritePatch", "Postfix", [written, false]);
            t.Check("a failed write writes no settings", !File.Exists(FilePath(save)));
            InvokePatch("WritePatch", "Postfix", [written, true]);
            t.Check("a completed write puts the settings in the save folder", File.Exists(FilePath(save)));

            VehicleAutopilotState.Remove(craft);
            UncompressedSave loaded = MakeSave("events", save);
            loaded.UniverseData = new UniverseData();
            object?[] prefixArgs = [loaded, null];
            InvokePatch("LoadPatch", "Prefix", prefixArgs);
            InvokePatch("LoadPatch", "Postfix", [loaded, prefixArgs[1]]);
            t.Check("a refused load restores nothing", !VehicleAutopilotState.TryGet(craft, out _));
            prefixArgs = [loaded, null];
            InvokePatch("LoadPatch", "Prefix", prefixArgs);
            loaded.UniverseData = new UniverseData();
            InvokePatch("LoadPatch", "Postfix", [loaded, prefixArgs[1]]);
            t.Check("a completed load restores the settings", VehicleAutopilotState.TryGet(craft, out VehicleAutopilotState s)
                && Math.Abs(s.LegsDeployBufferS - 1.5) < 1e-12);
        }
        finally
        {
            configPath.SetValue(null, oldPath);
            modDir.SetValue(null, oldDir);
            loadedEvent.SetValue(null, oldLoaded);
            writtenEvent.SetValue(null, oldWritten);
            saveIdField.SetValue(null, oldSaveId);
            saveDirField.SetValue(null, oldSaveDir);
            MultiPassRegistry.Reset();
            foreach (MultiPassExecution entry in oldEntries) MultiPassRegistry.Add(entry);
        }
    }

    // The SHOW IN MAP and SHOW IN WORLD switches are player preferences in ui.toml, read when the console loads its preferences and written when one changes.
    private static void CheckOverlayPreferences(TestContext t)
    {
        string[] keys = ["guidance_ascent_in_map", "guidance_impact_in_world", "guidance_gfold_in_world", "guidance_touchdown_in_world"];
        string[] fields = ["_showAscentOverlay", "_showImpactOverlay", "_showGfoldOverlay", "_show6DofOverlay"];
        FieldInfo[] infos = fields.Select(f => StaticField(typeof(GuidanceWindow), f)).ToArray();
        object?[] oldFields = infos.Select(f => f.GetValue(null)).ToArray();
        bool[] hadKeys = keys.Select(UiConfig.Has).ToArray();
        bool[] oldValues = keys.Select(k => UiConfig.GetBool(k, false)).ToArray();
        MethodInfo save = AccessTools.Method(typeof(GuidanceWindow), "SaveOverlayPreferences")
            ?? throw new MissingMethodException(typeof(GuidanceWindow).FullName, "SaveOverlayPreferences");
        try
        {
            bool[] stored = [false, true, true, true];
            for (int i = 0; i < keys.Length; i++)
                UiConfig.Set(keys[i], stored[i]);
            GuidanceWindow.LoadOverlayPreferences();
            t.Check("the overlay switches are read from ui.toml",
                Enumerable.Range(0, keys.Length).All(i => (bool)infos[i].GetValue(null)! == stored[i]));

            for (int i = 0; i < keys.Length; i++)
                infos[i].SetValue(null, !stored[i]);
            save.Invoke(null, null);
            t.Check("a changed overlay switch is written to ui.toml",
                Enumerable.Range(0, keys.Length).All(i => UiConfig.GetBool(keys[i], stored[i]) == !stored[i]));

            for (int i = 0; i < keys.Length; i++)
                UiConfig.Remove(keys[i]);
            GuidanceWindow.LoadOverlayPreferences();
            t.Check("without a preference the overlays keep their defaults",
                (bool)infos[0].GetValue(null)! && !(bool)infos[1].GetValue(null)! && !(bool)infos[2].GetValue(null)! && !(bool)infos[3].GetValue(null)!);
        }
        finally
        {
            for (int i = 0; i < keys.Length; i++)
            {
                infos[i].SetValue(null, oldFields[i]);
                if (hadKeys[i])
                    UiConfig.Set(keys[i], oldValues[i]);
                else
                    UiConfig.Remove(keys[i]);
            }
        }
    }

    // Both constructors touch the save folder of the game, and the patches need only the id, the folder and the universe data.
    private static UncompressedSave MakeSave(string id, DirectoryInfo folder)
    {
        var save = (UncompressedSave)RuntimeHelpers.GetUninitializedObject(typeof(UncompressedSave));
        FieldInfo idField = typeof(GameSave).GetField("<Id>k__BackingField", BindingFlags.Instance | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(typeof(GameSave).FullName, "<Id>k__BackingField");
        idField.SetValue(save, id);
        FieldInfo dirField = typeof(UncompressedSave).GetField(nameof(UncompressedSave.Directory), BindingFlags.Instance | BindingFlags.Public)
            ?? throw new MissingFieldException(typeof(UncompressedSave).FullName, nameof(UncompressedSave.Directory));
        dirField.SetValue(save, folder);
        return save;
    }

    private static void InvokePatch(string patch, string method, object?[] args)
    {
        MethodInfo target = AccessTools.Method(AccessTools.Inner(typeof(SaveLoadObserver), patch), method)
            ?? throw new MissingMethodException(typeof(SaveLoadObserver).FullName, patch + "." + method);
        target.Invoke(null, args);
    }

    private static FieldInfo StaticField(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.NonPublic)
           ?? throw new MissingFieldException(type.FullName, name);
}
