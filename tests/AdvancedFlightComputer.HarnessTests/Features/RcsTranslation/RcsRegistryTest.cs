using System.Reflection;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Persistence checks use isolated dictionaries and a unique temporary directory.
public sealed class RcsRegistryTest : AfcTest
{
    public override string Name => "afc-rcs-registry";

    protected override void Execute(TestContext t)
    {
        CheckSaveScopeChanges(t);
        CheckAtomicWrite(t);
        CheckTomlRoundTrip(t);
        CheckOptionsKeying(t);
        CheckUnknownEnumFallback(t);
        CheckStructuralParseFailures(t);
        CheckFailedLoadPreservesState(t);
        CheckInvalidActiveState(t);
    }

    private static RcsExecution Entry(string saveId, string vehicleId)
    {
        var execution = new RcsExecution { SaveId = saveId, VehicleId = vehicleId };
        execution.GetOrCreateOptions(100.0, 1.0).Mode = RcsExecutionMode.Rcs;
        return execution;
    }

    private static void CheckSaveScopeChanges(TestContext t)
    {
        RcsExecution source = Entry("", "ship");
        RcsExecution other = Entry("other", "ship");
        var entries = new Dictionary<(string SaveId, string VehicleId), RcsExecution>
        {
            [("", "ship")] = source,
            [("first", "stale")] = Entry("first", "stale"),
            [("other", "ship")] = other,
        };
        RcsExecRegistry.RekeyEntries(entries, "", "first");
        t.Check("first save promotes transient entry", source.SaveId == "first"
            && entries.TryGetValue(("first", "ship"), out var moved) && ReferenceEquals(source, moved));
        t.Check("first save removes previous destination entries", !entries.ContainsKey(("first", "stale")));
        RcsExecRegistry.RekeyEntries(entries, "first", "save-as");
        t.Check("Save-As moves source", source.SaveId == "save-as" && !entries.ContainsKey(("first", "ship")));
        entries[("overwrite", "old-ship")] = Entry("overwrite", "old-ship");
        RcsExecRegistry.RekeyEntries(entries, "save-as", "overwrite");
        t.Check("overwrite removes destination and moves source", source.SaveId == "overwrite"
            && !entries.ContainsKey(("overwrite", "old-ship")) && !entries.ContainsKey(("save-as", "ship")));
        t.Check("unrelated save preserved", ReferenceEquals(entries[("other", "ship")], other));
        RcsExecRegistry.RekeyEntries(entries, "overwrite", "overwrite");
        RcsExecRegistry.RekeyEntries(entries, "overwrite", "");
        t.Check("same or empty destination leaves state intact", entries.Count == 2
            && ReferenceEquals(entries[("overwrite", "ship")], source));
    }

    private static void CheckAtomicWrite(TestContext t)
    {
        string directory = Path.Combine(Path.GetTempPath(), "afc-rcs-registry-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        string path = Path.Combine(directory, "state.toml");
        try
        {
            RcsExecution first = Entry("first", "ship");
            RcsExecRegistry.WriteFile(path, [first]);
            string original = File.ReadAllText(path);
            bool failed = false;
            try
            {
                RcsExecRegistry.WriteFile(path, FailingEntries(first));
            }
            catch (InvalidOperationException)
            {
                failed = true;
            }
            t.Check("failed write keeps previous file", failed && File.ReadAllText(path) == original);
            t.Check("failed write removes its temporary file", Directory.GetFiles(directory, "*.tmp").Length == 0);

            RcsExecRegistry.WriteFile(path, InterleavedEntries(path, first));
            t.Check("overlapping writes keep separate temporary files", File.ReadAllText(path) == original);
            t.Check("successful write removes its temporary file", Directory.GetFiles(directory, "*.tmp").Length == 0);
        }
        finally
        {
            // Only this test creates files in its unique temporary directory.
            foreach (string file in Directory.GetFiles(directory))
                File.Delete(file);
            Directory.Delete(directory);
        }
    }

    private static IEnumerable<RcsExecution> FailingEntries(RcsExecution entry)
    {
        yield return entry;
        throw new InvalidOperationException("Test serialization failure");
    }

    private static IEnumerable<RcsExecution> InterleavedEntries(string path, RcsExecution entry)
    {
        yield return entry;
        RcsExecRegistry.WriteFile(path, [Entry("second", "other-ship")]);
    }

    private static void CheckTomlRoundTrip(TestContext t)
    {
        // A vehicle id with an embedded quote is the case a naive parser
        // truncates; the active block carries every resolved field.
        var exec = new RcsExecution { SaveId = "save-1", VehicleId = "veh \"q\" id" };
        exec.Options.Add(new RcsBurnOptions
        {
            BurnTimeSec = 1234.5,
            BurnDvMs = 0.5,
            Mode = RcsExecutionMode.Rcs,
            Attitude = RcsAttitudeStrategy.Align,
            Allocator = RcsAllocator.Lp,
        });
        exec.Options.Add(new RcsBurnOptions
        {
            BurnTimeSec = 6789.0,
            BurnDvMs = 2.0,
            Mode = RcsExecutionMode.Default,
            Attitude = RcsAttitudeStrategy.Hold,
            Allocator = RcsAllocator.Groups,
        });
        exec.ActiveBurnTimeSec = 1234.5;
        exec.ActiveBurnDvMs = 0.5;
        exec.ResolvedStrategy = RcsAttitudeStrategy.Align;
        exec.ResolvedAxis = 3;
        exec.ResolvedAllocator = RcsAllocator.Lp;
        exec.AlignCommanded = true;
        exec.ForcedRcsOn = true;

        var writer = new StringWriter();
        RcsExecRegistry.WriteToml(writer, new[] { exec });
        string[] lines = writer.ToString().Split('\n');

        var parsed = new Dictionary<(string SaveId, string VehicleId), RcsExecution>();
        bool parsedCleanly = RcsExecRegistry.ParseLines(lines, "round-trip", parsed);
        t.Check("RCS round-trip parse succeeds", parsedCleanly);

        t.Check("single entry", parsed.Count == 1);
        if (!parsed.TryGetValue(("save-1", "veh \"q\" id"), out RcsExecution? back))
        {
            t.Fail("escaped key", "parsed keys did not include the quoted id");
            return;
        }

        if (t.Check("two options", back.Options.Count == 2))
        {
            RcsBurnOptions a = back.Options[0];
            Near(t, "opt0 time", a.BurnTimeSec, 1234.5);
            Near(t, "opt0 dv", a.BurnDvMs, 0.5);
            t.Check("opt0 mode", a.Mode == RcsExecutionMode.Rcs);
            t.Check("opt0 attitude", a.Attitude == RcsAttitudeStrategy.Align);
            t.Check("opt0 allocator", a.Allocator == RcsAllocator.Lp);

            RcsBurnOptions b = back.Options[1];
            Near(t, "opt1 time", b.BurnTimeSec, 6789.0);
            Near(t, "opt1 dv", b.BurnDvMs, 2.0);
            t.Check("opt1 mode", b.Mode == RcsExecutionMode.Default);
            t.Check("opt1 attitude", b.Attitude == RcsAttitudeStrategy.Hold);
            t.Check("opt1 allocator", b.Allocator == RcsAllocator.Groups);
        }

        t.Check("active time restored", back.ActiveBurnTimeSec.HasValue
            && Math.Abs(back.ActiveBurnTimeSec.Value - 1234.5) < 1e-6);
        t.Check("active dv restored", back.ActiveBurnDvMs.HasValue
            && Math.Abs(back.ActiveBurnDvMs.Value - 0.5) < 1e-6);
        t.Check("resolved strategy", back.ResolvedStrategy == RcsAttitudeStrategy.Align);
        t.Check("resolved axis", back.ResolvedAxis == 3);
        t.Check("resolved allocator", back.ResolvedAllocator == RcsAllocator.Lp);
        t.Check("align commanded", back.AlignCommanded);
        t.Check("forced rcs on", back.ForcedRcsOn);
    }

    private static void CheckOptionsKeying(TestContext t)
    {
        var exec = new RcsExecution { SaveId = "s", VehicleId = "v" };

        RcsBurnOptions o1 = exec.GetOrCreateOptions(100.0, 5.0);
        t.Check("default allocator is Groups", o1.Allocator == RcsAllocator.Groups);
        // A small nudge (inside the match tolerance) must return the SAME
        // options instance and follow the burn to its new time, not orphan.
        RcsBurnOptions o1b = exec.GetOrCreateOptions(100.02, 5.0);
        t.Check("nudge keeps instance", ReferenceEquals(o1, o1b));
        Near(t, "nudge updates key", o1.BurnTimeSec, 100.02);
        t.Check("no duplicate on nudge", exec.Options.Count == 1);

        // A far burn is a new option, not a re-key of the first.
        RcsBurnOptions o2 = exec.GetOrCreateOptions(200.0, 5.0);
        t.Check("far burn is new", !ReferenceEquals(o1, o2));
        t.Check("two options tracked", exec.Options.Count == 2);

        t.Check("find within tolerance", ReferenceEquals(exec.FindOptions(100.03, 5.0), o1));
        t.Check("find misses far", exec.FindOptions(150.0, 5.0) == null);

        // Match tolerances: 0.05 s and 0.1 m/s, exclusive at the bound.
        var o = new RcsBurnOptions { BurnTimeSec = 100.0, BurnDvMs = 5.0 };
        t.Check("time inside", o.Matches(100.04, 5.0));
        t.Check("time outside", !o.Matches(100.06, 5.0));
        t.Check("dv inside", o.Matches(100.0, 5.05));
        t.Check("dv outside", !o.Matches(100.0, 5.2));
    }

    // A present-but-unrecognised enum token (a value renamed or removed by a
    // mod update, or an out-of-range ordinal) must fall back to the field's
    // default and keep the block, not drop it or carry an undefined enum.
    private static void CheckUnknownEnumFallback(TestContext t)
    {
        string[] lines =
        {
            "[[rcs_burn]]",
            "save_id = \"s\"",
            "vehicle_id = \"v\"",
            "burn_time_sec = 10",
            "burn_dv_ms = 1",
            "mode = \"Rcs\"",          // valid, preserved
            "attitude = \"Sideways\"", // no such value -> Auto
            "allocator = \"7\"",       // out-of-range ordinal -> Groups (the default)
        };
        var parsed = new Dictionary<(string SaveId, string VehicleId), RcsExecution>();
        bool parsedCleanly = RcsExecRegistry.ParseLines(lines, "unknown-enum", parsed);
        t.Check("unknown option enums recover safely", parsedCleanly);

        if (!t.Check("unknown-enum block kept",
                parsed.TryGetValue(("s", "v"), out RcsExecution? e) && e.Options.Count == 1))
            return;

        RcsBurnOptions o = parsed[("s", "v")].Options[0];
        t.Check("valid mode preserved", o.Mode == RcsExecutionMode.Rcs);
        t.Check("bad attitude falls back", o.Attitude == RcsAttitudeStrategy.Auto);
        t.Check("out-of-range allocator falls back", o.Allocator == RcsAllocator.Groups);
    }

    private static void CheckStructuralParseFailures(TestContext t)
    {
        var parsed = new Dictionary<(string SaveId, string VehicleId), RcsExecution>();
        t.Check("unknown RCS header fails parse",
            !RcsExecRegistry.ParseLines(["[broken]"], "bad-header", parsed));
        parsed.Clear();
        t.Check("invalid RCS assignment fails parse",
            !RcsExecRegistry.ParseLines(
                ["[[rcs_burn]]", "not an assignment"], "bad-assignment", parsed));
        parsed.Clear();
        t.Check("missing RCS fields fail parse",
            !RcsExecRegistry.ParseLines(
                ["[[rcs_burn]]", "save_id = \"broken\""], "missing-fields", parsed));
    }

    private static void CheckFailedLoadPreservesState(TestContext t)
    {
        FieldInfo configPath = Field(typeof(RcsExecRegistry), "_configPath");
        FieldInfo entriesField = Field(typeof(RcsExecRegistry), "_byKey");
        object? oldPath = configPath.GetValue(null);
        var entries = (Dictionary<(string SaveId, string VehicleId), RcsExecution>)entriesField.GetValue(null)!;
        var oldEntries = new Dictionary<(string SaveId, string VehicleId), RcsExecution>(entries);
        string temp = Path.Combine(Path.GetTempPath(), "afc-rcs-load-" + Guid.NewGuid().ToString("N"));
        string file = Path.Combine(temp, "rcs-exec.toml");
        Directory.CreateDirectory(temp);
        try
        {
            RcsExecRegistry.WriteFile(file, [Entry("disk", "disk-vehicle")]);
            configPath.SetValue(null, file);
            entries.Clear();
            RcsExecution live = Entry("live", "live-vehicle");
            entries[("live", "live-vehicle")] = live;

            using (new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.None))
                RcsExecRegistry.Load();

            t.Check("failed RCS load preserves live state",
                entries.TryGetValue(("live", "live-vehicle"), out RcsExecution? preserved)
                && ReferenceEquals(preserved, live)
                && !entries.ContainsKey(("disk", "disk-vehicle")));

            File.Delete(file);
            RcsExecRegistry.Load();
            t.Check("missing RCS file preserves live state",
                entries.TryGetValue(("live", "live-vehicle"), out preserved)
                && ReferenceEquals(preserved, live));

            RcsExecRegistry.WriteFile(file, [Entry("disk", "disk-vehicle")]);
            File.AppendAllLines(file, ["[[rcs_burn]]", "save_id = \"broken\""]);
            RcsExecRegistry.Load();
            t.Check("partial RCS parse keeps the readable blocks",
                entries.ContainsKey(("disk", "disk-vehicle"))
                && !entries.ContainsKey(("live", "live-vehicle")));

            // An empty save must not erase a file that did not load.
            File.WriteAllLines(file, ["[[rcs_burn]]", "save_id = \"broken\""]);
            string defective = File.ReadAllText(file);
            entries.Clear();
            RcsExecRegistry.Load();
            t.Check("a wholly unreadable RCS file loads nothing", entries.Count == 0);
            RcsExecRegistry.Save();
            t.Check("an empty RCS save cannot overwrite an unreadable file",
                File.ReadAllText(file) == defective);

            RcsExecRegistry.WriteFile(file, [Entry("disk", "disk-vehicle")]);
            entries.Clear();
            entries[("live", "live-vehicle")] = live;
            RcsExecRegistry.Load();
            t.Check("successful RCS load replaces live state",
                entries.ContainsKey(("disk", "disk-vehicle"))
                && !entries.ContainsKey(("live", "live-vehicle")));

            entries.Clear();
            RcsExecRegistry.Save();
            t.Check("an empty RCS save clears a file that read cleanly",
                !File.ReadAllText(file).Contains("[[rcs_burn]]", StringComparison.Ordinal));
        }
        finally
        {
            configPath.SetValue(null, oldPath);
            entries.Clear();
            foreach (var entry in oldEntries) entries.Add(entry.Key, entry.Value);
            if (Directory.Exists(temp)) Directory.Delete(temp, recursive: true);
        }
    }

    private static void CheckInvalidActiveState(TestContext t)
    {
        CheckRejectedActive(t, "undefined strategy", "resolved_strategy = \"99\"");
        CheckRejectedActive(t, "undefined allocator", "resolved_allocator = \"99\"");
        CheckRejectedActive(t, "undefined mode", "mode = \"99\"");
        CheckRejectedActive(t, "unresolved strategy", "resolved_strategy = \"Auto\"");
        CheckRejectedActive(t, "engine mode", "mode = \"Engine\"");
        CheckRejectedActive(t, "allocator mismatch", "resolved_allocator = \"Lp\"");
        CheckRejectedActive(t, "Align axis outside range", "resolved_axis = 99");
        CheckRejectedActive(t, "Hold axis mismatch",
            "resolved_strategy = \"Hold\"", "resolved_axis = 0");
        CheckRejectedActive(t, "Hold command mismatch",
            "resolved_strategy = \"Hold\"", "resolved_axis = -1", "align_commanded = true");
        CheckRejectedActive(t, "missing ownership flag", "forced_rcs_on =");
        CheckAcceptedActive(t, "Align after Hold fallback",
            "attitude = \"Hold\"", "resolved_strategy = \"Align\"", "resolved_axis = 3");
        CheckAcceptedActive(t, "Hold after Align fallback",
            "attitude = \"Align\"", "resolved_strategy = \"Hold\"", "resolved_axis = -1");
    }

    private static void CheckRejectedActive(TestContext t, string label, params string[] replacements)
    {
        RcsExecution? exec = ParseActive(replacements);
        bool keptOption = exec?.Options.Count == 1;
        t.Check(label + " keeps safe option", keptOption);
        t.Check(label + " does not restore activity", keptOption && !exec!.IsActive);
    }

    private static void CheckAcceptedActive(TestContext t, string label, params string[] replacements)
    {
        RcsExecution? exec = ParseActive(replacements);
        t.Check(label + " restores activity", exec is { IsActive: true });
    }

    private static RcsExecution? ParseActive(params string[] replacements)
    {
        string[] lines =
        {
            "[[rcs_burn]]",
            "save_id = \"s\"",
            "vehicle_id = \"v\"",
            "burn_time_sec = 10",
            "burn_dv_ms = 1",
            "mode = \"Rcs\"",
            "attitude = \"Align\"",
            "allocator = \"Groups\"",
            "active = true",
            "resolved_strategy = \"Align\"",
            "resolved_axis = 3",
            "resolved_allocator = \"Groups\"",
            "align_commanded = false",
            "forced_rcs_on = false",
        };
        foreach (string replacement in replacements)
        {
            int separator = replacement.IndexOf('=');
            string key = separator >= 0 ? replacement.Substring(0, separator).Trim() : replacement;
            int index = Array.FindIndex(lines,
                line => line.StartsWith(key + " =", StringComparison.Ordinal));
            if (index >= 0)
                lines[index] = replacement;
        }

        var parsed = new Dictionary<(string SaveId, string VehicleId), RcsExecution>();
        if (!RcsExecRegistry.ParseLines(lines, "invalid-active", parsed))
            return null;
        return parsed.GetValueOrDefault(("s", "v"));
    }

    private static FieldInfo Field(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.NonPublic)
           ?? throw new MissingFieldException(type.FullName, name);

    private static void Near(TestContext t, string label, double actual, double expected)
        => t.CheckRel(label, actual, expected, 1e-6, floor: 1.0);
}
