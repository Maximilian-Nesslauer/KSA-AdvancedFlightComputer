using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Pure persistence tests for the RCS registry: the TOML write/parse round-trip
// (including the escape-aware quoting that keeps a vehicle id containing a
// double quote from mis-keying, and the active-execution fields) plus the
// per-burn options keying that follows a burn as the user nudges it. No
// vehicle and no filesystem: WriteToml/ParseLines are exercised directly
// through in-memory buffers.
public sealed class RcsRegistryTest : AfcTest
{
    public override string Name => "afc-rcs-registry";

    protected override void Execute(TestContext t)
    {
        CheckTomlRoundTrip(t);
        CheckOptionsKeying(t);
        CheckUnknownEnumFallback(t);
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
        RcsExecRegistry.ParseLines(lines, "round-trip", parsed);

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
        RcsExecRegistry.ParseLines(lines, "unknown-enum", parsed);

        if (!t.Check("unknown-enum block kept",
                parsed.TryGetValue(("s", "v"), out RcsExecution? e) && e.Options.Count == 1))
            return;

        RcsBurnOptions o = parsed[("s", "v")].Options[0];
        t.Check("valid mode preserved", o.Mode == RcsExecutionMode.Rcs);
        t.Check("bad attitude falls back", o.Attitude == RcsAttitudeStrategy.Auto);
        t.Check("out-of-range allocator falls back", o.Allocator == RcsAllocator.Groups);
    }

    private static void Near(TestContext t, string label, double actual, double expected)
        => t.CheckRel(label, actual, expected, 1e-6, floor: 1.0);
}
