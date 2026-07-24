using AdvancedFlightComputer.Features.RcsTranslation;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

// Pure persistence tests for the RCS registry: the TOML write/parse round-trip
// (including the escape-aware quoting that keeps a vehicle id containing a
// double quote from mis-keying, and the active-execution fields) plus the
// per-burn options keying that follows a burn as the user nudges it. No
// vehicle and no filesystem: WriteToml/ParseLines are exercised directly
// through in-memory buffers.
public sealed class RcsRegistryTest : IHarnessTest
{
    public string Name => "afc-rcs-registry";

    public int Run(HeadlessSession session)
    {
        bool ok = true;
        ok &= CheckTomlRoundTrip();
        ok &= CheckOptionsKeying();
        ok &= CheckUnknownEnumFallback();
        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckTomlRoundTrip()
    {
        bool ok = true;

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

        ok &= Check("single entry", parsed.Count == 1);
        if (!parsed.TryGetValue(("save-1", "veh \"q\" id"), out RcsExecution? back))
        {
            HarnessLog.Line($"[{Name}] TEST escaped key: parsed keys did not include the quoted id => FAIL");
            return false;
        }

        ok &= Check("two options", back.Options.Count == 2);
        if (back.Options.Count == 2)
        {
            RcsBurnOptions a = back.Options[0];
            ok &= Near("opt0 time", a.BurnTimeSec, 1234.5);
            ok &= Near("opt0 dv", a.BurnDvMs, 0.5);
            ok &= Check("opt0 mode", a.Mode == RcsExecutionMode.Rcs);
            ok &= Check("opt0 attitude", a.Attitude == RcsAttitudeStrategy.Align);
            ok &= Check("opt0 allocator", a.Allocator == RcsAllocator.Lp);

            RcsBurnOptions b = back.Options[1];
            ok &= Near("opt1 time", b.BurnTimeSec, 6789.0);
            ok &= Near("opt1 dv", b.BurnDvMs, 2.0);
            ok &= Check("opt1 mode", b.Mode == RcsExecutionMode.Default);
            ok &= Check("opt1 attitude", b.Attitude == RcsAttitudeStrategy.Hold);
            ok &= Check("opt1 allocator", b.Allocator == RcsAllocator.Groups);
        }

        ok &= Check("active time restored", back.ActiveBurnTimeSec.HasValue
            && Math.Abs(back.ActiveBurnTimeSec.Value - 1234.5) < 1e-6);
        ok &= Check("active dv restored", back.ActiveBurnDvMs.HasValue
            && Math.Abs(back.ActiveBurnDvMs.Value - 0.5) < 1e-6);
        ok &= Check("resolved strategy", back.ResolvedStrategy == RcsAttitudeStrategy.Align);
        ok &= Check("resolved axis", back.ResolvedAxis == 3);
        ok &= Check("resolved allocator", back.ResolvedAllocator == RcsAllocator.Lp);
        ok &= Check("align commanded", back.AlignCommanded);
        ok &= Check("forced rcs on", back.ForcedRcsOn);
        return ok;
    }

    private bool CheckOptionsKeying()
    {
        bool ok = true;
        var exec = new RcsExecution { SaveId = "s", VehicleId = "v" };

        RcsBurnOptions o1 = exec.GetOrCreateOptions(100.0, 5.0);
        ok &= Check("default allocator is Groups", o1.Allocator == RcsAllocator.Groups);
        // A small nudge (inside the match tolerance) must return the SAME
        // options instance and follow the burn to its new time, not orphan.
        RcsBurnOptions o1b = exec.GetOrCreateOptions(100.02, 5.0);
        ok &= Check("nudge keeps instance", ReferenceEquals(o1, o1b));
        ok &= Near("nudge updates key", o1.BurnTimeSec, 100.02);
        ok &= Check("no duplicate on nudge", exec.Options.Count == 1);

        // A far burn is a new option, not a re-key of the first.
        RcsBurnOptions o2 = exec.GetOrCreateOptions(200.0, 5.0);
        ok &= Check("far burn is new", !ReferenceEquals(o1, o2));
        ok &= Check("two options tracked", exec.Options.Count == 2);

        ok &= Check("find within tolerance", ReferenceEquals(exec.FindOptions(100.03, 5.0), o1));
        ok &= Check("find misses far", exec.FindOptions(150.0, 5.0) == null);

        // Match tolerances: 0.05 s and 0.1 m/s, exclusive at the bound.
        var o = new RcsBurnOptions { BurnTimeSec = 100.0, BurnDvMs = 5.0 };
        ok &= Check("time inside", o.Matches(100.04, 5.0));
        ok &= Check("time outside", !o.Matches(100.06, 5.0));
        ok &= Check("dv inside", o.Matches(100.0, 5.05));
        ok &= Check("dv outside", !o.Matches(100.0, 5.2));
        return ok;
    }

    // A present-but-unrecognised enum token (a value renamed or removed by a
    // mod update, or an out-of-range ordinal) must fall back to the field's
    // default and keep the block, not drop it or carry an undefined enum.
    private bool CheckUnknownEnumFallback()
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

        bool ok = Check("unknown-enum block kept",
            parsed.TryGetValue(("s", "v"), out RcsExecution? e) && e.Options.Count == 1);
        if (ok)
        {
            RcsBurnOptions o = parsed[("s", "v")].Options[0];
            ok &= Check("valid mode preserved", o.Mode == RcsExecutionMode.Rcs);
            ok &= Check("bad attitude falls back", o.Attitude == RcsAttitudeStrategy.Auto);
            ok &= Check("out-of-range allocator falls back", o.Allocator == RcsAllocator.Groups);
        }
        return ok;
    }

    private bool Near(string label, double actual, double expected)
    {
        bool ok = Math.Abs(actual - expected) < 1e-6 * Math.Max(1.0, Math.Abs(expected));
        if (!ok)
            HarnessLog.Line($"[{Name}] TEST {label}: got {actual}, expected {expected} => FAIL");
        return ok;
    }

    private bool Check(string label, bool condition)
    {
        if (!condition)
            HarnessLog.Line($"[{Name}] TEST {label} => FAIL");
        return condition;
    }
}
