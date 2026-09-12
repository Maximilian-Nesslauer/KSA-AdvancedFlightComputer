using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Framework;

// What a test reports through: log prefix, check accounting, exit code.
//
// Every check is logged, pass or fail. A headless run leaves nothing behind but its log, so a silent
// pass would be indistinguishable from a check an early return never reached.
public sealed class TestContext
{
    public TestContext(string name, HeadlessSession session)
    {
        Name = name;
        Session = session;
    }

    public string Name { get; }

    public HeadlessSession Session { get; }

    public CelestialSystem System => Session.System;

    public int CheckCount { get; private set; }

    public int FailureCount { get; private set; }

    public int SkipCount { get; private set; }

    // Returns the outcome so a caller can branch on it; the record is kept either way.
    public bool Check(string label, bool pass, string? detail = null)
    {
        CheckCount++;
        if (!pass)
            FailureCount++;
        string body = detail == null ? label : $"{label}: {detail}";
        HarnessLog.Line($"[{Name}] TEST {body} => {TestSupport.Verdict(pass)}");
        return pass;
    }

    public bool CheckRel(string label, double actual, double expected, double relTol, double floor = 0.0)
        => Check(label, Approx.Rel(actual, expected, relTol, floor), $"got {actual}, expected {expected}");

    public bool CheckAbs(string label, double actual, double expected, double absTol)
        => Check(label, Approx.Abs(actual, expected, absTol), $"got {actual}, expected {expected}");

    public bool CheckMixed(string label, double actual, double expected, double absTol, double relTol)
        => Check(label, Approx.Mixed(actual, expected, absTol, relTol), $"got {actual}, expected {expected}");

    // A precondition the test could not meet, so whatever it was going to assert never ran.
    public bool Fail(string label, string detail)
    {
        CheckCount++;
        FailureCount++;
        HarnessLog.Line($"[{Name}] TEST {label}: {detail} => FAIL");
        return false;
    }

    // Not an assertion: measured values, traces, capability dumps.
    public void Info(string message) => HarnessLog.Line($"[{Name}] {message}");

    // A case this run cannot cover. Counted so the summary tells "asserted nothing" from "passed".
    public void Skip(string reason)
    {
        SkipCount++;
        HarnessLog.Line($"[{Name}] SKIP: {reason}");
    }

    public int Finish()
    {
        bool pass = FailureCount == 0;
        HarnessLog.Line($"[{Name}] {CheckCount} check(s), {FailureCount} failed, {SkipCount} skipped " +
                        $"=> {TestSupport.Verdict(pass)}");
        return pass ? 0 : 1;
    }
}
