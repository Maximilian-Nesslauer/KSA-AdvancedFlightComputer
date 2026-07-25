using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

// Covers the save-scoped reset list SaveLoadObserver's UncompressedSave.Load postfix and Mod.Unload
// both run. Two properties matter and neither needs a loaded save:
//
//  1. It must not throw. The load postfix wraps everything in one try/catch, so an exception in an
//     early reset would be swallowed and silently skip MultiPassRegistry.Load() and the SaveLoaded
//     event that follow it - the failure would look like "multi-pass forgot my execution", with one
//     warning far from the cause. Cold state and repeated calls are the cases that would trip it.
//
//  2. It must actually clear. ManeuverToolsWindow's shared fields are the part of the list with a
//     public surface, so they stand in for the whole: they are read by Patch_DrawPlanWindow in the
//     same frame to compute the maneuver, and a value carried into a freshly loaded world would plan
//     against the previous one's input.
public sealed class SaveScopedStateTest : IHarnessTest
{
    private const double MarkerAltitudeM = 123_456.0;
    private const double MarkerInclinationRad = 0.75;

    public string Name => "afc-save-scoped-reset";

    public int Run(HeadlessSession session)
    {
        bool ok = true;

        // Cold: nothing has been drawn or cached in this process yet.
        ok &= NoThrow("resets cold state");

        ManeuverToolsWindow.TargetAltitude = MarkerAltitudeM;
        ManeuverToolsWindow.TargetInclinationRad = MarkerInclinationRad;
        ManeuverToolsWindow.UseDescendingNode = true;

        ok &= NoThrow("resets populated state");
        ok &= Check("clears TargetAltitude", ManeuverToolsWindow.TargetAltitude == 0.0);
        ok &= Check("clears TargetInclinationRad", ManeuverToolsWindow.TargetInclinationRad == 0.0);
        ok &= Check("clears UseDescendingNode", !ManeuverToolsWindow.UseDescendingNode);
        ok &= Check("clears the target selection",
            ManeuverToolsWindow.GetSelectedTargetOrbiter() == null);

        // Idempotent: the load postfix and Mod.Unload can both run in one process.
        ok &= NoThrow("resets twice in a row");

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool NoThrow(string label)
    {
        try
        {
            SaveScopedState.ResetAll();
        }
        catch (Exception ex)
        {
            HarnessLog.Line($"[{Name}] TEST {label}: threw {ex.GetType().Name}: {ex.Message} => FAIL");
            return false;
        }
        HarnessLog.Line($"[{Name}] TEST {label} => PASS");
        return true;
    }

    private bool Check(string label, bool pass)
    {
        HarnessLog.Line($"[{Name}] TEST {label} => {TestSupport.Verdict(pass)}");
        return pass;
    }
}
