using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Covers the save-scoped reset list SaveLoadObserver's UncompressedSave.Load postfix and Mod.Unload
// both run. Two properties matter and neither needs a loaded save:
//
//  1. It must not throw. The load postfix wraps everything in one try/catch, so an exception in an
//     early reset would be swallowed and silently skip MultiPassRegistry.Load() and the SaveLoaded
//     event that follow it - the failure would look like "multi-pass forgot my execution", with one
//     warning far from the cause. Cold state and repeated calls are the cases that would trip it.
//
//  2. It must actually clear. OrbitToolForm's shared fields are the part of the list with a
//     public surface, so they stand in for the whole: they are read by OrbitToolsPanel in the
//     same frame to compute the maneuver, and a value carried into a freshly loaded world would plan
//     against the previous one's input.
public sealed class SaveScopedStateTest : AfcTest
{
    private const double MarkerAltitudeM = 123_456.0;
    private const double MarkerInclinationRad = 0.75;

    public override string Name => "afc-save-scoped-reset";

    protected override void Execute(TestContext t)
    {
        // Cold: nothing has been drawn or cached in this process yet.
        CheckNoThrow(t, "resets cold state");

        OrbitToolForm.TargetAltitude = MarkerAltitudeM;
        OrbitToolForm.TargetInclinationRad = MarkerInclinationRad;
        OrbitToolForm.UseDescendingNode = true;

        CheckNoThrow(t, "resets populated state");
        t.Check("clears TargetAltitude", OrbitToolForm.TargetAltitude == 0.0);
        t.Check("clears TargetInclinationRad", OrbitToolForm.TargetInclinationRad == 0.0);
        t.Check("clears UseDescendingNode", !OrbitToolForm.UseDescendingNode);
        t.Check("clears the target selection",
            OrbitToolForm.GetSelectedTargetOrbiter() == null);

        // Idempotent: the load postfix and Mod.Unload can both run in one process.
        CheckNoThrow(t, "resets twice in a row");
    }

    // "Must not throw" is the assertion, so this catches. Drift is let through: a renamed game
    // member resolves inside this try, and catching it would hide an infrastructure failure.
    private static void CheckNoThrow(TestContext t, string label)
    {
        try
        {
            SaveScopedState.ResetAll();
        }
        catch (Exception ex) when (!IsGameApiDrift(ex))
        {
            t.Check(label, false, $"threw {ex.GetType().Name}: {ex.Message}");
            return;
        }
        t.Check(label, true);
    }

    // Drift can arrive wrapped (a TypeInitializationException around a MissingMethodException), so
    // walk the chain, the same way HarnessRunner does.
    private static bool IsGameApiDrift(Exception e)
    {
        for (Exception? cur = e; cur != null; cur = cur.InnerException)
        {
            if (cur is MissingMemberException or TypeLoadException)
                return true;
        }
        return false;
    }
}
