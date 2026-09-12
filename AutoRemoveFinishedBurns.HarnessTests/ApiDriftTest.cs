using AutoRemoveFinishedBurns.Core;
using AutoRemoveFinishedBurns.Features;
using HarmonyLib;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;

namespace AutoRemoveFinishedBurns.HarnessTests;

// Catches game-update drift in everything the mod resolves by name: the reflection targets behind
// both features, and the IL shape the settings transpiler anchors on. All of it runs against the
// live game assembly with no vehicle or sim step needed, so this is the first test to look at when
// a new game build breaks the mod.
public sealed class ApiDriftTest : IHarnessTest
{
    public string Name => "arfb-api-drift";

    public int Run(HeadlessSession session)
    {
        bool detectionOk = GameReflection.ValidateDetection();
        HarnessLog.Line($"[arfb-api-drift] detection reflection targets => {TestSupport.Verdict(detectionOk)}");

        bool settingsOk = GameReflection.ValidateSettings();
        HarnessLog.Line($"[arfb-api-drift] settings reflection targets => {TestSupport.Verdict(settingsOk)}");

        bool anchorOk = false;
        if (settingsOk)
        {
            // The transpiler returns the instructions unchanged when it cannot find its anchor, so
            // "inserted exactly one call" is the pass signal for the IL shape it depends on.
            var original = PatchProcessor.GetOriginalInstructions(GameReflection.GameSettings_OnDrawUi);
            int patchedCount = SettingsTabPatch.Transpiler(original).Count();
            anchorOk = patchedCount == original.Count + 1;
            HarnessLog.Line($"[arfb-api-drift] settings transpiler anchor ({original.Count} -> {patchedCount} instructions) => {TestSupport.Verdict(anchorOk)}");
        }
        else
        {
            HarnessLog.Line("[arfb-api-drift] settings transpiler anchor => SKIPPED (reflection targets missing)");
        }

        bool ok = detectionOk && settingsOk && anchorOk;
        HarnessLog.Line($"[arfb-api-drift] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }
}
