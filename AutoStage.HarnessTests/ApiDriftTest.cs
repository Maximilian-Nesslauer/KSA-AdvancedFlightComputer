using System.Reflection;
using AutoStage.Core;
using HarmonyLib;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AutoStage.HarnessTests;

// Verifies AutoStage's grip on the game build without flying anything: every reflection target the
// mod validates at load must resolve, the gauge enum injection must land in
// GaugeButtonFlightComputer.EnumTypes, and every method the mod patches must actually carry that
// patch. This is the fast drift alarm on a game update.
public sealed class ApiDriftTest : IHarnessTest
{
    private const string HarmonyId = "com.maxi.autostage";

    public string Name => "autostage-api-drift";

    public int Run(HeadlessSession session)
    {
        AutoStageHost.EnsureInitialized();

        bool coreOk = GameReflection.ValidateAll();
        bool delayOk = GameReflection.ValidateIgnitionDelay();
        bool enumOk = AutoStageHost.GaugeEnumInjected();

        HarnessLog.Line($"[autostage-api-drift] core reflection={(coreOk ? "ok" : "MISSING")}, " +
                        $"ignition-delay reflection={(delayOk ? "ok" : "MISSING")}, " +
                        $"gauge enum injected={(enumOk ? "ok" : "MISSING")}");

        // The staging detector's caches are invalidated from postfixes, so a patch that silently
        // fails to apply does not throw: it leaves the mod reading a stale view of the sequence
        // list for the rest of the flight. Assert the hooks are really installed.
        bool patchesOk =
            CheckPatched(AccessTools.Method(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence)),
                "SequenceList.ActivateNextSequence")
            & CheckPatched(AccessTools.Method(typeof(SequenceList), nameof(SequenceList.ResetCaches)),
                "SequenceList.ResetCaches")
            & CheckPatched(AccessTools.Method(typeof(Universe), nameof(Universe.ApplyVehicleSolvers)),
                "Universe.ApplyVehicleSolvers")
            & CheckPatched(GameReflection.Vehicle_Dispose, "Vehicle.Dispose(bool)")
            & CheckPatched(AccessTools.Method(typeof(Vehicle), nameof(Vehicle.ToggleEnum)),
                "Vehicle.ToggleEnum")
            & CheckPatched(GameReflection.Vehicle_IsSet_Enum, "Vehicle.IsSet<Enum>")
            & CheckPatched(GameReflection.Vehicle_IsFlightComputerDisabled_Enum,
                "Vehicle.IsFlightComputerDisabled<Enum>")
            & CheckPatched(AccessTools.Method(typeof(Part), nameof(Part.DrawPartInfo)),
                "Part.DrawPartInfo")
            & CheckPatched(AccessTools.Method(typeof(GameSettings), nameof(GameSettings.OnDrawUi)),
                "GameSettings.OnDrawUi");

        // Harmony.GetPatchInfo cannot see this one. SettingsTabPatch is a transpiler that
        // searches OnDrawUi's IL for a call to ConsoleStyle.PopWidgetStyle(); if the anchor
        // moves it logs a warning, returns the instructions untouched, and still counts as
        // applied, so the settings page disappears while every check above stays green.
        bool anchorOk = SettingsAnchorPresent();
        HarnessLog.Line($"[autostage-api-drift] settings transpiler anchor " +
                        $"(ConsoleStyle.PopWidgetStyle in GameSettings.OnDrawUi) => {(anchorOk ? "ok" : "MISSING")}");

        bool activationOk = ActivationPathIntact();
        HarnessLog.Line($"[autostage-api-drift] stock activation path " +
                        $"(Part.ActivateSubtreeInStage in SequenceList.ActivateNextSequence) => " +
                        $"{(activationOk ? "ok" : "MISSING")}");

        bool ok = coreOk && delayOk && enumOk && patchesOk && anchorOk && activationOk;
        HarnessLog.Line($"[autostage-api-drift] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    // AutoStage reimplements ActivateNextSequence rather than patching it, and nothing the compiler
    // checks ties the clone to the original. Canary for stock changing how a row is activated.
    private static bool ActivationPathIntact()
    {
        MethodBase? target = AccessTools.Method(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence));
        MethodInfo? activate = AccessTools.Method(typeof(Part), nameof(Part.ActivateSubtreeInStage));
        if (target == null || activate == null)
            return false;

        foreach (CodeInstruction instruction in PatchProcessor.GetOriginalInstructions(target))
        {
            if (instruction.Calls(activate))
                return true;
        }
        return false;
    }

    // Reads the patched method's IL the same way the transpiler does, so this fails for the
    // same reason the transpiler would give up.
    private static bool SettingsAnchorPresent()
    {
        MethodInfo? anchor = AccessTools.Method(typeof(ConsoleStyle),
            nameof(ConsoleStyle.PopWidgetStyle), Type.EmptyTypes);
        MethodBase? target = AccessTools.Method(typeof(GameSettings), nameof(GameSettings.OnDrawUi));
        if (anchor == null || target == null)
            return false;

        foreach (CodeInstruction instruction in PatchProcessor.GetOriginalInstructions(target))
        {
            if (instruction.Calls(anchor))
                return true;
        }
        return false;
    }

    private static bool CheckPatched(MethodBase? target, string name)
    {
        if (target == null)
        {
            HarnessLog.Line($"[autostage-api-drift] patch target '{name}' does not resolve => MISSING");
            return false;
        }

        Patches? info = Harmony.GetPatchInfo(target);
        bool patched = info != null && (Owns(info.Prefixes) || Owns(info.Postfixes) || Owns(info.Transpilers));
        HarnessLog.Line($"[autostage-api-drift] '{name}' patched by {HarmonyId} => {(patched ? "ok" : "MISSING")}");
        return patched;
    }

    private static bool Owns(IEnumerable<Patch> patches)
    {
        foreach (Patch patch in patches)
        {
            if (patch.owner == HarmonyId) return true;
        }
        return false;
    }
}
