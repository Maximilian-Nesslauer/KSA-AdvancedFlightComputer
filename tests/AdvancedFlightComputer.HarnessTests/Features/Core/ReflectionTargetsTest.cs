using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Every string-keyed handle into the game resolves against the loaded build. That covers the
// reflection keys each feature validates before it patches, the IL anchors the transpilers look
// for, the typed planner accessors, which also have to read through, and the public planner setter
// the planner key routing sits on. A game-side rename fails here instead of silently disabling a
// feature at load.
public sealed class ReflectionTargetsTest : AfcTest
{
    public override string Name => "afc-reflection-targets";

    protected override void Execute(TestContext t)
    {
        t.Check("Core keys resolve", GameReflection.ValidateCore());
        t.Check("HyperbolicTargets keys resolve", GameReflection.ValidateHyperbolicTargets());
        t.Check("MultiPass keys resolve", GameReflection.ValidateMultiPass());
        t.Check("RcsTranslation keys resolve", GameReflection.ValidateRcsTranslation());
        t.Check("AutoStage keys resolve", GameReflection.ValidateAutoStage());
        t.Check("AutoRemove keys resolve", GameReflection.ValidateAutoRemove());
        t.Check("SettingsPage keys resolve", GameReflection.ValidateSettingsPage());

        t.Check("BurnContextMenu.Draw anchor", Patch_BurnContextMenu_Launcher.IsAnchorPresent);
        t.Check("TransferPlanner.set_ShowPlanWindow anchor", PlannerRoute.Setter != null);
        t.Check("FindClosestApproaches anchor", Patch_FindClosestApproaches.IsAnchorPresent);
        t.Check("GameSettings.OnDrawUi PopWidgetStyle anchor", ModSettingsPage.IsAnchorPresent);
        t.Check("ModLibrary.AllParts anchor", GameReflection.ModLibrary_AllParts != null);
        // RcsWarpPatch binds by its attribute, so the overload it names is checked here.
        t.Check("Universe.AutoWarpTo(UniverseTime, double, bool) anchor",
            AccessTools.Method(typeof(Universe), nameof(Universe.AutoWarpTo),
                [typeof(UniverseTime), typeof(double), typeof(bool)]) != null);

        // Stock fills _timeUnits in its field initialiser, so a null list means the accessor did not
        // read through.
        t.Check("StockPlanner reads _timeUnits", StockPlanner.TimeUnits != null);

        bool wasInjected = AutoStageFeature.GaugeEnumInjected;
        t.Check("AfcAutoStageToggle injects into GaugeButtonFlightComputer.EnumTypes",
            AutoStageFeature.InjectGaugeEnum() && AutoStageFeature.GaugeEnumInjected);
        if (!wasInjected)
            AutoStageFeature.RemoveGaugeEnum();

        // StagingExecution clones SequenceList.ActivateNextSequence rather than patching it, and
        // nothing the compiler checks ties the clone to stock's row activation.
        t.Check("SequenceList.ActivateNextSequence activates through Part.ActivateSubtreeInStage", StockActivationPathIntact());
    }

    private static bool StockActivationPathIntact()
    {
        MethodBase? target = AccessTools.Method(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence), [typeof(Vehicle)]);
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
}
