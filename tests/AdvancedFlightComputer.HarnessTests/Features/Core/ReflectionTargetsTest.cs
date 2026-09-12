using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Every string-keyed handle into the game resolves against the loaded build. That covers the
// reflection keys each feature validates before it patches, the IL anchors the transpilers look
// for, and the typed plan-window accessors, which also have to read and write through. A game-side
// rename fails here instead of silently disabling a feature at load.
public sealed class ReflectionTargetsTest : AfcTest
{
    public override string Name => "afc-reflection-targets";

    protected override void Execute(TestContext t)
    {
        t.Check("Core keys resolve", GameReflection.ValidateCore());
        t.Check("HyperbolicTargets keys resolve", GameReflection.ValidateHyperbolicTargets());
        t.Check("ManeuverTools keys resolve", GameReflection.ValidateManeuverTools());
        t.Check("PlanWindow keys resolve", GameReflection.ValidatePlanWindow());
        t.Check("MultiPass keys resolve", GameReflection.ValidateMultiPass());
        t.Check("RcsTranslation keys resolve", GameReflection.ValidateRcsTranslation());
        t.Check("AutoStage keys resolve", GameReflection.ValidateAutoStage());
        t.Check("AutoRemove keys resolve", GameReflection.ValidateAutoRemove());
        t.Check("SettingsPage keys resolve", GameReflection.ValidateSettingsPage());

        t.Check("DrawCorrectionTransfer anchor", PlanWindowPatchPipeline.HasCalculatedControlsAnchor);
        t.Check("Burn.Create anchor", PlanWindowPatchPipeline.HasCreateAnchor);
        t.Check("ConsoleStyle.PopWidgetStyle anchor", PlanWindowPatchPipeline.HasFallbackControlsAnchor);
        t.Check("BurnContextMenu.Draw anchor", Patch_BurnContextMenu_Launcher.IsAnchorPresent);
        t.Check("DrawSelectedTransfer anchor", Patch_TransferPlanner_DrawSelectedTransfer_Flyby.IsAnchorPresent);
        t.Check("DrawSelectedTransferUi anchor", Patch_TransferPlanner_DrawSelectedTransferUi_Flyby.IsAnchorPresent);
        t.Check("FindClosestApproaches anchor", Patch_FindClosestApproaches.IsAnchorPresent);
        t.Check("GameSettings.OnDrawUi PopWidgetStyle anchor", ModSettingsPage.IsAnchorPresent);
        t.Check("ModLibrary.AllParts anchor", GameReflection.ModLibrary_AllParts != null);

        // Stock initialises _transferType to its first entry, so a null key means the accessor did
        // not read through.
        t.Check("StockPlanner reads _transferType", StockPlanner.TransferTypeKey != null);

        bool before = StockPlanner.DisplaySelectedTransfer;
        StockPlanner.DisplaySelectedTransfer = !before;
        t.Check("StockPlanner writes _displaySelectedTransfer", StockPlanner.DisplaySelectedTransfer == !before);
        StockPlanner.DisplaySelectedTransfer = before;

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
