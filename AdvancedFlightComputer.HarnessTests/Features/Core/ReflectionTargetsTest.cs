using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;

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
        t.Check("HyperbolicTargets keys resolve", GameReflection.ValidateHyperbolicTargets());
        t.Check("ManeuverTools keys resolve", GameReflection.ValidateManeuverTools());
        t.Check("MultiPass keys resolve", GameReflection.ValidateMultiPass());
        t.Check("RcsTranslation keys resolve", GameReflection.ValidateRcsTranslation());

        t.Check("DrawCorrectionTransfer anchor", Patch_DrawPlanWindow_HohmannMultiPass.IsAnchorPresent);
        t.Check("Burn.Create anchor", Patch_DrawPlanWindow_CreateInterceptor.IsAnchorPresent);
        t.Check("ConsoleStyle.PopWidgetStyle anchor", Patch_DrawPlanWindow_HohmannFallback.IsAnchorPresent);
        t.Check("BurnContextMenu.Draw anchor", Patch_BurnContextMenu_Launcher.IsAnchorPresent);
        t.Check("DrawSelectedTransfer anchor", Patch_TransferPlanner_DrawSelectedTransfer_Flyby.IsAnchorPresent);
        t.Check("DrawSelectedTransferUi anchor", Patch_TransferPlanner_DrawSelectedTransferUi_Flyby.IsAnchorPresent);

        // Stock initialises _transferType to its first entry, so a null key means the accessor did
        // not read through.
        t.Check("StockPlanner reads _transferType", StockPlanner.TransferTypeKey != null);

        bool before = StockPlanner.DisplaySelectedTransfer;
        StockPlanner.DisplaySelectedTransfer = !before;
        t.Check("StockPlanner writes _displaySelectedTransfer", StockPlanner.DisplaySelectedTransfer == !before);
        StockPlanner.DisplaySelectedTransfer = before;
    }
}
