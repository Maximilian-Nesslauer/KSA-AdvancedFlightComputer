using System;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.PlanWindow;

/// <summary>
/// Skips stock's 3D preview of the selected transfer while a single burn flyby is
/// armed. Stock draws the porkchop entry aimed at the center, which for an armed
/// flyby is the impact trajectory the retarget exists to replace, so leaving it on
/// paints a second trajectory the Create button will never fly.
/// <see cref="HohmannFlybyUI.RenderPreview"/> draws the retargeted one instead,
/// from the postfix on the same <see cref="TransferPlanner.OnPreRender"/> pass.
///
/// This covers the orbit lines only. Stock's markers hang off the same
/// <c>_displaySelectedTransfer</c> toggle and are suppressed alongside by
/// <see cref="Patch_TransferPlanner_DrawSelectedTransferUi_Flyby"/>. The Lambert
/// preview has its own toggle and is deliberately left alone.
/// </summary>
[HarmonyPatch]
internal static class Patch_TransferPlanner_DrawSelectedTransfer_Flyby
{
    /// <summary>Whether stock still has the line and marker suppression anchor.</summary>
    public static bool IsAnchorPresent => GameReflection.TransferPlanner_DrawSelectedTransfer != null;

    static MethodBase TargetMethod() =>
        GameReflection.TransferPlanner_DrawSelectedTransfer ?? throw new InvalidOperationException(
            "[AFC] TransferPlanner.DrawSelectedTransfer(IViewport) not found; "
            + "patching this class requires an IsAnchorPresent check first.");

    static bool Prefix()
    {
        try
        {
            return !HohmannFlybyUI.SuppressesStockTransferPreview();
        }
        catch (Exception ex)
        {
            // Fails open, because a broken check must never remove stock's own
            // preview. Deduped because it runs per frame.
            LogHelper.WarnOnce("flyby-suppress-lines:" + ex.GetType().Name,
                $"[AFC] Flyby DrawSelectedTransfer prefix: {ex}; leaving stock preview on.");
            return true;
        }
    }
}
