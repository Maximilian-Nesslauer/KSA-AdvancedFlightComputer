using System;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>
/// Marker counterpart to <see cref="Patch_TransferPlanner_DrawSelectedTransfer_Flyby"/>.
/// Stock draws the selected transfer in two places off the same
/// <c>_displaySelectedTransfer</c> toggle: the orbit lines from
/// <see cref="TransferPlanner.OnPreRender"/>, and the Encounter / Pe / closest-
/// approach markers from <c>DrawSelectedTransferUi</c> during the plan window's
/// ImGui pass. Suppressing only the lines left the retargeted trajectory decorated
/// with the center-aimed plan's markers, which describe the impact the flyby
/// replaced.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), "DrawSelectedTransferUi")]
internal static class Patch_TransferPlanner_DrawSelectedTransferUi_Flyby
{
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransferUi",
            new[] { typeof(Viewport) }) != null;

    static bool Prefix()
    {
        try
        {
            return !HohmannFlybyUI.SuppressesStockTransferPreview();
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] Flyby DrawSelectedTransferUi prefix: {ex}; leaving stock markers on.");
            return true;
        }
    }
}
