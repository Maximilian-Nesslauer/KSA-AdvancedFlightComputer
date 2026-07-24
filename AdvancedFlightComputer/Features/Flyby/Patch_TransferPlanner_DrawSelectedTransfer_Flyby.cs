using System;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>
/// Skips stock's 3D preview of the selected transfer while a single-burn flyby is
/// armed. Stock draws the center-aimed porkchop entry, which for an armed flyby is
/// the impact trajectory the retarget exists to replace, so leaving it on paints a
/// second trajectory the Create button will never fly.
/// <see cref="HohmannFlybyUI.RenderPreview"/> draws the retargeted one instead,
/// from the postfix on the same <see cref="TransferPlanner.OnPreRender"/> pass.
///
/// Only the orbit lines are suppressed; stock's marker overlay
/// (<c>DrawSelectedTransferUi</c>) and the Lambert preview keep their own toggles.
/// Multi-pass is exempt because it intentionally lets stock render the final pass.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), "DrawSelectedTransfer")]
internal static class Patch_TransferPlanner_DrawSelectedTransfer_Flyby
{
    /// <summary>Whether the private stock method still exists in this build, so
    /// Mod.cs can skip the patch instead of failing to apply it.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransfer",
            new[] { typeof(Viewport) }) != null;

    static bool Prefix()
    {
        try
        {
            return !HohmannFlybyUI.SuppressesStockTransferPreview();
        }
        catch (Exception ex)
        {
            // Fail-open: a broken check must never remove stock's own preview.
            DefaultCategory.Log.Warning(
                $"[AFC] Flyby DrawSelectedTransfer prefix: {ex}; leaving stock preview on.");
            return true;
        }
    }
}
