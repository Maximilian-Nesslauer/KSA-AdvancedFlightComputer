using System;
using System.Reflection;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
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
/// This covers the orbit lines only. Stock's markers hang off the same
/// <c>_displaySelectedTransfer</c> toggle and are suppressed alongside by
/// <see cref="Patch_TransferPlanner_DrawSelectedTransferUi_Flyby"/>; the Lambert
/// preview has its own separate toggle and is deliberately left alone.
/// </summary>
[HarmonyPatch]
internal static class Patch_TransferPlanner_DrawSelectedTransfer_Flyby
{
    // IViewport here, IGameViewport on the DrawSelectedTransferUi sibling.
    // Shared by the gate and the target so they cannot drift.
    private static readonly Type[] Signature = { typeof(IViewport) };

    private static MethodInfo? Anchor =>
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransfer", Signature);

    /// <summary>Whether the private stock method still exists in this build, so
    /// Mod.cs can skip the patch instead of failing to apply it.</summary>
    public static bool IsAnchorPresent => Anchor != null;

    static MethodBase TargetMethod() =>
        Anchor ?? throw new InvalidOperationException(
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
            // Fail-open: a broken check must never remove stock's own preview.
            // Deduped: runs per frame.
            LogHelper.WarnOnce("flyby-suppress-lines:" + ex.GetType().Name,
                $"[AFC] Flyby DrawSelectedTransfer prefix: {ex}; leaving stock preview on.");
            return true;
        }
    }
}
