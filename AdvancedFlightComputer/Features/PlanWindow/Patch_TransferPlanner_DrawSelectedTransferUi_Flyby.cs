using System;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.PlanWindow;

/// <summary>
/// Marker counterpart to <see cref="Patch_TransferPlanner_DrawSelectedTransfer_Flyby"/>.
/// Stock draws the selected transfer in two places off the same
/// <c>_displaySelectedTransfer</c> toggle. The orbit lines come from
/// <see cref="TransferPlanner.OnPreRender"/>, and the encounter, periapsis and
/// closest approach markers come from <c>DrawSelectedTransferUi</c> during the
/// plan window's ImGui pass. Suppressing only the lines would leave the
/// retargeted trajectory decorated with the markers of the plan aimed at the
/// center, which describe the impact the flyby replaced.
/// </summary>
[HarmonyPatch]
internal static class Patch_TransferPlanner_DrawSelectedTransferUi_Flyby
{
    // IGameViewport here and IViewport on the DrawSelectedTransfer sibling.
    // The wrong one resolves to null and the patch never binds.
    private static readonly Type[] Signature = { typeof(IGameViewport) };

    private static MethodInfo? Anchor =>
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransferUi", Signature);

    public static bool IsAnchorPresent => Anchor != null;

    static MethodBase TargetMethod() =>
        Anchor ?? throw new InvalidOperationException(
            "[AFC] TransferPlanner.DrawSelectedTransferUi(IGameViewport) not found; "
            + "patching this class requires an IsAnchorPresent check first.");

    static bool Prefix()
    {
        try
        {
            return !HohmannFlybyUI.SuppressesStockTransferPreview();
        }
        catch (Exception ex)
        {
            // Deduped because it runs per frame.
            LogHelper.WarnOnce("flyby-suppress-markers:" + ex.GetType().Name,
                $"[AFC] Flyby DrawSelectedTransferUi prefix: {ex}; leaving stock markers on.");
            return true;
        }
    }
}
