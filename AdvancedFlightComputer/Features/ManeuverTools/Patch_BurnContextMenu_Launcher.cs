using System;
using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// IL transpiler on <see cref="BurnContextMenu.Draw"/> that injects
/// <see cref="BurnMenuLauncher.DrawInline"/> before the method's last
/// <see cref="ImGui.EndPopup"/> call.
///
/// A postfix cannot work here: Draw closes the popup before it returns, so
/// anything appended would land outside the popup scope and draw into whatever
/// window happens to be current.
///
/// Draw emits six separate EndPopup calls, one per early-return guard plus the one
/// closing the orbit branch after stock's own burn actions. The last in IL order is
/// that orbit-branch call, so anchoring on it sorts the AFC submenu below stock's
/// entries. Two things could move it in a later build: the compiler merging the
/// identical EndPopup-then-return tails into a shared epilogue, or another mode
/// branch being appended after the orbit branch. Neither is guarded here, because
/// the injected method re-checks its own preconditions; the failure mode is the
/// submenu appearing in an adjacent menu, not a wrong action.
/// </summary>
[HarmonyPatch(typeof(BurnContextMenu), nameof(BurnContextMenu.Draw))]
internal static class Patch_BurnContextMenu_Launcher
{
    /// <summary>Whether the patch target still exists.
    /// <see cref="ManeuverTools.ApplyPatches"/> uses this to decide whether to apply
    /// the patch at all. Resolved by name, not by <c>typeof</c>: BurnContextMenu
    /// arrived in 2026.8.3.5117, so on any earlier build the type is absent and a
    /// typeof would throw out of the gate that exists to prevent exactly that.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.TypeByName("KSA.BurnContextMenu") is Type menu
        && AccessTools.Method(menu, "Draw", Type.EmptyTypes) != null;

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var code = new List<CodeInstruction>(instructions);

        var anchor = AccessTools.Method(typeof(ImGui), nameof(ImGui.EndPopup), Type.EmptyTypes);
        var injectTarget = AccessTools.Method(typeof(BurnMenuLauncher),
            nameof(BurnMenuLauncher.DrawInline));

        if (anchor == null || injectTarget == null)
        {
            DefaultCategory.Log.Warning(
                "[AFC] BurnMenuLauncher transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving BurnContextMenu.Draw unmodified.");
            return code;
        }

        int anchorIndex = -1;
        int anchorCount = 0;
        for (int i = 0; i < code.Count; i++)
        {
            if (!code[i].Calls(anchor))
                continue;
            anchorIndex = i;
            anchorCount++;
        }

        if (anchorIndex < 0)
        {
            DefaultCategory.Log.Warning(
                "[AFC] BurnMenuLauncher transpiler: no ImGui.EndPopup() call found in " +
                "BurnContextMenu.Draw IL (" + code.Count + " IL instructions scanned); " +
                "the right-click shortcuts to the quick-tools stay hidden.");
            return code;
        }

        var endMenu = AccessTools.Method(typeof(ImGui), nameof(ImGui.EndMenu), Type.EmptyTypes);
        var apsisTarget = AccessTools.Method(typeof(BurnMenuLauncher),
            nameof(BurnMenuLauncher.DrawApsisEntry));
        var submenuEnds = new List<int>();
        if (endMenu != null && apsisTarget != null)
        {
            for (int i = 0; i < code.Count; i++)
            {
                if (code[i].Calls(endMenu))
                    submenuEnds.Add(i);
            }
        }

        if (submenuEnds.Count < 2)
            DefaultCategory.Log.Warning(
                "[AFC] BurnMenuLauncher transpiler: expected at least 2 ImGui.EndMenu() calls in " +
                "BurnContextMenu.Draw IL, found " + submenuEnds.Count +
                "; the apsis submenus get no AFC entry.");

        // Strictly descending: every insertion shifts the indices above it, so the
        // highest anchor has to be spliced first. The EndPopup anchor is the last
        // instruction of the three, hence first here.
        InsertBefore(code, anchorIndex, new CodeInstruction(OpCodes.Call, injectTarget));
        if (submenuEnds.Count >= 2)
        {
            InsertBefore(code, submenuEnds[1],
                new CodeInstruction(OpCodes.Ldc_I4, BurnMenuLauncher.ApoapsisSubmenu),
                new CodeInstruction(OpCodes.Call, apsisTarget!));
            InsertBefore(code, submenuEnds[0],
                new CodeInstruction(OpCodes.Ldc_I4, BurnMenuLauncher.PeriapsisSubmenu),
                new CodeInstruction(OpCodes.Call, apsisTarget!));
        }

        // Once per load: Harmony re-runs every transpiler on a method whenever
        // another patch lands on or leaves it.
        if (DebugConfig.ManeuverTools)
            LogHelper.DebugOnce("transpiler-burn-menu-launcher",
                "[AFC] BurnMenuLauncher transpiler: injected DrawInline before the last of " +
                anchorCount + " EndPopup call(s) and DrawApsisEntry into " +
                Math.Min(submenuEnds.Count, 2) + " submenu(s) (" + code.Count + " IL instructions).");

        return code;
    }

    /// <summary>Splices <paramref name="inserted"/> in front of the anchor, moving the
    /// anchor's labels onto the first inserted instruction. Without the move, every
    /// branch aiming at the anchor would jump straight past the injection.</summary>
    private static void InsertBefore(List<CodeInstruction> code, int anchorIndex,
        params CodeInstruction[] inserted)
    {
        inserted[0].labels.AddRange(code[anchorIndex].labels);
        code[anchorIndex].labels.Clear();
        code.InsertRange(anchorIndex, inserted);
    }
}
