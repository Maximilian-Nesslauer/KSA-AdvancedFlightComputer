using System;
using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// Insert the call inside the popup because a postfix would run after ImGui.EndPopup. The first two EndMenu calls in BurnContextMenu.Draw close the apse submenus.
[HarmonyPatch(typeof(BurnContextMenu), nameof(BurnContextMenu.Draw), new Type[] { })]
internal static class Patch_BurnContextMenu_Launcher
{
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
            LogHelper.WarnOnce("transpiler-burn-menu-missing",
                "[AFC] BurnMenuLauncher transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving BurnContextMenu.Draw unmodified.");
            return code;
        }

        int anchorIndex = -1;
        for (int i = 0; i < code.Count; i++)
        {
            if (!code[i].Calls(anchor))
                continue;
            anchorIndex = i;
        }

        if (anchorIndex < 0)
        {
            LogHelper.WarnOnce("transpiler-burn-menu-no-popup",
                "[AFC] BurnMenuLauncher transpiler: no ImGui.EndPopup() call found in " +
                "BurnContextMenu.Draw IL (" + code.Count + " IL instructions scanned); " +
                "the right-click shortcuts to the quick-tools stay hidden.");
            return code;
        }

        InsertShortcuts(code, anchorIndex, injectTarget);
        return code;
    }

    private static void InsertShortcuts(List<CodeInstruction> code, int anchorIndex,
        System.Reflection.MethodInfo injectTarget)
    {
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
            LogHelper.WarnOnce("transpiler-burn-menu-no-apses",
                "[AFC] BurnMenuLauncher transpiler: expected at least 2 ImGui.EndMenu() calls in " +
                "BurnContextMenu.Draw IL, found " + submenuEnds.Count +
                "; the apsis submenus get no AFC entry.");

        // Insert at descending indices so earlier anchors do not move.
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

        if (DebugConfig.ManeuverTools)
            LogHelper.DebugOnce("transpiler-burn-menu-launcher",
                "[AFC] BurnMenuLauncher transpiler: injected the popup shortcuts" +
                (submenuEnds.Count >= 2 ? " and both apse shortcuts." : "; apse shortcuts are unavailable."));
    }

    private static void InsertBefore(List<CodeInstruction> code, int anchorIndex,
        params CodeInstruction[] inserted)
    {
        TranspilerInsertion.MoveEntryMarkers(code[anchorIndex], inserted[0]);
        code.InsertRange(anchorIndex, inserted);
    }
}
