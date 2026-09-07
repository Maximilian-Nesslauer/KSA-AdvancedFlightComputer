using System;
using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using System.Text;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// Insert the call inside the popup because a postfix would run after ImGui.EndPopup.
[HarmonyPatch(typeof(BurnContextMenu), nameof(BurnContextMenu.Draw), new Type[] { })]
internal static class Patch_BurnContextMenu_Launcher
{
    // An apse burn changes the opposite apse. Match headings so a new submenu cannot move an entry.
    private static readonly (string Heading, int Submenu)[] ApseSubmenus =
    {
        ("At Periapsis", BurnMenuLauncher.PeriapsisSubmenu),
        ("At Apoapsis", BurnMenuLauncher.ApoapsisSubmenu),
    };

    // A menu label is loaded a few instructions ahead of the call that consumes it.
    private const int MaxIlGapLabelToBeginMenu = 8;

    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(BurnContextMenu), nameof(BurnContextMenu.Draw), Type.EmptyTypes) != null;

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

        // Earlier EndPopup calls belong to early returns. Insert before the final one.
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
        MethodInfo injectTarget)
    {
        // Insert the main group first so it does not change later submenu indices.
        InsertBefore(code, anchorIndex, new CodeInstruction(OpCodes.Call, injectTarget));

        var beginMenu = AccessTools.Method(typeof(ImGui), nameof(ImGui.BeginMenu),
            new[] { typeof(ImString), typeof(bool) });
        var endMenu = AccessTools.Method(typeof(ImGui), nameof(ImGui.EndMenu), Type.EmptyTypes);
        var apsisTarget = AccessTools.Method(typeof(BurnMenuLauncher),
            nameof(BurnMenuLauncher.DrawApsisEntry));
        if (beginMenu == null || endMenu == null || apsisTarget == null)
        {
            LogHelper.WarnOnce("transpiler-burn-menu-no-apses",
                "[AFC] BurnMenuLauncher transpiler: apse anchoring needs ImGui.BeginMenu " +
                "(anchor=" + (beginMenu != null ? "ok" : "MISSING") + "), ImGui.EndMenu " +
                "(anchor=" + (endMenu != null ? "ok" : "MISSING") + ") and DrawApsisEntry " +
                "(inject=" + (apsisTarget != null ? "ok" : "MISSING") +
                "); the apse submenus get no AFC entry.");
            ReportInsertion(bothApses: false);
            return;
        }

        var insertions = new List<(int End, int Submenu)>();
        foreach ((string heading, int submenu) in ApseSubmenus)
        {
            int begin = FindSubmenu(code, beginMenu, heading);
            int end = begin < 0 ? -1 : FindSubmenuEnd(code, beginMenu, endMenu, begin);
            if (end < 0)
            {
                LogHelper.WarnOnce("transpiler-burn-menu-heading-" + heading,
                    "[AFC] BurnMenuLauncher transpiler: no complete '" + heading + "' submenu " +
                    "found in BurnContextMenu.Draw IL; that submenu gets no AFC entry.");
                continue;
            }
            insertions.Add((end, submenu));
        }

        // Insert at descending indices so earlier anchors do not move.
        insertions.Sort((left, right) => right.End.CompareTo(left.End));
        foreach ((int end, int submenu) in insertions)
        {
            InsertBefore(code, end,
                new CodeInstruction(OpCodes.Ldc_I4, submenu),
                new CodeInstruction(OpCodes.Call, apsisTarget));
        }

        ReportInsertion(insertions.Count == ApseSubmenus.Length);
    }

    private static void ReportInsertion(bool bothApses)
    {
        if (DebugConfig.ManeuverTools)
            LogHelper.DebugOnce("transpiler-burn-menu-launcher",
                "[AFC] BurnMenuLauncher transpiler: injected the popup shortcuts" +
                (bothApses ? " and both apse shortcuts." : "; some apse shortcuts are unavailable."));
    }

    /// <summary>Finds the submenu with the specified heading.</summary>
    private static int FindSubmenu(List<CodeInstruction> code, MethodInfo beginMenu, string heading)
    {
        for (int i = 0; i < code.Count; i++)
        {
            if (code[i].Calls(beginMenu) && ReadMenuHeading(code, i) == heading)
                return i;
        }
        return -1;
    }

    /// <summary>Finds the matching end of a submenu, including nested menus.</summary>
    private static int FindSubmenuEnd(List<CodeInstruction> code, MethodInfo beginMenu,
        MethodInfo endMenu, int beginIndex)
    {
        int depth = 0;
        for (int i = beginIndex + 1; i < code.Count; i++)
        {
            if (code[i].Calls(beginMenu))
                depth++;
            else if (code[i].Calls(endMenu))
            {
                if (depth == 0)
                    return i;
                depth--;
            }
        }
        return -1;
    }

    /// <summary>Reads the UTF-8 literal passed to a <c>BeginMenu</c> call.</summary>
    private static string? ReadMenuHeading(List<CodeInstruction> code, int beginMenuIndex)
    {
        int stop = Math.Max(0, beginMenuIndex - MaxIlGapLabelToBeginMenu);
        for (int i = beginMenuIndex - 1; i >= stop; i--)
        {
            if (code[i].opcode != OpCodes.Ldsflda || code[i].operand is not FieldInfo data)
                continue;
            try
            {
                ReadOnlySpan<byte> bytes = RuntimeHelpers.CreateSpan<byte>(data.FieldHandle);
                int length = i + 1 < code.Count && TryReadInt32(code[i + 1], out int stated)
                             && stated >= 0 && stated <= bytes.Length
                    ? stated
                    : bytes.TrimEnd((byte)0).Length;
                return Encoding.UTF8.GetString(bytes[..length]);
            }
            catch (Exception)
            {
                // This static field does not contain a readable literal.
            }
        }
        return null;
    }

    private static bool TryReadInt32(CodeInstruction instruction, out int value)
    {
        value = 0;
        if (instruction.opcode == OpCodes.Ldc_I4 && instruction.operand is int wide)
            value = wide;
        else if (instruction.opcode == OpCodes.Ldc_I4_S && instruction.operand is sbyte narrow)
            value = narrow;
        else if (instruction.opcode.Value >= OpCodes.Ldc_I4_0.Value
                 && instruction.opcode.Value <= OpCodes.Ldc_I4_8.Value)
            value = instruction.opcode.Value - OpCodes.Ldc_I4_0.Value;
        else
            return false;
        return true;
    }

    private static void InsertBefore(List<CodeInstruction> code, int anchorIndex,
        params CodeInstruction[] inserted)
    {
        TranspilerInsertion.MoveEntryMarkers(code[anchorIndex], inserted[0]);
        code.InsertRange(anchorIndex, inserted);
    }
}
