using System;
using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static class Patch_DrawPlanWindow_CreateInterceptor
{
    private static readonly Type[] BurnCreateSig = new[]
    {
        typeof(OrbitPointCce), typeof(double), typeof(double3),
        typeof(PatchedConic), typeof(Vehicle),
    };

    // All ConsoleWidgets.PrimaryButton overloads return the click boolean.
    private const string PrimaryButtonName = nameof(ConsoleWidgets.PrimaryButton);

    // The footer separates the button from Burn.Create, so refuse an anchor that is too far away. This check cannot detect an unrelated PrimaryButton within that gap.
    private const int MaxIlGapPrimaryButtonToBurnCreate = 160;

    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSig) != null;

    private static bool IsPrimaryButtonCall(CodeInstruction ins)
        => (ins.opcode == OpCodes.Call || ins.opcode == OpCodes.Callvirt)
           && ins.operand is System.Reflection.MethodInfo mi
           && mi.DeclaringType == typeof(ConsoleWidgets)
           && mi.Name == PrimaryButtonName;

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var burnCreateAnchor = AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSig);
        var burnCreateReplacement = AccessTools.Method(typeof(HohmannCreateInterceptor),
            nameof(HohmannCreateInterceptor.CreateMaybeMultiPass));
        var clickGate = AccessTools.Method(typeof(HohmannCreateInterceptor),
            nameof(HohmannCreateInterceptor.ShouldAllowCreateClick));

        if (burnCreateAnchor == null || burnCreateReplacement == null)
        {
            LogHelper.WarnOnce("transpiler-create-interceptor-noburncreate",
                "[AFC] HohmannCreateInterceptor transpiler: Burn.Create or its replacement " +
                "not found; leaving DrawPlanWindow unmodified.");
            return instructions;
        }

        var list = new List<CodeInstruction>(instructions);

        int burnCreateIdx = list.FindIndex(ins => ins.Calls(burnCreateAnchor));
        int createButtonIdx = FindCreateButton(list, burnCreateIdx, clickGate, out int gapTooLarge);
        return RewriteCalls(list, burnCreateIdx, burnCreateReplacement,
            createButtonIdx, clickGate, gapTooLarge);
    }

    private static int FindCreateButton(List<CodeInstruction> code, int burnCreateIdx,
        System.Reflection.MethodInfo? clickGate, out int gapTooLarge)
    {
        gapTooLarge = 0;
        if (clickGate == null)
            return -1;

        for (int i = burnCreateIdx - 1; i >= 0; i--)
        {
            if (!IsPrimaryButtonCall(code[i]))
                continue;
            int gap = burnCreateIdx - i;
            if (gap <= MaxIlGapPrimaryButtonToBurnCreate)
                return i;
            gapTooLarge = gap;
            break;
        }
        return -1;
    }

    private static IEnumerable<CodeInstruction> RewriteCalls(List<CodeInstruction> list,
        int burnCreateIdx, System.Reflection.MethodInfo burnCreateReplacement,
        int createButtonIdx, System.Reflection.MethodInfo? gateForInjection, int gapTooLarge)
    {
        bool burnCreateReplaced = false;
        bool gateInjected = false;

        for (int i = 0; i < list.Count; i++)
        {
            if (i == burnCreateIdx && !burnCreateReplaced)
            {
                // A replacement keeps both entry and exit markers on the same instruction.
                var swap = new CodeInstruction(OpCodes.Call, burnCreateReplacement);
                swap.labels.AddRange(list[i].labels);
                swap.blocks.AddRange(list[i].blocks);
                yield return swap;
                burnCreateReplaced = true;
                continue;
            }

            yield return list[i];

            if (i == createButtonIdx && !gateInjected && gateForInjection != null)
            {
                // The gate consumes the boolean result from PrimaryButton and replaces it with its own result.
                yield return new CodeInstruction(OpCodes.Call, gateForInjection);
                gateInjected = true;
            }
        }

        Report(burnCreateReplaced, gateInjected, gapTooLarge);
    }

    private static void Report(bool burnCreateReplaced, bool gateInjected, int gapTooLarge)
    {
        if (!burnCreateReplaced)
            LogHelper.WarnOnce("transpiler-create-interceptor-noswap",
                "[AFC] HohmannCreateInterceptor transpiler: no Burn.Create call " +
                "found in DrawPlanWindow; create-button interception inactive.");
        else if (DebugConfig.MultiPass)
            LogHelper.DebugOnce("transpiler-create-interceptor-swap",
                "[AFC] HohmannCreateInterceptor transpiler: replaced Burn.Create " +
                "in DrawPlanWindow.");

        if (!gateInjected)
        {
            if (gapTooLarge > 0)
                LogHelper.WarnOnce("transpiler-create-interceptor-gap",
                    $"[AFC] HohmannCreateInterceptor transpiler: nearest PrimaryButton is " +
                    $"{gapTooLarge} IL instructions before Burn.Create (threshold " +
                    $"{MaxIlGapPrimaryButtonToBurnCreate}); stock layout likely refactored, " +
                    "skipping click gate (legacy registry-has fallback in " +
                    "CreateMaybeMultiPass still active, duplicate burn possible).");
            else
                LogHelper.WarnOnce("transpiler-create-interceptor-nogate",
                    "[AFC] HohmannCreateInterceptor transpiler: no preceding PrimaryButton " +
                    "call found near Burn.Create; click gate inactive (sync-gap duplicate-" +
                    "burn protection falls back to legacy registry-has branch in " +
                    "CreateMaybeMultiPass, which still queues a duplicate burn).");
        }
        else if (DebugConfig.MultiPass)
            LogHelper.DebugOnce("transpiler-create-interceptor-gate",
                "[AFC] HohmannCreateInterceptor transpiler: click gate injected after " +
                "PrimaryButton (sync-gap duplicate-burn protection active).");
    }
}
