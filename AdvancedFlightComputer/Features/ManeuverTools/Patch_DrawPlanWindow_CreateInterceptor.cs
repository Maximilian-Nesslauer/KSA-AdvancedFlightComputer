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

/// <summary>
/// IL transpiler on <see cref="TransferPlanner.DrawPlanWindow"/>.
/// Two related injections at the stock "Create" button site:
///
/// 1. Swaps the <see cref="Burn.Create"/> call for our
///    wrapper <see cref="HohmannCreateInterceptor.CreateMaybeMultiPass"/>,
///    so a single button click routes to multi-pass when armed for N>1
///    or falls through to a single burn otherwise.
/// 2. Inserts a call to <see cref="HohmannCreateInterceptor.ShouldAllowCreateClick"/>
///    immediately after the <see cref="ConsoleWidgets.PrimaryButton"/> call
///    that drives the same button (the nearest preceding one in the IL
///    before the Burn.Create call). Same stack effect (consumes
///    one bool, returns one bool); the gate absorbs the click when a
///    multi-pass exec is already in <see cref="MultiPassRegistry"/>,
///    keeping the sync-gap window after a save load from queueing a
///    duplicate burn (no dedup exists at BurnUpdateBuffer, FlightComputer.AddBurn,
///    or BurnPlan.AddBurnFromFlightComputerOnly).
///
/// Co-exists with <see cref="Patch_DrawPlanWindow_HohmannMultiPass"/>:
/// that one injects DrawInline before DrawCorrectionTransfer (different
/// IL site), this one rewrites a call and inserts a gate (also different
/// sites).
///
/// The invariant this relies on: DrawPlanWindow's own body contains
/// exactly one Burn.Create call site. Stock's other one belongs to
/// DrawCorrectionTransfer, a separate method body, so it is NOT touched
/// by this transpiler. Stock draws the Create button in the window footer
/// and reads the click back further down, so the button call still
/// precedes Burn.Create in the IL and the walk-back still lands on it;
/// the plain Calculate / Re-Calculate buttons use ConsoleWidgets.Button,
/// a different method, so they cannot be hit by mistake.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow_CreateInterceptor
{
    private static readonly Type[] BurnCreateSig = new[]
    {
        typeof(OrbitPointCce), typeof(double), typeof(double3),
        typeof(PatchedConic), typeof(Vehicle),
    };

    // Matched by name, not by signature: ConsoleWidgets declares three
    // PrimaryButton overloads that all forward to the same draw, and stock
    // currently calls the 1-arg one. Pinning that signature would silently drop
    // the click gate if stock switched overloads, which is a duplicate-burn
    // hazard rather than a cosmetic one.
    private const string PrimaryButtonName = nameof(ConsoleWidgets.PrimaryButton);

    // Sanity ceiling for the IL gap between the located PrimaryButton and
    // Burn.Create. Stock now draws the button in the footer and evaluates the
    // click in a separate block below EndFooter, so the two sites sit roughly
    // 60 instructions apart; 160 leaves room for moderate stock-side IL
    // changes but catches a wholesale refactor where the walk-back can no
    // longer be trusted to point at the "Create" button. The walk-back stops
    // at the first PrimaryButton, so the failure mode this guards is "stock
    // bloated the footer enough that our anchor is meaningless" - it does NOT
    // catch "stock inserted an unrelated PrimaryButton BETWEEN Create and
    // Burn.Create" (that case still slips through with a small gap; semantic
    // detection would need to inspect the label literal, which is not
    // tractable from CodeInstruction operands).
    private const int MaxIlGapPrimaryButtonToBurnCreate = 160;

    /// <summary>Returns true if the stock <see cref="Burn.Create"/>
    /// overload we anchor on exists. Mod.cs uses this to decide whether
    /// to apply the transpiler. The PrimaryButton anchor is checked
    /// inside the transpiler and degrades to "swap only, no gate" if
    /// missing, so it is not required for patch-apply eligibility.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSig) != null;

    /// <summary>True for a call to any ConsoleWidgets.PrimaryButton overload.
    /// Every overload returns the click bool, so the gate composes with all of
    /// them the same way.</summary>
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

        // Warnings are deduped like the success lines: Harmony re-runs every
        // transpiler on DrawPlanWindow whenever another patch lands on or leaves
        // it, and five AFC patches target that method.
        if (burnCreateAnchor == null)
        {
            LogHelper.WarnOnce("transpiler-create-interceptor-noburncreate",
                "[AFC] HohmannCreateInterceptor transpiler: stock Burn.Create " +
                "overload not found; leaving DrawPlanWindow unmodified.");
            foreach (var ins in instructions) yield return ins;
            yield break;
        }

        var list = new List<CodeInstruction>(instructions);

        // First pass: locate the Burn.Create call site (swap target and
        // anchor for the preceding PrimaryButton walk-back).
        int burnCreateIdx = -1;
        for (int i = 0; i < list.Count; i++)
        {
            if (list[i].Calls(burnCreateAnchor)) { burnCreateIdx = i; break; }
        }

        // Walk back from Burn.Create to find the nearest preceding
        // PrimaryButton call. By stock IL layout that is the "Create"
        // button itself. `gateForInjection` captures the (non-null)
        // clickGate reference only along the path where we found a
        // candidate, so the use site below needs no `null!` to satisfy
        // the nullable analyzer.
        int createButtonIdx = -1;
        System.Reflection.MethodInfo? gateForInjection = null;
        if (clickGate != null && burnCreateIdx >= 0)
        {
            for (int i = burnCreateIdx - 1; i >= 0; i--)
            {
                if (IsPrimaryButtonCall(list[i]))
                {
                    createButtonIdx = i;
                    gateForInjection = clickGate;
                    break;
                }
            }
        }

        // Sanity check (see MaxIlGapPrimaryButtonToBurnCreate comment): if
        // the gap is suspiciously large the walk-back probably anchors
        // on a button unrelated to "Create". Skip the gate rather than
        // absorb clicks on an unknown widget.
        int gapTooLarge = 0;
        if (createButtonIdx >= 0 && burnCreateIdx - createButtonIdx > MaxIlGapPrimaryButtonToBurnCreate)
        {
            gapTooLarge = burnCreateIdx - createButtonIdx;
            createButtonIdx = -1;
            gateForInjection = null;
        }

        bool burnCreateReplaced = false;
        bool gateInjected = false;

        for (int i = 0; i < list.Count; i++)
        {
            if (i == burnCreateIdx && !burnCreateReplaced)
            {
                // Carry labels AND exception block markers from the
                // original call so branches still target our
                // replacement and a (future) surrounding try/catch
                // doesn't lose its block boundary on the rewrite.
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
                // Stack after PrimaryButton: [bool clicked]. The gate's
                // signature is (bool) -> bool, so we just append a
                // call; the existing brfalse that gates the inner
                // block now reads our (possibly absorbed) result.
                yield return new CodeInstruction(OpCodes.Call, gateForInjection);
                gateInjected = true;
            }
        }

        if (!burnCreateReplaced)
            LogHelper.WarnOnce("transpiler-create-interceptor-noswap",
                "[AFC] HohmannCreateInterceptor transpiler: no Burn.Create call " +
                "found in DrawPlanWindow; create-button interception inactive.");
        // Once per load: Harmony re-runs this transpiler whenever another
        // patch lands on or leaves DrawPlanWindow, and each re-run would
        // repeat the two success lines.
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
