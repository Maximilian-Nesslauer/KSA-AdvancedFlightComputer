using System.Reflection;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.PlanWindow;

internal static class PlanWindowPatchPipeline
{
    private static readonly Type[] BurnCreateSignature =
    [
        typeof(OrbitPointCce), typeof(double), typeof(Brutal.Numerics.double3),
        typeof(PatchedConic), typeof(Vehicle),
    ];

    private const int MaxIlGapPrimaryButtonToBurnCreate = 160;

    /// <summary>Whether stock still has the anchor beside the calculated transfer controls.</summary>
    internal static bool HasCalculatedControlsAnchor => CalculatedControlsAnchor() != null;

    /// <summary>Whether stock still has the fallback anchor that runs on every frame.</summary>
    internal static bool HasFallbackControlsAnchor => FallbackControlsAnchor() != null;

    private static MethodInfo? CalculatedControlsAnchor()
        => AccessTools.Method(typeof(TransferPlanner), "DrawCorrectionTransfer", Type.EmptyTypes);

    private static MethodInfo? FallbackControlsAnchor()
        => AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle), Type.EmptyTypes);

    internal static bool HasCreateAnchor =>
        AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSignature) != null;

    internal static IEnumerable<CodeInstruction> Rewrite(
        IEnumerable<CodeInstruction> instructions)
    {
        IEnumerable<CodeInstruction> rewritten = InjectCalculatedControls(instructions);
        rewritten = InjectFallbackControls(rewritten);
        return InterceptCreate(rewritten);
    }

    internal static IEnumerable<CodeInstruction> InjectCalculatedControls(
        IEnumerable<CodeInstruction> instructions)
    {
        MethodInfo? anchor = CalculatedControlsAnchor();
        MethodInfo? injectTarget = AccessTools.Method(
            typeof(HohmannMultiPassUI), nameof(HohmannMultiPassUI.DrawInline));

        var code = new List<CodeInstruction>(instructions);
        if (anchor == null || injectTarget == null)
        {
            LogHelper.WarnOnce("transpiler-hohmann-multipass-missing",
                "[AFC] HohmannMultiPass transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving this DrawPlanWindow stage unmodified.");
            return code;
        }

        int index = code.FindIndex(instruction => instruction.Calls(anchor));
        if (index < 0)
        {
            LogHelper.WarnOnce("transpiler-hohmann-multipass-no-call",
                $"[AFC] HohmannMultiPass transpiler: no call to DrawCorrectionTransfer " +
                $"found in DrawPlanWindow IL ({code.Count} IL instructions scanned); " +
                "multi-pass section not injected.");
            return code;
        }

        var injected = new CodeInstruction(OpCodes.Call, injectTarget);
        TranspilerInsertion.MoveEntryMarkers(code[index], injected);
        code.Insert(index, injected);

        if (DebugConfig.MultiPass)
            LogHelper.DebugOnce("transpiler-hohmann-multipass",
                "[AFC] HohmannMultiPass transpiler: injected DrawInline before " +
                "DrawCorrectionTransfer.");
        return code;
    }

    internal static IEnumerable<CodeInstruction> InjectFallbackControls(
        IEnumerable<CodeInstruction> instructions)
    {
        MethodInfo? anchor = FallbackControlsAnchor();
        MethodInfo? injectTarget = AccessTools.Method(
            typeof(HohmannMultiPassUI), nameof(HohmannMultiPassUI.DrawInline));

        var code = new List<CodeInstruction>(instructions);
        if (anchor == null || injectTarget == null)
        {
            LogHelper.WarnOnce("transpiler-hohmann-fallback-missing",
                "[AFC] HohmannFallback transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving this DrawPlanWindow stage unmodified.");
            return code;
        }

        int index = code.FindIndex(instruction => instruction.Calls(anchor));
        if (index < 0)
        {
            LogHelper.WarnOnce("transpiler-hohmann-fallback-noanchor",
                "[AFC] HohmannFallback transpiler: no ConsoleStyle.PopWidgetStyle() call " +
                "found in DrawPlanWindow IL; fallback render inactive.");
            return code;
        }

        // The first injection wins. Keep the fallback, but report if stock moves it first.
        MethodInfo? calculated = CalculatedControlsAnchor();
        int calculatedIndex = calculated == null
            ? -1
            : code.FindIndex(instruction => instruction.Calls(calculated));
        if (calculatedIndex > index)
            LogHelper.WarnOnce("transpiler-hohmann-fallback-order",
                "[AFC] HohmannFallback transpiler: ConsoleStyle.PopWidgetStyle now runs before " +
                "DrawCorrectionTransfer in DrawPlanWindow, so the multi-pass section draws at " +
                "the end of the window body instead of next to the transfer controls.");

        var injected = new CodeInstruction(OpCodes.Call, injectTarget);
        TranspilerInsertion.MoveEntryMarkers(code[index], injected);
        code.Insert(index, injected);

        if (DebugConfig.MultiPass)
            LogHelper.DebugOnce("transpiler-hohmann-fallback",
                "[AFC] HohmannFallback transpiler: injected DrawInline before " +
                "ConsoleStyle.PopWidgetStyle.");
        return code;
    }

    internal static IEnumerable<CodeInstruction> InterceptCreate(
        IEnumerable<CodeInstruction> instructions)
    {
        MethodInfo? burnCreate = AccessTools.Method(
            typeof(Burn), nameof(Burn.Create), BurnCreateSignature);
        MethodInfo? replacement = AccessTools.Method(
            typeof(HohmannCreateInterceptor),
            nameof(HohmannCreateInterceptor.CreateMaybeMultiPass));
        MethodInfo? clickGate = AccessTools.Method(
            typeof(HohmannCreateInterceptor),
            nameof(HohmannCreateInterceptor.ShouldAllowCreateClick));

        var code = new List<CodeInstruction>(instructions);
        if (burnCreate == null || replacement == null)
        {
            LogHelper.WarnOnce("transpiler-create-interceptor-noburncreate",
                "[AFC] HohmannCreateInterceptor transpiler: Burn.Create or its " +
                "replacement not found; leaving this DrawPlanWindow stage unmodified.");
            return code;
        }

        int burnCreateIndex = code.FindIndex(instruction => instruction.Calls(burnCreate));
        int createButtonIndex = FindCreateButton(
            code, burnCreateIndex, clickGate, out int gapTooLarge);

        bool burnCreateReplaced = burnCreateIndex >= 0;
        if (burnCreateReplaced)
        {
            var swap = new CodeInstruction(OpCodes.Call, replacement);
            swap.labels.AddRange(code[burnCreateIndex].labels);
            swap.blocks.AddRange(code[burnCreateIndex].blocks);
            code[burnCreateIndex] = swap;
        }

        bool gateInjected = createButtonIndex >= 0 && clickGate != null;
        if (gateInjected)
            code.Insert(createButtonIndex + 1, new CodeInstruction(OpCodes.Call, clickGate));

        ReportCreateInterception(burnCreateReplaced, gateInjected, gapTooLarge);
        return code;
    }

    private static int FindCreateButton(
        List<CodeInstruction> code, int burnCreateIndex,
        MethodInfo? clickGate, out int gapTooLarge)
    {
        gapTooLarge = 0;
        if (clickGate == null || burnCreateIndex < 0)
            return -1;

        for (int i = burnCreateIndex - 1; i >= 0; i--)
        {
            if (!IsPrimaryButtonCall(code[i]))
                continue;

            int gap = burnCreateIndex - i;
            if (gap <= MaxIlGapPrimaryButtonToBurnCreate)
                return i;

            gapTooLarge = gap;
            break;
        }
        return -1;
    }

    private static bool IsPrimaryButtonCall(CodeInstruction instruction)
        => (instruction.opcode == OpCodes.Call || instruction.opcode == OpCodes.Callvirt)
           && instruction.operand is MethodInfo method
           && method.DeclaringType == typeof(ConsoleWidgets)
           && method.Name == nameof(ConsoleWidgets.PrimaryButton);

    private static void ReportCreateInterception(
        bool burnCreateReplaced, bool gateInjected, int gapTooLarge)
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
                    $"{MaxIlGapPrimaryButtonToBurnCreate}); skipping the click gate.");
            else
                LogHelper.WarnOnce("transpiler-create-interceptor-nogate",
                    "[AFC] HohmannCreateInterceptor transpiler: no preceding " +
                    "PrimaryButton call found near Burn.Create; click gate inactive.");
        }
        else if (DebugConfig.MultiPass)
            LogHelper.DebugOnce("transpiler-create-interceptor-gate",
                "[AFC] HohmannCreateInterceptor transpiler: injected the click gate " +
                "after PrimaryButton.");
    }
}
