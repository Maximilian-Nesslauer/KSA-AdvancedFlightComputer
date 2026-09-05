using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class ManeuverTranspilerTest : AfcTest
{
    public override string Name => "afc-maneuver-transpilers";

    protected override void Execute(TestContext t)
    {
        MethodInfo draw = AccessTools.Method(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), [typeof(IGameViewport)]);
        List<CodeInstruction> original = PatchProcessor.GetOriginalInstructions(draw);
        List<CodeInstruction> multiPass = Run(typeof(Patch_DrawPlanWindow_HohmannMultiPass), original);
        MethodInfo inline = AccessTools.Method(typeof(HohmannMultiPassUI), nameof(HohmannMultiPassUI.DrawInline));
        MethodInfo correction = AccessTools.Method(typeof(TransferPlanner), "DrawCorrectionTransfer", Type.EmptyTypes);
        int inlineIndex = multiPass.FindIndex(ins => ins.Calls(inline));
        t.Check("multi-pass is inserted before correction",
            CountCalls(multiPass, inline) == 1 && inlineIndex >= 0 && multiPass[inlineIndex + 1].Calls(correction));

        List<CodeInstruction> intercepted = Run(typeof(Patch_DrawPlanWindow_CreateInterceptor), multiPass);
        MethodInfo replacement = AccessTools.Method(typeof(HohmannCreateInterceptor), nameof(HohmannCreateInterceptor.CreateMaybeMultiPass));
        MethodInfo gate = AccessTools.Method(typeof(HohmannCreateInterceptor), nameof(HohmannCreateInterceptor.ShouldAllowCreateClick));
        t.Check("create interception composes with inline injection",
            CountCalls(intercepted, replacement) == 1 && CountCalls(intercepted, gate) == 1
            && CountCalls(intercepted, inline) == 1);

        CheckMenu(t);
        CheckExceptionRegions(t);
    }

    private static void CheckMenu(TestContext t)
    {
        MethodInfo draw = AccessTools.Method(typeof(BurnContextMenu), nameof(BurnContextMenu.Draw), Type.EmptyTypes);
        List<CodeInstruction> patched = Run(typeof(Patch_BurnContextMenu_Launcher), PatchProcessor.GetOriginalInstructions(draw));
        MethodInfo inline = AccessTools.Method(typeof(BurnMenuLauncher), nameof(BurnMenuLauncher.DrawInline));
        MethodInfo apse = AccessTools.Method(typeof(BurnMenuLauncher), nameof(BurnMenuLauncher.DrawApsisEntry));
        t.Check("menu has one main shortcut group and two apse shortcuts",
            CountCalls(patched, inline) == 1 && CountCalls(patched, apse) == 2);

        int apseIndex = patched.FindIndex(ins => ins.Calls(apse));
        int secondApseIndex = patched.FindIndex(apseIndex + 1, ins => ins.Calls(apse));
        t.Check("apse shortcut arguments follow stock submenu order",
            apseIndex > 0 && secondApseIndex > 0
            && patched[apseIndex - 1].opcode == OpCodes.Ldc_I4
            && Equals(patched[apseIndex - 1].operand, BurnMenuLauncher.PeriapsisSubmenu)
            && patched[secondApseIndex - 1].opcode == OpCodes.Ldc_I4
            && Equals(patched[secondApseIndex - 1].operand, BurnMenuLauncher.ApoapsisSubmenu));
    }

    private static void CheckExceptionRegions(TestContext t)
    {
        var harmony = new Harmony("afc.tests.maneuver-insertion-boundaries");
        try
        {
            foreach (bool atEnd in new[] { false, true })
            {
                string fixture = atEnd ? nameof(EndBoundaryFixture) : nameof(BeginBoundaryFixture);
                MethodInfo original = AccessTools.Method(typeof(ManeuverTranspilerTest), fixture);
                MethodInfo transpiler = AccessTools.Method(typeof(ManeuverTranspilerTest),
                    atEnd ? nameof(InsertAtEnd) : nameof(InsertAtBegin));
                harmony.Patch(original, transpiler: new HarmonyMethod(transpiler));
                t.Check(fixture + " catches the inserted exception", (int)original.Invoke(null, null)! == 42);
            }
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int BeginBoundaryFixture()
    {
        try { BoundaryAnchor(); }
        catch (InvalidOperationException) { return 42; }
        return 0;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int EndBoundaryFixture()
    {
        try
        {
            try { BoundaryAnchor(); }
            finally { BoundaryAnchor(); }
        }
        catch (InvalidOperationException) { return 42; }
        return 0;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BoundaryAnchor() { }

    private static void ThrowForBoundary() => throw new InvalidOperationException("Boundary test");

    private static IEnumerable<CodeInstruction> InsertAtBegin(IEnumerable<CodeInstruction> code)
        => InsertAtBoundary(code, ExceptionBlockType.BeginExceptionBlock);

    private static IEnumerable<CodeInstruction> InsertAtEnd(IEnumerable<CodeInstruction> code)
        => InsertAtBoundary(code, ExceptionBlockType.EndExceptionBlock);

    private static IEnumerable<CodeInstruction> InsertAtBoundary(IEnumerable<CodeInstruction> code, ExceptionBlockType boundary)
    {
        bool inserted = false;
        foreach (CodeInstruction instruction in code)
        {
            if (!inserted && instruction.blocks.Exists(block => block.blockType == boundary))
            {
                var call = new CodeInstruction(OpCodes.Call, AccessTools.Method(typeof(ManeuverTranspilerTest), nameof(ThrowForBoundary)));
                TranspilerInsertion.MoveEntryMarkers(instruction, call);
                yield return call;
                inserted = true;
            }
            yield return instruction;
        }
    }

    private static List<CodeInstruction> Run(Type patch, IEnumerable<CodeInstruction> code)
    {
        MethodInfo transpiler = patch.GetMethod("Transpiler", BindingFlags.Static | BindingFlags.NonPublic)!;
        return new List<CodeInstruction>((IEnumerable<CodeInstruction>)transpiler.Invoke(null, [code])!);
    }

    private static int CountCalls(List<CodeInstruction> code, MethodInfo method)
        => code.FindAll(ins => ins.Calls(method)).Count;
}
