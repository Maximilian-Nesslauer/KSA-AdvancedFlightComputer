using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using System.Collections.ObjectModel;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.PlanWindow;
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
        List<CodeInstruction> patched = Run(typeof(Patch_DrawPlanWindow), original);
        MethodInfo inline = AccessTools.Method(typeof(HohmannMultiPassUI), nameof(HohmannMultiPassUI.DrawInline));
        MethodInfo correction = AccessTools.Method(typeof(TransferPlanner), "DrawCorrectionTransfer", Type.EmptyTypes);
        MethodInfo popStyle = AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle), Type.EmptyTypes);
        int correctionIndex = patched.FindIndex(ins => ins.Calls(correction));
        int popStyleIndex = patched.FindIndex(ins => ins.Calls(popStyle));
        t.Check("one pipeline inserts both multi-pass draw callbacks",
            CountCalls(patched, inline) == 2
            && correctionIndex > 0 && patched[correctionIndex - 1].Calls(inline)
            && popStyleIndex > 0 && patched[popStyleIndex - 1].Calls(inline));

        MethodInfo replacement = AccessTools.Method(typeof(HohmannCreateInterceptor), nameof(HohmannCreateInterceptor.CreateMaybeMultiPass));
        MethodInfo gate = AccessTools.Method(typeof(HohmannCreateInterceptor), nameof(HohmannCreateInterceptor.ShouldAllowCreateClick));
        t.Check("the same pipeline installs create interception",
            CountCalls(patched, replacement) == 1 && CountCalls(patched, gate) == 1);

        CheckMenu(t);
        CheckExceptionRegions(t);
        CheckInstalledPatchCounts(t, draw);
    }

    private static void CheckInstalledPatchCounts(TestContext t, MethodInfo draw)
    {
        const string id = "afc.tests.plan-window-patch-counts";
        var harmony = new Harmony(id);
        MethodInfo preRender = AccessTools.Method(
            typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender), [typeof(IViewport)]);
        try
        {
            MethodInfo apply = typeof(AdvancedFlightComputer.Mod).GetMethod(
                "PatchPlanWindow", BindingFlags.Static | BindingFlags.NonPublic)!;
            apply.Invoke(null, [harmony]);

            Patches? drawPatches = Harmony.GetPatchInfo(draw);
            t.Check("DrawPlanWindow has one patch of each kind",
                CountOwned(drawPatches?.Prefixes, id) == 1
                && CountOwned(drawPatches?.Transpilers, id) == 1
                && CountOwned(drawPatches?.Postfixes, id) == 1);

            Patches? preRenderPatches = Harmony.GetPatchInfo(preRender);
            t.Check("OnPreRender has one AFC hook",
                CountOwned(preRenderPatches?.Prefixes, id) == 0
                && CountOwned(preRenderPatches?.Transpilers, id) == 0
                && CountOwned(preRenderPatches?.Postfixes, id) == 1);

            MethodInfo selectedTransfer = AccessTools.Method(
                typeof(TransferPlanner), "DrawSelectedTransfer", [typeof(IViewport)]);
            MethodInfo selectedTransferUi = AccessTools.Method(
                typeof(TransferPlanner), "DrawSelectedTransferUi", [typeof(IGameViewport)]);
            t.Check("PlanWindow owns both flyby suppression hooks",
                CountOwned(Harmony.GetPatchInfo(selectedTransfer)?.Prefixes, id) == 1
                && CountOwned(Harmony.GetPatchInfo(selectedTransferUi)?.Prefixes, id) == 1);
        }
        finally
        {
            harmony.UnpatchAll(id);
        }
    }

    private static int CountOwned(ReadOnlyCollection<Patch>? patches, string owner)
        => patches?.Count(patch => patch.owner == owner) ?? 0;

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
