using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using System.Text;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The orbit right-click shortcuts against the current game IL, and the exception boundaries of the
// shared transpiler insertion with Harmony-patched test methods. AFC has no transpiler on the planner.
public sealed class ManeuverTranspilerTest : AfcTest
{
    public override string Name => "afc-maneuver-transpilers";

    protected override void Execute(TestContext t)
    {
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

        // Also verify the headings because the submenu arguments alone do not prove placement.
        string? firstHeading = EnclosingMenuHeading(patched, apseIndex);
        string? secondHeading = EnclosingMenuHeading(patched, secondApseIndex);
        t.Check("apse shortcuts sit under their own headings",
            firstHeading == "At Periapsis" && secondHeading == "At Apoapsis",
            $"first='{firstHeading ?? "none"}' second='{secondHeading ?? "none"}'");
    }

    // Find the matching BeginMenu while accounting for nested menus.
    private static string? EnclosingMenuHeading(List<CodeInstruction> code, int index)
    {
        int depth = 0;
        for (int i = index - 1; i >= 0; i--)
        {
            if (CallsImGui(code[i], "EndMenu"))
                depth++;
            else if (CallsImGui(code[i], "BeginMenu"))
            {
                if (depth == 0)
                    return MenuLabel(code, i);
                depth--;
            }
        }
        return null;
    }

    // Match the operand because this test project does not reference the ImGui assembly.
    private static bool CallsImGui(CodeInstruction instruction, string name)
        => (instruction.opcode == OpCodes.Call || instruction.opcode == OpCodes.Callvirt)
           && instruction.operand is MethodInfo method
           && method.Name == name
           && method.DeclaringType?.Name == "ImGui";

    // UTF-8 literals use compiler-generated data fields with a trailing null byte.
    private static string? MenuLabel(List<CodeInstruction> code, int beginMenuIndex)
    {
        for (int i = beginMenuIndex - 1; i >= 0 && i >= beginMenuIndex - 8; i--)
        {
            if (code[i].opcode != OpCodes.Ldsflda || code[i].operand is not FieldInfo data)
                continue;
            try
            {
                return Encoding.UTF8.GetString(
                    RuntimeHelpers.CreateSpan<byte>(data.FieldHandle).TrimEnd((byte)0));
            }
            catch (Exception)
            {
                // This static field does not contain a readable literal.
            }
        }
        return null;
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
