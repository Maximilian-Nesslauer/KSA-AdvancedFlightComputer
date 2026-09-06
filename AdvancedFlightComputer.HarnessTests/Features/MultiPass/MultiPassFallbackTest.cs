using System.Reflection;
using System.Reflection.Emit;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class MultiPassFallbackTest : AfcTest
{
    public override string Name => "afc-multipass-fallback";

    protected override void Execute(TestContext t)
    {
        MethodInfo target = AccessTools.Method(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), [typeof(IGameViewport)]);
        MethodInfo anchor = AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle), Type.EmptyTypes);
        MethodInfo inline = AccessTools.Method(typeof(HohmannMultiPassUI), nameof(HohmannMultiPassUI.DrawInline));
        List<CodeInstruction> original = PatchProcessor.GetOriginalInstructions(target);
        List<CodeInstruction> patched = Run(original);
        int call = patched.FindIndex(ins => ins.Calls(inline));
        t.Check("current game anchor receives one fallback", patched.FindAll(ins => ins.Calls(inline)).Count == 1
            && call >= 0 && patched[call + 1].Calls(anchor));

        var generator = new DynamicMethod("markers", typeof(void), Type.EmptyTypes).GetILGenerator();
        Label label = generator.DefineLabel();
        var boundary = new CodeInstruction(OpCodes.Call, anchor);
        boundary.labels.Add(label);
        boundary.blocks.Add(new ExceptionBlock(ExceptionBlockType.BeginExceptionBlock));
        boundary.blocks.Add(new ExceptionBlock(ExceptionBlockType.EndExceptionBlock));
        var laterAnchor = new CodeInstruction(OpCodes.Call, anchor);
        List<CodeInstruction> synthetic = Run([boundary, laterAnchor]);
        t.Check("only first anchor is used", synthetic.Count == 3 && synthetic[0].Calls(inline)
            && synthetic[1] == boundary && synthetic[2] == laterAnchor);
        t.Check("branch labels enter inserted call", synthetic[0].labels.Contains(label) && boundary.labels.Count == 0);
        t.Check("begin marker moves before insertion", synthetic[0].blocks.Count == 1
            && synthetic[0].blocks[0].blockType == ExceptionBlockType.BeginExceptionBlock);
        t.Check("end marker stays after anchor", boundary.blocks.Count == 1
            && boundary.blocks[0].blockType == ExceptionBlockType.EndExceptionBlock);
        var noAnchor = new CodeInstruction(OpCodes.Ret);
        List<CodeInstruction> unchanged = Run([noAnchor]);
        t.Check("missing anchor leaves IL unchanged", unchanged.Count == 1 && unchanged[0] == noAnchor);
    }

    private static List<CodeInstruction> Run(IEnumerable<CodeInstruction> instructions)
        => new(PlanWindowPatchPipeline.InjectFallbackControls(instructions));
}
