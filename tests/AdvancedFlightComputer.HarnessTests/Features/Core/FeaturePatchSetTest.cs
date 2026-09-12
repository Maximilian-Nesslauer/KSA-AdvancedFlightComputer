using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class FeaturePatchSetTest : AfcTest
{
    public override string Name => "afc-feature-patch-rollback";

    protected override void Execute(TestContext t)
    {
        FeaturePatchSet patches = new("com.maxi.afc.harnesstests.rollback");
        Harmony other = new("com.maxi.afc.harnesstests.rollback-other");
        try
        {
            Patch(other, nameof(OtherTarget), nameof(AddTen));
            t.Check("independent owner starts patched", OtherTarget() == 11);
            t.Check("first feature applies", patches.TryApply("First",
                harmony => Patch(harmony, nameof(FirstTarget), nameof(AddTen))));
            t.Check("first feature changes its target", FirstTarget() == 11);

            bool applied = patches.TryApply("Failing", harmony =>
            {
                Patch(harmony, nameof(FirstTarget), nameof(AddHundred));
                Patch(harmony, nameof(OtherTarget), nameof(AddHundred));
                throw new InvalidOperationException("Expected patch-block failure");
            });
            t.Check("partial block reports failure", !applied);
            t.Check("rollback preserves earlier feature", FirstTarget() == 11);
            t.Check("rollback preserves independent owner", OtherTarget() == 11);

            t.Check("later feature still applies", patches.TryApply("Later",
                harmony => Patch(harmony, nameof(OtherTarget), nameof(AddHundred))));
            t.Check("later feature changes its target", OtherTarget() == 111);
            patches.UnpatchAll();
            t.Check("unload removes every feature owner", FirstTarget() == 1 && OtherTarget() == 11);
            patches.UnpatchAll();
            t.Check("repeated unload leaves independent owner", OtherTarget() == 11);
        }
        finally
        {
            patches.UnpatchAll();
            other.UnpatchAll(other.Id);
        }
    }

    private static void Patch(Harmony harmony, string target, string postfix)
        => harmony.Patch(AccessTools.Method(typeof(FeaturePatchSetTest), target, Type.EmptyTypes),
            postfix: new HarmonyMethod(typeof(FeaturePatchSetTest), postfix));

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int FirstTarget() => 1;

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int OtherTarget() => 1;

    private static void AddTen(ref int __result) => __result += 10;

    private static void AddHundred(ref int __result) => __result += 100;
}
