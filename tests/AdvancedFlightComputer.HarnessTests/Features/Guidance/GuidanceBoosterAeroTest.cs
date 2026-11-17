using System.Reflection;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A booster's aero, measured before it separates, against the same booster measured after (KsaBoosterGeometry).
//
// The convex plan prices a booster's return, and its landing burn through its terminal velocity, from a sweep of the
// game's own force function. That sweep once used the whole stack: a heavy's core, three cores wide in the stack, was
// priced at about half its real terminal velocity and came down short of its landing burn (2026-11-17). Now each
// booster is swept as the vehicle it becomes - its own colliders' box along its root part's axes, the properties the
// game derives from it - and that has to be what the game measures once it really is that vehicle.
//
// For each separating row of each save that carries a booster with a command pod: sweep the booster still attached,
// fire the row, find the vehicle the booster became and sweep it. The two must agree on the box, the reference area
// and the drag coefficients, and so on the drag area the terminal velocity comes from. Run under stock aero and under
// KSAero. Saves that are not installed are skipped.
public sealed class GuidanceBoosterAeroTest : AfcTest
{
    public override string Name => "afc-guidance-booster-aero";

    private static readonly string[] Saves = { "2stage_controlled_heavy", "boosters_controlled", "2stage_controlled" };
    private const int MaxRows = 8;

    private static readonly MethodInfo SequenceRoots = AccessTools.Method(typeof(GuidanceWindow), "SequenceRoots");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        int ran = 0;
        foreach (string save in Saves)
        {
            ran += Run(t, home, save, ksaero: false);
            using KsaeroScope aero = KsaeroScope.Apply();
            if (aero.Active)
                ran += Run(t, home, save, ksaero: true);
            else
                t.Info($"KSAero not loaded ({aero.Status}): stock aero only.");
        }
        if (ran == 0)
            t.Skip("none of the booster saves is installed.");
    }

    private static int Run(TestContext t, IParentBody home, string save, bool ksaero)
    {
        string tag = $"{save}{(ksaero ? " (KSAero)" : "")}";
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        int compared = 0;
        try
        {
            Vehicle stack;
            try
            {
                stack = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessBoosterAero",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Info($"{save}: {e.Message}");
                return 0;
            }
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 10);

            if (!KsaAeroSweep.TryBuild(stack, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result stackAero, out string stackError))
            {
                t.Fail(tag, "the stack could not be swept: " + stackError);
                return 0;
            }

            for (int row = 0; row < MaxRows; row++)
            {
                Sequence? next = NextRow(stack);
                if (next == null)
                    break;

                // Every booster this row drops that is a vehicle of its own - a pod aboard - swept while still attached.
                var roots = new List<Part>();
                SequenceRoots.Invoke(null, new object[] { next, roots });
                var before = new List<(Part Root, KsaAeroSweep.Result Aero)>();
                foreach (Part root in roots)
                {
                    if (!HasControl(stack, root))
                        continue;
                    if (KsaAeroSweep.TryBuildBooster(stack, root, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result pre, out string preError))
                        before.Add((root, pre));
                    else
                        t.Fail($"{tag}: '{root.DisplayName}'", "could not be swept attached: " + preError);
                }

                stack.Parts.SequenceList.ActivateNextSequence(stack);
                driver.Step(0.05, 20);

                foreach ((Part root, KsaAeroSweep.Result pre) in before)
                {
                    Vehicle? split = VehicleWith(t, root.InstanceId, stack);
                    if (split == null)
                    {
                        t.Fail($"{tag}: '{root.DisplayName}'", "no separated vehicle carries it");
                        continue;
                    }
                    if (!KsaAeroSweep.TryBuild(split, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result post, out string postError))
                    {
                        t.Fail($"{tag}: '{root.DisplayName}'", "could not be swept separated: " + postError);
                        continue;
                    }
                    Compare(t, $"{tag}: '{root.DisplayName}'", pre, post, stackAero);
                    compared++;
                }
            }
            if (compared == 0)
                t.Info($"{tag}: no row drops a booster with a pod.");
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
        return 1;
    }

    private static void Compare(TestContext t, string what, KsaAeroSweep.Result pre, KsaAeroSweep.Result post, KsaAeroSweep.Result stack)
    {
        static bool Near(double a, double b, double frac) => Math.Abs(a - b) <= frac * Math.Max(Math.Abs(a), Math.Abs(b)) + 1e-6;
        t.Info($"{what}: attached box {pre.BoxExtents.X:F2} x {pre.BoxExtents.Y:F2} x {pre.BoxExtents.Z:F2} m, separated {post.BoxExtents.X:F2} x {post.BoxExtents.Y:F2} x {post.BoxExtents.Z:F2} m; "
               + $"Cd tail-first {pre.CdTailFirst:F3} / {post.CdTailFirst:F3} on {pre.ReferenceArea:F2} / {post.ReferenceArea:F2} m^2; stack {stack.CdTailFirst:F3} on {stack.ReferenceArea:F2} m^2");
        t.Check($"{what}: the box is the separated vehicle's",
            Near(pre.BoxExtents.X, post.BoxExtents.X, 0.02) && Near(pre.BoxExtents.Y, post.BoxExtents.Y, 0.02) && Near(pre.BoxExtents.Z, post.BoxExtents.Z, 0.02));
        t.Check($"{what}: and so its reference area", Near(pre.ReferenceArea, post.ReferenceArea, 0.03), $"{pre.ReferenceArea:F2} against {post.ReferenceArea:F2} m^2");
        t.Check($"{what}: its drag tail-first, broadside and nose-first",
            Near(pre.CdTailFirst, post.CdTailFirst, 0.03) && Near(pre.CdBroadside, post.CdBroadside, 0.05) && Near(pre.CdNoseFirst, post.CdNoseFirst, 0.03),
            $"{pre.CdTailFirst:F3}/{pre.CdBroadside:F3}/{pre.CdNoseFirst:F3} against {post.CdTailFirst:F3}/{post.CdBroadside:F3}/{post.CdNoseFirst:F3}");
        // The terminal velocity goes as 1 / sqrt(Cd A): the number the landing reserve is sized from.
        double preCdA = pre.Table.Cd(0.3, 0.0) * pre.ReferenceArea, postCdA = post.Table.Cd(0.3, 0.0) * post.ReferenceArea;
        double stackCdA = stack.Table.Cd(0.3, 0.0) * stack.ReferenceArea;
        t.Check($"{what}: its drag area falling tail-first", Near(preCdA, postCdA, 0.03),
            $"{preCdA:F2} against {postCdA:F2} m^2; the stack's {stackCdA:F2} m^2 put its terminal velocity {Math.Sqrt(postCdA / stackCdA):F2} x its own");
    }

    private static bool HasControl(Vehicle vehicle, Part root)
    {
        HashSet<Part> parts = KsaBoosterGeometry.Subtree(root);
        foreach (Control c in vehicle.Parts.Modules.Get<Control>())
            if (parts.Contains(c.Parent.FullPart))
                return true;
        return false;
    }

    private static Vehicle? VehicleWith(TestContext t, uint rootId, Vehicle not)
    {
        foreach (Astronomical body in t.System.All.AsSpan())
        {
            if (body is not Vehicle v || ReferenceEquals(v, not) || v is KittenEva)
                continue;
            ReadOnlySpan<Part> parts = v.Parts.Parts;
            for (int i = 0; i < parts.Length; i++)
                if (parts[i].InstanceId == rootId)
                    return v;
        }
        return null;
    }

    private static Sequence? NextRow(Vehicle vehicle)
    {
        ReadOnlySpan<Sequence> sequences = vehicle.Parts.SequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
            if (!sequences[i].Activated && !sequences[i].Parts.IsEmpty)
                return sequences[i];
        return null;
    }
}
