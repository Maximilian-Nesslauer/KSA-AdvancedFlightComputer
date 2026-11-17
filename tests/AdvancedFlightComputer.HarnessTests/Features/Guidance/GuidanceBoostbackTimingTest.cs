using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceBoostbackTimingTest : AfcTest
{
    public override string Name => "afc-guidance-boostback-timing";

    protected override void Execute(TestContext t)
    {
        t.Check("a short return cannot wait until the apex for a small correction",
            !GuidanceWindow.HasSmallBoostbackCoastTime(62, 28, 14, 30, 8, 45));
        t.Check("the same short return must stop waiting for an engine-off turn as the ground approaches",
            !GuidanceWindow.HasSmallBoostbackCoastTime(35, 0, 14, 30, 8, 45));
        t.Check("a longer return can still wait for the apex and turn on RCS",
            GuidanceWindow.HasSmallBoostbackCoastTime(180, 28, 14, 30, 8, 45));
        t.Check("a craft without RCS authority cannot turn with the engine off",
            !GuidanceWindow.HasSmallBoostbackCoastTime(180, 0, double.PositiveInfinity, 30, 8, 45));
    }
}
