using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceBoostbackTimingTest : AfcTest
{
    public override string Name => "afc-guidance-boostback-timing";

    protected override void Execute(TestContext t)
    {
        t.Check("a short return cannot wait until the apex for a small correction",
            !GuidanceWindow.HasSmallBoostbackCoastTime(80, 28, 14, 30, 8, 45));
        t.Check("the same return has time to turn on RCS if it starts now",
            GuidanceWindow.HasSmallBoostbackCoastTime(80, 0, 14, 30, 8, 45));
        t.Check("the same short return must stop waiting for an engine-off turn as the ground approaches",
            !GuidanceWindow.HasSmallBoostbackCoastTime(35, 0, 14, 30, 8, 45));
        t.Check("a longer return can still wait for the apex and turn on RCS",
            GuidanceWindow.HasSmallBoostbackCoastTime(180, 28, 14, 30, 8, 45));
        t.Check("a craft without RCS authority cannot turn with the engine off",
            !GuidanceWindow.HasSmallBoostbackCoastTime(180, 0, double.PositiveInfinity, 30, 8, 45));
        t.Check("roll-only authority cannot turn the thrust axis",
            !GuidanceWindow.HasBoostbackTurnAuthority(new float3(1, 0, 0)));
        t.Check("turning requires both lateral axes",
            !GuidanceWindow.HasBoostbackTurnAuthority(new float3(0, 1, 0))
            && GuidanceWindow.HasBoostbackTurnAuthority(new float3(0, 1, 1)));
        t.Check("coasting needs a known impact and burn budget",
            !GuidanceWindow.HasSmallBoostbackCoastTime(double.NaN, 0, 14, 30, 8, 45)
            && !GuidanceWindow.HasSmallBoostbackCoastTime(180, 0, 14, 30, double.NaN, 45));

        var ambient = AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));
        var planTimeLeft = AccessTools.Method(typeof(GuidanceWindow), "BoostbackPlanTgo")
            .CreateDelegate<Func<double, double>>();
        var previous = ambient();
        var state = new VehicleAutopilotState
        {
            BoostbackPhase = GuidanceWindow.BoostbackPhase.Rotation,
            BoostbackHasPlan = true,
            BoostbackPlan = new BurnParameters { Duration = 0.75 },
            BoostbackPlanTime = 10.0
        };
        try
        {
            ambient() = state;
            t.Check("a subsecond boostback keeps its whole burn while turning", planTimeLeft(12.0) == 0.75);
            state.BoostbackPhase = GuidanceWindow.BoostbackPhase.Boostback;
            state.BoostbackPlanTime = 12.0;
            t.Check("time and cutoff advance once the boostback starts",
                planTimeLeft(12.25) == 0.5 && planTimeLeft(12.75) == 0.0);
        }
        finally
        {
            ambient() = previous;
        }
    }
}
