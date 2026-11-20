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
        // The coast-time rule for small boostbacks (HasSmallBoostbackCoastTime) is gone: a small boostback always waits for the apex and turns engine-off, never settling or turning lit (2026-11-20).
        t.Check("roll-only authority cannot turn the thrust axis",
            !GuidanceWindow.HasBoostbackTurnAuthority(new float3(1, 0, 0)));
        t.Check("turning requires both lateral axes",
            !GuidanceWindow.HasBoostbackTurnAuthority(new float3(0, 1, 0))
            && GuidanceWindow.HasBoostbackTurnAuthority(new float3(0, 1, 1)));

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
