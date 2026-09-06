using Brutal.Numerics;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Pure tests for the Auto attitude decision. The propellant a strategy would need (RcsEstimates.RequiredPropellantKg, which the sufficiency warning and the activation alert share) and the Hold vs Align resolution (RcsExecutor.ResolveStrategy), including the preference margin that keeps Auto from slewing for a marginal saving. Estimates are hand built, so no vehicle is involved.
public sealed class RcsEstimatesTest : AfcTest
{
    public override string Name => "afc-rcs-estimates";

    protected override void Execute(TestContext t)
    {
        CheckRequiredPropellant(t);
        CheckResolveStrategy(t);
        CheckRotationCost(t);
        CheckGroupTorqueCost(t);
    }

    // Hold 100 kg, Align 40 + 20 = 60 kg total, both feasible.
    private static RcsEstimates BothFeasible() => new()
    {
        Valid = true,
        HoldFeasible = true,
        HoldPropellantKg = 100.0,
        HoldDurationSec = 50.0,
        AlignFeasible = true,
        AlignPropellantKg = 40.0,
        AlignSlewPropellantKg = 20.0,
        AlignDurationSec = 30.0,
        AlignSlewDurationSec = 10.0,
        AlignAxis = 0,
    };

    private static void CheckRequiredPropellant(TestContext t)
    {
        RcsEstimates est = BothFeasible();

        Near(t, "hold required", est.RequiredPropellantKg(RcsAttitudeStrategy.Hold), 100.0);
        Near(t, "align required", est.RequiredPropellantKg(RcsAttitudeStrategy.Align), 60.0);
        // Auto takes the cheaper feasible strategy.
        Near(t, "auto required", est.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 60.0);

        // Only Hold feasible. Auto falls to Hold, Align reports nothing.
        RcsEstimates holdOnly = est;
        holdOnly.AlignFeasible = false;
        Near(t, "hold-only auto", holdOnly.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 100.0);
        Near(t, "hold-only align zero", holdOnly.RequiredPropellantKg(RcsAttitudeStrategy.Align), 0.0);

        // Only Align feasible. Auto falls to Align, Hold reports nothing.
        RcsEstimates alignOnly = est;
        alignOnly.HoldFeasible = false;
        Near(t, "align-only auto", alignOnly.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 60.0);
        Near(t, "align-only hold zero", alignOnly.RequiredPropellantKg(RcsAttitudeStrategy.Hold), 0.0);

        // Neither feasible. Nothing is required.
        RcsEstimates none = est;
        none.HoldFeasible = false;
        none.AlignFeasible = false;
        Near(t, "none required", none.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 0.0);
    }

    private static void CheckResolveStrategy(TestContext t)
    {
        RcsEstimates est = BothFeasible();

        // Explicit Hold is always Hold.
        var (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Hold, in est);
        t.Check("explicit hold", s == RcsAttitudeStrategy.Hold && ax == -1);

        // Explicit Align with a feasible axis aligns.
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Align, in est);
        t.Check("explicit align feasible", s == RcsAttitudeStrategy.Align && ax == 0);

        // Explicit Align with no feasible axis degrades to Hold.
        RcsEstimates noAlign = est;
        noAlign.AlignFeasible = false;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Align, in noAlign);
        t.Check("explicit align infeasible", s == RcsAttitudeStrategy.Hold && ax == -1);

        // Auto. Align total 60 clears 0.9 x Hold (90), so Auto slews.
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in est);
        t.Check("auto align cheaper", s == RcsAttitudeStrategy.Align && ax == 0);

        // Auto. Align total 95 does not clear the 90 margin, so Auto holds rather than pay a slew for a marginal saving.
        RcsEstimates marginal = est;
        marginal.AlignPropellantKg = 75.0;
        marginal.AlignSlewPropellantKg = 20.0;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in marginal);
        t.Check("auto margin holds", s == RcsAttitudeStrategy.Hold && ax == -1);

        // Auto. Hold infeasible but Align feasible forces the slew.
        RcsEstimates noHold = est;
        noHold.HoldFeasible = false;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in noHold);
        t.Check("auto hold infeasible", s == RcsAttitudeStrategy.Align && ax == 0);

        // Auto. Neither feasible degrades to Hold (the activation feasibility check refuses it afterwards).
        RcsEstimates none = est;
        none.HoldFeasible = false;
        none.AlignFeasible = false;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in none);
        t.Check("auto none feasible", s == RcsAttitudeStrategy.Hold && ax == -1);
    }

    private static void CheckRotationCost(TestContext t)
    {
        Near(t, "rotation flow per torque", RcsExecutor.RotationFlowPerTorque(2f, 8f), 0.25);
        Near(t, "no rotation authority", RcsExecutor.RotationFlowPerTorque(2f, 0f), 0.0);
        Near(t, "rotation authority threshold", RcsExecutor.RotationFlowPerTorque(2f, 1f), 0.0);
        Near(t, "no rotation flow", RcsExecutor.RotationFlowPerTorque(0f, 8f), 0.0);
    }

    private static void CheckGroupTorqueCost(TestContext t)
    {
        RcsCapabilitySnapshot cap = new()
        {
            Ax0 = new RcsAxisGroup { ForceN = 10f, MassFlowKgS = 1f, TorqueNm = new float3(0f, 20f, 0f) },
            RotationMassFlowKgS = new float3(0f, 2f, 0f),
            RotationTorqueNm = new float3(0f, 8f, 0f),
        };
        // One N s of translation leaves two N m s, requiring half a kilogram at this rotation group performance.
        Near(t, "group torque cost", RcsExecutor.GroupAttitudeFightPerImpulse(in cap, double3.UnitX), 0.5);
        cap.Ax0.TorqueNm = default;
        Near(t, "balanced group torque cost", RcsExecutor.GroupAttitudeFightPerImpulse(in cap, double3.UnitX), 0.0);
    }

    private static void Near(TestContext t, string label, double actual, double expected)
        => t.CheckRel(label, actual, expected, 1e-6, floor: 1.0);
}
