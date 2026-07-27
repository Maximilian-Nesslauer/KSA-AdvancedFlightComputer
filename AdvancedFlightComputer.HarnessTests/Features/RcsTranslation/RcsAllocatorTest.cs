using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the pure allocation math of the RCS translation executor: the
// per-axis impulse shaping (cap and minimum-impulse suppression), the
// per-thruster group pulse derivation, and the Hold-strategy performance
// model that feeds the Auto attitude decision. No vehicle involved.
public sealed class RcsAllocatorTest : AfcTest
{
    private const double FloatTol = 1e-4;

    public override string Name => "afc-rcs-allocator";

    protected override void Execute(TestContext t)
    {
        CheckShapeAxis(t);
        CheckMaxAxisPulse(t);
        CheckHoldPerformance(t);
        CheckCompletionFloor(t);
        CheckRemainingDuration(t);
        CheckCapabilityHelpers(t);
        CheckAttitudeFight(t);
    }

    private static void CheckShapeAxis(TestContext t)
    {
        // Cap at one control period of group force.
        t.CheckAbs("cap positive", RcsComputeControlPatch.ShapeAxis(
            50f, forcePos: 100f, forceNeg: 0f, minImpPos: 10f, minImpNeg: 0f, maxPulse: 0.1f), 10f, FloatTol);
        // Below half the minimum impulse: suppressed.
        t.CheckAbs("min impulse floor", RcsComputeControlPatch.ShapeAxis(
            4f, 100f, 0f, 10f, 0f, 0.1f), 0f, FloatTol);
        // At/above half the minimum impulse: passes through uncapped.
        t.CheckAbs("small but usable", RcsComputeControlPatch.ShapeAxis(
            6f, 100f, 0f, 10f, 0f, 0.1f), 6f, FloatTol);
        // No group in the demanded direction: nothing to fire.
        t.CheckAbs("missing negative group", RcsComputeControlPatch.ShapeAxis(
            -50f, 100f, 0f, 10f, 0f, 0.1f), 0f, FloatTol);
        // Negative direction with its own group.
        t.CheckAbs("cap negative", RcsComputeControlPatch.ShapeAxis(
            -50f, 0f, 100f, 0f, 10f, 0.1f), -10f, FloatTol);
    }

    private static void CheckMaxAxisPulse(TestContext t)
    {
        // Participating thruster fires for J / F_group.
        t.CheckAbs("positive pulse", RcsComputeControlPatch.MaxAxisPulse(
            0f, j: 50f, thrusterForce: 25f, groupForcePos: 100f, groupForceNeg: 0f), 0.5f, FloatTol);
        t.CheckAbs("negative pulse", RcsComputeControlPatch.MaxAxisPulse(
            0f, -50f, -25f, 0f, 100f), 0.5f, FloatTol);
        // Wrong-sign thruster does not participate.
        t.CheckAbs("non-participating", RcsComputeControlPatch.MaxAxisPulse(
            0f, 50f, -25f, 100f, 100f), 0f, FloatTol);
        // Max across axes, not sum.
        t.CheckAbs("max not sum", RcsComputeControlPatch.MaxAxisPulse(
            0.8f, 50f, 25f, 100f, 0f), 0.8f, FloatTol);
    }

    private static void CheckHoldPerformance(TestContext t)
    {
        RcsCapabilitySnapshot cap = default;
        var group = new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 1f };
        for (int i = 0; i < 6; i++)
            cap.Set(i, in group);
        cap.HasAnyTranslation = true;

        // Pure single-axis direction: full group force, single group flow.
        bool feasible = RcsExecutor.TryHoldPerformance(
            in cap, new double3(1.0, 0.0, 0.0), out double force, out double flow);
        t.Check("single axis feasible", feasible);
        t.CheckRel("single axis force", force, 100.0, 1e-6, floor: 1.0);
        t.CheckRel("single axis flow", flow, 0.1, 1e-6, floor: 1.0);

        // 45 degree in-plane direction: net force rises to F/cos45 with both
        // groups duty-cycled at full force, flow doubles.
        double s = Math.Sqrt(0.5);
        feasible = RcsExecutor.TryHoldPerformance(
            in cap, new double3(s, s, 0.0), out force, out flow);
        t.Check("diagonal feasible", feasible);
        t.CheckRel("diagonal force", force, 100.0 / s, 1e-6, floor: 1.0);
        t.CheckRel("diagonal flow", flow, 0.2, 1e-6, floor: 1.0);

        // Direction needing a missing group: infeasible.
        RcsCapabilitySnapshot oneSided = cap;
        oneSided.Set(1, new RcsAxisGroup());
        feasible = RcsExecutor.TryHoldPerformance(
            in oneSided, new double3(-1.0, 0.0, 0.0), out _, out _);
        t.Check("missing group infeasible", !feasible);
    }

    // The completion floor must mirror the worker's per-axis suppression:
    // when ShapeAxis would command nothing on any axis, the executor must
    // complete the burn instead of waiting forever. The mixed-floor case is
    // the regression from the first ingame stall: a residual just under the
    // strong axis's floor whose magnitude still exceeded a weak axis's
    // floor deadlocked the old magnitude-based check.
    private static void CheckCompletionFloor(TestContext t)
    {
        RcsCapabilitySnapshot cap = default;
        cap.Set(0, new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 10f });
        cap.Set(2, new RcsAxisGroup { ForceN = 2f, MassFlowKgS = 0.01f, MinImpulseNs = 0.2f });
        cap.HasAnyTranslation = true;

        // X component under the X floor (5), Y component under the Y floor
        // (0.1), but |impulse| = ~4.9 above the Y floor: worker fires
        // nothing, so the floor must report complete.
        t.Check("mixed floors complete", RcsExecutor.IsBelowImpulseFloor(
            new float3(4.9f, 0.05f, 0f), in cap));
        // X component above its floor: the worker still fires, not complete.
        t.Check("strong axis still firing", !RcsExecutor.IsBelowImpulseFloor(
            new float3(6f, 0f, 0f), in cap));
        // Weak axis alone above its own floor: still firing.
        t.Check("weak axis still firing", !RcsExecutor.IsBelowImpulseFloor(
            new float3(0f, 0.15f, 0f), in cap));
        // Residual along a direction with no group at all: nothing can
        // fire, so the burn completes with that residual reported.
        t.Check("missing group completes", RcsExecutor.IsBelowImpulseFloor(
            new float3(-6f, 0f, 0f), in cap));
    }

    // The BurnTarget duration mirror the countdown and warp-to-burn mark
    // read: the slowest demanded axis for groups (axes fire in parallel), the
    // pattern throughput cap for the LP. The lpUsable flag gates the LP branch
    // so a stale pattern during a slew or after staging falls back to groups.
    private static void CheckRemainingDuration(TestContext t)
    {
        var groups = new RcsWorkerCommand
        {
            Active = true,
            MaxPulseSec = 0.1f,
            AxisForcePos = new float3(100f, 50f, 0f),
            AxisForceNeg = new float3(100f, 50f, 0f),
        };
        // max(500/100, 100/50, 0) = 5 s: the X axis is the slowest.
        t.CheckAbs("groups slowest axis", RcsComputeControlPatch.RemainingDurationSec(
            groups, new float3(500f, 100f, 0f), lpUsable: false), 5f, FloatTol);
        // Negative demand uses the negative-axis force.
        t.CheckAbs("groups negative axis", RcsComputeControlPatch.RemainingDurationSec(
            groups, new float3(-300f, 0f, 0f), lpUsable: false), 3f, FloatTol);

        var lp = new RcsWorkerCommand
        {
            Active = true,
            MaxPulseSec = 0.1f,
            AxisForcePos = new float3(100f, 0f, 0f),
            AxisForceNeg = new float3(100f, 0f, 0f),
            LpSecondsPerImpulse = new float[1],
            LpDirBody = new float3(1f, 0f, 0f),
            LpImpulseCapNs = 200f,
        };
        // j = 1000, throughput 200 Ns per 0.1 s: 1000 * 0.1 / 200 = 0.5 s.
        t.CheckAbs("lp throughput cap", RcsComputeControlPatch.RemainingDurationSec(
            lp, new float3(1000f, 0f, 0f), lpUsable: true), 0.5f, FloatTol);
        // Same command with the LP disabled falls back to the group path.
        t.CheckAbs("lp disabled falls back", RcsComputeControlPatch.RemainingDurationSec(
            lp, new float3(500f, 0f, 0f), lpUsable: false), 5f, FloatTol);
    }

    private static void CheckCapabilityHelpers(TestContext t)
    {
        RcsCapabilitySnapshot cap = default;
        cap.Set(0, new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 1f });
        cap.Set(2, new RcsAxisGroup { ForceN = 250f, MassFlowKgS = 0.2f, MinImpulseNs = 2f });
        cap.Set(4, new RcsAxisGroup { ForceN = 50f, MassFlowKgS = 0.05f, MinImpulseNs = 0.5f });
        t.Check("best axis picks strongest", cap.BestAxis() == 2);

        RcsCapabilitySnapshot empty = default;
        t.Check("best axis none usable", empty.BestAxis() == -1);

        t.Check("dir +X", IsAxis(RcsCapabilitySnapshot.AxisDirection(0), 1f, 0f, 0f));
        t.Check("dir -X", IsAxis(RcsCapabilitySnapshot.AxisDirection(1), -1f, 0f, 0f));
        t.Check("dir +Y", IsAxis(RcsCapabilitySnapshot.AxisDirection(2), 0f, 1f, 0f));
        t.Check("dir -Y", IsAxis(RcsCapabilitySnapshot.AxisDirection(3), 0f, -1f, 0f));
        t.Check("dir +Z", IsAxis(RcsCapabilitySnapshot.AxisDirection(4), 0f, 0f, 1f));
        t.Check("dir -Z", IsAxis(RcsCapabilitySnapshot.AxisDirection(5), 0f, 0f, -1f));
    }

    private static bool IsAxis(float3 v, float x, float y, float z)
        => v.X == x && v.Y == y && v.Z == z;

    // The attitude-fight term: a group whose thrusters sit off the CoM leaves
    // residual torque a Hold burn must null, priced at the rotation groups'
    // flow per torque. Hand-checkable: 10 Nm pitch per 100 N of +X force is
    // 0.1 Nm/N, priced at 0.5/50 = 0.01 kg per Nm s, so 0.001 kg per Ns.
    private static void CheckAttitudeFight(TestContext t)
    {
        RcsCapabilitySnapshot cap = default;
        cap.Set(0, new RcsAxisGroup
        {
            ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 1f,
            TorqueNm = new float3(0f, 10f, 0f),
        });
        cap.RotationMassFlowKgS = new float3(0f, 0.5f, 0f);
        cap.RotationTorqueNm = new float3(0f, 50f, 0f);
        cap.HasAnyTranslation = true;

        t.CheckMixed("attitude fight per impulse",
            RcsExecutor.GroupAttitudeFightPerImpulse(in cap, new double3(1.0, 0.0, 0.0)), 0.001,
            absTol: 1e-9, relTol: 1e-6);

        // No rotation authority on the torqued axis: the hold cannot null it,
        // so the estimate adds nothing (the closed loop absorbs it at run time).
        RcsCapabilitySnapshot noRot = cap;
        noRot.RotationMassFlowKgS = default;
        noRot.RotationTorqueNm = default;
        t.CheckMixed("no rotation authority no cost",
            RcsExecutor.GroupAttitudeFightPerImpulse(in noRot, new double3(1.0, 0.0, 0.0)), 0.0,
            absTol: 1e-9, relTol: 1e-6);

        // A torque-free (balanced) group leaves no residual torque, so no fight.
        RcsCapabilitySnapshot balanced = cap;
        balanced.Set(0, new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 1f });
        t.CheckMixed("balanced group no fight",
            RcsExecutor.GroupAttitudeFightPerImpulse(in balanced, new double3(1.0, 0.0, 0.0)), 0.0,
            absTol: 1e-9, relTol: 1e-6);
    }
}
