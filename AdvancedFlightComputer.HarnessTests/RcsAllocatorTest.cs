using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the pure allocation math of the RCS translation executor: the
// per-axis impulse shaping (cap and minimum-impulse suppression), the
// per-thruster group pulse derivation, and the Hold-strategy performance
// model that feeds the Auto attitude decision. No vehicle involved.
public sealed class RcsAllocatorTest : IHarnessTest
{
    public string Name => "afc-rcs-allocator";

    public int Run(HeadlessSession session)
    {
        bool ok = true;
        ok &= CheckShapeAxis();
        ok &= CheckMaxAxisPulse();
        ok &= CheckHoldPerformance();
        ok &= CheckCompletionFloor();
        ok &= CheckRemainingDuration();
        ok &= CheckCapabilityHelpers();
        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckShapeAxis()
    {
        bool ok = true;
        // Cap at one control period of group force.
        ok &= Expect("cap positive", RcsComputeControlPatch.ShapeAxis(
            50f, forcePos: 100f, forceNeg: 0f, minImpPos: 10f, minImpNeg: 0f, maxPulse: 0.1f), 10f);
        // Below half the minimum impulse: suppressed.
        ok &= Expect("min impulse floor", RcsComputeControlPatch.ShapeAxis(
            4f, 100f, 0f, 10f, 0f, 0.1f), 0f);
        // At/above half the minimum impulse: passes through uncapped.
        ok &= Expect("small but usable", RcsComputeControlPatch.ShapeAxis(
            6f, 100f, 0f, 10f, 0f, 0.1f), 6f);
        // No group in the demanded direction: nothing to fire.
        ok &= Expect("missing negative group", RcsComputeControlPatch.ShapeAxis(
            -50f, 100f, 0f, 10f, 0f, 0.1f), 0f);
        // Negative direction with its own group.
        ok &= Expect("cap negative", RcsComputeControlPatch.ShapeAxis(
            -50f, 0f, 100f, 0f, 10f, 0.1f), -10f);
        return ok;
    }

    private bool CheckMaxAxisPulse()
    {
        bool ok = true;
        // Participating thruster fires for J / F_group.
        ok &= Expect("positive pulse", RcsComputeControlPatch.MaxAxisPulse(
            0f, j: 50f, thrusterForce: 25f, groupForcePos: 100f, groupForceNeg: 0f), 0.5f);
        ok &= Expect("negative pulse", RcsComputeControlPatch.MaxAxisPulse(
            0f, -50f, -25f, 0f, 100f), 0.5f);
        // Wrong-sign thruster does not participate.
        ok &= Expect("non-participating", RcsComputeControlPatch.MaxAxisPulse(
            0f, 50f, -25f, 100f, 100f), 0f);
        // Max across axes, not sum.
        ok &= Expect("max not sum", RcsComputeControlPatch.MaxAxisPulse(
            0.8f, 50f, 25f, 100f, 0f), 0.8f);
        return ok;
    }

    private bool CheckHoldPerformance()
    {
        bool ok = true;
        RcsCapabilitySnapshot cap = default;
        var group = new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 1f };
        for (int i = 0; i < 6; i++)
            cap.Set(i, in group);
        cap.HasAnyTranslation = true;

        // Pure single-axis direction: full group force, single group flow.
        bool feasible = RcsExecutor.TryHoldPerformance(
            in cap, new double3(1.0, 0.0, 0.0), out double force, out double flow);
        ok &= Check("single axis feasible", feasible);
        ok &= Near("single axis force", force, 100.0);
        ok &= Near("single axis flow", flow, 0.1);

        // 45 degree in-plane direction: net force rises to F/cos45 with both
        // groups duty-cycled at full force, flow doubles.
        double s = Math.Sqrt(0.5);
        feasible = RcsExecutor.TryHoldPerformance(
            in cap, new double3(s, s, 0.0), out force, out flow);
        ok &= Check("diagonal feasible", feasible);
        ok &= Near("diagonal force", force, 100.0 / s);
        ok &= Near("diagonal flow", flow, 0.2);

        // Direction needing a missing group: infeasible.
        RcsCapabilitySnapshot oneSided = cap;
        oneSided.Set(1, new RcsAxisGroup());
        feasible = RcsExecutor.TryHoldPerformance(
            in oneSided, new double3(-1.0, 0.0, 0.0), out _, out _);
        ok &= Check("missing group infeasible", !feasible);
        return ok;
    }

    // The completion floor must mirror the worker's per-axis suppression:
    // when ShapeAxis would command nothing on any axis, the executor must
    // complete the burn instead of waiting forever. The mixed-floor case is
    // the regression from the first ingame stall: a residual just under the
    // strong axis's floor whose magnitude still exceeded a weak axis's
    // floor deadlocked the old magnitude-based check.
    private bool CheckCompletionFloor()
    {
        bool ok = true;
        RcsCapabilitySnapshot cap = default;
        cap.Set(0, new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 10f });
        cap.Set(2, new RcsAxisGroup { ForceN = 2f, MassFlowKgS = 0.01f, MinImpulseNs = 0.2f });
        cap.HasAnyTranslation = true;

        // X component under the X floor (5), Y component under the Y floor
        // (0.1), but |impulse| = ~4.9 above the Y floor: worker fires
        // nothing, so the floor must report complete.
        ok &= Check("mixed floors complete", RcsExecutor.IsBelowImpulseFloor(
            new float3(4.9f, 0.05f, 0f), in cap));
        // X component above its floor: the worker still fires, not complete.
        ok &= Check("strong axis still firing", !RcsExecutor.IsBelowImpulseFloor(
            new float3(6f, 0f, 0f), in cap));
        // Weak axis alone above its own floor: still firing.
        ok &= Check("weak axis still firing", !RcsExecutor.IsBelowImpulseFloor(
            new float3(0f, 0.15f, 0f), in cap));
        // Residual along a direction with no group at all: nothing can
        // fire, so the burn completes with that residual reported.
        ok &= Check("missing group completes", RcsExecutor.IsBelowImpulseFloor(
            new float3(-6f, 0f, 0f), in cap));
        return ok;
    }

    // The BurnTarget duration mirror the countdown and warp-to-burn mark
    // read: the slowest demanded axis for groups (axes fire in parallel), the
    // pattern throughput cap for the LP. The lpUsable flag gates the LP branch
    // so a stale pattern during a slew or after staging falls back to groups.
    private bool CheckRemainingDuration()
    {
        bool ok = true;

        var groups = new RcsWorkerCommand
        {
            Active = true,
            MaxPulseSec = 0.1f,
            AxisForcePos = new float3(100f, 50f, 0f),
            AxisForceNeg = new float3(100f, 50f, 0f),
        };
        // max(500/100, 100/50, 0) = 5 s: the X axis is the slowest.
        ok &= Expect("groups slowest axis", RcsComputeControlPatch.RemainingDurationSec(
            groups, new float3(500f, 100f, 0f), lpUsable: false), 5f);
        // Negative demand uses the negative-axis force.
        ok &= Expect("groups negative axis", RcsComputeControlPatch.RemainingDurationSec(
            groups, new float3(-300f, 0f, 0f), lpUsable: false), 3f);

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
        ok &= Expect("lp throughput cap", RcsComputeControlPatch.RemainingDurationSec(
            lp, new float3(1000f, 0f, 0f), lpUsable: true), 0.5f);
        // Same command with the LP disabled falls back to the group path.
        ok &= Expect("lp disabled falls back", RcsComputeControlPatch.RemainingDurationSec(
            lp, new float3(500f, 0f, 0f), lpUsable: false), 5f);
        return ok;
    }

    private bool CheckCapabilityHelpers()
    {
        bool ok = true;
        RcsCapabilitySnapshot cap = default;
        cap.Set(0, new RcsAxisGroup { ForceN = 100f, MassFlowKgS = 0.1f, MinImpulseNs = 1f });
        cap.Set(2, new RcsAxisGroup { ForceN = 250f, MassFlowKgS = 0.2f, MinImpulseNs = 2f });
        cap.Set(4, new RcsAxisGroup { ForceN = 50f, MassFlowKgS = 0.05f, MinImpulseNs = 0.5f });
        ok &= Check("best axis picks strongest", cap.BestAxis() == 2);

        RcsCapabilitySnapshot empty = default;
        ok &= Check("best axis none usable", empty.BestAxis() == -1);

        ok &= Check("dir +X", IsAxis(RcsCapabilitySnapshot.AxisDirection(0), 1f, 0f, 0f));
        ok &= Check("dir -X", IsAxis(RcsCapabilitySnapshot.AxisDirection(1), -1f, 0f, 0f));
        ok &= Check("dir +Y", IsAxis(RcsCapabilitySnapshot.AxisDirection(2), 0f, 1f, 0f));
        ok &= Check("dir -Y", IsAxis(RcsCapabilitySnapshot.AxisDirection(3), 0f, -1f, 0f));
        ok &= Check("dir +Z", IsAxis(RcsCapabilitySnapshot.AxisDirection(4), 0f, 0f, 1f));
        ok &= Check("dir -Z", IsAxis(RcsCapabilitySnapshot.AxisDirection(5), 0f, 0f, -1f));
        return ok;
    }

    private static bool IsAxis(float3 v, float x, float y, float z)
        => v.X == x && v.Y == y && v.Z == z;

    private bool Expect(string label, float actual, float expected)
    {
        bool ok = Math.Abs(actual - expected) < 1e-4f;
        if (!ok)
            HarnessLog.Line($"[{Name}] TEST {label}: got {actual}, expected {expected} => FAIL");
        return ok;
    }

    private bool Near(string label, double actual, double expected)
    {
        bool ok = Math.Abs(actual - expected) < 1e-6 * Math.Max(1.0, Math.Abs(expected));
        if (!ok)
            HarnessLog.Line($"[{Name}] TEST {label}: got {actual}, expected {expected} => FAIL");
        return ok;
    }

    private bool Check(string label, bool condition)
    {
        if (!condition)
            HarnessLog.Line($"[{Name}] TEST {label} => FAIL");
        return condition;
    }
}
