using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// "Depart toward <see cref="TargetId"/> via the Lambert-optimal Hohmann
/// solution captured at intent creation, split into N perigee kicks with
/// the final pass landing at <see cref="TFinalSec"/>."
///
/// The Lambert inputs (T_final, D_final direction, v_inf magnitude) are
/// locked at Start so the asymptote phasing is preserved across N passes;
/// otherwise re-planning each pass would chase a moving target window
/// and drift the encounter geometry, which is how a cross-parent transfer
/// loses its encounter entirely.
///
/// Per-pass dV is recomputed against the live orbit each commit. Prior
/// passes share the splitter's equal-time / equal-dV allocation; the
/// final pass's magnitude is whatever brings periapsis speed to
/// <c>sqrt(v_inf^2 + 2 mu / r_p)</c> from the live pre-final orbit.
/// </summary>
internal sealed class HohmannTransferIntent : IManeuverIntent
{
    public const string HohmannTransferKind = "hohmann";

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public required string TargetId { get; init; }
    public required string ParentId { get; init; }

    /// <summary>Stock-Lambert burn time at which the final pass fires.
    /// Serialised as raw seconds.</summary>
    public required double TFinalSec { get; init; }

    /// <summary>Stock-Lambert dV vector at <see cref="TFinalSec"/> in
    /// the parking orbit's VLF frame. Direction is preserved across all
    /// passes; the magnitude is rescaled by RecomputePass for the final
    /// pass to hit the locked target.</summary>
    public required double3 DFinalVlf { get; init; }

    /// <summary>True for cross-parent transfers (Earth -> Mars):
    /// post-burn orbit is hyperbolic relative to the parking parent,
    /// target tracked via <see cref="VInfMs"/>. False for same-parent
    /// transfers (Earth -> Luna): post-burn orbit is bound, target
    /// tracked via <see cref="ApoTargetRadiusMeters"/>.</summary>
    public required bool IsCrossParent { get; init; }

    /// <summary>Hyperbolic excess velocity magnitude at the parking
    /// radius, derived from the stock Lambert EjectionVelocityCci.
    /// Used iff <see cref="IsCrossParent"/>.</summary>
    public required double VInfMs { get; init; }

    /// <summary>Target apoapsis radius (m, from parent center) the stock
    /// Hohmann was aiming for. Used iff !<see cref="IsCrossParent"/>.</summary>
    public required double ApoTargetRadiusMeters { get; init; }

    /// <summary>Original parking orbit period locked at intent creation.
    /// Critical: the integer-K-sum scheduling uses this fixed value as the
    /// "anchor period" for all passes. Per-pass K values are real, only
    /// the sum is rounded to an integer at initial-plan time so that the
    /// vehicle at pass-0 firing and at T_final share the same parking-orbit
    /// CCI direction. Reading <c>vehicle.Orbit.Period</c> at recompute time
    /// would give the chained orbit's period, breaking the chain alignment
    /// and causing subsequent passes to fire at apoapsis instead of
    /// periapsis.</summary>
    public required double ParkingPeriodSec { get; init; }

    public string Kind => HohmannTransferKind;

    // Shown verbatim by MultiPassUI.DrawBlockedByOtherExecution when the
    // user opens an AFC-handled plan type while a Hohmann exec is still
    // running on this vehicle.
    public string TypeKey => ManeuverTools.ManeuverTools.KeyStockHohmann;

    public bool IsSatisfied(Vehicle vehicle)
    {
        // Live periapsis speed already meets / exceeds v_p_target: priors
        // over-shot the locked goal and any further pass would push past
        // v_inf. The postfix uses this to complete the execution cleanly
        // instead of accumulating MaxConsecutiveScheduleFailures on a
        // "dV non-positive" planner failure.
        if (vehicle?.Orbit?.Parent == null) return false;
        if (vehicle.Orbit.Parent.Id != ParentId) return false;

        Orbit o = vehicle.Orbit;
        double mu = o.Mu;
        double rp = o.Periapsis;
        if (!(rp > 0.0)) return false;
        double a = o.SemiMajorAxis;
        double vpNow = Math.Sqrt(mu * (2.0 / rp - 1.0 / a));
        double vpTarget = ComputeVpTargetAt(rp, mu);

        // 1 m/s tolerance soaks up finite-burn / numerical noise without
        // letting a genuinely under-shot plan flip to "satisfied".
        return vpNow + 1.0 >= vpTarget;
    }

    // Null forces MultiPassUI.DrawActive (apse / inclination plan window)
    // to early-exit when a Hohmann execution is active; HohmannMultiPassUI
    // owns the user-facing status for this intent kind.
    public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle) => null;

    public PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode)
    {
        if (vehicle?.Orbit?.Parent == null)
            return PassPlanResult.Failure("vehicle has no orbit parent");
        if (vehicle.Orbit.Parent.Id != ParentId)
            return PassPlanResult.Failure(
                $"parent changed: was {ParentId}, now {vehicle.Orbit.Parent.Id}");

        SimTime now = Universe.GetElapsedSimTime();
        var tFinal = new SimTime(TFinalSec);
        if (tFinal.Seconds() < now.Seconds())
            return PassPlanResult.Failure(
                "Lambert window has passed; multi-pass cannot recover phasing");

        int remainingCount = passCountTotal - passIndex;
        if (remainingCount <= 0)
            return PassPlanResult.Failure(
                $"passIndex {passIndex} >= total {passCountTotal}");

        IOrbiter? target = ResolveTarget();
        if (target == null)
        {
            // Player-visible: silently cancelling a half-finished
            // departure would leave the vehicle in a strange in-between
            // state with no obvious reason; the postfix's
            // MaxConsecutiveScheduleFailures path triggers via the
            // returned Failure too, so this alert fires once per attempt.
            TimedAlert.Create(
                $"Hohmann multi-pass: target '{TargetId}' lost, cancelling.",
                Color.Red, 5.0);
            return PassPlanResult.Failure($"target '{TargetId}' no longer in system");
        }

        SequenceBurnState state = SequenceBurnState.Analyze(vehicle);

        var input = new HohmannMultiPassPlanner.HohmannPlanInput(
            Target: target,
            TFinal: tFinal,
            DFinalVlf: DFinalVlf,
            IsCrossParent: IsCrossParent,
            VInfMs: VInfMs,
            ApoTargetRadiusMeters: ApoTargetRadiusMeters);

        // Critical: pass the LOCKED parking period (not vehicle.Orbit.Period
        // which is the chained orbit at this point) so the K-schedule
        // remains anchored to the original parking-orbit geometry. The
        // planner re-derives the real-K schedule for the remaining passes
        // each call, anchoring times[0] to the vehicle's next chained
        // periapsis time (computed from live state). SplitMode comes from
        // MultiPassExecution.Mode (locked at intent creation): the planner
        // re-applies the same allocation policy to the live remaining dV
        // and current fuel state each recompute - so the policy is
        // consistent but per-pass dV evolves as fuel drains.
        var result = HohmannMultiPassPlanner.Plan(
            vehicle, input, passCountTotal, passIndex,
            ParkingPeriodSec, state, now, mode);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannTransferIntent.RecomputePass: vehicle='{0}' target='{1}' " +
                "passIndex={2}/{3} remaining={4} -> {5} pass(es) " +
                "(failed={6} reason='{7}')",
                vehicle.Id, TargetId, passIndex, passCountTotal, remainingCount,
                result.Passes.Length, result.Failed,
                result.FailureReason ?? "-"));

        if (result.Passes.Length == 0)
            return PassPlanResult.Failure(
                result.FailureReason ?? "planner produced no passes");
        return PassPlanResult.Success(result.Passes[0]);
    }

    private double ComputeVpTargetAt(double rp, double mu)
    {
        if (IsCrossParent)
            return Math.Sqrt(VInfMs * VInfMs + 2.0 * mu / rp);

        double aTarget = (rp + ApoTargetRadiusMeters) * 0.5;
        if (!(aTarget > 0.0)) return 0.0;
        double term = 2.0 / rp - 1.0 / aTarget;
        if (!(term > 0.0)) return 0.0;
        return Math.Sqrt(mu * term);
    }

    private IOrbiter? ResolveTarget()
    {
        if (Universe.CurrentSystem == null) return null;
        if (!Universe.CurrentSystem.All.TryGet(TargetId, out Astronomical? target))
            return null;
        return target as IOrbiter;
    }

    public void WriteToToml(TextWriter w)
    {
        w.WriteLine($"target_id = \"{TomlIo.Escape(TargetId)}\"");
        w.WriteLine($"parent_id = \"{TomlIo.Escape(ParentId)}\"");
        w.WriteLine(string.Format(Inv, "t_final_sec = {0:R}", TFinalSec));
        w.WriteLine(string.Format(Inv, "d_final_vlf_x = {0:R}", DFinalVlf.X));
        w.WriteLine(string.Format(Inv, "d_final_vlf_y = {0:R}", DFinalVlf.Y));
        w.WriteLine(string.Format(Inv, "d_final_vlf_z = {0:R}", DFinalVlf.Z));
        w.WriteLine(string.Format(Inv, "is_cross_parent = {0}", IsCrossParent ? "true" : "false"));
        w.WriteLine(string.Format(Inv, "v_inf_ms = {0:R}", VInfMs));
        w.WriteLine(string.Format(Inv, "apo_target_radius_m = {0:R}", ApoTargetRadiusMeters));
        w.WriteLine(string.Format(Inv, "parking_period_sec = {0:R}", ParkingPeriodSec));
    }

    public static HohmannTransferIntent? FromToml(IReadOnlyDictionary<string, string> kv)
    {
        if (!kv.TryGetValue("target_id", out string? tid) || string.IsNullOrEmpty(tid))
            return null;
        if (!kv.TryGetValue("parent_id", out string? pid) || string.IsNullOrEmpty(pid))
            return null;
        if (!TryParseDouble(kv, "t_final_sec", out double tFinal)) return null;
        if (!TryParseDouble(kv, "d_final_vlf_x", out double dx)) return null;
        if (!TryParseDouble(kv, "d_final_vlf_y", out double dy)) return null;
        if (!TryParseDouble(kv, "d_final_vlf_z", out double dz)) return null;
        bool isCrossParent = kv.TryGetValue("is_cross_parent", out string? cp) && cp == "true";
        if (!TryParseDouble(kv, "v_inf_ms", out double vInf)) return null;
        if (!TryParseDouble(kv, "apo_target_radius_m", out double apoTarget)) return null;
        // parking_period_sec anchors the K-schedule, and a 0.0 fallback would
        // break recompute scheduling, so an entry without it is rejected rather
        // than defaulted.
        if (!TryParseDouble(kv, "parking_period_sec", out double parkingPeriod)
            || !(parkingPeriod > 0.0))
            return null;
        return new HohmannTransferIntent
        {
            TargetId = tid,
            ParentId = pid,
            TFinalSec = tFinal,
            DFinalVlf = new double3(dx, dy, dz),
            IsCrossParent = isCrossParent,
            VInfMs = vInf,
            ApoTargetRadiusMeters = apoTarget,
            ParkingPeriodSec = parkingPeriod,
        };
    }

    private static bool TryParseDouble(
        IReadOnlyDictionary<string, string> kv, string key, out double value)
    {
        value = 0.0;
        return kv.TryGetValue(key, out string? s)
            && double.TryParse(s, NumberStyles.Float, Inv, out value);
    }
}
