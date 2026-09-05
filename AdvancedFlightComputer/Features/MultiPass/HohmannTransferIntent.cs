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

// The transfer goal stays fixed while each pass is recalculated from the live orbit.
internal sealed class HohmannTransferIntent : IManeuverIntent
{
    public const string HohmannTransferKind = "hohmann";

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public required string TargetId { get; init; }
    public required string ParentId { get; init; }
    public required double TFinalSec { get; init; }
    // The delta v uses the VLF frame of the original parking orbit.
    public required double3 DFinalVlf { get; init; }
    public required bool IsCrossParent { get; init; }
    public required double VInfMs { get; init; }
    public required double ApoTargetRadiusMeters { get; init; }
    // The original parking period anchors phasing even after prior burns change the live period.
    public required double ParkingPeriodSec { get; init; }

    public string Kind => HohmannTransferKind;
    public string TypeKey => ManeuverTools.ManeuverTools.KeyStockHohmann;

    public bool IsSatisfied(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return false;
        if (vehicle.Orbit.Parent.Id != ParentId) return false;

        Orbit o = vehicle.Orbit;
        double mu = o.Mu;
        double rp = o.Periapsis;
        if (!(rp > 0.0)) return false;
        double a = o.SemiMajorAxis;
        double vpNow = Math.Sqrt(mu * (2.0 / rp - 1.0 / a));
        double vpTarget = ComputeVpTargetAt(rp, mu);
        return vpNow + 1.0 >= vpTarget;
    }

    // Return null because HohmannMultiPassUI draws this intent rather than the generic active plan UI.
    public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle) => null;

    public PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode)
    {
        if (vehicle?.Orbit?.Parent == null)
            return PassPlanResult.Failure("vehicle has no orbit parent");
        if (vehicle.Orbit.Parent.Id != ParentId)
            return PassPlanResult.Failure(
                $"parent changed: was {ParentId}, now {vehicle.Orbit.Parent.Id}");

        UniverseTime now = Universe.GetElapsedTime();
        var tFinal = new UniverseTime(TFinalSec);
        if (tFinal < now)
            return PassPlanResult.Failure(
                "Lambert window has passed; multi-pass cannot recover phasing");

        int remainingCount = passCountTotal - passIndex;
        if (remainingCount <= 0)
            return PassPlanResult.Failure(
                $"passIndex {passIndex} >= total {passCountTotal}");

        IOrbiter? target = ResolveTarget();
        if (target == null)
        {
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

        return IntentPlanning.FirstPass(result);
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

    // UniverseTime rejects NaN, so corrupt saved values must be rejected before recompute.
    private static bool TryParseDouble(
        IReadOnlyDictionary<string, string> kv, string key, out double value)
    {
        value = 0.0;
        return kv.TryGetValue(key, out string? s)
            && double.TryParse(s, NumberStyles.Float, Inv, out value)
            && double.IsFinite(value);
    }
}
