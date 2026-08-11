using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// "Set the opposite apse to <see cref="TargetRadiusMeters"/> by
/// burning at this apse." Covers both Set Apoapsis (burn at periapsis,
/// burnTa=0) and Set Periapsis (burn at apoapsis, burnTa=Pi) via
/// <see cref="IsSetApoapsis"/>.
///
/// TargetRadiusMeters is absolute (from the parent's center) so it
/// stays meaningful as the orbit changes. If the vehicle SOI-
/// transitions, ParentId no longer matches and RecomputePass aborts.
/// </summary>
internal sealed class ApseIntent : IManeuverIntent
{
    public const string SetApoapsisKind = "set-ap";
    public const string SetPeriapsisKind = "set-pe";

    public required bool IsSetApoapsis { get; init; }
    public required double TargetRadiusMeters { get; init; }
    public required string ParentId { get; init; }

    public string Kind => IsSetApoapsis ? SetApoapsisKind : SetPeriapsisKind;

    public string TypeKey => IsSetApoapsis
        ? ManeuverTools.ManeuverTools.KeySetApoapsis
        : ManeuverTools.ManeuverTools.KeySetPeriapsis;

    public bool IsSatisfied(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return false;
        if (vehicle.Orbit.Parent.Id != ParentId) return false;
        double currentRadius = IsSetApoapsis ? vehicle.Orbit.Apoapsis : vehicle.Orbit.Periapsis;
        // 1m tolerance: vis-viva precision on the relevant apsis is well
        // below that for any practical orbit.
        return Math.Abs(currentRadius - TargetRadiusMeters) < 1.0;
    }

    public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return null;
        if (vehicle.Orbit.Parent.Id != ParentId) return null;

        double parentRadius = vehicle.Orbit.Parent.MeanRadius;
        double targetAltitude = TargetRadiusMeters - parentRadius;
        UniverseTime now = Universe.GetElapsedTime();

        return IsSetApoapsis
            ? OrbitManeuvers.ComputeSetApoapsis(vehicle.Orbit, targetAltitude, parentRadius, now)
            : OrbitManeuvers.ComputeSetPeriapsis(vehicle.Orbit, targetAltitude, parentRadius, now);
    }

    public PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode)
    {
        if (vehicle?.Orbit?.Parent == null)
            return PassPlanResult.Failure("vehicle has no orbit parent");
        if (vehicle.Orbit.Parent.Id != ParentId)
            return PassPlanResult.Failure(
                $"parent changed: was {ParentId}, now {vehicle.Orbit.Parent.Id}");

        OrbitManeuvers.ManeuverResult? maneuver = ComputeManeuver(vehicle);
        if (maneuver == null)
            return PassPlanResult.Failure(IsSetApoapsis
                ? "target apoapsis not reachable from current orbit"
                : "target periapsis not reachable from current orbit");

        int remainingCount = passCountTotal - passIndex;
        if (remainingCount <= 0)
            return PassPlanResult.Failure($"passIndex {passIndex} >= total {passCountTotal}");

        UniverseTime now = Universe.GetElapsedTime();
        SequenceBurnState state = SequenceBurnState.Analyze(vehicle);
        PassAllocation[] allocations = Splitter.Allocate(
            maneuver.Value.DvCci.Length(), remainingCount, mode, state);

        TrueAnomaly burnTa = IsSetApoapsis ? TrueAnomaly.Zero : new TrueAnomaly(Math.PI);
        var result = ApseBurnPlanner.Plan(
            vehicle, maneuver.Value.DvVlf, burnTa, allocations, now);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] ApseIntent.RecomputePass: vehicle='{0}' kind='{1}' passIndex={2}/{3} " +
                "totalDv={4:F1}m/s remaining={5} -> {6} pass(es) (failed={7} reason='{8}')",
                vehicle.Id, Kind, passIndex, passCountTotal,
                maneuver.Value.DvCci.Length(),
                remainingCount,
                result.Passes.Length,
                result.Failed,
                result.FailureReason ?? "-"));

        if (result.Passes.Length == 0)
            return PassPlanResult.Failure(result.FailureReason ?? "planner produced no passes");
        return PassPlanResult.Success(result.Passes[0]);
    }

    public void WriteToToml(TextWriter w)
    {
        w.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "target_radius_m = {0:R}", TargetRadiusMeters));
        w.WriteLine($"parent_id = \"{TomlIo.Escape(ParentId)}\"");
    }

    public static ApseIntent? FromToml(IReadOnlyDictionary<string, string> kv, bool isSetApoapsis)
    {
        if (!kv.TryGetValue("target_radius_m", out string? rs)
            || !double.TryParse(rs, NumberStyles.Float, CultureInfo.InvariantCulture, out double radius))
            return null;
        if (!kv.TryGetValue("parent_id", out string? pid))
            return null;
        return new ApseIntent
        {
            IsSetApoapsis = isSetApoapsis,
            TargetRadiusMeters = radius,
            ParentId = pid,
        };
    }
}
