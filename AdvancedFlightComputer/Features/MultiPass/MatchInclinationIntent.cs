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
/// "Match the orbital plane of <see cref="TargetId"/>, burning at the
/// chosen node (AN or DN of the vehicle's orbit relative to the target)."
/// The target is resolved live each pass: a moving vehicle target keeps
/// its current plane, a celestial target stays Keplerian-stable. If the
/// vehicle SOI-transitions away from <see cref="ParentId"/> or the target
/// disappears, RecomputePass aborts.
/// </summary>
internal sealed class MatchInclinationIntent : IManeuverIntent
{
    public const string MatchInclinationKind = "match-inc";

    public required string TargetId { get; init; }
    public required bool UseDescendingNode { get; init; }
    public required string ParentId { get; init; }

    public string Kind => MatchInclinationKind;

    public string TypeKey => ManeuverTools.ManeuverTools.KeyMatchInclination;

    public bool IsSatisfied(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return false;
        if (vehicle.Orbit.Parent.Id != ParentId) return false;
        Orbit? targetOrbit = ResolveTargetOrbit(vehicle);
        if (targetOrbit == null) return false;
        // Matches the relInc < 0.001 short-circuit inside
        // OrbitManeuvers.ComputeMatchInclination.
        return vehicle.Orbit.GetRelativeInclination(targetOrbit).Value() < 0.001;
    }

    public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return null;
        if (vehicle.Orbit.Parent.Id != ParentId) return null;

        Orbit? targetOrbit = ResolveTargetOrbit(vehicle);
        if (targetOrbit == null) return null;

        return OrbitManeuvers.ComputeMatchInclination(
            vehicle.Orbit, targetOrbit, UseDescendingNode, Universe.GetElapsedSimTime());
    }

    public PassPlanResult RecomputePass(
        Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode)
    {
        if (vehicle?.Orbit?.Parent == null)
            return PassPlanResult.Failure("vehicle has no orbit parent");
        if (vehicle.Orbit.Parent.Id != ParentId)
            return PassPlanResult.Failure(
                $"parent changed: was {ParentId}, now {vehicle.Orbit.Parent.Id}");

        Orbit? targetOrbit = ResolveTargetOrbit(vehicle);
        if (targetOrbit == null)
            return PassPlanResult.Failure($"target '{TargetId}' no longer in system");

        OrbitManeuvers.ManeuverResult? maneuver = OrbitManeuvers.ComputeMatchInclination(
            vehicle.Orbit, targetOrbit, UseDescendingNode, Universe.GetElapsedSimTime());
        if (maneuver == null)
            return PassPlanResult.Failure("inclinations already match");

        int remainingCount = passCountTotal - passIndex;
        if (remainingCount <= 0)
            return PassPlanResult.Failure($"passIndex {passIndex} >= total {passCountTotal}");

        SimTime now = Universe.GetElapsedSimTime();
        SequenceBurnState state = SequenceBurnState.Analyze(vehicle);
        PassAllocation[] allocations = Splitter.Allocate(
            maneuver.Value.DvCci.Length(), remainingCount, mode, state);

        var result = PlaneChangeBurnPlanner.PlanForMatch(
            vehicle, targetOrbit, UseDescendingNode, allocations, now);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] MatchInclinationIntent.RecomputePass: vehicle='{0}' target='{1}' " +
                "passIndex={2}/{3} totalDv={4:F1}m/s remaining={5} -> {6} pass(es) " +
                "(failed={7} reason='{8}')",
                vehicle.Id, TargetId, passIndex, passCountTotal,
                maneuver.Value.DvCci.Length(), remainingCount,
                result.Passes.Length, result.Failed, result.FailureReason ?? "-"));

        if (result.Passes.Length == 0)
            return PassPlanResult.Failure(result.FailureReason ?? "planner produced no passes");
        return PassPlanResult.Success(result.Passes[0]);
    }

    /// <summary>Finds the target IOrbiter (Vehicle or Celestial) by id,
    /// restricted to bodies sharing this intent's locked ParentId so a
    /// cross-SOI target doesn't accidentally match. Uses the system-wide
    /// lookup so we still resolve out-of-frame vehicles after a long
    /// warp. Reads <see cref="ParentId"/> directly (rather than the
    /// live vehicle.Parent) so the lock is explicit in the call.</summary>
    private Orbit? ResolveTargetOrbit(Vehicle vehicle)
    {
        if (Universe.CurrentSystem == null) return null;
        if (!Universe.CurrentSystem.All.TryGet(TargetId, out Astronomical? target))
            return null;
        if (target is not IOrbiter orbiter) return null;
        if (orbiter.Parent?.Id != ParentId) return null;
        return orbiter.Orbit;
    }

    public void WriteToToml(TextWriter w)
    {
        w.WriteLine($"target_id = \"{TomlIo.Escape(TargetId)}\"");
        w.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "use_descending_node = {0}", UseDescendingNode ? "true" : "false"));
        w.WriteLine($"parent_id = \"{TomlIo.Escape(ParentId)}\"");
    }

    public static MatchInclinationIntent? FromToml(IReadOnlyDictionary<string, string> kv)
    {
        if (!kv.TryGetValue("target_id", out string? tid) || string.IsNullOrEmpty(tid))
            return null;
        if (!kv.TryGetValue("parent_id", out string? pid) || string.IsNullOrEmpty(pid))
            return null;
        bool useDesc = kv.TryGetValue("use_descending_node", out string? dn) && dn == "true";
        return new MatchInclinationIntent
        {
            TargetId = tid,
            UseDescendingNode = useDesc,
            ParentId = pid,
        };
    }
}
