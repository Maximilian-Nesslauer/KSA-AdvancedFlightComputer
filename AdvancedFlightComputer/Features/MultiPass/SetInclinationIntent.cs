using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

internal sealed class SetInclinationIntent : IManeuverIntent
{
    public const string SetInclinationKind = "set-inc";

    public required double TargetInclinationRad { get; init; }
    public required OrbitManeuvers.InclinationReference Reference { get; init; }
    public required bool UseDescendingNode { get; init; }
    public required string ParentId { get; init; }

    public string Kind => SetInclinationKind;

    public string TypeKey => ManeuverTools.ManeuverTools.KeySetInclination;

    public bool IsSatisfied(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return false;
        if (vehicle.Orbit.Parent.Id != ParentId) return false;
        double currentInc = OrbitManeuvers.GetInclinationAgainst(vehicle.Orbit, Reference);
        // Use the same tolerance as OrbitManeuvers.ComputeSetInclination.
        return System.Math.Abs(TargetInclinationRad - currentInc) < 0.001;
    }

    public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return null;
        if (vehicle.Orbit.Parent.Id != ParentId) return null;

        return OrbitManeuvers.ComputeSetInclination(
            vehicle.Orbit, TargetInclinationRad, UseDescendingNode,
            Universe.GetElapsedTime(), Reference);
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
            return PassPlanResult.Failure("target inclination not reachable from current orbit");

        int remainingCount = passCountTotal - passIndex;
        if (remainingCount <= 0)
            return PassPlanResult.Failure($"passIndex {passIndex} >= total {passCountTotal}");

        UniverseTime now = Universe.GetElapsedTime();
        SequenceBurnState state = SequenceBurnState.Analyze(vehicle);
        PassAllocation[] allocations = Splitter.Allocate(
            maneuver.Value.DvCci.Length(), remainingCount, mode, state);

        var result = PlaneChangeBurnPlanner.PlanForSet(
            vehicle, TargetInclinationRad, Reference, UseDescendingNode, allocations, now, execution: true);

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] SetInclinationIntent.RecomputePass: vehicle='{0}' targetInc={1:F2}deg " +
                "passIndex={2}/{3} totalDv={4:F1}m/s remaining={5} -> {6} pass(es) " +
                "(failed={7} reason='{8}')",
                vehicle.Id, TargetInclinationRad * 180.0 / System.Math.PI,
                passIndex, passCountTotal,
                maneuver.Value.DvCci.Length(), remainingCount,
                result.Passes.Length, result.Failed, result.FailureReason ?? "-"));

        return IntentPlanning.FirstPass(result);
    }

    public void WriteToToml(TextWriter w)
    {
        w.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "target_inc_rad = {0:R}", TargetInclinationRad));
        w.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "reference = \"{0}\"", Reference));
        w.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "use_descending_node = {0}", UseDescendingNode ? "true" : "false"));
        w.WriteLine($"parent_id = \"{TomlIo.Escape(ParentId)}\"");
    }

    public static SetInclinationIntent? FromToml(IReadOnlyDictionary<string, string> kv)
    {
        if (!kv.TryGetValue("target_inc_rad", out string? incStr)
            || !double.TryParse(incStr, NumberStyles.Float, CultureInfo.InvariantCulture, out double inc))
            return null;
        if (!kv.TryGetValue("reference", out string? refStr)
            || !System.Enum.TryParse(refStr, out OrbitManeuvers.InclinationReference reference))
            return null;
        if (!kv.TryGetValue("parent_id", out string? pid) || string.IsNullOrEmpty(pid))
            return null;
        bool useDesc = kv.TryGetValue("use_descending_node", out string? dn) && dn == "true";
        return new SetInclinationIntent
        {
            TargetInclinationRad = inc,
            Reference = reference,
            UseDescendingNode = useDesc,
            ParentId = pid,
        };
    }
}
