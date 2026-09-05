using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

internal sealed class CircularizeIntent : IManeuverIntent
{
    public const string CircularizeApoapsisKind = "circularize-ap";
    public const string CircularizePeriapsisKind = "circularize-pe";
    // Use the same tolerance as OrbitManeuvers.ComputeCircularize.
    private const double CircularToleranceEcc = 0.001;

    public required bool IsAtApoapsis { get; init; }
    public required string ParentId { get; init; }

    public string Kind => IsAtApoapsis ? CircularizeApoapsisKind : CircularizePeriapsisKind;

    public string TypeKey => IsAtApoapsis
        ? ManeuverTools.ManeuverTools.KeyStockCircularizeApoapsis
        : ManeuverTools.ManeuverTools.KeyStockCircularizePeriapsis;

    public bool IsSatisfied(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return false;
        if (vehicle.Orbit.Parent.Id != ParentId) return false;
        return vehicle.Orbit.Eccentricity < CircularToleranceEcc;
    }

    public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null) return null;
        if (vehicle.Orbit.Parent.Id != ParentId) return null;

        return OrbitManeuvers.ComputeCircularize(
            vehicle.Orbit, IsAtApoapsis, Universe.GetElapsedTime());
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
            return PassPlanResult.Failure("orbit not circularizable from current state");

        int remainingCount = passCountTotal - passIndex;
        if (remainingCount <= 0)
            return PassPlanResult.Failure($"passIndex {passIndex} >= total {passCountTotal}");

        UniverseTime now = Universe.GetElapsedTime();
        SequenceBurnState state = SequenceBurnState.Analyze(vehicle);
        PassAllocation[] allocations = Splitter.Allocate(
            maneuver.Value.DvCci.Length(), remainingCount, mode, state);

        TrueAnomaly burnTa = IsAtApoapsis ? new TrueAnomaly(Math.PI) : TrueAnomaly.Zero;
        var result = ApseBurnPlanner.Plan(
            vehicle, maneuver.Value.DvVlf, burnTa, allocations, now, execution: true);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] CircularizeIntent.RecomputePass: vehicle='{0}' kind='{1}' " +
                "passIndex={2}/{3} totalDv={4:F1}m/s remaining={5} -> {6} pass(es) " +
                "(failed={7} reason='{8}')",
                vehicle.Id, Kind, passIndex, passCountTotal,
                maneuver.Value.DvCci.Length(), remainingCount,
                result.Passes.Length, result.Failed, result.FailureReason ?? "-"));

        return IntentPlanning.FirstPass(result);
    }

    public void WriteToToml(TextWriter w)
    {
        w.WriteLine($"parent_id = \"{TomlIo.Escape(ParentId)}\"");
    }

    public static CircularizeIntent? FromToml(
        IReadOnlyDictionary<string, string> kv, bool isAtApoapsis)
    {
        if (!kv.TryGetValue("parent_id", out string? pid) || string.IsNullOrEmpty(pid))
            return null;
        return new CircularizeIntent
        {
            IsAtApoapsis = isAtApoapsis,
            ParentId = pid,
        };
    }
}
