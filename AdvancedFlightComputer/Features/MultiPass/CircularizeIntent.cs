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
/// "Circularize the orbit at the chosen apse, burning at that apse."
/// IsAtApoapsis = true burns at apoapsis to raise periapsis; false burns
/// at periapsis to lower apoapsis. The goal radius is not stored: burns
/// at one apse do not move that apse to first order, so re-deriving the
/// circular target from the live orbit each pass is both simpler and
/// more robust than locking a radius at start.
/// </summary>
internal sealed class CircularizeIntent : IManeuverIntent
{
    public const string CircularizeApoapsisKind = "circularize-ap";
    public const string CircularizePeriapsisKind = "circularize-pe";

    // User-visible "circular enough" bar, shared with
    // OrbitManeuvers.ComputeCircularize so the planner stops producing
    // burns and IsSatisfied flips at the same threshold. Matches the
    // 0.001-radian inclination tolerance for symmetry across intents.
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
            vehicle, maneuver.Value.DvVlf, burnTa, allocations, now);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] CircularizeIntent.RecomputePass: vehicle='{0}' kind='{1}' " +
                "passIndex={2}/{3} totalDv={4:F1}m/s remaining={5} -> {6} pass(es) " +
                "(failed={7} reason='{8}')",
                vehicle.Id, Kind, passIndex, passCountTotal,
                maneuver.Value.DvCci.Length(), remainingCount,
                result.Passes.Length, result.Failed, result.FailureReason ?? "-"));

        if (result.Passes.Length == 0)
            return PassPlanResult.Failure(result.FailureReason ?? "planner produced no passes");
        return PassPlanResult.Success(result.Passes[0]);
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
