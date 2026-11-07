using System;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// The selected transfer of the TRANSFER page as the engines fly it, built the way stock builds its
/// own in TransferPlanner.SelectedPlan and BuildFlownPlan: the delta v refined for the finite burn with
/// FiniteBurnRefiner.RefineTransfer towards the arrival, and a plan that carries the burn arc. CREATE
/// creates the burn with the refined delta v, as stock's CREATE does, and the map draws this plan. The
/// plan is built again when the entry, the craft or its full-throttle phases change. Without thrust,
/// or when the refinement fails, the burn stays the impulsive one of the porkchop entry.
/// </summary>
internal static class TransferFlownPlan
{
    // The phase buffer size stock uses for the same comparison.
    private static readonly FiniteBurnPhase[] _phases = new FiniteBurnPhase[16];
    private static int _phaseCount;
    private static Vehicle? _source;
    private static UniverseTime _start, _transit;
    private static double3 _instantDvVlf;
    private static double3 _dvVlf;
    private static FlightPlan? _plan;
    private static bool _built;

    /// <summary>The plan to draw for the selected transfer: the finite plan, or the impulsive plan of the entry when none was built.</summary>
    public static FlightPlan Plan(Vehicle source, OrbitalTransfers.PorkChopEntry entry)
    {
        Refresh(source, entry);
        return _plan ?? entry.FlightPlan;
    }

    /// <summary>The plan for a sketch, with no check of the phases: FlightComputer.GetFullThrottlePhases follows the current mass, so the phases change every frame of a burn and <see cref="Plan"/> would rebuild the trajectory each time.</summary>
    public static FlightPlan Kept(Vehicle source, OrbitalTransfers.PorkChopEntry entry)
    {
        if (!Holds(source, entry.TransferData))
            Build(source, entry, FiniteBurns.FullThrottlePhases(source));
        return _plan ?? entry.FlightPlan;
    }

    /// <summary>The departure delta v, in the VLF frame at the departure time, that CREATE gives Burn.Create.</summary>
    public static double3 DvVlf(Vehicle source, OrbitalTransfers.PorkChopEntry entry)
    {
        Refresh(source, entry);
        return _dvVlf;
    }

    public static void Reset()
    {
        _source = null;
        _plan = null;
        _built = false;
        _phaseCount = 0;
    }

    private static void Refresh(Vehicle source, OrbitalTransfers.PorkChopEntry entry)
    {
        ReadOnlySpan<FiniteBurnPhase> phases = FiniteBurns.FullThrottlePhases(source);
        if (Holds(source, entry.TransferData) && phases.SequenceEqual(_phases.AsSpan(0, _phaseCount)))
            return;
        Build(source, entry, phases);
    }

    private static bool Holds(Vehicle source, OrbitalTransfers.TransferData data)
        => _built && ReferenceEquals(source, _source) && data.Start == _start && data.Transit == _transit && data.TransferDvVlf == _instantDvVlf;

    private static void Build(Vehicle source, OrbitalTransfers.PorkChopEntry entry, ReadOnlySpan<FiniteBurnPhase> phases)
    {
        OrbitalTransfers.TransferData data = entry.TransferData;
        _built = true;
        _source = source;
        _start = data.Start;
        _transit = data.Transit;
        _instantDvVlf = data.TransferDvVlf;
        _phaseCount = Math.Min(phases.Length, _phases.Length);
        phases.Slice(0, _phaseCount).CopyTo(_phases);
        _dvVlf = _instantDvVlf;
        _plan = null;

        PatchedConic? patch = source.FlightPlan.TryFindPatch(_start);
        if (patch == null)
            return;
        try
        {
            _dvVlf = FiniteBurns.Refine(patch.Orbit, _start, _instantDvVlf, phases, FiniteBurnGoal.Transfer(data.Arrival)).DvVlf;
            FlightPlan plan = FlightPlan.CreateUninitialized(source.Hash);
            if (FiniteBurns.TryAddFiniteBurn(plan, patch, _start, _dvVlf, phases, out bool impacted) == null)
                return;
            if (!impacted)
                plan.ComputeCompleteTrajectory(out _, plan.Patches.Count + 4, 8, TransferSession.Info?.Target);
            foreach (PatchedConic piece in plan.Patches)
            {
                piece.HidePatch = false;
                piece.Orbit.UpdateCachedPoints(UpdateTaskUtils.GenerateSpacedPoints(piece));
            }
            _plan = plan;
        }
        catch (Exception ex)
        {
            // The impulsive transfer stays a valid burn, so a failed refinement costs the finite correction, not the CREATE.
            _dvVlf = _instantDvVlf;
            _plan = null;
            LogHelper.WarnOnce("afc-transfer-flown:" + ex.GetType().Name,
                $"[AFC] The finite burn plan of the selected transfer failed, the impulsive transfer is used: {ex}");
        }
    }
}
