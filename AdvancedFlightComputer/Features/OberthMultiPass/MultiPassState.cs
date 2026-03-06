using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

/// <summary>
/// Called per pass during preview and creation to compute the dV vector for
/// a partial maneuver. The fraction parameter (0..1) is used for plane changes
/// (arcsin-based partial rotation). Apse burns use null GoalFunc (equal-VLF path).
/// </summary>
delegate OrbitManeuvers.ManeuverResult? GoalFunc(Orbit orbit, SimTime afterTime, double fraction);

record struct PassResult(SimTime BurnTime, double3 DvVlf, double EstimatedBurnTime, FlightPlan PreviewFP);

/// <summary>
/// Pure state container for OberthMultiPass.
/// </summary>
static class MultiPassState
{
    #region Preview state

    public static List<PassResult>? PreviewPasses;
    public static Vehicle? PreviewSource;
    public static bool ShowOrbitPreview;
    public static GoalFunc? PreviewGoal;       // null = apse burn (equal-VLF path)
    public static double3 PreviewDvDirection;  // unit vector for apse burns, e.g. (1,0,0) prograde

    #endregion

    #region Active split state

    public static Vehicle? Vehicle;
    public static List<Burn>? PassBurns;
    public static double3 OriginalDvVlf;
    public static SimTime OriginalBurnTime;
    public static int OriginalPassCount;
    public static int SelectedPassIndex;
    public static double[]? OriginalDvCapacities;

    #endregion

    #region Correction state

    public static GoalFunc? CorrectionGoal;  // null = generic (dV redistribution)

    #endregion

    #region Derived

    public static bool HasActiveSplit => Vehicle != null && PassBurns is { Count: > 0 };
    public static bool HasPreview     => PreviewPasses != null;
    public static bool HasExecutedPasses => HasActiveSplit && PassBurns!.Count < OriginalPassCount;

    #endregion

    /// <summary>
    /// Removes completed/deleted burns from PassBurns. Resets state if all
    /// burns are gone or if the controlled vehicle has changed.
    /// </summary>
    public static void ValidateState()
    {
        if (Vehicle == null || PassBurns == null)
            return;

        if (Vehicle != Program.ControlledVehicle)
        {
            Reset();
            return;
        }

        for (int i = PassBurns.Count - 1; i >= 0; i--)
        {
            if (!Vehicle.FlightComputer.BurnPlan.TryGetBurn(PassBurns[i]))
                PassBurns.RemoveAt(i);
        }

        if (PassBurns.Count == 0)
        {
            Reset();
            return;
        }

        SelectedPassIndex = Math.Clamp(SelectedPassIndex, 0, PassBurns.Count - 1);
    }

    /// <summary>
    /// Clears all preview state. Called when the plan type changes or burns are created.
    /// </summary>
    public static void ClearPreview()
    {
        PreviewPasses = null;
        PreviewSource = null;
        ShowOrbitPreview = false;
        PreviewGoal = null;
        PreviewDvDirection = default;
    }

    /// <summary>
    /// Resets all state (preview + active split + correction).
    /// Called on mod unload and when the active split completes.
    /// </summary>
    public static void Reset()
    {
        ClearPreview();
        Vehicle = null;
        PassBurns = null;
        OriginalDvVlf = default;
        OriginalBurnTime = default;
        OriginalPassCount = 0;
        SelectedPassIndex = 0;
        OriginalDvCapacities = null;
        CorrectionGoal = null;
    }
}
