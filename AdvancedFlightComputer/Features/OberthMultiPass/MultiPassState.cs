using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
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

record struct GoalFuncSet(
    GoalFunc? PreviewGoal,
    GoalFunc? CorrectionGoal,
    double3 DvDirection,
    TrueAnomaly BurnTa,
    Func<Orbit, double>? TotalAngleFunc);

/// <summary>
/// Pure state container for OberthMultiPass.
/// </summary>
static class MultiPassState
{
    #region Preview state

    public static List<PassResult>? PreviewPasses;
    public static Vehicle? PreviewSource;
    public static bool ShowOrbitPreview;
    public static bool PreviewFailed;          // true when preview was attempted but produced < 2 passes
    public static GoalFunc? PreviewGoal;       // null = apse burn (equal-VLF path)
    public static double3 PreviewDvDirection;  // unit vector for apse burns, e.g. (1,0,0) prograde

    #endregion

    #region Active split state

    public static Vehicle? Vehicle;
    public static List<Burn>? PassBurns;
    // The single burn recreated by RemoveAllBurns (BackToSingle).
    // Lifecycle: set in RemoveAllBurns -> consumed+cleared in CreateBurns -> cleared in Reset.
    // Must be removed before CreateBurns adds pass burns, otherwise CalculateNewFlightPlans
    // chains pass flight plans from the post-singleBurn orbit instead of the actual orbit.
    // If the user manually deletes it before clicking Create, TryGetBurn returns false and
    // the stale reference is harmlessly cleared.
    public static Burn? RestoredSingleBurn;
    public static double3 OriginalDvVlf;
    public static SimTime OriginalBurnTime;
    public static int OriginalPassCount;
    public static int SelectedPassIndex;
    // Phase 2.5: used for per-pass dV reset across correction cycles.
    public static double[]? OriginalDvCapacities;
    public static double[]? PlannedBurnTimes;  // EstimatedBurnTime per pass from BurnTimeSplitter, for burn time display
    public static double3 ActiveDvDirection;   // persists after ClearPreview; used by HandlePassCompletion
    public static TrueAnomaly ActiveBurnTa;    // persists after ClearPreview; used by HandlePassCompletion
    // True for apse burns (Set Ap/Pe), false for goal-based burns (plane changes).
    public static bool IsApseBurn;

    #endregion

    #region Correction state

    public static GoalFunc? CorrectionGoal;  // null = generic (dV redistribution)
    // Provides the exact geometric remaining angle for plane-change corrections.
    // Stored at CreateBurns time so HandlePassCompletion uses the precise formula
    // rather than the dV-inversion fallback in ComputeGoalPreview.
    public static Func<Orbit, double>? TotalAngleFunc;

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
            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    "[AFC] MultiPassState.ValidateState: all burns gone, resetting state.");
            Reset();
            return;
        }

        SelectedPassIndex = Math.Clamp(SelectedPassIndex, 0, PassBurns.Count - 1);
    }

    /// <summary>
    /// Extracts dV capacities from the current preview passes. Returns null if no
    /// preview exists. Used when transitioning from preview to active split or when
    /// recomputing corrected passes.
    /// </summary>
    public static double[]? ExtractPreviewDvCapacities()
    {
        if (PreviewPasses == null) return null;
        var caps = new double[PreviewPasses.Count];
        for (int i = 0; i < caps.Length; i++)
            caps[i] = PreviewPasses[i].DvVlf.Length();
        return caps;
    }

    /// <summary>
    /// Clears all preview state. Called when the plan type changes or burns are created.
    /// </summary>
    public static void ClearPreview()
    {
        PreviewPasses = null;
        PreviewSource = null;
        ShowOrbitPreview = false;
        PreviewFailed = false;
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
        RestoredSingleBurn = null;
        OriginalDvVlf = default;
        OriginalBurnTime = default;
        OriginalPassCount = 0;
        SelectedPassIndex = 0;
        OriginalDvCapacities = null;
        PlannedBurnTimes = null;
        ActiveDvDirection = default;
        ActiveBurnTa = TrueAnomaly.NaN;
        IsApseBurn = false;
        CorrectionGoal = null;
        TotalAngleFunc = null;
    }
}
