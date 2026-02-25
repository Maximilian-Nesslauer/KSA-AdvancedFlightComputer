using System.Collections.Generic;
using System.Diagnostics;
using AdvancedFlightComputer.Core;
using KSA;

namespace AdvancedFlightComputer.Features.StageInfo;

/// <summary>
/// Centralized cache for stage analysis results. Decoupled from UI rendering
/// so analysis is available regardless of whether the staging panel is visible.
///
/// Called from Patch_CorrectedBurnDuration.Postfix (on UpdateFromTaskResults)
/// which fires every frame for the controlled vehicle. Results are consumed by:
/// - StageInfoPanel (UI rendering when panel is visible)
/// - Patch_CorrectedBurnDuration (main-thread burn duration/ignition correction)
/// - CorrectedBurnState -> Patch_WorkerIgnitionTiming (worker-thread ignition)
///
/// Analysis is skipped when neither a burn is planned nor the staging panel is
/// visible, avoiding unnecessary computation.
/// </summary>
static class StageAnalysisCache
{
    private static VehicleBurnAnalysis? _cachedAnalysis;
    private static readonly Dictionary<int, StageBurnInfo> _stageInfoLookup = new();
    private static BurnAnalysis? _cachedBurnAnalysis;
    private static readonly Dictionary<int, BurnStageAllocation> _burnAllocationLookup = new();

    /// <summary>
    /// Set by DrawContentPrefix each frame the staging panel is rendered.
    /// Read and reset by Update() to determine if analysis is needed for
    /// UI display. One-frame lag when panel first opens is invisible.
    /// </summary>
    private static bool _panelNeedsData;

    #region Public API

    /// <summary>
    /// Signals that the staging panel needs analysis data next frame.
    /// Called from StageInfoPanel.DrawContentPrefix every rendered frame.
    /// </summary>
    public static void MarkPanelActive() => _panelNeedsData = true;

    /// <summary>
    /// Runs stage analysis if needed (burn planned or panel visible).
    /// Called once per frame from Patch_CorrectedBurnDuration.Postfix.
    /// </summary>
    public static void Update(Vehicle vehicle)
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        bool hasBurn = vehicle.FlightComputer.Burn != null;
        bool panelActive = _panelNeedsData;
        _panelNeedsData = false;

        if (!hasBurn && !panelActive)
        {
            Clear();
            return;
        }

        _cachedAnalysis = StageAnalyzer.Analyze(vehicle);

        _stageInfoLookup.Clear();
        foreach (var stage in _cachedAnalysis.Value.Stages)
            _stageInfoLookup[stage.StageNumber] = stage;

        if (hasBurn)
            UpdateBurnAnalysis(vehicle.FlightComputer.Burn!);
        else
            ClearBurnAnalysis();

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("StageAnalysisCache.Update", Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    public static VehicleBurnAnalysis? Analysis => _cachedAnalysis;
    public static BurnAnalysis? BurnAnalysis => _cachedBurnAnalysis;

    public static bool TryGetStageInfo(int stageNumber, out StageBurnInfo info)
        => _stageInfoLookup.TryGetValue(stageNumber, out info);

    public static bool TryGetBurnAllocation(int stageNumber, out BurnStageAllocation alloc)
        => _burnAllocationLookup.TryGetValue(stageNumber, out alloc);

    /// <summary>
    /// Returns the cached multi-stage burn time, or null if no burn analysis
    /// is available. Used by Patch_CorrectedBurnDuration and CorrectedBurnState.
    /// </summary>
    public static float? GetCorrectedBurnDuration()
        => _cachedBurnAnalysis?.TotalBurnTime;

    public static void Reset()
    {
        Clear();
        _panelNeedsData = false;
    }

    #endregion

    #region Private Helpers

    private static void UpdateBurnAnalysis(BurnTarget burn)
    {
        if (_cachedAnalysis == null)
        {
            ClearBurnAnalysis();
            return;
        }

        float requiredDv = burn.DeltaVToGoCci.Length();
        if (requiredDv <= 0f)
        {
            ClearBurnAnalysis();
            return;
        }

        _cachedBurnAnalysis = StageAnalyzer.AnalyzeBurn(_cachedAnalysis.Value, requiredDv);

        _burnAllocationLookup.Clear();
        foreach (var alloc in _cachedBurnAnalysis.Value.StageAllocations)
            _burnAllocationLookup[alloc.StageNumber] = alloc;
    }

    private static void ClearBurnAnalysis()
    {
        _cachedBurnAnalysis = null;
        _burnAllocationLookup.Clear();
    }

    private static void Clear()
    {
        _cachedAnalysis = null;
        _stageInfoLookup.Clear();
        ClearBurnAnalysis();
    }

    #endregion
}
