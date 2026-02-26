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
///
/// Supports dual analysis for VacAsl display mode. StageAnalyzer uses pooled
/// collections internally, so results must be copied into cache-owned lists
/// before the second Analyze() call overwrites the pool.
/// </summary>
static class StageAnalysisCache
{
    #region Primary Analysis

    private static VehicleBurnAnalysis? _cachedAnalysis;
    private static readonly List<StageBurnInfo> _cachedPrimaryStages = new();
    private static readonly Dictionary<int, StageBurnInfo> _stageInfoLookup = new();
    private static BurnAnalysis? _cachedBurnAnalysis;
    private static readonly List<BurnStageAllocation> _cachedPrimaryAllocations = new();
    private static readonly Dictionary<int, BurnStageAllocation> _burnAllocationLookup = new();

    #endregion

    #region Secondary Analysis (VacAsl mode)

    private static VehicleBurnAnalysis? _cachedSecondaryAnalysis;
    private static readonly List<StageBurnInfo> _cachedSecondaryStages = new();
    private static readonly Dictionary<int, StageBurnInfo> _secondaryStageInfoLookup = new();
    private static BurnAnalysis? _cachedSecondaryBurnAnalysis;
    private static readonly List<BurnStageAllocation> _cachedSecondaryAllocations = new();
    private static readonly Dictionary<int, BurnStageAllocation> _secondaryBurnAllocationLookup = new();

    #endregion

    #region Display State

    public static string PrimaryLabel { get; private set; } = "";
    public static string? SecondaryLabel { get; private set; }
    public static bool IsPrimaryCurrentCondition { get; private set; } = true;

    #endregion

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

        var env = StageInfoSettings.ResolveEnvironment(vehicle);
        PrimaryLabel = env.PrimaryLabel;
        SecondaryLabel = env.SecondaryLabel;
        IsPrimaryCurrentCondition = env.IsPrimaryCurrentCondition;

        RunPrimaryAnalysis(vehicle, env);

        if (env.SecondaryPressure.HasValue)
            RunSecondaryAnalysis(vehicle, env);
        else
            ClearSecondary();

        float requiredDv = hasBurn ? vehicle.FlightComputer.Burn!.DeltaVToGoCci.Length() : 0f;
        if (hasBurn && requiredDv > 0f)
        {
            UpdateBurnAnalysis(requiredDv);
            if (_cachedSecondaryAnalysis != null)
                UpdateSecondaryBurnAnalysis(requiredDv);
            else
                ClearSecondaryBurnAnalysis();
        }
        else
        {
            ClearBurnAnalysis();
            ClearSecondaryBurnAnalysis();
        }

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("StageAnalysisCache.Update", Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    public static VehicleBurnAnalysis? Analysis => _cachedAnalysis;
    public static BurnAnalysis? BurnAnalysis => _cachedBurnAnalysis;
    public static VehicleBurnAnalysis? SecondaryAnalysis => _cachedSecondaryAnalysis;
    public static BurnAnalysis? SecondaryBurnAnalysis => _cachedSecondaryBurnAnalysis;

    public static bool TryGetStageInfo(int stageNumber, out StageBurnInfo info)
        => _stageInfoLookup.TryGetValue(stageNumber, out info);

    public static bool TryGetBurnAllocation(int stageNumber, out BurnStageAllocation alloc)
        => _burnAllocationLookup.TryGetValue(stageNumber, out alloc);

    public static bool TryGetSecondaryStageInfo(int stageNumber, out StageBurnInfo info)
        => _secondaryStageInfoLookup.TryGetValue(stageNumber, out info);

    public static bool TryGetSecondaryBurnAllocation(int stageNumber, out BurnStageAllocation alloc)
        => _secondaryBurnAllocationLookup.TryGetValue(stageNumber, out alloc);

    /// <summary>
    /// Returns the cached multi-stage burn time, or null if no burn analysis
    /// is available. Used by Patch_CorrectedBurnDuration and CorrectedBurnState.
    /// </summary>
    public static float? GetCorrectedBurnDuration()
        => _cachedBurnAnalysis?.TotalBurnTime;

    public static void Reset()
    {
        Clear();
        ClearSecondary();
        _panelNeedsData = false;
        PrimaryLabel = "";
        SecondaryLabel = null;
        IsPrimaryCurrentCondition = true;
    }

    #endregion

    #region Private Helpers

    private static void RunPrimaryAnalysis(Vehicle vehicle, AnalysisEnvironment env)
    {
        var result = StageAnalyzer.Analyze(vehicle,
            ambientPressure: env.PrimaryPressure,
            surfaceGravityOverride: env.PrimarySurfaceGravity);

        _cachedPrimaryStages.Clear();
        _cachedPrimaryStages.AddRange(result.Stages);

        _cachedAnalysis = new VehicleBurnAnalysis
        {
            Stages = _cachedPrimaryStages,
            TotalDeltaV = result.TotalDeltaV,
            TotalBurnTime = result.TotalBurnTime
        };

        _stageInfoLookup.Clear();
        foreach (var stage in _cachedPrimaryStages)
            _stageInfoLookup[stage.StageNumber] = stage;
    }

    private static void RunSecondaryAnalysis(Vehicle vehicle, AnalysisEnvironment env)
    {
        var result = StageAnalyzer.Analyze(vehicle,
            ambientPressure: env.SecondaryPressure!.Value,
            surfaceGravityOverride: env.SecondarySurfaceGravity);

        _cachedSecondaryStages.Clear();
        _cachedSecondaryStages.AddRange(result.Stages);

        _cachedSecondaryAnalysis = new VehicleBurnAnalysis
        {
            Stages = _cachedSecondaryStages,
            TotalDeltaV = result.TotalDeltaV,
            TotalBurnTime = result.TotalBurnTime
        };

        _secondaryStageInfoLookup.Clear();
        foreach (var stage in _cachedSecondaryStages)
            _secondaryStageInfoLookup[stage.StageNumber] = stage;
    }

    private static void UpdateBurnAnalysis(float requiredDv)
    {
        if (_cachedAnalysis == null)
        {
            ClearBurnAnalysis();
            return;
        }

        var result = StageAnalyzer.AnalyzeBurn(_cachedAnalysis.Value, requiredDv);

        _cachedPrimaryAllocations.Clear();
        _cachedPrimaryAllocations.AddRange(result.StageAllocations);

        _cachedBurnAnalysis = new BurnAnalysis
        {
            RequiredDv = result.RequiredDv,
            AvailableDv = result.AvailableDv,
            TotalBurnTime = result.TotalBurnTime,
            IsSufficient = result.IsSufficient,
            StageAllocations = _cachedPrimaryAllocations
        };

        _burnAllocationLookup.Clear();
        foreach (var alloc in _cachedPrimaryAllocations)
            _burnAllocationLookup[alloc.StageNumber] = alloc;
    }

    private static void UpdateSecondaryBurnAnalysis(float requiredDv)
    {
        if (_cachedSecondaryAnalysis == null)
        {
            ClearSecondaryBurnAnalysis();
            return;
        }

        var result = StageAnalyzer.AnalyzeBurn(_cachedSecondaryAnalysis.Value, requiredDv);

        _cachedSecondaryAllocations.Clear();
        _cachedSecondaryAllocations.AddRange(result.StageAllocations);

        _cachedSecondaryBurnAnalysis = new BurnAnalysis
        {
            RequiredDv = result.RequiredDv,
            AvailableDv = result.AvailableDv,
            TotalBurnTime = result.TotalBurnTime,
            IsSufficient = result.IsSufficient,
            StageAllocations = _cachedSecondaryAllocations
        };

        _secondaryBurnAllocationLookup.Clear();
        foreach (var alloc in _cachedSecondaryAllocations)
            _secondaryBurnAllocationLookup[alloc.StageNumber] = alloc;
    }

    private static void ClearBurnAnalysis()
    {
        _cachedBurnAnalysis = null;
        _burnAllocationLookup.Clear();
    }

    private static void ClearSecondaryBurnAnalysis()
    {
        _cachedSecondaryBurnAnalysis = null;
        _secondaryBurnAllocationLookup.Clear();
    }

    private static void ClearSecondary()
    {
        _cachedSecondaryAnalysis = null;
        _cachedSecondaryStages.Clear();
        _secondaryStageInfoLookup.Clear();
        ClearSecondaryBurnAnalysis();
    }

    private static void Clear()
    {
        _cachedAnalysis = null;
        _cachedPrimaryStages.Clear();
        _stageInfoLookup.Clear();
        ClearBurnAnalysis();
    }

    #endregion
}
