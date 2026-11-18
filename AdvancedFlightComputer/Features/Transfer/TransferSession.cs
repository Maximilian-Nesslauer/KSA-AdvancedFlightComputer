using System;
using System.Threading;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// Scores the stock transfer window on workers and draws its Lambert costs as a binned grid.
/// Selection builds a stock trajectory, then uses the stock intercept and correction tasks.
/// </summary>
internal static class TransferSession
{
    public const int Size = 292;
    private const double TimeoutMinutes = 3.0;
    private const int AutomaticValidationLimit = 8;

    public static Vehicle? Source { get; private set; }
    public static IOrbiter? Destination { get; private set; }
    public static OrbitalTransfers.TransferInfo? Info { get; private set; }

    public static double MinTofSec;
    public static double MaxTofSec;

    public static bool Calculating { get; private set; }
    public static bool Calculated { get; private set; }
    public static int2 BestDv { get; private set; } = new(-1, -1);
    public static int2 BestDvRel { get; private set; } = new(-1, -1);
    public static int2 Selected { get; private set; } = new(-1, -1);
    public static OrbitalTransfers.PorkChopEntry? SelectedEntry { get; private set; }
    public static bool Refining => _bestAttempts.HasValue || InterceptPending;
    private static bool InterceptPending => _refine != null && !_refine.IsResultsReady;
    public static PorkchopRaster? RasterDv { get; private set; }
    public static PorkchopRaster? RasterRel { get; private set; }

    public static double DepartStartS { get; private set; }
    public static double DepartSpanS { get; private set; }
    public static double FlightMinS { get; private set; }
    public static double FlightSpanS { get; private set; }

    /// <summary>SELECT BEST DV + INTERCEPT: colour and select by the relative speed at the target as well.</summary>
    public static bool SelectBestRel;

    /// <summary>The transfer burn this session created. While it is planned, CREATE stays blocked, as stock blocks its own CREATE.</summary>
    public static Burn? TransferBurn { get; private set; }

    /// <summary>Draws the selected transfer in the world (MAP). The Hohmann multi-pass overlay follows the same switch.</summary>
    public static bool ShowMap = true;

    /// <summary>Set by the console each frame while the TRANSFER page is on screen. An uncommitted preview draws in the world only then, as stock draws its selection only while its window is open.</summary>
    public static bool PageVisible;

    /// <summary>The periapsis altitude a course correction aims for, m.</summary>
    public static double CorrectionPeAltitude = 100000.0;

    private static PorkchopScores? _scores;
    private static int? _bestAttempts;
    private static readonly HashSet<int2> _rejected = new();
    private static CancellationTokenSource? _cts;
    private static DateTime _startedAt;
    private static RefineBurnTask? _refine;
    private static CorrectionBurnTask? _correction;
    private static Burn? _correctionBurn;
    private static UniverseTime? _correctionTime;
    private static bool _correctionFailed;

    /// <summary>Progress of the calculation, 0 to 1.</summary>
    public static float Progress => Calculating ? _scores?.Progress ?? 0f : 0f;

    #region Source and destination

    /// <summary>Plans for <paramref name="source"/>. A new craft drops the calculation and the selection.</summary>
    public static void SetSource(Vehicle? source)
    {
        if (ReferenceEquals(source, Source))
            return;
        Source = source;
        Destination = null;
        ResetCalculation();
        Info = null;
    }

    /// <summary>Picks the destination and sizes the time of flight range from the Hohmann estimate, like TransferPlanner.SetTransferInfo.</summary>
    public static void SetDestination(IOrbiter? destination)
    {
        if (ReferenceEquals(destination, Destination) && Info != null)
            return;
        Destination = destination;
        ResetCalculation();
        Info = Source != null && destination != null ? TransferBounds.Build(Source, destination) : null;
        MinTofSec = Info?.MinTransferTimeOfFlight.Seconds() ?? 0.0;
        MaxTofSec = Info?.MaxTransferTimeOfFlight.Seconds() ?? 0.0;
        // An unbound target gets the same once-per-target note as in stock's planner when its ideal departure is already past.
        if (Info != null && destination?.Orbit != null && !destination.Orbit.IsBound())
            HyperbolicTargets.HyperbolicTargets.MaybeAlertDepartureWindowPast(destination, Info.HohmannTimeOfFlight);
    }

    #endregion

    #region Calculation

    /// <summary>Whether CALCULATE can start: a bound source orbit, a destination with bounds, and no calculation running.</summary>
    public static string CalculateRefusal()
    {
        if (Source == null || Source.IsDisposed)
            return "No craft";
        if (Source.Orbit?.IsBound() != true)
            return "Must have a valid orbit";
        if (Destination == null)
            return "No destination";
        if (Info == null)
            return "No transfer window for this destination";
        return "";
    }

    /// <summary>Starts a Lambert score calculation for the selected transfer window.</summary>
    public static void StartCalculation()
    {
        if (CalculateRefusal().Length > 0 || Info == null)
            return;
        ResetCalculation();

        OrbitalTransfers.TransferInfo info = Info;
        info.Starting = info.Vehicle.Orbit;
        info.MinTransferTimeOfFlight = new UniverseTime(Math.Max(1.0, MinTofSec));
        info.MaxTransferTimeOfFlight = new UniverseTime(Math.Max(MinTofSec + 1.0, MaxTofSec));
        UniverseTime now = Universe.GetElapsedTime();
        if (info.Source is not Vehicle)
            info.OptimalPhaseAngleTime = OrbitalTransfers.AlignmentTime(info, now);
        if (OrbitalTransfers.SameSoiTransfer(info))
            info.Source = info.Vehicle;

        double synodic = PatchedConic.CalculateSynodicPeriod(info.Source.Orbit, info.Target!.Orbit);
        UniverseTime range = info.MaxTransferTimeOfFlight - info.MinTransferTimeOfFlight;
        UniverseTime transitStep = range / 291L;
        UniverseTime startStep = new UniverseTime(synodic) / 291L;
        info.PorkChopData = new OrbitalTransfers.PorkChopEntry[Size, Size];
        DepartStartS = now.Seconds();
        DepartSpanS = synodic;
        FlightMinS = info.MinTransferTimeOfFlight.Seconds();
        FlightSpanS = range.Seconds();

        _cts = new CancellationTokenSource();
        _scores = new PorkchopScores(info, Size, now, info.MinTransferTimeOfFlight, startStep, transitStep, _cts.Token);
        _startedAt = DateTime.UtcNow;
        Calculating = true;
    }

    public static void CancelCalculation()
    {
        // The workers only poll IsCancellationRequested, which stays valid after Dispose.
        _cts?.Cancel();
        _cts?.Dispose();
        _cts = null;
        Calculating = false;
        _bestAttempts = null;
        if (!Calculated)
            _scores = null;
    }

    /// <summary>Collects the workers, polls the refinement and the correction, and drops a guard burn that is gone. Called every frame from the console tick.</summary>
    public static void Tick()
    {
        if (_scores != null && TransferBurn == null && !BasisCurrent())
            ResetCalculation();
        if (Calculating && !Calculated && _scores is { } scores)
        {
            if (scores.Completion.IsCompleted)
                Collect(scores);
            else if ((DateTime.UtcNow - _startedAt).TotalMinutes > TimeoutMinutes)
            {
                DefaultCategory.Log.Warning("[AFC] Transfer calculation took over three minutes and was cancelled, as stock's planner does.");
                CancelCalculation();
            }
        }
        if (_bestAttempts is int attempts)
            ValidateNextBest(attempts);

        if (_correction != null && _correction.IsResultsReady)
            FinishCorrection();

        if (TransferBurn != null)
        {
            Vehicle? source = Source;
            bool passed = TransferBurn.Time < Universe.GetElapsedTime();
            if (source == null || source.IsDisposed || !source.FlightComputer.BurnPlan.TryGetBurn(TransferBurn) || passed)
            {
                // A multi-pass keeps its current pass as the guard (see OnPassBurn), so a pass that ended only moves the guard.
                TransferBurn = null;
                _correctionTime = null;
                // Stock's DrawPlanWindow drops its result once the created burn time has passed, because every departure in it is now in the past.
                if (passed && !RunsTransferPasses(source))
                    ResetCalculation();
            }
        }
        if (_correctionBurn != null && (Source == null || !Source.FlightComputer.BurnPlan.TryGetBurn(_correctionBurn)))
            _correctionBurn = null;
    }

    private static bool RunsTransferPasses(Vehicle? source)
        => source != null && MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec) && exec.Intent is HohmannTransferIntent;

    private static void Collect(PorkchopScores scores)
    {
        OrbitalTransfers.TransferInfo? info = Info;
        if (info == null || _cts == null || _cts.IsCancellationRequested)
        {
            CancelCalculation();
            return;
        }
        if (scores.Failure is { } failure)
        {
            DefaultCategory.Log.Warning("[AFC] Transfer score calculation failed: " + failure);
            CancelCalculation();
            return;
        }
        _cts.Dispose();
        _cts = null;
        Calculated = true;

        var data = scores.Data;
        UpdateBestScores(info, data);
        RasterDv = PorkchopRaster.Build((r, c) => data[r, c].DepartureDv, (r, c) => data[r, c].Valid);
        RasterRel = PorkchopRaster.Build((r, c) => data[r, c].ArrivalSpeed, (r, c) => data[r, c].Valid);
        SelectBest(SelectBestRel);
    }

    private static void UpdateBestScores(OrbitalTransfers.TransferInfo info, PorkchopScores.Cell[,] data)
    {
        double bestDv = double.MaxValue, bestRelDv = double.MaxValue, bestRel = double.MaxValue;
        int2 best = new(-1, -1), bestR = new(-1, -1);
        for (int m = 0; m < Size; m++)
        {
            for (int n = 0; n < Size; n++)
            {
                var cell = data[m, n];
                if (!cell.Valid || _rejected.Contains(new int2(m, n)))
                    continue;
                double dv = cell.DepartureDv;
                if (dv < bestDv)
                {
                    best = new int2(m, n);
                    bestDv = dv;
                }
                double rel = cell.ArrivalSpeed;
                if (dv < bestRelDv && rel < bestRel)
                {
                    bestR = new int2(m, n);
                    bestRelDv = dv;
                    bestRel = rel;
                }
            }
        }
        info.BestDvTransferIndex = best;
        info.BestDvRelSpeedTransferIndex = bestR;
        BestDv = best;
        BestDvRel = bestR;
    }

    internal static PorkchopScores.Cell? ScoreAt(int row, int column)
        => Calculated && _scores is { } scores && (uint)row < Size && (uint)column < Size
            && scores.Data[row, column].Valid ? scores.Data[row, column] : null;

    internal static bool BasisCurrent() => _scores?.StillCurrent(Universe.GetElapsedTime()) == true;

    internal static bool IsValidated(int2 index) => ScoreAt(index.X, index.Y) != null
        && Info?.PorkChopData[index.X, index.Y]?.TransferData is { } transfer
        && !transfer.Impacts && !transfer.BadEncounter && !transfer.DepartureInfeasible;

    #endregion

    #region Selection

    /// <summary>
    /// Selects an entry of the porkchop, as stock's SetSelectedTransfer does: a copy of the entry, and
    /// when its trajectory has no encounter with the destination, the intercept refinement on the pool.
    /// </summary>
    public static void Select(int2 index)
    {
        if (!Refining)
            TrySelect(index, automatic: false);
    }

    internal static void SelectBest(bool byArrival)
    {
        if (!Calculated || Refining)
            return;
        SelectBestRel = byArrival;
        _bestAttempts = 0;
    }

    private static void ValidateNextBest(int attempts)
    {
        int2 choice = SelectBestRel && BestDvRel.X >= 0 ? BestDvRel : BestDv;
        if (choice.X >= 0 && attempts < AutomaticValidationLimit && !TrySelect(choice, automatic: true))
        {
            _rejected.Add(choice);
            if (Info != null && _scores != null)
                UpdateBestScores(Info, _scores.Data);
            _bestAttempts = attempts + 1;
            return;
        }
        if (choice.X < 0 || attempts >= AutomaticValidationLimit)
        {
            Selected = new int2(-1, -1);
            SelectedEntry = null;
            _refine = null;
        }
        _bestAttempts = null;
        Calculating = false;
    }

    private static bool TrySelect(int2 index, bool automatic)
    {
        OrbitalTransfers.TransferInfo? info = Info;
        if (info?.PorkChopData == null || info.Target is not { } target || InterceptPending
            || !BasisCurrent() || ScoreAt(index.X, index.Y) == null)
            return false;
        OrbitalTransfers.PorkChopEntry? cell = info.PorkChopData[index.X, index.Y];
        if (cell == null && _scores?.SolveForSelection(index.X, index.Y) is { } data)
        {
            cell = BuildSelection(info, target, data);
            info.PorkChopData[index.X, index.Y] = cell;
        }
        if (cell == null)
            return false;
        if (automatic && (cell.TransferData.Impacts || cell.TransferData.BadEncounter || cell.TransferData.DepartureInfeasible))
            return false;
        Selected = index;
        var entry = new OrbitalTransfers.PorkChopEntry(cell);
        SelectedEntry = entry;
        _refine = null;

        bool encounter = false;
        foreach (PatchedConic patch in entry.FlightPlan.Patches)
        {
            if (patch.EndTransition == PatchTransition.Encounter && patch.EncounterBody?.Id == Destination?.Id)
                encounter = true;
        }
        if (!encounter && !entry.TransferData.Impacts && !entry.TransferData.BadEncounter && !entry.TransferData.DepartureInfeasible)
            _refine = new RefineBurnTask(info, ref entry);
        return true;
    }

    private static OrbitalTransfers.PorkChopEntry BuildSelection(OrbitalTransfers.TransferInfo info, IOrbiter target,
                                                                 OrbitalTransfers.TransferData data)
    {
        FlightPlan flight = FlightPlan.CreateUninitialized(info.Vehicle.Hash);
        data.Point = info.Vehicle.Orbit.GetPointAt(data.Start);
        if (OrbitalTransfers.BuildFlightPlan(ref flight, info, data.Start, data.TransferDvVlf, out var point, out var closest))
        {
            data.ClosestApproachDistance = closest;
            data.Point = point;
            foreach (PatchedConic patch in flight.Patches)
            {
                if (patch.EndTransition == PatchTransition.Impact && patch.Orbit.Parent != target && patch.EndTime < data.Arrival)
                    data.Impacts = true;
                else if (patch.PrimaryBody.Id == target.Parent.Id)
                {
                    patch.CalculateSetTargetData(target, 4);
                    if (patch.TryFindClosestApproach(target) is { Body: not null } approach)
                        data.ClosestApproachDistance = Math.Min(data.ClosestApproachDistance, approach.ClosestDistance);
                }
                else if (patch.PrimaryBody.Id != info.Source.Id && patch.PrimaryBody.Id != info.Source.Parent.Id
                    && patch.PrimaryBody.Id != target.Id)
                    data.BadEncounter = true;
            }
        }
        else
            data.DepartureInfeasible = true;
        return new OrbitalTransfers.PorkChopEntry(data, flight);
    }

    /// <summary>Moves the selection by one cell, as stock's arrow buttons do.</summary>
    public static void Step(int dRow, int dCol)
    {
        if (Selected.X < 0)
            return;
        Select(new int2(Math.Clamp(Selected.X + dRow, 0, Size - 1), Math.Clamp(Selected.Y + dCol, 0, Size - 1)));
    }

    #endregion

    #region Commit guard and correction

    /// <summary>Records the burn a CREATE added, as stock records its _transferBurn.</summary>
    public static void OnCreated(Burn burn) => TransferBurn = burn;

    /// <summary>True while the one burn that CREATE made for the selection is in the burn plan. Stock then draws its trajectory, so the previews of the selection stop. A multi-pass keeps its own overlay.</summary>
    public static bool SingleBurnCreated
        => TransferBurn != null && Source != null && !MultiPass.MultiPassRegistry.Has(Source.Id);

    /// <summary>The pass a Hohmann multi-pass queued is the burn that blocks a second CREATE.</summary>
    public static void OnPassBurn(Burn burn)
    {
        if (Source != null && ReferenceEquals(burn.Vehicle, Source))
            TransferBurn = burn;
    }

    /// <summary>
    /// Drops the result once a transfer multi-pass of the session's craft has flown its final pass. Every
    /// departure in the result is then in the past, as after stock's own created burn.
    /// </summary>
    public static void OnTransferPassesCompleted(string vehicleId)
    {
        if (Source == null || Source.Id != vehicleId)
            return;
        TransferBurn = null;
        _correctionTime = null;
        ResetCalculation();
    }

    /// <summary>
    /// Whether a course correction is offered: the created transfer misses the destination and the
    /// planes differ by more than a degree, as in stock's DrawCorrectionTransfer.
    /// </summary>
    public static bool CorrectionOffered
    {
        get
        {
            if (Info?.Target == null || TransferBurn == null || Source == null || SelectedEntry == null || _correctionBurn != null)
                return false;
            if (_correctionTime is UniverseTime t && Source.FlightComputer.BurnPlan.TryGetBurn(t, out Burn? _))
                return false;
            if (Info.Target is not Astronomical target || TransferBurn.FlightPlan.Intersects(target))
                return false;
            if (!(Info.Starting.GetRelativeInclination(Info.Target.Orbit).Value() > Math.PI / 180.0))
                return false;
            var patches = TransferBurn.FlightPlan.Patches;
            return patches.Count > 0 && patches[patches.Count - 1].Orbit.IsBound();
        }
    }

    public static bool CorrectionCalculating => _correction != null && !_correction.IsResultsReady;

    public static bool CorrectionFailed => _correctionFailed;

    public static void StartCorrection()
    {
        if (!CorrectionOffered || CorrectionCalculating || Info == null || SelectedEntry == null || TransferBurn == null)
            return;
        Info.RequiredPeAltitude = CorrectionPeAltitude;
        _correctionFailed = false;
        _correction = new CorrectionBurnTask(Info, SelectedEntry.TransferData, TransferBurn.FlightPlan);
    }

    private static void FinishCorrection()
    {
        CorrectionBurnTask task = _correction!;
        _correction = null;
        if (!task.Success || TransferBurn == null || Source == null)
        {
            _correctionFailed = !task.Success;
            return;
        }
        // The task's own TransferData holds the result, because the selection can move while the task runs.
        OrbitalTransfers.TransferData data = task.TransferData;
        PatchedConic? patch = TransferBurn.FlightPlan.TryFindPatch(data.CorrectionStart);
        if (patch == null)
            return;
        Burn burn = Burn.Create(data.CorrectionPoint, data.CorrectionStart.Seconds(), data.CorrectionDvVlf, patch, Source);
        burn.IsGizmoActive = false;
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            Burn = burn,
            FlightComputer = Source.FlightComputer,
            AddBurn = true,
        });
        _correctionBurn = burn;
        _correctionTime = data.CorrectionStart;
    }

    #endregion

    #region Reset

    private static void ResetCalculation()
    {
        CancelCalculation();
        Calculated = false;
        _scores = null;
        _rejected.Clear();
        BestDv = new int2(-1, -1);
        BestDvRel = new int2(-1, -1);
        Selected = new int2(-1, -1);
        SelectedEntry = null;
        _refine = null;
        RasterDv = null;
        RasterRel = null;
        _correction = null;
        _correctionFailed = false;
    }

    /// <summary>Cancels the workers and drops every result, for a save load and the unload.</summary>
    public static void CancelAll()
    {
        ResetCalculation();
        Source = null;
        Destination = null;
        Info = null;
        TransferBurn = null;
        _correctionBurn = null;
        _correctionTime = null;
        MinTofSec = 0.0;
        MaxTofSec = 0.0;
        SelectBestRel = false;
        ShowMap = true;
        PageVisible = false;
        CorrectionPeAltitude = 100000.0;
    }

    #endregion
}
