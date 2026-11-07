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
/// The AFC porkchop calculation for the TRANSFER page, without a GPU texture. It reproduces the
/// orchestration of the private TransferTask.Run with the public TransferTask.WorkerTask: the same
/// bounds as the private TransferPlanner.SetTransferInfo (see TransferBounds), the same synodic sweep
/// in steps of 1/291, the same worker split, and the same best-entry filters. The workers run on the
/// thread pool as stock's do; the session reads their arrays only after every Worker.DoneEvent is set,
/// from the console tick on the main thread, also while the console is closed. The result is drawn as
/// a binned grid (PorkchopRaster). The selection, the intercept refinement (the public RefineBurnTask)
/// and the course correction (the public CorrectionBurnTask) follow stock's planner. A created transfer
/// burn blocks a second CREATE until it is flown or deleted, like stock's own guard.
/// The stock planner keeps its own calculation in its own TransferInfo, so the two never share state.
/// </summary>
internal static class TransferSession
{
    public const int Size = 292;
    private const double TimeoutMinutes = 3.0;

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
    public static bool Refining => _refine != null && !_refine.IsResultsReady;
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

    private static TransferTask.Worker[]? _workers;
    private static TransferTask.WorkerTask[]? _tasks;
    private static CancellationTokenSource? _cts;
    private static DateTime _startedAt;
    private static RefineBurnTask? _refine;
    private static CorrectionBurnTask? _correction;
    private static Burn? _correctionBurn;
    private static UniverseTime? _correctionTime;
    private static bool _correctionFailed;

    /// <summary>Progress of the calculation, 0 to 1.</summary>
    public static float Progress
    {
        get
        {
            if (_workers == null || _workers.Length == 0)
                return 0f;
            long done = 0, total = 0;
            foreach (TransferTask.Worker w in _workers)
            {
                if (w == null)
                    continue;
                done += w.AmountComplete;
                total += w.TotalWork;
            }
            return total > 0 ? Math.Clamp((float)done / total, 0f, 1f) : 0f;
        }
    }

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
        if (!(Source.Orbit?.Eccentricity < 1.0))
            return "Must have a valid orbit";
        if (Destination == null)
            return "No destination";
        if (Info == null)
            return "No transfer window for this destination";
        return "";
    }

    /// <summary>Starts the calculation: the orchestration of TransferTask.Run, with the workers queued on the thread pool.</summary>
    public static void StartCalculation()
    {
        if (CalculateRefusal().Length > 0 || Info == null)
            return;
        ResetCalculation();

        OrbitalTransfers.TransferInfo info = Info;
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
        CancellationToken token = _cts.Token;
        int workerCount = Math.Max(1, TransferTask.WorkerCount);
        int baseRows = Size / workerCount;
        int extra = Size % workerCount;
        _workers = new TransferTask.Worker[workerCount];
        _tasks = new TransferTask.WorkerTask[workerCount];
        int offset = 0;
        for (int i = 0; i < workerCount; i++)
        {
            int rows = baseRows + (i < extra ? 1 : 0);
            _workers[i] = new TransferTask.Worker(initialState: false, rows);
            _tasks[i] = new TransferTask.WorkerTask(_workers[i], new OrbitalTransfers.TransferInfo(info, usePorkChopData: false),
                new OrbitalTransfers.PorkChopEntry[rows, Size], offset, rows, now, info.MinTransferTimeOfFlight, transitStep, startStep, token);
            ThreadPool.QueueUserWorkItem(_tasks[i].ThreadPoolCallback, i);
            offset += rows;
        }
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
        _workers = null;
        _tasks = null;
    }

    /// <summary>Collects the workers, polls the refinement and the correction, and drops a guard burn that is gone. Called every frame from the console tick.</summary>
    public static void Tick()
    {
        if (Calculating && _workers != null && _tasks != null)
        {
            bool done = true;
            foreach (TransferTask.Worker w in _workers)
            {
                if (!w.DoneEvent.WaitOne(0))
                {
                    done = false;
                    break;
                }
            }
            if (done)
                Collect();
            else if ((DateTime.UtcNow - _startedAt).TotalMinutes > TimeoutMinutes)
            {
                DefaultCategory.Log.Warning("[AFC] Transfer calculation took over three minutes and was cancelled, as stock's planner does.");
                CancelCalculation();
            }
        }

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

    private static void Collect()
    {
        OrbitalTransfers.TransferInfo? info = Info;
        TransferTask.WorkerTask[] tasks = _tasks!;
        Calculating = false;
        _workers = null;
        _tasks = null;
        if (info == null || _cts == null || _cts.IsCancellationRequested)
            return;

        var data = new OrbitalTransfers.PorkChopEntry[Size, Size];
        foreach (TransferTask.WorkerTask task in tasks)
        {
            for (int k = 0; k < task.Rows; k++)
                for (int l = 0; l < Size; l++)
                    data[task.Offset + k, l] = task.PorkChopData[k, l];
        }
        info.PorkChopData = data;

        // The filters of TransferTask.Run: no impact, no encounter with another body, a feasible departure.
        double bestDv = double.MaxValue, bestRelDv = double.MaxValue, bestRel = double.MaxValue;
        int2 best = new(-1, -1), bestR = new(-1, -1);
        for (int m = 0; m < Size; m++)
        {
            for (int n = 0; n < Size; n++)
            {
                OrbitalTransfers.PorkChopEntry e = data[m, n];
                if (e?.TransferData == null)
                    continue;
                OrbitalTransfers.TransferData t = e.TransferData;
                if (t.Impacts || t.BadEncounter || t.DepartureInfeasible)
                    continue;
                double dv = t.TransferDvVlf.Length();
                if (dv < bestDv)
                {
                    best = new int2(m, n);
                    bestDv = dv;
                }
                double rel = t.RelativeVelocityCci.Length();
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

        RasterDv = PorkchopRaster.Build((r, c) => data[r, c]?.TransferData?.TransferDvVlf.Length() ?? double.NaN, (r, c) => Usable(data[r, c]));
        RasterRel = PorkchopRaster.Build((r, c) => data[r, c]?.TransferData?.RelativeVelocityCci.Length() ?? double.NaN, (r, c) => Usable(data[r, c]));
        Calculated = true;
        if (best.X >= 0)
            Select(SelectBestRel && bestR.X >= 0 ? bestR : best);
    }

    // Stock paints impacts and infeasible departures black and every other entry by its value.
    private static bool Usable(OrbitalTransfers.PorkChopEntry? e)
        => e?.TransferData != null && !e.TransferData.Impacts && !e.TransferData.DepartureInfeasible;

    #endregion

    #region Selection

    /// <summary>
    /// Selects an entry of the porkchop, as stock's SetSelectedTransfer does: a copy of the entry, and
    /// when its trajectory has no encounter with the destination, the intercept refinement on the pool.
    /// </summary>
    public static void Select(int2 index)
    {
        OrbitalTransfers.TransferInfo? info = Info;
        if (!Calculated || info?.PorkChopData == null || Refining)
            return;
        if (index.X < 0 || index.Y < 0 || index.X >= info.PorkChopData.GetLength(0) || index.Y >= info.PorkChopData.GetLength(1))
            return;
        OrbitalTransfers.PorkChopEntry? cell = info.PorkChopData[index.X, index.Y];
        if (cell == null)
            return;
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
        if (!encounter)
            _refine = new RefineBurnTask(info, ref entry);
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
            return patches.Count > 0 && patches[patches.Count - 1].Orbit.Eccentricity < 1.0;
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
