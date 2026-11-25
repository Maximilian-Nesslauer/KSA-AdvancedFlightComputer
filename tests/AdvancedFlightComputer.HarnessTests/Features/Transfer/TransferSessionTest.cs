using System.Reflection;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Console.Pages;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The AFC porkchop without the stock planner. A craft in a low orbit of the home body plans to the
// first moon of the home body, or without one to a sibling of the home body. The calculation runs on
// the thread pool with the public TransferTask.WorkerTask, the session collects it from its tick, the
// best entry is selected, the binned raster has cells and its lowest value is not above the best
// departure dV, and the calculation flag of the stock planner is not touched. A cancelled calculation
// never reports a result, and CancelAll drops everything. The best departure depends on where the
// destination is when the calculation starts, so a second case plans again at a fixed epoch. There the
// lowest Lambert costs belong to more than eight cells whose arcs pass below the surface of the home body.
public sealed class TransferSessionTest : AfcTest
{
    private static readonly TimeSpan CalculationTimeout = TimeSpan.FromMinutes(3);
    private static readonly TimeSpan RefineTimeout = TimeSpan.FromMinutes(1);
    private const double FixedEpochSeconds = 98589.69;
    private const double EpochStepSeconds = 10.0;

    public override string Name => "afc-transfer-session";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IOrbiter? destination = FindDestination(home);
        if (destination == null)
        {
            t.Skip("the loaded system has no moon of the home body and no sibling of it");
            return;
        }
        VehicleSave? save = DefaultVehicleSaves.FindSave("Rocket");
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("default vehicle", "Rocket is not available");
            return;
        }

        t.Info($"case: the current epoch {Universe.GetElapsedSeconds():F3} s");
        PlanOnce(t, home, destination, save, firstCase: true);
        CheckConcurrencyAndValidationRegressions(t, home, destination, save);

        // The sim clock only runs forward, so the fixed epoch is reachable only when no earlier test passed it.
        double wait = FixedEpochSeconds - Universe.GetElapsedSeconds();
        if (wait < 0.0)
        {
            t.Skip($"the sim clock is already at {Universe.GetElapsedSeconds():F3} s, after the fixed epoch {FixedEpochSeconds:F3} s");
            return;
        }
        t.Session.CreateDriver().StepFor(wait, EpochStepSeconds);
        t.Info($"case: the fixed epoch {Universe.GetElapsedSeconds():F3} s");
        PlanOnce(t, home, destination, save, firstCase: false);
    }

    private static void PlanOnce(TestContext t, IParentBody home, IOrbiter destination, VehicleSave save, bool firstCase)
    {
        FieldInfo? stockCalculated = AccessTools.Field(typeof(TransferPlanner), "_transferCalculated");
        object? stockBefore = stockCalculated?.GetValue(null);
        string id = "TransferSession_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        TransferSession.CancelAll();
        try
        {
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance!, id,
                OrbitFixtures.CircularAt(home, 400_000, Universe.GetElapsedTime()));
            t.Info($"planning from {((Astronomical)home).Id} orbit to {((Astronomical)destination).Id}");

            TransferSession.SetSource(vehicle);
            if (firstCase)
                CheckMissionDefault(t, vehicle);
            TransferSession.SetDestination(destination);
            t.Check("the destination has transfer bounds", TransferSession.Info != null
                && TransferSession.MinTofSec > 0.0 && TransferSession.MaxTofSec > TransferSession.MinTofSec,
                $"min {TransferSession.MinTofSec} s, max {TransferSession.MaxTofSec} s");
            string refusal = TransferSession.CalculateRefusal();
            if (!t.Check("CALCULATE is allowed", refusal.Length == 0, refusal))
                return;

            if (firstCase)
                CheckCancelReportsNothing(t);

            TransferSession.StartCalculation();
            t.Check("the calculation starts", TransferSession.Calculating);
            DateTime start = DateTime.UtcNow;
            while (TransferSession.Calculating && DateTime.UtcNow - start < CalculationTimeout)
            {
                Thread.Sleep(50);
                TransferSession.Tick();
            }
            t.Info($"calculation took {(DateTime.UtcNow - start).TotalSeconds:F1} s");
            if (!t.Check("the session collects a result", TransferSession.Calculated && !TransferSession.Calculating))
                return;

            OrbitalTransfers.TransferInfo info = TransferSession.Info!;
            LogPropagated(t, info);
            int2 best = TransferSession.BestDv;
            bool bestValid = best.X >= 0 && best.Y >= 0 && info.PorkChopData?[best.X, best.Y]?.TransferData != null;
            if (!t.Check("a best departure is found", bestValid, $"index {best.X}, {best.Y}"))
                return;
            double bestDv = info.PorkChopData![best.X, best.Y].TransferData.TransferDvVlf.Length();
            int propagated = info.PorkChopData.Cast<OrbitalTransfers.PorkChopEntry?>().Count(entry => entry != null);
            t.Check("only a short list of selected candidates propagated", propagated is > 0 and <= 8,
                $"{propagated} trajectories for {TransferSession.Size * TransferSession.Size} grid points");
            t.Check("the selected cell costs agree with the full transfer",
                TransferSession.ScoreAt(best.X, best.Y) is { } score && Math.Abs(score.DepartureDv - bestDv) < 0.001);
            t.Check("the best departure dV is finite and positive", double.IsFinite(bestDv) && bestDv > 0.0, $"{bestDv:F1} m/s");
            t.Check("the best entry is selected", TransferSession.Selected.X == best.X && TransferSession.Selected.Y == best.Y
                && TransferSession.SelectedEntry != null);

            CheckRaster(t, TransferSession.RasterDv, bestDv);

            // The refinement of the selection runs on the pool and reads the craft, so it has to end before the craft goes.
            DateTime refineStart = DateTime.UtcNow;
            while (TransferSession.Refining && DateTime.UtcNow - refineStart < RefineTimeout)
                Thread.Sleep(50);
            TransferSession.Tick();
            t.Check("the intercept refinement ends", !TransferSession.Refining);
            CheckDepartureImpactScore(t, info);

            if (stockCalculated != null)
                t.Check("the calculation flag of the stock planner is untouched", Equals(stockCalculated.GetValue(null), stockBefore));
            else
                t.Skip("TransferPlanner._transferCalculated not found, so the stock flag is not compared");

            TransferSession.CancelAll();
            t.Check("CancelAll drops the session", TransferSession.Source == null && TransferSession.Info == null
                && !TransferSession.Calculated && TransferSession.RasterDv == null && TransferSession.SelectedEntry == null);
        }
        finally
        {
            TransferSession.CancelAll();
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void CheckMissionDefault(TestContext t, Vehicle vehicle)
    {
        var choices = new List<TransferObject>();
        TransferBounds.Destinations(vehicle, choices);
        int mars = choices.FindIndex(choice => choice.GetKey() == "Mars");
        int other = choices.FindIndex(choice => choice.GetKey() != "Mars" && choice.Body is IOrbiter);
        if (mars < 0 || other < 0)
        {
            t.Skip("the source has no reachable Mars and alternative for the mission default");
            return;
        }

        var alternatives = new List<TransferObject> { choices[mars], choices[other] };
        var rules = Universe.Rules;
        bool originalLifeSupport = rules.LifeSupport;
        MissionPlan? originalMission = vehicle.Parts.Mission;
        IOrbiter? originalTarget = vehicle.Target;
        Part? originalTargetPart = vehicle.TargetPart;
        string Choice() => TransferPage.DefaultDestination(vehicle, alternatives).GetKey();
        string fallback = alternatives[1].GetKey();
        try
        {
            rules.LifeSupport = true;
            vehicle.SetTarget(alternatives[1].Body as IOrbiter);
            vehicle.Parts.Mission = null;
            t.Check("without a mission the craft's target remains the default", Choice() == fallback);
            vehicle.Parts.Mission = new MissionPlan { Destination = MissionDestination.Body, BodyId = "Mars" };
            t.Check("an active Mars mission takes priority over the craft's target", Choice() == "Mars");
            rules.LifeSupport = false;
            t.Check("with life support disabled the craft's target is used", Choice() == fallback);
            rules.LifeSupport = true;
            vehicle.Parts.Mission.Custom = true;
            t.Check("a custom mission leaves the existing default in place", Choice() == fallback);
            vehicle.Parts.Mission.Custom = false;
            vehicle.Parts.Mission.BodyId = "AFC-Harness-Unavailable-Mission";
            t.Check("an unavailable mission destination falls back to the craft's target", Choice() == fallback);
            vehicle.SetTarget(null);
            t.Check("without a reachable mission or target the first destination is selected", Choice() == "Mars");
        }
        finally
        {
            vehicle.SetTarget(originalTarget, originalTargetPart);
            vehicle.Parts.Mission = originalMission;
            rules.LifeSupport = originalLifeSupport;
        }
    }

    private static void LogPropagated(TestContext t, OrbitalTransfers.TransferInfo info)
    {
        if (info.PorkChopData == null)
            return;
        for (int m = 0; m < TransferSession.Size; m++)
        {
            for (int n = 0; n < TransferSession.Size; n++)
            {
                if (info.PorkChopData[m, n]?.TransferData is not { } data)
                    continue;
                t.Info($"propagated ({m}, {n}): dV {data.TransferDvVlf.Length():F1} m/s, impacts {data.Impacts}, " +
                       $"bad encounter {data.BadEncounter}, departure infeasible {data.DepartureInfeasible}, " +
                       $"{info.PorkChopData[m, n].FlightPlan.Patches.Count} patches");
            }
        }
        t.Info($"best {TransferSession.BestDv.X}, {TransferSession.BestDv.Y}, preset unavailable {TransferSession.PresetUnavailable}");
    }

    // The scorer marks a cell whose Lambert arc passes below the surface without a trajectory. The
    // cheapest such cell is propagated here, and the stock trajectory check has to find the impact too.
    private static void CheckDepartureImpactScore(TestContext t, OrbitalTransfers.TransferInfo info)
    {
        int2 flagged = new(-1, -1);
        double lowest = double.MaxValue;
        for (int m = 0; m < TransferSession.Size; m++)
        {
            for (int n = 0; n < TransferSession.Size; n++)
            {
                if (TransferSession.ScoreAt(m, n) is { DepartureImpact: true } score && score.DepartureDv < lowest)
                {
                    flagged = new int2(m, n);
                    lowest = score.DepartureDv;
                }
            }
        }
        if (flagged.X < 0)
        {
            t.Info("no scored cell passes below the surface of the departure body");
            return;
        }
        TransferSession.Select(flagged);
        // A selection without the impact starts a refinement that reads the craft, so it has to end before the craft goes.
        DateTime start = DateTime.UtcNow;
        while (TransferSession.Refining && DateTime.UtcNow - start < RefineTimeout)
            Thread.Sleep(50);
        TransferSession.Tick();
        OrbitalTransfers.TransferData? data = info.PorkChopData?[flagged.X, flagged.Y]?.TransferData;
        t.Check("the cheapest cell scored as a departure impact impacts in its propagated trajectory",
            data is { Impacts: true }, $"index {flagged.X}, {flagged.Y}, {lowest:F1} m/s");
    }

    private static void CheckConcurrencyAndValidationRegressions(
        TestContext t, IParentBody home, IOrbiter destination, VehicleSave save)
    {
        CheckBestDvRelTransposeInvariance(t);
        CheckCorrectionValidation(t);
        CheckRefineExceptionAndCancellation(t, home, destination, save);
        CheckNonCelestialOriginGuard(t, home, destination, save);
    }

    private static void CheckBestDvRelTransposeInvariance(TestContext t)
    {
        // Cell (0, 1) has very low arrival speed and huge departure dV, which used to trap coupled running minima.
        // Cell (1, 0) has strictly lower total (dv + rel) and must win regardless of grid orientation.
        var grid = new PorkchopScores.Cell[2, 2];
        var t0 = new UniverseTime(1000.0);
        var dt = new UniverseTime(500.0);
        grid[0, 0] = new PorkchopScores.Cell(t0, dt, 4000.0, 2000.0, false);
        grid[0, 1] = new PorkchopScores.Cell(t0, dt, 9000.0, 50.0, false);
        grid[1, 0] = new PorkchopScores.Cell(t0, dt, 1500.0, 300.0, false);
        grid[1, 1] = new PorkchopScores.Cell(t0, dt, 1200.0, 2500.0, false);

        var transposed = new PorkchopScores.Cell[2, 2];
        for (int r = 0; r < 2; r++)
            for (int c = 0; c < 2; c++)
                transposed[c, r] = grid[r, c];

        var (_, bestRel) = TransferSession.SelectBestCells(grid, null);
        var (_, bestRelT) = TransferSession.SelectBestCells(transposed, null);
        t.Check("BestDvRel minimizes total departure dV plus arrival speed",
            bestRel.X == 1 && bestRel.Y == 0, $"got ({bestRel.X}, {bestRel.Y})");
        t.Check("BestDvRel is invariant under grid transpose",
            bestRelT.X == bestRel.Y && bestRelT.Y == bestRel.X, $"got ({bestRelT.X}, {bestRelT.Y})");
    }

    private static void CheckCorrectionValidation(TestContext t)
    {
        UniverseTime now = Universe.GetElapsedTime();
        var nanData = new OrbitalTransfers.TransferData
        {
            CorrectionStart = now + 600.0,
            CorrectionDvVlf = new double3(double.NaN, 0.0, 0.0),
        };
        var pastData = new OrbitalTransfers.TransferData
        {
            CorrectionStart = now - 10.0,
            CorrectionDvVlf = new double3(10.0, 0.0, 0.0),
        };
        var validData = new OrbitalTransfers.TransferData
        {
            CorrectionStart = now + 600.0,
            CorrectionDvVlf = new double3(10.0, 0.0, 0.0),
        };

        t.Check("IsValidCorrection rejects NaN delta-V", !TransferSession.IsValidCorrection(nanData, now));
        t.Check("IsValidCorrection rejects past correction time", !TransferSession.IsValidCorrection(pastData, now));
        t.Check("IsValidCorrection accepts finite future correction", TransferSession.IsValidCorrection(validData, now));

        TransferSession.CancelAll();
        try
        {
            TransferSession.StartCorrectionForTest(nanData, (_, _) => throw new InvalidOperationException("synthetic correction failure"));
            DateTime waitStart = DateTime.UtcNow;
            while (TransferSession.CorrectionCalculating && DateTime.UtcNow - waitStart < TimeSpan.FromSeconds(5))
                Thread.Sleep(20);
            TransferSession.Tick();
            t.Check("correction task recovers cleanly from worker exceptions",
                !TransferSession.CorrectionCalculating && TransferSession.CorrectionFailed);
        }
        finally
        {
            TransferSession.CancelAll();
        }
    }

    private static void CheckRefineExceptionAndCancellation(
        TestContext t, IParentBody home, IOrbiter destination, VehicleSave save)
    {
        string id = "TransferSessionRefine_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        TransferSession.CancelAll();
        try
        {
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance!, id,
                OrbitFixtures.CircularAt(home, 400_000, Universe.GetElapsedTime()));
            TransferSession.SetSource(vehicle);
            TransferSession.SetDestination(destination);

            var dummy = new OrbitalTransfers.PorkChopEntry(
                new OrbitalTransfers.TransferData(), FlightPlan.CreateUninitialized(vehicle.Hash));
            TransferSession.StartRefineForTest(dummy, (_, _) => throw new InvalidOperationException("synthetic refine failure"));
            DateTime waitStart = DateTime.UtcNow;
            while (TransferSession.Refining && DateTime.UtcNow - waitStart < TimeSpan.FromSeconds(5))
                Thread.Sleep(20);
            t.Check("refine clears Refining even when the worker throws", !TransferSession.Refining);
            TransferSession.Tick();
            t.Check("session remains usable after a refine exception", !TransferSession.Refining && TransferSession.SelectedEntry == null);

            using var started = new ManualResetEventSlim(false);
            using var cancelledSeen = new ManualResetEventSlim(false);
            TransferSession.StartRefineForTest(dummy, (stop, _) =>
            {
                started.Set();
                DateTime deadline = DateTime.UtcNow + TimeSpan.FromSeconds(5);
                while (!stop.IsCancellationRequested && DateTime.UtcNow < deadline)
                    Thread.Sleep(10);
                if (stop.IsCancellationRequested)
                    cancelledSeen.Set();
                return true;
            });
            started.Wait(TimeSpan.FromSeconds(5));
            TransferSession.CancelAll();
            t.Check("CancelAll signals cancellation to an in-flight refine worker",
                cancelledSeen.Wait(TimeSpan.FromSeconds(5)) && !TransferSession.Refining && TransferSession.SelectedEntry == null);
        }
        finally
        {
            TransferSession.CancelAll();
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void CheckNonCelestialOriginGuard(
        TestContext t, IParentBody home, IOrbiter destination, VehicleSave save)
    {
        string id = "TransferSessionOrigin_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        try
        {
            // Look for an OrbitingStar with a child planet in the loaded system; otherwise exercise same-SOI home->destination.
            OrbitingStar? orbitingStar = null;
            IOrbiter? starChild = null;
            foreach (Astronomical body in t.System.All.AsSpan())
            {
                if (body is OrbitingStar star && star.Children.Count > 0)
                {
                    orbitingStar = star;
                    starChild = star.Children[0];
                    break;
                }
            }

            IParentBody spawnParent = orbitingStar ?? home;
            IOrbiter target = starChild ?? destination;
            double altitude = orbitingStar != null ? Math.Max(1_000_000.0, orbitingStar.MeanRadius * 2.0) : 400_000.0;
            vehicle = VehicleFixtures.SpawnDesign(t.System, spawnParent, save.VehicleSaveData.RootPartInstance!, id,
                OrbitFixtures.CircularAt(spawnParent, altitude, Universe.GetElapsedTime()));

            IOrbiter sourceOrbiter = (spawnParent as IOrbiter) ?? vehicle;
            var info = new OrbitalTransfers.TransferInfo(sourceOrbiter, target, vehicle, usePorkChopData: false);
            Exception? error = null;
            try
            {
                TransferSession.PrepareCalculationOrigin(info, Universe.GetElapsedTime());
            }
            catch (Exception ex)
            {
                error = ex;
            }
            t.Check("PrepareCalculationOrigin completes without throwing on same-SOI or OrbitingStar origins",
                error == null, error?.ToString() ?? "ok");
            if (OrbitalTransfers.SameSoiTransfer(new OrbitalTransfers.TransferInfo(sourceOrbiter, target, vehicle, usePorkChopData: false)))
                t.Check("PrepareCalculationOrigin switches same-SOI Source to Vehicle", ReferenceEquals(info.Source, vehicle));
        }
        finally
        {
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void CheckCancelReportsNothing(TestContext t)
    {
        TransferSession.StartCalculation();
        TransferSession.CancelCalculation();
        // The workers stop at their next cell once the token is cancelled.
        Thread.Sleep(500);
        TransferSession.Tick();
        t.Check("a cancelled calculation reports no result", !TransferSession.Calculating && !TransferSession.Calculated);
    }

    private static void CheckRaster(TestContext t, PorkchopRaster? raster, double bestDv)
    {
        if (!t.Check("the dV raster is built", raster != null))
            return;
        t.Check("the dV raster has cells", raster!.Runs.Count > 0, $"{raster.Runs.Count} runs");
        // The raster bins every usable entry, and the best departure also drops entries with a bad encounter, so the best value is not below the raster minimum.
        t.Check("the raster minimum is not above the best departure dV",
            raster.MinValue <= bestDv + 1e-6 && raster.MinValue <= raster.MaxValue,
            $"min {raster.MinValue:F1}, max {raster.MaxValue:F1}, best {bestDv:F1}");
    }

    private static IOrbiter? FindDestination(IParentBody home)
    {
        if (TestWorld.FindMoon(home) is Celestial moon)
            return moon;
        if (home is not IOrbiter homeOrbiter || homeOrbiter.Parent is not IParentBody star)
            return null;
        foreach (IOrbiter child in star.Children)
        {
            if (child is Celestial sibling && !ReferenceEquals(child, home) && sibling.Orbit != null)
                return sibling;
        }
        return null;
    }
}
