using System.Reflection;
using AdvancedFlightComputer.Features.Transfer;
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
// never reports a result, and CancelAll drops everything.
public sealed class TransferSessionTest : AfcTest
{
    private static readonly TimeSpan CalculationTimeout = TimeSpan.FromMinutes(3);
    private static readonly TimeSpan RefineTimeout = TimeSpan.FromMinutes(1);

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

        FieldInfo? stockCalculated = AccessTools.Field(typeof(TransferPlanner), "_transferCalculated");
        object? stockBefore = stockCalculated?.GetValue(null);
        string id = "TransferSession_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        TransferSession.CancelAll();
        try
        {
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance, id,
                OrbitFixtures.CircularAt(home, 400_000, Universe.GetElapsedTime()));
            t.Info($"planning from {((Astronomical)home).Id} orbit to {((Astronomical)destination).Id}");

            TransferSession.SetSource(vehicle);
            TransferSession.SetDestination(destination);
            t.Check("the destination has transfer bounds", TransferSession.Info != null
                && TransferSession.MinTofSec > 0.0 && TransferSession.MaxTofSec > TransferSession.MinTofSec,
                $"min {TransferSession.MinTofSec} s, max {TransferSession.MaxTofSec} s");
            string refusal = TransferSession.CalculateRefusal();
            if (!t.Check("CALCULATE is allowed", refusal.Length == 0, refusal))
                return;

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
            int2 best = TransferSession.BestDv;
            bool bestValid = best.X >= 0 && best.Y >= 0 && info.PorkChopData?[best.X, best.Y]?.TransferData != null;
            if (!t.Check("a best departure is found", bestValid, $"index {best.X}, {best.Y}"))
                return;
            double bestDv = info.PorkChopData![best.X, best.Y].TransferData.TransferDvVlf.Length();
            t.Check("the best departure dV is finite and positive", double.IsFinite(bestDv) && bestDv > 0.0, $"{bestDv:F1} m/s");
            t.Check("the best entry is selected", TransferSession.Selected.X == best.X && TransferSession.Selected.Y == best.Y
                && TransferSession.SelectedEntry != null);

            CheckRaster(t, TransferSession.RasterDv, bestDv);

            // The refinement of the selection runs on the pool and reads the craft, so it has to end before the craft goes.
            DateTime refineStart = DateTime.UtcNow;
            while (TransferSession.Refining && DateTime.UtcNow - refineStart < RefineTimeout)
                Thread.Sleep(50);
            t.Check("the intercept refinement ends", !TransferSession.Refining);

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
