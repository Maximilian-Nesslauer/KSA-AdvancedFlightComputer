using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class FlybyPreviewTest : AfcTest
{
    public override string Name => "afc-flyby-preview";
    private static int _solveCount;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home)) return;
        Celestial? moon = TestWorld.FindMoon(home);
        VehicleSave? save = DefaultVehicleSaves.FindSave("Rocket")
            ?? DefaultVehicleSaves.FindSave("Gemini7") ?? DefaultVehicleSaves.FindSave("Polaris");
        if (moon == null || save == null)
        {
            t.Skip("a home moon and a default vehicle save are required.");
            return;
        }

        Vehicle vehicle = VehicleFixtures.SpawnFromSaveData(
            t.System, home, save.VehicleSaveData, "FlybyPreview",
            OrbitFixtures.CircularAt(home, 400_000, Universe.GetElapsedTime()));
        var harmony = new Harmony("afc.tests.flyby-preview");
        bool wasEnabled = HohmannFlybyUI.Enabled;
        try
        {
            t.Session.CreateDriver().Step(1e-3, 2);
            harmony.Patch(AccessTools.Method(typeof(FlybyTargeting), nameof(FlybyTargeting.ComputeFlybyDeparture)),
                prefix: new HarmonyMethod(typeof(FlybyPreviewTest), nameof(CountSolve)));
            CheckConsumerReuse(t, vehicle, moon);
            CheckFreezeAndReset(t, vehicle, moon);
            CheckMissingPatch(t, vehicle, moon);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            HohmannFlybyUI.Reset();
            HohmannFlybyUI.Enabled = wasEnabled;
            HohmannMultiPassUI.Reset();
            VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void CountSolve() => _solveCount++;

    private static void CheckConsumerReuse(TestContext t, Vehicle vehicle, Celestial moon)
    {
        HohmannFlybyUI.Reset();
        HohmannFlybyUI.Enabled = true;
        AccessTools.Field(typeof(HohmannFlybyUI), "_flybyOn").SetValue(null, true);
        UniverseTime start = Universe.GetElapsedTime() + 60;
        UniverseTime transit = OrbitalTransfers.HohmannFlight(vehicle.Orbit, moon.Orbit);
        var info = new OrbitalTransfers.TransferInfo(vehicle, moon, vehicle, false);
        var entry = new OrbitalTransfers.PorkChopEntry(
            new OrbitalTransfers.TransferData { Start = start, Transit = transit },
            FlightPlan.CreateUninitialized(vehicle.Hash));
        _solveCount = 0;
        HohmannFlybyUI.ShowSingleDeparture(vehicle, entry, info);
        var center = new HohmannMultiPassPlanner.HohmannPlanInput(
            moon, start, double3.Zero, false, 0, moon.Orbit.SemiMajorAxis);
        var preview = HohmannMultiPassUI.MaybeApplyFlyby(vehicle, info, center, transit, out bool failed);
        var commit = HohmannMultiPassUI.MaybeApplyFlyby(vehicle, info, center, transit, out bool commitFailed);
        t.Check("single readout, multi-pass preview and commit share one solve", _solveCount == 1,
            $"actual solves={_solveCount}");
        t.Check("preview and commit use the same departure", preview == commit && failed == commitFailed);

        entry.TransferData.Start += 120;
        center = center with { TFinal = entry.TransferData.Start };
        var shifted = HohmannMultiPassUI.MaybeApplyFlyby(vehicle, info, center, transit, out _);
        HohmannFlybyUI.ShowSingleDeparture(vehicle, entry, info);
        t.Check("changed departure computes once for both consumers", _solveCount == 2,
            $"actual solves={_solveCount}");
        var key = FlybyTargeting.CaptureDepartureKey(
            vehicle, moon, center.TFinal, transit, ((IParentBody)moon).MeanRadius + 100_000, FlybySide.Inner);
        var solution = FlybyTargeting.GetDeparture(key);
        HohmannFlybyUI.ShowMultiPassDeparture(solution, null, (IParentBody)moon, entry);
        var displayed = AccessTools.Field(typeof(HohmannFlybyUI), "_displayedDeparture").GetValue(null);
        t.Check("shifted readout uses the selected solution", ReferenceEquals(displayed, solution));
        if (solution.Outcome.Result is { } result)
            t.Check("shifted plan uses the shared burn", shifted.TFinal == result.BurnTime
                && shifted.DFinalVlf == result.DvVlf);
        else
            t.Skip("shifted geometry has no usable flyby departure.");
    }

    private static void CheckFreezeAndReset(TestContext t, Vehicle vehicle, Celestial moon)
    {
        UniverseTime start = Universe.GetElapsedTime() + 60;
        UniverseTime transit = OrbitalTransfers.HohmannFlight(vehicle.Orbit, moon.Orbit);
        var key = FlybyTargeting.CaptureDepartureKey(
            vehicle, moon, start, transit, ((IParentBody)moon).MeanRadius + 100_000, FlybySide.Inner);
        FlybyTargeting.ResetDepartureCache();
        _solveCount = 0;
        var first = FlybyTargeting.GetDeparture(key);
        FlightComputerBurnMode previous = vehicle.FlightComputer.BurnMode;
        try
        {
            vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Auto;
            var drift = key with { Parking = key.Parking with { X = key.Parking.X + 10 } };
            t.Check("parking drift freezes under thrust", ReferenceEquals(first, FlybyTargeting.GetDeparture(drift)));
            var changed = FlybyTargeting.GetDeparture(drift with { Radius = key.Radius + 100 });
            t.Check("radius change bypasses thrust freeze", !ReferenceEquals(first, changed) && _solveCount == 2);
            FlybyTargeting.GetDeparture(changed.Key with { Side = FlybySide.Outer });
            t.Check("side change bypasses thrust freeze", _solveCount == 3);
            FlybyTargeting.GetDeparture(changed.Key with { Start = start + 1 });
            t.Check("time change bypasses thrust freeze", _solveCount == 4);
        }
        finally
        {
            vehicle.FlightComputer.BurnMode = previous;
        }
        HohmannFlybyUI.Reset();
        FlybyTargeting.GetDeparture(key);
        t.Check("UI reset releases the shared solve", _solveCount == 5);
    }

    private static void CheckMissingPatch(TestContext t, Vehicle vehicle, Celestial moon)
    {
        // FlightPlan.TryFindPatch includes both bounds. One second before the first start is outside every patch.
        UniverseTime before = vehicle.FlightPlan.Patches.Min(p => p.StartTime) - 1;
        t.Check("stock has no patch at the test departure", vehicle.FlightPlan.TryFindPatch(before) == null);
        var departure = new FlybyTargeting.FlybyResult(
            new double3(1, 0, 0), new double3(1, 0, 0), before, 1, 1, 1, false, 0, 1);
        FlybyPrediction prediction = HohmannFlybyUI.BuildPreview(
            vehicle, moon, (IParentBody)moon, departure, out FlightPlan? preview);
        t.Check("missing patch has an explicit reason", prediction.Status == FlybyPredictionStatus.NoCoveringPatch
            && prediction.PeriapsisRadius == null && preview == null && prediction.Reason.Length > 0);
        var emptyPlan = FlightPlan.CreateUninitialized(vehicle.Hash);
        prediction = FlybyPrediction.FromPlan(emptyPlan, (IParentBody)moon);
        t.Check("missing encounter has no numeric periapsis", prediction.Status == FlybyPredictionStatus.NoEncounter
            && prediction.PeriapsisRadius == null);
    }
}
