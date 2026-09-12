using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Drives the HyperbolicTargets patches against whatever unbound celestial the loaded
// system provides, which in Sol are the interstellar comets. Every check calls the
// stock entry point with the patches live, so what is asserted is the patched game
// behaviour, not the mod's helpers in isolation.
//
// A UniverseTime built from NaN throws, and an unbound orbit has a NaN Period. Two
// stock paths hit that for a comet, SetTransferInfo for the transfer window and
// PatchedConic.FindClosestApproaches for the encounter search, so two of the checks
// are "the stock call returns instead of throwing".
public sealed class HyperbolicTargetsTest : AfcTest
{
    private const double SpawnAltitudeM = 400_000.0;

    private static readonly string[] SpawnableVehicles = { "Rocket", "Gemini7", "Polaris" };

    public override string Name => "afc-hyperbolic-targets";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (home is not Celestial homeCelestial || homeCelestial.Parent is not StellarBody star)
        {
            t.Skip("home body does not orbit a star directly; heliocentric subcases not applicable.");
            return;
        }

        List<Celestial> unbound = FindUnboundChildren(t.System, star);
        if (unbound.Count == 0)
        {
            t.Skip("no unbound celestial orbits the home star; nothing to target.");
            return;
        }
        Celestial comet = unbound[0];
        t.Info($"unbound target: {comet.Id} e={comet.Orbit.Eccentricity:F4} soi={comet.SphereOfInfluence:E3}");

        VehicleSave? save = FirstAvailableSave();
        if (save == null)
        {
            t.Skip("no shipped default vehicle to spawn.");
            return;
        }

        SimDriver driver = t.Session.CreateDriver();
        var harmony = new Harmony("com.maxi.afc.harnesstests.hyperbolic-targets");
        Vehicle? vehicle = null;
        try
        {
            HyperbolicTargets.ApplyPatches(harmony);
            vehicle = VehicleFixtures.SpawnFromSaveData(
                t.System, home, save.VehicleSaveData, "HyperbolicTargets",
                OrbitFixtures.CircularAt(home, SpawnAltitudeM, Universe.GetElapsedTime()));
            driver.Step(1e-3, 2);

            UniverseTime hohmann = CheckHohmannFlight(t, homeCelestial, star, comet);
            CheckAlignmentTime(t, homeCelestial, comet, vehicle, hohmann);
            CheckPopulateWithPlanets(t, star, unbound, vehicle);
            CheckSetTransferInfo(t, comet, vehicle, hohmann);
            CheckTargetDataSurvivesUnboundTarget(t, homeCelestial, star, comet, vehicle);
            CheckRefineIntercept(t, homeCelestial, comet, vehicle, hohmann);
        }
        finally
        {
            try
            {
                if (vehicle != null)
                    VehicleSpawner.Despawn(vehicle);
            }
            finally
            {
                harmony.UnpatchAll(harmony.Id);
                Patch_SetTransferInfo.Reset();
            }
        }
    }

    private static List<Celestial> FindUnboundChildren(CelestialSystem system, StellarBody star)
    {
        var result = new List<Celestial>();
        ReadOnlySpan<Astronomical> all = system.All.AsSpan();
        for (int i = 0; i < all.Length; i++)
        {
            if (all[i] is Celestial c && c.Orbit != null && !c.Orbit.IsBound()
                && ReferenceEquals(c.Orbit.Parent, star))
                result.Add(c);
        }
        return result;
    }

    // The patched estimate substitutes the periapsis for the unbound end. The oracle
    // is stock's own formula on a bound pair it can compute, where a circular orbit
    // at that periapsis stands in for the comet, and both calls must agree.
    private static UniverseTime CheckHohmannFlight(
        TestContext t, Celestial home, StellarBody star, Celestial comet)
    {
        UniverseTime now = Universe.GetElapsedTime();
        UniverseTime patched = OrbitalTransfers.HohmannFlight(home.Orbit, comet.Orbit);
        Orbit standIn = VehicleSpawner.CircularCci(star, comet.Orbit.Periapsis, now);
        UniverseTime oracle = OrbitalTransfers.HohmannFlight(home.Orbit, standIn);

        bool positive = patched.Seconds() > 0.0 && double.IsFinite(patched.Seconds());
        t.Check("Hohmann estimate for an unbound destination",
            positive && Approx.Rel(patched.Seconds(), oracle.Seconds(), 1e-9),
            $"patched={patched.Seconds():F3}s oracle={oracle.Seconds():F3}s");
        return patched;
    }

    // The departure lands one Hohmann time before the comet's periapsis and never
    // before the requested start. The second call leaves HohmannTimeOfFlight at zero,
    // the way the plan window's alignment block does, and must derive it itself.
    private static void CheckAlignmentTime(
        TestContext t, Celestial home, Celestial comet, Vehicle vehicle, UniverseTime hohmann)
    {
        UniverseTime now = Universe.GetElapsedTime();
        UniverseTime expected = UniverseTime.Max(comet.Orbit.TimeAtPeriapsis - hohmann, now);

        var withEstimate = new OrbitalTransfers.TransferInfo(home, comet, vehicle, usePorkChopData: false)
        {
            HohmannTimeOfFlight = hohmann,
        };
        UniverseTime aligned = OrbitalTransfers.AlignmentTime(withEstimate, now);
        t.Check("alignment time with a stored estimate",
            aligned >= now && Approx.Abs(aligned.Seconds(), expected.Seconds(), 1e-3),
            $"got {aligned.Seconds():F3}s expected {expected.Seconds():F3}s");

        var withoutEstimate = new OrbitalTransfers.TransferInfo(home, comet, vehicle, usePorkChopData: false);
        UniverseTime derived = OrbitalTransfers.AlignmentTime(withoutEstimate, now);
        t.Check("alignment time derives a missing estimate",
            Approx.Abs(derived.Seconds(), expected.Seconds(), 1e-3),
            $"got {derived.Seconds():F3}s expected {expected.Seconds():F3}s");

        // The periapsis model has no phase angle, so an offset does not change the result.
        UniverseTime offset = OrbitalTransfers.AlignmentTime(withEstimate, now, 30.0);
        t.Check("alignment time with a phase offset",
            Approx.Abs(offset.Seconds(), expected.Seconds(), 1e-3),
            $"got {offset.Seconds():F3}s expected {expected.Seconds():F3}s");
    }

    // With the vehicle parked at a planet, the target list must carry every unbound
    // star child that has an SOI, and none without one, each exactly once.
    private static void CheckPopulateWithPlanets(
        TestContext t, StellarBody star, List<Celestial> unbound, Vehicle vehicle)
    {
        TransferObject previousSource = StockPlanner.SourceBody;
        StockPlanner.SourceBody = new TransferObject(vehicle);
        try
        {
            Span<TransferObject> list = stackalloc TransferObject[256];
            int count = 0;
            TransferPlanner.PopulateWithPlanets(list, ref count);

            var listed = new Dictionary<string, int>();
            for (int i = 0; i < count; i++)
            {
                string key = list[i].GetKey();
                listed[key] = listed.TryGetValue(key, out int n) ? n + 1 : 1;
            }

            foreach (Celestial c in unbound)
            {
                bool hasSoi = c.SphereOfInfluence > 0.0 && !double.IsNaN(c.SphereOfInfluence);
                int times = listed.TryGetValue(c.Id, out int n) ? n : 0;
                t.Check($"target list: {c.Id}", times == (hasSoi ? 1 : 0),
                    $"listed {times} time(s), soi={c.SphereOfInfluence:E3}");
            }
        }
        finally
        {
            StockPlanner.SourceBody = previousSource;
        }
    }

    // Stock's SetTransferInfo throws on the NaN Period of an unbound target. The
    // finalizer has to swallow that and leave a window sized from the Hohmann
    // estimate in both the TransferInfo and the two selected time fields.
    private static void CheckSetTransferInfo(
        TestContext t, Celestial comet, Vehicle vehicle, UniverseTime hohmann)
    {
        FieldInfo? destinationField = AccessTools.Field(typeof(TransferPlanner), "_destinationBody");
        MethodInfo? setTransferInfo = GameReflection.TransferPlanner_SetTransferInfo;
        FieldInfo? minField = GameReflection.TransferPlanner_selectedMinTime;
        FieldInfo? maxField = GameReflection.TransferPlanner_selectedMaxTime;
        FieldInfo? unitField = GameReflection.TransferPlanner_selectedTimeUnit;
        FieldInfo? infoField = GameReflection.TransferPlanner_transferInfo;
        if (destinationField == null || setTransferInfo == null || minField == null
            || maxField == null || unitField == null || infoField == null)
        {
            t.Fail("SetTransferInfo", "a TransferPlanner member this check drives is missing");
            return;
        }

        TransferObject previousSource = StockPlanner.SourceBody;
        object? previousDestination = destinationField.GetValue(null);
        object? previousInfo = infoField.GetValue(null);
        object? previousMin = minField.GetValue(null);
        object? previousMax = maxField.GetValue(null);
        object? previousUnit = unitField.GetValue(null);
        try
        {
            StockPlanner.SourceBody = new TransferObject(vehicle);
            destinationField.SetValue(null, new TransferObject(comet));

            Exception? thrown = null;
            try
            {
                setTransferInfo.Invoke(null, null);
            }
            catch (TargetInvocationException ex)
            {
                thrown = ex.InnerException ?? ex;
            }
            if (thrown != null)
            {
                t.Fail("SetTransferInfo on an unbound target", $"threw {thrown.GetType().Name}: {thrown.Message}");
                return;
            }

            OrbitalTransfers.TransferInfo? info = StockPlanner.TransferInfo;
            if (info == null)
            {
                t.Fail("SetTransferInfo on an unbound target", "TransferInfo is null afterwards");
                return;
            }

            double expectedMin = hohmann.Seconds() * HyperbolicTargets.MinTofRatio;
            double expectedMax = hohmann.Seconds() * HyperbolicTargets.MaxTofRatio;
            bool targetOk = ReferenceEquals(info.Target, comet);
            bool hohmannOk = Approx.Rel(info.HohmannTimeOfFlight.Seconds(), hohmann.Seconds(), 1e-9);
            bool minOk = Approx.Rel(info.MinTransferTimeOfFlight.Seconds(), expectedMin, 1e-9);
            bool maxOk = Approx.Rel(info.MaxTransferTimeOfFlight.Seconds(), expectedMax, 1e-9);
            var selectedMin = (UniverseTime)minField.GetValue(null)!;
            var selectedMax = (UniverseTime)maxField.GetValue(null)!;
            bool fieldsOk = selectedMin == info.MinTransferTimeOfFlight && selectedMax == info.MaxTransferTimeOfFlight;
            t.Check("SetTransferInfo on an unbound target", targetOk && hohmannOk && minOk && maxOk && fieldsOk,
                $"hohmann={info.HohmannTimeOfFlight.Seconds():F1}s min={info.MinTransferTimeOfFlight.Seconds():F1}s " +
                $"max={info.MaxTransferTimeOfFlight.Seconds():F1}s selectedMin={selectedMin.Seconds():F1}s " +
                $"selectedMax={selectedMax.Seconds():F1}s targetOk={targetOk}");
        }
        finally
        {
            StockPlanner.SourceBody = previousSource;
            destinationField.SetValue(null, previousDestination);
            infoField.SetValue(null, previousInfo);
            minField.SetValue(null, previousMin);
            maxField.SetValue(null, previousMax);
            unitField.SetValue(null, previousUnit);
        }
    }

    // A heliocentric patch asked for target nodes against the comet reaches stock's
    // closest approach search. With a finite patch end the search sizes its window
    // from the comet's NaN Period, which the guard skips, and without the guard the
    // UniverseTime constructor throws. An end of time patch takes stock's other
    // branch, which the guard leaves alone, and must survive on its own.
    private static void CheckTargetDataSurvivesUnboundTarget(
        TestContext t, Celestial home, StellarBody star, Celestial comet, Vehicle vehicle)
    {
        UniverseTime now = Universe.GetElapsedTime();
        double r = home.Orbit.SemiMajorAxis;
        Orbit heliocentric = VehicleSpawner.EllipticalCci(star, 0.9 * r, 1.5 * r, now);

        (string Label, UniverseTime End)[] cases =
        {
            ("finite patch end", now + 400.0 * 86400.0),
            ("end-of-time patch", UniverseTime.EndOfTime),
        };
        foreach ((string label, UniverseTime end) in cases)
        {
            var patch = new PatchedConic(
                now, end, PatchTransition.Burn, PatchTransition.Final, heliocentric, vehicle.Hash);
            try
            {
                UniverseTime expiry = patch.CalculateSetTargetData(comet, 16, computeClosestApproaches: true);
                t.Check($"target data against an unbound target, {label}", patch.TargetData != null,
                    $"expiry={(expiry.IsEndOfTime() ? "end of time" : expiry.Seconds().ToString("F0"))}");
            }
            catch (Exception ex) when (!IsGameApiDrift(ex))
            {
                t.Fail($"target data against an unbound target, {label}",
                    $"threw {ex.GetType().Name}: {ex.Message}");
            }
        }
    }

    // Let game API drift escape because catching it would hide an infrastructure failure.
    private static bool IsGameApiDrift(Exception e)
    {
        for (Exception? cur = e; cur != null; cur = cur.InnerException)
        {
            if (cur is MissingMemberException or TypeLoadException)
                return true;
        }
        return false;
    }

    // The refine step for an unbound target builds the plan from the Lambert dV and
    // reports the sampled closest approach.
    private static void CheckRefineIntercept(
        TestContext t, Celestial home, Celestial comet, Vehicle vehicle, UniverseTime hohmann)
    {
        UniverseTime now = Universe.GetElapsedTime();
        var info = new OrbitalTransfers.TransferInfo(home, comet, vehicle, usePorkChopData: false)
        {
            HohmannTimeOfFlight = hohmann,
        };
        var transferData = new OrbitalTransfers.TransferData
        {
            Start = now + 3600.0,
            Transit = hohmann,
        };
        if (!OrbitalTransfers.SolveLambert(info, ref transferData))
        {
            t.Skip("stock found no feasible departure for the comet transfer; refine subcase not applicable.");
            return;
        }

        var entry = new OrbitalTransfers.PorkChopEntry(transferData, FlightPlan.CreateUninitialized(vehicle.Hash));
        // The constructor queues a worker with no cancellation or join API.
        // TryFindIntercept reads only its arguments, so bypass the constructor and call it here to keep the vehicle and patches alive until the solve ends.
        var task = (RefineBurnTask)RuntimeHelpers.GetUninitializedObject(typeof(RefineBurnTask));
        bool success = task.TryFindIntercept(info, ref entry);
        OrbitalTransfers.PorkChopEntry refined = entry;
        double closest = refined.TransferData.ClosestApproachDistance;
        bool hasPlan = refined.FlightPlan != null && refined.FlightPlan.Patches.Count > 0;
        bool closestOk = double.IsFinite(closest) && closest < double.MaxValue;
        t.Check("refine intercept", success && hasPlan && closestOk,
            $"success={success} patches={(hasPlan ? refined.FlightPlan!.Patches.Count : 0)} " +
            $"closest={closest / 1000.0:F0}km dv={transferData.TransferDvVlf.Length():F1}m/s");
    }

    private static VehicleSave? FirstAvailableSave()
    {
        foreach (string id in SpawnableVehicles)
            if (DefaultVehicleSaves.FindSave(id) is VehicleSave save)
                return save;
        return null;
    }
}
