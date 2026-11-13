using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Features.MissionPlanner;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The crafts and the oracle of the tests on the way to a moon.
internal static class MoonTestWorld
{
    public const double ParkingAltitudeM = 200_000.0;
    public const double FlybyAltitudeM = 100_000.0;

    private static readonly string[] SpawnableVehicles = { "Rocket", "Gemini7", "Polaris" };

    public static bool Require(TestContext t, out IParentBody home, out Celestial moon, out VehicleSave save)
    {
        moon = null!;
        save = null!;
        if (!TestWorld.RequireHome(t, out home))
            return false;
        Celestial? found = TestWorld.FindMoon(home);
        if (found == null)
        {
            t.Skip("no moon under home body.");
            return false;
        }
        moon = found;
        foreach (string id in SpawnableVehicles)
            if (DefaultVehicleSaves.FindSave(id) is VehicleSave s)
            {
                save = s;
                return true;
            }
        t.Skip("no shipped default vehicle to spawn.");
        return false;
    }

    public static Vehicle Spawn(TestContext t, IParentBody parent, VehicleSave save, string name, Orbit orbit, List<Vehicle> spawned)
    {
        Vehicle v = VehicleFixtures.SpawnFromSaveData(t.System, parent, save.VehicleSaveData, name, orbit);
        spawned.Add(v);
        return v;
    }

    public static void Despawn(List<Vehicle> spawned)
    {
        if (Universe.IsAutoWarpActive)
            Universe.AutoWarpStop(true);
        foreach (Vehicle v in spawned)
        {
            RcsFlightSupport.CleanupBurns(v.FlightComputer);
            v.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            VehicleAutopilotState.Remove(v);
            VehicleSpawner.Despawn(v);
        }
    }

    public static double3 MoonNormal(Celestial moon, UniverseTime at)
    {
        StateVectors m = moon.Orbit.GetStateVectorsAt(at);
        return double3.Normalize(double3.Cross(m.PositionCci, m.VelocityCci));
    }

    // A circular parking orbit in the plane with normal n, the craft on the plane's ascending side of the equator.
    public static Orbit CircularInPlane(IParentBody home, double3 n, UniverseTime now)
    {
        double3 x = double3.Normalize(double3.Cross(n, double3.UnitZ));
        double radius = home.MeanRadius + ParkingAltitudeM;
        double3 v = double3.Normalize(double3.Cross(n, x)) * Math.Sqrt(home.Mu / radius);
        return Orbit.CreateFromStateCci(home, now, x * radius, v, VehicleSpawner.OrbitLineColor);
    }

    // The orbit of an injection that was flown half an hour ago from a parking orbit in the moon's plane and aimed at the moon's centre: the cheapest Lambert departure of the last parking revolution.
    // It is on an impact course, as a flown injection that missed its periapsis is.
    public static Orbit FlownInjection(IParentBody home, Celestial moon, UniverseTime now, out double dvMs)
    {
        Orbit parking = CircularInPlane(home, MoonNormal(moon, now), now);
        double hohmann = LunarTransferGeometry.HohmannTime(home.Mu, parking.SemiMajorAxis, moon.Orbit.SemiMajorAxis);
        double latest = now.Seconds() - 1800.0;
        dvMs = double.MaxValue;
        double bestStart = latest, bestFlight = hohmann;
        double3 bestVelocity = default;
        for (int i = 0; i <= 720; i++)
            for (int j = 0; j <= 24; j++)
            {
                double start = latest - i * parking.Period / 720.0;
                double flight = (0.6 + 0.4 * j / 24.0) * hohmann;
                StateVectors from = parking.GetStateVectorsAt(new UniverseTime(start));
                double3 to = moon.Orbit.GetStateVectorsAt(new UniverseTime(start + flight)).PositionCci;
                OrbitalTransfers.SuperiorLambert(home.Mu, from.PositionCci, to, new UniverseTime(flight), out double3 departure, out _);
                double cost = (departure - from.VelocityCci).Length();
                if (double.IsFinite(cost) && cost < dvMs)
                {
                    dvMs = cost;
                    bestStart = start;
                    bestFlight = flight;
                    bestVelocity = departure;
                }
            }
        StateVectors at = parking.GetStateVectorsAt(new UniverseTime(bestStart));
        return Orbit.CreateFromStateCci(home, new UniverseTime(bestStart), at.PositionCci, bestVelocity, VehicleSpawner.OrbitLineColor);
    }

    // A flyby orbit at the moon whose periapsis is two hours ahead, with the excess speed of a transfer from a low parking orbit.
    public static Orbit FlybyAtMoon(IParentBody moonBody, UniverseTime now, out UniverseTime periapsis)
    {
        periapsis = now + 7200.0;
        double rp = moonBody.MeanRadius + FlybyAltitudeM;
        double speed = Math.Sqrt(850.0 * 850.0 + 2.0 * moonBody.Mu / rp);
        return Orbit.CreateFromStateCci(moonBody, periapsis, new double3(rp, 0.0, 0.0), new double3(0.0, speed, 0.0),
            VehicleSpawner.OrbitLineColor);
    }

    // The game's own patched-conic propagation of an impulsive burn on the craft's flight plan, as FlightPlan.CalculateBurnPatch and ComputeCompleteTrajectory build it. Returns the periapsis radius at the moon, or NaN without an encounter.
    public static double PropagatedPeriapsis(Vehicle vehicle, Celestial moon, UniverseTime burnTime, double3 dvVlf)
    {
        PatchedConic? prePatch = vehicle.FlightPlan.TryFindPatch(burnTime);
        if (prePatch == null)
            return double.NaN;
        FlightPlan plan = FlightPlan.CreateUninitialized(vehicle.Hash);
        plan.ImpactClearanceMargin = vehicle.BoundingSphereRadiusBody;
        plan.Patches.Add(plan.CalculateBurnPatch(prePatch, prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime), dvVlf, burnTime));
        plan.ComputeCompleteTrajectory(out bool errors, 8, 8, moon, resolveImpactsCompletely: true);
        return errors ? double.NaN : FlybyPrediction.FromPlan(plan, (IParentBody)moon).PeriapsisRadius ?? double.NaN;
    }
}
