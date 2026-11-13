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

// The crafts and the oracle the two moon tests share.
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

// The planner math of a launch to a moon, each part against the game's own propagation: the injection from a parking orbit, the correction of a flown injection, the capture burn at the periapsis, and the launch plane that holds the moon at arrival.
public sealed class MoonInjectionTest : AfcTest
{
    public override string Name => "afc-moon-injection";

    protected override void Execute(TestContext t)
    {
        if (!MoonTestWorld.Require(t, out IParentBody home, out Celestial moon, out VehicleSave save))
            return;
        IParentBody moonBody = (IParentBody)moon;
        SimDriver driver = t.Session.CreateDriver();
        var spawned = new List<Vehicle>();
        try
        {
            UniverseTime now = Universe.GetElapsedTime();
            Vehicle inPlane = MoonTestWorld.Spawn(t, home, save, "MoonInjectionInPlane",
                MoonTestWorld.CircularInPlane(home, MoonTestWorld.MoonNormal(moon, now), now), spawned);
            Vehicle equatorial = MoonTestWorld.Spawn(t, home, save, "MoonInjectionEquatorial",
                OrbitFixtures.CircularAt(home, MoonTestWorld.ParkingAltitudeM, now), spawned);
            Vehicle flown = MoonTestWorld.Spawn(t, home, save, "MoonInjectionFlown",
                MoonTestWorld.FlownInjection(home, moon, now, out double flownDv), spawned);
            Vehicle atMoon = MoonTestWorld.Spawn(t, moonBody, save, "MoonInjectionAtMoon",
                MoonTestWorld.FlybyAtMoon(moonBody, now, out UniverseTime periapsis), spawned);
            // A site at 45 degrees north, outside the declination range of any moon near the equator or the ecliptic.
            double siteRadius = home.MeanRadius + MoonTestWorld.ParkingAltitudeM;
            Vehicle highSite = MoonTestWorld.Spawn(t, home, save, "MoonInjectionHighSite", Orbit.CreateFromStateCci(home, now,
                new double3(Math.Sqrt(0.5), 0.0, Math.Sqrt(0.5)) * siteRadius, new double3(0.0, Math.Sqrt(home.Mu / siteRadius), 0.0),
                VehicleSpawner.OrbitLineColor), spawned);
            driver.Step(1e-3, 2);

            double hohmannDv = MoonInjectionPlanner.HohmannDv(home.Mu, home.MeanRadius + MoonTestWorld.ParkingAltitudeM, moon.Orbit.SemiMajorAxis);
            CheckPlan(t, "in the moon's plane", inPlane, moon, Universe.GetElapsedTime() + 600.0, hohmannDv);
            CheckRefusedOutOfPlane(t, equatorial, moon);
            CheckCorrection(t, flown, moon, flownDv, hohmannDv);
            CheckCapture(t, atMoon, moonBody, periapsis);
            CheckLaunchPlane(t, "a site inside the declination range", equatorial, home, moon, save, spawned, driver, expectMoonPlane: true, hohmannDv);
            CheckLaunchPlane(t, "a site at 45 deg", highSite, home, moon, save, spawned, driver, expectMoonPlane: false, hohmannDv);
        }
        finally
        {
            MoonTestWorld.Despawn(spawned);
        }
    }

    private static void CheckPlan(TestContext t, string label, Vehicle vehicle, Celestial moon, UniverseTime notBefore, double hohmannDv)
    {
        var watch = System.Diagnostics.Stopwatch.StartNew();
        bool ok = MoonInjectionPlanner.TryPlan(vehicle, moon, MoonTestWorld.FlybyAltitudeM, notBefore,
            out MoonInjectionPlanner.MoonInjection plan, out string refusal);
        watch.Stop();
        if (!t.Check($"{label}: a plan exists", ok, $"refusal '{refusal}'"))
            return;

        IParentBody moonBody = (IParentBody)moon;
        double rp = moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM;
        double peKm = (plan.PredictedPeriapsisRadius - moonBody.MeanRadius) / 1000.0;
        double flightH = (plan.Arrival.Seconds() - plan.BurnTime.Seconds()) / 3600.0;
        double leadS = plan.BurnTime.Seconds() - notBefore.Seconds();
        string facts = $"burn in {leadS:F0} s after the earliest, dv {plan.DvVlf.Length():F1} m/s (impulsive {plan.ImpulsiveDvMs:F1}, Hohmann {hohmannDv:F1}), "
            + $"side {plan.Side}, flight {flightH:F1} h, predicted periapsis {peKm:F1} km, planned in {watch.ElapsedMilliseconds} ms";
        t.Info($"{label}: {facts}");

        t.Check($"{label}: the burn is not before the earliest time and within two parking revolutions",
            leadS >= -1.0 && leadS <= 2.0 * vehicle.Orbit.Period, facts);
        t.Check($"{label}: the delta v is within 10 % of the Hohmann delta v",
            Math.Abs(plan.ImpulsiveDvMs - hohmannDv) <= 0.10 * hohmannDv, facts);
        // The planner corrects its aim to 3 % of the altitude against the same propagation, so 10 % leaves room only for a correction that stopped one step short.
        t.Check($"{label}: the propagated periapsis is above the surface and within 10 % of the requested altitude",
            plan.PredictedPeriapsisRadius > moonBody.GetNearSurfaceRadius()
            && Math.Abs(plan.PredictedPeriapsisRadius - rp) <= 0.10 * MoonTestWorld.FlybyAltitudeM, facts);

        // How far a tenth of a m/s along the burn moves the periapsis, which is why the chain checks the flown orbit and corrects it.
        double nominal = MoonTestWorld.PropagatedPeriapsis(vehicle, moon, plan.BurnTime, plan.DvVlf);
        double3 along = plan.DvVlf / plan.DvVlf.Length();
        double more = MoonTestWorld.PropagatedPeriapsis(vehicle, moon, plan.BurnTime, plan.DvVlf + along * 0.1);
        double less = MoonTestWorld.PropagatedPeriapsis(vehicle, moon, plan.BurnTime, plan.DvVlf - along * 0.1);
        t.Info($"{label}: impulsive propagation gives periapsis {(nominal - moonBody.MeanRadius) / 1000.0:F1} km, "
            + $"with +0.1 m/s {(more - moonBody.MeanRadius) / 1000.0:F1} km, with -0.1 m/s {(less - moonBody.MeanRadius) / 1000.0:F1} km");
    }

    // Unless the moon happens to cross the equator at arrival, an equatorial parking orbit needs a plane change worth several km/s, and the planner must say so instead of offering it.
    private static void CheckRefusedOutOfPlane(TestContext t, Vehicle vehicle, Celestial moon)
    {
        bool ok = MoonInjectionPlanner.TryPlan(vehicle, moon, MoonTestWorld.FlybyAltitudeM, Universe.GetElapsedTime() + 600.0,
            out MoonInjectionPlanner.MoonInjection plan, out string refusal);
        t.Info($"equatorial: {(ok ? $"plan {plan.DvVlf.Length():F1} m/s" : "refused: " + refusal)}");
        double limit = 1.25 * MoonInjectionPlanner.HohmannDv(vehicle.Orbit.Parent.Mu, vehicle.Orbit.SemiMajorAxis, moon.Orbit.SemiMajorAxis);
        t.Check("equatorial: a plan is either within the delta-v limit or refused with the out-of-plane angle",
            ok ? plan.ImpulsiveDvMs <= limit : refusal.Contains("OUT OF THE PARKING PLANE"), ok ? $"{plan.ImpulsiveDvMs:F1} m/s" : refusal);
    }

    // A flown injection on an impact course: one small burn half an hour on must bring the propagated periapsis back to the request.
    private static void CheckCorrection(TestContext t, Vehicle vehicle, Celestial moon, double flownDv, double hohmannDv)
    {
        IParentBody moonBody = (IParentBody)moon;
        double before = MoonInjectionPlanner.PredictedPeriapsisRadius(vehicle, moonBody);
        UniverseTime burnTime = Universe.GetElapsedTime() + 1800.0;
        bool ok = MoonInjectionPlanner.TryPlanCorrection(vehicle, moon, MoonTestWorld.FlybyAltitudeM, burnTime,
            out MoonInjectionPlanner.MoonInjection plan, out string refusal);
        string start = $"flown injection {flownDv:F1} m/s (Hohmann {hohmannDv:F1}), orbit {vehicle.Orbit.Periapsis / 1000.0:F0} x {vehicle.Orbit.Apoapsis / 1000.0:F0} km from the centre, "
            + (double.IsNaN(before) ? "no encounter on the flight plan" : $"periapsis on the flight plan {(before - moonBody.MeanRadius) / 1000.0:F1} km");
        t.Info($"correction: {start}");
        if (!t.Check("correction: a plan exists for a flown injection that misses the periapsis", ok, $"refusal '{refusal}', {start}"))
            return;

        double rp = moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM;
        double independent = MoonTestWorld.PropagatedPeriapsis(vehicle, moon, plan.BurnTime, plan.DvVlf);
        string facts = $"dv {plan.DvVlf.Length():F2} m/s, {(plan.Side is { } side ? "aimed anew on side " + side : "side kept")}, predicted periapsis {(plan.PredictedPeriapsisRadius - moonBody.MeanRadius) / 1000.0:F1} km, "
            + $"impulsive propagation {(independent - moonBody.MeanRadius) / 1000.0:F1} km";
        t.Info($"correction: {facts}");
        t.Check("correction: the burn is at the requested time", Math.Abs(plan.BurnTime.Seconds() - burnTime.Seconds()) < 1.0, facts);
        t.Check("correction: it keeps the side and costs less than 1 % of the injection", plan.Side == null && plan.DvVlf.Length() < 0.01 * hohmannDv, facts);
        t.Check("correction: the predicted periapsis is within 10 % of the requested altitude",
            Math.Abs(plan.PredictedPeriapsisRadius - rp) <= 0.10 * MoonTestWorld.FlybyAltitudeM, facts);
        t.Check("correction: the game's impulsive propagation of the node agrees within 10 % of the altitude",
            Math.Abs(independent - rp) <= 0.10 * MoonTestWorld.FlybyAltitudeM, facts);
    }

    private static void CheckCapture(TestContext t, Vehicle vehicle, IParentBody moonBody, UniverseTime periapsis)
    {
        Orbit orbit = vehicle.Orbit;
        double rp = moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM;
        double speed = orbit.GetStateVectorsAt(periapsis).VelocityCci.Length();
        double forCircle = speed - Math.Sqrt(moonBody.Mu / rp);
        UniverseTime notBefore = Universe.GetElapsedTime() + 600.0;

        bool ok = MoonInjectionPlanner.TryPlanCapture(vehicle, moonBody, 1e9, notBefore, out MoonInjectionPlanner.MoonCapture circle, out string refusal);
        if (t.Check("capture: a plan exists with delta v to spare", ok, $"refusal '{refusal}'"))
        {
            string facts = $"dv {circle.DvVlf.Length():F1} m/s (impulsive circle {forCircle:F1}), predicted orbit "
                + $"{(circle.PeriapsisRadius - moonBody.MeanRadius) / 1000.0:F1} x {(circle.ApoapsisRadius - moonBody.MeanRadius) / 1000.0:F1} km";
            t.Info($"capture: {facts}");
            t.Check("capture: the burn is at the periapsis", Math.Abs(circle.BurnTime.Seconds() - periapsis.Seconds()) < 1.0, facts);
            t.Check("capture: the delta v is within 5 % of the impulsive circularisation", Math.Abs(circle.DvVlf.Length() - forCircle) <= 0.05 * forCircle, facts);
            t.Check("capture: the game's propagation of the burn gives an orbit within 10 km of the periapsis altitude all round",
                circle.Circular && Math.Abs(circle.PeriapsisRadius - rp) <= 10_000.0 && Math.Abs(circle.ApoapsisRadius - rp) <= 10_000.0, facts);
            // The same node as an impulse, as a second oracle that does not use the finite burn.
            StateVectors at = orbit.GetStateVectorsAt(circle.BurnTime);
            Orbit impulsive = OrbitFixtures.ApplyImpulse(orbit, circle.DvVlf.Transform(at.GetVlf2ParentCci().OrIdentity()), circle.BurnTime);
            t.Check("capture: the node as an impulse leaves a closed orbit inside the sphere of influence",
                impulsive.Eccentricity < 1.0 && impulsive.Apoapsis < moonBody.SphereOfInfluence,
                $"ecc {impulsive.Eccentricity:F4}, apoapsis {impulsive.Apoapsis / 1000.0:F0} km from the centre");
        }

        double budget = 0.6 * forCircle;
        ok = MoonInjectionPlanner.TryPlanCapture(vehicle, moonBody, budget, notBefore, out MoonInjectionPlanner.MoonCapture ellipse, out refusal);
        if (t.Check("capture: with 60 % of the circle's delta v the plan is an ellipse", ok && !ellipse.Circular, $"refusal '{refusal}'"))
        {
            string facts = $"dv {ellipse.DvVlf.Length():F1} m/s of {budget:F1}, predicted orbit "
                + $"{(ellipse.PeriapsisRadius - moonBody.MeanRadius) / 1000.0:F1} x {(ellipse.ApoapsisRadius - moonBody.MeanRadius) / 1000.0:F1} km";
            t.Info($"capture: {facts}");
            t.Check("capture: the ellipse stays within the budget and inside half the sphere of influence",
                ellipse.DvVlf.Length() <= budget && ellipse.ApoapsisRadius <= 0.5 * moonBody.SphereOfInfluence
                && ellipse.PeriapsisRadius > moonBody.GetNearSurfaceRadius(), facts);
        }

        ok = MoonInjectionPlanner.TryPlanCapture(vehicle, moonBody, 0.05 * forCircle, notBefore, out _, out refusal);
        t.Check("capture: with 5 % of the circle's delta v it is refused with both numbers", !ok && refusal.Contains("NEEDS") && refusal.Contains("HAS"), refusal);
    }

    // The launch plane from the craft's position as the site. A parking orbit in that plane must reach the moon for about the Hohmann delta v, which the injection planner checks against the game's propagation.
    private static void CheckLaunchPlane(TestContext t, string label, Vehicle site, IParentBody home, Celestial moon, VehicleSave save,
        List<Vehicle> spawned, SimDriver driver, bool expectMoonPlane, double hohmannDv)
    {
        bool ok = GuidanceWindow.TryMoonLaunchPlane(site, moon, 200.0, 200.0, out AscentPlaneTarget target);
        if (!t.Check($"launch plane, {label}: a plane exists", ok && target != null))
            return;
        UniverseTime now = Universe.GetElapsedTime();
        double inc = UpfgTarget.DegToRad(target!.IncDeg), lan = UpfgTarget.DegToRad(target.LanDeg);
        double3 normal = UpfgTarget.OrbitNormal(inc, lan);
        double3 sitePosition = site.Orbit.StateVectors.PositionCci;
        double latDeg = OrbitFixtures.Deg(Math.Asin(sitePosition.Z / sitePosition.Length()));
        double tiltDeg = OrbitFixtures.Deg(Math.Acos(Math.Clamp(double3.Dot(normal, MoonTestWorld.MoonNormal(moon, now)), -1.0, 1.0)));
        bool window = GuidanceWindow.TryLaunchWindow(sitePosition, home.GetAngularVelocity(), inc, lan, out double wait, out _, out _);
        string facts = $"site latitude {latDeg:F2} deg, plane inc {target.IncDeg:F2} deg, LAN {target.LanDeg:F2} deg, {tiltDeg:F2} deg from the moon's plane, "
            + $"window in {wait:F0} s, label '{target.Label}'";
        t.Info($"launch plane, {label}: {facts}");
        t.Check($"launch plane, {label}: the site passes under the plane", window && double.IsFinite(wait), facts);
        if (expectMoonPlane)
            t.Check($"launch plane, {label}: it is the moon's own orbit plane", tiltDeg < 0.01, facts);
        else
            t.Check($"launch plane, {label}: it is inclined just above the site's latitude", Math.Abs(target.IncDeg - (Math.Abs(latDeg) + 0.1)) < 0.01, facts);

        // The soonest arrival the launch planner pairs with this plane: the moon must lie in the plane then, and the injection is one parking orbit after the insertion.
        var inputs = new LunarLaunchInputs(0.0, expectMoonPlane ? PlaneControl.InPlane : PlaneControl.Inclination, target.IncDeg, false, 200.0, 200.0);
        double arrival = double.NaN;
        LunarLaunchPlan? paired = null;
        foreach (bool southbound in new[] { false, true })
        {
            LunarLaunchInputs tried = inputs with { Southbound = southbound };
            double a = LunarLaunchPlanner.SoonestArrival(site, moon, now.Seconds(), tried);
            if (double.IsNaN(a))
                continue;
            LunarLaunchPlan p = LunarLaunchPlanner.Solve(site, moon, now.Seconds(), tried with { ArrivalTime = a });
            if (p.HasPlane && Math.Abs(p.LanDeg - target.LanDeg) < 0.05)
            {
                arrival = a;
                paired = p;
            }
        }
        if (!t.Check($"launch plane, {label}: the launch planner gives this plane for its soonest arrival", paired != null, facts))
            return;
        double3 moonThen = LunarLaunchPlanner.MoonAt(moon, arrival);
        double outOfPlaneDeg = OrbitFixtures.Deg(Math.Asin(Math.Abs(double3.Dot(normal, moonThen / moonThen.Length()))));
        double parkingPeriod = 2.0 * Math.PI * Math.Sqrt(Math.Pow(home.MeanRadius + 200_000.0, 3) / home.Mu);
        string pairedFacts = $"arrival in {(arrival - now.Seconds()) / 3600.0:F1} h, moon {outOfPlaneDeg:F4} deg out of the plane then, "
            + $"parking coast {paired!.CoastSec:F0} s (one orbit {parkingPeriod:F0} s)";
        t.Info($"launch plane, {label}: {pairedFacts}");
        t.Check($"launch plane, {label}: the moon lies in the plane at the arrival", outOfPlaneDeg < 0.01, pairedFacts);
        t.Check($"launch plane, {label}: the parking coast before the injection is at least one orbit", paired.CoastSec >= parkingPeriod - 2.0, pairedFacts);

        // A craft parked in the plane, with the injection searched around the planned time.
        Vehicle parked = MoonTestWorld.Spawn(t, home, save, "MoonInjectionPlane" + (expectMoonPlane ? "In" : "Tilted"),
            MoonTestWorld.CircularInPlane(home, normal, now), spawned);
        driver.Step(1e-3, 2);
        UniverseTime notBefore = new UniverseTime(Math.Max(now.Seconds() + 600.0, paired.InjectionTime - 0.75 * parkingPeriod));
        bool planned = MoonInjectionPlanner.TryPlan(parked, moon, MoonTestWorld.FlybyAltitudeM, notBefore,
            out MoonInjectionPlanner.MoonInjection plan, out string refusal);
        IParentBody moonBody = (IParentBody)moon;
        string injection = planned
            ? $"dv {plan.ImpulsiveDvMs:F1} m/s (Hohmann {hohmannDv:F1}), {(plan.BurnTime.Seconds() - paired.InjectionTime) / 60.0:F1} min from the planned injection, "
              + $"periapsis {(plan.PredictedPeriapsisRadius - moonBody.MeanRadius) / 1000.0:F1} km"
            : $"refusal '{refusal}'";
        t.Info($"launch plane, {label}: injection from the plane: {injection}");
        t.Check($"launch plane, {label}: the injection from a parking orbit in the plane costs within 10 % of the Hohmann delta v and reaches the periapsis",
            planned && Math.Abs(plan.ImpulsiveDvMs - hohmannDv) <= 0.10 * hohmannDv
            && Math.Abs(plan.PredictedPeriapsisRadius - (moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM)) <= 0.10 * MoonTestWorld.FlybyAltitudeM, injection);
    }
}

// The chain after a completed ascent, stepped through GuidanceWindow.ApplyAutopilot with the stage set as the ascent's end or a completed burn leaves it, so the cases need no flight.
// Every node must be loaded with stock Auto armed and no claim held, and a player's burn, a removed node or Auto switched off must stop the chain, keep what the player has and put THEN back to its default.
public sealed class MoonChainTest : AfcTest
{
    public override string Name => "afc-moon-chain";

    private static void SetModActiveRaw(bool active) =>
        typeof(GuidanceWindow).GetField("_modActive", BindingFlags.NonPublic | BindingFlags.Static)!
            .SetValue(null, active);

    protected override void Execute(TestContext t)
    {
        if (!MoonTestWorld.Require(t, out IParentBody home, out Celestial moon, out VehicleSave save))
            return;
        IParentBody moonBody = (IParentBody)moon;
        SimDriver driver = t.Session.CreateDriver();
        Vehicle? previousFocus = Program.ControlledVehicle;
        bool previousModActive = GuidanceWindow.ModActive;
        var spawned = new List<Vehicle>();
        try
        {
            UniverseTime now = Universe.GetElapsedTime();
            Vehicle inPlane = MoonTestWorld.Spawn(t, home, save, "MoonChainInPlane",
                MoonTestWorld.CircularInPlane(home, MoonTestWorld.MoonNormal(moon, now), now), spawned);
            Vehicle withBurn = MoonTestWorld.Spawn(t, home, save, "MoonChainWithBurn",
                MoonTestWorld.CircularInPlane(home, MoonTestWorld.MoonNormal(moon, now), now), spawned);
            Vehicle flown = MoonTestWorld.Spawn(t, home, save, "MoonChainFlown",
                MoonTestWorld.FlownInjection(home, moon, now, out _), spawned);
            Vehicle atMoon = MoonTestWorld.Spawn(t, moonBody, save, "MoonChainAtMoon",
                MoonTestWorld.FlybyAtMoon(moonBody, now, out UniverseTime periapsis), spawned);
            driver.Step(1e-3, 2);

            SetModActiveRaw(true);
            CheckInjectionAndStops(t, inPlane, moon);
            CheckRefusedWithPlannedBurn(t, withBurn, moon, driver);
            CheckCourse(t, flown, moon, capture: false);
            CheckCapture(t, atMoon, moon, periapsis);
        }
        finally
        {
            SetModActiveRaw(previousModActive);
            Program.ControlledVehicle = previousFocus;
            MoonTestWorld.Despawn(spawned);
        }
    }

    // The state the end of an ascent with THEN set leaves (QueueMoonTransfer), with the settle time over.
    private static VehicleAutopilotState Queue(Vehicle vehicle, Celestial moon, MoonChainStage stage, bool capture)
    {
        Program.ControlledVehicle = vehicle;
        RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.MoonTransferId = moon.Id;
        state.MoonTransferCapture = capture;
        state.MoonTransferPeKm = MoonTestWorld.FlybyAltitudeM / 1000.0;
        state.MoonChain = stage;
        state.MoonChainMoonId = moon.Id;
        state.MoonChainPeKm = state.MoonTransferPeKm;
        state.MoonChainCapture = capture;
        state.MoonChainAfter = Universe.GetElapsedSeconds() - 1.0;
        state.MoonChainCorrections = 0;
        state.MoonChainStopRequested = false;
        state.MoonChainNode = null;
        state.MoonChainTarget = null;
        return state;
    }

    private static bool Armed(Vehicle vehicle, VehicleAutopilotState state)
    {
        FlightComputer fc = vehicle.FlightComputer;
        return state.MoonChainNode != null && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), state.MoonChainNode)
            && fc.Burn != null && ReferenceEquals(fc.Burn, state.MoonChainTarget) && fc.BurnMode == FlightComputerBurnMode.Auto
            && VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None;
    }

    private static bool Ended(VehicleAutopilotState state)
        => state.MoonChain == MoonChainStage.Idle && state.MoonTransferId.Length == 0 && !state.MoonTransferCapture
            && state.MoonChainNode == null && state.PlaneTarget == null;

    private static void CheckInjectionAndStops(TestContext t, Vehicle vehicle, Celestial moon)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState state = Queue(vehicle, moon, MoonChainStage.AfterAscent, capture: false);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"injection status: {state.Status}");
        if (!t.Check("injection: the node is the first executable burn, stock Auto is armed on it and guidance holds no claim",
                state.MoonChain == MoonChainStage.Injection && Armed(vehicle, state), $"stage {state.MoonChain}, status '{state.Status}'"))
            return;
        t.Check("injection: THEN stays set while the chain runs", state.MoonTransferId == moon.Id);

        // Auto switched off: the chain allows the moment a staging drops Auto, then stops and leaves the node to the player.
        Burn node = state.MoonChainNode;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("auto off: the chain waits out the grace time", state.MoonChain == MoonChainStage.Injection, state.Status);
        state.MoonChainManualSince = Universe.GetElapsedSeconds() - 60.0;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"auto off status: {state.Status}");
        t.Check("auto off: the chain stops, THEN is back at its default, and the node stays planned in Manual",
            Ended(state) && state.Status.Contains("STOPPED") && fc.BurnPlan.TryGetBurn(node) && fc.BurnMode == FlightComputerBurnMode.Manual, state.Status);

        // A node the player removed.
        state = Queue(vehicle, moon, MoonChainStage.AfterAscent, capture: true);
        GuidanceWindow.ApplyAutopilot(vehicle);
        if (!t.Check("removed node: the chain arms the injection again after a new ascent end",
                state.MoonChain == MoonChainStage.Injection && Armed(vehicle, state), state.Status))
            return;
        fc.RemoveBurn(state.MoonChainNode);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"removed node status: {state.Status}");
        t.Check("removed node: the chain stops, makes no new node, and THEN is back at its default",
            Ended(state) && state.Status.Contains("REMOVED") && fc.BurnPlan.BurnCount == 0, state.Status);

        // Stay in orbit picked on the LAUNCH page while the chain runs.
        state = Queue(vehicle, moon, MoonChainStage.AfterAscent, capture: false);
        GuidanceWindow.ApplyAutopilot(vehicle);
        node = state.MoonChainNode;
        state.MoonChainStopRequested = true;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"stop request status: {state.Status}");
        t.Check("stop request: the chain stops, THEN is back at its default, and the burn that has not started stays planned in Manual",
            Ended(state) && state.Status.Contains("STOPPED") && fc.BurnPlan.TryGetBurn(node) && fc.BurnMode == FlightComputerBurnMode.Manual, state.Status);
        fc.RemoveBurn(node);

        // Guidance switched off while the chain waits for its burn.
        state = Queue(vehicle, moon, MoonChainStage.AfterAscent, capture: false);
        GuidanceWindow.ApplyAutopilot(vehicle);
        node = state.MoonChainNode;
        bool armed = state.MoonChain == MoonChainStage.Injection && Armed(vehicle, state);
        SetModActiveRaw(false);
        GuidanceWindow.ApplyAutopilot(vehicle);
        SetModActiveRaw(true);
        t.Info($"guidance off status: {state.Status}");
        t.Check("guidance off: the chain ends and does not come back, and its burn stays planned in Manual",
            armed && Ended(state) && state.Status.Contains("SWITCHED OFF") && fc.BurnPlan.TryGetBurn(node) && fc.BurnMode == FlightComputerBurnMode.Manual, state.Status);
        fc.RemoveBurn(node);
    }

    private static void CheckRefusedWithPlannedBurn(TestContext t, Vehicle vehicle, Celestial moon, SimDriver driver)
    {
        VehicleAutopilotState state = Queue(vehicle, moon, MoonChainStage.AfterAscent, capture: false);
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.BurnSetup? setup = RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 900.0);
        if (setup == null)
        {
            t.Skip("player burn: no burn could be planned.");
            return;
        }
        state.MoonChainAfter = Universe.GetElapsedSeconds() - 1.0;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"player burn status: {state.Status}");
        t.Check("player burn: a craft with a planned burn gets no injection, keeps its burn in Manual, and THEN is back at its default",
            Ended(state) && fc.BurnPlan.BurnCount == 1 && ReferenceEquals(fc.BurnPlan.FirstBurn, setup.Burn)
            && fc.BurnMode == FlightComputerBurnMode.Manual, state.Status);
        t.Check("player burn: the status says why", state.Status.Contains("ALREADY PLANNED"), state.Status);
    }

    // After the injection burn: the flown orbit misses the periapsis, so the course check must arm one small correction.
    private static void CheckCourse(TestContext t, Vehicle vehicle, Celestial moon, bool capture)
    {
        IParentBody moonBody = (IParentBody)moon;
        VehicleAutopilotState state = Queue(vehicle, moon, MoonChainStage.Course, capture);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"course status: {state.Status}");
        if (!t.Check("course: a flown injection that misses the periapsis gets a correction node with stock Auto armed and no claim",
                state.MoonChain == MoonChainStage.Correction && state.MoonChainCorrections == 1 && Armed(vehicle, state),
                $"stage {state.MoonChain}, status '{state.Status}'"))
            return;
        Burn node = state.MoonChainNode;
        double predicted = MoonTestWorld.PropagatedPeriapsis(vehicle, moon, node.ImpulseTime, node.DeltaVVlf);
        string facts = $"node {node.DeltaVVlf.Length():F2} m/s in {node.ImpulseTime.Seconds() - Universe.GetElapsedSeconds():F0} s, "
            + $"the game's propagation of it gives periapsis {(predicted - moonBody.MeanRadius) / 1000.0:F1} km";
        t.Info($"course: {facts}");
        t.Check("course: the correction is small and, being above 1 m/s, placed at the first place half an hour ahead",
            node.DeltaVVlf.Length() < 30.0 && Math.Abs(node.ImpulseTime.Seconds() - Universe.GetElapsedSeconds() - 1800.0) < 5.0, facts);
        t.Check("course: the game's propagation of the correction node reaches the requested periapsis within 10 %",
            Math.Abs(predicted - (moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM)) <= 0.10 * MoonTestWorld.FlybyAltitudeM, facts);

        // Another burn planned beside the chain's: the player's plan wins.
        FlightComputer fc = vehicle.FlightComputer;
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(node.ImpulseTime + 600.0);
        if (patch == null)
            return;
        UniverseTime later = node.ImpulseTime + 600.0;
        Burn players = Burn.Create(patch.Orbit.GetPointAt(later), later.Seconds(), new double3(1.0, 0.0, 0.0), patch, vehicle);
        fc.AddBurn(players);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"second burn status: {state.Status}");
        t.Check("second burn: the chain stops and both burns stay planned",
            Ended(state) && state.Status.Contains("ANOTHER BURN") && fc.BurnPlan.BurnCount == 2, state.Status);
    }

    // Inside the moon's sphere with the capture asked for: the capture node at the periapsis.
    private static void CheckCapture(TestContext t, Vehicle vehicle, Celestial moon, UniverseTime periapsis)
    {
        IParentBody moonBody = (IParentBody)moon;
        VehicleAutopilotState state = Queue(vehicle, moon, MoonChainStage.Coast, capture: true);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Info($"capture status: {state.Status}");
        if (!t.Check("capture: in the moon's sphere the chain arms the capture node with stock Auto and no claim",
                state.MoonChain == MoonChainStage.Capture && Armed(vehicle, state), $"stage {state.MoonChain}, status '{state.Status}'"))
            return;
        Burn node = state.MoonChainNode;
        t.Check("capture: the node is at the periapsis", Math.Abs(node.ImpulseTime.Seconds() - periapsis.Seconds()) < 1.0,
            $"node at {node.ImpulseTime.Seconds():F1} s, periapsis at {periapsis.Seconds():F1} s");
        StateVectors at = vehicle.Orbit.GetStateVectorsAt(node.ImpulseTime);
        Orbit after = OrbitFixtures.ApplyImpulse(vehicle.Orbit, node.DeltaVVlf.Transform(at.GetVlf2ParentCci().OrIdentity()), node.ImpulseTime);
        t.Check("capture: the node as an impulse leaves a closed orbit inside the sphere of influence",
            after.Eccentricity < 1.0 && after.Apoapsis < moonBody.SphereOfInfluence,
            $"dv {node.DeltaVVlf.Length():F1} m/s, ecc {after.Eccentricity:F4}");
    }
}
