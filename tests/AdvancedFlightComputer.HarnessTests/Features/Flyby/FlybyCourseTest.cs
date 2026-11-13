using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The follow-up of a flown flyby departure, stepped by hand: the completion of the departure, the course check on the craft's own flight plan, the correction node from the shared corrector, and every way the player ends it.
// The crafts sit on the orbits a flown injection leaves, so no burn is flown here. The oracle of a correction node is the game's own propagation of it.
public sealed class FlybyCourseTest : AfcTest
{
    public override string Name => "afc-flyby-course";

    private static readonly FieldInfo CheckAfter = typeof(FlybyCourse).GetField("_checkAfter", BindingFlags.NonPublic | BindingFlags.Static)!;
    private static readonly FieldInfo StepsInCheck = typeof(FlybyCourse).GetField("_stepsInCheck", BindingFlags.NonPublic | BindingFlags.Static)!;

    protected override void Execute(TestContext t)
    {
        if (!MoonTestWorld.Require(t, out IParentBody home, out Celestial moon, out VehicleSave save))
            return;
        IParentBody moonBody = (IParentBody)moon;
        double rp = moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM;
        SimDriver driver = t.Session.CreateDriver();
        var spawned = new List<Vehicle>();
        try
        {
            UniverseTime now = Universe.GetElapsedTime();
            Orbit flownOrbit = MoonTestWorld.FlownInjection(home, moon, now, out _);
            Vehicle flown = MoonTestWorld.Spawn(t, home, save, "FlybyCourseFlown", flownOrbit, spawned);
            Vehicle withBurn = MoonTestWorld.Spawn(t, home, save, "FlybyCourseWithBurn", MoonTestWorld.FlownInjection(home, moon, now, out _), spawned);
            Vehicle atMoon = MoonTestWorld.Spawn(t, moonBody, save, "FlybyCourseAtMoon", MoonTestWorld.FlybyAtMoon(moonBody, now, out _), spawned);
            driver.Step(1e-3, 2);

            CheckTolerance(t, moonBody);
            Burn? correction = CheckCorrections(t, flown, moon, driver);
            if (correction != null)
            {
                // A craft on the orbit the first correction leaves, as an impulse.
                StateVectors at = flownOrbit.GetStateVectorsAt(correction.ImpulseTime);
                Orbit corrected = OrbitFixtures.ApplyImpulse(flownOrbit, correction.DeltaVVlf.Transform(at.GetVlf2ParentCci().OrIdentity()), correction.ImpulseTime);
                Vehicle onCourse = MoonTestWorld.Spawn(t, home, save, "FlybyCourseOnCourse", corrected, spawned);
                driver.Step(1e-3, 2);
                CheckOnCourse(t, onCourse, moon, driver, rp);
            }
            CheckPlayerStops(t, withBurn, moon, driver, rp);
            CheckNotFollowed(t, atMoon, withBurn, moon, rp);
            CheckSettleAndHolder(t, withBurn, moon, driver, rp);
            CheckQueuedNode(t, withBurn, moon, driver, rp);
        }
        finally
        {
            FlybyCourse.Reset();
            MoonTestWorld.Despawn(spawned);
        }
    }

    private static void CheckTolerance(TestContext t, IParentBody moonBody)
    {
        double r = moonBody.MeanRadius;
        t.Check("tolerance: 100 km asked, 109 km and 91 km are on course, 111 km, 89 km and no encounter are not",
            CourseCorrector.OnCourse(r + 109_000.0, moonBody, 100_000.0) && CourseCorrector.OnCourse(r + 91_000.0, moonBody, 100_000.0)
            && !CourseCorrector.OnCourse(r + 111_000.0, moonBody, 100_000.0) && !CourseCorrector.OnCourse(r + 89_000.0, moonBody, 100_000.0)
            && !CourseCorrector.OnCourse(double.NaN, moonBody, 100_000.0));
        t.Check("tolerance: 5000 km asked, 10 % either way is on course",
            CourseCorrector.OnCourse(r + 5_450_000.0, moonBody, 5_000_000.0) && !CourseCorrector.OnCourse(r + 5_550_000.0, moonBody, 5_000_000.0));
    }

    // A stand-in for the departure: a small node half an hour ahead, which the test then reports as flown.
    private static Burn? Departure(Vehicle vehicle, SimDriver driver)
        => RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 0.01, 1800.0)?.Burn;

    // The completion as SharedVehicleHooks.OnStockBurnEnded passes it on, with the settle time and the settle steps over, and one step of the follow-up.
    private static void CompleteAndStep(Vehicle vehicle, Burn node)
    {
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        FlybyCourse.OnBurnCompleted(vehicle, node);
        CheckAfter.SetValue(null, Universe.GetElapsedSeconds() - 1.0);
        StepsInCheck.SetValue(null, CourseCorrector.SettleSteps);
        FlybyCourse.TickVehicle(vehicle);
    }

    // A flown departure on an impact course gets a correction node in Manual. With the orbit unchanged after it, a second one follows, and after that the follow-up ends and says the course is off.
    private static Burn? CheckCorrections(TestContext t, Vehicle vehicle, Celestial moon, SimDriver driver)
    {
        IParentBody moonBody = (IParentBody)moon;
        double rp = moonBody.MeanRadius + MoonTestWorld.FlybyAltitudeM;
        FlightComputer fc = vehicle.FlightComputer;
        Burn? departure = Departure(vehicle, driver);
        if (departure == null)
        {
            t.Skip("no departure node could be planned.");
            return null;
        }
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, departure);
        t.Check("departure: the follow-up watches the created node", FlybyCourse.Stage == FlybyCourseStage.Departure && FlybyCourse.Follows(vehicle));
        FlybyCourse.TickVehicle(vehicle);
        t.Check("departure: a step before the burn is flown changes nothing",
            FlybyCourse.Stage == FlybyCourseStage.Departure && fc.BurnPlan.BurnCount == 1 && ReferenceEquals(fc.BurnPlan.FirstBurn, departure));

        double before = MoonInjectionPlanner.PredictedPeriapsisRadius(vehicle, moonBody);
        t.Info($"flown departure: periapsis on the flight plan {(before - moonBody.MeanRadius) / 1000.0:F1} km for {MoonTestWorld.FlybyAltitudeM / 1000.0:F0} km asked");
        CompleteAndStep(vehicle, departure);
        Burn? first = fc.BurnPlan.FindFirstExecutableBurn();
        t.Info($"first check: stage {FlybyCourse.Stage}, says '{FlybyCourse.LastResult}'");
        if (!t.Check("first check: the flown node is out of the plan and one correction node is loaded in Manual with no claim",
                FlybyCourse.Stage == FlybyCourseStage.Correction && FlybyCourse.Corrections == 1 && first != null && !ReferenceEquals(first, departure)
                && fc.BurnPlan.BurnCount == 1 && fc.Burn != null && fc.BurnMode == FlightComputerBurnMode.Manual
                && VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None,
                $"stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s), mode {fc.BurnMode}, says '{FlybyCourse.LastResult}'"))
            return null;
        double predicted = MoonTestWorld.PropagatedPeriapsis(vehicle, moon, first!.ImpulseTime, first.DeltaVVlf);
        string facts = $"node {first.DeltaVVlf.Length():F2} m/s in {first.ImpulseTime.Seconds() - Universe.GetElapsedSeconds():F0} s, "
            + $"the game's propagation of it gives periapsis {(predicted - moonBody.MeanRadius) / 1000.0:F1} km";
        t.Info($"first check: {facts}");
        t.Check("first check: the correction is small and the game's propagation of it reaches the asked periapsis within 10 %",
            first.DeltaVVlf.Length() < 30.0 && CourseCorrector.OnCourse(predicted, moonBody, MoonTestWorld.FlybyAltitudeM), facts);
        t.Check("first check: the result names the flown periapsis and the correction", FlybyCourse.LastResult.Contains("ASKED") && FlybyCourse.LastResult.Contains("CORRECTION"), FlybyCourse.LastResult);

        CompleteAndStep(vehicle, first);
        Burn? second = fc.BurnPlan.FindFirstExecutableBurn();
        t.Check("second check: a correction that left the course off gets a second node",
            FlybyCourse.Stage == FlybyCourseStage.Correction && FlybyCourse.Corrections == 2 && second != null && !ReferenceEquals(second, first) && fc.BurnPlan.BurnCount == 1,
            $"stage {FlybyCourse.Stage}, corrections {FlybyCourse.Corrections}, {fc.BurnPlan.BurnCount} burn(s)");
        if (second == null)
            return first;

        CompleteAndStep(vehicle, second);
        t.Info($"third check: says '{FlybyCourse.LastResult}'");
        t.Check("third check: after two corrections the follow-up ends, plans nothing more and says the course is off",
            FlybyCourse.Stage == FlybyCourseStage.Idle && fc.BurnPlan.BurnCount == 0 && FlybyCourse.LastResult.Contains("OFF COURSE AFTER 2 CORRECTIONS"),
            $"stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s), says '{FlybyCourse.LastResult}'");
        return first;
    }

    private static void CheckOnCourse(TestContext t, Vehicle vehicle, Celestial moon, SimDriver driver, double rp)
    {
        IParentBody moonBody = (IParentBody)moon;
        FlightComputer fc = vehicle.FlightComputer;
        Burn? departure = Departure(vehicle, driver);
        if (departure == null)
        {
            t.Skip("on course: no departure node could be planned.");
            return;
        }
        double predicted = MoonInjectionPlanner.PredictedPeriapsisRadius(vehicle, moonBody);
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, departure);
        CompleteAndStep(vehicle, departure);
        t.Info($"on course: periapsis on the flight plan {(predicted - moonBody.MeanRadius) / 1000.0:F1} km, says '{FlybyCourse.LastResult}'");
        t.Check("on course: a flown orbit inside the tolerance ends the follow-up with no node",
            FlybyCourse.Stage == FlybyCourseStage.Idle && fc.BurnPlan.BurnCount == 0 && FlybyCourse.LastResult.StartsWith("FLYBY ON COURSE", StringComparison.Ordinal),
            $"stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s), says '{FlybyCourse.LastResult}'");
    }

    private static void CheckPlayerStops(TestContext t, Vehicle vehicle, Celestial moon, SimDriver driver, double rp)
    {
        FlightComputer fc = vehicle.FlightComputer;

        // The departure node removed before it is flown.
        Burn? departure = Departure(vehicle, driver);
        if (departure == null)
        {
            t.Skip("stops: no departure node could be planned.");
            return;
        }
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, departure);
        FlybyCourse.TickVehicle(vehicle);
        fc.RemoveBurn(departure);
        FlybyCourse.TickVehicle(vehicle);
        t.Check("removed node: the follow-up ends and plans nothing", FlybyCourse.Stage == FlybyCourseStage.Idle && fc.BurnPlan.BurnCount == 0);
        FlybyCourse.OnBurnCompleted(vehicle, departure);
        t.Check("removed node: a late completion of it does not start a check", FlybyCourse.Stage == FlybyCourseStage.Idle);

        // A burn of the player's beside the flown departure at the check.
        departure = Departure(vehicle, driver);
        if (departure == null)
            return;
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, departure);
        UniverseTime later = departure.ImpulseTime + 600.0;
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(later);
        if (patch == null)
            return;
        Burn players = Burn.Create(patch.Orbit.GetPointAt(later), later.Seconds(), new double3(1.0, 0.0, 0.0), patch, vehicle);
        fc.AddBurn(players);
        CompleteAndStep(vehicle, departure);
        t.Check("player burn: with another burn planned the follow-up ends, adds no node and keeps the player's burns",
            FlybyCourse.Stage == FlybyCourseStage.Idle && fc.BurnPlan.BurnCount == 2 && fc.BurnPlan.TryGetBurn(players),
            $"stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s)");
        RcsFlightSupport.CleanupBurns(fc);

        // A correction node that the player changes.
        departure = Departure(vehicle, driver);
        if (departure == null)
            return;
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, departure);
        CompleteAndStep(vehicle, departure);
        Burn? correction = fc.BurnPlan.FindFirstExecutableBurn();
        if (!t.Check("changed node: a correction node is planned first", FlybyCourse.Stage == FlybyCourseStage.Correction && correction != null, FlybyCourse.LastResult))
            return;
        correction!.DeltaVVlf += new double3(0.5, 0.0, 0.0);
        FlybyCourse.TickVehicle(vehicle);
        t.Check("changed node: the follow-up ends and the node stays the player's",
            FlybyCourse.Stage == FlybyCourseStage.Idle && fc.BurnPlan.BurnCount == 1 && fc.BurnPlan.TryGetBurn(correction),
            $"stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s)");
        RcsFlightSupport.CleanupBurns(fc);
    }

    // A craft that already orbits the destination has no way to it inside one sphere of influence, so CREATE does not start a follow-up for it.
    private static void CheckNotFollowed(TestContext t, Vehicle vehicle, Vehicle donor, Celestial moon, double rp)
    {
        UniverseTime at = Universe.GetElapsedTime() + 600.0;
        // The node is never planned, so the patch of another craft serves for it.
        PatchedConic? patch = donor.FlightPlan.TryFindPatch(at);
        if (patch == null)
        {
            t.Skip("not followed: no flight-plan patch for a node.");
            return;
        }
        Burn node = Burn.Create(patch.Orbit.GetPointAt(at), at.Seconds(), new double3(1.0, 0.0, 0.0), patch, donor);
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, node);
        t.Check("not followed: a destination that does not orbit the craft's parent starts no follow-up", FlybyCourse.Stage == FlybyCourseStage.Idle && !FlybyCourse.Follows(vehicle));
    }

    // The course is read only after the settle steps, also when the settle time is over at once, and a craft that an AFC controller holds at the check keeps its burn plan and gets no node.
    private static void CheckSettleAndHolder(TestContext t, Vehicle vehicle, Celestial moon, SimDriver driver, double rp)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Burn? departure = Departure(vehicle, driver);
        if (departure == null)
        {
            t.Skip("settle: no departure node could be planned.");
            return;
        }
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, departure);
        fc.BurnMode = FlightComputerBurnMode.Manual;
        FlybyCourse.OnBurnCompleted(vehicle, departure);
        CheckAfter.SetValue(null, Universe.GetElapsedSeconds() - 1.0);
        for (int i = 0; i < CourseCorrector.SettleSteps - 1; i++)
            FlybyCourse.TickVehicle(vehicle);
        t.Check("settle: with the settle time over, the check still waits for the settle steps and leaves the flown node",
            FlybyCourse.Stage == FlybyCourseStage.Check && fc.BurnPlan.BurnCount == 1 && fc.BurnPlan.TryGetBurn(departure),
            $"stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s)");

        bool claimed = VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _);
        try
        {
            FlybyCourse.TickVehicle(vehicle);
            t.Check("holder: a craft that guidance holds at the check ends the follow-up, keeps its burn plan and gets no node",
                claimed && FlybyCourse.Stage == FlybyCourseStage.Idle && fc.BurnPlan.BurnCount == 1 && fc.BurnPlan.TryGetBurn(departure),
                $"claimed {claimed}, stage {FlybyCourse.Stage}, {fc.BurnPlan.BurnCount} burn(s)");
        }
        finally
        {
            VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // CREATE hands its node over before the game has added it to the plan (InputEvents.BurnUpdateBuffer), so the first steps see a node that is not planned yet.
    private static void CheckQueuedNode(TestContext t, Vehicle vehicle, Celestial moon, SimDriver driver, double rp)
    {
        FlightComputer fc = vehicle.FlightComputer;
        UniverseTime at = Universe.GetElapsedTime() + 1800.0;
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(at);
        if (patch == null)
        {
            t.Skip("queued node: no flight-plan patch for a node.");
            return;
        }
        Burn node = Burn.Create(patch.Orbit.GetPointAt(at), at.Seconds(), new double3(0.01, 0.0, 0.0), patch, vehicle);
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, node);
        FlybyCourse.TickVehicle(vehicle);
        t.Check("queued node: a step before the game has added the node keeps the follow-up", FlybyCourse.Stage == FlybyCourseStage.Departure);
        fc.AddBurn(node);
        driver.Step(1e-3, 2);
        FlybyCourse.TickVehicle(vehicle);
        fc.RemoveBurn(node);
        FlybyCourse.TickVehicle(vehicle);
        t.Check("queued node: once it was planned, its removal ends the follow-up", FlybyCourse.Stage == FlybyCourseStage.Idle);

        // The wait for the node counts steps, so the simulated time that one frame covers at a high simulation speed does not end it.
        at = Universe.GetElapsedTime() + 1800.0;
        patch = vehicle.FlightPlan.TryFindPatch(at);
        if (patch == null)
            return;
        node = Burn.Create(patch.Orbit.GetPointAt(at), at.Seconds(), new double3(0.01, 0.0, 0.0), patch, vehicle);
        FlybyCourse.OnDepartureCreated(vehicle, moon, rp, node);
        driver.Step(10.0, 1);
        FlybyCourse.TickVehicle(vehicle);
        t.Check("queued node: one step of 10 s before the game has added the node keeps the follow-up", FlybyCourse.Stage == FlybyCourseStage.Departure);
        FlybyCourse.TickVehicle(vehicle);
        FlybyCourse.TickVehicle(vehicle);
        t.Check("queued node: a node that the game never adds ends the follow-up after three steps", FlybyCourse.Stage == FlybyCourseStage.Idle);
    }
}
