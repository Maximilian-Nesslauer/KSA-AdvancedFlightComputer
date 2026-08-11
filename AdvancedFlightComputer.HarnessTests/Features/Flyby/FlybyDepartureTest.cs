using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// End-to-end geometry check for the flyby departure retarget against a real moon.
// A Lambert solve reaches its aim point at the arrival time by construction, so:
//   * the stock center-aimed transfer passes through the moon center (an impact),
//   * the flyby-retargeted transfer misses the moon center by ~b (the impact
//     parameter) and clears the surface,
//   * flipping the side reverses the miss direction.
// This exercises the same-parent (moon) retarget path without needing SOI
// propagation: the two-body closest approach to the moon center is the oracle.
public sealed class FlybyDepartureTest : AfcTest
{
    private const double SpawnAltitudeM = 400_000.0;
    private const double FlybyAltitudeM = 100_000.0;
    private const int SampleSteps = 600;

    private static readonly string[] SpawnableVehicles = { "Rocket", "Gemini7", "Polaris" };

    public override string Name => "afc-flyby-departure";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        Celestial? moon = TestWorld.FindMoon(home);
        if (moon == null)
        {
            t.Skip("no moon under home body to fly by.");
            return;
        }

        VehicleSave? save = FirstAvailableSave();
        if (save == null)
        {
            t.Skip("no shipped default vehicle to spawn.");
            return;
        }

        SimDriver driver = t.Session.CreateDriver();
        Vehicle vehicle = VehicleFixtures.SpawnFromSaveData(
            t.System, home, save.VehicleSaveData, "FlybyDeparture",
            OrbitFixtures.CircularAt(home, SpawnAltitudeM, Universe.GetElapsedTime()));
        try
        {
            driver.Step(1e-3, 2);
            RunChecks(t, home, moon, vehicle);
        }
        finally
        {
            VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void RunChecks(TestContext t, IParentBody home, Celestial moon, Vehicle vehicle)
    {
        UniverseTime now = Universe.GetElapsedTime();
        UniverseTime start = now + 60.0;
        UniverseTime transit = OrbitalTransfers.HohmannFlight(vehicle.Orbit, moon.Orbit);
        if (!(transit.Seconds() > 0.0))
        {
            t.Skip("non-positive Hohmann time of flight.");
            return;
        }

        IParentBody moonBody = (IParentBody)moon;
        double rp = moonBody.MeanRadius + FlybyAltitudeM;
        double surface = moonBody.GetNearSurfaceRadius();

        // Baseline: the stock center-aimed transfer impacts (passes through center).
        StateVectors sv = vehicle.Orbit.GetStateVectorsAt(start);
        double3 moonAtArrival = moon.Orbit.GetStateVectorsAt(start + transit).PositionCci;
        OrbitalTransfers.SuperiorLambert(
            home.Mu, sv.PositionCci, moonAtArrival, transit, out double3 vEjectCenter, out _);
        Orbit centerTransfer = Orbit.CreateFromStateCci(
            home, start, sv.PositionCci, vEjectCenter, vehicle.Orbit.OrbitLineColor);
        double centerCa = ClosestApproach(centerTransfer, moon.Orbit, start, transit, out _);
        t.Check("baseline center-aim impacts", centerCa < moonBody.MeanRadius,
            $"closest approach {centerCa:E6} (moon radius {moonBody.MeanRadius:E6})");

        // Outer: the periapsis must sit on the far side of the moon from the parent.
        CheckFlybySide(t, home, moon, vehicle, start, transit, rp, surface,
            FlybySide.Outer, out double3 missOuter);

        // Inner: same clearance, opposite side.
        CheckFlybySide(t, home, moon, vehicle, start, transit, rp, surface,
            FlybySide.Inner, out double3 missInner);

        if (missOuter.LengthSquared() > 0.0 && missInner.LengthSquared() > 0.0)
        {
            double dot = double3.Dot(missOuter.Normalized(), missInner.Normalized());
            t.Check("Inner vs Outer miss on opposite sides", dot < 0.0,
                $"miss-direction dot {dot:F3} (want < 0)");
        }
    }

    private static void CheckFlybySide(
        TestContext t, IParentBody home, Celestial moon, Vehicle vehicle, UniverseTime start, UniverseTime transit,
        double rp, double surface, FlybySide side, out double3 missVec)
    {
        missVec = double3.Zero;
        IParentBody moonBody = (IParentBody)moon;

        FlybyTargeting.FlybyOutcome outcome =
            FlybyTargeting.ComputeFlybyDeparture(vehicle, moon, start, transit, rp, side);
        if (outcome.Result == null)
        {
            // An axis nearly along the approach cannot be aimed at; that is a
            // property of the geometry, not a failure of the retarget.
            if (!outcome.CanReach(side))
                t.Skip($"side {side}: unreachable for this approach " +
                       $"(axis alignment {outcome.AxisAlignmentFor(side):F3}).");
            else
                t.Fail($"side {side}", "retarget returned no result");
            return;
        }
        FlybyTargeting.FlybyResult f = outcome.Result.Value;
        double b = f.ImpactParameterMeters;

        StateVectors sv = vehicle.Orbit.GetStateVectorsAt(f.BurnTime);
        Orbit transfer = Orbit.CreateFromStateCci(
            home, f.BurnTime, sv.PositionCci, sv.VelocityCci + f.DvCci, vehicle.Orbit.OrbitLineColor);
        double ca = ClosestApproach(transfer, moon.Orbit, f.BurnTime, transit, out missVec);

        bool clears = ca > surface;
        // Invert the parent-frame miss distance back through the impact-parameter
        // relation: that is the periapsis this approach actually buys, and it must
        // land on the requested one. Asserting on b alone would accept a miss
        // distance that still maps to an impact.
        double achievedRp = FlybyTargeting.PeriapsisForImpactParameter(f.VInfMs, ca, moonBody.Mu);
        bool rpOk = Approx.Rel(achievedRp, rp, 0.15);

        // The named side has to hold: the miss must lean along the requested axis
        // of the moon's own orbital frame (Outer away from the parent, Inner
        // toward it), which is the whole point of picking a side.
        double3 caMissHat = missVec.NormalizeOrZero();
        double3 moonRadialHat = moon.Orbit
            .GetStateVectorsAt(f.BurnTime).PositionCci.NormalizeOrZero();
        double radialLean = double3.Dot(caMissHat, moonRadialHat);
        bool sideOk = side == FlybySide.Outer ? radialLean > 0.0 : radialLean < 0.0;

        t.Check($"side {side}", clears && rpOk && sideOk,
            $"closest approach {ca:E6} b={b:E6} achievedRp={achievedRp:E6} (target {rp:E6}) " +
            $"rpOk={rpOk} clears(floor {surface:E6})={clears} " +
            $"radialLean={radialLean:F3} sideOk={sideOk} vInf={f.VInfMs:F1}");
    }

    // Minimum center-to-center distance between the transfer and the moon over
    // [start, start + 1.3*transit], plus the miss vector (transfer - moon) at that time.
    private static double ClosestApproach(
        Orbit transfer, Orbit moon, UniverseTime start, UniverseTime transit, out double3 missVec)
    {
        double a = start.Seconds();
        double bEnd = a + transit.Seconds() * 1.3;
        double best = double.MaxValue;
        missVec = double3.Zero;
        for (int i = 0; i <= SampleSteps; i++)
        {
            double time = a + (bEnd - a) * i / SampleSteps;
            var st = new UniverseTime(time);
            double3 d = transfer.GetStateVectorsAt(st).PositionCci - moon.GetStateVectorsAt(st).PositionCci;
            double len = d.Length();
            if (len < best) { best = len; missVec = d; }
        }
        return best;
    }

    private static VehicleSave? FirstAvailableSave()
    {
        foreach (string id in SpawnableVehicles)
            if (DefaultVehicleSaves.FindSave(id) is VehicleSave save)
                return save;
        return null;
    }
}
