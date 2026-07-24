using AdvancedFlightComputer.Features.Flyby;
using Brutal.Numerics;
using HeadlessHarness.Core;
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
// If no moon or spawnable vehicle is available it logs and passes (nothing to test).
public sealed class FlybyDepartureTest : IHarnessTest
{
    private const double SpawnAltitudeM = 400_000.0;
    private const double FlybyAltitudeM = 100_000.0;
    private const int SampleSteps = 600;

    private static readonly string[] SpawnableVehicles = { "Rocket", "Gemini7", "Polaris" };

    public string Name => "afc-flyby-departure";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        Celestial? moon = FindMoon(home);
        if (moon == null)
        {
            HarnessLog.Line($"[{Name}] SKIP: no moon under home body to fly by.");
            return 0;
        }

        VehicleSave? save = FirstAvailableSave();
        if (save == null)
        {
            HarnessLog.Line($"[{Name}] SKIP: no shipped default vehicle to spawn.");
            return 0;
        }

        SimDriver driver = session.CreateDriver();
        Vehicle vehicle = Spawn(session, home, save, "FlybyDeparture");
        try
        {
            driver.Step(1e-3, 2);
            return RunChecks(home, moon, vehicle) ? 0 : 1;
        }
        finally
        {
            VehicleSpawner.Despawn(vehicle);
        }
    }

    private bool RunChecks(IParentBody home, Celestial moon, Vehicle vehicle)
    {
        SimTime now = Universe.GetElapsedSimTime();
        SimTime start = new SimTime(now.Seconds() + 60.0);
        SimTime transit = OrbitalTransfers.HohmannFlight(vehicle.Orbit, moon.Orbit);
        if (!(transit.Seconds() > 0.0))
        {
            HarnessLog.Line($"[{Name}] SKIP: non-positive Hohmann time of flight.");
            return true;
        }

        double rp = ((IParentBody)moon).MeanRadius + FlybyAltitudeM;
        double surface = ((IParentBody)moon).GetNearSurfaceRadius();
        bool ok = true;

        // Baseline: the stock center-aimed transfer impacts (passes through center).
        StateVectors sv = vehicle.Orbit.GetStateVectorsAt(start);
        double3 moonAtArrival = moon.Orbit.GetStateVectorsAt(start + transit).PositionCci;
        OrbitalTransfers.SuperiorLambert(
            home.Mu, sv.PositionCci, moonAtArrival, transit, out double3 vEjectCenter, out _);
        Orbit centerTransfer = Orbit.CreateFromStateCci(
            home, start, sv.PositionCci, vEjectCenter, vehicle.Orbit.OrbitLineColor);
        double centerCa = ClosestApproach(centerTransfer, moon.Orbit, start, transit, out _);
        bool centerImpacts = centerCa < ((IParentBody)moon).MeanRadius;
        ok &= centerImpacts;
        HarnessLog.Line($"[{Name}] baseline center-aim closest approach {centerCa:E6} " +
            $"(moon radius {((IParentBody)moon).MeanRadius:E6}) impacts={centerImpacts} => {TestSupport.Verdict(centerImpacts)}");

        // Outer: the periapsis must sit on the far side of the moon from the parent.
        double3 missOuter = double3.Zero;
        ok &= CheckFlybySide(home, moon, vehicle, start, transit, rp, surface,
            FlybySide.Outer, out missOuter);

        // Inner: same clearance, opposite side.
        double3 missInner = double3.Zero;
        ok &= CheckFlybySide(home, moon, vehicle, start, transit, rp, surface,
            FlybySide.Inner, out missInner);

        if (missOuter.LengthSquared() > 0.0 && missInner.LengthSquared() > 0.0)
        {
            double dot = double3.Dot(missOuter.Normalized(), missInner.Normalized());
            bool opposite = dot < 0.0;
            ok &= opposite;
            HarnessLog.Line($"[{Name}] Inner vs Outer: miss-direction dot {dot:F3} (want < 0) => {TestSupport.Verdict(opposite)}");
        }

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok;
    }

    private bool CheckFlybySide(
        IParentBody home, Celestial moon, Vehicle vehicle, SimTime start, SimTime transit,
        double rp, double surface, FlybySide side, out double3 missVec)
    {
        missVec = double3.Zero;

        FlybyTargeting.FlybyOutcome outcome =
            FlybyTargeting.ComputeFlybyDeparture(vehicle, moon, start, transit, rp, side);
        if (outcome.Result == null)
        {
            // An axis nearly along the approach cannot be aimed at; that is a
            // property of the geometry, not a failure of the retarget.
            if (!outcome.CanReach(side))
            {
                HarnessLog.Line($"[{Name}] side {side}: unreachable for this approach " +
                    $"(axis alignment {outcome.AxisAlignmentFor(side):F3}), skipping.");
                return true;
            }
            HarnessLog.Line($"[{Name}] side {side}: retarget returned no result => FAIL");
            return false;
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
        double achievedRp = FlybyTargeting.PeriapsisForImpactParameter(
            f.VInfMs, ca, ((IParentBody)moon).Mu);
        bool rpOk = ManeuverTestSupport.NearRel(achievedRp, rp, 0.15);

        // The named side has to hold: the miss must lean along the requested axis
        // of the moon's own orbital frame (Outer away from the parent, Inner
        // toward it), which is the whole point of picking a side.
        double3 caMissHat = missVec.NormalizeOrZero();
        double3 moonRadialHat = moon.Orbit
            .GetStateVectorsAt(new SimTime(f.BurnTime.Seconds())).PositionCci.NormalizeOrZero();
        double radialLean = double3.Dot(caMissHat, moonRadialHat);
        bool sideOk = side == FlybySide.Outer ? radialLean > 0.0 : radialLean < 0.0;

        bool ok = clears && rpOk && sideOk;
        HarnessLog.Line($"[{Name}] side {side}: closest approach {ca:E6} b={b:E6} " +
            $"achievedRp={achievedRp:E6} (target {rp:E6}) rpOk={rpOk} " +
            $"clears(floor {surface:E6})={clears} " +
            $"radialLean={radialLean:F3} sideOk={sideOk} vInf={f.VInfMs:F1} " +
            $"=> {TestSupport.Verdict(ok)}");
        return ok;
    }

    // Minimum center-to-center distance between the transfer and the moon over
    // [start, start + 1.3*transit], plus the miss vector (transfer - moon) at that time.
    private static double ClosestApproach(
        Orbit transfer, Orbit moon, SimTime start, SimTime transit, out double3 missVec)
    {
        double a = start.Seconds();
        double bEnd = a + transit.Seconds() * 1.3;
        double best = double.MaxValue;
        missVec = double3.Zero;
        for (int i = 0; i <= SampleSteps; i++)
        {
            double t = a + (bEnd - a) * i / SampleSteps;
            var st = new SimTime(t);
            double3 d = transfer.GetStateVectorsAt(st).PositionCci - moon.GetStateVectorsAt(st).PositionCci;
            double len = d.Length();
            if (len < best) { best = len; missVec = d; }
        }
        return best;
    }

    private static Celestial? FindMoon(IParentBody home)
    {
        foreach (IOrbiter child in home.Children)
            if (child is Celestial moon && moon is IParentBody pb
                && pb.SphereOfInfluence > 0.0 && !double.IsNaN(pb.SphereOfInfluence))
                return moon;
        return null;
    }

    private static VehicleSave? FirstAvailableSave()
    {
        foreach (string id in SpawnableVehicles)
            if (DefaultVehicleSaves.FindSave(id) is VehicleSave save)
                return save;
        return null;
    }

    // Mirrors SequenceBurnStateTest.Spawn: a shipped default vehicle in a circular
    // orbit around the home body, added to the body's children.
    private static Vehicle Spawn(HeadlessSession session, IParentBody home, VehicleSave save, string id)
    {
        PartInstance design = save.VehicleSaveData.RootPartInstance
            ?? throw new InvalidOperationException($"default vehicle '{save.Id}' has no root part instance.");
        PartTree tree = PartTree.Deserialize(design);
        SimTime now = Universe.GetElapsedSimTime();
        Orbit orbit = VehicleSpawner.CircularCci(home, home.MeanRadius + SpawnAltitudeM, now);
        Vehicle vehicle = Vehicle.CreateVehicle(
            session.System, doubleQuat.Identity, double3.Zero, home, id, tree.Root, orbit);
        vehicle.Parts.SequenceList.SetActiveSequence(save.VehicleSaveData.ActiveSequence);
        vehicle.Parts.SequenceList.ApplyEnvironments(save.VehicleSaveData.SequenceEnvironments);
        vehicle.Parts.FuelLinks.ApplySaveData(save.VehicleSaveData.FuelLinks, design);
        home.Children.Add(vehicle);
        return vehicle;
    }
}
