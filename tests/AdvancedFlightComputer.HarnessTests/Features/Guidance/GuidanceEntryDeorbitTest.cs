using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceEntryDeorbitTest : AfcTest
{
    public override string Name => "afc-guidance-deorbit-only";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        double entry = home.GetAtmosphereRadius();
        if (!(entry > home.MeanRadius))
        {
            t.Skip("the system has no atmosphere on the home body");
            return;
        }
        double altitude = 0.25 * (entry - home.MeanRadius);
        double parking = Math.Max(200_000.0, entry - home.MeanRadius + 100_000.0);
        UniverseTime now = Universe.GetElapsedTime();
        Orbit source = OrbitFixtures.CircularAt(home, parking, now);
        var model = new UpfgVehicle();
        model.Stages.Add(new UpfgStage { Thrust = 300_000.0, Isp = 300.0, MassTotal = 10_000.0, MassDry = 5_000.0 });
        var settings = new DeorbitSettings(0, 0, 0, 0, 0, 0, 1, EntryPeriapsis: altitude);
        var request = new DeorbitRequest(source, now.Seconds(), 10_000.0, 1.0, settings,
            model, [new(0.1, 30_000.0, 10.0), new(1.0, 300_000.0, 100.0)], 0.02, 0.02);
        DeorbitPlan? plan = EntryDeorbitPlanner.Plan(request, out string refusal);
        if (!t.Check("the entry corridor produces a stock deorbit burn", plan != null, refusal))
            return;
        Orbit lowered = Orbit.CreateFromStateCci(home, new UniverseTime(plan!.DepartureTime),
            plan.DeparturePosition, plan.DepartureVelocity, source.OrbitLineColor);
        t.CheckAbs("the planned periapsis matches the entry target",
            lowered.GetPeriapsisPositionOrb().Length() - home.MeanRadius, altitude, 0.1);
        t.CheckAbs("the coast ends at the actual atmosphere radius", plan.ArrivalPosition.Length(), entry, 1.0);
        t.Check("the craft crosses the interface downwards after the burn",
            plan.ArrivalTime > plan.DepartureTime + 30.0
            && double3.Dot(plan.ArrivalPosition, plan.ArrivalVelocity) < 0.0
            && double3.Dot(plan.DeltaV, plan.DepartureVelocity) < 0.0);
        t.Check("a target above the atmosphere is refused", EntryDeorbitPlanner.Refusal(home, parking).Length > 0);
        const double measuredPreparation = 180.0;
        DeorbitPlan? retry = EntryDeorbitPlanner.Plan(request, out refusal, measuredPreparation);
        if (!t.Check("the retry uses the measured stock turn time",
            retry != null && retry.DepartureTime >= now.Seconds() + measuredPreparation
            && retry.DepartureTime > plan.DepartureTime, refusal))
            return;
        Orbit retriedOrbit = Orbit.CreateFromStateCci(home, new UniverseTime(retry!.DepartureTime),
            retry.DeparturePosition, retry.DepartureVelocity, source.OrbitLineColor);
        t.CheckAbs("the later burn preserves the selected entry periapsis",
            retriedOrbit.GetPeriapsisPositionOrb().Length() - home.MeanRadius, altitude, 0.1);
    }
}
