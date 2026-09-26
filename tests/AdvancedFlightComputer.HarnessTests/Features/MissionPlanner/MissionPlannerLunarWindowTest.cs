using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Features.MissionPlanner;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The mission planner's launch to meet the home body's moon, against the loaded system's own ephemeris and rotation.
// Nothing is flown: a craft is spawned co-rotating over a pad, and each check reads a plan or a commit.
//
// The plane is checked for containing the moon at arrival, both ways round (the inclination set and the node set), and the launch window
// for putting the site under that plane a lead after ignition, with the site carried by the game's own frames rather than the planner's
// rotation. LAUNCH NOW's plane has to contain the site and the moon and quote a window of now. SEND TO ASCENT has to become the ascent's
// target with the same window, which EXECUTE then arms for, and a committed ascent has to refuse the next one.
public sealed class MissionPlannerLunarWindowTest : AfcTest
{
    public override string Name => "afc-mission-planner-lunar-window";

    private const double SiteLatDeg = 28.5;
    private const double PadHeightM = 2000.0;
    private const double ParkingKm = 200.0;
    private const double ArrivalAfterSoonestS = 2.0 * 86400.0;

    private static readonly string[] SpawnableVehicles = { "Rocket", "Gemini7", "Polaris" };

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo TryChaseOrbit = AccessTools.Method(typeof(GuidanceWindow), "TryChaseOrbit");
    private static readonly MethodInfo ExecuteAscent = AccessTools.Method(typeof(GuidanceWindow), "ExecuteAscent");
    private static readonly MethodInfo AbortAscent = AccessTools.Method(typeof(GuidanceWindow), "AbortAscent");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        Celestial? moon = TestWorld.FindMoon(home);
        if (moon == null)
        {
            t.Skip("no moon under the home body to plan for.");
            return;
        }
        VehicleSave? save = FirstAvailableSave();
        if (save == null)
        {
            t.Skip("no shipped default vehicle to spawn.");
            return;
        }
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        UniverseTime now = Universe.GetElapsedTime();
        double lat = SiteLatDeg * Math.PI / 180.0;
        double3 dirCci = double3.Normalize(new double3(Math.Cos(lat), 0.0, Math.Sin(lat)).Transform(home.GetCcf2Cci()));
        double3 r = dirCci * (home.MeanRadius + PadHeightM);
        double3 v = double3.Cross(new double3(0.0, 0.0, home.GetAngularVelocity()), r);
        Orbit pad = Orbit.CreateFromStateCci(home, now, r, v, VehicleSpawner.OrbitLineColor);
        Vehicle vehicle = VehicleFixtures.SpawnFromSaveData(t.System, home, save.VehicleSaveData, "MissionPlannerPad", pad);

        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        bool previousPanel = GuidanceWindow.PanelVisible;
        try
        {
            Run(t, home, moon, vehicle);
        }
        finally
        {
            VehicleAutopilotState.Remove(vehicle);
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            GuidanceWindow.PanelVisible = previousPanel;
            AmbientState() = previousAmbient;
            VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void Run(TestContext t, IParentBody home, Celestial moon, Vehicle vehicle)
    {
        double now = Universe.GetElapsedSeconds();
        double mu = home.Mu;
        double parkingRadius = home.MeanRadius + ParkingKm * 1000.0;

        double soonest = LunarLaunchPlanner.EarliestArrival(moon, mu, parkingRadius, now);
        double transferThen = LunarTransferGeometry.HohmannTime(mu, parkingRadius, LunarLaunchPlanner.MoonAt(moon, soonest).Length());
        t.CheckAbs("the soonest arrival is one Hohmann transfer from now, s", soonest - now, transferThen, 1.0);

        double arrival = soonest + ArrivalAfterSoonestS;
        double3 moonAt = LunarLaunchPlanner.MoonAt(moon, arrival);
        double3 u = double3.Normalize(moonAt);
        double decDeg = Math.Asin(u.Z) * 180.0 / Math.PI;
        double incDeg = Math.Max(SiteLatDeg, Math.Abs(decDeg)) + 1.0;
        t.Info($"{moon.Id} at arrival: {moonAt.Length() / 1000.0:F0} km, declination {decDeg:F2} deg; planning at {incDeg:F2} deg.");

        var inputs = new LunarLaunchInputs(arrival, PlaneControl.Inclination, incDeg, 0.0, false, ParkingKm, ParkingKm);
        LunarLaunchPlan north = LunarLaunchPlanner.Solve(vehicle, moon, now, inputs);
        LunarLaunchPlan south = LunarLaunchPlanner.Solve(vehicle, moon, now, inputs with { Southbound = true });
        if (!t.Check("both planes of the inclination reach the moon", north.HasPlane && south.HasPlane
                && north.Problem.Length == 0 && south.Problem.Length == 0, $"'{north.Problem}' '{south.Problem}'"))
            return;

        foreach ((string name, LunarLaunchPlan plan) in new[] { ("northbound", north), ("southbound", south) })
        {
            double3 n = UpfgTarget.OrbitNormal(Rad(plan.IncDeg), Rad(plan.LanDeg));
            t.CheckAbs($"the {name} plane contains the moon at arrival, n . u", double3.Dot(n, u), 0.0, 1e-9);
            double along = double3.Dot(u, UpfgTarget.NodeDirection(Rad(plan.LanDeg)));
            t.Check($"the {name} plane meets it on its {name} half", name == "northbound" ? along >= 0.0 : along <= 0.0,
                $"u . node {along:F4}");
            CheckWindow(t, home, vehicle, now, plan, name);
        }
        t.Check("the two planes are different", AngleApart(north.LanDeg, south.LanDeg) > 1e-3,
            $"LAN {north.LanDeg:F3} and {south.LanDeg:F3}");

        double transfer = LunarTransferGeometry.HohmannTime(mu, parkingRadius, moonAt.Length());
        t.CheckAbs("the injection is one transfer before the arrival, s", north.InjectionTime, arrival - transfer, 1e-6);
        double3 atInjection = double3.Normalize(LunarLaunchPlanner.MoonAt(moon, north.InjectionTime));
        t.CheckAbs("the moon's travel is measured from the injection to the arrival, deg", north.MoonTravelDeg,
            Math.Acos(Math.Clamp(double3.Dot(u, atInjection), -1.0, 1.0)) * 180.0 / Math.PI, 1e-9);
        t.Check("the coast is from insertion to the injection", Math.Abs(north.CoastSec
            - (north.InjectionTime - now - north.WaitSec - LunarLaunchPlanner.InsertionAfterIgnitionS)) < 1e-6);

        // Setting the node gives the inclination back.
        LunarLaunchPlan byNode = LunarLaunchPlanner.Solve(vehicle, moon, now,
            inputs with { Control = PlaneControl.Node, LanDeg = north.LanDeg });
        t.CheckAbs("setting the node gives the inclination back, deg", byNode.IncDeg, incDeg, 1e-6);
        t.CheckAbs("and the same window, s", byNode.WaitSec, north.WaitSec, 1e-6);

        // Too shallow a plane for the moon's declination has no node.
        if (Math.Abs(decDeg) > 1.0)
        {
            LunarLaunchPlan shallow = LunarLaunchPlanner.Solve(vehicle, moon, now, inputs with { IncDeg = Math.Abs(decDeg) - 0.5 });
            t.Check("a plane shallower than the moon's declination is refused", !shallow.HasPlane && shallow.Problem.Length > 0,
                shallow.Problem);
        }

        CheckLaunchNow(t, home, moon, vehicle, now, arrival, inputs);
        CheckSendToAscent(t, home, vehicle, now, north);
    }

    // The site, carried by the game's own rotation, is under the plane when the crossing comes: a lead after ignition.
    private static void CheckWindow(TestContext t, IParentBody home, Vehicle vehicle, double now, LunarLaunchPlan plan, string name)
    {
        if (!t.Check($"the site passes under the {name} plane", plan.SiteReachesPlane, plan.Problem))
            return;
        double day = 2.0 * Math.PI / home.GetAngularVelocity();
        t.Check($"the {name} window is within a turn of the body", plan.WaitSec >= 0.0 && plan.WaitSec < day,
            $"{plan.WaitSec:F0} s of a {day:F0} s turn");
        double crossing = now + plan.WaitSec + GuidanceWindow.LanLeadSeconds;
        double3 site = double3.Normalize(SiteAt(home, vehicle, now, crossing));
        double3 n = UpfgTarget.OrbitNormal(Rad(plan.IncDeg), Rad(plan.LanDeg));
        t.CheckAbs($"a lead after the {name} window's ignition the site is in the plane, m", double3.Dot(n, site) * home.MeanRadius, 0.0, 50.0);
        double argLat = LunarTransferGeometry.ArgumentOfLatitude(site, Rad(plan.IncDeg), Rad(plan.LanDeg));
        t.Check($"the {name} window's crossing is the one it says", plan.Descending ? Math.Cos(argLat) <= 1e-9 : Math.Cos(argLat) >= -1e-9,
            $"{(plan.Descending ? "descending" : "ascending")}, argument of latitude {argLat * 180.0 / Math.PI:F2} deg");
    }

    private static void CheckLaunchNow(TestContext t, IParentBody home, Celestial moon, Vehicle vehicle, double now,
                                       double arrival, LunarLaunchInputs inputs)
    {
        if (!t.Check("LAUNCH NOW finds a plane", LunarLaunchPlanner.TryLaunchNowPlane(vehicle, moon, arrival, out double incDeg, out double lanDeg)))
            return;
        double3 n = UpfgTarget.OrbitNormal(Rad(incDeg), Rad(lanDeg));
        double3 u = double3.Normalize(LunarLaunchPlanner.MoonAt(moon, arrival));
        double3 site = double3.Normalize(SiteAt(home, vehicle, now, now + GuidanceWindow.LanLeadSeconds));
        t.CheckAbs("LAUNCH NOW's plane contains the moon at arrival", double3.Dot(n, u), 0.0, 1e-9);
        t.CheckAbs("and the site a lead from now, m", double3.Dot(n, site) * home.MeanRadius, 0.0, 50.0);
        t.Check("and is flown eastward", incDeg <= 90.0, $"{incDeg:F2} deg");
        LunarLaunchPlan plan = LunarLaunchPlanner.Solve(vehicle, moon, now, inputs with { Control = PlaneControl.Node, LanDeg = lanDeg });
        t.CheckAbs("its inclination follows from its node, deg", plan.IncDeg, incDeg, 1e-6);
        t.Check("its window is now", plan.SiteReachesPlane && plan.WaitSec < 1.0, $"T-{plan.WaitSec:F2} s");
    }

    private static void CheckSendToAscent(TestContext t, IParentBody home, Vehicle vehicle, double now, LunarLaunchPlan plan)
    {
        SharedVehicleHooks.GuidanceEnabled = false;
        t.Check("guidance that did not load refuses the plan", GuidanceWindow.PlaneTargetRefusal(vehicle) == GuidanceFeature.UnavailableReason);
        SharedVehicleHooks.GuidanceEnabled = true;
        GuidanceWindow.SetModActive(true);

        var target = new AscentPlaneTarget(plan.IncDeg, plan.LanDeg, ParkingKm, ParkingKm, "harness plan");
        if (!t.Check("SEND TO ASCENT is accepted", GuidanceWindow.TrySendPlaneTarget(vehicle, target, out string why), why))
            return;
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        t.Check("the plane is the ascent's target", ReferenceEquals(state.PlaneTarget, target) && state.TargetId.Length == 0);
        t.Check("and drives its orbit", state.IncDeg == plan.IncDeg && state.LanDeg == plan.LanDeg
            && state.PeKm == ParkingKm && state.ApKm == ParkingKm && !state.ArgPeFixed,
            $"inc {state.IncDeg:F3}, LAN {state.LanDeg:F3}, {state.PeKm:F0} x {state.ApKm:F0} km");
        t.CheckAbs("the ascent's window is the planner's, s", state.LaunchTargetTime - now, plan.WaitSec, 1.0);

        AmbientState() = state;
        object?[] args = { vehicle, vehicle.Orbit, home, home.MeanRadius, null };
        var status = (GuidanceWindow.ChaseStatus)TryChaseOrbit.Invoke(null, args)!;
        var chase = (GuidanceWindow.ChasePlan)args[4]!;
        t.Check("the ascent chases the plane", status == GuidanceWindow.ChaseStatus.Ok, status.ToString());
        t.Check("with the plane's orbit and no periapsis to copy", chase.IncDeg == plan.IncDeg && chase.LanDeg == plan.LanDeg
            && chase.PeKm == ParkingKm && chase.ApKm == ParkingKm && double.IsNaN(chase.ArgPeDeg));
        t.CheckAbs("and the planner's window, s", chase.WaitSec, plan.WaitSec, 1e-6);

        // EXECUTE arms for the window rather than launching now. The convex plan is left out: calculating one is its own test's business.
        state.FlyConvexAscent = false;
        ExecuteAscent.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
        t.Check("EXECUTE arms for the plane's window", state.LaunchArmed && !state.Running,
            $"armed {state.LaunchArmed}, running {state.Running}");
        t.Check("a committed ascent refuses the next plan",
            !GuidanceWindow.TrySendPlaneTarget(vehicle, target with { LanDeg = plan.LanDeg + 1.0 }, out string refusal)
            && ReferenceEquals(state.PlaneTarget, target), refusal);
        AbortAscent.Invoke(null, null);
        t.Check("ABORT disarms it", !state.LaunchArmed && !state.Running);
    }

    // Where the pad is at time, turned by the body's own CCI/CCF frames.
    private static double3 SiteAt(IParentBody home, Vehicle vehicle, double now, double time)
        => vehicle.Orbit.StateVectors.PositionCci.Transform(home.GetCci2Ccf(new UniverseTime(now)))
            .Transform(home.GetCcf2Cci(new UniverseTime(time)));

    private static double Rad(double deg) => deg * Math.PI / 180.0;

    private static double AngleApart(double a, double b)
    {
        double d = ((a - b) % 360.0 + 360.0) % 360.0;
        return Math.Min(d, 360.0 - d);
    }

    private static VehicleSave? FirstAvailableSave()
    {
        foreach (string id in SpawnableVehicles)
            if (DefaultVehicleSaves.FindSave(id) is VehicleSave save)
                return save;
        return null;
    }
}
