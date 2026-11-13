using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The REACH row of the launch form on a craft over the Starbase pad, at latitude 26 deg: an equatorial target is out of
// direct reach and the row prices its plane change, the station plane is in reach and shows nothing, and a LAN that the
// site has turned away from shows as a plane change that FROM POSITION removes, for a climb that keeps the LAN. LAUNCH
// with the gravity turn refuses the equatorial plane and sets a stale LAN over the pad for the node picked, and while it
// waits with no target the LAN follows the pad. Into a target plane the gravity turn starts only at the window: LAUNCH,
// NOW and an armed launch that fires late are refused with the next window, and NOW keeps the countdown. A plane that
// never passes over the site is flown only while the site lies within 0.5 deg of it, and a retrograde body still brings
// a window. It measures
// the first stage's thrust-to-weight ratio of each save on the pad. It then asks the calculated climb for both the
// cheapest plane (inclination 26.0 deg) and the equatorial one, and logs what the plan makes of each, because the climb
// accepts an inclination below the site latitude and flies the plane change itself.
public sealed class GuidanceAscentReachTest : AfcTest
{
    public override string Name => "afc-guidance-ascent-reach";

    private const double SiteLatDeg = 25.996143841726674;
    private const double SiteLonDeg = -97.15441574026951;
    private const double PadHeightM = 300.0;
    private const double SpawnClimbMs = 15.0;
    private const double TargetAltKm = 200.0;
    private const double PlanTimeoutS = 180.0;

    private static readonly string[] Saves = { "Test Vehicle 1", "Test Vehicle 2" };

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo LanOverheadAtNode = AccessTools.Method(typeof(GuidanceWindow), "LanOverheadAtNode");

    private static readonly MethodInfo TryReachFacts = AccessTools.Method(typeof(GuidanceWindow), "TryReachFacts");

    private static readonly MethodInfo StartGuidance = AccessTools.Method(typeof(GuidanceWindow), "StartGuidance");

    private static readonly MethodInfo AbortAscent = AccessTools.Method(typeof(GuidanceWindow), "AbortAscent");

    private static readonly MethodInfo MeasurePadTwr = AccessTools.Method(typeof(GuidanceWindow), "MeasurePadTwr");

    private static readonly MethodInfo TryChaseOrbit = AccessTools.Method(typeof(GuidanceWindow), "TryChaseOrbit");

    private static readonly MethodInfo LaunchNowAscent = AccessTools.Method(typeof(GuidanceWindow), "LaunchNowAscent");

    private static readonly MethodInfo StepLaunchWindow = AccessTools.Method(typeof(GuidanceWindow), "StepLaunchWindow");

    // Read by reflection, so the test also builds where the state has no such field and then fails the checks that need it.
    private static readonly FieldInfo? LanFollowsPadField = AccessTools.Field(typeof(VehicleAutopilotState), "LanFollowsPad");

    private static bool FollowsPad(VehicleAutopilotState state) => LanFollowsPadField?.GetValue(state) is true;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (home is not Celestial body)
        {
            t.Skip("the home body is not a celestial with terrain.");
            return;
        }
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(Saves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", Saves)}' is in the game's Vehicles folder.");
            return;
        }
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        Vehicle? previousFocus = Program.ControlledVehicle;
        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.ascent-reach");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            double lat = SiteLatDeg * Math.PI / 180.0, lon = SiteLonDeg * Math.PI / 180.0;
            var dirCcf = new double3(Math.Cos(lat) * Math.Cos(lon), Math.Cos(lat) * Math.Sin(lon), Math.Sin(lat));
            double terrain = body.GetTerrainHeightFromDirCcf(dirCcf);
            if (!double.IsFinite(terrain) || terrain < 0.0)
                terrain = 0.0;
            double3 dirCci = double3.Normalize(dirCcf.Transform(home.GetCcf2Cci()));
            double3 r = dirCci * (home.MeanRadius + terrain + PadHeightM);
            double3 v = double3.Cross(new double3(0, 0, home.GetAngularVelocity()), r) + SpawnClimbMs * dirCci;
            Orbit pad = Orbit.CreateFromStateCci(home, Universe.GetElapsedTime(), r, v, VehicleSpawner.OrbitLineColor);
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessAscentReach", pad);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }
            // The other saves 500 m above the pad and each other, for the TWR only.
            var measured = new List<(Vehicle Craft, string Save)> { (vehicle, saves[0]) };
            for (int i = 1; i < saves.Count; i++)
            {
                double3 rOther = r + dirCci * (500.0 * i);
                Orbit above = Orbit.CreateFromStateCci(home, Universe.GetElapsedTime(), rOther, double3.Cross(new double3(0, 0, home.GetAngularVelocity()), rOther), VehicleSpawner.OrbitLineColor);
                try
                {
                    measured.Add((VehicleSpawner.SpawnFromSave(saves[i], t.System, home, "HarnessAscentReachTwr" + i, above), saves[i]));
                }
                catch (InvalidOperationException e)
                {
                    t.Info($"'{saves[i]}' not spawned for the TWR: {e.Message}");
                }
            }
            t.Info($"'{saves[0]}' spawned over Starbase, lat {SiteLatDeg:F2}, {vehicle.TotalMass / 1000.0:F1} t");
            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 2);
            // After the steps, because a craft can have no sequence performance before its first one.
            foreach ((Vehicle craft, string save) in measured)
                CheckPadTwr(t, craft, home, save);

            VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
            state.PeKm = TargetAltKm;
            state.ApKm = TargetAltKm;
            state.TargetId = "";
            state.ArgPeFixed = false;
            state.ReserveArmed = false;
            state.FlyConvexAscent = true;
            AmbientState() = state;

            CheckRow(t, vehicle, home);
            CheckGravityTurnPlane(t, vehicle, home, driver, state);
            CheckGravityTurnWindow(t, vehicle, home, driver, state);
            CheckPlaneOffSite(t, vehicle, home, driver, state);
            CheckLanFollowsPad(t, vehicle, home, state);
            Plan(t, vehicle, home, driver, state, LaunchReach.NearestDirectIncDeg(SiteLatDeg, 0.0), out double massDueEast);
            Plan(t, vehicle, home, driver, state, 0.0, out double massEquator);
            if (double.IsFinite(massDueEast) && double.IsFinite(massEquator))
                t.Info($"equatorial plan carries {(massDueEast - massEquator) / 1000.0:F2} t less to orbit than the plan due east ({massEquator / 1000.0:F2} t against {massDueEast / 1000.0:F2} t)");
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private static ReachFacts? Reach(Vehicle vehicle, IParentBody home)
    {
        object?[] args = { vehicle.Orbit, home, false, GuidanceWindow.ChaseStatus.NoTarget, default(GuidanceWindow.ChasePlan), null };
        return (bool)TryReachFacts.Invoke(null, args)! ? (ReachFacts)args[5]! : null;
    }

    private static void SetPlane(VehicleAutopilotState state, Vehicle vehicle, IParentBody home, double incDeg)
    {
        state.IncDeg = incDeg;
        state.LanDeg = LanOver(vehicle, home, incDeg, false);
        state.LanSeeded = true;
    }

    private static void CheckRow(TestContext t, Vehicle vehicle, IParentBody home)
    {
        VehicleAutopilotState state = AmbientState();
        SetPlane(state, vehicle, home, 0.0);
        ReachFacts? equator = Reach(vehicle, home);
        if (!t.Check("the pad has reach facts", equator != null))
            return;
        ReachFacts f = equator!.Value;
        t.Info($"equator: site lat {f.SiteLatDeg:F3}, plane change {f.PlaneChangeDeg:F2} deg, least {f.LeastPlaneChangeDeg:F2} deg, {f.PlaneChangeMs:F0} m/s, row '{PageText.Reach(in f)}'");
        t.CheckAbs("the site latitude, deg", f.SiteLatDeg, SiteLatDeg, 0.05);
        t.Check("an equatorial target shows a plane change out of reach", f.Shows && f.OutOfReach);
        t.CheckAbs("its plane change is the site latitude, deg", f.PlaneChangeDeg, SiteLatDeg, 0.1);
        t.CheckAbs("its price at 200 km, m/s", f.PlaneChangeMs, 3504.0, 25.0);
        t.Check("the row offers 26.0 deg", f.OfferIncDeg == 26.0, f.OfferIncDeg.ToString("F2"));

        // With the calculated climb the node is solved over the pad, so the pitch program is what reads a fixed LAN.
        state.FlyConvexAscent = false;
        SetPlane(state, vehicle, home, 51.6);
        ReachFacts? station = Reach(vehicle, home);
        t.Check("the station plane over the site shows nothing", station is { Shows: false }, station?.PlaneChangeDeg.ToString("F3") ?? "no facts");
        state.LanDeg += 20.0;
        ReachFacts? turnStale = Reach(vehicle, home);
        t.Check("the gravity turn sets the LAN over the pad at LAUNCH, so a stale LAN shows nothing for it", turnStale is { Shows: false }, turnStale?.PlaneChangeDeg.ToString("F3") ?? "no facts");
        state.FlyConvexAscent = true;
        state.ConvexOptimiseTiming = false;
        ReachFacts? stale = Reach(vehicle, home);
        t.Check("a LAN turned 20 deg away shows a plane change that is not the inclination's, for a climb that keeps the LAN", stale is { Shows: true, OutOfReach: false } && double.IsNaN(stale.Value.OfferIncDeg),
            stale?.PlaneChangeDeg.ToString("F2") ?? "no facts");
        state.ConvexOptimiseTiming = true;
        state.LanDeg -= 20.0;
        ReachFacts? solved = Reach(vehicle, home);
        t.Check("the calculated climb solves the node, so the station plane shows nothing there", solved is { Shows: false });
    }

    private static double LanOver(Vehicle vehicle, IParentBody home, double incDeg, bool descending)
        => (double)LanOverheadAtNode.Invoke(null, new object[] { vehicle.Orbit.StateVectors.PositionCci, incDeg, home, descending })!;

    // The LAUNCH commit of the gravity turn, through the entry point that the commit, the backup and the armed window share.
    private static void CheckGravityTurnPlane(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver, VehicleAutopilotState state)
    {
        state.FlyConvexAscent = false;
        state.Engage = false;
        state.LaunchDescending = false;
        state.ChaseDriven = false;
        try
        {
            SetPlane(state, vehicle, home, 0.0);
            StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
            t.Info($"inc 0.0, gravity turn: status '{state.Status}'");
            t.Check("LAUNCH refuses a gravity turn into the equator from Starbase", !state.Running && state.Status.StartsWith("REFUSED: INC 0.0 deg OUT OF REACH FROM LATITUDE 26.0 deg"), state.Status);
            t.Check("the refusal names the nearest inclination", state.Status.EndsWith("NEAREST: INC 26.0 deg."), state.Status);

            foreach (bool descending in new[] { false, true })
            {
                string node = descending ? "descending" : "ascending";
                SetPlane(state, vehicle, home, 51.6);
                double over = LanOver(vehicle, home, 51.6, descending);
                state.LanDeg += 20.0;
                state.LaunchDescending = descending;
                StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
                bool running = state.Running;
                double lan = state.LanDeg;
                AbortAscent.Invoke(null, new object[] { vehicle });
                driver.Step(0.02, 2);
                t.Check($"LAUNCH flies the station plane, {node} node", running, state.Status);
                t.CheckAbs($"LAUNCH sets the stale LAN over the pad, {node} node, deg", Math.IEEERemainder(lan - over, 360.0), 0.0, 0.01);

                // The site, where the body carries it in the 180 s lead of the LAN, lies in that plane, and the plane carries it north-east at the ascending node and south-east at the descending one.
                double3 n = UpfgTarget.OrbitNormal(UpfgTarget.DegToRad(51.6), UpfgTarget.DegToRad(lan));
                double3 rNow = vehicle.Orbit.StateVectors.PositionCci;
                double turn = home.GetAngularVelocity() * 180.0;
                double3 site = double3.Normalize(new double3(Math.Cos(turn) * rNow.X - Math.Sin(turn) * rNow.Y, Math.Sin(turn) * rNow.X + Math.Cos(turn) * rNow.Y, rNow.Z));
                double change = LaunchReach.PlaneChangeDeg(n.X, n.Y, n.Z, site.X, site.Y, site.Z);
                double3 along = double3.Cross(n, site);
                double3 north = double3.Normalize(new double3(0, 0, 1) - site * site.Z);
                double northward = double3.Dot(along, north);
                t.Info($"{node} node: LAN {lan:F2} deg, site {change:F4} deg off the plane, northward part of the track {northward:F3}");
                t.Check($"the site lies in the plane, {node} node", change < 0.05, change.ToString("F4"));
                t.Check($"the track at the site heads {(descending ? "south" : "north")}, {node} node", descending ? northward < -0.1 : northward > 0.1, northward.ToString("F3"));
            }
        }
        finally
        {
            if (state.Running)
                AbortAscent.Invoke(null, new object[] { vehicle });
            state.LaunchDescending = false;
            state.FlyConvexAscent = true;
            driver.Step(0.02, 2);
        }
    }

    private static GuidanceWindow.ChaseStatus Chase(Vehicle vehicle, IParentBody home, out GuidanceWindow.ChasePlan plan)
    {
        object?[] args = { vehicle, vehicle.Orbit, home, home.MeanRadius, null };
        var status = (GuidanceWindow.ChaseStatus)TryChaseOrbit.Invoke(null, args)!;
        plan = (GuidanceWindow.ChasePlan)args[4]!;
        return status;
    }

    // A plane target of the station inclination over the pad now, and one whose node lies 10 deg east of it, so its window is ahead.
    // The sim step clears a plane target that no moon sets (RefreshMoonLaunchPlane), so each case sets it again and steps only after.
    private static void SetTargetPlane(VehicleAutopilotState state, double lanDeg)
    {
        state.PlaneTarget = new AscentPlaneTarget(51.6, lanDeg, TargetAltKm, TargetAltKm, "test plane");
        state.IncDeg = 51.6;
        state.LanDeg = lanDeg;
        state.ChaseDriven = true;
    }

    private static void CheckGravityTurnWindow(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver, VehicleAutopilotState state)
    {
        state.FlyConvexAscent = false;
        state.Engage = false;
        state.LaunchDescending = false;
        double over = LanOver(vehicle, home, 51.6, false);
        try
        {
            SetTargetPlane(state, over);
            Chase(vehicle, home, out GuidanceWindow.ChasePlan atWindow);
            StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
            bool running = state.Running;
            t.Info($"target plane over the pad: wait {atWindow.WaitSec:F1} s, status '{state.Status}'");
            AbortAscent.Invoke(null, new object[] { vehicle });
            driver.Step(0.02, 2);
            t.Check("the gravity turn flies a target plane at its window", running, state.Status);

            SetTargetPlane(state, over + 10.0);
            Chase(vehicle, home, out GuidanceWindow.ChasePlan ahead);
            StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
            string refusal = state.Status;
            t.Info($"target plane 10 deg east: wait {ahead.WaitSec:F0} s, {(ahead.Descending ? "desc" : "asc")} node, status '{refusal}'");
            t.Check("the next window is ahead", ahead.WaitSec > 600.0, ahead.WaitSec.ToString("F0"));
            t.Check("LAUNCH off the window refuses the gravity turn", !state.Running && refusal.StartsWith("REFUSED: SITE ", StringComparison.Ordinal), refusal);
            t.Check("the refusal gives the next window", refusal.Contains("NEXT WINDOW: T-", StringComparison.Ordinal) && refusal.EndsWith("LAUNCH ARMS FOR IT.", StringComparison.Ordinal), refusal);
            if (state.Running)
            {
                AbortAscent.Invoke(null, new object[] { vehicle });
                driver.Step(0.02, 2);
            }

            // NOW of the warp offer, while the countdown to that window is armed.
            SetTargetPlane(state, over + 10.0);
            state.Status = "";
            state.LaunchArmed = true;
            state.LaunchTargetTime = Universe.GetElapsedSeconds() + ahead.WaitSec;
            LaunchNowAscent.Invoke(null, new object[] { vehicle, vehicle.Orbit, home, ahead });
            t.Info($"NOW off the window: status '{state.Status}', armed {state.LaunchArmed}, running {state.Running}");
            t.Check("NOW refuses the gravity turn off the window", !state.Running && state.Status.StartsWith("REFUSED: SITE ", StringComparison.Ordinal), state.Status);
            t.Check("the countdown stays after NOW", state.LaunchArmed && !state.Running);
            if (state.Running)
            {
                AbortAscent.Invoke(null, new object[] { vehicle });
                driver.Step(0.02, 2);
            }

            // An armed launch that fires when the site is no longer under the plane, as after a warp step far past the window.
            SetTargetPlane(state, over + 10.0);
            state.Status = "";
            state.LaunchArmed = true;
            state.LaunchTargetTime = Universe.GetElapsedSeconds() - 1.0;
            StepLaunchWindow.Invoke(null, new object[] { vehicle, vehicle.Orbit, home, home.MeanRadius });
            t.Info($"armed launch off the window: status '{state.Status}', armed {state.LaunchArmed}, running {state.Running}");
            t.Check("an armed launch off the window is refused and disarmed", !state.Running && !state.LaunchArmed && state.Status.StartsWith("REFUSED: SITE ", StringComparison.Ordinal), state.Status);

            // An armed launch to the descending node that fires late, at the window of the ascending node, flies the ascending node's heading.
            SetTargetPlane(state, over);
            state.Status = "";
            state.LaunchArmed = true;
            state.LaunchDescending = true;
            state.LaunchTargetTime = Universe.GetElapsedSeconds() - 1.0;
            StepLaunchWindow.Invoke(null, new object[] { vehicle, vehicle.Orbit, home, home.MeanRadius });
            t.Info($"armed for the descending node, fired at the ascending window: status '{state.Status}', running {state.Running}, {(state.LaunchDescending ? "desc" : "asc")} node");
            t.Check("a late armed launch at the other node's window flies that node", state.Running && !state.LaunchDescending, state.Status);
            if (state.Running)
            {
                AbortAscent.Invoke(null, new object[] { vehicle });
                driver.Step(0.02, 2);
            }
            state.LaunchDescending = false;
        }
        finally
        {
            if (state.Running || state.LaunchArmed)
                AbortAscent.Invoke(null, new object[] { vehicle });
            state.PlaneTarget = null;
            state.ChaseDriven = false;
            state.FlyConvexAscent = true;
            driver.Step(0.02, 2);
        }
    }

    // A target plane inclined 0.3 deg less than the site latitude never passes over the site, so no window serves it. The gravity turn starts only when the site is at the plane's northernmost point, within 0.5 deg of it, and is refused elsewhere.
    // A retrograde body (Celestial.ComputeAngularVelocity gives a negative rate) still brings a window, and the site lies in the plane after the wait and the lead.
    private static void CheckPlaneOffSite(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver, VehicleAutopilotState state)
    {
        state.FlyConvexAscent = false;
        state.Engage = false;
        state.LaunchDescending = false;
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        double latDeg = UpfgTarget.RadToDeg(Math.Asin(r.Z / r.Length()));
        double raDeg = UpfgTarget.RadToDeg(Math.Atan2(r.Y, r.X));
        double incDeg = latDeg - 0.3;
        try
        {
            state.PlaneTarget = new AscentPlaneTarget(incDeg, raDeg - 90.0, TargetAltKm, TargetAltKm, "test plane");
            state.IncDeg = incDeg;
            state.LanDeg = raDeg - 90.0;
            state.ChaseDriven = true;
            var status = Chase(vehicle, home, out _);
            StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
            bool running = state.Running;
            t.Info($"plane at inc {incDeg:F2} deg, site at its northernmost point: chase {status}, status '{state.Status}'");
            AbortAscent.Invoke(null, new object[] { vehicle });
            driver.Step(0.02, 2);
            t.Check("a plane inclined below the site latitude has no window", status == GuidanceWindow.ChaseStatus.PlaneUnreachable, status.ToString());
            t.Check("the gravity turn flies it with the site within 0.5 deg of the plane", running, state.Status);

            state.PlaneTarget = new AscentPlaneTarget(incDeg, raDeg, TargetAltKm, TargetAltKm, "test plane");
            state.IncDeg = incDeg;
            state.LanDeg = raDeg;
            state.ChaseDriven = true;
            state.Status = "";
            StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
            t.Info($"same plane, site at its node: status '{state.Status}'");
            t.Check("the gravity turn refuses it with the site at the node", !state.Running && state.Status.StartsWith("REFUSED: SITE ", StringComparison.Ordinal)
                && state.Status.EndsWith("NEVER PASSES OVER THE SITE, SO NO WINDOW COMES.", StringComparison.Ordinal), state.Status);
        }
        finally
        {
            if (state.Running || state.LaunchArmed)
                AbortAscent.Invoke(null, new object[] { vehicle });
            state.PlaneTarget = null;
            state.ChaseDriven = false;
            state.FlyConvexAscent = true;
            driver.Step(0.02, 2);
        }

        double incR = UpfgTarget.DegToRad(51.6);
        double lanR = UpfgTarget.DegToRad(raDeg + 10.0);
        double3 n = UpfgTarget.OrbitNormal(incR, lanR);
        foreach (double omega in new[] { home.GetAngularVelocity(), -home.GetAngularVelocity() })
        {
            string turn = omega > 0.0 ? "prograde" : "retrograde";
            bool window = GuidanceWindow.TryLaunchWindow(r, omega, incR, lanR, out double wait, out bool descending, out _);
            double a = omega * (wait + 180.0);
            double3 later = new double3(r.X * Math.Cos(a) - r.Y * Math.Sin(a), r.X * Math.Sin(a) + r.Y * Math.Cos(a), r.Z);
            double off = LaunchReach.PlaneChangeDeg(n.X, n.Y, n.Z, later.X, later.Y, later.Z);
            t.Info($"{turn} body: wait {wait:F0} s, {(descending ? "desc" : "asc")} node, site {off:F4} deg off the plane after the wait and the lead");
            t.Check($"a {turn} body brings a window", window && double.IsFinite(wait) && wait > 0.0, wait.ToString("F0"));
            t.Check($"the site lies in the plane at the window, {turn} body", off < 0.05, off.ToString("F4"));
        }
    }

    // With no target, the LAN of the gravity turn follows the pad on the sim step for the node picked, and the calculated climb keeps the LAN.
    private static void CheckLanFollowsPad(TestContext t, Vehicle vehicle, IParentBody home, VehicleAutopilotState state)
    {
        try
        {
            state.PlaneTarget = null;
            state.TargetId = "";
            state.ChaseDriven = false;
            state.FlyConvexAscent = false;
            state.LaunchDescending = true;
            state.IncDeg = 51.6;
            state.LanDeg = LanOver(vehicle, home, 51.6, false) + 20.0;
            StepLaunchWindow.Invoke(null, new object[] { vehicle, vehicle.Orbit, home, home.MeanRadius });
            double desc = LanOver(vehicle, home, 51.6, true);
            t.Info($"gravity turn, DESC NODE: LAN {state.LanDeg:F3}, over the pad {desc:F3}, follows {FollowsPad(state)}");
            t.Check("the gravity turn's LAN follows the pad", FollowsPad(state));
            t.CheckAbs("the PLANE row LAN is the one LAUNCH flies at the descending node, deg", Math.IEEERemainder(state.LanDeg - desc, 360.0), 0.0, 0.01);

            state.IncDeg = 0.0;
            state.LanDeg = 123.0;
            StepLaunchWindow.Invoke(null, new object[] { vehicle, vehicle.Orbit, home, home.MeanRadius });
            t.Check("an inclination that LAUNCH refuses keeps its LAN", !FollowsPad(state) && state.LanDeg == 123.0, state.LanDeg.ToString("F3"));

            state.FlyConvexAscent = true;
            state.IncDeg = 51.6;
            state.LanDeg = 123.0;
            StepLaunchWindow.Invoke(null, new object[] { vehicle, vehicle.Orbit, home, home.MeanRadius });
            t.Check("the calculated climb keeps the LAN", !FollowsPad(state) && state.LanDeg == 123.0, state.LanDeg.ToString("F3"));
        }
        finally
        {
            state.LaunchDescending = false;
            state.FlyConvexAscent = true;
        }
    }

    // The first-stage TWR of the save on the pad, from the stage list the TWR row reads.
    private static void CheckPadTwr(TestContext t, Vehicle vehicle, IParentBody home, string save)
    {
        float pressure = vehicle.PhysicsEnvironment.AtmosphericPressure;
        vehicle.Parts.PerformanceSequences.RecomputeForFlight(pressure);
        var f = (LaunchTwrFacts)MeasurePadTwr.Invoke(null, new object[] { vehicle, home, (double)pressure })!;
        var parts = new Dictionary<UpfgStage, HashSet<Part>>();
        UpfgVehicle model = KsaVehicleAdapter.Build(vehicle, pressure, new[] { (double)pressure }, parts);
        for (int i = 0; i < model.Stages.Count; i++)
        {
            UpfgStage s = model.Stages[i];
            t.Info($"'{save}' stage {i + 1}: seq {s.Seq}, {s.Engines} engine(s), {s.Thrust / 1000.0:F0} kN, at the pad {(s.ThrustAtPressure is { Length: > 0 } p ? p[0] / 1000.0 : double.NaN):F0} kN, "
                 + $"{s.MassTotal / 1000.0:F1} -> {s.MassDry / 1000.0:F1} t, engine parts {(parts.TryGetValue(s, out HashSet<Part>? set) ? set.Count : -1)}");
        }
        t.Info($"'{save}' on the pad at {pressure:F0} Pa: lift-off TWR {f.LiftoffTwr:F3}, lowest later first-stage TWR {f.LowestTwr:F3} at stage {f.LowestStage + 1}, row '{(f.Shows ? PageText.Twr(in f) : "")}'");
        t.Check($"'{save}': the first stage has a TWR", f.Known);
        t.Check($"'{save}': the craft lifts off", f.Known && !f.CannotLiftOff, f.LiftoffTwr.ToString("F3"));
        // Test Vehicle 1's core burns on after its solid boosters drop with less thrust than weight at the pad's pressure, which the gravity turn cannot fly; Test Vehicle 2's core can.
        if (save == "Test Vehicle 1")
            t.Check($"'{save}': the row warns about the core after the boosters, stage 2", f.Shows && f.LowestStage == 1 && f.LowestTwr < 1.0, f.LowestTwr.ToString("F3"));
        else if (save == "Test Vehicle 2")
            t.Check($"'{save}': no row", !f.Shows, f.LowestTwr.ToString("F3"));
    }

    private static void Plan(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver, VehicleAutopilotState state, double incDeg, out double massToOrbit)
    {
        massToOrbit = double.NaN;
        SetPlane(state, vehicle, home, incDeg);
        state.AscentPlan = null;
        state.AscentPlanRequested = true;
        driver.Step(0.02);
        AscentPlanJob? job = state.AscentPlanJob;
        if (!t.Check($"inc {incDeg:F1}: Calculate starts a solve", job != null, $"status '{state.AscentPlanStatus}'"))
            return;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        while (!job!.IsDone && clock.Elapsed.TotalSeconds < PlanTimeoutS)
            Thread.Sleep(50);
        if (!job.IsDone)
        {
            job.Cancel();
            t.Info($"inc {incDeg:F1}: the solve did not finish within {PlanTimeoutS:F0} s");
            return;
        }
        driver.Step(0.02);
        AscentPlan? plan = state.AscentPlan;
        foreach (string note in job.Notes)
            t.Info($"inc {incDeg:F1}: attempt: {note}");
        if (plan?.Usable != true)
        {
            t.Info($"inc {incDeg:F1}: no usable plan, status '{state.AscentPlanStatus}'");
            return;
        }
        AscentSolution sol = plan.Solution;
        massToOrbit = sol.FinalMass;
        t.Info($"inc {incDeg:F1}: {sol.Message}, {plan.StagesPlanned} of {plan.StagesAvailable} stages, burns {string.Join("/", sol.BurnTime.Select(b => b.ToString("F1")))} s, "
             + $"{sol.FinalMass / 1000.0:F2} t to orbit, LAN {plan.LanDeg:F2} deg, insertion miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s");
    }
}
