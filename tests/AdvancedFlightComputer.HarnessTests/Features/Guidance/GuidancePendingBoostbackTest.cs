using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidancePendingBoostbackTest : AfcTest
{
    public override string Name => "afc-guidance-pending-boostback";
    private static int _engages;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> Ambient =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        var save = DefaultVehicleSaves.FindSave("Rocket");
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("fixture", "Rocket is not available");
            return;
        }
        var previous = Ambient();
        bool active = GuidanceWindow.ModActive;
        Vehicle? vehicle = null;
        var harmony = new Harmony("com.maxi.afc.harnesstests.pendingboostback");
        try
        {
            harmony.Patch(AccessTools.Method(typeof(KsaAeroSweep), nameof(KsaAeroSweep.MeshRebuilding)),
                prefix: new HarmonyMethod(typeof(GuidancePendingBoostbackTest), nameof(Rebuilding)));
            harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), "ExecuteBoostback"),
                prefix: new HarmonyMethod(typeof(GuidancePendingBoostbackTest), nameof(CountEngages)));
            GuidanceWindow.SetModActive(true);
            var driver = t.Session.CreateDriver();
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcPendingBoostback", OrbitFixtures.CircularAt(home, 500_000, driver.Elapsed));
            driver.Step(0.02, 2);
            var state = VehicleAutopilotState.For(vehicle);
            Ambient() = state;
            state.HousekeepingRetryTick = long.MaxValue;
            state.Engage = true;
            state.BoostbackEngagePending = true;
            _engages = 0;
            GuidanceWindow.ApplyAutopilot(vehicle);
            var read = GuidanceWindow.ReadMode(vehicle, state);
            t.Check("a mesh rebuild keeps the request pending", state.BoostbackEngagePending && _engages == 0);
            t.Check("the waiting request offers STOP BOOSTBACK", read.Activity.BoostbackPending
                && StopResolver.Resolve(read.Activity) == StopTarget.Boostback && read.Tone == ModeTone.Waiting
                && read.Phase.Contains("AERO MESH"));
            GuidanceWindow.StopGuidance(vehicle, StopResolver.Resolve(read.Activity));
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("abort cancels before the mesh becomes ready", !state.BoostbackEngagePending && _engages == 0);

            state.BoostbackEngagePending = true;
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("the next request waits on the same rebuild", state.BoostbackEngagePending && _engages == 0);
            state.FlyConvexAscent = false;
            state.PeKm = 200;
            state.ApKm = 200;
            AccessTools.Method(typeof(GuidanceWindow), "ExecuteAscent").Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
            t.Check("a new ascent commit cancels pending boostback", state.Running && !state.BoostbackEngagePending);
            AccessTools.Method(typeof(GuidanceWindow), "AbortAscent").Invoke(null, new object[] { vehicle });
            harmony.Unpatch(AccessTools.Method(typeof(KsaAeroSweep), nameof(KsaAeroSweep.MeshRebuilding)), HarmonyPatchType.All, harmony.Id);
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("a ready mesh starts no cancelled boostback", _engages == 0 && !state.BoostbackEngagePending);
            harmony.Unpatch(AccessTools.Method(typeof(GuidanceWindow), "ExecuteBoostback"), HarmonyPatchType.All, harmony.Id);
            CheckStockDeorbitTakeover(t, harmony);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            if (vehicle != null)
            {
                VehicleAutopilotState.Remove(vehicle);
                VehicleSpawner.Despawn(vehicle);
            }
            GuidanceWindow.SetModActive(active);
            Ambient() = previous;
        }
    }

    private static bool Rebuilding(ref bool __result)
    {
        __result = true;
        return false;
    }

    private static bool CountEngages()
    {
        _engages++;
        return false;
    }

    private static void CheckStockDeorbitTakeover(TestContext t, Harmony harmony)
    {
        using DeorbitFixture? fixture = DeorbitFixture.Open(t);
        if (fixture == null)
            return;
        Vehicle craft = fixture.Vehicle;
        var state = VehicleAutopilotState.For(craft);
        state.HousekeepingRetryTick = long.MaxValue;
        GuidanceDeorbitLifecycleTest.Seed(fixture, state);
        if (!GuidanceDeorbitLifecycleTest.Arm(t, fixture, state))
            return;

        // Hold a valid older table while making the new sweep refuse.
        IParentBody parent = craft.Orbit.Parent!;
        if (!t.Check("the airless craft has an aero table",
            KsaAeroSweep.TryBuild(craft, parent, Universe.GetElapsedSeconds(), out var aero, out string error), error))
            return;
        state.Aero = aero;
        Burn node = state.DeorbitNode;
        BurnTarget target = state.DeorbitTarget;
        FlightComputer fc = craft.FlightComputer;
        TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: true);
        var inputs = AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");
        MethodInfo resample = AccessTools.Method(typeof(GuidanceWindow), "ResampleBoostbackAero");
        harmony.Patch(AccessTools.Method(typeof(KsaAeroSweep), nameof(KsaAeroSweep.MeshRebuilding)),
            prefix: new HarmonyMethod(typeof(GuidancePendingBoostbackTest), nameof(MeshReady)));
        harmony.Patch(resample,
            prefix: new HarmonyMethod(typeof(GuidancePendingBoostbackTest), nameof(RefuseSweep)));
        state.BoostbackEngagePending = true;
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("a refused takeover is consumed and leaves the deorbit burn alone",
            !state.BoostbackEngagePending && state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitBurn
            && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), node) && ReferenceEquals(fc.Burn, target)
            && fc.BurnMode == FlightComputerBurnMode.Auto && inputs(craft).EngineOn && inputs(craft).EngineThrottle == 0.63f
            && state.BoostbackStatus.Contains("TEST SWEEP REFUSAL"));

        harmony.Unpatch(resample, HarmonyPatchType.All, harmony.Id);
        state.BoostbackEngagePending = true;
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("a ready boostback takes over the stock deorbit on the same step",
            !state.BoostbackEngagePending && state.LandingPhase == GuidanceWindow.LandingPhase.Idle
            && !state.DeorbitNodeClaimed && !fc.BurnPlan.TryGetBurn(node)
            && fc.BurnMode == FlightComputerBurnMode.Manual && state.ControlAcquired
            && VehicleControlOwnership.Holds(craft, ControlClaimant.Guidance));
    }

    private static bool MeshReady(ref bool __result)
    {
        __result = false;
        return false;
    }

    private static bool RefuseSweep()
    {
        Ambient().AeroError = "TEST SWEEP REFUSAL";
        return false;
    }
}
