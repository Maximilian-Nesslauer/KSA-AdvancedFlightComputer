using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Checks the claim table, RCS refusal and release after cancellation.
public sealed class VehicleControlOwnershipTest : AfcTest
{
    public override string Name => "afc-control-ownership";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no RCS test vehicle save present.");
            return;
        }

        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        TheClaimIsExclusivePerVehicle(t, home, saves[0]);
        AnRcsBurnRefusesAHeldCraft(t, home, saves[0]);
        TheExecutionGivesItsClaimBack(t, home, saves[0]);
        ARestoredExecutionTakesTheClaim(t, home, saves[0]);
        AHeldCraftGetsNoRcsStep(t, home, saves[0]);
        TheClaimRecordsItsId(t, home, saves[0]);
        AFailedHandBackKeepsTheClaim(t, home, saves[0]);
    }

    private static void TheClaimIsExclusivePerVehicle(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaim",
            (vehicle, driver) =>
            {
                try
                {
                    t.Check("an untouched craft is unclaimed",
                        VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None);

                    t.Check("the first claimant gets the craft",
                        VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _)
                        && VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance));

                    t.Check("the holder can claim again",
                        VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _));

                    bool refused = !VehicleControlOwnership.TryClaim(
                        vehicle, ControlClaimant.RcsTranslation, out ControlClaimant holder);
                    t.Check("a second claimant is refused and told who holds it",
                        refused && holder == ControlClaimant.Guidance);

                    VehicleControlOwnership.Release(vehicle, ControlClaimant.RcsTranslation);
                    t.Check("a release by someone else changes nothing",
                        VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance));

                    VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
                    t.Check("the holder can give the craft back",
                        VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None);
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    private static void AnRcsBurnRefusesAHeldCraft(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaimRefusal",
            (vehicle, driver) =>
            {
                try
                {
                    if (!Arm(t, vehicle, driver))
                        return;
                    VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _);

                    RcsExecutor.Activate(vehicle);
                    bool active = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) && exec.IsActive;
                    t.Check("an RCS burn does not engage on a craft guidance holds", !active);
                    t.Check("the refusal leaves the claim where it was",
                        VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance));
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    private static void TheExecutionGivesItsClaimBack(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaimRelease",
            (vehicle, driver) =>
            {
                try
                {
                    if (!Arm(t, vehicle, driver))
                        return;

                    RcsExecutor.Activate(vehicle);
                    if (!t.Check("the RCS burn engages on an unclaimed craft",
                            VehicleControlOwnership.Holds(vehicle, ControlClaimant.RcsTranslation)))
                        return;

                    RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? running);
                    RcsExecutor.Cancel(vehicle, running!, "harness claim test");
                    driver.Step(0.05, 20);
                    t.Check("the claim comes back when the execution ends",
                        VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None);
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    // Models an active execution without a claim, but does not load a save.
    private static void ARestoredExecutionTakesTheClaim(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaimRestore",
            (vehicle, driver) =>
            {
                try
                {
                    if (!Arm(t, vehicle, driver))
                        return;
                    RcsExecutor.Activate(vehicle);
                    if (!t.Check("the burn engages before the claim is dropped",
                            VehicleControlOwnership.Holds(vehicle, ControlClaimant.RcsTranslation)))
                        return;

                    VehicleControlOwnership.ReleaseAll(vehicle);
                    RcsDriverPatch.TickVehicle(vehicle);
                    t.Check("a running execution takes the claim back on the next step",
                        VehicleControlOwnership.Holds(vehicle, ControlClaimant.RcsTranslation));
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    private static void AHeldCraftGetsNoRcsStep(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaimSkip",
            (vehicle, driver) =>
            {
                try
                {
                    if (!Arm(t, vehicle, driver))
                        return;
                    RcsExecutor.Activate(vehicle);
                    if (!t.Check("the burn engages before the craft changes hands",
                            VehicleControlOwnership.Holds(vehicle, ControlClaimant.RcsTranslation)))
                        return;

                    VehicleControlOwnership.ReleaseAll(vehicle);
                    VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _);
                    RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);

                    RcsDriverPatch.TickVehicle(vehicle);
                    t.Check("a held craft gets no command from the RCS step",
                        !RcsCommandChannel.TryGet(vehicle.FlightComputer.BurnPlan, out _));
                    t.Check("the RCS step leaves the other claim alone",
                        VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance));
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    // Checks only the recorded ID because the rename fixture cannot be removed reliably from the shared session.
    private static void TheClaimRecordsItsId(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaimId",
            (vehicle, driver) =>
            {
                try
                {
                    VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.RcsTranslation, out _);
                    t.Check("a claim records the id it was taken under",
                        VehicleControlOwnership.ClaimedId(vehicle) == vehicle.Id);
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    // Force cleanup to fail and check that the 6-DOF exit retains ownership.
    private static void AFailedHandBackKeepsTheClaim(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessClaimHandback",
            (vehicle, driver) =>
            {
                FieldInfo ambient = typeof(GuidanceWindow).GetField(
                    "_s", BindingFlags.NonPublic | BindingFlags.Static)!;
                object? previousAmbient = ambient.GetValue(null);
                Harmony harmony = new("com.maxi.afc.harnesstests.ownership.handback");
                harmony.Patch(Guidance("Step6Dof"), prefix: Prefix(nameof(EndTheStep)));
                harmony.Patch(Guidance("HandBackVehicle"), prefix: Prefix(nameof(FailTheHandBack)));
                try
                {
                    _state = VehicleAutopilotState.For(vehicle);
                    _state.EngagePending = true;

                    GuidanceWindow.ApplyAutopilot(vehicle);
                    t.Check("a pending 6-DOF start retains its claim after failed cleanup",
                        VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.Guidance);
                    t.Check("a failed hand-back keeps the craft",
                        VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance));

                    bool refused = !VehicleControlOwnership.TryClaim(
                        vehicle, ControlClaimant.RcsTranslation, out _);
                    t.Check("no other feature can take it while cleanup is pending", refused);
                }
                finally
                {
                    harmony.UnpatchAll(harmony.Id);
                    // ApplyAutopilot rebinds the ambient state, and the state object goes away
                    // below, so the sweep gets its previous one back.
                    ambient.SetValue(null, previousAmbient);
                    _state = null;
                    VehicleControlOwnership.ReleaseAll(vehicle);
                    VehicleAutopilotState.Remove(vehicle);
                }
            });
    }

    private static VehicleAutopilotState? _state;

    private static MethodInfo Guidance(string name) =>
        typeof(GuidanceWindow).GetMethod(name, BindingFlags.NonPublic | BindingFlags.Static)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static HarmonyMethod Prefix(string name) => new(typeof(VehicleControlOwnershipTest), name);

    // Ends the mode inside the step, so the exit below it runs in this same call.
    private static bool EndTheStep()
    {
        if (_state != null)
        {
            _state.Active = false;
            _state.EngagePending = false;
        }
        return false;
    }

    private static bool FailTheHandBack(ref bool __result)
    {
        __result = false;
        return false;
    }

    private static bool Arm(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        fc.BurnMode = FlightComputerBurnMode.Manual;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
        driver.Step(0.05, 40);
        if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
        {
            t.Skip("save has no active RCS translation capability.");
            return false;
        }
        if (RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 30.0) == null)
        {
            t.Fail("claim setup", "no patch or loaded burn target");
            return false;
        }
        return true;
    }
}
