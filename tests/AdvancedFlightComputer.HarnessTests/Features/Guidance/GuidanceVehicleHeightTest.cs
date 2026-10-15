using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The measured vehicle height (CoM to the lowest collider point along the thrust axis) for craft with landing legs.
//   stowed:   legs stowed measure their deployed geometry, and the craft as it is measures no taller.
//   deployed: once the game itself has animated the legs out, its live collider poses agree with the deployed pose this
//             mod reconstructs from the animation keyframes - the check that the reconstruction composes transforms
//             the way the game does.
//   retract:  stowing them again leaves the touchdown height where it was.
//   refresh:  the guidance readout puts the measurement in VehicleHeightM while auto is on and leaves it alone when off.
public sealed class GuidanceVehicleHeightTest : AfcTest
{
    private const double StepDt = 0.05;
    private const int MaxAnimationSteps = 400;

    public override string Name => "afc-guidance-vehicle-height";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves("hopper", "lunarlauncher", "2stage_controlled");
        if (saves.Count == 0)
        {
            t.Skip("no landing-leg vehicle save on this machine");
            return;
        }
        foreach (string save in saves)
            Run(t, home, save);
    }

    private static void Run(TestContext t, IParentBody home, string save)
    {
        Vehicle? craft = null;
        try
        {
            craft = VehicleSpawner.SpawnFromSave(save, t.System, home, "AfcVehicleHeight_" + Guid.NewGuid().ToString("N"),
                OrbitFixtures.CircularAt(home, 600_000, Universe.GetElapsedTime()));
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(StepDt, 2);

            SetLegs(craft, deployed: false);
            if (!Settle(driver, craft))
            {
                t.Fail($"{save}: stow", "the legs were still moving after the step budget");
                return;
            }
            if (!t.Check($"{save}: measures stowed", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result stowed)))
                return;
            t.Info($"{save}: stowed - deployed {stowed.DeployedM:F3} m, as is {stowed.CurrentM:F3} m, " +
                   $"{stowed.Colliders} colliders, {stowed.Stowed}/{stowed.Deployables} deployables stowed");
            if (stowed.Deployables == 0)
            {
                t.Info($"{save}: no deploy/retract animations, leg checks skipped");
                return;
            }
            t.Check($"{save}: stowed legs are reported", stowed.Stowed == stowed.Deployables);
            // Not strictly lower: legs on an upper stage (lunarlauncher's lander) sit above the booster's bottom either way.
            t.Check($"{save}: deployed legs reach no higher than stowed ones", stowed.DeployedM >= stowed.CurrentM - 1e-6,
                $"deployed {stowed.DeployedM:F3} m, stowed {stowed.CurrentM:F3} m");

            SetLegs(craft, deployed: true);
            if (!Settle(driver, craft))
            {
                t.Fail($"{save}: deploy", "the legs were still moving after the step budget");
                return;
            }
            if (!t.Check($"{save}: measures deployed", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result deployed)))
                return;
            t.Info($"{save}: deployed - deployed {deployed.DeployedM:F3} m, as is {deployed.CurrentM:F3} m");
            t.Check($"{save}: deployed legs are reported", deployed.Stowed == 0);
            t.CheckAbs($"{save}: the game's deployed pose matches the reconstructed one", deployed.CurrentM, deployed.DeployedM, 0.005);
            t.CheckAbs($"{save}: deploying does not move the touchdown height", deployed.DeployedM, stowed.DeployedM, 0.02);

            SetLegs(craft, deployed: false);
            Settle(driver, craft);
            if (t.Check($"{save}: measures retracted", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result retracted)))
            {
                t.CheckAbs($"{save}: retracting does not move the touchdown height", retracted.DeployedM, deployed.DeployedM, 0.02);
                t.CheckAbs($"{save}: retracted matches the first stow", retracted.CurrentM, stowed.CurrentM, 0.005);
            }

            CheckRefresh(t, craft, save, stowed.DeployedM);
        }
        finally
        {
            if (craft != null)
            {
                VehicleAutopilotState.Remove(craft);
                VehicleSpawner.Despawn(craft);
            }
        }
    }

    private static void CheckRefresh(TestContext t, Vehicle craft, string save, double expected)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        var refresh = AccessTools.Method(typeof(GuidanceWindow), "RefreshVehicleHeight");
        try
        {
            ambient.SetValue(null, state);
            state.VehicleHeightAuto = true;
            state.VehicleHeightM = 1234;
            refresh.Invoke(null, new object[] { craft });
            t.CheckAbs($"{save}: auto puts the measurement in the vehicle height", state.VehicleHeightM, expected, 0.02);
            t.Check($"{save}: the status says the legs are stowed", state.VehicleHeightStatus.StartsWith("legs stowed"), state.VehicleHeightStatus);

            state.VehicleHeightAuto = false;
            state.VehicleHeightM = 7;
            state.VehicleHeightNextMs = 0;
            refresh.Invoke(null, new object[] { craft });
            t.Check($"{save}: a manual height is left alone", state.VehicleHeightM == 7);
        }
        finally
        {
            ambient.SetValue(null, previous);
        }
    }

    private static void SetLegs(Vehicle craft, bool deployed)
    {
        foreach (KeyframeAnimationModule anim in craft.Parts.Modules.Get<KeyframeAnimationModule>())
        {
            if (anim.ShowDeployRetract && anim.Extension == null)
                anim.TimeGoal = deployed ? anim.Shared.Duration : 0f;
        }
    }

    private static bool Settle(SimDriver driver, Vehicle craft)
    {
        for (int i = 0; i < MaxAnimationSteps; i++)
        {
            if (!KeyframeAnimationModule.AnyAnimating(craft.Parts.States))
            {
                // One more step, so a module that reached its goal this step has re-applied its transforms.
                driver.Step(StepDt, 2);
                return true;
            }
            driver.Step(StepDt);
        }
        return false;
    }
}
