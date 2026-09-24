using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceTerminalBurnTest : AfcTest
{
    public override string Name => "afc-guidance-terminal-burn";

    protected override void Execute(TestContext t)
    {
        using DeorbitFixture? fixture = DeorbitFixture.Open(t);
        if (fixture == null) return;
        Vehicle craft = fixture.Vehicle;
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        var properties = AccessTools.FieldRefAccess<Vehicle, VehicleProperties>("_props");
        Situation situation = properties(craft).Situation;
        try
        {
            ambient.SetValue(null, state);
            state.Engage = state.AutoStage = true;
            state.ControlAcquired = true;
            VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
            state.LandingPhase = GuidanceWindow.LandingPhase.TerminalBrake;
            state.GfoldThrottle = 0.5;
            IParentBody parent = craft.Orbit.Parent;
            double now = Universe.GetElapsedSeconds();
            double3 up = craft.Orbit.StateVectors.PositionCci.Normalized();
            double terrain = ((Celestial)parent).GetTerrainHeightFromDirCcf(up.Transform(parent.GetCci2Ccf()));
            double3 position = up * (parent.MeanRadius + terrain + state.VehicleHeightM + 500);
            double3 rising = up * 15 + double3.Cross(parent.GetAngularVelocityCci(), position);
            Orbit orbit = Orbit.CreateFromStateCci(parent, new UniverseTime(now), position, rising, craft.Orbit.OrbitLineColor);
            AccessTools.Method(typeof(GuidanceWindow), "StepTerminalBurn")
                .Invoke(null, new object[] { craft, orbit, parent, now });
            t.Check("a stopped or rising braking burn returns to an engine-off coast",
                state.LandingPhase == GuidanceWindow.LandingPhase.TerminalCoast && state.GfoldThrottle == 0 && state.HasCommand);
            t.Check("the terminal handoff keeps the guidance claim", state.ControlAcquired
                && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);

            // Contact arrives on the same step as the coast-to-burn transition.
            state.TouchdownPrevPhase = GuidanceWindow.LandingPhase.TerminalCoast;
            state.LandingPhase = GuidanceWindow.LandingPhase.TerminalBrake;
            state.LandingTouchdownArmed = true;
            properties(craft).Situation = Situation.Landed;
            AccessTools.Method(typeof(GuidanceWindow), "StepLanding")
                .Invoke(null, new object[] { craft, orbit, parent, parent.Mu, parent.MeanRadius });
            t.Check("a terminal phase transition cannot hide touchdown", state.LandingPhase == GuidanceWindow.LandingPhase.Done
                && state.LandingCutPending && state.GfoldThrottle == 0 && !state.HasCommand);
        }
        finally
        {
            properties(craft).Situation = situation;
            VehicleControlOwnership.ReleaseAll(craft);
            VehicleAutopilotState.Remove(craft);
            ambient.SetValue(null, previous);
        }
    }
}
