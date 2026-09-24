using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceVerticalApproachTest : AfcTest
{
    public override string Name => "afc-guidance-vertical-approach";

    protected override void Execute(TestContext t)
    {
        using DeorbitFixture? fixture = DeorbitFixture.Open(t);
        if (fixture == null) return;
        VehicleAutopilotState state = VehicleAutopilotState.For(fixture.Vehicle);
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        try
        {
            ambient.SetValue(null, state);
            state.GfoldApproach = GuidanceWindow.GfoldApproach.BrakeAtGate;
            state.VehicleHeightM = 15;
            state.LandingVerticalGateM = 500;
            state.GfoldPointingDeg = 89;
            state.GfoldGlideSlopeDeg = 1;
            state.AimAltKm = 0.1;
            state.DescentRate = 20;
            state.GfoldArrivalTime = 101;
            state.GfoldAltM = 480;
            state.GfoldHoverHandoffAltM = 600;
            t.CheckAbs("horizontal braking targets a point above the landing legs", Read("GfoldSolverTargetAltM"), 515, 0.001);
            t.CheckAbs("the gate is passed sinking at the gate descent rate", Read("GfoldSolverArrivalRateMs"), 20, 0.001);
            t.Check("a high hover setting cannot bypass horizontal braking", !HoverReady());
            t.CheckAbs("UPFG uses the same minimum gate height", Read("LandingBrakeGateAltitude"), 515, 0.001);
            t.Check("a fast low approach keeps the high target", !Enter(480, 100, 50, 5) && AtGate());
            t.Check("a sideways craft cannot start powered final descent", !Enter(480, 2, 50, 80) && AtGate());
            t.Check("a craft far from the site keeps braking", !Enter(480, 2, 1000, 5) && AtGate());
            state.GfoldForceSearch = false;
            state.GfoldArrivalTime = 101;
            state.GfoldLastSolveTime = 0;
            t.Check("slow upright arrival starts final descent", Enter(480, 2, 50, 5)
                && state.GfoldApproach == GuidanceWindow.GfoldApproach.VerticalFinal);
            t.Check("hover is eligible after the vertical approach gate", HoverReady());
            t.CheckAbs("the final target puts the landing legs at the surface", Read("GfoldSolverTargetAltM"), 15, 0.001);
            t.CheckAbs("the final target arrives at rest", Read("GfoldSolverArrivalRateMs"), 0, 0.001);
            t.CheckAbs("the final thrust cone keeps the engines down", Read("GfoldActivePointingDeg"), 15, 0.001);
            t.CheckAbs("the final approach slope is at least 30 degrees", Read("GfoldActiveGlideSlopeDeg"), 30, 0.001);
            t.Check("the new surface target forces a fresh time search", state.GfoldForceSearch
                && state.GfoldArrivalTime > 101 && double.IsNegativeInfinity(state.GfoldLastSolveTime));
            t.Check("the final transition runs once", !Enter(100, 1, 2, 2));
            state.GfoldApproach = GuidanceWindow.GfoldApproach.BrakeAtGate;
            t.Check("a craft above the gate keeps braking to it", !Abandon(650) && AtGate());
            t.Check("a craft below the gate outside its limits lands on the site directly", Abandon(450)
                && state.GfoldApproach == GuidanceWindow.GfoldApproach.Direct && HoverReady() && state.GfoldForceSearch);
            t.CheckAbs("the direct target puts the landing legs at the surface", Read("GfoldSolverTargetAltM"), 15, 0.001);
            state.GfoldApproach = GuidanceWindow.GfoldApproach.BrakeAtGate;
            state.GfoldThrottle = 0.4;
            state.GfoldEngineOn = true;
            bool coast = (bool)AccessTools.Method(typeof(GuidanceWindow), "TryStartTerminalBurn")
                .Invoke(null, new object[] { fixture.Vehicle, 480.0, 2.0, 50.0, 100.0 })!;
            t.Check("the high-thrust fixture starts an engine-off terminal coast", coast
                && state.LandingPhase == GuidanceWindow.LandingPhase.TerminalCoast
                && state.GfoldThrottle == 0 && !state.GfoldEngineOn && state.GfoldPlan == null);
            state.UseSixDofLanding = true;
            state.LandingVerticalGateM = double.NaN;
            t.CheckAbs("6-DOF ignores the G-FOLD gate setting", Read("LandingBrakeGateAltitude"), 100, 0.001);
        }
        finally
        {
            VehicleAutopilotState.Remove(fixture.Vehicle);
            ambient.SetValue(null, previous);
        }

        double Read(string name) => (double)AccessTools.Property(typeof(GuidanceWindow), name).GetValue(null)!;
        bool HoverReady() => (bool)AccessTools.Property(typeof(GuidanceWindow), "GfoldReadyForHover").GetValue(null)!;
        bool AtGate() => state.GfoldApproach == GuidanceWindow.GfoldApproach.BrakeAtGate;
        bool Abandon(double altitude) =>
            (bool)AccessTools.Method(typeof(GuidanceWindow), "TryAbandonVerticalGate")
                .Invoke(null, new object[] { altitude, 100.0 })!;
        bool Enter(double altitude, double speed, double distance, double tilt) =>
            (bool)AccessTools.Method(typeof(GuidanceWindow), "TryBeginVerticalDescent")
                .Invoke(null, new object[] { altitude, speed, distance, tilt, 100.0 })!;
    }
}
