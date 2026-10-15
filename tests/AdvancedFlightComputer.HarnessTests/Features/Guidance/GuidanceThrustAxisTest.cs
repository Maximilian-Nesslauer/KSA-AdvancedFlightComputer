using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The guidance thrust axis follows the flight computer's control frame: X of Tnm2Asmb while the frame is TNM, body +X in Helm.
// The live part checks that the main-thread flight computer gets the TNM frame back from the worker while an engine burns, which the helper depends on.
public sealed class GuidanceThrustAxisTest : AfcTest
{
    public override string Name => "afc-guidance-thrust-axis";

    private const double AxisTolerance = 1e-9;

    protected override void Execute(TestContext t)
    {
        using DeorbitFixture? fixture = DeorbitFixture.Open(t);
        if (fixture == null) return;
        Vehicle craft = fixture.Vehicle;
        SyntheticFrames(t, craft);
        LiveBurn(t, craft);
    }

    private static void SyntheticFrames(TestContext t, Vehicle craft)
    {
        FlightComputer fc = craft.FlightComputer;
        ControlFrame frame = fc.ActiveControlFrame;
        doubleQuat tnm2Asmb = fc.Tnm2Asmb;
        try
        {
            double trim = 5 * Math.PI / 180;
            fc.Tnm2Asmb = doubleQuat.CreateFromAxisAngle(double3.UnitZ, trim);
            doubleQuat body2Cci = KsaFrameBridge.BodyToCci(craft);
            double3 nose = KsaFrameBridge.BodyNoseCci(craft);
            // Built in two steps, asmb then CCI, so that a wrong Concatenate order in the helper shows.
            double3 expected = double3.UnitX.Transform(fc.Tnm2Asmb).Transform(body2Cci);

            fc.ActiveControlFrame = ControlFrame.Helm;
            double3 helm = KsaFrameBridge.ThrustAxisCci(craft);
            t.Check("in the Helm frame the thrust axis is body +X", (helm - nose).Length() < AxisTolerance,
                $"difference {(helm - nose).Length():E2}");

            fc.ActiveControlFrame = ControlFrame.Tnm;
            double3 tnm = KsaFrameBridge.ThrustAxisCci(craft);
            t.Check("in the TNM frame the thrust axis is X of Tnm2Asmb in CCI", (tnm - expected).Length() < AxisTolerance,
                $"difference {(tnm - expected).Length():E2}");
            double angle = Math.Acos(Math.Clamp(double3.Dot(tnm, nose), -1, 1));
            t.CheckAbs("the TNM thrust axis is off body +X by the trim angle", angle, trim, 1e-9);
        }
        finally
        {
            fc.ActiveControlFrame = frame;
            fc.Tnm2Asmb = tnm2Asmb;
        }
    }

    private static void LiveBurn(TestContext t, Vehicle craft)
    {
        if (!KsaGimbalControl.AnyEngineActive(craft.Parts.Modules.Get<GimbalController>()) || !TestSupport.AnyActiveEngineFed(craft))
        {
            t.Skip("The saved craft has no active, fed, gimballed engine to light.");
            return;
        }
        FlightComputer fc = craft.FlightComputer;
        bool rails = PhysicsBubble._forceOffRails;
        try
        {
            PhysicsBubble._forceOffRails = true;
            // The guidance modes hold a Manual burn mode and a Custom track, and with these only thrust selects TNM.
            fc.BurnMode = FlightComputerBurnMode.Manual;
            fc.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
            SimDriver driver = t.Session.CreateDriver();

            TestSupport.SetManualControlInputs(craft, 0f, engineOn: false);
            driver.Step(0.01, 2);
            t.Check("before ignition the main-thread flight computer is in the Helm frame", fc.ActiveControlFrame.IsHelm(),
                $"frame {fc.ActiveControlFrame}");

            TestSupport.SetManualControlInputs(craft, 1f, engineOn: true);
            driver.Step(0.01, 5);
            t.Check("a burning engine puts the main-thread flight computer in the TNM frame", fc.ActiveControlFrame.IsTnm(),
                $"frame {fc.ActiveControlFrame}");
            double3 burning = KsaFrameBridge.ThrustAxisCci(craft);
            double3 expected = double3.UnitX.Transform(doubleQuat.Concatenate(fc.Tnm2Asmb, KsaFrameBridge.BodyToCci(craft)));
            t.Check("while burning the thrust axis is the TNM axis", (burning - expected).Length() < AxisTolerance,
                $"difference {(burning - expected).Length():E2}");
            double trimDeg = Math.Acos(Math.Clamp(double3.Dot(burning, KsaFrameBridge.BodyNoseCci(craft)), -1, 1)) * 180 / Math.PI;
            t.Info($"trimmed thrust axis is {trimDeg:F3} deg off body +X on the saved craft.");
            // A sign or frame error shows as an angle near 90 or 180 deg, a real trim is a few degrees.
            t.Check("the trimmed thrust axis stays within a trim angle of body +X", trimDeg < 30, $"{trimDeg:F3} deg");

            TestSupport.SetManualControlInputs(craft, 0f, engineOn: false);
            driver.Step(0.01, 5);
            t.Check("with the engine off and a Manual burn mode the frame returns to Helm", fc.ActiveControlFrame.IsHelm(),
                $"frame {fc.ActiveControlFrame}, track {fc.AttitudeTrackTarget}");
            double3 off = KsaFrameBridge.ThrustAxisCci(craft);
            double3 nose = KsaFrameBridge.BodyNoseCci(craft);
            t.Check("with the engine off the thrust axis is body +X", (off - nose).Length() < AxisTolerance,
                $"difference {(off - nose).Length():E2}");
        }
        finally
        {
            TestSupport.SetManualControlInputs(craft, 0f, engineOn: false);
            PhysicsBubble._forceOffRails = rails;
        }
    }
}
