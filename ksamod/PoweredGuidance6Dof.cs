using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// "6dof" sub-tab under Landing — the frame bridge, shown live.
//
// Not guidance yet. The 6-DOF SCvx model works in a different inertial frame, a
// different body-axis convention and a different quaternion convention from KSA
// (see KsaFrameBridge). Every one of those fails SILENTLY: get a sign wrong and
// the symptom is "the controller is unstable" days later, not an exception here.
//
// So the bridge gets its own readout first. The round-trip error is the number
// that matters — it converts the live attitude into the model and back, and
// catches an axis swap, a quaternion handedness error, a transposed site frame
// and a sign flip all at once.
public static partial class PoweredGuidanceWindow
{
    private static void Draw6DofTab(Vehicle vehicle, IParentBody parent, double bodyRadius)
    {
        ImGui.TextWrapped(
            "Live conversion of this vehicle's state into the 6-DOF solver's frame. " +
            "Verification only - nothing is commanded from here.");
        ImGui.Separator();

        double3 siteCci = SiteDirCciAt(parent, 0) * (bodyRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame frame = KsaFrameBridge.BuildSiteFrame(siteCci);
        double[] x = KsaFrameBridge.ToModelState(vehicle, frame);

        // --- The check that justifies trusting the rest ---
        double errDeg = KsaFrameBridge.RoundTripErrorDeg(vehicle, frame);
        ImGui.SeparatorText("Round trip");
        if (errDeg < 1e-6)
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                $"Attitude round-trip error {errDeg:E2} deg - conventions agree.");
        else
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"ROUND-TRIP ERROR {errDeg:F6} deg - the bridge is WRONG, do not fly this.");

        // Independent sanity check the round trip cannot give you: the round trip
        // would still pass if BOTH directions shared the same wrong axis convention.
        // Comparing the model's "up" against the physical local vertical catches
        // that, because the site frame is built from real geometry.
        ImGui.SeparatorText("Attitude sanity");
        KsaFrameBridge.ModelAttitude(vehicle, frame, out double qw, out double qx, out double qy, out double qz);
        double3 thrustAxisSite = ThrustAxisInSite(qw, qx, qy, qz);
        double tiltDeg = Math.Acos(Math.Clamp(thrustAxisSite.Z, -1.0, 1.0)) * 180.0 / Math.PI;
        ImGui.Text($"Model body +Z (thrust axis) in site frame: " +
                   $"({thrustAxisSite.X,7:F4},{thrustAxisSite.Y,7:F4},{thrustAxisSite.Z,7:F4})");
        ImGui.Text($"  -> tilt from local vertical: {tiltDeg,6:F2} deg");
        ImGui.TextWrapped(
            "A vehicle standing upright on the pad should read close to (0,0,1) and " +
            "0 deg. If it reads (0,0,-1) the thrust axis is inverted; if the tilt " +
            "disagrees with the navball, the body-axis derivation is wrong.");

        // --- The state vector the solver would receive ---
        ImGui.SeparatorText("Model state  [r v q w m]");
        ImGui.Text($"r  ({x[0],10:F1},{x[1],10:F1},{x[2],10:F1}) m      (z = height above site)");
        ImGui.Text($"v  ({x[3],10:F2},{x[4],10:F2},{x[5],10:F2}) m/s    (surface-relative)");
        ImGui.Text($"q  ({x[6],9:F5},{x[7],9:F5},{x[8],9:F5},{x[9],9:F5})  scalar-first");
        ImGui.Text($"w  ({x[10],10:F4},{x[11],10:F4},{x[12],10:F4}) rad/s  (model body axes)");
        ImGui.Text($"m  {x[13],10:F0} kg");

        double qNorm = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        if (Math.Abs(qNorm - 1.0) > 1e-9)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"|q| = {qNorm:F12}, should be 1 - conversion is not a rotation.");

        // --- Where the derived body axes actually point ---
        ImGui.SeparatorText("Derived body axes");
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        ImGui.TextWrapped(
            "Model body axes in KSA body coordinates, derived from the engines' own " +
            "measured thrust direction rather than assumed:");
        ImGui.Text($"  model +X = ({mx.X,7:F4},{mx.Y,7:F4},{mx.Z,7:F4})");
        ImGui.Text($"  model +Y = ({my.X,7:F4},{my.Y,7:F4},{my.Z,7:F4})");
        ImGui.Text($"  model +Z = ({mz.X,7:F4},{mz.Y,7:F4},{mz.Z,7:F4})   <- thrust axis");

        // --- Site frame ---
        ImGui.SeparatorText("Site frame");
        ImGui.Text($"origin  lat {_siteLatDeg,9:F5}  lon {_siteLonDeg,9:F5} deg");
        ImGui.Text($"radius  {siteCci.Length() / 1000.0,10:F3} km from body centre");
        ImGui.TextWrapped(
            "+Z is local vertical, so the model's flat-ground constant-gravity " +
            "assumption is locally true. This is NOT the G-FOLD frame, which is X-up.");
    }

    // Model body +Z expressed in the site frame. Local to the tab because it is a
    // display concern; the bridge exposes the CCI version that guidance would use.
    private static double3 ThrustAxisInSite(double qw, double qx, double qy, double qz)
    {
        KsaFrameBridge.QuatToMatrix(qw, qx, qy, qz, out _, out _, out double3 c2);
        return c2;
    }
}
