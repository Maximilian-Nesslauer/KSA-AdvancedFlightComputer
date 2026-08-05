using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Scvx;

// "6dof" sub-tab under Landing — the frame bridge, and the MPC guidance built on it.
//
// The 6-DOF SCvx model works in a different inertial frame, a different body-axis
// convention and a different quaternion convention from KSA (see KsaFrameBridge).
// Every one of those fails SILENTLY: get a sign wrong and the symptom is "the
// controller is unstable" days later, not an exception here. So the bridge readout
// comes first and the round-trip error is the number that matters — it catches an
// axis swap, a quaternion handedness error, a transposed site frame and a sign flip
// all at once.
public static partial class PoweredGuidanceWindow
{
    private static Ksa6DofGuidance _sixDof;
    private static bool _sixDofActive;          // read by ApplyAutopilot
    private static bool _sixDofEngagePending;   // set by the draw, consumed by the step
    private static string _sixDofError = "";
    private static double _sixDofLastReplan;

    private static int _sixDofNodes = 30;
    private static double _sixDofTiltDeg = 30.0;
    private static double _sixDofThrottleFloor = 0.40;
    private static double _sixDofSigmaSeed = 20.0;
    private static double _sixDofTargetAltM = 20.0;
    private static double _sixDofReplanSec = 1.0;

    private static void Draw6DofTab(Vehicle vehicle, IParentBody parent, double bodyRadius)
    {
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

        // Independent sanity check the round trip cannot give: the round trip would
        // still pass if BOTH directions shared the same wrong axis convention.
        // Comparing the model's "up" against the physical local vertical catches that,
        // because the site frame is built from real geometry.
        ImGui.SeparatorText("Attitude sanity");
        KsaFrameBridge.ModelAttitude(vehicle, frame, out double qw, out double qx, out double qy, out double qz);
        double3 thrustAxisSite = ThrustAxisInSite(qw, qx, qy, qz);
        double tiltDeg = Math.Acos(Math.Clamp(thrustAxisSite.Z, -1.0, 1.0)) * 180.0 / Math.PI;
        ImGui.Text($"Model body +Z (thrust axis) in site frame: " +
                   $"({thrustAxisSite.X,7:F4},{thrustAxisSite.Y,7:F4},{thrustAxisSite.Z,7:F4})");
        ImGui.Text($"  -> tilt from local vertical: {tiltDeg,6:F2} deg");

        // --- The state vector the solver receives ---
        ImGui.SeparatorText("Model state  [r v q w m]");
        ImGui.Text($"r  ({x[0],10:F1},{x[1],10:F1},{x[2],10:F1}) m      (z = height above site)");
        ImGui.Text($"v  ({x[3],10:F2},{x[4],10:F2},{x[5],10:F2}) m/s    (surface-relative)");
        ImGui.Text($"q  ({x[6],9:F5},{x[7],9:F5},{x[8],9:F5},{x[9],9:F5})  scalar-first");
        ImGui.Text($"w  ({x[10],10:F4},{x[11],10:F4},{x[12],10:F4}) rad/s  (model body axes)");
        ImGui.Text($"m  {x[13],10:F0} kg");

        Draw6DofGuidance(vehicle, x);
    }

    // Model body +Z expressed in the site frame. Local to the tab because it is a
    // display concern; the bridge exposes the CCI version guidance would use.
    private static double3 ThrustAxisInSite(double qw, double qx, double qy, double qz)
    {
        KsaFrameBridge.QuatToMatrix(qw, qx, qy, qz, out _, out _, out double3 c2);
        return c2;
    }

    // ---- Guidance UI (draw side: sets flags only) ----

    private static void Draw6DofGuidance(Vehicle vehicle, double[] x)
    {
        ImGui.SeparatorText("Guidance (MPC)");
        ImGui.TextWrapped(
            "Re-solves from the live vehicle state each cycle and applies the " +
            "optimiser's own controls, interpolated along the fresh trajectory. " +
            "There is no trajectory tracking - the feedback IS the re-solve.");

        if (!_sixDofActive)
        {
            if (ImGui.Button("Engage 6-DOF guidance"))
                _sixDofEngagePending = true;
            ImGui.TextWrapped("Cold solve takes ~1.7 s on the sim thread - engage during a coast.");
        }
        else
        {
            if (ImGui.Button("Disengage"))
                Disengage6Dof(vehicle);
            ImGui.SameLine();
            ImGui.Checkbox("Show plan overlay", ref _show6DofOverlay);
        }

        if (_sixDofError.Length > 0)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f), _sixDofError);

        if (ImGui.CollapsingHeader("Parameters"))
        {
            ImGui.InputInt("Nodes", ref _sixDofNodes);
            ImGui.InputDouble("Tilt limit (deg)", ref _sixDofTiltDeg);
            ImGui.InputDouble("Throttle floor", ref _sixDofThrottleFloor);
            ImGui.InputDouble("Burn time seed (s)", ref _sixDofSigmaSeed);
            ImGui.InputDouble("Target altitude (m)", ref _sixDofTargetAltM);
            ImGui.InputDouble("Re-solve every (s)", ref _sixDofReplanSec);
        }

        if (_sixDof == null || !_sixDof.HasPlan)
            return;

        ImGui.SeparatorText("Plan");
        ImGui.Text($"status {_sixDof.Status}   solves {_sixDof.SolveCount}   " +
                   $"last {_sixDof.LastIterations} iters ({_sixDof.AcceptedSteps} accepted) " +
                   $"in {_sixDof.LastSolveMs:F0} ms");
        // Plan age is time since the last SUCCESSFUL solve — the plan's own clock.
        // Under a healthy MPC it sawtooths between 0 and the cadence. If it climbs
        // past that, re-solves are failing and the command is being read further and
        // further along a trajectory that is no longer being refreshed: the plan's
        // time index outruns the vehicle.
        double age = _sixDof.PlanElapsed;
        bool stale = age > _sixDofReplanSec * 2.5;
        ImGui.Text($"burn time {_sixDof.Sigma,6:F1} s");
        if (stale)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"PLAN AGE {age:F1} s - re-solves are failing, this plan is stale " +
                $"(cadence {_sixDofReplanSec:F1} s). Commands are running ahead of the vehicle.");
        else
            ImGui.Text($"plan age  {age,6:F1} s   (node spacing {_sixDof.Sigma / Math.Max(_sixDofNodes - 1, 1),5:F2} s)");

        // Node 0 is an equality constraint, so this is ~0 on any usable plan. It is
        // THE check that the MPC re-anchored at the vehicle instead of serving a
        // stale trajectory — which is what "the plan starts a node below" looked like.
        ImGui.Text($"anchor offset {_sixDof.AnchorOffsetM,8:F2} m");

        // Pure diagnostics; nothing acts on these. Under MPC, drift between re-solves
        // is expected — what matters is that it RESETS each cycle rather than growing.
        _sixDof.Diagnostics(x, out double pe, out double ve, out double ae);
        ImGui.SeparatorText("Drift since last solve");
        ImGui.Text($"position {pe,8:F1} m   velocity {ve,7:F2} m/s   attitude {ae,6:F2} deg");

        // Commanded vs delivered torque — the link the drift numbers cannot see. A gap
        // means the plan is asking for torque this vehicle does not have.
        ImGui.SeparatorText("Torque commanded vs delivered (KSA body axes)");
        TvcAllocationResult a = KsaGimbalControl.LastAllocation;
        ImGui.Text($"cmd  ({KsaGimbalControl.TorqueXNm / 1000.0,9:F1},{KsaGimbalControl.TorqueYNm / 1000.0,9:F1},{KsaGimbalControl.TorqueZNm / 1000.0,9:F1}) kN-m");
        ImGui.Text($"got  ({a.AchievedTorque.X / 1000.0,9:F1},{a.AchievedTorque.Y / 1000.0,9:F1},{a.AchievedTorque.Z / 1000.0,9:F1}) kN-m");
        ImGui.Text($"max  ({a.MaxTorque.X / 1000.0,9:F1},{a.MaxTorque.Y / 1000.0,9:F1},{a.MaxTorque.Z / 1000.0,9:F1}) kN-m");
        if (a.SaturationScale < 0.999)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f),
                $"ALLOCATOR SATURATED - delivering {a.SaturationScale * 100.0:F0}% of the demand.");
    }

    private static void Disengage6Dof(Vehicle vehicle)
    {
        _sixDofActive = false;
        _sixDofEngagePending = false;
        _sixDof = null;
        _gimbalMode = 0;
        KsaGimbalControl.Disengage();
        if (vehicle != null)
        {
            ref ManualControlInputs inputs = ref ManualInputs(vehicle);
            inputs.EngineOn = false;
            inputs.EngineThrottle = 0f;
        }
    }

    // ---- Execute (runs from the PrepareWorker prefix, never the draw) ----
    //
    // Draw-time writes to the flight computer or _manualControlInputs are erased one
    // frame later, so the tab only sets flags and every command is issued here.
    internal static void Step6Dof(Vehicle vehicle)
    {
        // Caught here rather than left to Mod's prefix handler, which logs and
        // swallows — for an engage failure that is indistinguishable from the button
        // doing nothing at all.
        try
        {
            Step6DofCore(vehicle);
        }
        catch (Exception e)
        {
            _sixDofError = "6-DOF step failed: " + e.Message;
            _sixDofEngagePending = false;
            Disengage6Dof(vehicle);
        }
    }

    private static void Step6DofCore(Vehicle vehicle)
    {
        IParentBody parent = vehicle.Orbit.Parent;
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame frame = KsaFrameBridge.BuildSiteFrame(siteCci);
        double[] x = KsaFrameBridge.ToModelState(vehicle, frame);
        double now = SimNow();

        if (_sixDofEngagePending)
        {
            _sixDofEngagePending = false;
            if (!Engage6Dof(vehicle, parent, siteCci, x, now))
                return;
        }

        if (_sixDof == null || !_sixDof.HasPlan)
            return;

        // THE MPC STEP: re-solve from the MEASURED state on a cadence. This is where
        // all the feedback in the system comes from — there is nothing else.
        //
        // The cadence timer advances ONLY ON SUCCESS. Previously it was stamped
        // before the attempt, so a failed re-solve burned the whole interval before
        // trying again while Command kept advancing the plan clock — the plan's time
        // index ran on along a trajectory that was never refreshed, which is the
        // "green dot outruns the vehicle" symptom. A failed solve now retries on the
        // next step instead of letting the clock run.
        if (now - _sixDofLastReplan >= _sixDofReplanSec)
        {
            if (_sixDof.Update(x, now))
            {
                _sixDofLastReplan = now;
                _sixDofError = "";
            }
            else
            {
                _sixDofError = "re-solve failed: " + _sixDof.Error;
            }
        }

        if (!_sixDof.Command(now, out double3 torqueModel, out double throttle))
            return;

        // Model body axes -> KSA body axes. The allocator works in KSA's frame; a
        // missed conversion here would put roll torque on the pitch axis.
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        double3 torqueKsa = torqueModel.X * mx + torqueModel.Y * my + torqueModel.Z * mz;

        KsaGimbalControl.SetTarget(vehicle);
        KsaGimbalControl.TorqueXNm = torqueKsa.X;
        KsaGimbalControl.TorqueYNm = torqueKsa.Y;
        KsaGimbalControl.TorqueZNm = torqueKsa.Z;
        KsaGimbalControl.Mode = GimbalOverrideMode.Lsq;

        ref ManualControlInputs manual = ref ManualInputs(vehicle);
        manual.EngineOn = throttle > 0.02;
        manual.EngineThrottle = (float)throttle;
    }

    private static bool Engage6Dof(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                   double[] x, double now)
    {
        if (!Ksa6DofSetup.TryBuild(vehicle, parent, siteCci, _sixDofNodes, _sixDofTiltDeg,
                                   _sixDofThrottleFloor, _sixDofSigmaSeed,
                                   out Scvx6DofConfig cfg,
                                   out Dynamics6Dof.Params dyn, out string error))
        {
            _sixDofError = "cannot plan: " + error;
            return false;
        }

        // Target: hover point above the pad, upright and at rest. Mass is free, so the
        // terminal state carries 13 of the 14 components.
        var xf = new double[14];
        xf[2] = _sixDofTargetAltM;
        xf[6] = 1.0;

        _sixDof = new Ksa6DofGuidance(cfg, dyn);
        if (!_sixDof.Plan(x, xf, _sixDofSigmaSeed, now))
        {
            _sixDofError = "cold solve failed: " + _sixDof.Error;
            _sixDof = null;
            return false;
        }

        _sixDofError = "";
        _sixDofActive = true;
        _sixDofLastReplan = now;
        return true;
    }
}
