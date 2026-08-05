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

    // 50. Measured in CLOSED LOOP (Scvx.Console --mpc, zero dispersion, so any plan
    // movement is the model's own error rather than disturbance):
    //   nodes   miss   path/direct   plan jump   ADMM/cycle
    //      20   1.9 m     1.50          4.0 m       2377
    //      30   2.1 m     1.36          2.4 m       1785
    //      50   1.6 m     1.28          1.3 m       1988
    //      80   4.3 m     1.25          1.0 m        821
    //
    // The plan JUMPING between re-solves is DISCRETISATION ERROR — the gap between
    // the trapezoidal collocation and the true dynamics — and it falls monotonically
    // with node count. No weight, scale or conditioning change moved it at all. The
    // over-long curved path improves with nodes too, and the vehicle never actually
    // moves AWAY from the target (away-from-target is 0.0 m at every node count), so
    // the "loops" are a long curve, not a loop.
    //
    // 20 was the worst point on this curve on ALL THREE reported symptoms at once —
    // most plan jump, longest path, and the MOST ADMM iterations. More nodes does not
    // cost more here: the smoother problem converges in fewer ADMM iterations.
    private static int _sixDofNodes = 50;
    private static double _sixDofTiltDeg = 60.0;
    private static double _sixDofThrottleFloor = 0.40;
    private static bool _sixDofFloorAuto = true;    // track the vehicle's real minimum throttle
    private static double _sixDofSigmaSeed = 20.0;
    private static double _sixDofTargetAltM = 10.0;
    // Approach corridor and climb limit. Both OFF by default — they change the shape
    // of every trajectory, so they are opt-in rather than something that silently
    // alters a configuration that already flies. Both are soft (penalised slack) and
    // skip node 0; see Scvx6DofConfig.GlideSlopeWeight for why that is a correctness
    // requirement, not a nicety.
    private static double _sixDofGlideSlopeDeg;      // 0 = off; degrees above horizontal
    private static bool _sixDofVzEnabled;
    private static double _sixDofVzMaxMs = 0.5;

    // Hand over to the terminal hover controller for the last stretch. Default ON
    // and above the target altitude, so the solver is never asked to fly the part
    // of the trajectory it is worst at — see the handover in Step6DofCore.
    private static bool _sixDofHoverHandoff = true;
    private static double _sixDofHoverHandoffAltM = 30.0;
    // Cadence in SECONDS of wall clock. The scale-free quantity is really plan NODES
    // — node spacing is sigma/(N-1), so a fixed interval becomes an ever-larger
    // fraction of a node as sigma shrinks through the burn, drifting toward the
    // stale-warm-start cliff exactly when the vehicle is closest to the ground. But
    // seconds is what the frame budget is denominated in, and it is what you can
    // reason about while flying, so it is what the knob exposes.
    //
    // Measured (Scvx.Console --rh --cadence), worst-case solve vs advance per cycle:
    //   0.25 nd -> 63 ms | 0.5 nd -> 71 ms | 1.0 nd -> 335 ms | 3.0 nd -> 2829 ms
    // Past ~2 nodes the warm start is too stale, the tight trust region fails and it
    // thrashes. At the default 0.1 s and N=80 that bound is far away (a 20 s burn is
    // 0.25 s/node, so 0.1 s is 0.4 of a node) — but it TIGHTENS as sigma falls, so
    // the readout below reports the cadence in nodes as well to keep it visible.
    private static double _sixDofReplanSec = 0.1;
    private static double _sixDofThrustFrac = 1.0;   // share of total thrust the burn uses

    // Objective regulariser weights. Both were originally the Python test case's
    // (W_DU 0.2, W_W 1.0), where they came to 121% of the fuel term — dominating the
    // objective and, because both shrink as burn time grows, pinning sigma at its
    // upper bound. Turned down twice after flight testing. Exposed because the right
    // value depends on the vehicle and is easiest to find by flying it.
    private static double _sixDofRateDampShare = 0.002;   // rate penalty as a share of fuel
    // W_DU. Raised back to 0.05 after flight testing: unlike W_W (which was pinning
    // burn time at its upper bound and deserved cutting), THIS term has a distinct
    // job — it is the only thing keeping the control profile CONTINUOUS. Min-fuel
    // with no control-rate penalty is bang-bang, and the optimum genuinely has the
    // thrust direction jumping between nodes. Measured on a realistic start:
    //   W_DU 0.000 -> 19.1 deg thrust-direction jump node-to-node (visible kinks)
    //   W_DU 0.010 ->  5.9 deg
    //   W_DU 0.050 ->  3.9 deg
    // It also suppresses out-of-plane wandering (15.9 m -> 8.3 m) by discouraging the
    // solver from hopping between local optima. Cost is a slightly longer path.
    private static double _sixDofControlSmooth = 0.05;    // W_DU

    // Proximal conditioning. The regularisers above were also holding P together;
    // turning them down for a correct trajectory tripled the ADMM iteration count.
    // This puts the conditioning back without the bias. 0.05 keeps the answer and is
    // ~3.8x faster; higher is faster still but starts moving the solution.
    private static double _sixDofProximal = 0.05;
    private static double _sixDofOffDiag, _sixDofAsym;   // diagonal-approximation validity

    // Fixed burn time (see Ksa6DofGuidance.FixedTime). Free final time makes the
    // dynamics bilinear in (sigma, x, u) and is the root of sigma pinning, the
    // regularisers biasing the trajectory, and loitering. Gfold already works this way.
    // Default back to FREE burn time. Fixing it removed the regulariser/sigma
    // coupling exactly as predicted, but did not fix the kinks, the wandering or the
    // solve times — so it is kept as an option rather than imposed.
    private static bool _sixDofFixedTime;

    // Touchdown latch. A vehicle sitting on the pad ALREADY reports terrain contact
    // (the launch-pad collider counts), so "cut on contact" must not fire until the
    // vehicle has first been observed OFF the ground. Same trap the landing flow hit.
    private static bool _sixDofTouchdownArmed;
    private static double _sixDofLastThrottle;
    private static int _sixDofSigmaSamples = 5;

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

        // KSA knows the real floor; the 0.40 default was the Python test case's value
        // and overstating it is what makes an otherwise fine vehicle "over-powered".
        if (_sixDofFloorAuto && !_sixDofActive)
            _sixDofThrottleFloor = Ksa6DofSetup.VehicleThrottleFloor(vehicle);

        if (!_sixDofActive)
        {
            if (ImGui.Button("Engage 6-DOF guidance"))
                _sixDofEngagePending = true;
            ImGui.TextWrapped("Cold solve takes ~1.7 s on the sim thread - engage during a coast.");
            Draw6DofFeasibility(vehicle);
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
            ImGui.Checkbox("Throttle floor from vehicle", ref _sixDofFloorAuto);
            if (!_sixDofFloorAuto)
                ImGui.InputDouble("Throttle floor", ref _sixDofThrottleFloor);
            ImGui.Checkbox("Fixed burn time", ref _sixDofFixedTime);
            ImGui.InputDouble("Burn time seed (s)", ref _sixDofSigmaSeed);
            if (_sixDofFixedTime)
                ImGui.InputInt("Burn-time search samples", ref _sixDofSigmaSamples);
            ImGui.InputDouble("Target altitude (m)", ref _sixDofTargetAltM);
            ImGui.InputDouble("Glide slope (deg, 0 = off)", ref _sixDofGlideSlopeDeg);
            if (_sixDofGlideSlopeDeg > 0.0)
                ImGui.TextWrapped(
                    "Degrees above the horizontal at the target - LARGER is steeper and tighter. " +
                    "The plan respects exactly this, so leave a couple of degrees of margin: a " +
                    "trajectory that rides the boundary is one disturbance away from being outside it.");
            ImGui.Checkbox("Limit climb rate", ref _sixDofVzEnabled);
            if (_sixDofVzEnabled)
                ImGui.InputDouble("Max climb rate (m/s)", ref _sixDofVzMaxMs);
            ImGui.Checkbox("Hand off to terminal hover", ref _sixDofHoverHandoff);
            if (_sixDofHoverHandoff)
            {
                ImGui.InputDouble("Handoff altitude (m)", ref _sixDofHoverHandoffAltM);
                if (_sixDofHoverHandoffAltM <= _sixDofTargetAltM)
                    ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                        "handoff is at or below the target altitude - it will never fire, " +
                        "because the plan levels off at the target and never descends past it.");
            }
            ImGui.InputDouble("Re-solve every (s)", ref _sixDofReplanSec);
            ImGui.InputDouble("Thrust fraction", ref _sixDofThrustFrac);
            ImGui.InputDouble("Rate damping (share of fuel)", ref _sixDofRateDampShare);
            ImGui.InputDouble("Control smoothing (W_DU)", ref _sixDofControlSmooth);
            ImGui.InputDouble("Proximal conditioning", ref _sixDofProximal);
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
        double cadenceS = _sixDofReplanSec;
        bool stale = age > cadenceS * 2.5;
        double sg = _sixDof.Sigma;
        // Solver health. The occasional multi-second solve was an ITERATION cap that
        // did not bound TIME: one ADMM iteration costs ~20x more at N=80 than at
        // N=30, so a fixed 2000-iteration budget was 1.5 s there. The cap is derived
        // from this measured cost each cycle instead.
        ImGui.Text($"solver {_sixDof.MsPerAdmmIteration * 1000.0,6:F0} us/ADMM-iter   " +
                   $"budget {_sixDof.SubproblemBudgetMs:F0} ms -> cap " +
                   $"{(int)(_sixDof.SubproblemBudgetMs / Math.Max(_sixDof.MsPerAdmmIteration, 1e-4))} iters   " +
                   $"escalations {_sixDof.Escalations}");
        if (_sixDofFixedTime)
        {
        ImGui.Text($"burn time {sg,6:F1} s   FIXED (committed {_sixDof.CommittedSigma:F1} s, counting down)");
            if (_sixDof.SearchLog.Length > 0)
                ImGui.TextWrapped("search: " + _sixDof.SearchLog);
        }
        bool atMax = sg >= _sixDof.SigmaMax * 0.999, atMin = sg <= _sixDof.SigmaMin * 1.001;
        if (_sixDofFixedTime)
        {
            // sigma is pinned by construction, so the bound warnings below are moot.
        }
        else if (atMax || atMin)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f),
                $"burn time {sg:F1} s - PINNED AT {(atMax ? "MAX" : "MIN")} " +
                $"[{_sixDof.SigmaMin:F1}, {_sixDof.SigmaMax:F1}] s. The bound is dictating the " +
                "trajectory, not the physics - if it cannot hover it will loop to burn the time.");
        else
            ImGui.Text($"burn time {sg,6:F1} s   (bounds {_sixDof.SigmaMin:F1} - {_sixDof.SigmaMax:F1} s)");
        if (_sixDof.FellBack)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                "re-solve needed the WIDE TRUST REGION retry - this is what costs ~500 ms.");
        if (stale)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"PLAN AGE {age:F1} s - re-solves are failing, this plan is stale " +
                $"(cadence {cadenceS:F2} s). Commands are running ahead of the vehicle.");
        else
        {
            // Cadence in nodes is the number that decides whether the warm start is
            // still fresh, and it moves on its own as sigma shrinks even though the
            // knob is fixed — so show it, and flag the ~2-node cliff.
            double nodeDt = _sixDof.Sigma / Math.Max(_sixDofNodes - 1, 1);
            double cadenceNodes = cadenceS / Math.Max(nodeDt, 1e-6);
            ImGui.Text($"plan age  {age,6:F2} s   cadence {cadenceS,5:F2} s = " +
                       $"{cadenceNodes,4:F2} nodes   (node spacing {nodeDt,5:F2} s)");
            if (cadenceNodes > 2.0)
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    $"cadence is {cadenceNodes:F1} NODES - past ~2 the warm start is too stale " +
                    "and solves thrash. Shorten the re-solve interval.");
        }

        // Node 0 is an equality constraint, so this is ~0 on any usable plan. It is
        // THE check that the MPC re-anchored at the vehicle instead of serving a
        // stale trajectory — which is what "the plan starts a node below" looked like.
        ImGui.Text($"anchor offset {_sixDof.AnchorOffsetM,8:F2} m");

        // The physicality check. Virtual control is a SLACK variable in the dynamics
        // constraint, so an unconverged plan contains motion no force produced — it
        // cannot be flown at any thrust. Plans above tolerance are now refused, so a
        // green reading here is what makes the displayed trajectory meaningful.
        double def = _sixDof.LastDefect;
        if (def <= _sixDof.DefectTolerance)
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                $"dynamics defect {def:E2}  (tol {_sixDof.DefectTolerance:E0}) - plan is physical");
        else
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"dynamics defect {def:E2} EXCEEDS {_sixDof.DefectTolerance:E0} - " +
                "plan is NOT physically realisable and was refused.");

        // Pure diagnostics; nothing acts on these. Under MPC, drift between re-solves
        // is expected — what matters is that it RESETS each cycle rather than growing.
        _sixDof.Diagnostics(x, out double pe, out double ve, out double ae);
        ImGui.SeparatorText("Drift since last solve");
        ImGui.Text($"position {pe,8:F1} m   velocity {ve,7:F2} m/s   attitude {ae,6:F2} deg");

        // Objective breakdown. Fuel must dominate: both regularisers get CHEAPER as
        // burn time grows, so if either rivals fuel the optimiser is minimising them
        // instead and will push sigma to its upper bound.
        _sixDof.ObjectiveTerms(out double jFuel, out double jDu, out double jW);
        ImGui.SeparatorText("Objective");
        double denom = Math.Max(jFuel, 1e-12);
        ImGui.Text($"fuel            {jFuel:E3}");
        ImGui.Text($"control smooth  {jDu:E3}   ({jDu / denom * 100,5:F0}% of fuel)");
        ImGui.Text($"rate damping    {jW:E3}   ({jW / denom * 100,5:F0}% of fuel)");
        if (jDu + jW > jFuel)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f),
                "Regularisers exceed fuel - this is NOT min-fuel any more, and both " +
                "shrink as burn time grows, so sigma will run to its upper bound.");

        // Is the diagonal-inertia approximation actually valid for this vehicle?
        // For a truly axisymmetric booster both of these are ~0 and the approximation
        // is EXACT rather than approximate — and the arbitrary roll reference that
        // BodyAxes picks becomes harmless, since the transverse inertia is degenerate.
        ImGui.SeparatorText("Inertia (model body axes)");
        double3 inr = _sixDof.Inertia;
        ImGui.Text($"Ixx {inr.X:E3}   Iyy {inr.Y:E3}   Izz {inr.Z:E3} kg m2");
        bool diagOk = _sixDofOffDiag < 0.02, axiOk = _sixDofAsym < 0.05;
        ImGui.TextColored(diagOk ? new float4(0.4f, 1f, 0.5f, 1f) : new float4(1f, 0.5f, 0.3f, 1f),
            $"off-diagonal {_sixDofOffDiag * 100,6:F2}%% of diagonal" +
            (diagOk ? " - diagonal model is exact here" : " - REAL COUPLING IS BEING DISCARDED"));
        ImGui.TextColored(axiOk ? new float4(0.4f, 1f, 0.5f, 1f) : new float4(1f, 0.8f, 0.3f, 1f),
            $"transverse asymmetry {_sixDofAsym * 100,6:F2}%%" +
            (axiOk ? " - axisymmetric" : " - not axisymmetric, roll reference matters"));

        // MODEL vs REALITY on lateral force. The model couples lateral force and
        // pitch/yaw torque rigidly through one engine at LArm; the allocator makes the
        // torque with every gimbal it has and produces whatever force falls out. A
        // mismatch here means the plan's TRANSLATIONAL dynamics are wrong even when
        // attitude tracks perfectly — the vehicle gets a different sideways push than
        // was planned, drifts, and the next re-solve starts somewhere unexpected.
        // Commanded throttle, with the vehicle's own floor beside it. If the command
        // ever sits at or below the floor the engine is at its minimum and the plan
        // has no downward authority left.
        ImGui.SeparatorText("Throttle");
        ImGui.Text($"commanded {_sixDofLastThrottle * 100,5:F1} %   " +
                   $"vehicle floor {Ksa6DofSetup.VehicleThrottleFloor(vehicle) * 100,4:F1} %   engine ON while guiding");

        ImGui.SeparatorText("Lateral force: model vs allocator");
        double2 mf = _sixDof.LastLateralForce;
        TvcAllocationResult al = KsaGimbalControl.LastAllocation;
        KsaFrameBridge.BodyAxes(vehicle, out double3 bx, out double3 by, out _);
        double afx = al.AchievedForce.X * bx.X + al.AchievedForce.Y * bx.Y + al.AchievedForce.Z * bx.Z;
        double afy = al.AchievedForce.X * by.X + al.AchievedForce.Y * by.Y + al.AchievedForce.Z * by.Z;
        ImGui.Text($"model  ({mf.X / 1000.0,9:F1},{mf.Y / 1000.0,9:F1}) kN");
        ImGui.Text($"actual ({afx / 1000.0,9:F1},{afy / 1000.0,9:F1}) kN");
        double mfn = Math.Sqrt(mf.X * mf.X + mf.Y * mf.Y);
        double afn = Math.Sqrt(afx * afx + afy * afy);
        double rel = Math.Max(mfn, afn) > 1.0 ? Math.Abs(mfn - afn) / Math.Max(mfn, afn) : 0.0;
        if (rel > 0.25)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f),
                $"MISMATCH {rel * 100:F0}%% - the model's translational dynamics do not " +
                "match what the vehicle actually gets sideways.");

        // Commanded vs delivered torque — the link the drift numbers cannot see. A gap
        // means the plan is asking for torque this vehicle does not have.
        ImGui.SeparatorText("Torque commanded vs delivered (KSA body axes)");
        TvcAllocationResult a = KsaGimbalControl.LastAllocation;
        ImGui.Text($"cmd  ({KsaGimbalControl.TorqueXNm / 1000.0,9:F1},{KsaGimbalControl.TorqueYNm / 1000.0,9:F1},{KsaGimbalControl.TorqueZNm / 1000.0,9:F1}) kN-m");
        ImGui.Text($"got  ({a.AchievedTorque.X / 1000.0,9:F1},{a.AchievedTorque.Y / 1000.0,9:F1},{a.AchievedTorque.Z / 1000.0,9:F1}) kN-m");
        ImGui.Text($"max  ({a.MaxTorque.X / 1000.0,9:F1},{a.MaxTorque.Y / 1000.0,9:F1},{a.MaxTorque.Z / 1000.0,9:F1}) kN-m");
        if (a.SaturationScale < 0.999)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f),
                $"ALLOCATOR SATURATED - delivering {a.SaturationScale * 100.0:F0}%% of the demand.");
    }

    /// <param name="cutEngine">
    /// False when handing the vehicle to another controller rather than ending the
    /// flight. Cutting the engine on a handover would drop thrust for the frame
    /// between this and the next controller's first command — survivable high up,
    /// not at the handover altitude, which is exactly where it would happen.
    /// </param>
    private static void Disengage6Dof(Vehicle vehicle, bool cutEngine = true)
    {
        _sixDofActive = false;
        _sixDofEngagePending = false;
        _sixDof = null;
        _gimbalMode = 0;
        KsaGimbalControl.Disengage();
        if (vehicle != null && cutEngine)
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

        // TOUCHDOWN. Without this the guidance kept commanding the plan's terminal
        // control forever after arriving - nothing watched the ground, nothing cut
        // the engine. Armed only once the vehicle has been seen OFF the ground,
        // because a vehicle on the pad already reports contact against the pad.
        //
        // This MUST live in the step, not the draw: the draw only runs when the tab
        // is open, and writes from it are erased by the sim copy-back a frame later.
        if (!vehicle.Situation.HasAnyContact())
        {
            _sixDofTouchdownArmed = true;
        }
        else if (_sixDofTouchdownArmed)
        {
            Disengage6Dof(vehicle);
            _sixDofError = "touchdown - engine cut, 6-DOF guidance disengaged.";
            return;
        }

        // HAND OFF THE LAST STRETCH TO TERMINAL HOVER, as G-FOLD does.
        //
        // The final metres are the worst part of the trajectory for this solver and
        // the easiest for the hover PID. The horizon has collapsed to almost
        // nothing, so the plan is a handful of nodes over a second or two and the
        // trust region is the binding constraint; meanwhile the terminal state is
        // exactly what a hover controller is built for — near-zero velocity,
        // upright, holding a point. Optimising a descent is the wrong question by
        // then.
        //
        // Handing over ABOVE the target altitude on purpose: the 6-DOF plan aims at
        // _sixDofTargetAltM and this fires on the way down to it. A handover set
        // below the target would simply never trigger, which the UI warns about.
        if (_sixDofHoverHandoff && x[2] <= _sixDofHoverHandoffAltM)
        {
            Disengage6Dof(vehicle, cutEngine: false);
            StartTerminalHover();
            _landingStatus = $"6-DOF handoff to terminal hover at {x[2]:F0} m.";
            _sixDofError = _landingStatus;
            return;
        }

        // THE MPC STEP: re-solve from the MEASURED state on a cadence. This is where
        // all the feedback in the system comes from — there is nothing else.
        //
        // The cadence timer advances ONLY ON SUCCESS. Previously it was stamped
        // before the attempt, so a failed re-solve burned the whole interval before
        // trying again while Command kept advancing the plan clock — the plan's time
        // index ran on along a trajectory that was never refreshed, which is the
        // "green dot outruns the vehicle" symptom. A failed solve now retries on the
        // next step instead of letting the clock run.
        double cadence = Math.Clamp(_sixDofReplanSec, 0.02, 5.0);
        // Refresh inertia from the live vehicle before re-solving. It changes as
        // propellant drains, and a stale value is a SYSTEMATIC torque error that MPC
        // structurally cannot correct — re-anchoring the state does not fix the model.
        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz,
                             out _sixDofOffDiag, out _sixDofAsym);
        _sixDof.SetInertia(ixx, iyy, izz);

        if (now - _sixDofLastReplan >= cadence)
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

        // Past the end of the plan the node index clamps, so the terminal control is
        // held indefinitely. That is a reasonable hold-in-place fallback, but it is
        // NOT guidance any more and must not look like it.
        if (_sixDof.PlanElapsed > _sixDof.Sigma)
            _sixDofError = $"plan expired {_sixDof.PlanElapsed - _sixDof.Sigma:F1} s ago - " +
                           "holding the terminal command, no longer guiding.";

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
        // ENGINE STAYS LIT WHILE GUIDING. This was `throttle > 0.02`, copied from the
        // G-FOLD path where it is correct — that planner has NO thrust floor and plans
        // genuine coasts, so a near-zero command means "actually stop burning".
        //
        // The 6-DOF model is the opposite: its throttle box guarantees T >= Tmin > 0,
        // so it NEVER asks for a coast. And KSA clamps a vehicle's MinimumThrottle as
        // low as 0.01, so with the floor now read from the vehicle a perfectly normal
        // min-fuel opening command of 1% fell below the 2% threshold and shut the
        // engine down. That loses THRUST AND TORQUE together — gimbals have no
        // authority without thrust — so the vehicle went into free fall with no
        // attitude control on the very first step.
        manual.EngineOn = true;
        manual.EngineThrottle = (float)throttle;
        _sixDofLastThrottle = throttle;
    }

    private static bool Engage6Dof(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                   double[] x, double now)
    {
        // Target: hover point above the pad, upright and at rest. Mass is free, so the
        // terminal state carries 13 of the 14 components. Built BEFORE the config
        // because the problem scaling is sized from the x0 -> xf extent.
        var xf = new double[14];
        xf[2] = _sixDofTargetAltM;
        xf[6] = 1.0;

        if (!Ksa6DofSetup.TryBuild(vehicle, parent, siteCci, _sixDofNodes, _sixDofTiltDeg,
                                   _sixDofThrottleFloor, _sixDofSigmaSeed, _sixDofThrustFrac,
                                   _sixDofRateDampShare, _sixDofControlSmooth,
                                   _sixDofProximal,
                                   _sixDofGlideSlopeDeg, _sixDofVzEnabled ? _sixDofVzMaxMs : -1.0,
                                   x, xf,
                                   out Scvx6DofConfig cfg,
                                   out Dynamics6Dof.Params dyn, out string error))
        {
            _sixDofError = "cannot plan: " + error;
            return false;
        }

        _sixDof = new Ksa6DofGuidance(cfg, dyn) { FixedTime = _sixDofFixedTime };

        // Fixed time needs a burn time CHOSEN, so search for it the way Gfold does
        // rather than trusting the seed. Each sample warm-starts from the last, so
        // the sweep is much cheaper than N independent cold solves.
        bool ok = _sixDofFixedTime
            ? _sixDof.PlanSearch(x, xf, _sixDofSigmaSeed, now, Math.Clamp(_sixDofSigmaSamples, 1, 12))
            : _sixDof.Plan(x, xf, _sixDofSigmaSeed, now);
        if (!ok)
        {
            _sixDofError = "cold solve failed: " + _sixDof.Error;
            _sixDof = null;
            return false;
        }

        _sixDofError = "";
        _sixDofActive = true;
        _sixDofTouchdownArmed = false;
        _sixDofLastReplan = now;
        return true;
    }

    // Whether a plan can exist AT ALL, shown before engaging rather than after a
    // failed solve. Thrust-to-weight at MINIMUM throttle is the number that decides
    // it: the model's throttle box means the engine cannot go below Tmin while lit,
    // so if Tmin exceeds weight the vehicle must tilt to shed the excess, and beyond
    // the tilt limit no descent exists. That single ratio explains infeasible solves,
    // spiral trajectories, and the long solve times that come from retrying them.
    private static void Draw6DofFeasibility(Vehicle vehicle)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null)
            return;

        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));

        // Must use the SAME pressure-corrected thrust the planner does. This panel
        // reading vacuum thrust while the plan used something else is precisely how
        // the atmospheric shortfall stayed hidden: "max TWR" looked healthy because
        // it was quoting a number the vehicle could only reach in space.
        double ambientPa = KsaEnginePerf.AmbientPressureAt(
            parent, siteCci.Length() - parent.MeanRadius + _sixDofTargetAltM);
        (double thrust, _) = KsaEnginePerf.AtPressure(vehicle, ambientPa);
        if (thrust <= 0.0)
            return;
        thrust *= Math.Clamp(_sixDofThrustFrac, 0.01, 1.0);

        double g = parent.Mu / (siteCci.Length() * siteCci.Length());
        double mass = vehicle.TotalMass;
        Ksa6DofSetup.ThrottleMargin(thrust, _sixDofThrottleFloor, mass, g,
                                    out double twrMin, out double needTilt);

        ImGui.SeparatorText("Feasibility");
        ImGui.Text($"thrust used {thrust / 1e6,6:F2} MN   weight {mass * g / 1e6,6:F2} MN   " +
                   $"max TWR {thrust / (mass * g),5:F2}");

        // The air, and what it costs. On an airless world this reads 0 Pa / 100%
        // and the whole line is a no-op; anywhere else it is the number that
        // decides whether the plan is flyable.
        double vac = KsaEnginePerf.VacuumThrust(vehicle) * Math.Clamp(_sixDofThrustFrac, 0.01, 1.0);
        double frac = vac > 0.0 ? thrust / vac : 1.0;
        ImGui.Text($"ambient {ambientPa / 1000.0,6:F1} kPa (at target alt)   " +
                   $"thrust {frac * 100.0,5:F1} % of vacuum {vac / 1e6:F2} MN");
        if (frac < 0.97)
            ImGui.TextColored(new float4(0.6f, 0.85f, 1f, 1f),
                $"planning against sea-level performance - vacuum thrust would over-promise by " +
                $"{(1.0 / Math.Max(frac, 1e-6) - 1.0) * 100.0:F0} %%.");
        ImGui.Text($"TWR at min throttle {twrMin,5:F2}   " +
                   $"(floor {_sixDofThrottleFloor:F2}, vehicle can do {Ksa6DofSetup.VehicleThrottleFloor(vehicle):F2})");

        if (needTilt >= _sixDofTiltDeg)
        {
            double feasibleFloor = Math.Cos(_sixDofTiltDeg * Math.PI / 180.0) * mass * g / thrust;
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"OVER-POWERED - needs {needTilt:F0} deg tilt just to stop climbing " +
                $"(limit {_sixDofTiltDeg:F0}). No descent exists.");
            ImGui.TextWrapped(
                $"Fix: throttle floor below {feasibleFloor:F2}, or thrust fraction " +
                $"~{feasibleFloor / Math.Max(_sixDofThrottleFloor, 1e-6):F2} to plan on fewer engines.");
        }
        else if (twrMin > 1.0)
        {
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                $"Min thrust exceeds weight - needs {needTilt:F0} deg of tilt to hold " +
                "altitude, so expect the path to curve rather than descend straight.");
        }
        else
        {
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                "Can hover at minimum throttle - a straight descent is available.");
        }
    }
}
