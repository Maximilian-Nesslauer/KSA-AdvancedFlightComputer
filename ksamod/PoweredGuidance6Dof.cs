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
    // Approach corridor and climb limit. Both are SOFT — they start at node 1 and
    // carry a penalised slack — which is what makes these otherwise aggressive
    // defaults safe; see Scvx6DofConfig.GlideSlopeWeight for why that is a
    // correctness requirement and not a nicety. 10 degrees above the horizontal is a
    // shallow corridor that mainly stops the trajectory going wide, rather than a
    // tight one the plan would have to ride.
    private static double _sixDofGlideSlopeDeg = 10.0;   // 0 = off; degrees above horizontal
    private static bool _sixDofVzEnabled = true;
    // Zero, i.e. "never climb". Safe as a hard-looking number only because the
    // constraint is SOFT: it starts at node 1 and carries a penalised slack, so a
    // vehicle that is already climbing gets an expensive plan rather than no plan.
    private static double _sixDofVzMaxMs;

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
    // Telemetry, default ON. The whole point is to catch a bad run without having to
    // reproduce it, so it needs to already be recording when one happens.
    private static bool _sixDofLogging = true;
    private static double _sixDofLastThrottle;
    // Thrust bookkeeping for the readout: what the optimiser asked for against what
    // the vehicle can actually produce right now.
    private static double _sixDofDemandN, _sixDofCapabilityN;
    private static bool _sixDofThrustSaturated;
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

        // Telemetry. Cheap enough to leave on: rows buffer in memory and flush on an
        // interval, so the sim thread never waits on the disk.
        ImGui.Checkbox("Log telemetry to file", ref _sixDofLogging);
        if (SixDofLog.Enabled)
        {
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                $"logging {SixDofLog.RunName} - {SixDofLog.RowsWritten} rows");
            ImGui.Text(SixDofLog.Directory);
            if (ImGui.Button("Flush log now"))
                SixDofLog.Flush();
        }
        else if (_sixDofLogging)
            ImGui.Text("logging will start when guidance engages");
        if (SixDofLog.LastError.Length > 0)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f), "log error: " + SixDofLog.LastError);
        }

        if (_sixDofError.Length > 0)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f), _sixDofError);

        if (ImGui.CollapsingHeader("Parameters"))
        {
            ImGui.InputInt("Nodes", ref _sixDofNodes);
            _sixDofNodes = Math.Clamp(_sixDofNodes, MinNodes, MaxNodes);
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
            ImGui.Checkbox("Reduce nodes on approach", ref _sixDofNodeGates);
            if (_sixDofNodeGates)
                ImGui.TextWrapped(
                    "Node count is derived from the target SPACING below, not from " +
                    "altitude: collocation error depends on sigma/(N-1), so as burn time " +
                    "counts down the count follows. Snapped to rungs (50/40/30/25/20/15/" +
                    "10/5) and one-way, because each change rebuilds the solver and loses " +
                    "the ADMM warm start; the plan itself carries across by resampling.");
            if (_sixDofNodeGates)
            {
                ImGui.InputDouble("Target node spacing (s)", ref _sixDofNodeDtTarget);
                if (_sixDof != null && _sixDof.HasPlan)
                    ImGui.Text($"  sigma {_sixDof.Sigma,5:F1} s / {_sixDof.Nodes} nodes = " +
                               $"{_sixDof.Sigma / Math.Max(_sixDof.Nodes - 1, 1),4:F2} s actual");
            }
            ImGui.Checkbox("Spread cold solve over frames", ref _sixDofSpreadCold);
            if (_sixDofSpreadCold)
            {
                ImGui.InputDouble("Gap between iterations (s)", ref _sixDofColdIntervalS, 0.05, 0.1, "%.2f");
                _sixDofColdIntervalS = Math.Clamp(_sixDofColdIntervalS, 0.0, 1.0);
                ImGui.TextWrapped(
                    "Runs the cold solve one SCvx iteration at a time instead of blocking for " +
                    "the whole thing, waiting this long between iterations so they land as " +
                    "separate short hitches rather than merging into one visible freeze. At " +
                    "0.25 s the solve takes about a second in total - longer than blocking, " +
                    "but spread. The vehicle keeps falling meanwhile, which is absorbed " +
                    "because every iteration re-anchors at the measured state.");
            }
            ImGui.Checkbox("Estimate unmodelled acceleration", ref _sixDofBiasEnabled);
            if (_sixDofBiasEnabled)
                ImGui.TextWrapped(
                    "Measures what the model is missing - gravity error, thrust calibration, " +
                    "drag - and adds it to the planner's gravity, so the optimiser plans " +
                    "around it. Plain MPC cannot: it corrects the STATE each cycle but keeps " +
                    "planning with the same wrong model, so it meets the same error every time.");
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

        double3 bias = _sixDof.AccelBias;
        double biasMag = Math.Sqrt(bias.X * bias.X + bias.Y * bias.Y + bias.Z * bias.Z);
        if (_sixDofBiasEnabled)
        {
            double3 g0 = _sixDof.BaseGravity;
            ImGui.Text($"unmodelled accel ({bias.X,6:F2},{bias.Y,6:F2},{bias.Z,6:F2}) m/s2   " +
                       $"|{biasMag,5:F2}|   (model g {-g0.Z:F2} -> {-(g0.Z + bias.Z):F2})");
            if (biasMag > 0.15 * Math.Abs(g0.Z))
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    $"model is off by {biasMag / Math.Abs(g0.Z) * 100.0:F0}%% of gravity - " +
                    "being corrected, but worth knowing where it comes from.");
        }

        // DEMAND vs CAPABILITY. The throttle is now demand/capability using KSA's own
        // live figure, so these two being close is what "the thrust we command is the
        // thrust we get" looks like. A capability far from the plan's Tmax is not an
        // error - the plan is deliberately built against the conservative
        // target-altitude figure - but the ratio is the number that used to be
        // silently wrong, so it is on screen.
        if (_sixDofCapabilityN > 1.0)
        {
            ImGui.Text($"thrust demand {_sixDofDemandN / 1e6,6:F2} MN   " +
                       $"live capability {_sixDofCapabilityN / 1e6,6:F2} MN   " +
                       $"(plan Tmax {_sixDof.Tmax / 1e6:F2} MN)   " +
                       $"throttle {_sixDofLastThrottle * 100.0,3:F0} %");
            if (_sixDofThrustSaturated)
                ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                    "THRUST SATURATED - the plan is asking for more than the vehicle has, " +
                    "so the trajectory being flown is not the one that was planned.");
        }
        if (_sixDofNodeGates)
            ImGui.Text($"nodes {_sixDof.Nodes,5}   gate steps {_sixDofGateChanges}   " +
                       $"cold restarts {_sixDofRecoveries}");

        // THE health number. Under a working MPC this is 0 almost always: a refusal
        // means the vehicle is flying the PREVIOUS plan, so a sustained run of them is
        // open-loop flight however healthy everything else looks.
        if (_sixDofRefusalRun > 0)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f),
                $"{_sixDofRefusalRun} consecutive re-solves REFUSED - flying a stale plan. " +
                $"Cold restart at {RefusalsBeforeRestart}.");

        // The physicality check. Virtual control is a SLACK variable in the dynamics
        // constraint, so an unconverged plan contains motion no force produced — it
        // cannot be flown at any thrust. Plans above tolerance are now refused, so a
        // green reading here is what makes the displayed trajectory meaningful.
        //
        // Reported in METRES, which is also how it is now judged. The scaled figure
        // is normalised by the range to the target, so it climbs on an approach even
        // when nothing about the plan changed - it was rejecting centimetre-accurate
        // trajectories inside 100 m. See Ksa6DofGuidance.Finish.
        double defM = _sixDof.LastDefectM;
        if (defM <= _sixDof.MaxDefectM)
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                $"dynamics defect {defM:F2} m  (limit {_sixDof.MaxDefectM:F2} m) - plan is physical");
        else
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"dynamics defect {defM:F2} m EXCEEDS {_sixDof.MaxDefectM:F2} m - plan refused. " +
                "Almost always too few nodes: the collocation error grows with node spacing.");

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
    /// <summary>
    /// Altitude gates and the node count to run below each, highest first. Read as
    /// "below 1000 m, 50 nodes". Above the first gate the configured count applies,
    /// clamped to <see cref="MaxNodes"/>.
    ///
    /// Ten nodes per step, 50 down to 10 between 1000 m and 100 m. The point is to
    /// hold node SPACING roughly constant rather than any particular count: spacing
    /// is sigma/(N-1), and sigma shrinks as the vehicle closes in, so the same count
    /// buys steadily finer resolution than the problem needs. The bands tighten
    /// toward the ground for the same reason - the last 150 m go by far quicker than
    /// the first 250.
    ///
    /// The cost of a step is a solver rebuild and the loss of the ADMM warm start
    /// (the sparsity pattern is frozen at the node count), which is why this is a
    /// ladder and not a continuously tracked value. The reference TRAJECTORY carries
    /// across by resampling, and that is the seed that actually matters.
    /// </summary>
    /// <summary>
    /// Node counts the ladder is allowed to take, coarsest last. Quantised rather
    /// than continuous because every change of N rebuilds the solver and throws away
    /// the ADMM warm start - a continuously tracked count would pay that every cycle.
    /// </summary>
    private static readonly int[] NodeRungs = [50, 40, 30, 25, 20, 15, 10, 5];

    /// <summary>
    /// Target node SPACING, seconds. This is the quantity the ladder is really
    /// derived from.
    ///
    /// Collocation error depends on spacing, not on node count, and spacing is
    /// sigma/(N-1). Measured across a descent (Scvx.Console --nodes), pooling every
    /// altitude and node count together:
    ///
    ///     dt &lt; 2 s     defect &lt;= 0.09 m
    ///     dt 2 - 4 s   median 0.12 m
    ///     dt 4 - 8 s   median 0.36 m, worst 0.77 m
    ///     dt &gt; 8 s     median 0.57 m, worst 0.90 m
    ///
    /// A dt band predicts defect far better than a node count does, because two
    /// vehicles at the same altitude with different burn times need different counts
    /// for the same accuracy. Targeting 1.5 s leaves roughly a 10x margin on the 1 m
    /// flight gate.
    ///
    /// 1.5 s looked safe on that evidence and was NOT, in flight. The sweep advances
    /// the state ALONG the plan, so it measures collocation error with no tracking
    /// error present; a real descent has both, and their combination is what the gate
    /// sees. Flight log 20260806-142450 stepped to 5 nodes at 261 m with a nominal dt
    /// of 1.48 s, and the defect immediately blew past 390 m and refused every
    /// subsequent re-solve - leaving the vehicle open loop.
    ///
    /// 0.85 s is calibrated instead against the altitude ladder that DID work,
    /// swept against the burn times actually flown in that log. It reproduces the old
    /// gates almost exactly - 20 nodes at 1000 m, 15 at 500, 10 at 100-350 - and its
    /// worst disagreement anywhere is one rung. The one deliberate difference is at
    /// the bottom: where the old ladder dropped to 5 it now holds 10, because 5 is
    /// the rung that failed.
    ///
    /// The floor matters more than the ceiling here. Too many nodes costs solve time,
    /// which the wall-clock budget already bounds; too few costs the PLAN, and a
    /// refused plan means flying open loop.
    /// </summary>
    private static double _sixDofNodeDtTarget = 0.85;

    /// <summary>
    /// Hard bounds on node count, ladder or configured.
    ///
    /// The floor is 5 because the ladder now goes there. That is AGGRESSIVE: the
    /// binding constraint at low node counts is not accuracy in the abstract but the
    /// defect gate in Finish - collocation error grows with node SPACING, and a plan
    /// that trips the gate is refused, which leaves the vehicle flying a stale
    /// open-loop trajectory. Run `Scvx.Console --nodes` for the measured defect and
    /// solve time against altitude before changing the ladder.
    /// </summary>
    private const int MinNodes = 5;
    private const int MaxNodes = 50;

    // Consecutive refused re-solves since the last node step, and the coarsest rung
    // index the ladder may still move to. A refused plan means the vehicle is flying
    // the previous one OPEN LOOP, so a step that causes them has to be undone -
    // one-way stepping on its own cannot recover from a rung that does not work.
    private static int _sixDofRefusalRun;
    private static int _sixDofRungFloor = int.MaxValue;
    // Rung index a back-off has already been attempted from, so a failed attempt is
    // not repeated. -1 means none tried yet.
    private static int _sixDofBackedOffTo = -1;
    private static int _sixDofRecoveries;

    // Refusals tolerated before undoing a node step. Long enough that one awkward
    // cycle does not trigger it, short enough that the vehicle is not open loop for
    // long: at a 0.1 s cadence this is half a second.
    private const int RefusalsBeforeBackOff = 5;

    // Refusals after which the plan is abandoned and cold-started again. Well past the
    // back-off threshold, so the cheaper remedy is tried first, but far short of the
    // ~100 consecutive refusals seen in flight log 20260807-093052.
    private const int RefusalsBeforeRestart = 15;

    // Drift past which the warm seed has stopped being worth having and a cold restart
    // is the cheaper option. Taken as the larger of this and 5% of altitude, so it
    // scales with how much room there is: 20 m matters at 100 m and is noise at 2 km.
    private const double ColdRestartDriftM = 20.0;

    private static bool _sixDofNodeGates = true;
    // Seed the cold solve from a convex 3-DOF G-FOLD solve.
    //
    // OFF, because it was MEASURED not to help (Scvx.Console --seed). The idea is
    // sound in general - SCvx refines a reference rather than searching for one - but
    // there is almost no headroom to win here: at the node counts now flown the
    // straight-line seed already converges in 6 to 8 SCvx iterations and 130-260 ms
    // total, so a seed costing 300-1000 ms to compute cannot pay for itself, and on
    // several cases it made the first subproblem fail outright.
    //
    // Kept behind a flag with its test, because it should be revisited if cold-solve
    // cost ever becomes the bottleneck again - at higher node counts, or from a much
    // worse initial state, the balance would be different.
    private static bool _sixDofGfoldSeed;

    // Spread the cold solve over frames instead of blocking on it.
    //
    // A blocking cold solve costs 130-280 ms in the sim thread's frame, which is a
    // visible hitch. SCvx is iterative and the solver keeps its state between calls,
    // so the same work can be done one iteration per frame. Measured, a flyable plan
    // arrives in about three frames and the vehicle falls 1-6 m in the meantime -
    // absorbed automatically, because each frame re-anchors at the measured state
    // exactly as the warm loop does.
    private static bool _sixDofSpreadCold = true;
    // Gap between cold-solve iterations, in seconds. At 0.25 s the handful of
    // iterations a cold solve needs lands across about a second, as separate short
    // hitches rather than one merged freeze.
    private static double _sixDofColdIntervalS = 0.25;
    private static bool _sixDofConverging;      // cold solve in progress, not yet flyable
    private static int _sixDofColdFrames;
    private static int _sixDofGateIndex = -1;    // -1 = above every gate
    private static int _sixDofGateChanges;

    /// <summary>
    /// Step the node count down when the vehicle drops past a gate, carrying the
    /// current plan across by resampling it onto the new node count.
    ///
    /// A failed rebuild leaves the existing guidance untouched and simply retries at
    /// the next gate — losing the optimisation is survivable, losing the plan is not.
    /// </summary>
    private static void StepNodeGates(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                      double[] x, double now)
    {
        // RECOVERY. A run of refusals means the vehicle is flying an ageing
        // open-loop plan, and the commonest cause is a node step that landed on a
        // count too coarse for the conditions - the flight log for 20260806-142450
        // shows a step to 5 nodes at 261 m taking the defect from 0.49 m to 390 m and
        // refusing every re-solve thereafter. One-way stepping cannot recover from
        // that on its own, so step BACK UP one rung and refuse to go that coarse
        // again for the rest of the run.
        if (_sixDofRefusalRun >= RefusalsBeforeBackOff && _sixDofGateIndex > _sixDofBackedOffTo)
        {
            int finer = _sixDofGateIndex - 1;
            // Record the attempt BEFORE trying, so a rebuild that fails is not retried
            // every five cycles forever. Flight log 20260807-093052 did exactly that:
            // the same back-off to 25 nodes failed and was retried ~40 times, each one
            // a full config build plus a failed cold solve, which is pure added cost
            // on top of an already-failing loop.
            _sixDofBackedOffTo = _sixDofGateIndex;
            _sixDofRungFloor = Math.Min(_sixDofRungFloor, finer);
            _sixDofRefusalRun = 0;

            if (RebuildAt(vehicle, parent, siteCci, x, now, NodeRungs[finer]))
            {
                SixDofLog.Event(now,
                    $"NODE BACK-OFF at alt {x[2]:F0} m: {NodeRungs[_sixDofGateIndex]} -> " +
                    $"{NodeRungs[finer]} nodes, and no coarser this run");
                _sixDofGateIndex = finer;
                _sixDofBackedOffTo = finer;
            }
            else
            {
                SixDofLog.Event(now,
                    $"NODE BACK-OFF at alt {x[2]:F0} m to {NodeRungs[finer]} nodes FAILED; " +
                    "not retrying - the node count is not what is wrong");
            }
            return;
        }

        // Nodes needed to hold the target spacing at the CURRENT burn time. Sigma is
        // what sets spacing, and sigma counts down through the descent, so this falls
        // naturally without reference to altitude at all - and it adapts to a fast
        // entry or a slow one, which an altitude ladder cannot.
        double sigma = _sixDof.Sigma;
        if (sigma <= 0.0)
            return;
        int ideal = (int)Math.Ceiling(sigma / Math.Max(_sixDofNodeDtTarget, 0.05)) + 1;

        // Snap to a rung: the COARSEST count that still delivers at least the
        // resolution asked for. Falls back to the FINEST rung, not the coarsest, when
        // the burn is long enough that no rung reaches the target - wanting more
        // resolution than is on offer must give the most available, and defaulting
        // the other way would hand a 60-node problem 5 nodes.
        int nodes = MaxNodes;
        for (int i = NodeRungs.Length - 1; i >= 0; i--)
            if (NodeRungs[i] >= ideal) { nodes = NodeRungs[i]; break; }
        nodes = Math.Clamp(nodes, MinNodes, MaxNodes);

        // One-way only: never step back up. Sigma wobbles cycle to cycle, and
        // rebuilding the solver to add a node the problem does not really need costs
        // a warm start for nothing.
        int want0 = Array.IndexOf(NodeRungs, nodes);
        if (want0 < 0)
            return;
        // Never move coarser than a rung already shown to be unflyable this run.
        int want = Math.Min(want0, _sixDofRungFloor);
        if (want <= _sixDofGateIndex)
            return;
        nodes = NodeRungs[want];
        if (nodes >= _sixDof.Nodes)
        {
            _sixDofGateIndex = want;   // already at or below this count, nothing to do
            return;
        }

        if (RebuildAt(vehicle, parent, siteCci, x, now, nodes))
            _sixDofGateChanges++;
        _sixDofGateIndex = want;
    }

    /// <summary>
    /// Rebuild the guidance at a different node count, carrying the current plan
    /// across by resampling. Returns false and leaves the existing guidance untouched
    /// if anything fails — losing the node change is survivable, losing the plan is
    /// not.
    /// </summary>
    private static bool RebuildAt(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                  double[] x, double now, int nodes)
    {
        var xf = new double[14];
        xf[2] = _sixDofTargetAltM;
        TerminalAttitude(x, xf);

        if (!Ksa6DofSetup.TryBuild(vehicle, parent, siteCci, nodes, _sixDofTiltDeg,
                                   _sixDofThrottleFloor, _sixDofSigmaSeed, _sixDofThrustFrac,
                                   _sixDofRateDampShare, _sixDofControlSmooth,
                                   _sixDofProximal,
                                   _sixDofGlideSlopeDeg, _sixDofVzEnabled ? _sixDofVzMaxMs : -1.0,
                                   x, xf,
                                   out Scvx6DofConfig cfg,
                                   out Dynamics6Dof.Params dyn, out _))
        {
            SixDofLog.Event(now, $"NODE REBUILD at {nodes} nodes rejected by TryBuild");
            return false;
        }

        var next = new Ksa6DofGuidance(cfg, dyn) { FixedTime = _sixDofFixedTime };
        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz);
        next.SetInertia(ixx, iyy, izz);
        next.SetAccelBias(_sixDof.AccelBias);

        double sigmaWas = _sixDof.Sigma;
        if (!next.SeedFrom(_sixDof, x, now))
        {
            SixDofLog.Event(now, $"NODE STEP at alt {x[2]:F0} m FAILED to reseed at {nodes} " +
                                 $"nodes ({next.Error}) - staying at {_sixDof.Nodes}");
            return false;
        }

        SixDofLog.Event(now, $"NODE STEP at alt {x[2]:F0} m: {_sixDof.Nodes} -> {nodes} nodes " +
                             $"(sigma {sigmaWas:F1} s -> dt {sigmaWas / Math.Max(nodes - 1, 1):F2} s, " +
                             $"defect {next.LastDefectM:F2} m)");
        _sixDof = next;
        _sixDofLastReplan = now;
        return true;
    }

    // Set by the cadence gate so a row can say whether THIS cycle re-solved, rather
    // than only reporting the last solve's result forever. Without that distinction
    // a run of refusals is indistinguishable from a run of cycles that simply were
    // not due to solve.
    private static bool _sixDofDidSolve, _sixDofSolveOk;

    private static void LogCycle(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                 double[] x, double now, double thrustN, double capability,
                                 double throttle, double3 torqueModel, double ambientPa)
    {
        if (!SixDofLog.Enabled)
            return;

        double g = parent.Mu / (siteCci.Length() * siteCci.Length());
        double mass = vehicle.TotalMass;
        double weight = Math.Max(mass * g, 1.0);
        double altToGo = x[2] - _sixDofTargetAltM;
        double descent = -x[5];
        double netDecel = (capability / weight - 1.0) * g;
        double stopDist = descent > 0.0 && netDecel > 0.01
            ? descent * descent / (2.0 * netDecel)
            : 0.0;

        // Signed distance outside the glideslope cone, same convention the solver
        // enforces: positive means outside.
        double glideViol = 0.0;
        if (_sixDofGlideSlopeDeg > 0.0)
        {
            double cot = 1.0 / Math.Tan(Math.Clamp(_sixDofGlideSlopeDeg, 1e-3, 89.999) * Math.PI / 180.0);
            glideViol = Math.Sqrt(x[0] * x[0] + x[1] * x[1]) - cot * (x[2] - _sixDofTargetAltM);
        }

        double r22 = 1.0 - 2.0 * (x[7] * x[7] + x[8] * x[8]);
        TvcAllocationResult a = KsaGimbalControl.LastAllocation;

        var row = new SixDofLog.CycleRow
        {
            T = now, Alt = x[2],
            Rx = x[0], Ry = x[1], Rz = x[2], Vx = x[3], Vy = x[4], Vz = x[5],
            Qw = x[6], Qx = x[7], Qy = x[8], Qz = x[9],
            TiltDeg = Math.Acos(Math.Clamp(r22, -1.0, 1.0)) * 180.0 / Math.PI,
            Wx = x[10], Wy = x[11], Wz = x[12], Mass = x[13],
            Solved = _sixDofDidSolve && _sixDofSolveOk,
            Status = _sixDofDidSolve ? _sixDof.Status.ToString() : "(no solve this cycle)",
            ScvxIters = _sixDof.LastIterations, Accepted = _sixDof.AcceptedSteps,
            SolveMs = _sixDof.LastSolveMs, Admm = 0,
            DefectM = _sixDof.LastDefectM, DefectLimitM = _sixDof.MaxDefectM,
            AnchorM = _sixDof.AnchorOffsetM,
            FellBack = _sixDof.FellBack, Escalations = _sixDof.Escalations,
            Nodes = _sixDof.Nodes, Sigma = _sixDof.Sigma, PlanElapsed = _sixDof.PlanElapsed,
            ThrustDemandN = thrustN, CapabilityN = capability, Throttle = throttle,
            Saturated = _sixDofThrustSaturated,
            TauX = torqueModel.X, TauY = torqueModel.Y, TauZ = torqueModel.Z,
            AllocX = a.AchievedTorque.X, AllocY = a.AchievedTorque.Y, AllocZ = a.AchievedTorque.Z,
            AllocSat = a.SaturationScale,
            Twr = capability / weight, TwrMin = _sixDofThrottleFloor * capability / weight,
            AmbientPa = ambientPa,
            AltToGo = altToGo, DescentRate = descent, StopDistM = stopDist,
            GlideViolM = glideViol,
            BiasX = _sixDof.AccelBias.X, BiasY = _sixDof.AccelBias.Y, BiasZ = _sixDof.AccelBias.Z,
            Error = _sixDofError,
        };
        SixDofLog.Cycle(row);
        _sixDofDidSolve = false;
    }

    // Acceleration-bias estimator state. Reset on engage.
    private static double[] _sixDofPrevV;
    private static double _sixDofPrevT;
    private static double3 _sixDofBias;
    private static bool _sixDofBiasEnabled = true;

    /// <summary>
    /// Estimate the acceleration the model does NOT account for, and hand it to the
    /// solver as a gravity offset.
    ///
    /// residual = measured acceleration - (delivered thrust / m + modelled gravity)
    ///
    /// Everything unmodelled lands here together - a mis-set gravity, a thrust
    /// calibration error, aerodynamic drag - which is the point: the correction does
    /// not need to know which. It also tracks drag DOWN as speed comes off, instead
    /// of assuming a fixed penalty.
    ///
    /// Three guards, none of them decoration:
    ///   - a time constant, because a one-frame velocity difference is mostly noise
    ///     and feeding that into the plan would make the trajectory jitter;
    ///   - a clamp, so a physics glitch or a stage separation cannot hand the solver
    ///     an absurd gravity and make every subsequent plan nonsense;
    ///   - a dt window, since the first cycle after engage has no previous sample and
    ///     a paused or warped frame produces a meaningless one.
    /// </summary>
    private static void UpdateAccelBias(double[] x, double now, double thrustDelivered)
    {
        if (_sixDof == null)
            return;
        if (!_sixDofBiasEnabled)
        {
            _sixDof.SetAccelBias(default);
            return;
        }

        double dt = now - _sixDofPrevT;
        if (_sixDofPrevV == null || dt < 1e-3 || dt > 0.5)
        {
            _sixDofPrevV = [x[3], x[4], x[5]];
            _sixDofPrevT = now;
            return;
        }

        // Measured acceleration, site frame.
        double ax = (x[3] - _sixDofPrevV[0]) / dt;
        double ay = (x[4] - _sixDofPrevV[1]) / dt;
        double az = (x[5] - _sixDofPrevV[2]) / dt;
        _sixDofPrevV = [x[3], x[4], x[5]];
        _sixDofPrevT = now;

        // Modelled acceleration: thrust along the body +Z axis, plus the gravity the
        // config was built with (NOT the biased value, or the estimate feeds itself).
        double qw = x[6], qx = x[7], qy = x[8], qz = x[9];
        double zx = 2.0 * (qx * qz + qw * qy);
        double zy = 2.0 * (qy * qz - qw * qx);
        double zz = 1.0 - 2.0 * (qx * qx + qy * qy);
        double m = Math.Max(x[13], 1.0);
        double3 g0 = _sixDof.BaseGravity;

        double rx = ax - (thrustDelivered / m * zx + g0.X);
        double ry = ay - (thrustDelivered / m * zy + g0.Y);
        double rz = az - (thrustDelivered / m * zz + g0.Z);

        // ~2 s time constant at a 0.02 s step: slow enough to ignore per-frame noise,
        // fast enough to follow drag falling away through the descent.
        double alpha = Math.Clamp(dt / 2.0, 0.0, 1.0);
        _sixDofBias = new double3(
            _sixDofBias.X + alpha * (rx - _sixDofBias.X),
            _sixDofBias.Y + alpha * (ry - _sixDofBias.Y),
            _sixDofBias.Z + alpha * (rz - _sixDofBias.Z));

        const double Limit = 5.0;
        _sixDof.SetAccelBias(new double3(
            Math.Clamp(_sixDofBias.X, -Limit, Limit),
            Math.Clamp(_sixDofBias.Y, -Limit, Limit),
            Math.Clamp(_sixDofBias.Z, -Limit, Limit)));
    }

    /// <summary>
    /// Terminal attitude: UPRIGHT, at whatever yaw the vehicle already has.
    ///
    /// This used to be the identity quaternion, which pins the yaw as well - and yaw
    /// about the thrust axis is the one attitude degree of freedom a landing does not
    /// care about. Pinning it is not free: it is a ROLL in the model's body frame,
    /// the axis with by far the least authority, since roll comes only from the
    /// off-axis vernier gimbals (measured ~700x weaker than pitch/yaw on this
    /// vehicle). A vehicle that happens to be yawed 180 degrees - which is exactly
    /// what the flight logs show, qw ~ 0 and qz ~ -1 - is then required to perform a
    /// 180 degree roll on its weakest axis for no reason at all, and the optimiser
    /// spends real control authority and real trajectory shape doing it. Measured
    /// offline on the logged state, that alone turns a 1 degree bearing sweep into 26.
    ///
    /// Keeping the current yaw means levelling out is the ONLY attitude work left.
    /// Projecting q onto (w, 0, 0, z) and renormalising is exactly "same yaw, no
    /// tilt": the x and y components ARE the tilt.
    /// </summary>
    private static void TerminalAttitude(double[] x, double[] xf)
    {
        double qw = x[6], qz = x[9];
        double m = Math.Sqrt(qw * qw + qz * qz);
        if (m < 1e-9)
        {
            // Tilted a full 180 degrees, so there is no yaw to preserve. Identity is
            // as good an answer as any and keeps the quaternion a unit.
            xf[6] = 1.0; xf[7] = xf[8] = xf[9] = 0.0;
            return;
        }
        xf[6] = qw / m;
        xf[7] = 0.0;
        xf[8] = 0.0;
        xf[9] = qz / m;
    }

    private static void Disengage6Dof(Vehicle vehicle, bool cutEngine = true)
    {
        SixDofLog.Stop();
        _sixDofActive = false;
        _sixDofEngagePending = false;
        _sixDofConverging = false;
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

        // CIRCUIT BREAKER. A long run of refusals means the plan is stale, the vehicle
        // has diverged from it, and every further Update is running the wide-trust-
        // region retry - so failure costs MORE than success and the loop digs itself
        // deeper. Once that happens the answer is not to keep hammering it: throw the
        // plan away and cold-start again, spread over frames so it does not stall the
        // sim thread.
        // ESCALATION LADDER. Each rung is only reached when the cheaper one below has
        // stopped working, and the trigger for the top rung is DRIFT rather than a
        // failure count - because drift is the cause and failures are the symptom.
        //
        //   1. warm re-solve, tight trust region        (Update, normal case)
        //   2. wide trust region, re-linearised         (Update's retry, on divergence)
        //   3. cold re-seed from the current state      (here, spread over frames)
        //
        // A warm start is an advantage only while the vehicle is NEAR its plan. Past
        // that, the stale plan is a worse seed than a straight line from where the
        // vehicle actually is, because the straight line is at least self-consistent.
        // So once drift passes that point there is nothing to be gained by re-solving
        // from it, however many times we try - and waiting for fifteen failures first
        // just means fifteen expensive cycles of flying open loop.
        double drift = _sixDof != null ? _sixDof.MeasureDrift(x, now) : 0.0;
        double driftLimit = Math.Max(ColdRestartDriftM, 0.05 * Math.Max(x[2], 1.0));
        bool driftedOff = _sixDof != null && _sixDof.HasPlan && drift > driftLimit;

        if (!_sixDofConverging && _sixDof != null &&
            (driftedOff || _sixDofRefusalRun >= RefusalsBeforeRestart))
        {
            var xfR = new double[14];
            xfR[2] = _sixDofTargetAltM;
            TerminalAttitude(x, xfR);
            SixDofLog.Event(now,
                $"COLD RESTART at alt {x[2]:F0} m: " +
                (driftedOff ? $"drifted {drift:F0} m from the plan (limit {driftLimit:F0} m)"
                            : $"{_sixDofRefusalRun} refused re-solves") +
                $" - plan {_sixDof.PlanElapsed:F1} s stale, defect {_sixDof.LastDefectM:F0} m");
            _sixDof.ColdIterationIntervalS = _sixDofColdIntervalS;
            _sixDof.BeginCold(x, xfR, Math.Max(_sixDof.Sigma, _sixDofSigmaSeed));
            _sixDofConverging = true;
            _sixDofColdFrames = 0;
            _sixDofRefusalRun = 0;
            _sixDofRecoveries++;
            _sixDofError = "re-converging...";
        }

        // Advance a spread cold solve. Until it produces a flyable plan there is
        // nothing to command, so this returns without touching the actuators - the
        // vehicle carries on doing whatever it was doing.
        if (_sixDofConverging)
        {
            _sixDofColdFrames++;
            if (_sixDof.StepCold(x, now))
            {
                _sixDofConverging = false;
                _sixDofError = "";
                SixDofLog.Event(now,
                    $"COLD SOLVE CONVERGED in {_sixDof.LastIterations} iterations spread over " +
                    $"{_sixDofColdFrames} frames, defect {_sixDof.LastDefectM:F2} m, " +
                    $"sigma {_sixDof.Sigma:F1} s");
                SixDofLog.PlanSnapshot(now, _sixDof.Nodes, _sixDof.PlanState, _sixDof.PlanControl);
            }
            else
            {
                _sixDofError = $"converging... {_sixDof.LastIterations} iterations, " +
                               $"defect {_sixDof.LastDefectM:F1} m";
                // Give up rather than fall forever if it is not going to converge.
                if (_sixDofColdFrames > 240)
                {
                    SixDofLog.Event(now, "COLD SOLVE gave up after 240 frames: " + _sixDof.Error);
                    Disengage6Dof(vehicle);
                    _sixDofError = "cold solve failed to converge: " + _sixDof.Error;
                }
                return;
            }
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
            SixDofLog.Event(now, $"TOUCHDOWN at alt {x[2]:F1} m, vz {x[5]:F2} m/s, " +
                                 $"lateral {Math.Sqrt(x[0] * x[0] + x[1] * x[1]):F1} m from target");
            SixDofLog.Stop();
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
            SixDofLog.Event(now, $"HANDOFF to terminal hover at alt {x[2]:F1} m, " +
                                 $"vz {x[5]:F2} m/s, lateral {Math.Sqrt(x[0] * x[0] + x[1] * x[1]):F1} m");
            SixDofLog.Stop();
            Disengage6Dof(vehicle, cutEngine: false);
            StartTerminalHover();
            _landingStatus = $"6-DOF handoff to terminal hover at {x[2]:F0} m.";
            _sixDofError = _landingStatus;
            return;
        }

        // Estimate what the model is missing BEFORE re-solving, so the optimiser
        // plans around it rather than rediscovering it every cycle.
        UpdateAccelBias(x, now, _sixDofLastThrottle * _sixDofCapabilityN);

        // NODE SCHEDULE: drop node count at fixed altitude GATES.
        //
        // The quantity worth holding roughly constant is node SPACING, not node
        // count. Node spacing is sigma/(N-1), so as the horizon collapses on
        // approach the same N buys ever finer resolution than the problem needs,
        // and the solve is paying for nodes that are no longer earning anything.
        // The earlier measurement that fewer nodes means more plan jump (4.0 m at
        // N=20 against 1.0 m at N=80) was taken over a FULL-LENGTH trajectory,
        // where N=20 meant coarse spacing; close in, the same count is fine.
        //
        // Stepped at gates rather than recomputed continuously, on purpose. Every
        // change of N rebuilds the solver and throws away the ADMM warm start (the
        // sparsity pattern is frozen at the node count), so a continuously-tracked
        // N would pay that on EVERY cycle and never keep a warm start at all. Four
        // transitions over a descent is affordable; four hundred is not.
        //
        // One-way, too: the ladder never steps back up. Re-crossing a gate during a
        // brief climb would rebuild twice for no benefit.
        if (_sixDofNodeGates)
            StepNodeGates(vehicle, parent, siteCci, x, now);

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
            _sixDofDidSolve = true;
            if (_sixDof.Update(x, now))
            {
                _sixDofLastReplan = now;
                _sixDofError = "";
                _sixDofSolveOk = true;
                _sixDofRefusalRun = 0;
                SixDofLog.PlanSnapshot(now, _sixDof.Nodes, _sixDof.PlanState, _sixDof.PlanControl);
            }
            else
            {
                _sixDofError = "re-solve failed: " + _sixDof.Error;
                _sixDofSolveOk = false;
                _sixDofRefusalRun++;
                // Every refusal, with its reason. A refused re-solve silently leaves
                // the vehicle on a stale open-loop plan, so a run of these in the
                // event log is the signature to look for first.
                SixDofLog.Event(now, "RE-SOLVE REFUSED: " + _sixDof.Error);
            }
        }

        // Past the end of the plan the node index clamps, so the terminal control is
        // held indefinitely. That is a reasonable hold-in-place fallback, but it is
        // NOT guidance any more and must not look like it.
        if (_sixDof.PlanElapsed > _sixDof.Sigma)
            _sixDofError = $"plan expired {_sixDof.PlanElapsed - _sixDof.Sigma:F1} s ago - " +
                           "holding the terminal command, no longer guiding.";

        if (!_sixDof.Command(now, out double3 torqueModel, out double thrustN))
            return;

        // CLOSE THE LOOP ON THRUST. The optimiser asks for newtons; KSA's throttle is
        // a fraction of what the engines can produce AT THIS INSTANT, so the demand is
        // divided by the vehicle's live capability rather than by the plan's Tmax.
        //
        // This is what makes the commanded thrust the delivered thrust regardless of
        // engine performance. The plan's Tmax is fixed at solve time and evaluated at
        // the TARGET altitude, so it is deliberately conservative and drifts further
        // from the truth the higher the vehicle is; dividing by it under-throttles by
        // exactly that ratio the whole way down. MPC cannot see this - re-solving
        // fixes the state estimate, not the actuator mapping, so every replan is
        // executed just as wrongly as the one before. That is the shortfall that made
        // the vehicle sink below its own plan until only a loop was feasible.
        double ambientPa = KsaEnginePerf.AmbientPressureAt(
            parent, siteCci.Length() - parent.MeanRadius + x[2]);
        double capability = KsaEnginePerf.ActiveThrustCapability(vehicle, ambientPa);

        // Invert the REAL thrust curve rather than dividing by full-throttle thrust.
        // Thrust is not proportional to throttle in an atmosphere - see
        // KsaEnginePerf.ThrustAtThrottle - so demand/capability under-delivers by a
        // near-constant amount, worst at low throttle, which is precisely where a
        // descent spends its time.
        //
        // Falling back to the linear estimate when there is nothing to invert is not
        // a nicety: before ignition capability is 0 and dividing gives NaN.
        double throttle = KsaEnginePerf.ThrottleForThrust(vehicle, thrustN, ambientPa);
        if (throttle < 0.0)
        {
            double denom = capability > 1.0 ? capability : _sixDof.Tmax;
            throttle = thrustN / denom;
        }
        throttle = Math.Clamp(throttle, 0.0, 1.0);

        _sixDofDemandN = thrustN;
        _sixDofCapabilityN = capability;
        // Saturation is the honest failure signal here: the plan is asking for more
        // than the vehicle physically has, so the trajectory being flown is not the
        // one that was planned, and no amount of feedback recovers it.
        _sixDofThrustSaturated = capability > 1.0 && thrustN > capability;

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

        LogCycle(vehicle, parent, siteCci, x, now, thrustN, capability, throttle,
                 torqueModel, ambientPa);
    }

    private static bool Engage6Dof(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                   double[] x, double now)
    {
        // Target: hover point above the pad, upright and at rest. Mass is free, so the
        // terminal state carries 13 of the 14 components. Built BEFORE the config
        // because the problem scaling is sized from the x0 -> xf extent.
        var xf = new double[14];
        xf[2] = _sixDofTargetAltM;
        TerminalAttitude(x, xf);

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

        // SEED FROM G-FOLD. SCvx refines a reference rather than searching for one,
        // so the seed decides how many iterations the cold solve needs and which local
        // solution it walks toward. G-FOLD solves the same landing under a convex
        // 3-DOF model in a few milliseconds, with no initial guess of its own and no
        // local minima, and its golden-section search over time of flight also
        // supplies a burn time far better than a fixed guess.
        //
        // Strictly an optimisation: any failure falls back to the straight-line seed,
        // because a worse guess is enormously better than not engaging.
        // SPREAD: start the cold solve and return. Step6DofCore advances it a
        // bounded slice at a time until the plan is flyable, so engaging costs a
        // few milliseconds of this frame rather than the whole solve.
        if (_sixDofSpreadCold && !_sixDofFixedTime && !_sixDofGfoldSeed)
        {
            _sixDof.ColdIterationIntervalS = _sixDofColdIntervalS;
            _sixDof.BeginCold(x, xf, _sixDofSigmaSeed);
            _sixDofConverging = true;
            _sixDofColdFrames = 0;
            _sixDofActive = true;
            _sixDofTouchdownArmed = false;
            _sixDofGateIndex = -1;
            _sixDofGateChanges = 0;
            _sixDofRefusalRun = 0;
            _sixDofRungFloor = int.MaxValue;
            _sixDofPrevV = null;
            _sixDofBias = default;
            _sixDofLastReplan = now;
            _sixDofError = "converging...";
            if (_sixDofLogging)
            {
                SixDofLog.Start(vehicle.ToString(), parent.ToString());
                SixDofLog.Event(now, $"ENGAGED (spread cold solve)  nodes {_sixDofNodes}  " +
                                     $"target alt {_sixDofTargetAltM:F0} m  cadence {_sixDofReplanSec:F2} s");
            }
            return true;
        }

        bool ok = false;
        string seedNote = "";
        if (_sixDofGfoldSeed && !_sixDofFixedTime &&
            Ksa6DofGfoldSeed.TryBuild(x, xf, cfg, dyn, _sixDofNodes,
                                      out double[] gx, out double[] gu, out double gSigma,
                                      out seedNote))
        {
            ok = _sixDof.PlanFromSeed(x, xf, gx, gu, gSigma, now);
            if (!ok)
                seedNote += $" (rejected: {_sixDof.Error}) - retrying from the straight-line seed";
        }

        if (!ok)
        {
            ok = _sixDofFixedTime
                ? _sixDof.PlanSearch(x, xf, _sixDofSigmaSeed, now, Math.Clamp(_sixDofSigmaSamples, 1, 12))
                : _sixDof.Plan(x, xf, _sixDofSigmaSeed, now);
        }
        if (!ok)
        {
            _sixDofError = "cold solve failed: " + _sixDof.Error;
            _sixDof = null;
            return false;
        }

        _sixDofError = "";
        _sixDofActive = true;
        _sixDofTouchdownArmed = false;
        _sixDofGateIndex = -1;
        _sixDofGateChanges = 0;
        _sixDofLastReplan = now;
        _sixDofPrevV = null;
        _sixDofRefusalRun = 0;
        _sixDofRungFloor = int.MaxValue;
        _sixDofBackedOffTo = -1;
        _sixDofRecoveries = 0;
        _sixDofBias = default;

        if (_sixDofLogging)
        {
            SixDofLog.Start(vehicle.ToString(), parent.ToString());
            SixDofLog.Event(now,
                $"ENGAGED  nodes {_sixDofNodes}  tilt {_sixDofTiltDeg:F0} deg  " +
                $"floor {_sixDofThrottleFloor:F2}  target alt {_sixDofTargetAltM:F0} m  " +
                $"glideslope {_sixDofGlideSlopeDeg:F0} deg  " +
                $"vzMax {(_sixDofVzEnabled ? _sixDofVzMaxMs.ToString("F1") : "off")}  " +
                $"cadence {_sixDofReplanSec:F2} s  gates {(_sixDofNodeGates ? "on" : "off")}");
            SixDofLog.Event(now,
                (seedNote.Length > 0 ? seedNote + "  |  " : "") +
                $"cold solve: {_sixDof.Status}, {_sixDof.LastIterations} iters, " +
                $"defect {_sixDof.LastDefectM:F2} m, sigma {_sixDof.Sigma:F1} s, " +
                $"Tmax {_sixDof.Tmax / 1e6:F2} MN");
            SixDofLog.PlanSnapshot(now, _sixDof.Nodes, _sixDof.PlanState, _sixDof.PlanControl);
        }
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

        // CAN THIS BE DONE AT ALL? The question the rest of the panel does not ask.
        //
        // Thrust only buys deceleration ABOVE hover, so the usable figure is
        // (TWR - 1) * g, not TWR * g. Stopping from speed v therefore needs
        // v^2 / (2 * (TWR-1) * g) of altitude, and if that exceeds what is left the
        // landing is impossible for ANY guidance law - the solver will return its
        // best effort, which is a long curving miss that looks exactly like a
        // controller bug and is not one.
        //
        // Measured in closed loop against this criterion (Scvx.Console --mpc --grav,
        // 300 m up at 50 m/s down): TWR 2.00 needs 127 m and misses by 4.8 m; TWR
        // 1.50 needs 255 m and misses by 39 m; TWR 1.25 needs 510 m of the 300 m
        // available and misses by 173 m. The failure tracks reachability, not
        // conditioning - Earth at TWR 2.0 actually solves FASTER than the Moon.
        //
        // A necessary condition, not a sufficient one: it assumes thrust straight up,
        // so any tilt to kill downrange makes it worse.
        double[] xNow = KsaFrameBridge.ToModelState(vehicle, KsaFrameBridge.BuildSiteFrame(siteCci));
        double altToGo = xNow[2] - _sixDofTargetAltM;
        double descent = -xNow[5];
        if (altToGo > 1.0 && descent > 1.0)
        {
            double twrNow = thrust / (mass * g);
            double netDecel = (twrNow - 1.0) * g;
            double stopDist = netDecel > 0.01 ? descent * descent / (2.0 * netDecel) : double.PositiveInfinity;
            double twrNeeded = 1.0 + descent * descent / (2.0 * altToGo * g);

            ImGui.Text($"descending {descent,5:F0} m/s with {altToGo,6:F0} m to go   " +
                       $"needs {(double.IsInfinity(stopDist) ? 99999.0 : stopDist),6:F0} m to stop");
            if (stopDist >= altToGo)
                ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                    $"CANNOT STOP IN TIME - needs TWR {twrNeeded:F2}, has {twrNow:F2}. No guidance " +
                    "can land this; the plan will be a long curving miss. Burn earlier, or arrive slower.");
            else if (stopDist > 0.8 * altToGo)
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    $"MARGINAL - {stopDist / altToGo * 100.0:F0}%% of the remaining altitude is needed just " +
                    $"to stop, leaving almost none to null downrange. Needs TWR {twrNeeded:F2}, has {twrNow:F2}.");
        }

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
