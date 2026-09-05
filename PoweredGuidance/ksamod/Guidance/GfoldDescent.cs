using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using Gfold;
using KSA;

// Convex (G-FOLD) powered descent from the high gate to the surface, flown as
// committed-trajectory tracking: solve a min-fuel plan, fly it by time index
// (feed-forward the planned thrust + PD feedback on the planned state), re-solve
// on a cadence. Also home to the G-FOLD debug window, which plots every series
// of the committed optimal trajectory.
public static partial class PoweredGuidanceWindow
{
    // Every knob and every piece of committed state this descent runs on lives on the
    // vehicle (VehicleAutopilotState): the pointing cone, the throttle bounds and the
    // vehicle height are airframe properties, the feedback gains are tuned per craft,
    // and the plan itself obviously belongs to the craft flying it.

    // What the solver aims the CoM at: the surface plus the vehicle height, so the
    // legs (not the CoM) meet the ground at rest. G-FOLD always plans the whole way
    // to the surface; the hover handoff simply cuts over in the last stretch.
    private static double GfoldSolverTargetAltM => _s.VehicleHeightM;
    private const double GfoldMinTf = 4.0;
    private const double GfoldCoastThrottle = 0.02;  // below this, cut the engine (true coast)

    // Flight-time search bounds. SearchTfMax is the cold-start ceiling; the two
    // bracket factors are the window searched around the previous solution's
    // remaining time when one is available (see SolveGfoldPlan). Wide enough to
    // absorb a plan that has drifted, narrow enough to cut most of the coarse scan.
    private const double SearchTfMax = 120.0;
    private const double SearchBracketLo = 0.5;
    private const double SearchBracketHi = 2.0;

    // Committed-trajectory tracking: the solved descent plan is flown by time
    // index (feed-forward the planned thrust at the current time + light PD
    // feedback on the reference state) and re-solved on a cadence, rather than
    // applying node 0. This follows the plan's coast (throttle down) and brake
    // arcs instead of freezing the first node.
    // Command smoothing: the throttle and thrust direction are low-pass filtered
    // toward the freshly-computed command with this time constant, so per-frame
    // feedback noise and re-solve steps don't reach the engine/gimbal as chatter.

    private static void StepGfoldDescent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                         double bodyRadius, double now)
    {
        if (_s.Engage && _s.AutoStage)
            AutoSequence(vehicle);

        double3 siteCci = SiteDirCciAt(parent, 0) * (bodyRadius + SiteTerrainHeight(parent));
        var frame = KsaGfold.BuildFrame(siteCci);
        double3 v = orbit.StateVectors.VelocityCci;

        // The flown state is the CoM — the vehicle-height allowance lives in the
        // solver TARGET (see GfoldSolverTargetAltM), so attitude changes don't
        // perturb the reference state. _s.GfoldAltM is height above touchdown: zero
        // when the CoM sits _s.VehicleHeightM over the pad, i.e. legs on the ground.
        double3 r = orbit.StateVectors.PositionCci;
        double3 vSrf = v - double3.Cross(parent.GetAngularVelocityCci(), r);
        _s.GfoldAltM = double3.Dot(r - siteCci, frame.Ex) - _s.VehicleHeightM;
        _s.GfoldSpeedMs = vSrf.Length();
        double3 gfLocal = frame.PointToLocal(r);   // X-up frame
        RecordGfoldTrace(Math.Sqrt(gfLocal.Y * gfLocal.Y + gfLocal.Z * gfLocal.Z), gfLocal.X);

        // Hand off to the terminal hover controller for the last stretch: G-FOLD
        // brings the vehicle down to the handoff height (slow and near-vertical),
        // and the hover flies the final touchdown. This is the only exit from the
        // G-FOLD descent now — G-FOLD never lands the vehicle itself.
        if (_s.GfoldAltM <= _s.GfoldHoverHandoffAltM)
        {
            StartTerminalHover(vehicle);
            _s.LandingStatus = $"G-FOLD handoff to terminal hover at {_s.GfoldAltM:F0} m.";
            return;
        }

        // Inside the last GfoldMinTf seconds before the planned arrival, the distance
        // still to fly is too small for any valid flight time: tf >= TfMin (the search
        // floor) overshoots it, so a re-solve goes degenerate and reports the target
        // unreachable right before the handoff. Freeze the committed plan there — it
        // already terminates at the target — and just fly it down.
        bool terminalWindow = _s.GfoldPlan != null && _s.GfoldArrivalTime - now <= GfoldMinTf;
        if (!terminalWindow &&
            (_s.GfoldPlan == null || now - _s.GfoldLastSolveTime >= _s.GfoldIntervalS))
            SolveGfoldPlan(vehicle, parent, frame, siteCci, r, now);

        // Fly the committed plan by time index every frame (feed-forward + PD). Track
        // in the LIVE site frame (rebuilt this step), not the solve-time frame: the
        // site is body-fixed, so its CCI position rotates with the body, and the live
        // frame carries the plan around with it so we keep aiming at the real pad.
        if (_s.GfoldPlan != null)
            TrackGfoldPlan(frame, r, vSrf, vehicle.TotalMass, now);
    }

    // Solve a fresh descent plan from the current state and commit it. A min-fuel
    // search (so it coasts/brakes optimally — "throttles down") to the site; if the
    // site is unreachable in the remaining time the search floats the touchdown to
    // the closest point, so this degrades gracefully instead of going infeasible.
    private static void SolveGfoldPlan(Vehicle vehicle, IParentBody parent,
                                       KsaGfold.Frame frame, double3 siteCci, double3 comPos, double now)
    {
        GfoldParams p = KsaGfold.BuildParams(
            vehicle, parent, frame, siteCci, comPos, _s.GfoldGlideSlopeDeg, _s.GfoldPointingDeg, _s.GfoldVMaxMs,
            GfoldSolverTargetAltM, 0.0, _s.GfoldThrottleMin, _s.GfoldThrottleMax);
        if (p == null)
        {
            _s.LandingStatus = "G-FOLD: no engine — holding.";
            return;
        }

        // Mark the attempt now (not only on success) so a failing solve retries on the
        // normal cadence rather than every frame while we keep flying the last plan.
        _s.GfoldLastSolveTime = now;

        // GfoldPlanner.SolveTimeLimitS is process-wide, which is safe ONLY because the
        // solve below is synchronous and on this thread: each vehicle sets it
        // immediately before its own call and the call has returned before any other
        // vehicle is serviced. Move the solve to a worker and this becomes a race —
        // see GfoldSolveMs for why that move is worth making anyway.
        GfoldPlanner.SolveTimeLimitS = _s.GfoldSolveTimeLimitS > 0 ? _s.GfoldSolveTimeLimitS : null;
        var solveClock = System.Diagnostics.Stopwatch.StartNew();

        try
        {
            // On the cadence, prefer to keep the arrival time set at handoff (solve
            // the remaining time); only re-run the full search when that fails or
            // we have no plan yet.
            GfoldTrajectory traj = null;
            if (!_s.GfoldForceSearch && _s.GfoldPlan != null && _s.GfoldArrivalTime - now > GfoldMinTf)
            {
                double remaining = _s.GfoldArrivalTime - now;
                GfoldTrajectory t = GfoldPlanner.SolveMinFuel(
                    p, remaining, _s.GfoldNodes, [GfoldSolverTargetAltM, 0.0, 0.0],
                    options: GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg });
                if (t.Status is ConicStatus.Optimal or ConicStatus.OptimalInaccurate)
                    traj = t;
            }
            if (traj == null)
            {
                // Warm-start the SEARCH from the last solution. The solver itself
                // can't be warm started — ECOS is an interior-point method and the
                // previous optimum sits on the boundary, the worst possible starting
                // iterate — but the flight-time search around it can be.
                //
                // The committed plan's remaining time IS the previous solution
                // carried forward: _s.GfoldArrivalTime was set to now + tf* when that
                // search ran, so (_s.GfoldArrivalTime - now) is tf* minus the time
                // since. That's the best available estimate of the new optimum, so
                // bracket it instead of rescanning the full range. Searching
                // [4, 120] from cold spends 8 coarse points plus ~10 golden-section
                // steps, at up to two SOCP solves each — 35-40 solves. A tight
                // bracket collapses the coarse scan onto the plausible range.
                //
                // Both bounds move, not just the upper one: with a 20 s remaining
                // flight the old floor of 4 s was as wasteful as the old 120 s
                // ceiling, just at the other end.
                // _s.GfoldForceSearch means the previous solution is no longer a valid
                // guess — a retarget replaces _s.GfoldArrivalTime with a placeholder
                // far in the future purely to escape the terminal freeze, so
                // bracketing around it would spend a narrow search on a fabricated
                // centre and then fall back anyway. Go straight to the full range.
                double tfLo = GfoldMinTf, tfHi = SearchTfMax;
                bool bracketed = !_s.GfoldForceSearch
                    && _s.GfoldPlan != null && _s.GfoldArrivalTime - now > GfoldMinTf;
                if (bracketed)
                {
                    double expected = _s.GfoldArrivalTime - now;
                    tfLo = Math.Max(expected * SearchBracketLo, GfoldMinTf);
                    tfHi = Math.Min(expected * SearchBracketHi, SearchTfMax);
                    bracketed = tfHi > tfLo;
                }

                GfoldPlanner.SearchResult best = bracketed
                    ? GfoldPlanner.SearchMinFuel(
                        p, _s.GfoldNodes, tfLo: tfLo, tfHi: tfHi,
                        options: GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg })
                    : null;

                // A bracket can only lose solutions that lie outside it, so the full
                // range is still tried before declaring the site unreachable. Costs
                // the old price only when the cheap window genuinely found nothing —
                // which is also exactly when the vehicle's situation has changed
                // enough that the previous solution was a bad guess.
                if (best == null)
                    best = GfoldPlanner.SearchMinFuel(
                        p, _s.GfoldNodes, tfLo: GfoldMinTf, tfHi: SearchTfMax,
                        options: GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg });

                if (best == null)
                {
                    FailGfold($"G-FOLD unreachable: alt {_s.GfoldAltM:F0} m, {_s.GfoldSpeedMs:F0} m/s, " +
                              $"TWR {p.ThrustMax / (vehicle.TotalMass * p.GravityMag):F1}, fuel {p.FuelMass:F0} kg");
                    return;
                }
                traj = best.Trajectory;
                _s.GfoldArrivalTime = now + best.TimeOfFlight;
            }

            _s.GfoldSolveMs = solveClock.Elapsed.TotalMilliseconds;
            _s.GfoldStatus = traj.Status;
            _s.GfoldPlan = traj;
            _s.GfoldPlanStart = now;
            _s.GfoldThrustMax = p.ThrustMax;
            _s.GfoldFailStreak = 0;
            _s.GfoldForceSearch = false;
            _s.LandingStatus = "";
        }
        catch (Exception e)
        {
            _s.GfoldSolveMs = solveClock.Elapsed.TotalMilliseconds;
            FailGfold("G-FOLD: " + e.Message);
        }
    }

    // Fly the committed plan: feed-forward the planned thrust at the current time
    // plus PD feedback on the planned state, expressed in the given (live) site
    // frame so the plan stays locked to the body-fixed, rotating landing pad.
    private static void TrackGfoldPlan(KsaGfold.Frame f, double3 r, double3 vSrf, double mass, double now)
    {
        GfoldTrajectory plan = _s.GfoldPlan;
        int n = plan.Nodes;
        double elapsed = now - _s.GfoldPlanStart;

        // Reference STATE at the current time: interpolate from node 0. The plan
        // starts at the current state, so tracking error is ~0 just after a solve;
        // reading it one node ahead (as the feed-forward does) would feed a whole
        // step of expected motion ~v*dt back as phantom error and tilt the command
        // past the pointing cone.
        double sf = Math.Clamp(elapsed / plan.Dt, 0.0, n - 1);
        int s0 = Math.Clamp((int)Math.Floor(sf), 0, n - 2);
        double sfrac = Math.Clamp(sf - s0, 0.0, 1.0);
        double3 refPos = Lerp(Node(plan.Position, s0), Node(plan.Position, s0 + 1), sfrac);
        double3 refVel = Lerp(Node(plan.Velocity, s0), Node(plan.Velocity, s0 + 1), sfrac);

        // Feed-forward THRUST: skip node 0 (its control is the unconstrained
        // artifact), so the first step uses node 1's real, cone-respecting thrust.
        double tf = Math.Max(elapsed / plan.Dt, 1.0);
        int t0 = Math.Clamp((int)Math.Floor(tf), 1, n - 2);
        double tfrac = Math.Clamp(tf - t0, 0.0, 1.0);
        double3 ff = Lerp(Node(plan.AccelCmd, t0), Node(plan.AccelCmd, t0 + 1), tfrac);

        double3 curPos = f.PointToLocal(r);
        double3 curVel = f.VecToLocal(vSrf);
        double3 fb = _s.GfoldKp * (refPos - curPos) + _s.GfoldKd * (refVel - curVel);
        double3 cmd = ff + fb; // local thrust acceleration

        double targetThrottle = Math.Clamp(cmd.Length() * mass / Math.Max(_s.GfoldThrustMax, 1.0), 0.0, 1.0);

        // Direction: clamp the command to within the pointing cone of local up. The
        // plan respects the pointing limit, but the PD feedback can tilt past it, so
        // the limit must be re-applied here or it isn't enforced on the vehicle.
        double3 dirLocal = ClampToCone(cmd, _s.GfoldPointingDeg);
        double3 targetDir = double3.Normalize(f.VecToCci(dirLocal));
        if (!double.IsFinite(targetDir.X) || !double.IsFinite(targetDir.Y) || !double.IsFinite(targetDir.Z))
            targetDir = _s.CommandDir.Length() > 0.5 ? _s.CommandDir : f.Ex;

        // First-order low-pass toward the fresh command, so feedback noise and
        // re-solve steps don't reach the engine/gimbal as chatter.
        double dt = Math.Clamp(now - _s.GfoldLastTrackTime, 0.0, 0.25);
        _s.GfoldLastTrackTime = now;
        double a = (!_s.GfoldTrackInit || _s.GfoldSmoothTau <= 1e-3)
            ? 1.0
            : 1.0 - Math.Exp(-dt / _s.GfoldSmoothTau);
        _s.GfoldTrackInit = true;

        _s.GfoldThrottle += a * (targetThrottle - _s.GfoldThrottle);
        double3 blended = _s.CommandDir.Length() > 0.5 ? _s.CommandDir + a * (targetDir - _s.CommandDir) : targetDir;
        if (blended.Length() > 1e-6)
            _s.CommandDir = double3.Normalize(blended);
        _s.HasCommand = true;
    }

    // Unit thrust direction of v, clamped to within maxAngleDeg of local up (+X).
    // If v tilts past the cone, it's pushed onto the cone surface keeping azimuth.
    private static double3 ClampToCone(double3 v, double maxAngleDeg)
    {
        double len = v.Length();
        if (len < 1e-9) return new double3(1, 0, 0);
        double3 d = v * (1.0 / len);
        double cosMax = Math.Cos(Math.Clamp(maxAngleDeg, 0.0, 90.0) * Math.PI / 180.0);
        if (d.X >= cosMax) return d;                       // already within the cone
        double sinMax = Math.Sqrt(Math.Max(1.0 - cosMax * cosMax, 0.0));
        var horiz = new double3(0, d.Y, d.Z);
        double h = horiz.Length();
        double3 hUnit = h > 1e-9 ? horiz * (1.0 / h) : new double3(0, 1, 0);
        return new double3(cosMax, sinMax * hUnit.Y, sinMax * hUnit.Z);
    }

    // A failed solve holds the last command briefly; a short run of failures gives
    // the vehicle back rather than flying a stale (often sideways) command in.
    private static void FailGfold(string message)
    {
        _s.GfoldFailStreak++;
        // A failed re-solve is not fatal once we hold a feasible plan: keep flying the
        // last committed trajectory (the solver usually only chokes on the degenerate
        // last few metres, where the existing plan lands fine) and just tell the user.
        // Only give up when there's nothing to fly — no plan was ever found.
        if (_s.GfoldPlan != null)
        {
            _s.LandingStatus = $"G-FOLD re-solve failed ({_s.GfoldFailStreak}) — flying last trajectory. {message}";
            return;
        }
        _s.LandingStatus = message;
        if (_s.GfoldFailStreak > 3)
        {
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            _s.LandingStatus = "G-FOLD found no trajectory — vehicle is yours.";
        }
    }

    // ----- G-FOLD tuning popup -----

    private static bool _showGfoldParams;

    private static void DrawGfoldParamsWindow()
    {
        if (!_showGfoldParams)
            return;

        // Note: the UPFG->G-FOLD handoff gate lives in the Deorbit sub-tab, not
        // here — it governs when the braking burn ends, which is a deorbit-phase
        // decision, not a G-FOLD tuning one.
        ImGui.Begin("G-FOLD params", ImGuiWindowFlags.AlwaysAutoResize);
        ImGui.InputDouble("Glide slope (deg)", ref _s.GfoldGlideSlopeDeg);
        ImGui.InputDouble("Thrust pointing (deg)", ref _s.GfoldPointingDeg);
        ImGui.InputDouble("Max speed (m/s)", ref _s.GfoldVMaxMs);
        ImGui.InputDouble("Solver min thrust (frac)", ref _s.GfoldThrottleMin);
        ImGui.InputDouble("Solver max thrust (frac)", ref _s.GfoldThrottleMax);
        ImGui.InputDouble("Thrust smoothing (0=off)", ref _s.GfoldSlewReg);
        ImGui.InputDouble("Re-solve interval (s)", ref _s.GfoldIntervalS);
        ImGui.InputInt("Nodes", ref _s.GfoldNodes);

        // The solver swap, in front of whoever is flying it. Clarabel is the default and
        // is GPLv3; SCS is MIT and is what an MIT release needs. They are fed the
        // identical assembled problem, so switching mid-descent compares like with
        // like — and the solve time beside the status is the number that decides it.
        //
        // The solver selector that used to sit here is gone: Clarabel is the only
        // backend now. There is no tolerance knob either, and that is a property of the
        // algorithm rather than an omission — an interior-point method's cost scales
        // with log(1/eps) rather than 1/eps, so accuracy is nearly free and the
        // tolerance stops being something a pilot should be tuning. The time limit
        // stays, because Clarabel reports hitting it as a first-class status.
        ImGui.InputDouble("Solve time limit (s, 0=none)", ref _s.GfoldSolveTimeLimitS);

        ImGui.InputDouble("Hover handoff alt (m)", ref _s.GfoldHoverHandoffAltM);
        ImGui.InputDouble("Vehicle height (m)", ref _s.VehicleHeightM);
        ImGui.InputDouble("Track gain Kp", ref _s.GfoldKp);
        ImGui.InputDouble("Track gain Kd", ref _s.GfoldKd);
        ImGui.InputDouble("Command smoothing (s)", ref _s.GfoldSmoothTau);
        if (ImGui.Button("Close"))
            _showGfoldParams = false;
        ImGui.End();
    }

    // ----- G-FOLD debug window -----

    private static bool _showGfoldDebug;

    // Every series of the committed optimal trajectory plotted against plan time,
    // with a cursor at "now" so you can watch the vehicle walk along the plan and
    // each re-solve reshape it. Reads only the committed plan — pure display.
    private static void DrawGfoldDebugWindow()
    {
        if (!_showGfoldDebug)
            return;

        ImGui.Begin("G-FOLD debug", ImGuiWindowFlags.AlwaysAutoResize);
        GfoldTrajectory plan = _s.GfoldPlan;
        if (plan == null)
        {
            ImGui.Text("No committed plan yet — appears once a G-FOLD descent solves.");
            ImGui.End();
            return;
        }

        double elapsed = Math.Clamp(SimNow() - _s.GfoldPlanStart, 0.0, plan.TimeOfFlight);
        float cursor = plan.TimeOfFlight > 1e-9 ? (float)(elapsed / plan.TimeOfFlight) : 0f;
        int n = plan.Nodes;

        // Solve cost is a first-class readout, not a diagnostic: this runs on the sim
        // thread, so anything past a frame time (16.7 ms at 60 Hz) is a stutter the
        // player feels once every re-solve interval.
        ImGui.Text($"status {_s.GfoldStatus}   nodes {n}   dt {plan.Dt:F2} s   tf {plan.TimeOfFlight:F1} s");
        ImGui.Text($"last solve {_s.GfoldSolveMs:F1} ms   " +
                   $"({_s.GfoldSolveMs / 16.7:F1} frames at 60 Hz)");
        ImGui.Text($"fuel used {plan.FuelUsed:F0} kg   landing err {plan.LandingErrorNorm:F1} m   plan t+{elapsed:F1} s");

        // Two plots per row to keep the window a sane height.
        PlotPair("r.up (m)", i => plan.Position[i][0],
                 "r.y (m)", i => plan.Position[i][1], n, cursor);
        PlotPair("r.z (m)", i => plan.Position[i][2],
                 "speed (m/s)", i => VecLen(plan.Velocity[i]), n, cursor);
        PlotPair("v.up (m/s)", i => plan.Velocity[i][0],
                 "v.y (m/s)", i => plan.Velocity[i][1], n, cursor);
        PlotPair("v.z (m/s)", i => plan.Velocity[i][2],
                 "|u| (m/s2)", i => VecLen(plan.AccelCmd[i]), n, cursor);
        PlotPair("u.up (m/s2)", i => plan.AccelCmd[i][0],
                 "u.y (m/s2)", i => plan.AccelCmd[i][1], n, cursor);
        PlotPair("u.z (m/s2)", i => plan.AccelCmd[i][2],
                 "sigma (m/s2)", i => plan.Sigma[i], n, cursor);
        PlotPair("throttle (%)", i => 100.0 * plan.Sigma[i] * plan.Mass[i] / Math.Max(_s.GfoldThrustMax, 1.0),
                 "mass (kg)", i => plan.Mass[i], n, cursor);

        ImGui.End();
    }

    private static double VecLen(double[] v)
        => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    private static void PlotPair(string labelA, Func<int, double> a,
                                 string labelB, Func<int, double> b, int n, float cursor)
    {
        PlotSeries(labelA, a, n, cursor);
        ImGui.SameLine();
        PlotSeries(labelB, b, n, cursor);
    }

    // A small draw-list line plot of one plan series vs node index (= plan time),
    // with min/max labels, the value at the time cursor, and the cursor itself.
    private static void PlotSeries(string label, Func<int, double> series, int n, float cursor)
    {
        var size = new float2(280f, 84f);
        float2 origin = ImGui.GetCursorScreenPos();
        ImGui.Dummy(size);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();

        dl.AddRectFilled(origin, origin + size, new ImColor8(16, 20, 26), 4f);
        dl.AddRect(origin, origin + size, new ImColor8(95, 100, 105), 4f);

        double min = double.PositiveInfinity, max = double.NegativeInfinity;
        for (int i = 0; i < n; i++)
        {
            double v = series(i);
            if (!double.IsFinite(v)) continue;
            if (v < min) min = v;
            if (v > max) max = v;
        }
        if (!double.IsFinite(min))
        {
            dl.AddText(origin + new float2(8f, 8f), new ImColor8(200, 120, 120), label + " (no data)");
            return;
        }
        double span = max - min;
        if (span < 1e-9) span = 1.0;

        const float padX = 8f, padTop = 18f, padBot = 8f;
        float w = size.X - 2f * padX;
        float h = size.Y - padTop - padBot;
        var points = new float2[n];
        for (int i = 0; i < n; i++)
        {
            double v = series(i);
            if (!double.IsFinite(v)) v = min;
            points[i] = new float2(
                origin.X + padX + (n > 1 ? (float)i / (n - 1) : 0f) * w,
                origin.Y + padTop + h - (float)((v - min) / span) * h);
        }
        dl.AddPolyline(points, new ImColor8(60, 220, 90), ImDrawFlags.None, 2f);

        // Time cursor + value there.
        int ci = Math.Clamp((int)MathF.Round(cursor * (n - 1)), 0, n - 1);
        float cx = origin.X + padX + cursor * w;
        _ovSeg[0] = new float2(cx, origin.Y + padTop);
        _ovSeg[1] = new float2(cx, origin.Y + padTop + h);
        dl.AddPolyline(_ovSeg, new ImColor8(255, 215, 60), ImDrawFlags.None, 1f);
        dl.AddCircleFilled(points[ci], 3f, new ImColor8(255, 215, 60));

        var axisCol = new ImColor8(160, 165, 170);
        dl.AddText(origin + new float2(padX, 3f), new ImColor8(205, 215, 225),
            $"{label}   now {series(ci):G4}");
        dl.AddText(origin + new float2(padX, padTop - 2f), axisCol, $"{max:G4}");
        dl.AddText(origin + new float2(padX, size.Y - padBot - 14f), axisCol, $"{min:G4}");
    }
}
