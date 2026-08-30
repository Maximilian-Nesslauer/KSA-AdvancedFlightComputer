using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// Terminal hover: a simple pilot-in-the-loop final descent. Nulls the rates
// (velocity setpoints start at zero laterally), then descends on a quadratic
// velocity profile — v = touchdown rate + k·(h − h0)² — dropping to a constant
// touchdown rate for the last h0 metres. A per-axis PID on vertical and lateral
// velocity turns the setpoint errors into a thrust command (gravity fed forward
// on the vertical axis). The velocity setpoints can be nudged around with the
// numpad (8/2 = N/S, 4/6 = W/E, 9/3 = up/down, 5 = zero) or the on-screen
// buttons, so the touchdown point can be steered by eye.
public static partial class PoweredGuidanceWindow
{
    // The descent profile, the PID gains, the integrator state and the player's
    // velocity setpoints are all per vehicle (VehicleAutopilotState) — a hovering
    // craft's integrators and nudged setpoints are the last thing that should follow
    // the camera to another vehicle.
    private const double TermILimit = 3.0;           // integrator clamp, m/s²

    // Public because the per-vehicle state holds three of these.
    public struct Pid { public double I, PrevErr; }

    private static void StartTerminalHover()
    {
        _s.Engage = true;
        _s.AutoStage = true;
        _s.Running = false;
        // Continue the flown path when hover takes over from G-FOLD - it is the same
        // descent - but start clean when it is engaged cold, so the plot is not
        // showing a previous attempt.
        if (_s.LandingPhase != LandingPhase.GfoldDescent)
            ResetGfoldTrace();
        _s.LandingPhase = LandingPhase.TerminalHover;
        _s.TermPidUp = _s.TermPidE = _s.TermPidN = default;
        _s.TermInit = false;
        _s.TermSetE = _s.TermSetN = _s.TermSetUp = 0.0;
        _s.HasCommand = false;
        _s.TermTabSelectPending = true;
        _s.LandingStatus = "Terminal hover engaged.";
    }

    // Thrust-to-weight at the current mass and local gravity — hover needs > 1.
    private static double TerminalTwr(Vehicle vehicle, Orbit orbit, double mu)
    {
        double thrustMax = KsaEnginePerf.VacuumThrust(vehicle);
        double rLen = orbit.StateVectors.PositionCci.Length();
        double g = mu / (rLen * rLen);
        double weight = vehicle.TotalMass * g;
        return weight > 0 ? thrustMax / weight : 0.0;
    }

    // Height of the touchdown plane above the terrain DIRECTLY BELOW the vehicle
    // (not the landing site — a hover can drift anywhere), minus the vehicle
    // height so zero means legs on the ground.
    private static double TerminalHeight(Orbit orbit, IParentBody parent, double bodyRadius)
    {
        double3 r = orbit.StateVectors.PositionCci;
        double3 dirCcf = double3.Normalize(r).Transform(parent.GetCci2Ccf());
        double terrain = (parent as Celestial)?.GetTerrainHeightFromDirCcf(dirCcf) ?? 0.0;
        if (!double.IsFinite(terrain))
            terrain = 0.0;
        return r.Length() - (bodyRadius + terrain) - _s.VehicleHeightM;
    }

    // Per-frame terminal-hover control, dispatched from StepLanding.
    private static void StepTerminalHover(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                          double mu, double bodyRadius, double now)
    {
        if (_s.Engage && _s.AutoStage)
            AutoSequence(vehicle);

        double3 r = orbit.StateVectors.PositionCci;
        double3 up = double3.Normalize(r);
        double3 vSrf = orbit.StateVectors.VelocityCci
                     - double3.Cross(parent.GetAngularVelocityCci(), r);
        double h = TerminalHeight(orbit, parent, bodyRadius);
        _s.GfoldAltM = h;
        _s.GfoldSpeedMs = vSrf.Length();

        // Same trace the G-FOLD plot draws: hover is the last stretch of the same
        // descent, so the flown path carries straight on rather than starting again.
        double3 padCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        double3 padLocal = KsaGfold.BuildFrame(padCci).PointToLocal(r);   // X-up frame
        RecordGfoldTrace(Math.Sqrt(padLocal.Y * padLocal.Y + padLocal.Z * padLocal.Z), padLocal.X);

        double vUp = double3.Dot(vSrf, up);

        // No altitude-based cutoff here. Touchdown is decided solely by KSA's own
        // contact flag, in StepLanding — see HasTouchedDown. The old test cut at
        // h <= 0.05 m, but h is a terrain-height sample minus an assumed vehicle
        // height, so it could sit above zero with the legs already down (or trip
        // early over rough ground). The hover just keeps flying the profile until
        // something actually touches.

        // Descent-rate setpoint from the quadratic profile, plus the user's
        // vertical nudge bias. Above _s.TermConstAltM the rate grows with height
        // squared (gentle flare); inside it the rate is constant for touchdown.
        double dh = Math.Max(h - _s.TermConstAltM, 0.0);
        double vDesc = Math.Min(_s.TermTouchdownRate + _s.TermQuadK * dh * dh, _s.TermMaxDescRate);
        double vSetUp = -vDesc + _s.TermSetUp;

        (double3 east, double3 north) = EnuBasis(up);
        double vE = double3.Dot(vSrf, east);
        double vN = double3.Dot(vSrf, north);

        double dt = Math.Clamp(now - _s.TermLastTime, 0.0, 0.25);
        _s.TermLastTime = now;
        if (!_s.TermInit)
        {
            dt = 0.0;
            _s.TermInit = true;
        }

        double g = mu / (r.Length() * r.Length());
        double aUp = g + StepPid(ref _s.TermPidUp, vSetUp - vUp, _s.TermKpV, _s.TermKiV, _s.TermKdV, dt);
        double aE = StepPid(ref _s.TermPidE, _s.TermSetE - vE, _s.TermKpL, _s.TermKiL, _s.TermKdL, dt);
        double aN = StepPid(ref _s.TermPidN, _s.TermSetN - vN, _s.TermKpL, _s.TermKiL, _s.TermKdL, dt);

        // Local (x = up) command: throttle from the magnitude, direction clamped
        // to the tilt cone so lateral authority never flips the vehicle over.
        var local = new double3(Math.Max(aUp, 0.0), aE, aN);
        double thrustMax = KsaEnginePerf.VacuumThrust(vehicle);
        _s.GfoldThrottle = thrustMax > 0
            ? Math.Clamp(local.Length() * vehicle.TotalMass / thrustMax, 0.0, 1.0)
            : 0.0;
        double3 dirLocal = ClampToCone(local, _s.TermMaxTiltDeg);
        double3 dir = dirLocal.X * up + dirLocal.Y * east + dirLocal.Z * north;
        if (dir.Length() > 1e-6)
        {
            _s.CommandDir = double3.Normalize(dir);
            _s.HasCommand = true;
        }
    }

    private static double StepPid(ref Pid s, double err, double kp, double ki, double kd, double dt)
    {
        s.I = Math.Clamp(s.I + ki * err * dt, -TermILimit, TermILimit);
        double d = (dt > 1e-6 && kd > 0) ? kd * (err - s.PrevErr) / dt : 0.0;
        s.PrevErr = err;
        return kp * err + s.I + d;
    }

    // The Terminal sub-tab: TWR check, live readout, and the setpoint nudge pad.
    private static void DrawTerminalTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                        double mu, double bodyRadius)
    {
        bool active = _s.LandingPhase == LandingPhase.TerminalHover;

        double twr = TerminalTwr(vehicle, orbit, mu);
        if (twr < 1.0)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"TWR {twr:F2} < 1 — hover NOT possible (thrust cannot hold weight).");
        else
            ImGui.Text($"TWR {twr:F2} (local gravity)");

        if (!active)
        {
            if (ImGui.Button("Start terminal hover now", new float2(360f, 40f)))
                StartTerminalHover();
            ImGui.Text("Hover takes over from G-FOLD (checkbox there) or right here.");
        }
        else
        {
            double h = TerminalHeight(orbit, parent, bodyRadius);
            double3 r = orbit.StateVectors.PositionCci;
            double3 up = double3.Normalize(r);
            double3 vSrf = orbit.StateVectors.VelocityCci
                         - double3.Cross(parent.GetAngularVelocityCci(), r);
            (double3 east, double3 north) = EnuBasis(up);
            ImGui.Text($"Alt (legs) {h,7:F1} m    v-up {double3.Dot(vSrf, up),6:F1} m/s    throttle {_s.GfoldThrottle * 100,4:F0} %");
            ImGui.Text($"v East {double3.Dot(vSrf, east),6:F1}  v North {double3.Dot(vSrf, north),6:F1} m/s");

            if (ImGui.Button("Abort (engines off)"))
                AbortLanding();
        }

        // --- Setpoint nudges: numpad while hovering, or the buttons below ---
        ImGui.SeparatorText("Velocity setpoints (m/s)");
        ImGui.Text($"East {_s.TermSetE,6:F1}   North {_s.TermSetN,6:F1}   Vertical bias {_s.TermSetUp,6:F1}");

        if (active)
        {
            // Numpad nudges (repeat on hold). Numpad keys are unlikely to clash
            // with game bindings; the buttons below always work regardless.
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad8)) _s.TermSetN += _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad2)) _s.TermSetN -= _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad6)) _s.TermSetE += _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad4)) _s.TermSetE -= _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad9)) _s.TermSetUp += _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad3)) _s.TermSetUp -= _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad5, false)) ZeroTerminalSetpoints();
        }

        var pad = new float2(70f, 26f);
        ImGui.Dummy(new float2(78f, 1f)); ImGui.SameLine();
        if (ImGui.Button("N (8)", pad)) _s.TermSetN += _s.TermNudgeStep;
        if (ImGui.Button("W (4)", pad)) _s.TermSetE -= _s.TermNudgeStep;
        ImGui.SameLine();
        if (ImGui.Button("ZERO (5)", pad)) ZeroTerminalSetpoints();
        ImGui.SameLine();
        if (ImGui.Button("E (6)", pad)) _s.TermSetE += _s.TermNudgeStep;
        ImGui.Dummy(new float2(78f, 1f)); ImGui.SameLine();
        if (ImGui.Button("S (2)", pad)) _s.TermSetN -= _s.TermNudgeStep;
        if (ImGui.Button("Up (9)", pad)) _s.TermSetUp += _s.TermNudgeStep;
        ImGui.SameLine();
        if (ImGui.Button("Down (3)", pad)) _s.TermSetUp -= _s.TermNudgeStep;

        ImGui.Text("Numpad: 8/2 N/S, 4/6 W/E, 9/3 up/down, 5 zero (while hovering).");

        if (ImGui.Button(_showTermParams ? "Close params" : "Hover params..."))
            _showTermParams = !_showTermParams;
    }

    private static void ZeroTerminalSetpoints()
    {
        _s.TermSetE = 0.0;
        _s.TermSetN = 0.0;
        _s.TermSetUp = 0.0;
    }

    // Terminal-hover tuning, in its own popup (no ascent/other params mixed in).
    private static bool _showTermParams;

    private static void DrawTermParamsWindow()
    {
        if (!_showTermParams)
            return;

        ImGui.Begin("Hover params", ImGuiWindowFlags.AlwaysAutoResize);
        ImGui.InputDouble("Touchdown rate (m/s)", ref _s.TermTouchdownRate);
        ImGui.InputDouble("Constant-rate zone (m)", ref _s.TermConstAltM);
        ImGui.InputDouble("Profile quad k (1/(m s))", ref _s.TermQuadK);
        ImGui.InputDouble("Max descent rate (m/s)", ref _s.TermMaxDescRate);
        ImGui.InputDouble("Max tilt (deg)", ref _s.TermMaxTiltDeg);
        ImGui.InputDouble("Vertical Kp", ref _s.TermKpV);
        ImGui.InputDouble("Vertical Ki", ref _s.TermKiV);
        ImGui.InputDouble("Vertical Kd", ref _s.TermKdV);
        ImGui.InputDouble("Lateral Kp", ref _s.TermKpL);
        ImGui.InputDouble("Lateral Ki", ref _s.TermKiL);
        ImGui.InputDouble("Lateral Kd", ref _s.TermKdL);
        ImGui.InputDouble("Nudge step (m/s)", ref _s.TermNudgeStep);
        if (ImGui.Button("Close"))
            _showTermParams = false;
        ImGui.End();
    }
}
