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
    // Descent profile (all tuning lives in the Adjust-params window).
    private static double _termTouchdownRate = 0.5;  // m/s, constant final descent
    private static double _termConstAltM = 0.5;      // constant-rate zone height
    private static double _termQuadK = 0.01;         // m^-1 s^-1: v = touch + k·(h-h0)²
    private static double _termMaxDescRate = 15.0;   // profile cap
    private static double _termMaxTiltDeg = 20.0;    // thrust tilt limit off vertical
    // PID gains: velocity error (m/s) -> commanded acceleration (m/s²).
    private static double _termKpV = 1.5, _termKiV = 0.0, _termKdV = 0.0;
    private static double _termKpL = 0.10, _termKiL = 0.0, _termKdL = 0.0;
    private const double TermILimit = 3.0;           // integrator clamp, m/s²
    private static double _termNudgeStep = 0.5;      // m/s per key press / click

    // Velocity setpoint offsets (m/s): east/north lateral targets, and a vertical
    // bias added on top of the descent profile (positive slows/reverses descent).
    private static double _termSetE, _termSetN, _termSetUp;

    private struct Pid { public double I, PrevErr; }
    private static Pid _termPidUp, _termPidE, _termPidN;
    private static double _termLastTime;
    private static bool _termInit;

    private static bool _termTabSelectPending;

    private static void StartTerminalHover()
    {
        _engage = true;
        _autoStage = true;
        _running = false;
        _landingPhase = LandingPhase.TerminalHover;
        _termPidUp = _termPidE = _termPidN = default;
        _termInit = false;
        _termSetE = _termSetN = _termSetUp = 0.0;
        _hasCommand = false;
        _termTabSelectPending = true;
        _landingStatus = "Terminal hover engaged.";
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
        return r.Length() - (bodyRadius + terrain) - _vehicleHeightM;
    }

    // Per-frame terminal-hover control, dispatched from StepLanding.
    private static void StepTerminalHover(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                          double mu, double bodyRadius, double now)
    {
        if (_engage && _autoStage)
            AutoSequence(vehicle);

        double3 r = orbit.StateVectors.PositionCci;
        double3 up = double3.Normalize(r);
        double3 vSrf = orbit.StateVectors.VelocityCci
                     - double3.Cross(parent.GetAngularVelocityCci(), r);
        double h = TerminalHeight(orbit, parent, bodyRadius);
        _gfoldAltM = h;
        _gfoldSpeedMs = vSrf.Length();

        double vUp = double3.Dot(vSrf, up);
        if (h <= 0.05 && vUp > -2.0 * Math.Max(_termTouchdownRate, 0.5))
        {
            _gfoldThrottle = 0.0;
            _landingPhase = LandingPhase.Done;
            _landingCutPending = true;
            _landingStatus = $"Terminal hover touchdown ({_gfoldSpeedMs:F1} m/s).";
            return;
        }

        // Descent-rate setpoint from the quadratic profile, plus the user's
        // vertical nudge bias. Above _termConstAltM the rate grows with height
        // squared (gentle flare); inside it the rate is constant for touchdown.
        double dh = Math.Max(h - _termConstAltM, 0.0);
        double vDesc = Math.Min(_termTouchdownRate + _termQuadK * dh * dh, _termMaxDescRate);
        double vSetUp = -vDesc + _termSetUp;

        (double3 east, double3 north) = EnuBasis(up);
        double vE = double3.Dot(vSrf, east);
        double vN = double3.Dot(vSrf, north);

        double dt = Math.Clamp(now - _termLastTime, 0.0, 0.25);
        _termLastTime = now;
        if (!_termInit)
        {
            dt = 0.0;
            _termInit = true;
        }

        double g = mu / (r.Length() * r.Length());
        double aUp = g + StepPid(ref _termPidUp, vSetUp - vUp, _termKpV, _termKiV, _termKdV, dt);
        double aE = StepPid(ref _termPidE, _termSetE - vE, _termKpL, _termKiL, _termKdL, dt);
        double aN = StepPid(ref _termPidN, _termSetN - vN, _termKpL, _termKiL, _termKdL, dt);

        // Local (x = up) command: throttle from the magnitude, direction clamped
        // to the tilt cone so lateral authority never flips the vehicle over.
        var local = new double3(Math.Max(aUp, 0.0), aE, aN);
        double thrustMax = KsaEnginePerf.VacuumThrust(vehicle);
        _gfoldThrottle = thrustMax > 0
            ? Math.Clamp(local.Length() * vehicle.TotalMass / thrustMax, 0.0, 1.0)
            : 0.0;
        double3 dirLocal = ClampToCone(local, _termMaxTiltDeg);
        double3 dir = dirLocal.X * up + dirLocal.Y * east + dirLocal.Z * north;
        if (dir.Length() > 1e-6)
        {
            _commandDir = double3.Normalize(dir);
            _hasCommand = true;
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
        bool active = _landingPhase == LandingPhase.TerminalHover;

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
            ImGui.Text($"Alt (legs) {h,7:F1} m    v-up {double3.Dot(vSrf, up),6:F1} m/s    throttle {_gfoldThrottle * 100,4:F0} %");
            ImGui.Text($"v East {double3.Dot(vSrf, east),6:F1}  v North {double3.Dot(vSrf, north),6:F1} m/s");

            if (ImGui.Button("Abort (engines off)"))
                AbortLanding();
        }

        // --- Setpoint nudges: numpad while hovering, or the buttons below ---
        ImGui.SeparatorText("Velocity setpoints (m/s)");
        ImGui.Text($"East {_termSetE,6:F1}   North {_termSetN,6:F1}   Vertical bias {_termSetUp,6:F1}");

        if (active)
        {
            // Numpad nudges (repeat on hold). Numpad keys are unlikely to clash
            // with game bindings; the buttons below always work regardless.
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad8)) _termSetN += _termNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad2)) _termSetN -= _termNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad6)) _termSetE += _termNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad4)) _termSetE -= _termNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad9)) _termSetUp += _termNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad3)) _termSetUp -= _termNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad5, false)) ZeroTerminalSetpoints();
        }

        var pad = new float2(70f, 26f);
        ImGui.Dummy(new float2(78f, 1f)); ImGui.SameLine();
        if (ImGui.Button("N (8)", pad)) _termSetN += _termNudgeStep;
        if (ImGui.Button("W (4)", pad)) _termSetE -= _termNudgeStep;
        ImGui.SameLine();
        if (ImGui.Button("ZERO (5)", pad)) ZeroTerminalSetpoints();
        ImGui.SameLine();
        if (ImGui.Button("E (6)", pad)) _termSetE += _termNudgeStep;
        ImGui.Dummy(new float2(78f, 1f)); ImGui.SameLine();
        if (ImGui.Button("S (2)", pad)) _termSetN -= _termNudgeStep;
        if (ImGui.Button("Up (9)", pad)) _termSetUp += _termNudgeStep;
        ImGui.SameLine();
        if (ImGui.Button("Down (3)", pad)) _termSetUp -= _termNudgeStep;

        ImGui.Text("Numpad: 8/2 N/S, 4/6 W/E, 9/3 up/down, 5 zero (while hovering).");

        if (ImGui.Button(_showTermParams ? "Close params" : "Hover params..."))
            _showTermParams = !_showTermParams;
    }

    private static void ZeroTerminalSetpoints()
    {
        _termSetE = 0.0;
        _termSetN = 0.0;
        _termSetUp = 0.0;
    }

    // Terminal-hover tuning, in its own popup (no ascent/other params mixed in).
    private static bool _showTermParams;

    private static void DrawTermParamsWindow()
    {
        if (!_showTermParams)
            return;

        ImGui.Begin("Hover params", ImGuiWindowFlags.AlwaysAutoResize);
        ImGui.InputDouble("Touchdown rate (m/s)", ref _termTouchdownRate);
        ImGui.InputDouble("Constant-rate zone (m)", ref _termConstAltM);
        ImGui.InputDouble("Profile quad k (1/(m s))", ref _termQuadK);
        ImGui.InputDouble("Max descent rate (m/s)", ref _termMaxDescRate);
        ImGui.InputDouble("Max tilt (deg)", ref _termMaxTiltDeg);
        ImGui.InputDouble("Vertical Kp", ref _termKpV);
        ImGui.InputDouble("Vertical Ki", ref _termKiV);
        ImGui.InputDouble("Vertical Kd", ref _termKdV);
        ImGui.InputDouble("Lateral Kp", ref _termKpL);
        ImGui.InputDouble("Lateral Ki", ref _termKiL);
        ImGui.InputDouble("Lateral Kd", ref _termKdL);
        ImGui.InputDouble("Nudge step (m/s)", ref _termNudgeStep);
        if (ImGui.Button("Close"))
            _showTermParams = false;
        ImGui.End();
    }
}
