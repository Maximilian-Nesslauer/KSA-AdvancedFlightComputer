#nullable disable

using AdvancedFlightComputer.Guidance.Gfold;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

public static partial class GuidanceWindow
{
    private const double TerminalBurnTiltDeg = 15;

    private static bool TryStartTerminalBurn(Vehicle vehicle, double altitude, double horizontalSpeed, double distance, double now)
    {
        if (!AtVerticalApproachGate(altitude, horizontalSpeed, distance)
            || !(TerminalMinThrottleTwr(vehicle, vehicle.Orbit, vehicle.Parent.Mu) > 1)) return false;
        // The engine stops before the turn, so actual tilt does not delay this handoff.
        StartTerminalBurn(vehicle, now);
        return true;
    }

    private static void StartTerminalBurn(Vehicle vehicle, double now)
    {
        _s.LandingPhase = LandingPhase.TerminalCoast;
        _s.GfoldPlan = null;
        _s.GfoldEngineOn = false;
        _s.GfoldThrottle = 0;
        _s.GfoldTrackInit = false;
        _s.TerminalBurnLastTime = now;
        _s.TerminalIgnitionHeight = 0;
        _s.TerminalBurnSeconds = 0;
        _s.LandingStatus = "Horizontal braking complete. Coasting and turning upright for the landing burn.";
        GuidanceLog.Info(vehicle, _s.LandingStatus);
    }

    private static void StepTerminalBurn(Vehicle vehicle, Orbit orbit, IParentBody parent, double now)
    {
        if (!PrepareLandingEngines(vehicle, parent, now, requireAirless: true)) return;

        double3 position = orbit.StateVectors.PositionCci;
        double3 up = position.Normalized();
        double3 velocity = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), position);
        double velocityUp = double3.Dot(velocity, up);
        double3 lateral = velocity - velocityUp * up;
        double height = TerminalHeight(orbit, parent, parent.MeanRadius);
        double gravity = parent.Mu / position.LengthSquared();
        double pressure = KsaEnginePerf.AmbientPressureAt(parent, position.Length() - parent.MeanRadius);
        double minimum = KsaEnginePerf.ThrustAtThrottle(vehicle, vehicle.GetMinThrottle(), pressure) / vehicle.TotalMass;
        double maximum = KsaEnginePerf.ThrustAtThrottle(vehicle, 1, pressure) / vehicle.TotalMass;
        double step = now - _s.TerminalBurnLastTime;
        _s.TerminalBurnLastTime = now;
        if (!(step > 0)) step = DeorbitControlStep(vehicle);

        // Reserve the full permitted tilt for lateral braking when sizing the vertical stopping distance.
        double tiltReserve = Math.Cos(TerminalBurnTiltDeg * Math.PI / 180);
        if (!(maximum * tiltReserve > gravity))
        {
            RefuseLandingEngines("The landing engine cannot stop the descent with the required tilt reserve.");
            return;
        }
        double minimumPulse = KsaEnginePerf.MinimumPulse(vehicle);
        TerminalBraking.Command command = TerminalBraking.Evaluate(height, velocityUp, gravity,
            minimum * tiltReserve, maximum * tiltReserve, step, minimumPulse,
            _s.LandingPhase == LandingPhase.TerminalBrake);
        _s.TerminalIgnitionHeight = command.IgnitionHeight;
        _s.TerminalBurnSeconds = command.BurnSeconds;
        _s.GfoldAltM = height;
        _s.GfoldSpeedMs = velocity.Length();

        double3 site = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        double3 local = KsaGfold.BuildFrame(site).PointToLocal(position);
        RecordGfoldTrace(HorizontalLength(local), local.X);

        // Coast attitude anticipates the braking direction, so ignition does not start a turn.
        double vertical = command.EngineOn ? command.Acceleration : Math.Max(gravity, minimum);
        double3 lateralCommand = -lateral / Math.Max(command.BurnSeconds, 1);
        double lateralLimit = vertical * Math.Tan(TerminalBurnTiltDeg * Math.PI / 180);
        if (lateralCommand.Length() > lateralLimit)
            lateralCommand *= lateralLimit / lateralCommand.Length();
        _s.CommandDir = (vertical * up + lateralCommand).Normalized();
        _s.HasCommand = true;

        double3 thrustAxis = ThrustAxisCci(vehicle);
        double actualUp = double3.Dot(thrustAxis, up);
        double alignment = AngleBetween(thrustAxis, _s.CommandDir) * 180 / Math.PI;
        bool ignite = command.EngineOn && actualUp >= tiltReserve && alignment <= TerminalBurnTiltDeg;
        if ((minimum * actualUp - gravity) * Math.Max(step, minimumPulse) >= -velocityUp)
            ignite = false;
        // The control law keeps its minimum below the engine minimum by the tilt reserve, so a burn it asks for is raised to the real minimum instead of being refused.
        KsaEnginePerf.ThrustCommand thrust = ignite
            ? KsaEnginePerf.CommandForThrust(vehicle, Math.Max(command.Acceleration / actualUp, minimum) * vehicle.TotalMass, pressure)
            : default;
        // The measured attitude can make the real minimum too large even when the reserved cone fitted.
        if (thrust.Status == KsaEnginePerf.ThrustStatus.BelowMinimum) ignite = false;
        _s.GfoldThrottle = ignite ? thrust.Throttle : 0;
        _s.GfoldThrustStatus = command.EngineOn && !ignite
            ? thrust.Status == KsaEnginePerf.ThrustStatus.BelowMinimum ? thrust.Message : "Aligning for the landing burn."
            : thrust.Message;
        LandingPhase next = ignite ? LandingPhase.TerminalBrake : LandingPhase.TerminalCoast;
        if (_s.LandingPhase != next)
            GuidanceLog.Info(vehicle, $"terminal {next}: height {height:F2} m, sink {-velocityUp:F2} m/s, horizontal {lateral.Length():F2} m/s, ignition height {command.IgnitionHeight:F2} m.");
        _s.LandingPhase = next;
        _s.LandingStatus = ignite ? "Final landing burn. Braking to touchdown."
            : "Engine off. Falling to the landing burn.";
        if (GuidanceLog.Enabled && now - _s.LastGuidanceLogTime >= 1)
        {
            _s.LastGuidanceLogTime = now;
            GuidanceLog.Debug(vehicle, $"terminal landing: height {height:F2} m, sink {-velocityUp:F2} m/s, horizontal {lateral.Length():F2} m/s"
                + $", ignition height {command.IgnitionHeight:F2} m, minimum TWR {minimum / gravity:F2}, throttle {_s.GfoldThrottle:F3}, tilt {Math.Acos(Math.Clamp(actualUp, -1, 1)) * 180 / Math.PI:F1} deg.");
        }
    }
}
