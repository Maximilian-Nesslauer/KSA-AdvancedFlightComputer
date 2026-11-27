#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Ui.Logic;

public static partial class GuidanceWindow
{
    /// <summary>
    /// THE CUT SUPERVISOR's step (#335; the rule is CutSupervisor in Ui/Logic): run by GuidanceFeature after the modes have stepped, so it sees and overrides the engine command touchdown guidance left, and owns nothing else. While touchdown guidance flies a craft it holds, it reads the legs' height above the ground under the craft (LegsHeightM, a radio altitude) and the vertical speed, and when the rule cuts it turns the engine off, puts touchdown guidance into its flight to contact with no relight left, and from then on holds the engine off every step to contact. Each cut, and the first step of each shortfall, is logged with the numbers behind it.
    /// </summary>
    internal static void StepCutSupervisor(Vehicle vehicle)
    {
        if (vehicle == null || vehicle.IsDisposed || !VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState state))
            return;
        try
        {
            StepCutSupervisor(vehicle, state);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-cut-supervisor-{vehicle.Id}:{ex.GetType().Name}",
                $"[AFC] The cut supervisor failed on '{vehicle.Id}': {ex}");
        }
    }

    private static void StepCutSupervisor(Vehicle vehicle, VehicleAutopilotState s)
    {
        if (!s.CutSupervisorOn || !s.Active || s.Guidance == null
            || !VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance))
        {
            s.CutSup = default;
            s.CutSupLastT = double.NaN;
            s.CutSupAction = CutSupervisorAction.Idle;
            return;
        }

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        double now = SimNow();
        double dt = double.IsFinite(s.CutSupLastT) ? now - s.CutSupLastT : 0.0;
        s.CutSupLastT = now;

        double3 r = orbit.StateVectors.PositionCci;
        double3 up = double3.Normalize(r);
        double3 vSrf = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        double vUp = double3.Dot(vSrf, up);
        double legs = LegsHeightM(vehicle, s);
        bool contact = vehicle.Situation.HasAnyContact();
        ref ManualControlInputs manual = ref ManualInputs(vehicle);
        bool lit = manual.EngineOn && manual.EngineThrottle > 0f;

        var settings = new CutSupervisorSettings(s.CutSupervisorSinkMs, s.CutSupervisorShortfallM, s.CutSupervisorMaxHeightM);
        double before = s.CutSup.ShortfallM;
        CutSupervisorAction action = CutSupervisor.Step(ref s.CutSup, in settings, legs, vUp, dt, lit, contact);
        s.CutSupAction = action;

        switch (action)
        {
            case CutSupervisorAction.Cut:
            {
                manual.EngineOn = false;
                manual.EngineThrottle = 0f;
                s.LastThrottle = 0.0;
                // Touchdown guidance flies the rest to contact, with no relight left, so nothing it does lights the engine again.
                s.SixDofToContact = true;
                s.SixDofContactRelights = SixDofMaxContactRelights;
                s.SixDofContactEngineLit = false;
                double g = parent.Mu / r.LengthSquared();
                double3 horizontal = vSrf - vUp * up;
                string why = FormattableString.Invariant(
                    $"CUT SUPERVISOR: ENGINE CUT, legs {legs:F2} m above the ground, {vUp:F2} m/s, shortfall {s.CutSup.ShortfallM:F2} m below {s.CutSupervisorSinkMs:F2} m/s, ")
                    + FormattableString.Invariant($"{horizontal.Length():F1} m/s sideways, tilt {TiltFromUpDeg(vehicle, up):F1} deg; the drop lands at about {CutSupervisor.ImpactSpeedMs(legs, vUp, g):F1} m/s.");
                SixDofLog.Event(s, now, why);
                GuidanceLog.Info(vehicle, why);
                s.WarnError(why);
                break;
            }
            case CutSupervisorAction.HoldOff:
                manual.EngineOn = false;
                manual.EngineThrottle = 0f;
                s.LastThrottle = 0.0;
                break;
            case CutSupervisorAction.Building when before <= 0.0:
                SixDofLog.Event(s, now, FormattableString.Invariant(
                    $"CUT SUPERVISOR: legs {legs:F2} m above the ground at {vUp:F2} m/s, slower than {s.CutSupervisorSinkMs:F2} m/s down: shortfall building toward {s.CutSupervisorShortfallM:F2} m."));
                break;
        }

        // THE LOG THROUGH TO CONTACT. While the plan flies, ApplyPlanCommand writes the cycle row; after its end nothing did, and every landing's last seconds went unrecorded. A row each step from the flight to contact on.
        if (s.SixDofToContact && SixDofLog.Enabled)
            LogContactCycle(vehicle, s, parent);
    }

    /// <summary>The thrust axis's angle from <paramref name="up"/>, deg.</summary>
    private static double TiltFromUpDeg(Vehicle vehicle, double3 up)
        => Math.Acos(Math.Clamp(double3.Dot(KsaFrameBridge.ThrustAxisCci(vehicle), up), -1.0, 1.0)) * 180.0 / Math.PI;

    /// <summary>A cycle row for a craft flying to contact after its plan: the state, the throttle flown and no planned torque. Binds _s to <paramref name="s"/>, as the step it follows did.</summary>
    private static void LogContactCycle(Vehicle vehicle, VehicleAutopilotState s, IParentBody parent)
    {
        _s = s;
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame frame = KsaFrameBridge.BuildSiteFrame(siteCci);
        double[] x = KsaFrameBridge.ToModelState(vehicle, frame);
        double pressure = KsaEnginePerf.AmbientPressureAt(parent, siteCci.Length() - parent.MeanRadius + x[2]);
        ref ManualControlInputs manual = ref ManualInputs(vehicle);
        double throttle = manual.EngineOn ? manual.EngineThrottle : 0.0;
        double capability = KsaEnginePerf.ThrustAtThrottle(vehicle, 1.0, pressure);
        double thrustN = throttle > 0.0 ? KsaEnginePerf.ThrustAtThrottle(vehicle, throttle, pressure) : 0.0;
        LogCycle(vehicle, parent, siteCci, x, SimNow(), thrustN, capability, throttle, default, pressure);
    }
}
