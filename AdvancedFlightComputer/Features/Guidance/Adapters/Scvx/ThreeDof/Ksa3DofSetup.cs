#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Measures the live vehicle and body into the 3-DOF glide-and-burn problem: the point-mass model in the landing-site frame, and the solver configs. The 3-DOF's counterpart of Ksa6DofSetup.
/// </summary>
public static class Ksa3DofSetup
{
    /// <summary>Attitude rate the plans may ask of the vehicle, rad/s.</summary>
    public const double RateMax = 10.0 * Math.PI / 180.0;

    /// <summary>
    /// The planner's model of this vehicle over this body, in the site frame: central gravity and the frame's rotation from the body, the aerodynamics from the boostback's sweep, and the lit engines' thrust law - vacuum thrust and the nozzle's back-pressure loss - measured from the game at vacuum and at sea level.
    /// </summary>
    public static KsaPointMassModel BuildModel(Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame,
                                               KsaAeroSweep.Result aero, out string error)
    {
        error = "";
        (double vacuum, double flow) = KsaEnginePerf.UncappedAtPressure(vehicle, 0.0);
        if (!(vacuum > 0.0) || !(flow > 0.0))
        {
            error = "No active, supplied engine to plan a landing burn with.";
            return null;
        }
        double p0 = aero.Atmosphere?.SeaLevelPressure ?? 0.0;
        double seaLevel = p0 > 0.0 ? KsaEnginePerf.UncappedAtPressure(vehicle, p0).thrust : vacuum;
        double exitArea = p0 > 0.0 ? Math.Max((vacuum - seaLevel) / p0, 0.0) : 0.0;

        double3 centre = frame.PosToLocal(double3.Zero);
        double3 omega = frame.VecToLocal(parent.GetAngularVelocityCci());
        AeroTable lift = aero.LiftTable != null
            ? KsaPointMassModel.LiftSlopeTable(aero.MachGrid, aero.AlphaGridDeg, aero.LiftTable)
            : null;
        return new KsaPointMassModel
        {
            Mu = parent.Mu,
            MeanRadius = parent.MeanRadius,
            CentreX = centre.X, CentreY = centre.Y, CentreZ = centre.Z,
            OmegaX = omega.X, OmegaY = omega.Y, OmegaZ = omega.Z,
            Atmosphere = aero.Atmosphere,
            Drag = aero.Table,
            LiftSlope = lift,
            ReferenceArea = aero.ReferenceArea,
            VacuumThrust = vacuum,
            ExitArea = exitArea,
            MassFlow = flow,
        };
    }

    /// <summary>
    /// The glide-and-burn config (glideIntervals &gt; 0) or the burn-only one (0). Path limits soft and the terminal soft, the weights the offline closed loop was tuned with; ground and dry mass as state floors.
    /// </summary>
    public static Scvx3DofConfig Config(Vehicle vehicle, int nodes, int glideIntervals, double alphaMaxDeg)
    {
        var min = new double[PointMass3Dof.NX];
        var max = new double[PointMass3Dof.NX];
        Array.Fill(min, double.NegativeInfinity);
        Array.Fill(max, double.PositiveInfinity);
        min[PointMass3Dof.IR + 2] = 0.0;
        min[PointMass3Dof.IM] = Math.Max(vehicle.TotalMass - vehicle.PropellantMass, 1.0);
        return new Scvx3DofConfig
        {
            Nodes = nodes,
            GlideIntervals = glideIntervals,
            ThrottleFloor = Math.Clamp(vehicle.GetMinThrottle(), 0.01, 0.95),
            AlphaMaxDeg = Math.Clamp(alphaMaxDeg, 1.0, 45.0),
            StateMin = min,
            StateMax = max,
            TerminalAttitude = [0, 0, 1],
            AttitudeRateMax = RateMax,
            AttitudeAnchor = true,
            PathSlackWeight = 1e3,
            TerminalMissWeight = 1e3,
            TerminalSpeedWeight = 1e4,
            GlideSigmaMax = 400,
            BurnSigmaMin = 1,
            BurnSigmaMax = 120,
        };
    }
}
