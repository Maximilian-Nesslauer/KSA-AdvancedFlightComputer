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

    /// <summary>Share of the vehicle's turn acceleration the plan may use, the rest left to the attitude loop that follows it - the same idea as the throttle ceilings.</summary>
    public const double TurnAccelShare = 0.75;

    /// <summary>
    /// The turn acceleration the entry guidance plans to, rad/s^2: <see cref="TurnAccelShare"/> of what KSA's flight computer says the RCS gives about the weaker lateral control axis - the authority the vehicle has with its engines off, which is how it flies the glide, and a floor under the burn, where the gimbals add theirs. Not the gimbals': how the vehicle turns is not this solver's to model (see Scvx3DofConfig.AttitudeAccelMax). 0, no limit, when the flight computer reports none.
    /// </summary>
    public static double TurnAccelLimit(Vehicle vehicle)
    {
        FlightComputer fc = vehicle?.FlightComputer;
        if (fc == null) return 0.0;
        double rcs = Math.Min(fc.RcsTorqueAuthority.Y, fc.RcsTorqueAuthority.Z);
        return rcs > 0.0 && double.IsFinite(rcs) ? TurnAccelShare * rcs : 0.0;
    }

    /// <summary>
    /// The planner's model of this vehicle over this body, in the site frame: central gravity and the frame's rotation from the body, the aerodynamics from the boostback's sweep, and the lit engines' thrust and flow sampled from the game over throttle and pressure (<see cref="EngineTable"/>).
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
            Engine = p0 > 0.0 ? EngineTable(vehicle, p0) : null,
        };
    }

    // Throttle breakpoints as fractions of the way from the minimum to full: close together near the minimum, where the nozzle's loss bends most.
    private static readonly double[] ThrottleSpacing = [0.0, 0.03, 0.07, 0.12, 0.18, 0.26, 0.35, 0.45, 0.56, 0.68, 0.84, 1.0];

    // Pressure breakpoints as fractions of sea level, a little past it for a site below the datum.
    private static readonly double[] PressureSpacing = [0.0, 0.05, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.0, 1.1];

    /// <summary>
    /// The lit engines as the game computes them, over throttle from the minimum to full and over pressure from vacuum to past sea level. Read once, here on the main thread, so the solves on the worker read nothing of the game's. See EngineThrustTable for why the constant-loss law is not enough. Null if the game returns nothing usable, which leaves the model on that law.
    /// </summary>
    public static EngineThrustTable EngineTable(Vehicle vehicle, double seaLevelPressure)
    {
        double floor = Math.Clamp(vehicle.GetMinThrottle(), 0.01, 0.95);
        var throttles = new double[ThrottleSpacing.Length];
        for (int j = 0; j < throttles.Length; j++)
            throttles[j] = floor + (1.0 - floor) * ThrottleSpacing[j];
        var pressures = new double[PressureSpacing.Length];
        for (int i = 0; i < pressures.Length; i++)
            pressures[i] = PressureSpacing[i] * seaLevelPressure;

        var thrust = new double[pressures.Length * throttles.Length];
        var flow = new double[throttles.Length];
        for (int i = 0; i < pressures.Length; i++)
            for (int j = 0; j < throttles.Length; j++)
            {
                (double t, double m) = KsaEnginePerf.UncappedAtThrottle(vehicle, throttles[j], pressures[i]);
                if (!double.IsFinite(t) || !double.IsFinite(m) || !(m > 0.0))
                    return null;
                thrust[i * throttles.Length + j] = t;
                // The flow is the chamber's, the same at every pressure; read at the first.
                if (i == 0)
                    flow[j] = m;
            }
        return new EngineThrustTable(pressures, throttles, thrust, flow);
    }

    /// <summary>
    /// The glide-and-burn config (glideIntervals &gt; 0) or the burn-only one (0). Path limits soft and the terminal soft, the weights the offline closed loop was tuned with; ground and dry mass as state floors.
    /// </summary>
    /// <param name="comFloorM">The CoM's height over the site with the legs on the ground - the vehicle height - which is where the ground is for a plan that flies the CoM.</param>
    /// <param name="accelMaxMs2">Largest thrust acceleration the burn may plan, m/s^2, or 0 for none; see Scvx3DofConfig.AccelMax.</param>
    /// <param name="turnAccelRadS2">Largest turn acceleration the plan may use, rad/s^2, or 0 for none; see Scvx3DofConfig.AttitudeAccelMax and TurnAccelLimit.</param>
    /// <param name="floorMargin">Throttle the plan keeps in hand above the engines' minimum, for the burn tracker; see VehicleAutopilotState.LandingPlanFloorMargin.</param>
    public static Scvx3DofConfig Config(Vehicle vehicle, int nodes, int glideIntervals, double alphaMaxDeg,
                                        double comFloorM = 0.0, double accelMaxMs2 = 0.0, double floorMargin = 0.0,
                                        double turnAccelRadS2 = 0.0)
    {
        var min = new double[PointMass3Dof.NX];
        var max = new double[PointMass3Dof.NX];
        Array.Fill(min, double.NegativeInfinity);
        Array.Fill(max, double.PositiveInfinity);
        min[PointMass3Dof.IR + 2] = Math.Max(comFloorM, 0.0);
        min[PointMass3Dof.IM] = Math.Max(vehicle.TotalMass - vehicle.PropellantMass, 1.0);
        double minimum = Math.Clamp(vehicle.GetMinThrottle(), 0.01, 0.95);
        double floor = Math.Clamp(minimum + Math.Max(floorMargin, 0.0), minimum, Math.Max(minimum, 0.8));
        // Never under what the minimum throttle already gives at this mass: a limit below it could only ever be met through its slack.
        double minimumAccel = floor * KsaEnginePerf.UncappedAtPressure(vehicle, 0.0).thrust / Math.Max(vehicle.TotalMass, 1.0);
        return new Scvx3DofConfig
        {
            Nodes = nodes,
            GlideIntervals = glideIntervals,
            ThrottleFloor = floor,
            AccelMax = accelMaxMs2 > 0.0 ? Math.Max(accelMaxMs2, 1.05 * minimumAccel) : 0.0,
            AlphaMaxDeg = Math.Clamp(alphaMaxDeg, 1.0, 45.0),
            StateMin = min,
            StateMax = max,
            TerminalAttitude = [0, 0, 1],
            AttitudeRateMax = RateMax,
            AttitudeAccelMax = Math.Max(turnAccelRadS2, 0.0),
            // Virtual control stays on XScale here, unlike 6-DOF's (Ksa6DofSetup.VirtualControlVelocityScaleMs). Priced at 5 m/s, the 2026-10-13 headless 7 t booster's locked burn solves ran out of ADMM iterations twelve times before ignition and handed 6-DOF a burn it had no plan for, and offline the 2 km cross-range cold solve gave up at both engine scales.
            AttitudeAnchor = true,
            PathSlackWeight = 1e3,
            // Position above speed, as the 6-DOF's (see Ksa6DofSetup.TryBuild): a squeezed burn should arrive a little fast rather than stop at rest above the pad, which a booster that cannot throttle below its weight cannot recover from.
            TerminalMissWeight = 1e5,
            TerminalSpeedWeight = 1e4,
            GlideSigmaMax = 400,
            BurnSigmaMin = 1,
            BurnSigmaMax = 120,
        };
    }
}
