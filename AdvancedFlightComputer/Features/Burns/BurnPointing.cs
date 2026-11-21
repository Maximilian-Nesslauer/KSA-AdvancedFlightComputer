using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

namespace AdvancedFlightComputer.Features.Burns;

/// <summary>
/// Whether stock AUTO can turn a craft to the burn attitude with RCS, and whether the burn of the next node lasts long
/// enough for the engine to settle its aim. It reads the values that FlightComputer.ComputeControl left from the last
/// physics step of the craft, so a craft that was just built or staged needs one step first.
/// </summary>
internal static class BurnPointing
{
    // FlightComputer.SolveMinimumDurationThrottleCap settles the gimbal loop over this control period to within 0.1 deg.
    private const double GimbalControlPeriod = 0.1;
    private const double GimbalSettleTolerance = Math.PI / 1800;

    /// <summary>The warning for the next node of <paramref name="craft"/> before stock AUTO flies it, or none.</summary>
    public static PointingCheck CheckNextNode(Vehicle craft)
    {
        FlightComputer? computer = craft.FlightComputer;
        if (computer == null || computer.BurnPlan.FindFirstExecutableBurn() is not Burn node)
            return PointingCheck.None;
        BurnTarget? loaded = StockBurnIdentity.IsLoaded(computer.Burn, node) ? computer.Burn : null;
        double deltaV = loaded != null ? loaded.DeltaVTargetCci.Length() : node.DeltaVVlf.Length();
        float? planned = loaded != null ? loaded.PlannedThrottle : node.PlannedThrottle;
        double burnSeconds = BurnTurnRules.FlownSeconds(FullThrottleSeconds(computer, deltaV), planned, computer.ActiveEnginePerformanceMax.MinThrottle);
        return Check(computer, burnSeconds);
    }

    public static PointingCheck Check(FlightComputer computer, double burnSeconds)
    {
        // Before ignition FlightComputer.UpdateTvcParams gives an unlit engine no torque, so
        // FlightComputer.UpdateActiveControlSystems leaves pitch and yaw to RCS or to nothing.
        AttitudeControlSystem pitch = computer.ActiveControlSystem.Y;
        AttitudeControlSystem yaw = computer.ActiveControlSystem.Z;
        bool gimbalSteers = pitch == AttitudeControlSystem.Tvc || yaw == AttitudeControlSystem.Tvc;
        bool rcs = pitch == AttitudeControlSystem.Rcs && yaw == AttitudeControlSystem.Rcs;
        return BurnTurnRules.Classify(gimbalSteers, rcs, computer.RCSMode != FlightComputerRCSMode.Enabled, computer.AngleDeadband,
            RestingStallEdge(computer), burnSeconds, MinimumBurnSeconds(computer));
    }

    /// <summary>The largest pitch or yaw error, in radians, at which stock leaves a craft at rest unturned toward a burn attitude.</summary>
    public static double RestingStallEdge(FlightComputer computer)
        => Math.Max(BurnTurnRules.StallEdge(computer.AngleDeadband, computer.RateBit.Y, computer.RcsTorqueAuthority.Y),
            BurnTurnRules.StallEdge(computer.AngleDeadband, computer.RateBit.Z, computer.RcsTorqueAuthority.Z));

    /// <summary>
    /// The full-throttle burn length below which FlightComputer.SolveMinimumDurationThrottleCap lowers the throttle so that
    /// the engine can steer out the pointing error before cutoff. TvcTorqueAuthorityMax is set from the maximum thrust of
    /// the gimbals, so it holds before ignition too (FlightComputer.UpdateTvcParams).
    /// </summary>
    public static double MinimumBurnSeconds(FlightComputer computer)
    {
        float authority = MathF.Min(computer.TvcTorqueAuthorityMax.Y, computer.TvcTorqueAuthorityMax.Z);
        float frequency = MathF.Min(computer.TvcNaturalFrequency.Y, computer.TvcNaturalFrequency.Z);
        double pointingError = MathF.Sqrt(2f) * computer.AngleDeadband;
        double settle = FlightComputer.ComputeTvcTrackSettleDuration(GimbalControlPeriod, authority, frequency, pointingError, GimbalSettleTolerance);
        return BurnTurnRules.MinimumBurnSeconds(authority, pointingError, settle);
    }

    // The same rocket equation as FlightComputer.ComputeBurnDuration, which is internal.
    private static double FullThrottleSeconds(FlightComputer computer, double deltaV)
    {
        ActiveEnginePerformance full = computer.ActiveEnginePerformanceMax;
        double exhaust = full.ExhaustVelocity;
        double mass = computer.TotalMassPropsBody.Mass;
        if (!(full.MassFlowRate > 0f) || !(exhaust > 0) || !(mass > 0))
            return 0.0;
        return mass * (1.0 - Math.Exp(-deltaV / exhaust)) / full.MassFlowRate;
    }
}
