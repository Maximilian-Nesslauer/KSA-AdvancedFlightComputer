using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static partial class RcsExecutor
{
    #region Estimates

    /// <summary>The cost is measured in kg per N s of net impulse along uCtrl. Each demanded group contributes its direction weight multiplied by its mass flow per unit force. Unusable groups are excluded to match worker suppression.</summary>
    private static double GroupCostPerNs(in RcsCapabilitySnapshot cap, double3 uCtrl)
    {
        Span<double> weight = stackalloc double[6]
        {
            Math.Max(uCtrl.X, 0.0), Math.Max(-uCtrl.X, 0.0),
            Math.Max(uCtrl.Y, 0.0), Math.Max(-uCtrl.Y, 0.0),
            Math.Max(uCtrl.Z, 0.0), Math.Max(-uCtrl.Z, 0.0),
        };
        double cost = 0.0;
        for (int i = 0; i < 6; i++)
        {
            // Use the same component threshold as TryHoldPerformance.
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            if (g.IsUsable)
                cost += weight[i] * g.MassFlowKgS / g.ForceN;
        }
        return cost;
    }

    public static RcsEstimates ComputeEstimates(
        Vehicle vehicle, BurnTarget bt, in RcsCapabilitySnapshot cap)
    {
        RcsEstimates est = default;
        est.AlignAxis = -1;
        float3 togo = bt.DeltaVToGoCci;
        float dv = togo.Length();
        if (dv <= 0f || !cap.HasAnyTranslation)
            return est;
        float mass = vehicle.FlightComputer.TotalMassPropsBody.Mass;
        est.Valid = true;

        // The weakest required axis limits net force. Each group contributes its duty scaled mass flow.
        double3 uCtrl = double3.Unpack(togo)
            .Transform(vehicle.GetCtrl2Cci().Inverse()).NormalizeOrZero();
        est.HoldFeasible = TryHoldPerformance(in cap, uCtrl, out double holdForce, out double holdMassFlow);
        if (est.HoldFeasible && holdForce > 0.0)
        {
            est.HoldDurationSec = mass * dv / holdForce;
            est.HoldPropellantKg = est.HoldDurationSec * holdMassFlow
                + mass * dv * GroupAttitudeFightPerImpulse(in cap, uCtrl);
        }

        // Align combines translation along the strongest axis with an estimate of the propellant needed to turn and stop.
        int best = cap.BestAxis();
        if (best >= 0)
        {
            RcsAxisGroup g = cap.Get(best);
            est.AlignAxis = best;
            est.AlignFeasible = true;
            est.AlignDurationSec = mass * dv / g.ForceN;
            // Aligned translation still incurs the selected group residual torque.
            est.AlignPropellantKg = est.AlignDurationSec * g.MassFlowKgS
                + mass * dv * GroupAttitudeFightPerImpulse(
                    in cap, double3.Unpack(RcsCapabilitySnapshot.AxisDirection(best)));

            double3 axisCci = double3.Unpack(RcsCapabilitySnapshot.AxisDirection(best))
                .Transform(vehicle.GetCtrl2Cci());
            double3 uCci = double3.Unpack(togo).NormalizeOrZero();
            double theta = MathEx.SafeAcos(double3.Dot(axisCci, uCci));
            FlightComputer fc = vehicle.FlightComputer;
            double alpha = Math.Min(fc.RcsTorqueAuthority.Y, fc.RcsTorqueAuthority.Z);
            if (theta > AlignMinThetaRad)
            {
                if (alpha <= MinSlewAlphaRadS2)
                {
                    est.AlignFeasible = false;
                }
                else
                {
                    double omega = Math.Min(fc.RateLimit, Math.Sqrt(theta * alpha));
                    double thrustOn = 2.0 * omega / alpha;
                    double coast = Math.Max(0.0, theta - omega * omega / alpha) / Math.Max(omega, 1e-9);
                    est.AlignSlewDurationSec = thrustOn + coast;
                    double slewMassFlow = SlewMassFlowFactor
                        * (cap.RotationMassFlowKgS.Y + cap.RotationMassFlowKgS.Z);
                    est.AlignSlewPropellantKg = thrustOn * slewMassFlow;
                }
            }
        }
        return est;
    }

    internal static bool TryHoldPerformance(
        in RcsCapabilitySnapshot cap, double3 uCtrl, out double netForce, out double massFlow)
    {
        netForce = 0.0;
        massFlow = 0.0;
        if (uCtrl.IsNearlyZero())
            return false;

        Span<double> weight = stackalloc double[6];
        weight[0] = Math.Max(uCtrl.X, 0.0);
        weight[1] = Math.Max(-uCtrl.X, 0.0);
        weight[2] = Math.Max(uCtrl.Y, 0.0);
        weight[3] = Math.Max(-uCtrl.Y, 0.0);
        weight[4] = Math.Max(uCtrl.Z, 0.0);
        weight[5] = Math.Max(-uCtrl.Z, 0.0);

        double maxNet = double.PositiveInfinity;
        for (int i = 0; i < 6; i++)
        {
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            if (!g.IsUsable)
                return false;
            maxNet = Math.Min(maxNet, g.ForceN / weight[i]);
        }
        if (!double.IsFinite(maxNet) || maxNet <= 0.0)
            return false;

        netForce = maxNet;
        for (int i = 0; i < 6; i++)
        {
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            massFlow += g.MassFlowKgS * (weight[i] * maxNet / g.ForceN);
        }
        return true;
    }

    /// <summary>Each demanded group contributes residual torque in proportion to its direction weight and force. The rotation groups supply the propellant cost per unit torque. The result is measured in kg per N s of net impulse. Axes without usable rotation authority currently contribute no estimated cost.</summary>
    internal static double GroupAttitudeFightPerImpulse(in RcsCapabilitySnapshot cap, double3 uCtrl)
    {
        Span<double> weight = stackalloc double[6]
        {
            Math.Max(uCtrl.X, 0.0), Math.Max(-uCtrl.X, 0.0),
            Math.Max(uCtrl.Y, 0.0), Math.Max(-uCtrl.Y, 0.0),
            Math.Max(uCtrl.Z, 0.0), Math.Max(-uCtrl.Z, 0.0),
        };
        double3 torquePerForce = default;
        for (int i = 0; i < 6; i++)
        {
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            if (!g.IsUsable)
                continue;
            double s = weight[i] / g.ForceN;
            torquePerForce.X += g.TorqueNm.X * s;
            torquePerForce.Y += g.TorqueNm.Y * s;
            torquePerForce.Z += g.TorqueNm.Z * s;
        }
        double cost =
            AxisAttitudeCost(Math.Abs(torquePerForce.X), cap.RotationMassFlowKgS.X, cap.RotationTorqueNm.X)
            + AxisAttitudeCost(Math.Abs(torquePerForce.Y), cap.RotationMassFlowKgS.Y, cap.RotationTorqueNm.Y)
            + AxisAttitudeCost(Math.Abs(torquePerForce.Z), cap.RotationMassFlowKgS.Z, cap.RotationTorqueNm.Z);
        return AttitudeFightFactor * cost;
    }

    private static double AxisAttitudeCost(double torquePerForce, float rotFlowKgS, float rotTorqueNm)
        => torquePerForce * RotationFlowPerTorque(rotFlowKgS, rotTorqueNm);

    // Measure cost in kg per N m s. The current model assigns zero cost when rotation authority is unavailable.
    internal static double RotationFlowPerTorque(float rotFlowKgS, float rotTorqueNm)
        => rotTorqueNm > MinSlackTorqueNm && rotFlowKgS > 0f
            ? (double)rotFlowKgS / rotTorqueNm
            : 0.0;

    public static string AxisName(int idx) => idx switch
    {
        0 => "+X", 1 => "-X", 2 => "+Y", 3 => "-Y", 4 => "+Z", _ => "-Z",
    };

    #endregion
}
