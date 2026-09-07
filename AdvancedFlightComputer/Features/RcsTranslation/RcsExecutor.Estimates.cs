using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static partial class RcsExecutor
{
    #region Estimates

    // Price the worker's continuous pulse demand in kg per N s of requested translation impulse.
    internal static double GroupCostPerNs(in RcsCapabilitySnapshot cap, double3 uCtrl)
    {
        GroupDemand(in cap, uCtrl, out double cost, out _);
        return cost;
    }

    private static void GroupDemand(in RcsCapabilitySnapshot cap, double3 uCtrl,
        out double costPerNs, out double3 torquePerNs)
    {
        Span<double> weight = stackalloc double[6]
        {
            Math.Max(uCtrl.X, 0.0), Math.Max(-uCtrl.X, 0.0),
            Math.Max(uCtrl.Y, 0.0), Math.Max(-uCtrl.Y, 0.0),
            Math.Max(uCtrl.Z, 0.0), Math.Max(-uCtrl.Z, 0.0),
        };
        Span<double> seconds = stackalloc double[6];
        seconds.Clear();
        costPerNs = 0.0;
        torquePerNs = default;
        for (int i = 0; i < 6; i++)
        {
            RcsAxisGroup group = cap.Get(i);
            if (weight[i] < 1e-4 || !group.IsUsable)
                continue;
            seconds[i] = weight[i] / group.ForceN;
            costPerNs += group.MassFlowKgS * seconds[i];
            torquePerNs += double3.Unpack(group.TorqueNm) * seconds[i];
        }
        // Remove repeated flow and torque because a shared thruster fires at the maximum requested axis pulse.
        for (int membership = 1; membership < 27; membership++)
        {
            ref readonly RcsSharedContribution shared = ref cap.SharedContributions[membership];
            if (shared.MassFlowKgS <= 0f)
                continue;
            int digits = membership;
            double sum = 0.0;
            double maximum = 0.0;
            for (int axis = 0; axis < 3; axis++, digits /= 3)
            {
                int sign = digits % 3;
                if (sign == 0)
                    continue;
                double pulse = seconds[2 * axis + sign - 1];
                sum += pulse;
                maximum = Math.Max(maximum, pulse);
            }
            double duplicate = sum - maximum;
            costPerNs -= shared.MassFlowKgS * duplicate;
            torquePerNs -= double3.Unpack(shared.TorqueNm) * duplicate;
        }
        costPerNs = Math.Max(0.0, costPerNs);
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
            if (theta > AlignMinThetaRad)
                EstimateSlew(vehicle.FlightComputer, in cap, theta, ref est);
        }
        return est;
    }

    /// <summary>Estimate an acceleration and braking turn with the flight computer rate limit. Low angular acceleration makes Align infeasible.</summary>
    private static void EstimateSlew(
        FlightComputer fc, in RcsCapabilitySnapshot cap, double theta, ref RcsEstimates est)
    {
        double alpha = Math.Min(fc.RcsTorqueAuthority.Y, fc.RcsTorqueAuthority.Z);
        if (alpha <= MinSlewAlphaRadS2)
        {
            est.AlignFeasible = false;
            return;
        }
        double omega = Math.Min(fc.RateLimit, Math.Sqrt(theta * alpha));
        double thrustOn = 2.0 * omega / alpha;
        double coast = Math.Max(0.0, theta - omega * omega / alpha) / Math.Max(omega, 1e-9);
        est.AlignSlewDurationSec = thrustOn + coast;
        double slewMassFlow = SlewMassFlowFactor
            * (cap.RotationMassFlowKgS.Y + cap.RotationMassFlowKgS.Z);
        est.AlignSlewPropellantKg = thrustOn * slewMassFlow;
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
        massFlow = maxNet * GroupCostPerNs(in cap, uCtrl);
        return true;
    }

    /// <summary>Price residual angular impulse in kg per N s of requested translation impulse. Axes without usable rotation authority currently contribute no estimated cost.</summary>
    internal static double GroupAttitudeFightPerImpulse(in RcsCapabilitySnapshot cap, double3 uCtrl)
    {
        GroupDemand(in cap, uCtrl, out _, out double3 torquePerForce);
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
