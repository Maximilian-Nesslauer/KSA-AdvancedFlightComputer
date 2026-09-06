using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static partial class RcsExecutor
{
    #region LP allocation

    private const double TorqueSlackPriceFactor = 1.0;

    // The attitude hold cannot absorb torque below this authority, measured in N m, so the LP allows no slack there.
    private const float MinSlackTorqueNm = 1f;

    private static double SlackPrice(float rotFlowKgS, float rotTorqueNm)
        => TorqueSlackPriceFactor * RotationFlowPerTorque(rotFlowKgS, rotTorqueNm);

    // Drop near zero LP members that would produce minimum pulse puffs. Their residual torque remains in the reported pattern torque.
    private const float LpDutyFloor = 0.01f;

    // Refresh on configuration changes, elapsed cadence, or direction drift. Failed attempts also advance the cadence.
    private static void EnsureLpSolution(
        Vehicle vehicle, FlightComputer fc, RcsExecution exec, float3 impulseCtrl, double nowSec)
    {
        RcsWrenchTable w = RefreshWrench(vehicle, fc, exec, nowSec);

        float3 dir = impulseCtrl.NormalizeOrZero();
        if (dir.IsExactlyZero())
            return;
        // Record failed attempts too so an infeasible solve respects the refresh interval.
        bool drifted = float3.Dot(dir, exec.LpDirCtrl) < 0.999f;
        if (!drifted && nowSec - exec.LpSolvedAtSec <= EstimateRefreshSec)
            return;
        exec.LpSolvedAtSec = nowSec;
        exec.LpDirCtrl = dir;

        if (w.UsableCount == 0)
        {
            DropLpSolution(exec, "no usable thrusters");
            return;
        }

        // Allow priced torque slack only on axes with rotation authority. Other axes retain zero torque constraints.
        ref readonly RcsCapabilitySnapshot cap = ref exec.Capability;
        Span<double> rotPrice = stackalloc double[3]
        {
            SlackPrice(cap.RotationMassFlowKgS.X, cap.RotationTorqueNm.X),
            SlackPrice(cap.RotationMassFlowKgS.Y, cap.RotationTorqueNm.Y),
            SlackPrice(cap.RotationMassFlowKgS.Z, cap.RotationTorqueNm.Z),
        };
        BuildLpProblem(w, dir, rotPrice, out double[] columns, out double[] cost, out int[] map, out double[] rhs);
        int n = cost.Length;

        double[]? x = RcsLpSolver.Solve(6, n, columns, cost, rhs);
        if (x == null)
        {
            DropLpSolution(exec,
                "the force constraint is infeasible for this layout/direction");
            return;
        }

        StoreLpSolution(vehicle, fc, exec, w, x, cost, map);
    }

    private static void DropLpSolution(RcsExecution exec, string reason)
    {
        exec.LpSecondsPerImpulse = null;
        exec.LpCostPerImpulse = 0.0;
        exec.LpSlackCostPerImpulse = 0.0;
        exec.LpResidualTorquePerNs = default;
        if (!exec.LpFallbackLogged)
        {
            exec.LpFallbackLogged = true;
            DefaultCategory.Log.Warning(
                $"[AFC] RCS LP allocator falling back to axis groups: {reason}.");
        }
    }

    // Completion requires every participating pulse to fall below its own minimum pulse floor.
    private static bool IsBelowLpFloor(float3 impulseCtrl, FlightComputer fc, RcsExecution exec)
    {
        float[] x = exec.LpSecondsPerImpulse!;
        List<ThrusterController> thrusters = fc.VehicleConfig.Thrusters;
        if (x.Length != thrusters.Count)
            return false;
        float j = float3.Dot(impulseCtrl, exec.LpDirCtrl);
        if (j <= 0f)
            return false;
        for (int i = 0; i < x.Length; i++)
        {
            if (x[i] <= 0f)
                continue;
            if (x[i] * j >= MinImpulseSuppressionFactor * thrusters[i].MinimumPulseTime)
                return false;
        }
        return true;
    }

    #endregion

    private static RcsWrenchTable RefreshWrench(Vehicle vehicle, FlightComputer fc, RcsExecution exec, double nowSec)
    {
        List<ThrusterController> thrusters = fc.VehicleConfig.Thrusters;
        RcsCtrlFrame ctrl = RcsCtrlFrame.For(vehicle);
        exec.Wrench ??= new RcsWrenchTable();
        if (nowSec - exec.WrenchBuiltAtSec > CapabilityRefreshSec
            || !exec.Wrench.Matches(thrusters, in ctrl))
        {
            exec.Wrench.Build(vehicle, fc, in ctrl);
            exec.WrenchBuiltAtSec = nowSec;
            exec.LpSolvedAtSec = double.NegativeInfinity;
            // Refresh capability with the wrench table across staging. A lagging control frame affects the price only.
            exec.Capability = RcsCapability.Probe(vehicle);
            exec.CapabilityProbedAtSec = nowSec;
        }

        return exec.Wrench;
    }

    private static void BuildLpProblem(
        RcsWrenchTable w, float3 dir, ReadOnlySpan<double> rotPrice,
        out double[] columns, out double[] cost, out int[] map, out double[] rhs)
    {
        int slackCount = 0;
        for (int a = 0; a < 3; a++)
        {
            if (rotPrice[a] > 0.0)
                slackCount += 2;
        }

        int n = w.UsableCount + slackCount;
        columns = new double[n * 6];
        cost = new double[n];
        map = new int[w.UsableCount];
        int k = 0;
        for (int i = 0; i < w.Count; i++)
        {
            if (!w.Usable[i])
                continue;
            columns[k * 6 + 0] = w.ForceCtrl[i].X;
            columns[k * 6 + 1] = w.ForceCtrl[i].Y;
            columns[k * 6 + 2] = w.ForceCtrl[i].Z;
            columns[k * 6 + 3] = w.TorqueCtrl[i].X;
            columns[k * 6 + 4] = w.TorqueCtrl[i].Y;
            columns[k * 6 + 5] = w.TorqueCtrl[i].Z;
            cost[k] = w.MassFlow[i];
            map[k] = i;
            k++;
        }
        int s = 0;
        for (int a = 0; a < 3; a++)
        {
            if (rotPrice[a] <= 0.0)
                continue;
            columns[(k + s) * 6 + 3 + a] = 1.0;
            cost[k + s] = rotPrice[a];
            s++;
            columns[(k + s) * 6 + 3 + a] = -1.0;
            cost[k + s] = rotPrice[a];
            s++;
        }
        rhs = new double[] { dir.X, dir.Y, dir.Z, 0.0, 0.0, 0.0 };
    }

    private static void StoreLpSolution(
        Vehicle vehicle, FlightComputer fc, RcsExecution exec, RcsWrenchTable w,
        double[] x, double[] cost, int[] map)
    {
        float maxX = 0f;
        for (int j = 0; j < w.UsableCount; j++)
            maxX = Math.Max(maxX, (float)x[j]);
        if (maxX <= 0f)
        {
            DropLpSolution(exec, "the solver returned an empty firing pattern");
            return;
        }

        // Sum torque after the duty floor. Removed members no longer cancel their share.
        float[] secondsPerImpulse = new float[fc.VehicleConfig.Thrusters.Count];
        double costPerImpulse = 0.0;
        Span<double> resTau = stackalloc double[3];
        int support = 0;
        for (int j = 0; j < w.UsableCount; j++)
        {
            float xi = (float)x[j];
            if (xi < LpDutyFloor * maxX)
                continue;
            secondsPerImpulse[map[j]] = xi;
            costPerImpulse += cost[j] * x[j];
            resTau[0] += w.TorqueCtrl[map[j]].X * x[j];
            resTau[1] += w.TorqueCtrl[map[j]].Y * x[j];
            resTau[2] += w.TorqueCtrl[map[j]].Z * x[j];
            support++;
        }

        double slackCost = 0.0;
        for (int j = w.UsableCount; j < cost.Length; j++)
            slackCost += cost[j] * x[j];

        exec.LpSecondsPerImpulse = secondsPerImpulse;
        exec.LpImpulseCapNs = MaxPulseSec / maxX;
        exec.LpCostPerImpulse = costPerImpulse;
        exec.LpSlackCostPerImpulse = slackCost;
        exec.LpResidualTorquePerNs = new float3(
            (float)resTau[0], (float)resTau[1], (float)resTau[2]);

        LogLpPattern(vehicle, exec, w, secondsPerImpulse, support, maxX, costPerImpulse, slackCost);
    }

    private static void LogLpPattern(
        Vehicle vehicle, RcsExecution exec, RcsWrenchTable w, float[] secondsPerImpulse,
        int support, float maxX, double costPerImpulse, double slackCost)
    {
        // Log only a changed support set to avoid repeated per tick patterns.
        if (DebugConfig.RcsTranslation)
        {
            int signature = support;
            for (int i = 0; i < secondsPerImpulse.Length; i++)
            {
                if (secondsPerImpulse[i] > 0f)
                    signature = signature * 31 + i;
            }
            if (signature != exec.LpLoggedSupportSignature)
            {
                exec.LpLoggedSupportSignature = signature;
                float3 res = exec.LpResidualTorquePerNs;
                DefaultCategory.Log.Debug(
                    $"[AFC] RCS LP solved: vehicle='{vehicle.Id}' {support}/{w.UsableCount} thrusters, " +
                    $"{costPerImpulse * 1e6:F2}mg per Ns, cap {exec.LpImpulseCapNs:F0}Ns/tick" +
                    (slackCost > 0.0
                        ? $", slack tau=({res.X:F2},{res.Y:F2},{res.Z:F2})Nms/Ns " +
                          $"({slackCost * 1e6:F2}mg per Ns)"
                        : string.Empty));
                for (int i = 0; i < secondsPerImpulse.Length; i++)
                {
                    if (secondsPerImpulse[i] <= 0f)
                        continue;
                    float3 f = w.ForceCtrl[i];
                    DefaultCategory.Log.Debug(
                        $"[AFC]   LP thruster {i}: duty={secondsPerImpulse[i] / maxX * 100f:F0}% " +
                        $"F=({f.X / 1000f:F1},{f.Y / 1000f:F1},{f.Z / 1000f:F1})kN " +
                        $"|tau|={w.TorqueCtrl[i].Length() / 1000f:F1}kNm");
                }
            }
        }
    }
}
