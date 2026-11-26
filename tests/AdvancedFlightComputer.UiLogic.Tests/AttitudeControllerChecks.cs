using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTorque;

internal static partial class Program
{
    private static void AttitudeControllerRules()
    {
        ZeroLagRateFilterChecks();
        ConditionalAntiWindupChecks();
        GimbalSlewResidualChecks();
        JointFinRcsAllocationChecks();
    }

    private static void ZeroLagRateFilterChecks()
    {
        // With RateFilterS = 0, a step in rate error passes through with zero phase lag on the very first step.
        AttitudeControlMath.FilterRateError(0.4, -0.2, 0.1, 0.0, 0.0, 0.0,
            rateKnown: true, rateFilterS: 0.0, dt: 0.05,
            out double rx, out double ry, out double rz);
        Check("zero-lag rate filter passes raw step immediately",
            Math.Abs(rx - 0.4) < 1e-12 && Math.Abs(ry + 0.2) < 1e-12 && Math.Abs(rz - 0.1) < 1e-12,
            $"got ({rx:G6}, {ry:G6}, {rz:G6})");

        // Positive RateFilterS still smooths via backward Euler when explicitly configured.
        AttitudeControlMath.FilterRateError(1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            rateKnown: true, rateFilterS: 0.05, dt: 0.05,
            out rx, out ry, out rz);
        Check("explicit RateFilterS = 0.05 smooths by dt/(tau+dt) = 0.5",
            Math.Abs(rx - 0.5) < 1e-12, $"got {rx:G6}");
    }

    private static void ConditionalAntiWindupChecks()
    {
        const double within = 3.0 * Math.PI / 180.0;
        const double cap = 8.0 * Math.PI / 180.0;
        const double err = 0.02; // 1.15 deg, inside window

        // Unsaturated: integrates normally on all three axes.
        AttitudeControlMath.UpdateIntegral(
            0.0, 0.0, 0.0,
            err, -err, 0.01,
            rollFree: false, withinRad: within, capRadS: cap, dt: 0.1,
            lastDemandX: 1000.0, lastDemandY: -1000.0, lastDemandZ: 200.0,
            lastShortfallX: 0.0, lastShortfallY: 0.0, lastShortfallZ: 0.0,
            saturatedShare: 0.15,
            out double ix, out double iy, out double iz);
        Check("unsaturated integral accumulates on all axes",
            Math.Abs(ix - 0.002) < 1e-12 && Math.Abs(iy + 0.002) < 1e-12 && Math.Abs(iz - 0.001) < 1e-12);

        // Saturated on X only (positive shortfall + positive error): X freezes, Y and Z still integrate.
        AttitudeControlMath.UpdateIntegral(
            ix, iy, iz,
            err, -err, 0.01,
            rollFree: false, withinRad: within, capRadS: cap, dt: 0.1,
            lastDemandX: 1000.0, lastDemandY: -1000.0, lastDemandZ: 200.0,
            lastShortfallX: 400.0, lastShortfallY: 0.0, lastShortfallZ: 0.0,
            saturatedShare: 0.15,
            out double ix2, out double iy2, out double iz2);
        Check("saturated X axis freezes while unsaturated Y and Z keep integrating",
            Math.Abs(ix2 - ix) < 1e-12 && Math.Abs(iy2 - (iy - 0.002)) < 1e-12 && Math.Abs(iz2 - (iz + 0.001)) < 1e-12,
            $"ix {ix2:G6} vs {ix:G6}");

        // Unwinding: X still has positive shortfall, but error reverses to negative -> X unwinds immediately.
        AttitudeControlMath.UpdateIntegral(
            ix2, iy2, iz2,
            -err, 0.0, 0.0,
            rollFree: false, withinRad: within, capRadS: cap, dt: 0.1,
            lastDemandX: 1000.0, lastDemandY: 0.0, lastDemandZ: 0.0,
            lastShortfallX: 400.0, lastShortfallY: 0.0, lastShortfallZ: 0.0,
            saturatedShare: 0.15,
            out double ix3, out _, out _);
        Check("reversed error unwinds saturated axis immediately",
            Math.Abs(ix3 - (ix2 - 0.002)) < 1e-12, $"got {ix3:G6}");
    }

    private static void GimbalSlewResidualChecks()
    {
        const float maxAngle = 0.105f;
        const float wn = 28.5f;
        const float dt = 0.05f;

        // From rest (angle = 0, rate = 0) commanding full deflection (1.0): 1 step only moves a small fraction,
        // leaving a large transient slew residual for fins/RCS.
        AttitudeControlMath.StepGimbalNormalized(
            0f, 0f, 0f, 0f,
            1.0, 0.0, maxAngle, maxAngle, wn, dt, constrainToCircle: true,
            out double u1Y, out double u1Z);
        Check("1-step gimbal slew from rest covers only a fraction of full command",
            u1Y > 0.05 && u1Y < 0.25 && Math.Abs(u1Z) < 1e-9,
            $"u1 = ({u1Y:F4}, {u1Z:F4})");

        // Stepping repeatedly reaches full command in ~0.48 s without overshooting 1.0.
        float ay = 0f, ry = 0f;
        for (int k = 0; k < 20; k++)
            AttitudeControlMath.StepGimbalAxis(ref ay, ref ry, maxAngle, maxAngle, wn, dt);
        Check("rate-capped gimbal reaches steady-state command within 1 s without overshoot",
            Math.Abs(ay - maxAngle) < 1e-4f && ay <= maxAngle + 1e-6f,
            $"ay = {ay:F6} vs {maxAngle:F6}");

        // Once already at command, 1-step reachable equals steady-state command (zero residual).
        AttitudeControlMath.StepGimbalNormalized(
            0.5f * maxAngle, 0f, -0.3f * maxAngle, 0f,
            0.5, -0.3, maxAngle, maxAngle, wn, dt, constrainToCircle: true,
            out double eqY, out double eqZ);
        Check("settled gimbal has zero 1-step residual",
            Math.Abs(eqY - 0.5) < 1e-6 && Math.Abs(eqZ + 0.3) < 1e-6,
            $"got ({eqY:F6}, {eqZ:F6})");

        // Diagonal overshoot is projected back onto the circular cone.
        AttitudeControlMath.StepGimbalNormalized(
            0.8f * maxAngle, 0f, 0.8f * maxAngle, 0f,
            0.8, 0.8, maxAngle, maxAngle, wn, dt, constrainToCircle: true,
            out double cy, out double cz);
        double norm = Math.Sqrt(cy * cy + cz * cz);
        Check("circular cone constraint clamps diagonal gimbal state to unit circle",
            norm <= 1.0 + 1e-5, $"norm = {norm:F6}");
    }

    private static void JointFinRcsAllocationChecks()
    {
        // Two fins: fin 0 gives pitch (axis 0), fin 1 gives yaw (axis 1), neither gives roll (axis 2).
        // Fin range [-30, 30] deg, 1000 N m/deg -> max 30 kN m each; 1-step reach 1.5 deg (30 deg/s * 0.05 s) = 1.5 kN m.
        var fins = new AeroTorqueAllocator(2);
        double[] defl = [-30.0, 0.0, 30.0];
        double[] tFin0 = [-30_000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30_000.0, 0.0, 0.0];
        double[] tFin1 = [0.0, -30_000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30_000.0, 0.0];
        double[] zeroForce = new double[9];
        fins.SetSurface(0, defl, tFin0, zeroForce);
        fins.SetSurface(1, defl, tFin1, zeroForce);

        // Slender-booster RCS: pitch/yaw thrusters have 15x longer lever arm than roll thrusters.
        var rcs = new RcsTorqueAllocator(6);
        double[] rcsTorques =
        [
             15_000.0, 0.0, 0.0,   -15_000.0, 0.0, 0.0,
             0.0, 15_000.0, 0.0,    0.0, -15_000.0, 0.0,
             0.0, 0.0, 1_000.0,     0.0, 0.0, -1_000.0,
        ];
        double[] rcsFlows = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        Array.Copy(rcsTorques, rcs.Torque, rcsTorques.Length);
        Array.Copy(rcsFlows, rcs.Flow, rcsFlows.Length);

        double[] priority = [1.0, 1.0, 0.3];
        double perKgS = AttitudeControlMath.BackupPricePerKgS(rcs.Torque, rcs.Flow, priority, 0.02, 0.004, 0.25);
        double bestCostPerNm = (1.0 / 15_000.0) * perKgS;
        double rollCostPerNm = (1.0 / 1_000.0) * perKgS;
        Check("joint backup pricing keeps fins cheaper than best RCS and roll RCS cheaper than roll shortfall",
            bestCostPerNm > 0.002 && rollCostPerNm < priority[2] * 0.5,
            $"best={bestCostPerNm:F4}, roll={rollCostPerNm:F4}, rollPrio={priority[2]:F2}");

        var backupCost = new double[6];
        for (int i = 0; i < 6; i++)
            backupCost[i] = rcsFlows[i] * perKgS;

        // Demand: 5 kN m pitch (exceeds 1.5 kN m fin 1-step reach), 1 kN m yaw (within fin reach), 400 N m roll (RCS only).
        var finCmd = new double[2];
        var curDeg = new double[2];
        var maxStep = new double[] { 1.5, 1.5 };
        var backupDuty = new double[6];
        AeroTorqueAllocator.Result r = fins.Allocate(5_000.0, 1_000.0, 400.0, priority, finCmd, curDeg, maxStep, rcsTorques, backupCost, backupDuty);

        double rcsTx = 0.0, rcsTy = 0.0, rcsTz = 0.0;
        for (int i = 0; i < 6; i++)
        {
            rcsTx += rcsTorques[3 * i] * backupDuty[i];
            rcsTy += rcsTorques[3 * i + 1] * backupDuty[i];
            rcsTz += rcsTorques[3 * i + 2] * backupDuty[i];
        }
        Check("joint allocation uses full fin reach on pitch, fins alone on yaw, and RCS on pitch residual + roll",
            r.Solved
            && Math.Abs(r.TorqueX - 1_500.0) < 10.0
            && Math.Abs(r.TorqueY - 1_000.0) < 10.0
            && Math.Abs(rcsTx - 3_500.0) < 10.0
            && Math.Abs(rcsTy) < 1.0
            && Math.Abs(rcsTz - 400.0) < 10.0,
            $"fin=({r.TorqueX:F1},{r.TorqueY:F1},{r.TorqueZ:F1}), rcs=({rcsTx:F1},{rcsTy:F1},{rcsTz:F1})");
    }
}
