using System;
using Brutal.Numerics;
using KSA;

/// <summary>
/// Physical thrust-vector control allocation: given a desired body torque in N·m,
/// solve for the per-gimbal deflections that produce it.
///
/// WHY NOT USE KSA'S OWN ALLOCATION. The game's ComputeTvcControl offers the SAME
/// normalized demand vector to every gimbal and lets each independently pick a
/// direction from its own moment arm. That is a heuristic, not an inverse:
///   - the demand is DIMENSIONLESS. Its magnitude scales deflection directly and
///     is then clamped to [-1,1], so there is nowhere to put a torque in N·m.
///   - nothing solves "produce exactly tau". The realized torque is the sum over
///     gimbals and is generally NOT parallel to the demand, so commanding pure
///     roll leaks pitch and yaw.
///   - every gimbal gets the same deflection magnitude regardless of its thrust,
///     so a 6 MN main and a small vernier contribute wildly unequally.
/// For SCvx that matters: the model's dynamics are J*wdot = tau with tau a control
/// we choose and trust. Commanding through a heuristic makes the realized torque a
/// nonlinear function of the demand, and the planned attitude stops matching the
/// flown one — which is the main thing 6-DOF buys over the 3-DOF plan.
///
/// WHAT THIS DOES INSTEAD. Build the true allocation matrix B (3 x 2N) mapping
/// per-gimbal commands to body torque, then solve the regularized minimum-norm
/// problem
///     u = B^T (B B^T + lambda I)^-1 tau
/// B B^T is only 3x3, so this is a closed-form inverse, not an iterative solve —
/// cheap enough to run every flight-computer step. Minimum-norm is the right
/// objective here: it spreads effort across gimbals in proportion to their
/// effectiveness, which is what makes the verniers take roll and the main engine
/// take pitch/yaw with no per-vehicle configuration.
/// </summary>
public static class KsaTvcAllocator
{
    /// <summary>Central-difference step, in command units. Large enough to beat float
    /// noise, small enough that sin() is still linear: at a 10 deg gimbal this costs
    /// under 0.2% slope error.</summary>
    private const double DiffStep = 0.5;

    /// <summary>
    /// Tikhonov term, relative to trace(B B^T). Without it an axis with no authority
    /// at all (e.g. roll on a vehicle with no off-axis gimbals) makes B B^T singular
    /// and the solve explodes. With it, an unreachable torque simply produces a small
    /// command instead of a huge one.
    /// </summary>
    private const double RelativeRegularization = 1e-6;

    /// <summary>
    /// Thrust direction in the VEHICLE assembly frame for a given normalized command.
    ///
    /// Mirrors RocketNozzle.UpdateState exactly. The game rotates assembly-frame
    /// vectors by (Gimbal2Asmb * state * Gimbal2Asmb^-1), which is just "apply the
    /// state rotation in the gimbal's own frame" — so we take the rest direction into
    /// gimbal frame via Data.VehicleAsmb2Gimbal, rotate, and come back.
    /// </summary>
    public static double3 ThrustDirection(GimbalController gc, double commandY, double commandZ)
    {
        Gimbal g = gc.Gimbal;
        doubleQuat vehicleToGimbal = doubleQuat.Unpack(in gc.Data.VehicleAsmb2Gimbal);

        float3 restF = gc.Data.ThrustDirVehicleAsmb;
        double3 restInGimbal = new double3(restF.X, restF.Y, restF.Z).Transform(vehicleToGimbal);

        double angleY = Math.Clamp(commandY, -1.0, 1.0) * g.AxisY.MaxAngle;
        double angleZ = Math.Clamp(commandZ, -1.0, 1.0) * g.AxisZ.MaxAngle;
        doubleQuat deflect = doubleQuat.Concatenate(
            doubleQuat.CreateFromAxisAngle(double3.UnitY, angleY),
            doubleQuat.CreateFromAxisAngle(double3.UnitZ, angleZ));

        return restInGimbal.Transform(deflect).Transform(doubleQuat.Inverse(vehicleToGimbal));
    }

    /// <summary>
    /// Fill the torque and force Jacobians for one gimbal: columns 2i and 2i+1 of
    /// B (N·m per unit command) and Bf (N per unit command).
    ///
    /// The moment arm is taken at the REST thrust position, ignoring the small
    /// translation of the nozzle as it swings about its pivot. That is second order
    /// (offset x sin(10 deg)) and is exactly the approximation KSA's own
    /// ComputeTvcControl and UpdateTvcParams make, so we stay consistent with the
    /// plant rather than more accurate than it.
    /// </summary>
    private static void FillColumns(GimbalController gc, float3 comAsmb, double thrust,
                                    double[] bTorque, double[] bForce, int gimbalIndex)
    {
        float3 armF = gc.Data.ThrustPosVehicleAsmb - comAsmb;
        var arm = new double3(armF.X, armF.Y, armF.Z);

        for (int axis = 0; axis < 2; axis++)
        {
            double cy = axis == 0 ? DiffStep : 0.0;
            double cz = axis == 1 ? DiffStep : 0.0;

            double3 plus = ThrustDirection(gc, cy, cz);
            double3 minus = ThrustDirection(gc, -cy, -cz);

            double3 dForce = (plus - minus) * (thrust / (2.0 * DiffStep));
            double3 dTorque = double3.Cross(arm, dForce);

            int col = 2 * gimbalIndex + axis;
            bTorque[3 * col + 0] = dTorque.X;
            bTorque[3 * col + 1] = dTorque.Y;
            bTorque[3 * col + 2] = dTorque.Z;
            bForce[3 * col + 0] = dForce.X;
            bForce[3 * col + 1] = dForce.Y;
            bForce[3 * col + 2] = dForce.Z;
        }
    }

    /// <summary>
    /// Solve for the deflections producing <paramref name="desiredTorque"/> (N·m, body frame).
    ///
    /// <paramref name="thrusts"/> is per-gimbal thrust in N — pass the live thrust to
    /// command, or Data.MaximumThrust to preview what full thrust would allow.
    /// Commands come back in <paramref name="commands"/> as 2 per gimbal (Y then Z).
    /// </summary>
    public static TvcAllocationResult Solve(ReadOnlySpan<GimbalController> gimbals,
                                            ReadOnlySpan<double> thrusts,
                                            float3 comAsmb,
                                            double3 desiredTorque,
                                            double[] commands)
    {
        int n = gimbals.Length;
        var result = new TvcAllocationResult { GimbalCount = n, SaturationScale = 1.0 };
        if (n == 0)
            return result;

        int cols = 2 * n;
        var bTorque = new double[3 * cols];
        var bForce = new double[3 * cols];
        for (int i = 0; i < n; i++)
            FillColumns(gimbals[i], comAsmb, thrusts[i], bTorque, bForce, i);

        // G = B B^T, 3x3 symmetric.
        Span<double> g = stackalloc double[9];
        for (int c = 0; c < cols; c++)
        {
            double x = bTorque[3 * c], y = bTorque[3 * c + 1], z = bTorque[3 * c + 2];
            g[0] += x * x; g[1] += x * y; g[2] += x * z;
            g[4] += y * y; g[5] += y * z;
            g[8] += z * z;
        }
        g[3] = g[1]; g[6] = g[2]; g[7] = g[5];

        double trace = g[0] + g[4] + g[8];
        if (trace <= 0.0)
            return result;                        // no authority at all (no thrust)

        double lambda = RelativeRegularization * trace;
        g[0] += lambda; g[4] += lambda; g[8] += lambda;

        if (!Invert3x3(g, out Span<double> inv))
            return result;

        // w = G^-1 tau, then u = B^T w.
        double wx = inv[0] * desiredTorque.X + inv[1] * desiredTorque.Y + inv[2] * desiredTorque.Z;
        double wy = inv[3] * desiredTorque.X + inv[4] * desiredTorque.Y + inv[5] * desiredTorque.Z;
        double wz = inv[6] * desiredTorque.X + inv[7] * desiredTorque.Y + inv[8] * desiredTorque.Z;

        double peak = 0.0;
        for (int c = 0; c < cols; c++)
        {
            double u = bTorque[3 * c] * wx + bTorque[3 * c + 1] * wy + bTorque[3 * c + 2] * wz;
            commands[c] = u;
            peak = Math.Max(peak, Math.Abs(u));
        }

        // Saturation: scale the WHOLE solution rather than clipping per component.
        // Clipping would change the torque DIRECTION, which for attitude control is
        // worse than delivering less of the right torque.
        if (peak > 1.0)
        {
            double scale = 1.0 / peak;
            for (int c = 0; c < cols; c++)
                commands[c] *= scale;
            result.SaturationScale = scale;
        }

        // Report what the linear model says we actually get, including the lateral
        // force that comes along for free — gimballing for torque always tilts the
        // thrust vector, and that coupling is real.
        double3 torque = default, force = default;
        for (int c = 0; c < cols; c++)
        {
            double u = commands[c];
            torque.X += bTorque[3 * c] * u;
            torque.Y += bTorque[3 * c + 1] * u;
            torque.Z += bTorque[3 * c + 2] * u;
            force.X += bForce[3 * c] * u;
            force.Y += bForce[3 * c + 1] * u;
            force.Z += bForce[3 * c + 2] * u;
        }
        result.AchievedTorque = torque;
        result.AchievedForce = force;

        // Per-axis capability, for scaling a UI slider or clamping a guidance demand:
        // the torque produced by driving the min-norm solution for that axis to
        // saturation. Unlike KSA's TvcTorqueAuthority (a sum of absolute values, so
        // an optimistic bound that assumes every gimbal serves that axis maximally),
        // this is achievable by construction.
        result.MaxTorque = new double3(
            AxisCapability(bTorque, cols, inv, 0),
            AxisCapability(bTorque, cols, inv, 1),
            AxisCapability(bTorque, cols, inv, 2));
        return result;
    }

    // Largest torque about one axis alone that the min-norm solution can deliver
    // before some gimbal saturates.
    private static double AxisCapability(double[] bTorque, int cols, ReadOnlySpan<double> inv, int axis)
    {
        double wx = inv[axis], wy = inv[3 + axis], wz = inv[6 + axis];
        double peak = 0.0, torque = 0.0;
        for (int c = 0; c < cols; c++)
        {
            double u = bTorque[3 * c] * wx + bTorque[3 * c + 1] * wy + bTorque[3 * c + 2] * wz;
            peak = Math.Max(peak, Math.Abs(u));
            torque += bTorque[3 * c + axis] * u;
        }
        return peak > 0.0 ? torque / peak : 0.0;
    }

    private static bool Invert3x3(ReadOnlySpan<double> m, out Span<double> inv)
    {
        double c0 = m[4] * m[8] - m[5] * m[7];
        double c1 = m[5] * m[6] - m[3] * m[8];
        double c2 = m[3] * m[7] - m[4] * m[6];
        double det = m[0] * c0 + m[1] * c1 + m[2] * c2;

        inv = new double[9];
        if (Math.Abs(det) < 1e-30)
            return false;

        double d = 1.0 / det;
        inv[0] = c0 * d;
        inv[1] = (m[2] * m[7] - m[1] * m[8]) * d;
        inv[2] = (m[1] * m[5] - m[2] * m[4]) * d;
        inv[3] = c1 * d;
        inv[4] = (m[0] * m[8] - m[2] * m[6]) * d;
        inv[5] = (m[2] * m[3] - m[0] * m[5]) * d;
        inv[6] = c2 * d;
        inv[7] = (m[1] * m[6] - m[0] * m[7]) * d;
        inv[8] = (m[0] * m[4] - m[1] * m[3]) * d;
        return true;
    }
}

public struct TvcAllocationResult
{
    public int GimbalCount;

    /// <summary>Torque the linear model says the returned commands deliver, N·m.</summary>
    public double3 AchievedTorque;

    /// <summary>Lateral force that comes with it, N. Gimballing for torque always tilts the thrust vector.</summary>
    public double3 AchievedForce;

    /// <summary>Per-axis achievable torque at saturation, N·m. Honest, unlike KSA's TvcTorqueAuthority.</summary>
    public double3 MaxTorque;

    /// <summary>1 when unsaturated; below 1 the demand was scaled down to fit, preserving direction.</summary>
    public double SaturationScale;
}
