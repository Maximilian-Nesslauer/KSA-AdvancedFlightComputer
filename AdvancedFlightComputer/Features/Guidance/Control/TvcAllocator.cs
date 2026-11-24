#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;

/// <summary>
/// Physical thrust-vector control allocation: given a desired body torque in N*m, solve for the per-gimbal deflections that produce it.
///  WHY A SEPARATE ALLOCATION. The game's ComputeTvcControl turns a NORMALIZED per-axis demand into a torque with the ranges from TvcAllocation.ComputeTorqueRange and solves for it with the iterative TvcAllocation.Solve. The 6-DOF plan commands a torque in N*m through the gimbal override, and its setup also needs the linear model behind the solve: the per-axis capability that bounds the planned torque, and the lateral force that comes with it.
/// For SCvx that matters: the model's dynamics are J*wdot = tau with tau a control we choose and trust. The allocation must deliver the planned torque and report what it actually delivers, or the planned attitude stops matching the flown one - which is the main thing 6-DOF buys over the 3-DOF plan.
///  WHAT THIS DOES. Build the linear allocation matrix B (3 x 2N) mapping per-gimbal commands to body torque, then solve the regularized minimum-norm problem u = B^T (B B^T + lambda I)^-1 tau B B^T is only 3x3, so this is a closed-form inverse, not an iterative solve - cheap enough to run every flight-computer step. Minimum-norm is the right objective here: it spreads effort across gimbals in proportion to their effectiveness, which is what makes the verniers take roll and the main engine take pitch/yaw with no per-vehicle configuration.
/// </summary>
public static class KsaTvcAllocator
{
    /// <summary>Central-difference step, in command units. Large enough to beat float
    /// noise, small enough that sin() is still linear: at a 10 deg gimbal this costs
    /// under 0.2% slope error.</summary>
    private const double DiffStep = 0.5;

    /// <summary>
    /// Tikhonov term, relative to trace(B B^T). Without it an axis with no authority at all (e.g. roll on a vehicle with no off-axis gimbals) makes B B^T singular and the solve explodes. With it, an unreachable torque simply produces a small command instead of a huge one.
    /// </summary>
    private const double RelativeRegularization = 1e-6;

    /// <summary>
    /// Thrust direction in the VEHICLE assembly frame for a given normalized command.
    ///  Mirrors TvcAllocation.ComputeThrustAsmb, in double. The rest direction is stored in the gimbal's own frame as Data.InitialDirGimbal, so we apply the deflection rotation Gimbal.ComputeGimbal2Asmb there and then rotate into the vehicle assembly frame with Data.Gimbal2VehicleAsmb.
    /// </summary>
    public static double3 ThrustDirection(GimbalController gc, double commandY, double commandZ)
    {
        Gimbal g = gc.Gimbal;
        doubleQuat gimbalToVehicle = doubleQuat.Unpack(in gc.Data.Gimbal2VehicleAsmb);

        float3 restF = gc.Data.InitialDirGimbal;
        var restInGimbal = new double3(restF.X, restF.Y, restF.Z);

        double angleY = Math.Clamp(commandY, -1.0, 1.0) * g.AxisY.MaxAngle;
        double angleZ = Math.Clamp(commandZ, -1.0, 1.0) * g.AxisZ.MaxAngle;
        doubleQuat deflect = Gimbal.ComputeGimbal2Asmb(angleY, angleZ);

        return restInGimbal.Transform(deflect).Transform(gimbalToVehicle);
    }

    /// <summary>
    /// Where the thrust acts, VEHICLE assembly frame, for a given normalized command: the gimbal's pivot plus its thrust offset, both swung by the deflection. Mirrors TvcAllocation.ComputeThrustAsmb, in double - Data.ThrustOffsetGimbal rotated by Gimbal.ComputeGimbal2Asmb in the gimbal's frame, then into the vehicle assembly frame with Data.Gimbal2VehicleAsmb.
    /// </summary>
    public static double3 ThrustLocation(GimbalController gc, double commandY, double commandZ)
    {
        Gimbal g = gc.Gimbal;
        doubleQuat gimbalToVehicle = doubleQuat.Unpack(in gc.Data.Gimbal2VehicleAsmb);
        float3 offF = gc.Data.ThrustOffsetGimbal;
        double angleY = Math.Clamp(commandY, -1.0, 1.0) * g.AxisY.MaxAngle;
        double angleZ = Math.Clamp(commandZ, -1.0, 1.0) * g.AxisZ.MaxAngle;
        doubleQuat deflect = Gimbal.ComputeGimbal2Asmb(angleY, angleZ);
        return GimbalPivot(gc) + new double3(offF.X, offF.Y, offF.Z).Transform(deflect).Transform(gimbalToVehicle);
    }

    /// <summary>The gimbal's pivot, VEHICLE assembly frame (Data.PivotPosVehicleAsmb).</summary>
    public static double3 GimbalPivot(GimbalController gc)
    {
        float3 pivot = gc.Data.PivotPosVehicleAsmb;
        return new double3(pivot.X, pivot.Y, pivot.Z);
    }

    /// <summary>
    /// Fill the torque and force Jacobians for one gimbal: columns 2i and 2i+1 of B (N*m per unit command) and Bf (N per unit command).
    ///  THE TORQUE IS DIFFERENTIATED WHOLE, the thrust's point swinging about the pivot with its direction, because that is what the plant does (RocketNozzle.UpdateState). The swing is not second order: the point moves by the nozzle's length times the deflection while the FULL thrust acts on it, which is the same order as the side force, and the two together act on the arm to the PIVOT, not to the thrust point - the nozzle's length cancels. Taken at the rest thrust point the arm was the nozzle's length too long, and on the 2026-10-08 booster every torque came out 1.37x what the engines delivered: measured from the flight, the booster turned as if its inertia were 1.46e6 kg m^2 against the 1.07e6 KSA integrates it with.
    /// </summary>
    internal static void FillColumns(GimbalController gc, float3 comAsmb, double thrust,
                                    double[] bTorque, double[] bForce, int gimbalIndex)
    {
        var com = new double3(comAsmb.X, comAsmb.Y, comAsmb.Z);

        for (int axis = 0; axis < 2; axis++)
        {
            double cy = axis == 0 ? DiffStep : 0.0;
            double cz = axis == 1 ? DiffStep : 0.0;

            double3 plus = ThrustDirection(gc, cy, cz);
            double3 minus = ThrustDirection(gc, -cy, -cz);
            double3 torquePlus = double3.Cross(ThrustLocation(gc, cy, cz) - com, plus * thrust);
            double3 torqueMinus = double3.Cross(ThrustLocation(gc, -cy, -cz) - com, minus * thrust);

            double3 dForce = (plus - minus) * (thrust / (2.0 * DiffStep));
            double3 dTorque = (torquePlus - torqueMinus) * (1.0 / (2.0 * DiffStep));

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
    /// Solve for the deflections producing <paramref name="desiredTorque"/> (N*m, body frame).
    ///
    /// <paramref name="thrusts"/> is per-gimbal thrust in N - pass the live thrust to
    /// command, or Data.MaximumThrust to preview what full thrust would allow.
    /// Commands come back in <paramref name="commands"/> as 2 per gimbal (Y then Z).
    /// </summary>
    /// <summary>An axis whose capability is under this share of the strongest axis's is the weak one, allocated last (see Solve).</summary>
    private const double WeakAxisShare = 0.25;

    /// <param name="redistribute">When a gimbal reaches its stop, hold it there and give the rest of the demand to the gimbals still free (Redistribute). False scales the whole solution to the first gimbal at its stop.</param>
    /// <param name="coupled">The part of <paramref name="desiredTorque"/> that comes with its pitch and yaw on its own: the roll an off-centre engine's deflection brings (Ksa6DofGuidance.CoupledRoll). Its weak-axis share is solved with the strong axes rather than taken out and put back.</param>
    public static TvcAllocationResult Solve(ReadOnlySpan<GimbalController> gimbals,
                                            ReadOnlySpan<double> thrusts,
                                            float3 comAsmb,
                                            double3 desiredTorque,
                                            double[] commands,
                                            bool redistribute = true,
                                            double3 coupled = default)
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

        // THE WEAK AXIS IS CLAMPED TO ITS CAPABILITY, AND ONLY IT. Without the clamp a weak axis (roll here: 700x weaker than pitch/yaw) sends the least-squares solution far outside the deflection box, and the uniform saturation scaling below then throttles the strong axes too.
        //  The strong axes were clamped per axis as well until 2026-11-21, and that bent their direction. Each vehicle axis's capability is that axis ALONE, with no torque about the other two, and for an engine off the stack's axis on a diagonal (one engine of a ring, a pair beside the center) pure pitch or pure yaw each drags roll in that only the verniers can take out: their caps came out small while the direction the engine's own moment needs, which brings no roll, had over twice that (one 10 deg engine of the triple plate: 1429 kN m clamped, 3332 kN m along the direction, afc-guidance-engine-moment-diag). A demand along that direction was clamped to the box corner of the two small caps and scaled down to fit, so the gimbals held under half the moment they could. The strong axes now keep their direction and scale together, the saturation policy below.
        result.DemandClamped = true;
        double3 cap = new double3(
            Math.Abs(AxisCapability(bTorque, cols, inv, 0)),
            Math.Abs(AxisCapability(bTorque, cols, inv, 1)),
            Math.Abs(AxisCapability(bTorque, cols, inv, 2)));

        // THE WEAK AXIS GIVES WAY. The gimbals' deflection is one budget: torque about the weak axis (roll, for engines near the thrust line) is made by deflecting them against each other, and pitch and yaw by deflecting them together. Each axis's demand is inside what that axis can do alone, but both at their limits ask every gimbal for twice its travel, and scaling the whole solution then HALVED the pitch and yaw torque for the sake of a roll torque a hundredth of its size. On 2026-11-18 a booster that the air's roll moment had turning 3-6 deg/s at the hand-over asked 15-18 kN m of roll, the allocation was scaled to 0.5-0.7 on 85 % of the steps, the craft got half the pitch and yaw torque its plan flew on, and it left the plan within a second. Before KSA 5901 nothing rolled a booster, the roll demand was near nothing, and the allocation was whole.
        //  So an axis with under WeakAxisShare of the strongest's capability is allocated SEPARATELY and LAST: the other two take what they need, scaled together only if they alone exceed the travel, and the weak axis gets as much of its demand as the travel left over allows.
        int weak = -1;
        {
            double cx = cap.X, cy = cap.Y, cz = cap.Z;
            double strongest = Math.Max(cx, Math.Max(cy, cz));
            if (cx < WeakAxisShare * strongest && cx <= cy && cx <= cz) weak = 0;
            else if (cy < WeakAxisShare * strongest && cy <= cx && cy <= cz) weak = 1;
            else if (cz < WeakAxisShare * strongest && cz <= cx && cz <= cy) weak = 2;
        }
        // THE WEAK AXIS'S COUPLED SHARE (coupled) goes with the strong axes: the roll an off-centre engine's deflection brings with its pitch and yaw. Solved with the strong axes at zero roll, the verniers would take it out and the weak step would put it back, and on one engine of a triple plate the verniers' stops were the whole craft's limit. Only the rest of the weak demand is allocated apart.
        double coupledWeak = weak == 0 ? coupled.X : weak == 1 ? coupled.Y : weak == 2 ? coupled.Z : 0.0;
        double weakDemand = weak == 0 ? Math.Clamp(desiredTorque.X - coupledWeak, -cap.X, cap.X)
                          : weak == 1 ? Math.Clamp(desiredTorque.Y - coupledWeak, -cap.Y, cap.Y)
                          : weak == 2 ? Math.Clamp(desiredTorque.Z - coupledWeak, -cap.Z, cap.Z) : 0.0;
        var strong = new double3(weak == 0 ? coupledWeak : desiredTorque.X, weak == 1 ? coupledWeak : desiredTorque.Y, weak == 2 ? coupledWeak : desiredTorque.Z);

        // w = G^-1 tau, then u = B^T w.
        double wx = inv[0] * strong.X + inv[1] * strong.Y + inv[2] * strong.Z;
        double wy = inv[3] * strong.X + inv[4] * strong.Y + inv[5] * strong.Z;
        double wz = inv[6] * strong.X + inv[7] * strong.Y + inv[8] * strong.Z;

        double peak = 0.0;
        for (int i = 0; i < n; i++)
        {
            int c0 = 2 * i, c1 = c0 + 1;
            double u0 = bTorque[3 * c0] * wx + bTorque[3 * c0 + 1] * wy + bTorque[3 * c0 + 2] * wz;
            double u1 = bTorque[3 * c1] * wx + bTorque[3 * c1 + 1] * wy + bTorque[3 * c1 + 2] * wz;
            commands[c0] = u0;
            commands[c1] = u1;
            // KSA.Gimbal.ConstrainStateToCircle enforces a circular cone ||(u_y, u_z)||_2 <= 1, not a square box.
            peak = Math.Max(peak, Math.Sqrt(u0 * u0 + u1 * u1));
        }

        // Saturation: scale the WHOLE solution rather than clipping per component, so the torque DIRECTION survives.
        //  That policy is only sound when the axes have COMPARABLE authority. Measured in flight this vehicle has roll capability of 2.8 kN-m against 1902 kN-m in pitch and yaw - a factor of 700. A roll demand a few percent over its limit then dragged the entire command down to 29%, so pitch and yaw arrived at a third of their value despite being well inside capability. Preserving the direction of a vector whose smallest component is 700x cheaper to satisfy is not a meaningful thing to preserve.
        //  The weak axis is therefore clamped to what it can do and allocated apart, in the travel left (above and below); uniform scaling then handles only the strong axes' magnitude, rather than one weak axis vetoing the other two.
        //  And the scaling only finds the first gimbal at its stop: the rest of the demand then goes to the gimbals still free (Redistribute), so the smallest actuator no longer sets the whole craft's torque.
        if (peak > 1.0)
        {
            double scale = 1.0 / peak;
            for (int c = 0; c < cols; c++)
                commands[c] *= scale;
            result.SaturationScale = scale;
            if (redistribute)
            {
                Redistribute(bTorque, cols, strong, weak, lambda, commands, trackWeak: coupledWeak != 0.0);
                // The share of the strong axes' demand the allocation gives along it, their own axes only.
                double3 got = default;
                for (int c = 0; c < cols; c++)
                    got += new double3(bTorque[3 * c], bTorque[3 * c + 1], bTorque[3 * c + 2]) * commands[c];
                double3 lat = strong;
                if (weak == 0) { got = new double3(0.0, got.Y, got.Z); lat = new double3(0.0, lat.Y, lat.Z); }
                else if (weak == 1) { got = new double3(got.X, 0.0, got.Z); lat = new double3(lat.X, 0.0, lat.Z); }
                else if (weak == 2) { got = new double3(got.X, got.Y, 0.0); lat = new double3(lat.X, lat.Y, 0.0); }
                double want = double3.Dot(lat, lat);
                result.SaturationScale = want > 0.0 ? Math.Clamp(double3.Dot(got, lat) / want, 0.0, 1.0) : 1.0;
            }
        }

        // The weak axis, in the circular-cone travel left: largest share s in [0, 1] keeping ||u_i + s * ur_i||_2 <= 1 for every gimbal.
        result.WeakAxisScale = 1.0;
        if (weak >= 0 && weakDemand != 0.0)
        {
            double rx = inv[weak] * weakDemand, ry = inv[3 + weak] * weakDemand, rz = inv[6 + weak] * weakDemand;
            double share = 1.0;
            for (int i = 0; i < n; i++)
            {
                int c0 = 2 * i, c1 = c0 + 1;
                double ur0 = bTorque[3 * c0] * rx + bTorque[3 * c0 + 1] * ry + bTorque[3 * c0 + 2] * rz;
                double ur1 = bTorque[3 * c1] * rx + bTorque[3 * c1 + 1] * ry + bTorque[3 * c1 + 2] * rz;
                double aQuad = ur0 * ur0 + ur1 * ur1;
                if (aQuad < 1e-18)
                    continue;
                double u0 = commands[c0], u1 = commands[c1];
                double bQuad = 2.0 * (u0 * ur0 + u1 * ur1);
                double cQuad = u0 * u0 + u1 * u1 - 1.0;
                double disc = bQuad * bQuad - 4.0 * aQuad * cQuad;
                double sLim = disc <= 0.0 ? 0.0 : Math.Max(0.0, (-bQuad + Math.Sqrt(disc)) / (2.0 * aQuad));
                share = Math.Min(share, sLim);
            }
            share = Math.Clamp(share, 0.0, 1.0);
            for (int c = 0; c < cols; c++)
                commands[c] += share * (bTorque[3 * c] * rx + bTorque[3 * c + 1] * ry + bTorque[3 * c + 2] * rz);
            result.WeakAxisScale = share;
        }

        // Report what the linear model says we actually get, including the lateral force that comes along for free - gimballing for torque always tilts the thrust vector, and that coupling is real.
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

        // Per-axis capability, for scaling a UI slider or clamping a guidance demand: the torque produced by driving the min-norm solution for that axis to saturation.
        // This is achievable by construction. It is a torque in N*m about a KSA body axis, while KSA's TvcTorqueAuthority is an angular acceleration about a control axis, which FlightComputer.UpdateTvcParams takes from the smaller of the two TvcAllocation.ComputeTorqueRange directions, scaled to the current thrust and divided by the control-axis inertia.
        result.MaxTorque = new double3(
            AxisCapability(bTorque, cols, inv, 0),
            AxisCapability(bTorque, cols, inv, 1),
            AxisCapability(bTorque, cols, inv, 2));
        return result;
    }

    // Largest torque about one axis alone that the min-norm solution can deliver before some gimbal saturates its circular cone.
    private static double AxisCapability(double[] bTorque, int cols, ReadOnlySpan<double> inv, int axis)
    {
        double wx = inv[axis], wy = inv[3 + axis], wz = inv[6 + axis];
        double peak = 0.0, torque = 0.0;
        int n = cols / 2;
        for (int i = 0; i < n; i++)
        {
            int c0 = 2 * i, c1 = c0 + 1;
            double u0 = bTorque[3 * c0] * wx + bTorque[3 * c0 + 1] * wy + bTorque[3 * c0 + 2] * wz;
            double u1 = bTorque[3 * c1] * wx + bTorque[3 * c1 + 1] * wy + bTorque[3 * c1 + 2] * wz;
            peak = Math.Max(peak, Math.Sqrt(u0 * u0 + u1 * u1));
            torque += bTorque[3 * c0 + axis] * u0 + bTorque[3 * c1 + axis] * u1;
        }
        return peak > 0.0 ? torque / peak : 0.0;
    }

    /// <summary>
    /// SATURATED GIMBALS HOLD, THE OTHERS CARRY ON (cascaded generalised inverse). <paramref name="commands"/> comes in as the min-norm solution scaled until its first gimbal reached a stop. Each pass holds every gimbal at its circular-cone stop ||(u_y, u_z)||_2 = 1, solves the part of the strong axes' demand still missing on the gimbals left free, and moves them along that solution until the next reaches its circular-cone stop or the demand is met. The weak axis is held where the commands have it (its row's target is zero), and is allocated afterwards in the travel left, as before.
    ///  WHY. Scaling the whole solution to the first gimbal at its stop let the smallest actuator set the craft's torque. On 2026-11-21 one 10 deg engine of a triple plate landed with its main gimbal at a third of its travel while a 70 deg vernier sat at its stop, and the allocation delivered 51-69 % of the demand for the last 1.5 s of the burn: the craft could not make the turn its plan flew and touched down at 4.5 m/s sideways.
    ///  Where the free gimbals cannot make the missing torque without roll (a single engine whose verniers are at their stops), the solve is a least-squares compromise between the two, the regularisation keeping it well defined.
    /// </summary>
    /// <param name="trackWeak">The strong target carries the weak axis's coupled share (Solve's coupled): the passes keep making it, rather than holding the weak axis where the commands have it.</param>
    private static void Redistribute(double[] b, int cols, double3 strong, int weak, double lambda, double[] commands, bool trackWeak = false)
    {
        int n = cols / 2;
        Span<bool> held = stackalloc bool[n];
        Span<double> g = stackalloc double[9];
        double size = Math.Sqrt(double3.Dot(strong, strong));
        for (int pass = 0; pass < n; pass++)
        {
            int free = 0;
            for (int i = 0; i < n; i++)
            {
                double u0 = commands[2 * i], u1 = commands[2 * i + 1];
                if (!held[i] && Math.Sqrt(u0 * u0 + u1 * u1) >= 1.0 - 1e-9)
                    held[i] = true;
                if (!held[i])
                    free++;
            }
            if (free == 0)
                return;

            double3 got = default;
            for (int c = 0; c < cols; c++)
                got += new double3(b[3 * c], b[3 * c + 1], b[3 * c + 2]) * commands[c];
            var r = new double3(weak == 0 && !trackWeak ? 0.0 : strong.X - got.X, weak == 1 && !trackWeak ? 0.0 : strong.Y - got.Y,
                                weak == 2 && !trackWeak ? 0.0 : strong.Z - got.Z);
            if (!(Math.Sqrt(double3.Dot(r, r)) > 1e-9 * (size + 1.0)))
                return;

            g.Clear();
            for (int i = 0; i < n; i++)
            {
                if (held[i])
                    continue;
                for (int axis = 0; axis < 2; axis++)
                {
                    int c = 2 * i + axis;
                    double x = b[3 * c], y = b[3 * c + 1], z = b[3 * c + 2];
                    g[0] += x * x; g[1] += x * y; g[2] += x * z;
                    g[4] += y * y; g[5] += y * z;
                    g[8] += z * z;
                }
            }
            g[3] = g[1]; g[6] = g[2]; g[7] = g[5];
            g[0] += lambda; g[4] += lambda; g[8] += lambda;
            if (!Invert3x3(g, out Span<double> inv))
                return;
            double wx = inv[0] * r.X + inv[1] * r.Y + inv[2] * r.Z;
            double wy = inv[3] * r.X + inv[4] * r.Y + inv[5] * r.Z;
            double wz = inv[6] * r.X + inv[7] * r.Y + inv[8] * r.Z;

            // As far along this solution as the first free gimbal's circular-cone stop allows, or all of it.
            double step = 1.0;
            for (int i = 0; i < n; i++)
            {
                if (held[i])
                    continue;
                int c0 = 2 * i, c1 = c0 + 1;
                double du0 = b[3 * c0] * wx + b[3 * c0 + 1] * wy + b[3 * c0 + 2] * wz;
                double du1 = b[3 * c1] * wx + b[3 * c1 + 1] * wy + b[3 * c1 + 2] * wz;
                double aQuad = du0 * du0 + du1 * du1;
                if (aQuad < 1e-18)
                    continue;
                double u0 = commands[c0], u1 = commands[c1];
                double bQuad = 2.0 * (u0 * du0 + u1 * du1);
                double cQuad = u0 * u0 + u1 * u1 - 1.0;
                double disc = bQuad * bQuad - 4.0 * aQuad * cQuad;
                double room = disc <= 0.0 ? 0.0 : Math.Max(0.0, (-bQuad + Math.Sqrt(disc)) / (2.0 * aQuad));
                step = Math.Min(step, room);
            }
            for (int i = 0; i < n; i++)
            {
                if (held[i])
                    continue;
                int c0 = 2 * i, c1 = c0 + 1;
                commands[c0] += step * (b[3 * c0] * wx + b[3 * c0 + 1] * wy + b[3 * c0 + 2] * wz);
                commands[c1] += step * (b[3 * c1] * wx + b[3 * c1 + 1] * wy + b[3 * c1 + 2] * wz);
            }
            if (step >= 1.0)
                return;
        }
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

    /// <summary>Torque the linear model says the returned commands deliver, N*m.</summary>
    public double3 AchievedTorque;

    /// <summary>Lateral force that comes with it, N. Gimballing for torque always tilts the thrust vector.</summary>
    public double3 AchievedForce;

    /// <summary>Per-axis achievable torque at saturation, N*m.</summary>
    public double3 MaxTorque;

    /// <summary>1 when unsaturated; below 1 the demand was scaled down to fit, preserving direction.</summary>
    public double SaturationScale;

    /// <summary>True once the demand has been clipped per axis to real capability.</summary>
    public bool DemandClamped;

    /// <summary>The share of the weak axis's demand (roll, for engines near the thrust line) that fitted in the gimbal travel the other two left; 1 when all of it did, or when no axis is weak.</summary>
    public double WeakAxisScale;
}
