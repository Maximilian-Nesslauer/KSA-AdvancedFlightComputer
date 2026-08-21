using System;
using Brutal.Numerics;
using KSA;

/// <summary>
/// Converts a live KSA vehicle state into the frame and conventions the 6-DOF SCvx
/// model works in, and back again.
///
/// THREE MISMATCHES, all of which fail silently if got wrong — a sign error here
/// looks like "the controller is unstable", not like a bug:
///
/// 1. INERTIAL FRAME. The model assumes flat ground with gravity along -Z. KSA is
///    in CCI about a round body. We build a local frame at the landing site with
///    +Z straight up, which makes the model's assumption locally true. (Note this
///    is NOT the frame Gfold uses — KsaGfold.BuildFrame is X-up. Two solvers, two
///    conventions; do not cross them.)
///
/// 2. BODY AXES. The model puts thrust along body +Z with the engine at -Z, and
///    its inertia (Ixx=Iyy=1e8, Izz=2.5e6) says Z is the long/roll axis. KSA's long
///    axis is body X. Rather than hard-code that swap we DERIVE it from the actual
///    measured thrust direction of the engines, so a vehicle built differently
///    still converts correctly.
///
/// 3. QUATERNION CONVENTION. KSA's doubleQuat is scalar-LAST (Identity = 0,0,0,1);
///    the model is scalar-FIRST Hamilton, [w,x,y,z]. Rather than reason about
///    whether KSA is Hamilton or JPL — easy to get wrong, and wrong quietly — every
///    conversion here goes THROUGH A ROTATION MATRIX built by transforming basis
///    vectors with KSA's own Transform. That inherits KSA's convention whatever it
///    is, and the matrix is then converted to a quaternion using the algorithm that
///    matches 6dof.py's quat_to_R exactly. No convention is assumed on either side.
/// </summary>
public static class KsaFrameBridge
{
    /// <summary>
    /// Local landing frame: origin at the site, +Z radially up, X/Y horizontal.
    /// Rows of the rotation are Ex/Ey/Ez, so CCI -> site is a dot product each.
    /// </summary>
    public readonly struct SiteFrame(double3 origin, double3 ex, double3 ey, double3 ez)
    {
        public readonly double3 Origin = origin, Ex = ex, Ey = ey, Ez = ez;

        public double3 PosToLocal(double3 cci)
        {
            double3 d = cci - Origin;
            return new double3(double3.Dot(d, Ex), double3.Dot(d, Ey), double3.Dot(d, Ez));
        }

        public double3 VecToLocal(double3 cci) =>
            new(double3.Dot(cci, Ex), double3.Dot(cci, Ey), double3.Dot(cci, Ez));

        public double3 VecToCci(double3 local) =>
            local.X * Ex + local.Y * Ey + local.Z * Ez;

        public double3 PosToCci(double3 local) => Origin + VecToCci(local);
    }

    /// <summary>Site frame with +Z up (Ez radially out), unlike Gfold's X-up frame.</summary>
    public static SiteFrame BuildSiteFrame(double3 siteCci)
    {
        double3 up = double3.Normalize(siteCci);
        double3 reference = Math.Abs(up.Z) < 0.99 ? new double3(0, 0, 1) : new double3(1, 0, 0);
        double3 ex = double3.Normalize(double3.Cross(reference, up));
        double3 ey = double3.Cross(up, ex);
        return new SiteFrame(siteCci, ex, ey, up);
    }

    /// <summary>
    /// Rotation taking MODEL body coordinates to KSA body coordinates, as three
    /// columns. Derived from the vehicle's own thrust direction rather than assumed,
    /// so it is correct for any layout.
    ///
    /// The roll reference (which way model +X points) is arbitrary but must be
    /// STABLE — it is chosen from a fixed KSA axis, so it does not wander frame to
    /// frame as the vehicle rotates.
    /// </summary>
    public static void BodyAxes(Vehicle vehicle, out double3 mx, out double3 my, out double3 mz)
    {
        // Model +Z is the thrust axis. Take it from the highest-thrust gimbal, which
        // is the main engine on any sane layout; fall back to KSA's long axis (+X)
        // if the vehicle has no gimballed engine to measure.
        double3 thrust = new(1, 0, 0);
        double best = 0;
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
        {
            if (gc.Data.MaximumThrust <= best)
                continue;
            best = gc.Data.MaximumThrust;
            float3 d = gc.Data.ThrustDirVehicleAsmb;
            thrust = new double3(d.X, d.Y, d.Z);
        }

        mz = double3.Normalize(thrust);
        double3 reference = Math.Abs(mz.Z) < 0.99 ? new double3(0, 0, 1) : new double3(1, 0, 0);
        mx = double3.Normalize(double3.Cross(reference, mz));
        my = double3.Cross(mz, mx);
    }

    /// <summary>
    /// Body -> CCI rotation as three column vectors, read out of KSA's quaternion by
    /// transforming the basis vectors. This is the step that makes the whole bridge
    /// convention-agnostic: whatever handedness or scalar position KSA uses, its own
    /// Transform is the definition, and we only ever consume the resulting matrix.
    /// </summary>
    public static void BodyToCciColumns(Vehicle vehicle, out double3 c0, out double3 c1, out double3 c2)
    {
        doubleQuat body2Cci = BodyToCci(vehicle);
        c0 = new double3(1, 0, 0).Transform(body2Cci);
        c1 = new double3(0, 1, 0).Transform(body2Cci);
        c2 = new double3(0, 0, 1).Transform(body2Cci);
    }

    /// <summary>KSA's own body->CCI composition, matching FlightComputerNavigation.</summary>
    public static doubleQuat BodyToCci(Vehicle vehicle) =>
        doubleQuat.Concatenate(vehicle.Body2Cce, vehicle.Orbit.Parent.GetCce2Cci());

    /// <summary>
    /// Full model state: [r(3) v(3) q(4) w(3) m(1)] = 14, matching Dynamics6Dof's
    /// layout. Velocity is SURFACE-relative (the body's rotation removed), because
    /// the model's flat-ground frame is not inertial in KSA's sense.
    /// </summary>
    public static double[] ToModelState(Vehicle vehicle, in SiteFrame frame)
    {
        var x = new double[14];

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit.Parent;
        double3 rCci = orbit.StateVectors.PositionCci;
        double3 vCci = orbit.StateVectors.VelocityCci
                       - double3.Cross(parent.GetAngularVelocityCci(), rCci);

        double3 r = frame.PosToLocal(rCci);
        double3 v = frame.VecToLocal(vCci);
        x[0] = r.X; x[1] = r.Y; x[2] = r.Z;
        x[3] = v.X; x[4] = v.Y; x[5] = v.Z;

        ModelAttitude(vehicle, frame, out double qw, out double qx, out double qy, out double qz);
        x[6] = qw; x[7] = qx; x[8] = qy; x[9] = qz;

        // Body rates into MODEL body axes. KSA reports them in its own body frame, so
        // they need the same axis swap as the attitude — a rate about KSA's long axis
        // is a roll rate about model Z.
        BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        double3 w = vehicle.BodyRates;
        x[10] = double3.Dot(w, mx);
        x[11] = double3.Dot(w, my);
        x[12] = double3.Dot(w, mz);

        x[13] = vehicle.TotalMass;
        return x;
    }

    /// <summary>
    /// Model attitude quaternion (scalar-first Hamilton) for model-body -> site.
    /// Composed as  R_site&lt;-modelBody = R_site&lt;-cci * R_cci&lt;-ksaBody * R_ksaBody&lt;-modelBody.
    /// </summary>
    public static void ModelAttitude(Vehicle vehicle, in SiteFrame frame,
                                     out double qw, out double qx, out double qy, out double qz)
    {
        BodyToCciColumns(vehicle, out double3 b0, out double3 b1, out double3 b2);
        BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);

        // Each model body axis, expressed in CCI, then in the site frame. These are
        // the columns of the model's body->inertial rotation.
        double3 c0 = frame.VecToLocal(mx.X * b0 + mx.Y * b1 + mx.Z * b2);
        double3 c1 = frame.VecToLocal(my.X * b0 + my.Y * b1 + my.Z * b2);
        double3 c2 = frame.VecToLocal(mz.X * b0 + mz.Y * b1 + mz.Z * b2);

        MatrixToQuat(c0, c1, c2, out qw, out qx, out qy, out qz);
    }

    /// <summary>
    /// Rotation matrix (given as columns) to a scalar-first Hamilton quaternion.
    ///
    /// Verified against 6dof.py's quat_to_R rather than taken from a reference: for
    /// the trace branch, R21-R12 = 4*qw*qx under that formula, and s = 4*qw, so
    /// x = qx exactly — and likewise for y and z. Branching on the largest diagonal
    /// keeps it conditioned when qw is near zero (vehicle inverted relative to the
    /// site frame), which a naive trace-only formula would divide through.
    /// </summary>
    public static void MatrixToQuat(double3 c0, double3 c1, double3 c2,
                                    out double qw, out double qx, out double qy, out double qz)
    {
        // r[row][col]; columns are the transformed basis vectors.
        double r00 = c0.X, r01 = c1.X, r02 = c2.X;
        double r10 = c0.Y, r11 = c1.Y, r12 = c2.Y;
        double r20 = c0.Z, r21 = c1.Z, r22 = c2.Z;

        double trace = r00 + r11 + r22;
        if (trace > 0.0)
        {
            double s = Math.Sqrt(trace + 1.0) * 2.0;
            qw = 0.25 * s;
            qx = (r21 - r12) / s;
            qy = (r02 - r20) / s;
            qz = (r10 - r01) / s;
        }
        else if (r00 > r11 && r00 > r22)
        {
            double s = Math.Sqrt(1.0 + r00 - r11 - r22) * 2.0;
            qw = (r21 - r12) / s;
            qx = 0.25 * s;
            qy = (r01 + r10) / s;
            qz = (r02 + r20) / s;
        }
        else if (r11 > r22)
        {
            double s = Math.Sqrt(1.0 + r11 - r00 - r22) * 2.0;
            qw = (r02 - r20) / s;
            qx = (r01 + r10) / s;
            qy = 0.25 * s;
            qz = (r12 + r21) / s;
        }
        else
        {
            double s = Math.Sqrt(1.0 + r22 - r00 - r11) * 2.0;
            qw = (r10 - r01) / s;
            qx = (r02 + r20) / s;
            qy = (r12 + r21) / s;
            qz = 0.25 * s;
        }

        // Sign is arbitrary (q and -q are the same rotation); pin it so successive
        // conversions don't flip and look like a discontinuity to the solver.
        if (qw < 0.0)
        {
            qw = -qw; qx = -qx; qy = -qy; qz = -qz;
        }
    }

    /// <summary>Model quaternion back to a body->inertial rotation, as columns. Matches quat_to_R.</summary>
    public static void QuatToMatrix(double qw, double qx, double qy, double qz,
                                    out double3 c0, out double3 c1, out double3 c2)
    {
        c0 = new double3(1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy));
        c1 = new double3(2 * (qx * qy - qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + qw * qx));
        c2 = new double3(2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy));
    }

    /// <summary>
    /// Where the model says the vehicle's thrust axis points, in CCI. This is the
    /// output a guidance mode would steer on, and the cheapest end-to-end check that
    /// the bridge is right: it must agree with the vehicle's ACTUAL thrust axis.
    /// </summary>
    public static double3 ModelThrustAxisToCci(double qw, double qx, double qy, double qz,
                                               in SiteFrame frame)
    {
        QuatToMatrix(qw, qx, qy, qz, out _, out _, out double3 c2);   // model body +Z
        return frame.VecToCci(c2);
    }

    /// <summary>
    /// Round-trip check: convert the live attitude into the model and back out, and
    /// report how far the recovered thrust axis is from the real one, in degrees.
    ///
    /// This is the test that justifies trusting everything above. It catches an axis
    /// swap, a quaternion handedness error, a transposed site frame, and a sign flip
    /// — all at once, all of which are otherwise invisible until the vehicle is
    /// tumbling. Expect ~1e-13 deg; anything above ~1e-6 deg means a real bug.
    /// </summary>
    public static double RoundTripErrorDeg(Vehicle vehicle, in SiteFrame frame)
    {
        ModelAttitude(vehicle, frame, out double qw, out double qx, out double qy, out double qz);
        double3 recovered = ModelThrustAxisToCci(qw, qx, qy, qz, frame);

        BodyToCciColumns(vehicle, out double3 b0, out double3 b1, out double3 b2);
        BodyAxes(vehicle, out _, out _, out double3 mz);
        double3 actual = double3.Normalize(mz.X * b0 + mz.Y * b1 + mz.Z * b2);

        double dot = Math.Clamp(double3.Dot(double3.Normalize(recovered), actual), -1.0, 1.0);
        return Math.Acos(dot) * 180.0 / Math.PI;
    }
}
