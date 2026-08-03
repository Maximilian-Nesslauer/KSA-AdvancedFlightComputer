using System;
using Brutal.Numerics;
using Gfold;
using KSA;

// Bridges KSA's live vehicle/world state to the standalone G-FOLD convex
// powered-descent solver (Gfold.Core) and back. G-FOLD works in a local
// "x is up" frame at the landing site, with uniform gravity and a frame it
// treats as inertial. We build that frame, express the vehicle state in it
// (surface-relative, so vf = 0 means at rest on the ground), and after a solve
// map the first commanded acceleration back to a CCI thrust direction +
// throttle.
//
// Approximations (all good for a short lunar terminal descent, flagged because
// they matter for fast-rotating or atmospheric bodies):
//  - constant gravity, evaluated at the site;
//  - the site frame rotates with the body but is treated as inertial — the
//    Coriolis/centrifugal terms the original paper carries are dropped, the
//    same simplification as Gfold.Core's v_dot = g + u dynamics;
//  - no aerodynamics (correct for vacuum worlds; would need a separate entry
//    phase in atmosphere).
internal static class KsaGfold
{
    private const double G0 = 9.80665;

    // Orthonormal site frame: X = local up (radial), Y/Z span the horizon. The
    // horizontal axes are arbitrary — G-FOLD's dynamics and glideslope cone are
    // symmetric in the horizontal plane — so any stable pair will do.
    internal readonly struct Frame
    {
        public readonly double3 Origin;          // site at terrain height, CCI
        public readonly double3 Ex, Ey, Ez;

        public Frame(double3 origin, double3 ex, double3 ey, double3 ez)
        {
            Origin = origin; Ex = ex; Ey = ey; Ez = ez;
        }

        public double3 PointToLocal(double3 cci)
        {
            double3 d = cci - Origin;
            return new double3(double3.Dot(d, Ex), double3.Dot(d, Ey), double3.Dot(d, Ez));
        }

        public double3 VecToLocal(double3 cci) =>
            new double3(double3.Dot(cci, Ex), double3.Dot(cci, Ey), double3.Dot(cci, Ez));

        public double3 VecToCci(double3 local) =>
            local.X * Ex + local.Y * Ey + local.Z * Ez;
    }

    internal static Frame BuildFrame(double3 siteCci)
    {
        double3 up = double3.Normalize(siteCci);
        double3 reference = Math.Abs(up.Z) < 0.99 ? new double3(0, 0, 1) : new double3(1, 0, 0);
        double3 ey = double3.Normalize(double3.Cross(up, reference));
        double3 ez = double3.Cross(up, ey);
        return new Frame(siteCci, up, ey, ez);
    }

    // Builds the G-FOLD parameter set from the live vehicle and site. Returns
    // null if the vehicle has no usable engine (no thrust to plan with).
    // refPosCci is the reference point the solve plans for — the vehicle CoM.
    // (The vehicle-height allowance is applied by the caller as an offset on the
    // TARGET altitude, never by shifting this reference: an attitude-dependent
    // reference point injects modelling error as the vehicle rotates.)
    // arrivalAltM / arrivalRateMs implement "Option B": the target is a point
    // arrivalAltM directly above the pad, reached descending vertically at
    // arrivalRateMs with zero horizontal velocity — so G-FOLD nulls cross-range up
    // high and the terminal vertical phase only has to kill the remaining sink.
    internal static GfoldParams BuildParams(
        Vehicle vehicle, IParentBody parent, Frame frame, double3 siteCci, double3 refPosCci,
        double glideSlopeDeg, double pointingDeg, double vMax,
        double arrivalAltM, double arrivalRateMs, double throttleMin, double throttleMax)
    {
        (double thrust, double massFlow) = KsaEnginePerf.Vacuum(vehicle);
        double exhaustVel = massFlow > 0 ? thrust / massFlow : 0.0;
        if (thrust <= 0 || exhaustVel <= 0)
            return null;

        double mass = vehicle.TotalMass;
        double prop = vehicle.PropellantMass;

        double siteR = siteCci.Length();
        double g = parent.Mu / (siteR * siteR);

        double3 r = refPosCci;   // vehicle CoM
        double3 v = vehicle.Orbit.StateVectors.VelocityCci;
        // Surface-relative velocity: the site frame co-rotates with the body, so
        // remove the rotation to make "land at rest on the ground" = vf 0.
        double3 vSrf = v - double3.Cross(parent.GetAngularVelocityCci(), r);

        double3 rLocal = frame.PointToLocal(r);
        double3 vLocal = frame.VecToLocal(vSrf);

        // Only the speed cap is loosened to admit a fast handoff state — it relaxes
        // back toward the set value as the vehicle slows, so it never ratchets. The
        // glideslope and pointing angles are passed through exactly as set, so the
        // user's numbers are what's actually enforced; the fixed initial state is
        // handled by skipping node 0 (RelaxInitialPath), not by overriding them.
        double speed = vLocal.Length();
        double vMaxEff = Math.Max(vMax, 1.3 * speed);

        // Solver thrust bounds (fractions of max), set by the user. These constrain the
        // planned trajectory only; the tracker still commands the full 0-100% range.
        double tMin = Math.Clamp(throttleMin, 0.01, 0.95);
        double tMax = Math.Clamp(throttleMax, tMin + 0.02, 1.0);

        return new GfoldParams
        {
            GravityMag = g,
            DryMass = Math.Max(mass - prop, 1.0),
            FuelMass = Math.Max(prop, 0.0),
            Isp = exhaustVel / G0,
            ThrustMax = thrust,
            ThrottleMin = tMin,
            ThrottleMax = tMax,
            VMax = vMaxEff,
            GlideSlopeDeg = glideSlopeDeg,
            PointingMaxDeg = Math.Clamp(pointingDeg, 1.0, 89.0),
            R0 = [rLocal.X, rLocal.Y, rLocal.Z],
            V0 = [vLocal.X, vLocal.Y, vLocal.Z],
            Rf = [arrivalAltM, 0.0, 0.0],      // above the pad (x = up)
            Vf = [-arrivalRateMs, 0.0, 0.0],   // descending vertically, no cross-range
        };
    }

    // The first-node control of a solved trajectory, mapped back to the world:
    // a unit CCI thrust direction and a throttle fraction (commanded thrust over
    // max). G-FOLD's u already excludes gravity, so thrust = mass * |u|.
    internal static (double3 dirCci, double throttle) FirstControl(
        GfoldTrajectory traj, Frame frame, double mass, double thrustMax)
    {
        double[] u0 = traj.AccelCmd[0];
        var uLocal = new double3(u0[0], u0[1], u0[2]);
        double accel = uLocal.Length();

        // Guard against a non-finite or zero command (degenerate/inaccurate solve):
        // point up at the throttle the magnitude implies.
        if (!IsFinite(uLocal) || accel < 1e-9)
            return (frame.Ex, Throttle(accel, mass, thrustMax));

        // Node 0's pointing is unconstrained (RelaxInitialPath), so clamp the
        // command to the upper hemisphere — never thrust below the local horizon
        // (into the ground), which would only ever be a solver artefact.
        var dirLocal = new double3(Math.Max(uLocal.X, 0.0), uLocal.Y, uLocal.Z);
        if (dirLocal.Length() < 1e-9)
            dirLocal = new double3(1, 0, 0);

        double3 dirCci = double3.Normalize(frame.VecToCci(dirLocal));
        return (dirCci, Throttle(accel, mass, thrustMax));
    }

    private static double Throttle(double accel, double mass, double thrustMax) =>
        thrustMax > 0 ? Math.Clamp(accel * mass / thrustMax, 0.0, 1.0) : 0.0;

    private static bool IsFinite(double3 v) =>
        !double.IsNaN(v.X) && !double.IsNaN(v.Y) && !double.IsNaN(v.Z) &&
        !double.IsInfinity(v.X) && !double.IsInfinity(v.Y) && !double.IsInfinity(v.Z);
}
