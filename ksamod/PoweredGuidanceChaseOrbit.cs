using System;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Upfg;

// The launch-to-target geometry, extracted from the Ascent tab's draw so the gauge
// panel and the legacy tab compute it exactly once between them. Pure computation:
// no ImGui, no state writes.
public static partial class PoweredGuidanceWindow
{
    public enum ChaseStatus
    {
        NoTarget,           // nothing picked
        NotFound,           // an id is set but no such vehicle exists any more
        DifferentBody,      // target orbits something else entirely
        PlaneUnreachable,   // target inclination is below the launch site latitude
        Ok,
    }

    /// <summary>
    /// The chase orbit to fly and the launch window to fly it at. Orbit fields are
    /// valid for both <see cref="ChaseStatus.Ok"/> and
    /// <see cref="ChaseStatus.PlaneUnreachable"/> — only <see cref="WaitSec"/> is
    /// meaningless in the latter, because there is no crossing to wait for.
    /// </summary>
    public struct ChasePlan
    {
        public Vehicle Target;
        public double IncDeg, LanDeg, PeKm, ApKm;    // the chase orbit
        public double TargetPeKm, TargetApKm;        // the target's own, for display
        public double WaitSec;
    }

    private static ChaseStatus TryChaseOrbit(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                             double bodyRadius, out ChasePlan plan)
    {
        plan = default;

        if (_s.TargetId.Length == 0)
            return ChaseStatus.NoTarget;

        Vehicle target = FindVehicleById(_s.TargetId, vehicle);
        if (target == null)
            return ChaseStatus.NotFound;

        Orbit targetOrbit = target.Orbit;
        if (!ReferenceEquals(targetOrbit.Parent, orbit.Parent))
            return ChaseStatus.DifferentBody;

        plan.Target = target;

        // Target plane straight from its state vectors: n = r × v. With our LAN
        // convention Normal = (sin i sin Ω, −sin i cos Ω, cos i), so Ω = atan2(nx, −ny).
        double3 rt = targetOrbit.StateVectors.PositionCci;
        double3 vt = targetOrbit.StateVectors.VelocityCci;
        double3 n = double3.Normalize(double3.Cross(rt, vt));
        double incT = Math.Acos(Math.Clamp(n.Z, -1.0, 1.0));
        double lanT = Wrap2Pi(Math.Atan2(n.X, -n.Y));

        plan.TargetPeKm = (targetOrbit.Periapsis - bodyRadius) / 1000.0;
        plan.TargetApKm = (targetOrbit.Apoapsis - bodyRadius) / 1000.0;

        // Chase orbit: circular, with semi-major axis the chosen offset below the
        // target's. A true co-elliptic depends on launch phasing anyway — circular
        // is a clean baseline to correct from once up.
        double targetSmaKm = (targetOrbit.Periapsis + targetOrbit.Apoapsis) / 2000.0;
        double chaseAltKm = targetSmaKm - bodyRadius / 1000.0 - _s.ChaseOffsetKm;

        plan.IncDeg = UpfgTarget.RadToDeg(incT);
        plan.LanDeg = UpfgTarget.RadToDeg(lanT);
        plan.PeKm = chaseAltKm;
        plan.ApKm = chaseAltKm;

        // Launch window: how long until the body's rotation carries the launch site
        // under the target plane, at the requested (ascending/descending) crossing.
        double3 r = orbit.StateVectors.PositionCci;
        double lat = Math.Asin(Math.Clamp(r.Z / r.Length(), -1.0, 1.0));
        double ra = Math.Atan2(r.Y, r.X);
        double tanRatio = Math.Tan(lat) / Math.Tan(Math.Max(incT, 1e-6));
        if (Math.Abs(tanRatio) > 1.0)
        {
            plan.WaitSec = double.NaN;
            return ChaseStatus.PlaneUnreachable;
        }

        double delta = Math.Asin(Math.Clamp(tanRatio, -1.0, 1.0));
        double raRequired = _s.LaunchDescending ? lanT + Math.PI - delta : lanT + delta;
        double omega = parent.GetAngularVelocity();

        // LAUNCH EARLY, by the same lead the LAN seeding uses (see LanLeadSeconds).
        // The instant the site is in the plane is the wrong instant to LIGHT THE
        // ENGINE: the pad goes on being carried east through the vertical rise and the
        // turn, so a vehicle that ignites in the plane is already out of it by the time
        // it is flying, and the guidance yaws to chase a node it has gone past. What we
        // want is the launch time whose plane crossing lands in the middle of that,
        // i.e. solve for T where the site's right ascension T + lead from now is the
        // one the plane needs.
        //
        // The lead goes INSIDE the wrap, not subtracted after it: subtracting after
        // would turn a window two minutes out into a negative countdown.
        if (omega <= 1e-12)
        {
            plan.WaitSec = double.NaN;
            return ChaseStatus.Ok;
        }

        double raNow = ra + omega * LanLeadSeconds;
        double wait = Wrap2Pi(raRequired - raNow) / omega;

        // ... AND THE WRAP IS NOT ALLOWED TO COST A WHOLE REVOLUTION. Inside the lead
        // window — the ideal ignition is behind us but the plane crossing itself is
        // still ahead — the wrap reports the NEXT revolution's launch, so pressing
        // EXECUTE armed a countdown of most of a day and looked like a dead button.
        // Going now is at most LanLeadSeconds late, which only means the crossing
        // lands earlier in the ascent than the lead intends; waiting a revolution for
        // that is absurd.
        double period = 2.0 * Math.PI / omega;
        if (wait > period - LanLeadSeconds)
            wait = 0.0;

        plan.WaitSec = wait;
        return ChaseStatus.Ok;
    }

    /// <summary>
    /// Drives the target-orbit inputs from the chase plan. The gauge panel calls this
    /// every frame while a target is selected — which is why those inputs are greyed
    /// out there: they are outputs of the target pick, not independent settings.
    /// </summary>
    private static void ApplyChaseOrbit(in ChasePlan plan)
    {
        _s.IncDeg = plan.IncDeg;
        _s.LanDeg = plan.LanDeg;
        _s.PeKm = plan.PeKm;
        _s.ApKm = plan.ApKm;
        _s.LanSeeded = true;
    }
}
