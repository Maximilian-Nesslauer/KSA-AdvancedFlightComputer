#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;

// The launch-to-target geometry the Ascent tab draws.
// Pure computation: no ImGui, no state writes.
public static partial class GuidanceWindow
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
    /// <see cref="ChaseStatus.PlaneUnreachable"/> - only <see cref="WaitSec"/> is
    /// meaningless in the latter, because there is no crossing to wait for.
    /// </summary>
    public struct ChasePlan
    {
        public Vehicle Target;
        public double IncDeg, LanDeg, PeKm, ApKm;    // the chase orbit
        public double ArgPeDeg;                      // the chase orbit's, which is the target's; NaN when the target is too round to have one
        public double TargetPeKm, TargetApKm;        // the target's own, for display
        public double WaitSec;                       // to ignition for the next crossing, whichever node that is
        public bool Descending;                      // the next crossing is the descending one (south-easterly launch)
        public bool NearestDescending;               // the crossing nearest in time, just missed or coming up, is the descending one
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

        // Target plane straight from its state vectors: n = r * v.
        // With our LAN convention Normal = (sin i sin Omega, -sin i cos Omega, cos i), so Omega = atan2(nx, -ny).
        // An exactly equatorial plane has no node, and atan2(0, -0) would call it pi: KSA's own elements put the node on +X there (LAN 0), and the argument of periapsis copied below is measured from it, so it has to be the same node.
        double3 rt = targetOrbit.StateVectors.PositionCci;
        double3 vt = targetOrbit.StateVectors.VelocityCci;
        double3 n = double3.Normalize(double3.Cross(rt, vt));
        double incT = Math.Acos(Math.Clamp(n.Z, -1.0, 1.0));
        double lanT = n.X == 0.0 && n.Y == 0.0 ? 0.0 : Wrap2Pi(Math.Atan2(n.X, -n.Y));

        plan.TargetPeKm = (targetOrbit.Periapsis - bodyRadius) / 1000.0;
        plan.TargetApKm = (targetOrbit.Apoapsis - bodyRadius) / 1000.0;

        // Chase orbit: CO-ELLIPTIC with the target - its eccentricity and argument of periapsis, on a semi-major axis the chosen offset below its own.
        // Sharing the line of apsides keeps the chase orbit inside the target's all the way round, where a circular one under an eccentric target crosses it.
        // Against a circular target this is the circular chase orbit it replaces.
        double targetSma = (targetOrbit.Periapsis + targetOrbit.Apoapsis) / 2.0;
        double ecc = (targetOrbit.Apoapsis - targetOrbit.Periapsis)
                   / (targetOrbit.Apoapsis + targetOrbit.Periapsis);
        double chaseSma = targetSma - _s.ChaseOffsetKm * 1000.0;
        double argPe = UpfgTarget.ArgumentOfPeriapsisOf(rt, vt, parent.Mu, lanT);

        plan.IncDeg = UpfgTarget.RadToDeg(incT);
        plan.LanDeg = UpfgTarget.RadToDeg(lanT);
        plan.PeKm = (chaseSma * (1.0 - ecc) - bodyRadius) / 1000.0;
        plan.ApKm = (chaseSma * (1.0 + ecc) - bodyRadius) / 1000.0;
        plan.ArgPeDeg = double.IsNaN(argPe) ? double.NaN : UpfgTarget.RadToDeg(argPe);

        // Launch window: how long until the body's rotation carries the launch site under the target plane, at whichever crossing - ascending or descending - comes first.
        // Either puts the vehicle in the same plane; they differ only in heading.
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
        double omega = parent.GetAngularVelocity();

        // LAUNCH EARLY, by the same lead the LAN seeding uses (see LanLeadSeconds).
        // The instant the site is in the plane is the wrong instant to LIGHT THE ENGINE: the pad goes on being carried east through the vertical rise and the turn, so a vehicle that ignites in the plane is already out of it by the time it is flying, and the guidance yaws to chase a node it has gone past.
        // What we want is the launch time whose plane crossing lands in the middle of that, i.e. solve for T where the site's right ascension T + lead from now is the one the plane needs.
        //
        // The lead goes INSIDE the wrap, not subtracted after it: subtracting after would turn a window two minutes out into a negative countdown.
        if (omega <= 1e-12)
        {
            plan.WaitSec = double.NaN;
            return ChaseStatus.Ok;
        }

        double raNow = ra + omega * LanLeadSeconds;
        double period = 2.0 * Math.PI / omega;

        double WaitFor(bool descending)
        {
            double raRequired = descending ? lanT + Math.PI - delta : lanT + delta;
            double wait = Wrap2Pi(raRequired - raNow) / omega;

            // ...
            // AND THE WRAP IS NOT ALLOWED TO COST A WHOLE REVOLUTION.
            // Inside the lead window - the ideal ignition is behind us but the plane crossing itself is still ahead - the wrap reports the NEXT revolution's launch, so pressing EXECUTE armed a countdown of most of a day and looked like a dead button.
            // Going now is at most LanLeadSeconds late, which only means the crossing lands earlier in the ascent than the lead intends; waiting a revolution for that is absurd.
            if (wait > period - LanLeadSeconds)
                wait = 0.0;
            return wait;
        }

        double waitAscending = WaitFor(false);
        double waitDescending = WaitFor(true);
        plan.Descending = waitDescending < waitAscending;
        plan.WaitSec = Math.Min(waitAscending, waitDescending);

        // For launching off the window: the crossing nearest in time, counting one just missed as behind by the rest of the revolution it would otherwise wait.
        plan.NearestDescending = Math.Min(waitDescending, period - waitDescending)
                               < Math.Min(waitAscending, period - waitAscending);
        return ChaseStatus.Ok;
    }

    /// <summary>
    /// Drives the target-orbit inputs from the chase plan. The gauge panel calls this
    /// every frame while a target is selected - which is why those inputs are greyed
    /// out there: they are outputs of the target pick, not independent settings.
    ///
    /// The argument of periapsis follows the target's only while MatchTargetArgPe is
    /// set, so clearing that leaves the chase orbit to insert at its own periapsis. A
    /// target too round to have one leaves nothing to copy, and the chase is free.
    /// </summary>
    private static void ApplyChaseOrbit(in ChasePlan plan)
    {
        _s.IncDeg = plan.IncDeg;
        _s.LanDeg = plan.LanDeg;
        _s.PeKm = plan.PeKm;
        _s.ApKm = plan.ApKm;
        bool hasArgPe = !double.IsNaN(plan.ArgPeDeg);
        if (hasArgPe)
            _s.ArgPeDeg = plan.ArgPeDeg;
        _s.ArgPeFixed = _s.MatchTargetArgPe && hasArgPe;
        _s.LanSeeded = true;
        // The node follows the next crossing only until a launch is committed: an armed countdown is to one particular crossing, and once flying, the other becoming the next must not turn the ascent's heading round.
        if (!_s.Running && !_s.LaunchArmed)
            _s.LaunchDescending = plan.Descending;
    }
}
