#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;

/// <summary>
/// An orbit plane for the ascent to launch into, fixed in inertial space, with the orbit to insert into in it. The ascent chases it as it does a target vessel's plane, so the launch arms for its launch window. Label names it on the LAUNCH page.
/// </summary>
public sealed record AscentPlaneTarget(double IncDeg, double LanDeg, double PeKm, double ApKm, string Label);

// The launch-to-target geometry the LAUNCH page draws.
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

    /// <summary>The ascent has a plane to wait for: a target vessel's, or a plane target.</summary>
    private static bool HasLaunchTarget => _s.TargetId.Length > 0 || _s.PlaneTarget != null;

    private static ChaseStatus TryChaseOrbit(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                             double bodyRadius, out ChasePlan plan)
    {
        plan = default;

        double incT, lanT;
        if (_s.TargetId.Length == 0)
        {
            // A plane target is its own chase orbit: the plane as given, and the orbit it was planned with. It has no argument of periapsis to copy, so the insertion stays free.
            AscentPlaneTarget planned = _s.PlaneTarget;
            if (planned == null)
                return ChaseStatus.NoTarget;
            incT = UpfgTarget.DegToRad(planned.IncDeg);
            lanT = UpfgTarget.DegToRad(planned.LanDeg);
            plan.IncDeg = planned.IncDeg;
            plan.LanDeg = planned.LanDeg;
            plan.PeKm = plan.TargetPeKm = planned.PeKm;
            plan.ApKm = plan.TargetApKm = planned.ApKm;
            plan.ArgPeDeg = double.NaN;
        }
        else
        {
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
            incT = Math.Acos(Math.Clamp(n.Z, -1.0, 1.0));
            lanT = n.X == 0.0 && n.Y == 0.0 ? 0.0 : Wrap2Pi(Math.Atan2(n.X, -n.Y));

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
        }

        if (!TryLaunchWindow(orbit.StateVectors.PositionCci, parent.GetAngularVelocity(), incT, lanT,
                out plan.WaitSec, out plan.Descending, out plan.NearestDescending))
            return ChaseStatus.PlaneUnreachable;
        return ChaseStatus.Ok;
    }

    /// <summary>
    /// The launch window into the plane (incT, lanT) from the site at CCI position <paramref name="r"/>, on a body turning at <paramref name="omega"/> rad/s: the wait to ignition for the next time the body's rotation carries the site under the plane, at whichever crossing - ascending or descending - comes first.
    /// Either puts the vehicle in the same plane; they differ only in heading.
    /// False when the plane never passes over the site, its inclination below the site's latitude.
    /// A body that does not turn never brings the site under the plane, and the wait is NaN.
    /// </summary>
    internal static bool TryLaunchWindow(double3 r, double omega, double incT, double lanT,
                                         out double waitSec, out bool descending, out bool nearestDescending)
    {
        waitSec = double.NaN;
        descending = false;
        nearestDescending = false;

        double lat = Math.Asin(Math.Clamp(r.Z / r.Length(), -1.0, 1.0));
        double ra = Math.Atan2(r.Y, r.X);
        double tanRatio = Math.Tan(lat) / Math.Tan(Math.Max(incT, 1e-6));
        if (Math.Abs(tanRatio) > 1.0)
            return false;

        double delta = Math.Asin(Math.Clamp(tanRatio, -1.0, 1.0));

        // LAUNCH EARLY, by the same lead the LAN seeding uses (see LanLeadSeconds).
        // The instant the site is in the plane is the wrong instant to LIGHT THE ENGINE: the pad goes on being carried east through the vertical rise and the turn, so a vehicle that ignites in the plane is already out of it by the time it is flying, and the guidance yaws to chase a node it has gone past.
        // What we want is the launch time whose plane crossing lands in the middle of that, i.e. solve for T where the site's right ascension T + lead from now is the one the plane needs.
        //
        // The lead goes INSIDE the wrap, not subtracted after it: subtracting after would turn a window two minutes out into a negative countdown.
        if (omega <= 1e-12)
            return true;

        double raNow = ra + omega * LanLeadSeconds;
        double period = 2.0 * Math.PI / omega;

        double WaitFor(bool descendingNode)
        {
            double raRequired = descendingNode ? lanT + Math.PI - delta : lanT + delta;
            double wait = Wrap2Pi(raRequired - raNow) / omega;

            // ...
            // AND THE WRAP IS NOT ALLOWED TO COST A WHOLE REVOLUTION.
            // Inside the lead window - the ideal ignition is behind us but the plane crossing itself is still ahead - the wrap reports the NEXT revolution's launch, so the commit armed a countdown of most of a day and looked like a dead button.
            // Going now is at most LanLeadSeconds late, which only means the crossing lands earlier in the ascent than the lead intends; waiting a revolution for that is absurd.
            if (wait > period - LanLeadSeconds)
                wait = 0.0;
            return wait;
        }

        double waitAscending = WaitFor(false);
        double waitDescending = WaitFor(true);
        descending = waitDescending < waitAscending;
        waitSec = Math.Min(waitAscending, waitDescending);

        // For launching off the window: the crossing nearest in time, counting one just missed as behind by the rest of the revolution it would otherwise wait.
        nearestDescending = Math.Min(waitDescending, period - waitDescending)
                          < Math.Min(waitAscending, period - waitAscending);
        return true;
    }

    /// <summary>
    /// Drives the target-orbit inputs from the chase plan. The launch window step calls
    /// this while a target is selected and no launch is armed, and the launch commit once
    /// more as it is clicked. That is why the LAUNCH page greys those inputs: they are
    /// outputs of the target pick, not independent settings.
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
