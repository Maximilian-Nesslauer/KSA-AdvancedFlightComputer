#nullable disable

using System;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// The desired insertion orbit, expressed the way UPFG needs it: a target radius, speed, flight-path angle and orbital-plane normal in the inertial (CCI) frame.
//
// This target builder converts UI inputs in km and degrees into the inertial CCI values that UPFG requires. The body radius and gravitational parameter come from the live KSA celestial body.
//
// THE ARGUMENT OF PERIAPSIS IS FREE OR FIXED. Free is the classic ascent target: insert at one chosen point of the ellipse - periapsis, unless UpfgInsertionSearch has found a cheaper one further round - with the downrange position left to the solver, so the shape and plane of the orbit are hit and its periapsis falls wherever the burn happens to end. Fixed pins the whole ellipse in its plane and lets the insertion point slide along it instead: each solve finds the true anomaly the ellipse has under the predicted cutoff and aims at the radius, speed and flight-path angle it has there (InsertionToward). The downrange position is still free either way - with the argument of periapsis fixed it decides WHERE on the ellipse the vehicle arrives, rather than where the ellipse is.
public sealed class UpfgTarget
{
    /// <summary>
    /// Below this eccentricity a fixed argument of periapsis falls back to free. A nearly round orbit has no orientation worth holding, and the one it has swings through large angles for tiny errors in the cutoff state. The maneuver tools treat the same value as circular.
    /// </summary>
    public const double MinArgPeEccentricity = 1e-3;

    public double Radius;     // free insertion radius (m, from body centre)
    public double Velocity;   // free insertion speed (m/s)
    public double Fpa;        // free insertion flight-path angle (rad), positive climbing
    public double3 Normal;    // unit orbital-plane normal in CCI
    public double Inclination;// rad
    public double Lan;        // rad
    public double Pe;         // periapsis radius (m)
    public double Ap;         // apoapsis radius (m)
    public double Ecc;
    public double Sma;        // semi-major axis (m)
    public double Mu;         // gravitational parameter the ellipse belongs to (m^3/s^2)
    public double ArgPe = double.NaN; // argument of periapsis (rad), NaN while free
    public double MinCutoffRadius;    // lowest radius an insertion may sit at (m), 0 for none
    public double3 Rdes;      // desired cutoff/landing position, CCI (modes 2/3)
    public double DescentRate; // desired downward speed at Rdes, m/s (mode 3)

    // The free insertion: its true anomaly, and whether the floor rather than the point asked for decided it.
    private double _freeNu;
    private bool _freeFloorLimited;

    public bool ArgPeFixed => !double.IsNaN(ArgPe);

    /// <summary>
    /// True when the whole ellipse lies under the floor: its apoapsis is no higher than MinCutoffRadius, so there is no point on it the floor allows and no crossing to move an insertion to. The floor is then not applied at all - nothing about the insertion is FloorLimited - and it is for the caller to refuse the target, as the ascent does, rather than fly an insertion that only looks guarded.
    /// </summary>
    public bool InsideFloor => MinCutoffRadius > 0.0 && Ap <= MinCutoffRadius;

    /// <summary>The lowest true anomaly an insertion may use on the climbing side, rad in [0, pi): where the ellipse climbs through the floor, or periapsis when the floor is under it or cannot be met at all.</summary>
    public double FloorAnomaly => InsideFloor ? 0.0 : TrueAnomalyAtRadius(MinCutoffRadius);

    /// <summary>True anomaly a free target inserts at, rad in [0, pi]: zero is periapsis.</summary>
    public double FreeInsertionAnomaly => _freeNu;

    /// <summary>
    /// A point on the target ellipse to insert at. TrueAnomaly is measured from periapsis in (-pi, pi], so its sign is the sign of the flight-path angle. RawTrueAnomaly is the point asked for before the cutoff floor moved it, which is what the next solve's rate limit continues from. The default value is not Valid: nothing has been aimed at yet.
    /// </summary>
    public struct Insertion
    {
        public bool Valid;
        public double Radius;         // m, from body centre
        public double Speed;          // m/s
        public double Fpa;            // rad, positive climbing
        public double TrueAnomaly;    // rad, (-pi, pi]
        public double RawTrueAnomaly; // rad, (-pi, pi], before the floor
        public bool FloorLimited;     // the floor moved the insertion off the point asked for
    }

    // peKm / apKm are altitudes above the surface; incDeg / lanDeg define the plane. argPeDeg fixes the argument of periapsis, and NaN leaves it free. minCutoffRadius is the lowest radius an insertion may be placed at - the top of the atmosphere, so the engines are never cut inside it. freeInsertionNuDeg is where a free target inserts, as a true anomaly on the climbing side: zero is periapsis.
    public static UpfgTarget FromOrbit(double peKm, double apKm, double incDeg, double lanDeg,
                                       double bodyRadius, double mu,
                                       double argPeDeg = double.NaN, double minCutoffRadius = 0.0,
                                       double freeInsertionNuDeg = 0.0)
    {
        var t = new UpfgTarget();

        double pe = peKm * 1000.0 + bodyRadius;
        double ap = apKm * 1000.0 + bodyRadius;
        if (ap < pe) (ap, pe) = (pe, ap);

        t.Pe = pe;
        t.Ap = ap;
        t.Ecc = (ap - pe) / (ap + pe);
        t.Sma = (pe + ap) / 2.0;
        t.Mu = mu;
        t.MinCutoffRadius = Math.Max(minCutoffRadius, 0.0);

        t.Inclination = DegToRad(incDeg);
        t.Lan = DegToRad(lanDeg);
        t.Normal = OrbitNormal(t.Inclination, t.Lan);

        if (!double.IsNaN(argPeDeg) && t.Ecc >= MinArgPeEccentricity)
            t.ArgPe = WrapTwoPi(DegToRad(argPeDeg));

        t.SetFreeInsertion(DegToRad(freeInsertionNuDeg));
        return t;
    }

    /// <summary>
    /// This target with the argument of periapsis left free and the insertion at true anomaly <paramref name="nu"/>, the floor still applying - the candidates UpfgInsertionSearch prices.
    /// </summary>
    public UpfgTarget WithFreeInsertion(double nu)
    {
        var t = (UpfgTarget)MemberwiseClone();
        t.ArgPe = double.NaN;
        t.SetFreeInsertion(nu);
        return t;
    }

    // Insert at the point asked for on the climbing side, periapsis by default (flight-path angle zero there) - unless that is under the floor, in which case the ellipse is joined where it climbs through the floor, the way PEGAS inserts at a cutoff altitude above periapsis. An ellipse entirely under the floor has no such crossing, and the floor is left out (see InsideFloor).
    private void SetFreeInsertion(double nu)
    {
        double asked = double.IsFinite(nu) ? Math.Clamp(nu, 0.0, Math.PI) : 0.0;
        double floorNu = FloorAnomaly;
        _freeFloorLimited = floorNu > asked;
        _freeNu = Math.Max(asked, floorNu);
        StateAt(_freeNu, out Radius, out Velocity, out Fpa);
    }

    /// <summary>
    /// Where a cutoff in direction <paramref name="dir"/> should insert. FREE: the insertion SetFreeInsertion chose, whatever the direction. FIXED: the point of the ellipse under the direction, whose true anomaly is the direction's argument of latitude less the argument of periapsis.
    ///
    /// RATE-LIMITED against <paramref name="previous"/>, to at most maxStep radians of true anomaly per call. The direction is UPFG's predicted cutoff, which is far from converged on the first solves after engaging, and passing it straight through would move the target radius with it - which moves the prediction in turn. In converged flight the predicted cutoff barely moves in inertial space, so the limit only binds while the solution settles.
    ///
    /// THE FLOOR WINS over the argument of periapsis. A point under MinCutoffRadius is replaced by the floor crossing on the side the previous insertion was on, climbing when there was none, so the insertion cannot chatter between the two crossings - and the flight-path angle between signs - while the prediction hovers over periapsis. The orbit flown then has the requested shape and plane with its periapsis rotated by the difference, which FloorLimited reports.
    /// </summary>
    public Insertion InsertionToward(double3 dir, Insertion previous, double maxStep)
    {
        if (!ArgPeFixed)
        {
            return new Insertion
            {
                Valid = true, Radius = Radius, Speed = Velocity, Fpa = Fpa,
                TrueAnomaly = _freeNu, RawTrueAnomaly = _freeNu, FloorLimited = _freeFloorLimited,
            };
        }

        double raw = WrapPi(ArgumentOfLatitude(dir) - ArgPe);
        if (previous.Valid && maxStep < Math.PI)
            raw = WrapPi(previous.RawTrueAnomaly
                + Math.Clamp(WrapPi(raw - previous.RawTrueAnomaly), -maxStep, maxStep));

        return FixedInsertionAt(raw, !previous.Valid || previous.TrueAnomaly >= 0.0);
    }

    /// <summary>
    /// A fixed target's insertion held at <paramref name="held"/>'s true anomaly rather than following the cutoff - how UpfgGuidance flies the end of the burn (see its AimHoldTgoS). Radius, speed and flight-path angle are read again from this target, so a target edited while the aim is held still applies, and so does the floor, on the side the held insertion is on.
    /// </summary>
    public Insertion HeldInsertion(Insertion held)
    {
        if (!ArgPeFixed || !held.Valid)
            return InsertionToward(default, held, 0.0);
        return FixedInsertionAt(held.RawTrueAnomaly, held.TrueAnomaly >= 0.0);
    }

    // The fixed ellipse's insertion at raw true anomaly raw, moved to the floor crossing on the climbing side or the descending one when raw is under the floor.
    private Insertion FixedInsertionAt(double raw, bool climbingSide)
    {
        double nu = raw;
        bool floorLimited = false;
        if (!InsideFloor && RadiusAt(raw) < MinCutoffRadius)
        {
            double crossing = FloorAnomaly;
            nu = climbingSide ? crossing : -crossing;
            floorLimited = true;
        }

        StateAt(nu, out double radius, out double speed, out double fpa);
        return new Insertion
        {
            Valid = true, Radius = radius, Speed = speed, Fpa = fpa,
            TrueAnomaly = nu, RawTrueAnomaly = raw, FloorLimited = floorLimited,
        };
    }

    // The true anomaly at which the ellipse climbs through radius r, in [0, pi]: zero at or below periapsis, pi at or above apoapsis - the highest point, not a crossing (see InsideFloor).
    public double TrueAnomalyAtRadius(double r)
    {
        if (r <= Pe || Ecc < 1e-12)
            return 0.0;
        if (r >= Ap)
            return Math.PI;
        double p = Sma * (1.0 - Ecc * Ecc);
        return Math.Acos(Math.Clamp((p / r - 1.0) / Ecc, -1.0, 1.0));
    }

    public double RadiusAt(double nu) => Sma * (1.0 - Ecc * Ecc) / (1.0 + Ecc * Math.Cos(nu));

    // Radius, speed and flight-path angle of the target ellipse at true anomaly nu. The angle is signed: positive from periapsis up to apoapsis, negative on the way back down.
    public void StateAt(double nu, out double radius, out double speed, out double fpa)
    {
        double denom = 1.0 + Ecc * Math.Cos(nu);
        radius = Sma * (1.0 - Ecc * Ecc) / denom;
        speed = Math.Sqrt(Math.Max(Mu * (2.0 / radius - 1.0 / Sma), 0.0));
        fpa = Math.Atan2(Ecc * Math.Sin(nu), denom);
    }

    // The in-plane angle from the ascending node to dir, in the direction of motion, rad in [0, 2pi). Only the part of dir in the plane counts.
    public double ArgumentOfLatitude(double3 dir)
    {
        double3 node = NodeDirection(Lan);
        double3 ahead = double3.Cross(Normal, node);
        return WrapTwoPi(Math.Atan2(double3.Dot(dir, ahead), double3.Dot(dir, node)));
    }

    // Calculate the plane normal from inclination and longitude of ascending node, with Z as the celestial polar axis in CCI.
    public static double3 OrbitNormal(double inc, double lan)
    {
        return new double3(
            Math.Sin(inc) * Math.Sin(lan),
            -Math.Sin(inc) * Math.Cos(lan),
            Math.Cos(inc));
    }

    /// <summary>
    /// Unit direction of the ascending node of a plane with this LAN. Taken from the LAN rather than from the normal so that an equatorial plane, whose normal no longer says where any node is, keeps the reference direction its LAN names - KSA's own elements use +X there, which is LAN 0.
    /// </summary>
    public static double3 NodeDirection(double lan) => new double3(Math.Cos(lan), Math.Sin(lan), 0.0);

    // Unit direction of periapsis, CCI, for an argument of periapsis measured in the plane of (inc, lan).
    public static double3 PeriapsisDirection(double inc, double lan, double argPe)
    {
        double3 node = NodeDirection(lan);
        double3 ahead = double3.Cross(OrbitNormal(inc, lan), node);
        return Math.Cos(argPe) * node + Math.Sin(argPe) * ahead;
    }

    /// <summary>
    /// The argument of periapsis of the orbit through (r, v), rad in [0, 2pi), measured from the ascending node of LAN <paramref name="lan"/> in the direction of motion. NaN when the orbit is too nearly circular to have one (see <see cref="MinArgPeEccentricity"/>).
    ///
    /// For an inclined orbit whose LAN is passed in, this is the number KSA's Orbit.ArgumentOfPeriapsis reports: both measure the eccentricity vector from the node, on the side the orbit is travelling. Passing the LAN rather than deriving the node from r x v is what keeps an equatorial orbit consistent with the LAN it was given.
    /// </summary>
    public static double ArgumentOfPeriapsisOf(double3 r, double3 v, double mu, double lan)
    {
        double rm = r.Length();
        double3 h = double3.Cross(r, v);
        if (!(rm > 0.0) || !(h.Length() > 0.0) || !(mu > 0.0))
            return double.NaN;

        double3 ecc = ((double3.Dot(v, v) - mu / rm) * r - double3.Dot(r, v) * v) * (1.0 / mu);
        if (!(ecc.Length() >= MinArgPeEccentricity))
            return double.NaN;

        double3 node = NodeDirection(lan);
        double3 ahead = double3.Cross(double3.Normalize(h), node);
        return WrapTwoPi(Math.Atan2(double3.Dot(ecc, ahead), double3.Dot(ecc, node)));
    }

    public static double DegToRad(double d) => d * Math.PI / 180.0;
    public static double RadToDeg(double r) => r * 180.0 / Math.PI;

    public static double WrapTwoPi(double a)
    {
        a %= 2.0 * Math.PI;
        return a < 0.0 ? a + 2.0 * Math.PI : a;
    }

    // (-pi, pi]
    public static double WrapPi(double a)
    {
        a = WrapTwoPi(a);
        return a > Math.PI ? a - 2.0 * Math.PI : a;
    }
}
