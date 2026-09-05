using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>
/// Retargets a stock transfer so the arrival is a flyby at a chosen periapsis
/// instead of an impact aimed at the center. Stock's
/// <see cref="OrbitalTransfers.SolveLambert"/> aims at the target's center at
/// arrival. This aims at center + b * n_hat instead, where b is the impact
/// parameter that produces the requested periapsis and n_hat is perpendicular to
/// the approach relative velocity, then solves the departure again so the burn
/// itself yields the flyby.
///
/// Two gravitational parameters are in play and must not be conflated. The
/// Lambert solve is about the shared parent of the transfer, which is Earth for
/// LEO to Luna and the Sun for LEO to Mars. The impact parameter and the
/// periapsis speed use the target's own mu from <see cref="IParentBody.Mu"/>.
/// Stock's course correction reads <c>target.Orbit.Mu</c>, which resolves to the
/// parent's mu, and is deliberately not followed here.
///
/// The B plane relation takes the relative speed at the SOI boundary so the
/// achieved patched conic periapsis matches the game's own model. It reads
/// v_p^2 = v_soi^2 - 2 mu / r_soi + 2 mu / r_p and b = r_p v_p / v_soi.
/// </summary>
internal static class FlybyTargeting
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // Coarse samples over the arrival window before Brent refinement. 48 resolves
    // the minimum well below the Brent tolerance for Earth to Luna and Earth to
    // Mars.
    private const int ClosestApproachCoarseSteps = 48;

    // Fixed point passes for v_soi. The aim offset is about b, a few thousand km
    // for a moon, so v_soi shifts by well under one percent per pass and three
    // passes converge with margin.
    private const int DefaultIterations = 3;

    /// <summary>Smallest usable axis alignment, which is the sine of the angle
    /// between a side's axis and the approach relative velocity. Below roughly 9
    /// degrees the projected offset direction is numerical noise rather than the
    /// requested side, so the solver refuses and the UI disables that side.</summary>
    public const double MinAxisAlignment = 0.15;

    /// <summary>The departure burn in VLF and CCI plus the geometry it was built
    /// from. <see cref="BurnTime"/> is the transfer Start when the target shares
    /// the vehicle's parent. When it does not, it carries the true anomaly shift
    /// stock applies to a hyperbolic escape.
    ///
    /// <see cref="VInfMs"/> is the speed relative to the target at the SOI
    /// boundary, which is the divisor in b = r_p v_p / v_soi. It is a few percent
    /// above the asymptotic excess sqrt(2E). <see cref="PlannerVInfMs"/> and
    /// <see cref="PlannerApoTargetMeters"/> are the energy descriptors in the
    /// parking frame that a split into several passes locks. A departure to
    /// another parent carries the ejection excess and a NaN apoapsis. A departure
    /// within the same parent carries the apoapsis after the burn and a NaN
    /// excess.</summary>
    public readonly record struct FlybyResult(
        double3 DvVlf,
        double3 DvCci,
        UniverseTime BurnTime,
        double ImpactParameterMeters,
        double VInfMs,
        double TargetPeRadiusMeters,
        bool IsCrossParent,
        double PlannerVInfMs,
        double PlannerApoTargetMeters);

    #region Reference radius resolution

    /// <summary>Turns the value the user entered and its reference into a
    /// periapsis radius measured from the target center.</summary>
    public static double ResolvePeriapsisRadius(
        IParentBody target, double value, FlybyReference reference)
    {
        return reference switch
        {
            FlybyReference.Center => value,
            FlybyReference.Atmosphere => target.GetAtmosphereRadius() + value,
            _ => target.MeanRadius + value,
        };
    }

    /// <summary>Lowest periapsis radius the game treats as clear of the body. That
    /// is the terrain ceiling on an airless body and the top of the atmosphere on
    /// one with an atmosphere, see <see cref="Astronomical.GetNearSurfaceRadius"/>
    /// and the <c>AtmosphericBody</c> override. Below it a flyby impacts or enters
    /// the atmosphere, so the UI blocks Create.</summary>
    public static double MinFlybyRadius(IParentBody target) => target.GetNearSurfaceRadius();

    /// <summary><see cref="IParentBody.GetAtmosphereRadius"/> returns 0 for airless
    /// bodies, so a positive value means the Atmosphere reference is meaningful.</summary>
    public static bool HasAtmosphere(IParentBody target) => target.GetAtmosphereRadius() > 0.0;

    #endregion

    #region Impact parameter closed forms

    /// <summary>Impact parameter b that yields flyby periapsis <paramref name="rpRadius"/>
    /// for the speed <paramref name="vSoi"/> relative to the target, measured at
    /// radius <paramref name="soiRadius"/> about a body of gravitational parameter
    /// <paramref name="muTarget"/>. With an infinite SOI radius vSoi is the
    /// asymptotic excess and this is the textbook relation. NaN on inputs that are
    /// not physical. The retarget solves with this form, so a test of it tests what
    /// flies.</summary>
    public static double ImpactParameterForPeriapsis(
        double vSoi, double rpRadius, double muTarget, double soiRadius = double.PositiveInfinity)
    {
        if (!(vSoi > 0.0) || !(rpRadius > 0.0) || !(muTarget > 0.0) || !(soiRadius > 0.0))
            return double.NaN;
        double vpSquared = vSoi * vSoi - 2.0 * muTarget / soiRadius + 2.0 * muTarget / rpRadius;
        if (!(vpSquared > 0.0))
            return double.NaN;
        return rpRadius * Math.Sqrt(vpSquared) / vSoi;
    }

    /// <summary>Inverse of <see cref="ImpactParameterForPeriapsis"/>. From
    /// (b v_soi)^2 = r_p^2 w^2 + 2 mu r_p with w^2 = v_soi^2 - 2 mu / r_soi, the
    /// positive root is written as (b v_soi)^2 / (mu + sqrt(mu^2 + w^2 (b v_soi)^2)),
    /// which stays accurate when mu dominates and needs no special case at w = 0.</summary>
    public static double PeriapsisForImpactParameter(
        double vSoi, double b, double muTarget, double soiRadius = double.PositiveInfinity)
    {
        if (!(vSoi > 0.0) || !(b > 0.0) || !(muTarget > 0.0) || !(soiRadius > 0.0))
            return double.NaN;
        double wSquared = vSoi * vSoi - 2.0 * muTarget / soiRadius;
        double bv = b * vSoi;
        double discriminant = muTarget * muTarget + wSquared * bv * bv;
        if (!(discriminant >= 0.0))
            return double.NaN;
        return bv * bv / (muTarget + Math.Sqrt(discriminant));
    }

    #endregion

    #region Cross parent detection

    /// <summary>True when the departure is a hyperbolic escape, which means the
    /// target orbits a different parent than the vehicle, for example LEO to Mars.
    /// A direct compare of the parent ids rather than
    /// <see cref="OrbitalTransfers.SameSoiTransfer"/>, because stock rewrites the
    /// transfer source to the vehicle for transfers inside one SOI, which makes a
    /// second check through it unreliable.</summary>
    public static bool IsCrossParentTransfer(Vehicle source, IOrbiter target)
    {
        string? sp = source.Orbit.Parent?.Id;
        string? tp = target.Orbit?.Parent?.Id;
        return sp == null || tp == null || sp != tp;
    }

    #endregion

    #region Retarget

    /// <summary>Retargets the departure of a stock transfer, given as its Start and
    /// Transit from a selected porkchop entry, so it flies by <paramref name="target"/>
    /// at <paramref name="targetPeRadius"/> on the requested <paramref name="side"/>.
    ///
    /// <see cref="FlybyOutcome.Result"/> is null on a degenerate geometry, which
    /// means the requested side's axis is nearly parallel to the approach or the
    /// solve is not physical. The caller then falls back to the stock burn aimed at
    /// the center and warns. The axis alignments are reported either way so the UI
    /// can show which sides this approach can reach.</summary>
    public static FlybyOutcome ComputeFlybyDeparture(
        Vehicle source, IOrbiter target, UniverseTime start, UniverseTime transit,
        double targetPeRadius, FlybySide side, int iterations = DefaultIterations)
    {
        // Vehicle.Orbit throws on an empty plan rather than returning null, so only
        // the target orbit and parent need a null guard.
        if (target.Orbit?.Parent == null) return FlybyOutcome.Unavailable;
        if (target is not IParentBody targetBody) return FlybyOutcome.Unavailable;
        if (!(targetPeRadius > 0.0)) return FlybyOutcome.Unavailable;

        IParentBody lambertParent = target.Orbit.Parent;
        double muParent = lambertParent.Mu;
        double muTarget = targetBody.Mu;
        double soiTarget = targetBody.SphereOfInfluence;
        if (!(muParent > 0.0) || !(muTarget > 0.0) || !(soiTarget > 0.0))
            return FlybyOutcome.Unavailable;

        bool isCross = IsCrossParentTransfer(source, target);

        // OrbitalTransfers.SingleImpulseHyperbolicEscape throws on a hyperbolic
        // parking orbit and nothing downstream would catch it. Stock guards the same
        // way at its own call sites.
        if (isCross && source.Orbit.Eccentricity >= 1.0) return FlybyOutcome.Unavailable;

        // The Lambert is solved in the CCI frame of the target's parent. When the
        // vehicle shares that parent the departure body is the vehicle. Otherwise it
        // is the celestial the vehicle is parked at, whose orbit shares the target's
        // parent, as in stock's SolveLambert.
        Orbit sourceInFrame;
        if (isCross)
        {
            if (source.Orbit.Parent is not Celestial parkingCelestial
                || parkingCelestial.Orbit?.Parent?.Id != lambertParent.Id)
                return FlybyOutcome.Unavailable;
            sourceInFrame = parkingCelestial.Orbit;
        }
        else
        {
            sourceInFrame = source.Orbit;
        }

        OffsetSolve? solve = SolveOffsetTransfer(
            muParent, sourceInFrame, target.Orbit, start, transit,
            targetPeRadius, muTarget, soiTarget, side, iterations,
            out double radialAlign, out double normalAlign);
        if (solve == null)
            return new FlybyOutcome(null, radialAlign, normalAlign);
        OffsetSolve s = solve.Value;

        FlybyResult? departure = isCross
            ? BuildCrossParentDeparture(source, lambertParent, start, s, targetPeRadius)
            : BuildSameParentDeparture(source, start, s, targetPeRadius);
        if (departure is not FlybyResult result)
            return new FlybyOutcome(null, radialAlign, normalAlign);

        if (!IsFinite(result.DvVlf) || !(result.DvVlf.Length() > 0.0))
            return new FlybyOutcome(null, radialAlign, normalAlign);

        if (DebugConfig.Flyby)
            LogDeparture(source, target, isCross, targetPeRadius, side, s, result, radialAlign, normalAlign);

        return new FlybyOutcome(result, radialAlign, normalAlign);
    }

    /// <summary>Result of a retarget attempt plus how well each named side axis can
    /// be reached for this approach. An alignment is the sine of the angle between
    /// the axis and the approach relative velocity. At 1 the axis is fully usable,
    /// and near 0 the offset would have to point along the approach, which cannot
    /// move the periapsis. <see cref="MinAxisAlignment"/> is the cutoff the UI uses.</summary>
    public readonly record struct FlybyOutcome(
        FlybyResult? Result,
        double RadialAxisAlignment,
        double NormalAxisAlignment)
    {
        public static FlybyOutcome Unavailable => new(null, 0.0, 0.0);

        /// <summary>Whether the solve got far enough to measure the axes. It bails
        /// out earlier for reasons unrelated to the side, such as a bad target, no
        /// SOI, or an approach too slow for the requested periapsis, and reporting
        /// those as "no side is reachable" would disable the whole picker.</summary>
        public bool HasAxisData => RadialAxisAlignment > 0.0 || NormalAxisAlignment > 0.0;

        public bool CanReach(FlybySide side) =>
            !HasAxisData || AxisAlignmentFor(side) >= MinAxisAlignment;

        public double AxisAlignmentFor(FlybySide side) =>
            side is FlybySide.Inner or FlybySide.Outer
                ? RadialAxisAlignment
                : NormalAxisAlignment;
    }

    /// <summary>Departure state from the Lambert solve at the offset aim point, in
    /// the CCI frame of the target's parent. <see cref="DepartureDeltaCci"/> is the
    /// injection dV when the vehicle shares that parent, and otherwise the
    /// hyperbolic excess relative to the parking parent, which stock names
    /// DepartureVelocityCci.</summary>
    private readonly record struct OffsetSolve(
        double3 DepartureDeltaCci,
        UniverseTime Transit,
        double ImpactParameter,
        double VInf,
        double3 OffsetDir,
        UniverseTime CaTime);

    private static OffsetSolve? SolveOffsetTransfer(
        double muParent, Orbit sourceInFrame, Orbit targetOrbit,
        UniverseTime start, UniverseTime transit, double rpRadius, double muTarget,
        double soiTarget, FlybySide side, int iterations,
        out double radialAlign, out double normalAlign)
    {
        radialAlign = 0.0;
        normalAlign = 0.0;
        StateVectors departure = sourceInFrame.GetStateVectorsAt(start);
        double3 departurePos = departure.PositionCci;
        byte4 lineColor = sourceInFrame.OrbitLineColor;

        // Seed with the stock solve aimed at the center.
        double3 arrivalCenter = targetOrbit.GetStateVectorsAt(start + transit).PositionCci;
        OrbitalTransfers.SuperiorLambert(
            muParent, departurePos, arrivalCenter, transit, out double3 vDeparture, out _);

        UniverseTime lastTransit = transit;
        double lastB = double.NaN;
        double lastVSoi = double.NaN;
        double3 lastOffsetDir = double3.Zero;
        UniverseTime lastCaTime = default;

        for (int iter = 0; iter < Math.Max(1, iterations); iter++)
        {
            Orbit transfer = Orbit.CreateFromStateCci(
                targetOrbit.Parent, start, departurePos, vDeparture, lineColor);

            UniverseTime caTime = FindClosestApproach(
                transfer, targetOrbit, start, start + lastTransit);

            // The relative velocity at closest approach sets the offset plane. A
            // little earlier, at the SOI boundary, it sets the flyby energy.
            double3 vRelCa = transfer.GetStateVectorsAt(caTime).VelocityCci
                             - targetOrbit.GetStateVectorsAt(caTime).VelocityCci;
            double vRelCaLen = vRelCa.Length();
            if (!(vRelCaLen > 0.0)) return null;

            UniverseTime soiTime = caTime - soiTarget / vRelCaLen;
            double vSoi = (transfer.GetStateVectorsAt(soiTime).VelocityCci
                           - targetOrbit.GetStateVectorsAt(soiTime).VelocityCci).Length();
            double b = ImpactParameterForPeriapsis(vSoi, rpRadius, muTarget, soiTarget);
            if (!(b > 0.0)) return null;

            StateVectors targetAtCa = targetOrbit.GetStateVectorsAt(caTime);
            if (!TryResolveSideOffset(targetAtCa, vRelCa / vRelCaLen, side,
                    out double3 offsetDir, out radialAlign, out normalAlign))
                return null;

            double3 aim = targetAtCa.PositionCci + b * offsetDir;

            UniverseTime newTransit = caTime - start;
            if (!(newTransit > 0.0)) return null;

            OrbitalTransfers.SuperiorLambert(
                muParent, departurePos, aim, newTransit, out vDeparture, out _);

            lastTransit = newTransit;
            lastB = b;
            lastVSoi = vSoi;
            lastOffsetDir = offsetDir;
            lastCaTime = caTime;
        }

        double3 departureDelta = vDeparture - departure.VelocityCci;
        if (!IsFinite(departureDelta)) return null;
        return new OffsetSolve(
            departureDelta, lastTransit, lastB, lastVSoi, lastOffsetDir, lastCaTime);
    }

    /// <summary>Named sides live in the target's orbital frame, where radial is the
    /// target's radius from its parent and normal is its orbit normal. The offset
    /// has to stay perpendicular to the approach relative velocity, so the requested
    /// axis is projected into that plane, and how much survives the projection is
    /// the alignment the UI gates on. False when the wanted axis lies too close to
    /// the approach, where the surviving component is noise rather than the side.</summary>
    private static bool TryResolveSideOffset(
        StateVectors targetAtCa, double3 vRelHat, FlybySide side,
        out double3 offsetDir, out double radialAlign, out double normalAlign)
    {
        double3 radialAxis = targetAtCa.PositionCci.NormalizeOrZero();
        double3 normalAxis = double3.Cross(
            targetAtCa.PositionCci, targetAtCa.VelocityCci).NormalizeOrZero();

        radialAlign = PerpendicularComponent(radialAxis, vRelHat).Length();
        normalAlign = PerpendicularComponent(normalAxis, vRelHat).Length();

        double3 wantedAxis = side switch
        {
            FlybySide.Inner => -radialAxis,
            FlybySide.Outer => radialAxis,
            FlybySide.North => normalAxis,
            _ => -normalAxis,
        };

        double3 wantedPerp = PerpendicularComponent(wantedAxis, vRelHat);
        offsetDir = wantedPerp.NormalizeOrZero();
        return wantedPerp.Length() >= MinAxisAlignment;
    }

    private static FlybyResult BuildSameParentDeparture(
        Vehicle source, UniverseTime start, OffsetSolve s, double rpRadius)
    {
        // DepartureDeltaCci is the injection dV directly, as in FinalizeLambert's
        // Source == Vehicle branch. The burn is at the transfer Start.
        double3 dvCci = s.DepartureDeltaCci;
        StateVectors sv = source.Orbit.GetStateVectorsAt(start);
        double3 dvVlf = ToVlf(sv, dvCci);

        Orbit postBurn = Orbit.CreateFromStateCci(
            source.Orbit.Parent, start, sv.PositionCci, sv.VelocityCci + dvCci,
            source.Orbit.OrbitLineColor);
        double apoTarget = postBurn.IsBound() ? postBurn.Apoapsis : double.NaN;

        return new FlybyResult(dvVlf, dvCci, start,
            s.ImpactParameter, s.VInf, rpRadius, IsCrossParent: false,
            PlannerVInfMs: double.NaN, PlannerApoTargetMeters: apoTarget);
    }

    private static FlybyResult? BuildCrossParentDeparture(
        Vehicle source, IParentBody lambertParent, UniverseTime start,
        OffsetSolve s, double rpRadius)
    {
        // DepartureDeltaCci is the heliocentric excess relative to the parking
        // parent. Rotate it into the parking parent's frame and solve the single
        // impulse escape, as FinalizeLambert's Source != Vehicle branch does,
        // including its true anomaly shift of the burn.
        IParentBody parkingParent = source.Orbit.Parent!;
        double muParking = parkingParent.Mu;

        doubleQuat cci2Cce = lambertParent.GetCci2Cce();
        doubleQuat cce2Cci = parkingParent.GetCce2Cci();
        doubleQuat toParking = doubleQuat.Concatenate(cci2Cce, cce2Cci);
        double3 velSoiExit = s.DepartureDeltaCci.Transform(toParking);

        StateVectors sv = source.Orbit.GetStateVectorsAt(start);
        // Null when the parking orbit cannot reach the requested velocity at the
        // SOI exit, which stock reports as an infeasible departure.
        if (OrbitalTransfers.SingleImpulseHyperbolicEscape(
                muParking, source.Orbit, sv.PositionCci, sv.VelocityCci, velSoiExit)
            is not OrbitalTransfers.SingleImpulseTransfer impulse)
            return null;

        double3 dvCci = impulse.VelocityDeparture - impulse.VelParking;

        UniverseTime toBurnTa = source.Orbit.GetTimeFromPeTo(impulse.BurnTrueAnomaly);
        UniverseTime toCurrentTa = source.Orbit.GetTimeFromPeTo(sv.TrueAnomaly);
        UniverseTime burnTime = start + (toBurnTa - toCurrentTa);

        double3 dvVlf = ToVlf(source.Orbit.GetStateVectorsAt(burnTime), dvCci);
        double plannerVInf = s.DepartureDeltaCci.Length();
        return new FlybyResult(dvVlf, dvCci, burnTime,
            s.ImpactParameter, s.VInf, rpRadius, IsCrossParent: true,
            PlannerVInfMs: plannerVInf, PlannerApoTargetMeters: double.NaN);
    }

    #endregion

    #region Helpers

    private static double3 ToVlf(StateVectors sv, double3 dvCci)
    {
        doubleQuat parentCci2Vlf = sv.GetVlf2ParentCci().OrIdentity().Inverse();
        return dvCci.Transform(parentCci2Vlf);
    }

    /// <summary>Time of closest approach of <paramref name="transfer"/> to
    /// <paramref name="target"/> between <paramref name="tStart"/> and
    /// <paramref name="tEnd"/>. A coarse scan brackets the minimum, then Brent
    /// refinement takes over, which is the shape of the stock closest approach
    /// search.
    ///
    /// The scan runs on seconds from <paramref name="tStart"/> rather than absolute
    /// sim seconds, because a UniverseTime holds 128 bit nanoseconds and an absolute
    /// double would spend most of its mantissa on the epoch instead of on the window.</summary>
    private static UniverseTime FindClosestApproach(
        Orbit transfer, Orbit target, UniverseTime tStart, UniverseTime tEnd)
    {
        // Skip the first second so the search never latches onto the departure
        // point, where the distance is small from a low parking orbit, and widen
        // past the seed arrival so an offset transfer's true closest approach, a
        // touch later or earlier, stays bracketed.
        double span = (tEnd - tStart).Seconds();
        double a = 1.0;
        double bEnd = span * 1.25;

        double Dist(double dt)
        {
            UniverseTime t = tStart + dt;
            double3 p = transfer.GetStateVectorsAt(t).PositionCci
                        - target.GetStateVectorsAt(t).PositionCci;
            return p.Length();
        }

        double bestT = a;
        double bestD = double.MaxValue;
        double step = (bEnd - a) / ClosestApproachCoarseSteps;
        if (!(step > 0.0)) return tStart + a;
        for (int i = 0; i <= ClosestApproachCoarseSteps; i++)
        {
            double t = a + i * step;
            double d = Dist(t);
            if (d < bestD) { bestD = d; bestT = t; }
        }

        double lo = Math.Max(a, bestT - step);
        double hi = Math.Min(bEnd, bestT + step);
        double refined = MathEx.BrentMin(Dist, lo, hi, 1e-06);
        // A degenerate propagation makes Dist NaN and Brent can carry that out.
        // UniverseTime rejects NaN, so fall back to the bracketed coarse sample.
        return tStart + (double.IsFinite(refined) ? refined : bestT);
    }

    /// <summary>Component of <paramref name="v"/> perpendicular to the unit vector
    /// <paramref name="unitAxis"/>. Its length is the sine of the angle between
    /// them, which is what makes it double as the reachability measure.</summary>
    private static double3 PerpendicularComponent(double3 v, double3 unitAxis) =>
        v - unitAxis * double3.Dot(v, unitAxis);

    private static bool IsFinite(double3 v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);

    /// <summary>Offset direction in the target's own orbital frame, so the flyby
    /// side reads as physics instead of a raw CCI vector. The offset is always
    /// perpendicular to the approach, so an offset purely along the track is
    /// generally not reachable.</summary>
    private static void LogDeparture(
        Vehicle source, IOrbiter target, bool isCross, double targetPeRadius, FlybySide side,
        OffsetSolve s, FlybyResult result, double radialAlign, double normalAlign)
    {
        StateVectors tsv = target.Orbit.GetStateVectorsAt(s.CaTime);
        double3 radial = tsv.PositionCci.NormalizeOrZero();
        double3 alongTrack = tsv.VelocityCci.NormalizeOrZero();
        double3 normal = double3.Cross(tsv.PositionCci, tsv.VelocityCci).NormalizeOrZero();

        DefaultCategory.Log.Debug(string.Format(Inv,
            "[AFC] FlybyTargeting.ComputeFlybyDeparture: vehicle='{0}' target='{1}' " +
            "cross={2} rp={3:F0}m b={4:F0}m vSoi={5:F1}m/s side={6} " +
            "|dvVlf|={7:F1}m/s burnT={8:F0}s offsetDir[radial={9:F2} alongTrack={10:F2} " +
            "normal={11:F2}] axisAlign[radial={12:F2} normal={13:F2}]",
            source.Id, (target as Astronomical)?.Id ?? "?", isCross, targetPeRadius,
            s.ImpactParameter, s.VInf, side, result.DvVlf.Length(),
            result.BurnTime.Seconds(),
            double3.Dot(s.OffsetDir, radial),
            double3.Dot(s.OffsetDir, alongTrack),
            double3.Dot(s.OffsetDir, normal),
            radialAlign, normalAlign));
    }

    #endregion
}
